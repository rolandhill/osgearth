/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
* http://osgearth.org
*
* osgEarth is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "TileNode"
#include "MPTerrainEngineNode"
#include "MPGeometry"

#include <osg/ClusterCullingCallback>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Uniform>

using namespace osgEarth_engine_mp;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TileNode] "


//----------------------------------------------------------------------------

TileNode::TileNode( const TileKey& key, const TileModel* model ) :
_key               ( key ),
_model             ( model ),
_bornTime          ( 0.0 ),
_lastTraversalFrame( 0 )
{
    this->setName( key.str() );

    osg::StateSet* stateset = getOrCreateStateSet();

    // TileKey uniform.
    _keyUniform = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "oe_tile_key");
    _keyUniform->setDataVariance( osg::Object::STATIC );
    _keyUniform->set( osg::Vec4f(0,0,0,0) );
    stateset->addUniform( _keyUniform );

    // born-on date uniform.
    _bornUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_tile_birthtime");
    _bornUniform->set( -1.0f );
    stateset->addUniform( _bornUniform );

    _usedLastFrame = false;
    _terrainEngineNode = 0L;
}


void
TileNode::setLastTraversalFrame(unsigned frame)
{
  _lastTraversalFrame = frame;
}


osg::BoundingSphere
TileNode::computeBound() const
{
    osg::BoundingSphere bs = osg::MatrixTransform::computeBound();

    unsigned tw, th;
    _key.getProfile()->getNumTiles(_key.getLOD(), tw, th);

    // swap the Y index.
    _keyUniform->set( osg::Vec4f(
        _key.getTileX(),
        th-_key.getTileY()-1.0,
        _key.getLOD(),
        bs.radius()) );

    return bs;
}


void
TileNode::traverse( osg::NodeVisitor& nv )
{
//        CheckOrphanedBoundaries();

    // TODO: not sure we need this.
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
//        CheckOrphanedBoundaries();

        osg::ClusterCullingCallback* ccc = dynamic_cast<osg::ClusterCullingCallback*>(getCullCallback());
        if (ccc)
        {
            if (ccc->cull(&nv,0,static_cast<osg::State *>(0))) return;
        }

        // reset the "birth" time if necessary - this is the time at which the
        // node passes cull
        const osg::FrameStamp* fs = nv.getFrameStamp();
        if ( fs )
        {
            unsigned frame = fs->getFrameNumber();

            if ( (frame - _lastTraversalFrame > 1) || (_bornTime == 0.0) )
            {
                _bornTime = fs->getReferenceTime();
                _bornUniform->set( (float)_bornTime );
            }

            _lastTraversalFrame = frame;
        }

        // If this tile is being traversed, then it is being used in display.
        // We therefore notify the TerrainEngineNode if it is being turned on from off.
        if(!_usedLastFrame)
        {
            if(isValid()) _terrainEngineNode->RegisterChangedTileNode(this, MPTerrainEngineNode::Side_All);
        }

        _usedLastFrame = true;

//        getVertexArray()->dirty();
    }

    if(getVertexArray()) getVertexArray()->dirty();
    osg::MatrixTransform::traverse( nv );
}


void
TileNode::releaseGLObjects(osg::State* state) const
{
    if ( _model.valid() )
        _model->releaseGLObjects( state );
}

void
TileNode::resetUsedLastFrameFlag()
{
    _usedLastFrame = false;

    //Clear all boudary node data
    _boundTileW = 0L;
    _boundTileN = 0L;
    _boundTileE = 0L;
    _boundTileS = 0L;

    _boundTileW_pending = 0L;
    _boundTileN_pending = 0L;
    _boundTileE_pending = 0L;
    _boundTileS_pending = 0L;
}

osg::Vec3Array*
TileNode::getVertexArray()
{
    osg::Vec3Array* va = 0L;

    if(getNumChildren() > 0)
    {
        osg::Geode* geode = dynamic_cast<osg::Geode*>(getChild(0));
        if(geode)
        {
            if(geode->getNumDrawables() > 0)
            {
                MPGeometry* mpg = dynamic_cast<MPGeometry*>(geode->getDrawable(0));
                if(mpg)
                {
                    va = dynamic_cast<osg::Vec3Array*>(mpg->getVertexArray());
                }
            }
        }
    }

    return va;
}

void TileNode::CheckOrphanedBoundaries()
{
    //Check if the boundry tiles are still active. If not, then remove any adjustments
    if(_boundTileW.valid() && !_boundTileW->getUsedLastFrame())
    {
        ResetEdgeW();
    }

    if(_boundTileN.valid() && !_boundTileN->getUsedLastFrame())
    {
        ResetEdgeN();
    }

    if(_boundTileE.valid() && !_boundTileE->getUsedLastFrame())
    {
        ResetEdgeE();
    }

    if(_boundTileS.valid() && !_boundTileS->getUsedLastFrame())
    {
        ResetEdgeS();
    }
}

void
TileNode::AdjustEdges()
{
    osg::Vec3d center = getMatrix().getTrans();

    std::cout << " --------- New Tile\n";

    // Check each boundary to see if an adjustment is required
    if(_boundTileW_pending.valid()) AdjustEdgeW(center);
    if(_boundTileN_pending.valid()) AdjustEdgeN(center);
    if(_boundTileE_pending.valid()) AdjustEdgeE(center);
    if(_boundTileS_pending.valid()) AdjustEdgeS(center);
}

void
TileNode::AdjustEdgeW(osg::Vec3d center)
{
    _boundTileW_pending->CheckOrphanedBoundaries();

    const TileKey& key = getTileModel()->_tileKey;
    unsigned int lod = key.getLOD();
//    unsigned int x = key.getTileX();
    unsigned int y = key.getTileY();

    const TileKey& bkey = _boundTileW_pending->getTileModel()->_tileKey;
    unsigned int blod = bkey.getLOD();
//    unsigned int bx = bkey.getTileX();
    unsigned int by = bkey.getTileY();

    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();

    // Get heightfield of bounding tile
    osg::HeightField* bhf  = _boundTileW_pending->getTileModel()->_elevationData.getHeightField();
    int brows = bhf->getNumRows();
    int bcols = bhf->getNumColumns();

    float step = 1.0f;
    unsigned int mult = 1;
//    unsigned int equivx = bx;
    unsigned int equivy = by;
    for(int i = blod; i < lod; i++)
    {
        step = step * 0.5f;
        mult *= 2;
//        equivx *= 2;
        equivy *= 2;
    }

    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    int tilesfromtop = y - equivy;
    int tilesfrombottom = mult - tilesfromtop - 1;
    int start = (float)tilesfrombottom * step * (float)brows + 0.1f;

//    int start = (y - equivy) / mult * brows;

    // Flip the start point over as Tiles are +ve down and HeightField is +ve up;
//    start = brows - ((int)((float)brows * step + 0.1f)) - start;

    float cell = start;

    // Loop through this tiles boundary heights
    for(int j = 0; j < rows; j++)
    {
        // Get the lower coordinate
        int cell0 = cell + 0.1f;

        float z =0.0f;

        if(cell0 == cell || cell0 == brows)
        {
            z = bhf->getHeight(bcols - 1, cell0);
        }
        else
        {
            // get the boundary height at each coordinate then interpolate
            float z0 = bhf->getHeight(bcols - 1, cell0);
            float z1 = bhf->getHeight(bcols - 1, cell0 + 1);

            if(z0 == NO_DATA_VALUE || z1 == NO_DATA_VALUE)
            {
                z = NO_DATA_VALUE;
            }
            else
            {
                z = z0 + (z1 - z0) * (cell - (float)cell0);
            }
        }

        if(z != NO_DATA_VALUE)
        {
            osg::Vec3d ndc( 0.0, ((double)j)/(double)(rows-1), z);

            osg::Vec3d model;
            _model->_tileLocator->unitToModel( ndc, model );
            osg::Vec3d v = model - center;

            (*va)[j * cols] = v;
        }

        cell += step;
    }

    va->dirty();

    _boundTileW = _boundTileW_pending;
    _boundTileW_pending = 0L;
}

void
TileNode::AdjustEdgeN(osg::Vec3d center)
{
    _boundTileN_pending->CheckOrphanedBoundaries();

    const TileKey& key = getTileModel()->_tileKey;
    unsigned int lod = key.getLOD();
    unsigned int x = key.getTileX();
//    unsigned int y = key.getTileY();

    const TileKey& bkey = _boundTileN_pending->getTileModel()->_tileKey;
    unsigned int blod = bkey.getLOD();
    unsigned int bx = bkey.getTileX();
//    unsigned int by = bkey.getTileY();

    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();

    // Get heightfield of bounding tile
    osg::HeightField* bhf  = _boundTileN_pending->getTileModel()->_elevationData.getHeightField();
    int brows = bhf->getNumRows();
    int bcols = bhf->getNumColumns();

    float step = 1.0;
    unsigned int equivx = bx;
    unsigned int mult = 1;
//    unsigned int equivy = by;
    for(int i = blod; i < lod; i++)
    {
        step = step * 0.5;
        mult *= 2;
//        equivx *= 2;
        equivx *= 2;
    }


    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    int start = ((float)(x - equivx)) *step * (float)bcols + 0.1f;

    float cell = start;

    // Loop through this tiles boundary heights
    for(int i = 0; i < cols; i++)
    {
        // Get the lower coordinate
        int cell0 = cell + 0.1f;

        float z =0.0f;

        if(cell0 == cell || cell0 == bcols)
        {
            z = bhf->getHeight(cell0, 0);
        }
        else
        {
            // get the boundary height at each coordinate then interpolate
            float z0 = bhf->getHeight(cell0, 0);
            float z1 = bhf->getHeight(cell0 + 1, 0);

            if(z0 == NO_DATA_VALUE || z1 == NO_DATA_VALUE)
            {
                z = NO_DATA_VALUE;
            }
            else
            {
                z = z0 + (z1 - z0) * (cell - cell0);
            }
        }

        if(z != NO_DATA_VALUE)
        {
            osg::Vec3d ndc( ((double)i)/(double)(cols-1), 1.0, z);

            osg::Vec3d model;
            _model->_tileLocator->unitToModel( ndc, model );
            osg::Vec3d v = model - center;

            (*va)[(rows - 1) * cols + i] = v;
        }

        cell += step;
    }

    va->dirty();

    _boundTileN = _boundTileN_pending;
    _boundTileN_pending = 0L;
}

void
TileNode::AdjustEdgeE(osg::Vec3d center)
{
    _boundTileE_pending->CheckOrphanedBoundaries();

    const TileKey& key = getTileModel()->_tileKey;
    unsigned int lod = key.getLOD();
//    unsigned int x = key.getTileX();
    unsigned int y = key.getTileY();

    const TileKey& bkey = _boundTileE_pending->getTileModel()->_tileKey;
    unsigned int blod = bkey.getLOD();
//    unsigned int bx = bkey.getTileX();
    unsigned int by = bkey.getTileY();

    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();

    // Get heightfield of bounding tile
    osg::HeightField* bhf  = _boundTileE_pending->getTileModel()->_elevationData.getHeightField();
    int brows = bhf->getNumRows();
    int bcols = bhf->getNumColumns();

    float step = 1.0;
    unsigned int mult = 1;
//    unsigned int equivx = bx;
    unsigned int equivy = by;
    for(int i = blod; i < lod; i++)
    {
        step = step * 0.5;
        mult *= 2;
//        equivx *= 2;
        equivy *= 2;
    }

    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    int tilesfromtop = y - equivy;
    int tilesfrombottom = mult - tilesfromtop - 1;
    int start = (float)tilesfrombottom * step * (float)brows + 0.1f;
//    int start = (y - equivy) / mult * brows;

    // Flip the start point over as Tiles are +ve down and HeightField is +ve up;
//    start = brows - ((int)((float)brows * step + 0.1f)) - start;

    float cell = start;

    // Loop through this tiles boundary heights
    for(int j = 0; j < rows; j++)
    {
        // Get the lower coordinate
        int cell0 = cell + 0.1f;

        float z =0.0f;

        if(cell0 == cell || cell0 == brows)
        {
            z = bhf->getHeight(0, cell0);
        }
        else
        {
            // get the boundary height at each coordinate then interpolate
            float z0 = bhf->getHeight(0, cell0);
            float z1 = bhf->getHeight(0, cell0 + 1);

            if(z0 == NO_DATA_VALUE || z1 == NO_DATA_VALUE)
            {
                z = NO_DATA_VALUE;
            }
            else
            {
                z = z0 + (z1 - z0) * (cell - cell0);
            }
        }

        if(z != NO_DATA_VALUE)
        {
            osg::Vec3d ndc( 1.0, ((double)j)/(double)(rows-1), z);

            osg::Vec3d model;
            _model->_tileLocator->unitToModel( ndc, model );
            osg::Vec3d v = model - center;

            (*va)[j * cols + (cols - 1)] = v;
        }

        cell += step;
    }

    va->dirty();

    _boundTileE = _boundTileE_pending;
    _boundTileE_pending = 0L;
}

void
TileNode::AdjustEdgeS(osg::Vec3d center)
{
    _boundTileS_pending->CheckOrphanedBoundaries();

    const TileKey& key = getTileModel()->_tileKey;
    unsigned int lod = key.getLOD();
    unsigned int x = key.getTileX();
//    unsigned int y = key.getTileY();

    const TileKey& bkey = _boundTileS_pending->getTileModel()->_tileKey;
    unsigned int blod = bkey.getLOD();
    unsigned int bx = bkey.getTileX();
//    unsigned int by = bkey.getTileY();

    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();

    // Get heightfield of bounding tile
    osg::HeightField* bhf  = _boundTileS_pending->getTileModel()->_elevationData.getHeightField();
    int brows = bhf->getNumRows();
    int bcols = bhf->getNumColumns();

    float step = 1.0;
    unsigned int equivx = bx;
    unsigned int mult = 1;
//    unsigned int equivy = by;
    for(int i = blod; i < lod; i++)
    {
        step = step * 0.5;
        mult *= 2;
        equivx *= 2;
//        equivy *= 2;
    }

    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    int start = ((float)(x - equivx)) *step * (float)bcols + 0.1f;

    float cell = start;

    // Loop through this tiles boundary heights
    for(int i = 0; i < cols; i++)
    {
        // Get the lower coordinate
        int cell0 = cell + 0.1f;

        float z =0.0f;

        if(cell0 == cell || cell0 == brows)
        {
            z = bhf->getHeight(cell0, brows - 1);
        }
        else
        {
            // get the boundary height at each coordinate then interpolate
            float z0 = bhf->getHeight(cell0, brows - 1);
            float z1 = bhf->getHeight(cell0 + 1, brows - 1);

            if(z0 == NO_DATA_VALUE || z1 == NO_DATA_VALUE)
            {
                z = NO_DATA_VALUE;
            }
            else
            {
                z = z0 + (z1 - z0) * (cell - cell0);
            }
        }

        if(z != NO_DATA_VALUE)
        {
            osg::Vec3d ndc( ((double)i)/(double)(cols-1), 0.0, z);

            osg::Vec3d model;
            _model->_tileLocator->unitToModel( ndc, model );
            osg::Vec3d v = model - center;

            (*va)[i] = v;
        }

        cell += step;
    }

    va->dirty();

    _boundTileS = _boundTileS_pending;
    _boundTileS_pending = 0L;
}

void
TileNode::ResetEdgeW()
{
    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();
    osg::Vec3 origin = hf->getOrigin();
    float xint = hf->getXInterval();
    float yint = hf->getYInterval();

    for(int j = 0; j < rows; j++)
    {
        float z = hf->getHeight(0, j);
        if(z != NO_DATA_VALUE)
        {
            osg::Vec3d ndc( 0.0, ((double)j)/(double)(rows-1), z);

            osg::Vec3d model;
            _model->_tileLocator->unitToModel( ndc, model );
            osg::Vec3d v = model - getMatrix().getTrans();

            (*va)[j * cols] = v;
        }
    }

    va->dirty();

    _boundTileW = 0L;
}

void
TileNode::ResetEdgeN()
{
    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();
    osg::Vec3 origin = hf->getOrigin();
    float xint = hf->getXInterval();
    float yint = hf->getYInterval();

    for(int i = 0; i < cols; i++)
    {
        float z = hf->getHeight(i, rows - 1);
        if(z != NO_DATA_VALUE)
        {
            osg::Vec3d ndc( ((double)i)/(double)(cols-1), 1.0, z);

            osg::Vec3d model;
            _model->_tileLocator->unitToModel( ndc, model );
            osg::Vec3d v = model - getMatrix().getTrans();

            (*va)[(rows - 1) * cols + i] = v;
        }
    }

    va->dirty();

    _boundTileN = 0L;
}

void
TileNode::ResetEdgeE()
{
    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();
    osg::Vec3 origin = hf->getOrigin();
    float xint = hf->getXInterval();
    float yint = hf->getYInterval();

    for(int j = 0; j < rows; j++)
    {
        float z = hf->getHeight(cols - 1, j);
        if(z != NO_DATA_VALUE)
        {
            osg::Vec3d ndc( 1.0, ((double)j)/(double)(rows-1), z);

            osg::Vec3d model;
            _model->_tileLocator->unitToModel( ndc, model );
            osg::Vec3d v = model - getMatrix().getTrans();

            (*va)[j * cols + (cols - 1)] = v;
        }
    }

    va->dirty();

    _boundTileE = 0L;
}

void
TileNode::ResetEdgeS()
{
    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();
    osg::Vec3 origin = hf->getOrigin();
    float xint = hf->getXInterval();
    float yint = hf->getYInterval();

    for(int i = 0; i < cols; i++)
    {
        float z = hf->getHeight(i, 0);
        if(z != NO_DATA_VALUE)
        {
            osg::Vec3d ndc( ((double)i)/(double)(cols-1), 0.0, z);

            osg::Vec3d model;
            _model->_tileLocator->unitToModel( ndc, model );
            osg::Vec3d v = model - getMatrix().getTrans();

            (*va)[i] = v;
        }
    }

    va->dirty();

    _boundTileS = 0L;
}

