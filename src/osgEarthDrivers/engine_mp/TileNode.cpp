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
_lastTraversalFrame( 0 ),
_dirty             ( false ),
_outOfDate         ( false )
{
    this->setName( key.str() );

    // revisions are initially in sync:
    if ( model )
        _maprevision = model->_revision;

    // NOTE:
    // We have temporarily disabled setting of the "birth time" uniform.
    // Having a uniform on each TileNode adds a StateGraph for each TileNode and slows
    // down the DRAW time considerably. Until we find a better solution, no
    // birth time uniform here. (That only affects LOD Blending atm)

    //osg::StateSet* stateset = getOrCreateStateSet();

    // born-on date uniform.
    _bornUniform = new osg::Uniform(osg::Uniform::FLOAT, "oe_tile_birthtime");
    _bornUniform->set( -1.0f );
    //stateset->addUniform( _bornUniform );

    _usedLastFrame = false;
    _terrainEngineNode = 0L;
}


void
TileNode::setLastTraversalFrame(unsigned frame)
{
    _lastTraversalFrame = frame;
}


void
TileNode::traverse( osg::NodeVisitor& nv )
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
//        CheckOrphanedBoundaries();

        osg::ClusterCullingCallback* ccc = dynamic_cast<osg::ClusterCullingCallback*>(getCullCallback());
        if (ccc)
        {
            if (ccc->cull(&nv,0,static_cast<osg::State *>(0))) return;
        }

        // if this tile is marked dirty, bump the marker so the engine knows it
        // needs replacing.
        if ( _dirty || _model->_revision != _maprevision )
        {
            _outOfDate = true;
        }

        // reset the "birth" time if necessary - this is the time at which the
        // node passes cull (not multi-view compatible)
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
    osg::MatrixTransform::releaseGLObjects( state );

    if ( _model.valid() )
        _model->releaseGLObjects( state );
}

void
TileNode::resizeGLObjectBuffers(unsigned maxSize)
{
    osg::MatrixTransform::resizeGLObjectBuffers( maxSize );

    if ( _model.valid() )
        const_cast<TileModel*>(_model.get())->resizeGLObjectBuffers( maxSize );
}

void
TileNode::resetUsedLastFrameFlag()
{
    _usedLastFrame = false;

    //Clear all boudary node data
//    _boundTileW = 0L;
//    _boundTileN = 0L;
//    _boundTileE = 0L;
//    _boundTileS = 0L;

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
    unsigned int y = key.getTileY();

    const TileKey& bkey = _boundTileW_pending->getTileModel()->_tileKey;
    unsigned int blod = bkey.getLOD();
    unsigned int by = bkey.getTileY();

    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();
    osg::Vec3Array* bva = _boundTileW_pending->getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();

    // Get heightfield of bounding tile
    osg::HeightField* bhf  = _boundTileW_pending->getTileModel()->_elevationData.getHeightField();
    int brows = bhf->getNumRows();
    int bcols = bhf->getNumColumns();

    double step = 1.0;
    unsigned int mult = 1;
    unsigned int equivy = by;
    for(int i = blod; i < lod; i++)
    {
        step = step * 0.5;
        mult *= 2;
        equivy *= 2;
    }

    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    int tilesfromtop = y - equivy;
    int tilesfrombottom = mult - tilesfromtop - 1;
    int start = (float)tilesfrombottom * step * (float)brows + 0.1;

    double cell = start;

    osg::Vec3d shift = _boundTileW_pending->getMatrix().getTrans() - getMatrix().getTrans();

    // Loop through this tiles boundary heights
    for(int j = 0; j < rows; j++)
    {
        // Get the lower coordinate
        int cell0 = cell + 0.001;

        float z = 0.0;
        osg::Vec3 vert;

        if(fabs(cell0 - cell) < 0.0001 || cell0 == brows - 1)
        {
            z = bhf->getHeight(bcols - 1, cell0);
            vert = (*bva)[cell0 * bcols + bcols - 1];
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
                osg::Vec3 vert0 = (*bva)[cell0 * bcols + bcols - 1];
                osg::Vec3 vert1 = (*bva)[(cell0 + 1) * bcols + bcols - 1];
                vert = vert0 + ((vert1 - vert0) * (cell - (float)cell0));
            }
        }

        if(z != NO_DATA_VALUE)
        {
            (*va)[j * cols] = vert + shift;
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

    const TileKey& bkey = _boundTileN_pending->getTileModel()->_tileKey;
    unsigned int blod = bkey.getLOD();
    unsigned int bx = bkey.getTileX();

    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();
    osg::Vec3Array* bva = _boundTileN_pending->getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();

    // Get heightfield of bounding tile
    osg::HeightField* bhf  = _boundTileN_pending->getTileModel()->_elevationData.getHeightField();
    int brows = bhf->getNumRows();
    int bcols = bhf->getNumColumns();

    double step = 1.0;
    unsigned int equivx = bx;
    unsigned int mult = 1;
    for(int i = blod; i < lod; i++)
    {
        step = step * 0.5;
        mult *= 2;
        equivx *= 2;
    }


    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    int start = ((float)(x - equivx)) *step * (float)bcols + 0.1;

    double cell = start;

    osg::Vec3d shift = _boundTileN_pending->getMatrix().getTrans() - getMatrix().getTrans();

    // Loop through this tiles boundary heights
    for(int i = 0; i < cols; i++)
    {
        // Get the lower coordinate
        int cell0 = cell + 0.001;

        float z = 0.0;
        osg::Vec3 vert;

        if(fabs(cell0 - cell) < 0.0001 || cell0 == bcols - 1)
        {
            z = bhf->getHeight(cell0, 0);
            vert = (*bva)[cell0];
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
                osg::Vec3 vert0 = (*bva)[cell0];
                osg::Vec3 vert1 = (*bva)[cell0 + 1];
                vert = vert0 + ((vert1 - vert0) * (cell - (float)cell0));
            }
        }

        if(z != NO_DATA_VALUE)
        {
            (*va)[(rows - 1) * cols + i] = vert + shift;;
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
    unsigned int y = key.getTileY();

    const TileKey& bkey = _boundTileE_pending->getTileModel()->_tileKey;
    unsigned int blod = bkey.getLOD();
    unsigned int by = bkey.getTileY();

    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();
    osg::Vec3Array* bva = _boundTileE_pending->getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();

    // Get heightfield of bounding tile
    osg::HeightField* bhf  = _boundTileE_pending->getTileModel()->_elevationData.getHeightField();
    int brows = bhf->getNumRows();
    int bcols = bhf->getNumColumns();

    double step = 1.0;
    unsigned int mult = 1;
    unsigned int equivy = by;
    for(int i = blod; i < lod; i++)
    {
        step = step * 0.5;
        mult *= 2;
        equivy *= 2;
    }

    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    int tilesfromtop = y - equivy;
    int tilesfrombottom = mult - tilesfromtop - 1;
    int start = (float)tilesfrombottom * step * (float)brows + 0.1;

    double cell = start;

    osg::Vec3d shift = _boundTileE_pending->getMatrix().getTrans() - getMatrix().getTrans();

    // Loop through this tiles boundary heights
    for(int j = 0; j < rows; j++)
    {
        // Get the lower coordinate
        int cell0 = cell + 0.001;

        float z = 0.0;
        osg::Vec3 vert;

        if(fabs(cell0 - cell) < 0.0001 || cell0 == brows - 1)
        {
            z = bhf->getHeight(0, cell0);
            vert = (*bva)[cell0 * bcols];
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
                osg::Vec3 vert0 = (*bva)[cell0 * bcols];
                osg::Vec3 vert1 = (*bva)[(cell0 + 1) * bcols];
                vert = vert0 + ((vert1 - vert0) * (cell - (float)cell0));
            }
        }

        if(z != NO_DATA_VALUE)
        {
            (*va)[j * cols + (cols - 1)] = vert + shift;;
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

    const TileKey& bkey = _boundTileS_pending->getTileModel()->_tileKey;
    unsigned int blod = bkey.getLOD();
    unsigned int bx = bkey.getTileX();

    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();
    osg::Vec3Array* bva = _boundTileS_pending->getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();

    // Get heightfield of bounding tile
    osg::HeightField* bhf  = _boundTileS_pending->getTileModel()->_elevationData.getHeightField();
    int brows = bhf->getNumRows();
    int bcols = bhf->getNumColumns();

    double step = 1.0;
    unsigned int equivx = bx;
    unsigned int mult = 1;
    for(int i = blod; i < lod; i++)
    {
        step = step * 0.5;
        mult *= 2;
        equivx *= 2;
    }

    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    int start = ((float)(x - equivx)) * step * (float)bcols + 0.1;

    double cell = start;

    osg::Vec3d shift = _boundTileS_pending->getMatrix().getTrans() - getMatrix().getTrans();

    // Loop through this tiles boundary heights
    for(int i = 0; i < cols; i++)
    {
        // Get the lower coordinate
        int cell0 = cell + 0.001;

        float z = 0.0;
        osg::Vec3 vert;

        if(fabs(cell0 - cell) < 0.0001 || cell0 == brows - 1)
        {
            z = bhf->getHeight(cell0, brows - 1);
            vert = (*bva)[(brows - 1) * bcols + cell0];
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
                osg::Vec3 vert0 = (*bva)[(brows - 1) * bcols + cell0];
                osg::Vec3 vert1 = (*bva)[(brows - 1) * bcols + cell0 + 1];
                vert = vert0 + ((vert1 - vert0) * (cell - (float)cell0));
            }
        }

        if(z != NO_DATA_VALUE)
        {
            (*va)[i] = vert + shift;;
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

