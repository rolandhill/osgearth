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
    // TODO: not sure we need this.
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        //Check if the boundry tiles are still active. If not, then remove any adjustments
        if(_boundTileW.valid() && !_boundTileW->getUsedLastFrame())
        {
            _boundTileW = 0L;
            ResetEdgeW();
        }

        if(_boundTileN.valid() && !_boundTileN->getUsedLastFrame())
        {
            _boundTileN = 0L;
            ResetEdgeN();
        }

        if(_boundTileE.valid() && !_boundTileE->getUsedLastFrame())
        {
            _boundTileE = 0L;
            ResetEdgeE();
        }

        if(_boundTileS.valid() && !_boundTileS->getUsedLastFrame())
        {
            _boundTileS = 0L;
            ResetEdgeS();
        }

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
            _terrainEngineNode->RegisterChangedTileNode(this, MPTerrainEngineNode::Side_All);
        }
        _usedLastFrame = true;
    }

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
//    const TileKey& key = getTileModel()->_tileKey;
//    unsigned int lod = key.getLOD();
//    unsigned int x = key.getTileX();
//    unsigned int y = key.getTileY();
//
//    std::cout << "\n Adjusting West Side of " << this << "(" << x << ", " << y << ", " << lod << ")" << "\n";
//
//    const TileKey& key2 = _boundTileW_pending->getTileModel()->_tileKey;
//    unsigned int lod2 = key2.getLOD();
//    unsigned int x2 = key2.getTileX();
//    unsigned int y2 = key2.getTileY();
//    std::cout << "\n                  using " << _boundTileW << "(" << x2 << ", " << y2 << ", " << lod2 << ")" << "\n";
//
    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();
    osg::Vec3 origin = hf->getOrigin();
    float xint = hf->getXInterval();
    float yint = hf->getYInterval();

//    if(_model->_sampleRatio != 1.0f) std::cout << "Sample Ratio is: " << _model->_sampleRatio;

    // Get heightfield of bounding tile
    osg::HeightField* bhf  = _boundTileW_pending->getTileModel()->_elevationData.getHeightField();
    int brows = bhf->getNumRows();
    int bcols = bhf->getNumColumns();
    osg::Vec3 borigin = bhf->getOrigin();
    float bxint = bhf->getXInterval();
    float byint = bhf->getYInterval();

//    if(_boundTileW_pending->getTileModel()->_sampleRatio != 1.0f) std::cout << "Sample Ratio is: " << _boundTileW_pending->getTileModel()->_sampleRatio;

//    if(lod2 < lod)
//    {
//        x = 1;
//    }
    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    float start = (origin.y() - borigin.y()) / byint;

    // Calculate the step amount (in cells) in the boundary tile for each 1 cell step in this tile
    float step = yint / byint;

    float cell = start;

    // Loop through this tiles boundary heights
    for(int j = 0; j < rows; j++)
    {
        // Get the lower coordinate
        int cell0 = (int)cell;

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
    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();
    osg::Vec3 origin = hf->getOrigin();
    float xint = hf->getXInterval();
    float yint = hf->getYInterval();

    // Get heightfield of bounding tile
    osg::HeightField* bhf  = _boundTileN_pending->getTileModel()->_elevationData.getHeightField();
    int brows = bhf->getNumRows();
    int bcols = bhf->getNumColumns();
    osg::Vec3 borigin = bhf->getOrigin();
    float bxint = bhf->getXInterval();
    float byint = bhf->getYInterval();

    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    float start = (origin.x() - borigin.x()) / bxint;

    // Calculate the step amount (in cells) in the boundary tile for each 1 cell step in this tile
    float step = xint / bxint;

    float cell = start;

    // Loop through this tiles boundary heights
    for(int i = 0; i < cols; i++)
    {
        // Get the lower coordinate
        int cell0 = (int)cell;

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
    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();
    osg::Vec3 origin = hf->getOrigin();
    float xint = hf->getXInterval();
    float yint = hf->getYInterval();

    // Get heightfield of bounding tile
    osg::HeightField* bhf  = _boundTileE_pending->getTileModel()->_elevationData.getHeightField();
    int brows = bhf->getNumRows();
    int bcols = bhf->getNumColumns();
    osg::Vec3 borigin = bhf->getOrigin();
    float bxint = bhf->getXInterval();
    float byint = bhf->getYInterval();

    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    float start = (origin.y() - borigin.y()) / byint;

    // Calculate the step amount (in cells) in the boundary tile for each 1 cell step in this tile
    float step = yint / byint;

    float cell = start;

    // Loop through this tiles boundary heights
    for(int j = 0; j < rows; j++)
    {
        // Get the lower coordinate
        int cell0 = (int)cell;

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
    // Get the vertex array to adjust
    osg::Vec3Array* va = getVertexArray();

    // Get heightfield of this tile
    osg::HeightField* hf  = _model->_elevationData.getHeightField();
    int rows = hf->getNumRows();
    int cols = hf->getNumColumns();
    osg::Vec3 origin = hf->getOrigin();
    float xint = hf->getXInterval();
    float yint = hf->getYInterval();

    // Get heightfield of bounding tile
    osg::HeightField* bhf  = _boundTileS_pending->getTileModel()->_elevationData.getHeightField();
    int brows = bhf->getNumRows();
    int bcols = bhf->getNumColumns();
    osg::Vec3 borigin = bhf->getOrigin();
    float bxint = bhf->getXInterval();
    float byint = bhf->getYInterval();

    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    float start = (origin.x() - borigin.x()) / bxint;

    // Calculate the step amount (in cells) in the boundary tile for each 1 cell step in this tile
    float step = xint / bxint;

    float cell = start;

    // Loop through this tiles boundary heights
    for(int i = 0; i < cols; i++)
    {
        // Get the lower coordinate
        int cell0 = (int)cell;

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
}

