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

    // If we have a valid model then reserve space in the index array
    if(_model)
    {
        osg::HeightField* hf  = _model->_elevationData.getHeightField();
        int rows = hf->getNumRows();
        int cols = hf->getNumColumns();

        _indices.reserve(rows * cols);
    }
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

        // Flag that this tile is being used
        _usedLastFrame = true;
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
    // Flag that this tile is not being used
    _usedLastFrame = false;

    // Reset pending operations
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
    //Check if the boundry tiles are still being used in display. If not, then remove any adjustments that may have been made
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
TileNode::AdjustEdges(bool sameLod)
{
    osg::Vec3d center = getMatrix().getTrans();

    // Check each boundary to see if an adjustment is required
    if(_boundTileW_pending.valid())
    {
        // Don't try to match adjoining tiles with the same LOD, just reset them.
        if(sameLod && getTileModel()->_tileKey.getLOD() == _boundTileW_pending->getTileModel()->_tileKey.getLOD())
        {
            ResetEdgeW();
            // Reset the adjoinging edge of our same-lod neighbour if it hasn't already been reset.
            if(_boundTileW_pending->getBoundTileE())
            {
                _boundTileW_pending->ResetEdgeE();
                _boundTileW_pending->setBoundTileE(0L);
            }
            _boundTileW_pending = 0L;
        }
        else if(!sameLod && getTileModel()->_tileKey.getLOD() != _boundTileW_pending->getTileModel()->_tileKey.getLOD()) AdjustEdgeW(center);
    }

    if(_boundTileN_pending.valid())
    {
        // Don't try to match adjoining tiles with the same LOD, just reset them.
        if(sameLod && getTileModel()->_tileKey.getLOD() == _boundTileN_pending->getTileModel()->_tileKey.getLOD())
        {
            ResetEdgeN();
            // Reset the adjoinging edge of our same-lod neighbour if it hasn't already been reset.
            if(_boundTileN_pending->getBoundTileS())
            {
                _boundTileN_pending->ResetEdgeS();
                _boundTileN_pending->setBoundTileS(0L);
            }
            _boundTileN_pending = 0L;
        }
        else if(!sameLod && getTileModel()->_tileKey.getLOD() != _boundTileN_pending->getTileModel()->_tileKey.getLOD()) AdjustEdgeN(center);
    }

    if(_boundTileE_pending.valid())
    {
        // Don't try to match adjoining tiles with the same LOD, just reset them.
        if(sameLod && getTileModel()->_tileKey.getLOD() == _boundTileE_pending->getTileModel()->_tileKey.getLOD())
        {
            ResetEdgeE();
            // Reset the adjoinging edge of our same-lod neighbour if it hasn't already been reset.
            if(_boundTileE_pending->getBoundTileW())
            {
                _boundTileE_pending->ResetEdgeW();
                _boundTileE_pending->setBoundTileW(0L);
            }
            _boundTileE_pending = 0L;
        }
        else if(!sameLod && getTileModel()->_tileKey.getLOD() != _boundTileE_pending->getTileModel()->_tileKey.getLOD()) AdjustEdgeE(center);
    }

    if(_boundTileS_pending.valid())
    {
        // Don't try to match adjoining tiles with the same LOD, just reset them.
        if(sameLod && getTileModel()->_tileKey.getLOD() == _boundTileS_pending->getTileModel()->_tileKey.getLOD())
        {
            ResetEdgeS();
            // Reset the adjoinging edge of our same-lod neighbour if it hasn't already been reset.
            if(_boundTileS_pending->getBoundTileN())
            {
                _boundTileS_pending->ResetEdgeN();
                _boundTileS_pending->setBoundTileN(0L);
            }
            _boundTileS_pending = 0L;
        }
        else if(!sameLod && getTileModel()->_tileKey.getLOD() != _boundTileS_pending->getTileModel()->_tileKey.getLOD()) AdjustEdgeS(center);
    }

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
    double start = (double)tilesfrombottom * step * (double)(brows - 1);

    double cell = start;

    osg::Vec3d shift = _boundTileW_pending->getMatrix().getTrans() - getMatrix().getTrans();

    std::vector< short >& bindices = _boundTileW_pending->getIndices();

    // Loop through this tiles boundary heights
    for(int j = 0; j < rows; j++)
    {
        int index = _indices[j * cols];
        if(index >= 0)
        {
            // Get the lower coordinate
            int cell0 = cell + 0.001;

            osg::Vec3 vert;
            int bindex = bindices[cell0 * bcols + bcols - 1];

            if(fabs(cell0 - cell) < 0.001 || cell0 == brows - 1)
            {
                if(bindex >= 0) vert = (*bva)[ bindex ];
            }
            else
            {
                // get the vertex at each coordinate then interpolate
                int bindex1 = bindices[(cell0 + 1) * bcols + bcols - 1];

                if(bindex < 0 || bindex1 < 0)
                {
                    bindex = -1;
                }
                else
                {
                    osg::Vec3 vert0 = (*bva)[ bindex ];
                    osg::Vec3 vert1 = (*bva)[ bindex1 ];
                    vert = vert0 + ((vert1 - vert0) * (cell - (float)cell0));
                }
            }

            if(bindex >= 0)
            {
                (*va)[ index ] = vert + shift;
            }

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
    for(int i = blod; i < lod; i++)
    {
        step = step * 0.5;
        equivx *= 2;
    }


    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    double start = ((double)(x - equivx)) *step * (double)(bcols - 1);

    double cell = start;

    osg::Vec3d shift = _boundTileN_pending->getMatrix().getTrans() - getMatrix().getTrans();

    std::vector< short >& bindices = _boundTileN_pending->getIndices();

    // Loop through this tiles boundary heights
    for(int i = 0; i < cols; i++)
    {
        int index = _indices[(rows - 1) * cols + i];
        if(index >= 0)
        {
            // Get the lower coordinate
            int cell0 = cell + 0.001;

            osg::Vec3 vert;
            int bindex = bindices[cell0];

            if(fabs(cell0 - cell) < 0.0001 || cell0 == bcols - 1)
            {
                if(bindex >= 0) vert = (*bva)[ bindex ];
            }
            else
            {
                // get the boundary height at each coordinate then interpolate
                int bindex1 = bindices[cell0 + 1];

                if(bindex < 0 || bindex1 < 0)
                {
                    bindex = -1;
                }
                else
                {
                    osg::Vec3 vert0 = (*bva)[ bindex ];
                    osg::Vec3 vert1 = (*bva)[ bindex1 ];
                    vert = vert0 + ((vert1 - vert0) * (cell - (float)cell0));
                }
            }

            if(bindex >= 0)
            {
                (*va)[ index ] = vert + shift;;
            }
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
    double start = (double)tilesfrombottom * step * (double)(brows - 1);

    double cell = start;

    osg::Vec3d shift = _boundTileE_pending->getMatrix().getTrans() - getMatrix().getTrans();

    std::vector< short >& bindices = _boundTileE_pending->getIndices();

    // Loop through this tiles boundary heights
    for(int j = 0; j < rows; j++)
    {
        int index = _indices[j * cols + (cols - 1)];
        if(index >= 0)
        {
            // Get the lower coordinate
            int cell0 = cell + 0.001;

            osg::Vec3 vert;
            int bindex = bindices[cell0 * bcols];

            if(fabs(cell0 - cell) < 0.0001 || cell0 == brows - 1)
            {
                if(bindex >= 0) vert = (*bva)[ bindex ];
            }
            else
            {
                // get the boundary height at each coordinate then interpolate
                int bindex1 = bindices[(cell0 + 1) * bcols];

                if(bindex < 0 || bindex1 < 0)
                {
                    bindex = -1;
                }
                else
                {
                    osg::Vec3 vert0 = (*bva)[ bindex ];
                    osg::Vec3 vert1 = (*bva)[ bindex1 ];
                    vert = vert0 + ((vert1 - vert0) * (cell - (float)cell0));
                }
            }

            if(bindex >= 0)
            {
                (*va)[ index ] = vert + shift;;
            }
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
    for(int i = blod; i < lod; i++)
    {
        step = step * 0.5;
        equivx *= 2;
    }

    // Figure out the start coordinate in cells of the start of this tile, within the boundary tile
    double start = ((double)(x - equivx)) * step * (double)(bcols - 1);

    double cell = start;

    osg::Vec3d shift = _boundTileS_pending->getMatrix().getTrans() - getMatrix().getTrans();

    std::vector< short >& bindices = _boundTileS_pending->getIndices();

    // Loop through this tiles boundary heights
    for(int i = 0; i < cols; i++)
    {
        int index = _indices[i];
        if(index >= 0)
        {
            // Get the lower coordinate
            int cell0 = cell + 0.001;

            osg::Vec3 vert;
            int bindex = bindices[(brows - 1) * bcols + cell0];

            if(fabs(cell0 - cell) < 0.0001 || cell0 == bcols - 1)
            {
                if(bindex >= 0) vert = (*bva)[ bindex ];
            }
            else
            {
                // get the boundary height at each coordinate then interpolate
                int bindex1 = bindices[(brows - 1) * bcols + cell0 + 1];

                if(bindex < 0 || bindex1 < 0)
                {
                    bindex = -1;
                }
                else
                {
                    osg::Vec3 vert0 = (*bva)[ bindex ];
                    osg::Vec3 vert1 = (*bva)[ bindex1 ];
                    vert = vert0 + ((vert1 - vert0) * (cell - (float)cell0));
                }
            }

            if(bindex >= 0)
            {
                (*va)[ index ] = vert + shift;;
            }
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
        int index = _indices[j * cols];
        if(index >= 0)
        {
            float z = hf->getHeight(0, j);
            if(z != NO_DATA_VALUE)
            {
                osg::Vec3d ndc( 0.0, ((double)j)/(double)(rows-1), z);

                osg::Vec3d model;
                _model->_tileLocator->unitToModel( ndc, model );
                osg::Vec3d v = model - getMatrix().getTrans();

                (*va)[ index ] = v;
            }
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
        int index = _indices[(rows - 1) * cols + i];
        if(index >= 0)
        {
            float z = hf->getHeight(i, rows - 1);
            if(z != NO_DATA_VALUE)
            {
                osg::Vec3d ndc( ((double)i)/(double)(cols-1), 1.0, z);

                osg::Vec3d model;
                _model->_tileLocator->unitToModel( ndc, model );
                osg::Vec3d v = model - getMatrix().getTrans();

                (*va)[ index ] = v;
            }
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
        int index = _indices[j * cols + (cols - 1)];
        if(index >= 0)
        {
            float z = hf->getHeight(cols - 1, j);
            if(z != NO_DATA_VALUE)
            {
                osg::Vec3d ndc( 1.0, ((double)j)/(double)(rows-1), z);

                osg::Vec3d model;
                _model->_tileLocator->unitToModel( ndc, model );
                osg::Vec3d v = model - getMatrix().getTrans();

                (*va)[ index ] = v;
            }
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
        int index = _indices[i];
        if(index >= 0)
        {
            float z = hf->getHeight(i, 0);
            if(z != NO_DATA_VALUE)
            {
                osg::Vec3d ndc( ((double)i)/(double)(cols-1), 0.0, z);

                osg::Vec3d model;
                _model->_tileLocator->unitToModel( ndc, model );
                osg::Vec3d v = model - getMatrix().getTrans();

                (*va)[ index ] = v;
            }
        }
    }

    va->dirty();

    _boundTileS = 0L;
}

