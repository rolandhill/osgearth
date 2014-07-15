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
#include "TilePagedLOD"
#include "TileGroup"
#include "MPTerrainEngineNode"
#include "MPGeometry"

#include <osg/ClusterCullingCallback>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Uniform>

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TileNode] "


//----------------------------------------------------------------------------

TileNode::TileNode( const TileKey& key, const TileModel* model ) :
_key               ( key ),
_model             ( model ),
_lastTraversalFrame( 0 ),
_dirty             ( false ),
_outOfDate         ( false )
{
    this->setName( key.str() );

    _frameNumber = 0;
    _terrainEngineNode = 0L;

    // revisions are initially in sync:
    if ( model )
    {
        _maprevision = model->_revision;
        if ( model->requiresUpdateTraverse() )
        {
            this->setNumChildrenRequiringUpdateTraversal(1);
        }

        osg::HeightField* hf  = _model->_elevationData.getHeightField();
        int rows = hf->getNumRows();
        int cols = hf->getNumColumns();

//        Threading::ScopedWriteLock exclusiveLock( _indicesMutex );
//        _indices.reserve(rows * cols);
    }
}


void
TileNode::setLastTraversalFrame(unsigned frame)
{
    _lastTraversalFrame = frame;
}


void
TileNode::traverse( osg::NodeVisitor& nv )
{
    if ( _model.valid() )
    {
        if ( nv.getVisitorType() == nv.CULL_VISITOR )
        {
            // If this tile is being traversed, then it is being used in display.
            // We therefore notify the TerrainEngineNode if it is being turned on from off.
            // We also need to reset the UsedLastFrame flags for higher LODs.
            if( _frameNumber != nv.getFrameStamp()->getFrameNumber() )
            {
                if(isValid()) _terrainEngineNode->RegisterChangedTileNode(this, MPTerrainEngineNode::Side_All);
            }

            // Update the frame number when this tile was last displayed
            _frameNumber = nv.getFrameStamp()->getFrameNumber();

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
        }
        else if (nv.getVisitorType() == nv.UPDATE_VISITOR)
        {
            _model->updateTraverse(nv);
        }
    }

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

void TileNode::CheckOrphanedBoundaries(unsigned int framenumber)
{
    //Check if the boundary tiles are still being used in display. If not, then remove any adjustments that may have been made
    if(_boundTileW.valid() && framenumber != _boundTileW->getFrameNumber())
    {
        ResetEdgeW();
    }

    if(_boundTileN.valid() && framenumber != _boundTileN->getFrameNumber())
    {
        ResetEdgeN();
    }

    if(_boundTileE.valid() && framenumber != _boundTileE->getFrameNumber())
    {
        ResetEdgeE();
    }

    if(_boundTileS.valid() && framenumber != _boundTileS->getFrameNumber())
    {
        ResetEdgeS();
    }
}

void
TileNode::AdjustEdges()
{
    osg::Vec3d center = getMatrix().getTrans();

    // Check each boundary to see if an adjustment is required
    if(_boundTileW_pending.valid())
    {
        AdjustEdgeW(center);
    }

    if(_boundTileN_pending.valid())
    {
        AdjustEdgeN(center);
    }

    if(_boundTileE_pending.valid())
    {
        AdjustEdgeE(center);
    }

    if(_boundTileS_pending.valid())
    {
        AdjustEdgeS(center);
    }

}

void
TileNode::AdjustEdgeW(osg::Vec3d center)
{
    if( !_boundTileW_pending->isValid() ) return;

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
    Threading::ScopedWriteLock exclusiveLock( _indicesMutex );
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
    if( !_boundTileN_pending->isValid() ) return;

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
    Threading::ScopedWriteLock exclusiveLock( _indicesMutex );
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
    if( !_boundTileE_pending->isValid() ) return;

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
    Threading::ScopedWriteLock exclusiveLock( _indicesMutex );
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
    if( !_boundTileS_pending->isValid() ) return;

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
    Threading::ScopedWriteLock exclusiveLock( _indicesMutex );
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

    Threading::ScopedWriteLock exclusiveLock( _indicesMutex );
    for(int j = 0; j < rows; j++)
    {
        int index = _indices[j * cols];
        if(index >= 0)
        {
            float z = hf->getHeight(0, j);
            osg::Vec3d ndc( 0.0, ((double)j)/(double)(rows-1), z);

            osg::Vec3d model;
            _model->_tileLocator->unitToModel( ndc, model );
            osg::Vec3d v = model - getMatrix().getTrans();

            (*va)[ index ] = v;
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

    Threading::ScopedWriteLock exclusiveLock( _indicesMutex );
    for(int i = 0; i < cols; i++)
    {
        int index = _indices[(rows - 1) * cols + i];
        if(index >= 0)
        {
            float z = hf->getHeight(i, rows - 1);
            osg::Vec3d ndc( ((double)i)/(double)(cols-1), 1.0, z);

            osg::Vec3d model;
            _model->_tileLocator->unitToModel( ndc, model );
            osg::Vec3d v = model - getMatrix().getTrans();

            (*va)[ index ] = v;
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

    Threading::ScopedWriteLock exclusiveLock( _indicesMutex );
    for(int j = 0; j < rows; j++)
    {
        int index = _indices[j * cols + (cols - 1)];
        if(index >= 0)
        {
            float z = hf->getHeight(cols - 1, j);
            osg::Vec3d ndc( 1.0, ((double)j)/(double)(rows-1), z);

            osg::Vec3d model;
            _model->_tileLocator->unitToModel( ndc, model );
            osg::Vec3d v = model - getMatrix().getTrans();

            (*va)[ index ] = v;
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

    Threading::ScopedWriteLock exclusiveLock( _indicesMutex );
    for(int i = 0; i < cols; i++)
    {
        int index = _indices[i];
        if(index >= 0)
        {
            float z = hf->getHeight(i, 0);
            osg::Vec3d ndc( ((double)i)/(double)(cols-1), 0.0, z);

            osg::Vec3d model;
            _model->_tileLocator->unitToModel( ndc, model );
            osg::Vec3d v = model - getMatrix().getTrans();

            (*va)[ index ] = v;
        }
    }

    va->dirty();

    _boundTileS = 0L;
}

void
TileNode::StoreIndices(std::vector<int>& indices)
{
    Threading::ScopedWriteLock exclusiveLock( _indicesMutex );
    _indices.clear();
    _indices.reserve(indices.size());

    std::vector<int>::iterator iit;
    for(iit = indices.begin(); iit != indices.end(); iit++)
    {
        _indices.push_back((short) *iit);
    }
}
