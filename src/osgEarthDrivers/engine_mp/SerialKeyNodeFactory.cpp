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
#include "SerialKeyNodeFactory"
#include "DynamicLODScaleCallback"
#include "FileLocationCallback"
#include "TilePagedLOD"

#include <osgEarth/Registry>
#include <osgEarth/HeightFieldUtils>
#include <osgEarth/Progress>
#include <osg/PagedLOD>
#include <osg/CullStack>
#include <osg/Uniform>

#include <osgEarth/MapNode>

using namespace osgEarth_engine_mp;
using namespace osgEarth;
using namespace OpenThreads;

#define LC "[SerialKeyNodeFactory] "

namespace
{
    struct MyProgressCallback : public osgEarth::ProgressCallback
    {
        osg::observer_ptr<osg::PagedLOD> _plod;

        MyProgressCallback( osg::PagedLOD* plod )
            : _plod(plod) { }

        bool isCanceled() const
        {
            if ( _canceled )
                return true;

            if ( !_plod.valid() )
            {
                _canceled = true;
                OE_INFO << "CANCEL, plod = null." << std::endl;
            }
            else
            {
                osg::ref_ptr<osg::PagedLOD> plod = _plod.get();
                if ( !plod.valid() )
                {
                    _canceled = true;
                    OE_INFO << "CANCEL, plod = null." << std::endl;
                }
                else
                {
                    osg::ref_ptr<osg::Referenced> dbr = plod->getDatabaseRequest( 1 );
                    if ( !dbr.valid() || dbr->referenceCount() < 2 )
                    {
                        _canceled = true;
                        OE_INFO << "CANCEL, REFCOUNT = " << dbr->referenceCount() << std::endl;
                    }
                }
            }

            return _canceled;
        }
    };
}


SerialKeyNodeFactory::SerialKeyNodeFactory(TileModelFactory*             modelFactory,
                                           TileModelCompiler*            modelCompiler,
                                           TileNodeRegistry*             liveTiles,
                                           TileNodeRegistry*             deadTiles,
                                           const MPTerrainEngineOptions& options,
                                           const MapInfo&                mapInfo,
                                           TerrainNode*                  terrain,
                                           UID                           engineUID ) :
_modelFactory    ( modelFactory ),
_modelCompiler   ( modelCompiler ),
_liveTiles       ( liveTiles ),
_deadTiles       ( deadTiles ),
_options         ( options ),
_mapInfo         ( mapInfo ),
_terrain         ( terrain ),
_engineUID       ( engineUID )
{
    //nop
}


osg::Node*
SerialKeyNodeFactory::createTile(TileModel* model,
                                 bool       tileHasRealData)
{
    // compile the model into a node:
    TileNode* tileNode = _modelCompiler->compile( model );

    // see if this tile might have children.
    bool prepareForChildren =
        (tileHasRealData || (_options.minLOD().isSet() && model->_tileKey.getLOD() < *_options.minLOD())) &&
        model->_tileKey.getLOD() < *_options.maxLOD();

    osg::Node* result = 0L;

    if ( prepareForChildren )
    {
        //Compute the min range based on the 2D size of the tile
        osg::BoundingSphere bs = tileNode->getBound();
        GeoExtent extent = model->_tileKey.getExtent();
        GeoPoint lowerLeft(extent.getSRS(), extent.xMin(), extent.yMin(), 0.0, ALTMODE_ABSOLUTE);
        GeoPoint upperRight(extent.getSRS(), extent.xMax(), extent.yMax(), 0.0, ALTMODE_ABSOLUTE);
        osg::Vec3d ll, ur;
        lowerLeft.toWorld( ll );
        upperRight.toWorld( ur );
        double radius = (ur - ll).length() / 2.0;
        float minRange = (float)(radius * _options.minTileRangeFactor().value());

        osgDB::Options* dbOptions = Registry::instance()->cloneOrCreateOptions();

        TileGroup* plod = new TileGroup(tileNode, _engineUID, _liveTiles.get(), _deadTiles.get(), dbOptions);
        plod->setSubtileRange( minRange );


#if USE_FILELOCATIONCALLBACK
        dbOptions->setFileLocationCallback( new FileLocationCallback() );
#endif

        result = plod;
    }
    else
    {
        result = tileNode;
    }

    // this one rejects back-facing tiles:
    if ( _mapInfo.isGeocentric() && _options.clusterCulling() == true )
    {
        osg::HeightField* hf =
            model->_elevationData.getHeightField();

        result->addCullCallback( HeightFieldUtils::createClusterCullingCallback(
            hf,
            tileNode->getKey().getProfile()->getSRS()->getEllipsoid(),
            *_options.verticalScale() ) );
    }

    return result;
}


osg::Node*
SerialKeyNodeFactory::createRootNode( const TileKey& key )
{
    osg::ref_ptr<TileModel> model;
    bool                    real;

    _modelFactory->createTileModel( key, model, real, _options.noDataHeight().value() );
    return createTile( model.get(), real );
}


osg::Node*
SerialKeyNodeFactory::createNode( const TileKey& key, ProgressCallback* progress )
{
    osg::ref_ptr<TileModel> model;
    bool                    isReal;

    if ( progress && progress->isCanceled() )
        return 0L;

    _modelFactory->createTileModel(key, model, isReal, _options.noDataHeight().value());

    if ( progress && progress->isCanceled() )
        return 0L;

    if ( isReal || _options.minLOD().isSet() || key.getLOD() == 0 )
    {
        return createTile( model.get(), isReal);
    }
    else
    {
        return 0L;
    }
}
