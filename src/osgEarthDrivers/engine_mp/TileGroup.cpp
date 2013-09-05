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
#include "TileGroup"
#include "TileNodeRegistry"
#include "TilePagedLOD"

using namespace osgEarth_engine_mp;
using namespace osgEarth;

#define LC "[TileGroup] "

//#define OE_TEST OE_INFO
#define OE_TEST OE_NULL

RootTileGroup::RootTileGroup()
{
    //nop
}

void
RootTileGroup::addRootKey(const TileKey&    key,
                          osg::Node*        node,
                          const UID&        engineUID,
                          TileNodeRegistry* live,
                          TileNodeRegistry* dead,
                          osgDB::Options*   dbOptions)
{
    TilePagedLOD* lod = new TilePagedLOD(this, key, engineUID, live, dead);
    lod->setDatabaseOptions( dbOptions );
    lod->addChild( node );
    lod->setNumChildrenThatCannotBeExpired( 1 );
    lod->setFamilyReady( true );
    this->addChild( lod );
}

//----------------------------------------------------------------

TileGroup::TileGroup() :
_tilenode      ( 0L ),
_ignoreSubtiles( false ),
_subtileRange  ( FLT_MAX ),
_forceSubdivide( false )
{
    //nop
}


TileGroup::TileGroup(TileNode*         tilenode,
                     const UID&        engineUID,
                     TileNodeRegistry* live,
                     TileNodeRegistry* dead,
                     osgDB::Options*   dbOptions) :
_ignoreSubtiles( false ),
_subtileRange  ( FLT_MAX ),
_forceSubdivide( false )
{
    this->addChild( tilenode );
    _tilenode = tilenode;

    for(unsigned q=0; q<4; ++q)
    {
        TileKey subkey = tilenode->getKey().createChildKey(q);
        TilePagedLOD* lod = new TilePagedLOD(this, subkey, engineUID, live, dead);
        lod->setDatabaseOptions( dbOptions );
        lod->setCenter( tilenode->getBound().center() );
        lod->setRadius( tilenode->getBound().radius() );
        this->addChild( lod );
    }
}

void
TileGroup::setForceSubdivide(bool value)
{
    _forceSubdivide = value;
}

void
TileGroup::setTileNode(TileNode* tilenode)
{
    _tilenode = tilenode;
    this->setChild( 0, tilenode );

    // Should not really need to do this, but ok
    for(unsigned q=0; q<4; ++q)
    {
        TilePagedLOD* lod = static_cast<TilePagedLOD*>(_children[1+q].get());
        lod->setCenter( tilenode->getBound().center() );
        lod->setRadius( tilenode->getBound().radius() );
    }
}


void
TileGroup::setSubtileRange(float range)
{
    _subtileRange = range;
}


osg::BoundingSphere
TileGroup::computeBound() const
{
    if ( _tilenode )
        return _tilenode->computeBound();
    else
        return osg::Group::computeBound();
}


void
TileGroup::traverse(osg::NodeVisitor& nv)
{
    if ( _tilenode && nv.getTraversalMode() == nv.TRAVERSE_ACTIVE_CHILDREN )
    {
        float range = nv.getDistanceToViewPoint( getBound().center(), true );

        // collect information about the paged children:
        bool     considerSubtiles      = false;
        bool     subtileFamilyReady    = false;

        if ( range <= _subtileRange )
        {
            // if we're ignoring subtiles (because we preivously determined that they
            // were all upsampled), check to see if we need to re-access.
            if ( _ignoreSubtiles )
            {
                if ( getTileNode()->isOutOfDate() )
                {
                    _ignoreSubtiles = false;
                }
            }

            // if we're in range, consider whether to use the subtiles.
            if ( !_ignoreSubtiles )
            {
                unsigned numSubtilesLoaded     = 0;
                unsigned numSubtilesUpsampled  = 0;
                unsigned numSubtilesLoading    = 0;

                considerSubtiles = true;

                // collect stats on the (potential) subtiles:
                subtileFamilyReady = true;

                for( unsigned q=0; q<4; ++q )
                {
                    TilePagedLOD* plod = static_cast<TilePagedLOD*>(_children[1+q].get());

                    if ( plod->isCanceled() )
                    {
                        considerSubtiles = false;
                        break;
                    }
                    if ( plod->isLoaded() || plod->getNumChildrenThatCannotBeExpired() > 0 )
                        ++numSubtilesLoaded;

                    if ( plod->isUpsampled() )
                        ++numSubtilesUpsampled;

                    if ( plod->isLoading() )
                        ++numSubtilesLoading;
                }

                // if we don't have a complete set of loaded subtiles, OR is ALL
                // subtiles are upsampled, don't use them. (NOTE: numSubtilesLoading
                // also includes tiles that are updating/replacing their data, so do NOT
                // include this in the test.)
                if ( numSubtilesLoaded < 4 )
                {
                    subtileFamilyReady = false;
                }

                // if all the subtiles contain upsampled data, and none of them are trying
                // to load new data, we can ignore them all. (..unless "force" is on, which is the
                // case if we are trying to read a minLOD for the terrain.)
                if ( numSubtilesUpsampled >= 4 && numSubtilesLoading == 0 && !_forceSubdivide )
                {
                    considerSubtiles = false;
                    _ignoreSubtiles = true;
                }
            }
        }

        if ( considerSubtiles )
        {
            _tilenode->resetUsedLastFrameFlag();

            for( unsigned q=0; q<4; ++q )
            {
                TilePagedLOD* plod = static_cast<TilePagedLOD*>(_children[1+q].get());
                plod->setFamilyReady( subtileFamilyReady );
                plod->accept( nv );
            }

            // update the TileNode so it knows what frame we're in.
            if ( nv.getFrameStamp() )
            {
                _tilenode->setLastTraversalFrame( nv.getFrameStamp()->getFrameNumber() );
            }
        }

        if ( !considerSubtiles || !subtileFamilyReady || range > _subtileRange )
        {
            if(!_tilenode->getUsedLastFrame())
            {
                for( unsigned q=0; q<4; ++q )
                {
                    TilePagedLOD* tpl = static_cast<TilePagedLOD*>( getChild(1+q) );
                    tpl->resetUsedLastFrameFlags();
                }
            }

            _tilenode->accept( nv );
        }
    }
    else
    {
        osg::Group::traverse( nv );
    }
}

void
TileGroup::resetUsedLastFrameFlags()
{
	_tilenode->resetUsedLastFrameFlag();

	for( unsigned q=0; q<4; ++q )
	{
		TilePagedLOD* tpl = static_cast<TilePagedLOD*>( getChild(1+q) );
		tpl->resetUsedLastFrameFlags();
	}

}

void
TileGroup::GetDisplayedTilesForTarget(unsigned int x, unsigned int y, unsigned int lod, MPTerrainEngineNode::Side side, std::vector< TileNode* >& tnv)
{
    if(_tilenode->getUsedLastFrame())
    {
        tnv.push_back(_tilenode);

        const TileKey& key = _tilenode->getTileModel()->_tileKey;
        unsigned int lod = key.getLOD();
        unsigned int x = key.getTileX();
        unsigned int y = key.getTileY();
//        std::cout << "      Displayed TileNode: " << _tilenode << " (" << x << ", " << y << ", " << lod << ")" << "\n";
    }
    else
    {
        // If our current LOD is less than or equal to the target LOD, then we know that one TileNode will cover the entire boundary requested.
        // If this LOD is greater than the target LOD, then we will need to include all subtiles along the requested boundary.
        unsigned int thislod = _tilenode->getKey().getLOD();

        if(thislod < lod)
        {
            // Work out the index of the subtile that will lead us to our target tile.
            // We do this by looking at the most significant bit of the provided x & y location at LOD0 to provide the x & y index of the target subtile,
            // working our way to the LSB at LOD lod.

            // Calculate which bit to use
            unsigned int bit = 0x01 << ( lod - thislod - 1);
            unsigned int ix = x & bit;
            unsigned int iy = y & bit;

            // Combine to form subtile index
            unsigned index = 0;
            if(iy) index += 2;
            if(ix) index += 1;

            CollectTargetTiles(index, x, y, lod, side, tnv);
        }
        else
        {
            //We must include all tiles along the requested boundary
            if(side == MPTerrainEngineNode::Side_W)
            {
                CollectTargetTiles(0, x, y, lod, side, tnv);
                CollectTargetTiles(2, x, y, lod, side, tnv);
            }
            else if(side == MPTerrainEngineNode::Side_N)
            {
                CollectTargetTiles(0, x, y, lod, side, tnv);
                CollectTargetTiles(1, x, y, lod, side, tnv);
            }
            else if(side == MPTerrainEngineNode::Side_E)
            {
                CollectTargetTiles(1, x, y, lod, side, tnv);
                CollectTargetTiles(3, x, y, lod, side, tnv);
            }
            else if(side == MPTerrainEngineNode::Side_S)
            {
                CollectTargetTiles(2, x, y, lod, side, tnv);
                CollectTargetTiles(3, x, y, lod, side, tnv);
            }
        }
    }
}

void
TileGroup::CollectTargetTiles(unsigned int subtile,  int x, unsigned int y, unsigned int lod, MPTerrainEngineNode::Side side, std::vector< TileNode* >& tnv)
{
    if ( numSubtilesLoaded() != 4 )
    {
        // We push a NULL pointer onto the vector so that we know how many TileNodes have been used.
        tnv.push_back(0L);
        return;
    }

    //Tilenode is the first child, so we need to add 1 to get to first subtile
    TilePagedLOD* tpl = static_cast<TilePagedLOD*>( getChild(1+subtile) );

    osg::Node* node = tpl->getChild(0);

    // Node will usually be a TileGroup
    TileGroup* tg = dynamic_cast<TileGroup*>(node);
    if(tg)
    {
        tg->GetDisplayedTilesForTarget(x, y, lod, side, tnv);
    }
    else
    {
        // If TilePageLOD parents a TileNode then we have reached a leaf node.
        TileNode* tn = dynamic_cast<TileNode*>(node);
        if(tn)
        {
            if(tn->getUsedLastFrame())
            {
                tnv.push_back(tn);
            }
        }
    }
}
