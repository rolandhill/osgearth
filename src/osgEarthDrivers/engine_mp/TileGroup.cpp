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
#include "TilePagedLOD"
#include "TileNode"

#include <osg/NodeVisitor>

using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;

#define LC "[TileGroup] "

namespace
{
    struct UpdateAgent : public osg::PagedLOD
    {
        UpdateAgent(TileGroup* tilegroup) : _tilegroup(tilegroup)
        {
            std::string fn = Stringify()
                << tilegroup->getKey().str()
                << "." << tilegroup->getEngineUID()
                << ".osgearth_engine_mp_standalone_tile";

            this->setFileName(0, fn);
            this->setRange   (0, 0, FLT_MAX);
            this->setCenter  (tilegroup->getBound().center());
        }

        virtual bool addChild(osg::Node* node)
        {
            if ( node )
            {
                osg::ref_ptr<TileGroup> tilegroup;
                if ( _tilegroup.lock(tilegroup) )
                {
                    tilegroup->applyUpdate( node );
                    this->_perRangeDataList.resize(0);
                }
            }
            else
            {
                OE_DEBUG << LC << "Internal: UpdateAgent for " << _tilegroup->getKey().str() << "received a NULL add."
                    << std::endl;
            }
            return true;
        }

        osg::observer_ptr<TileGroup> _tilegroup;
    };
}

//------------------------------------------------------------------------

TileGroup::TileGroup(const TileKey&    key, 
                     const UID&        engineUID,
                     TileNodeRegistry* live,
                     TileNodeRegistry* dead) :
_key      ( key ),
_engineUID( engineUID ),
_live     ( live ),
_dead     ( dead )
{
    this->setName( key.str() );
}

TileNode*
TileGroup::getTileNode(unsigned q)
{
    osg::Node* child = getChild(q);
    TilePagedLOD* plod = dynamic_cast<TilePagedLOD*>( child );
    if ( plod ) return plod->getTileNode();
    return static_cast<TileNode*>( child );
}

void
TileGroup::applyUpdate(osg::Node* node)
{
    if ( node )
    {
        OE_DEBUG << LC << "Update received for tile " << _key.str() << std::endl;

        TileGroup* update = dynamic_cast<TileGroup*>( node );
        if ( !update )
        {
            OE_WARN << LC << "Internal error: update was not a TileGroup" << std::endl;
            return;
        }

        if ( update->getNumChildren() < 4 )
        {
            OE_WARN << LC << "Internal error: update did not have 4 children" << std::endl;
            return;
        }

        for(unsigned i=0; i<4; ++i)
        {
            TileNode* newTileNode = dynamic_cast<TileNode*>( update->getChild(i) );
            if ( !newTileNode )
            {
                OE_WARN << LC << "Internal error; update child was not a TileNode" << std::endl;
                return;
            }

            osg::ref_ptr<TileNode> oldTileNode = 0L;

            TilePagedLOD* plod = dynamic_cast<TilePagedLOD*>(_children[i].get());
            if ( plod )
            {
                oldTileNode = plod->getTileNode();
                plod->setTileNode( newTileNode );
                if ( _live.valid() )
                    _live->move( oldTileNode.get(), _dead.get() );
            }
            else
            {
                // must be a TileNode leaf, so replace it here.
                oldTileNode = dynamic_cast<TileNode*>(_children[i].get());
                if ( !oldTileNode.valid() )
                {
                    OE_WARN << LC << "Internal error; existing child was not a TilePagedLOD or a TileNode" << std::endl;
                    return;
                }

                this->setChild( i, newTileNode );
                if ( _live.valid() )
                    _live->move( oldTileNode.get(), _dead.get() );
            }

            if ( _live.valid() )
                _live->add( newTileNode );
        }
    }

    // deactivate the update agent
    _updateAgent = 0L;
}

void
TileGroup::traverse(osg::NodeVisitor& nv)
{
    if ( nv.getVisitorType() == nv.CULL_VISITOR )
    {
        // only check for update if an update isn't already in progress:
        if ( !_updateAgent.valid() )
        {
            bool updateRequired = false;
            for( unsigned q=0; q<4; ++q)
            {
                if ( getTileNode(q)->isOutOfDate() )
                {
                    updateRequired = true;
                    break;
                }
            }

            if ( updateRequired )
            {
                // lock keeps multiple traversals from doing the same thing
                Threading::ScopedMutexLock exclusive( _updateMutex );

                // double check to prevent a race condition:
                if ( !_updateAgent.valid() )
                {
                    _updateAgent = new UpdateAgent(this);
                }
            }
        }

        if ( _updateAgent.valid() )
        {
            _updateAgent->accept( nv );
        }

        //If this TileGroup is being traversed, then the sibling TileNode is not, so we reset some values.
        TilePagedLOD* tpl = static_cast<TilePagedLOD*>( getParent(0) );
        if(tpl->getNumChildren() > 0)
        {
            TileNode* tn = dynamic_cast<TileNode*>( tpl->getChild(0) );
            if(tn)
            {
                tn->resetUsedLastFrameFlag();
            }
        }
    }

    osg::Group::traverse( nv );
}

void
TileGroup::resetUsedLastFrameFlags()
{
    if( getNumChildren() < 4 ) return;

	for( unsigned q=0; q<4; ++q )
	{
        osg::Node* node = getChild(q);
		TilePagedLOD* tpl = dynamic_cast<TilePagedLOD*>( node );
		if(tpl)
		{
            TileNode* tn = tpl->getTileNode();
            if(tn)
            {
                if(tn->getUsedLastFrame())
                {
                    // If this TileNode was used then just reset it
                    tn->resetUsedLastFrameFlag();
                }
                else
                {
                    // ... Otherwise propogate the reset request
                    if(tpl->getNumChildren() > 1)
                    {
                        TileGroup* tg = dynamic_cast<TileGroup*>(tpl->getChild(1));
                        if(tg)
                        {
                            tg->resetUsedLastFrameFlags();
                        }
                    }
                }
            }
		}
	}

}


void
TileGroup::GetDisplayedTilesForTarget(unsigned int x, unsigned int y, unsigned int lod, MPTerrainEngineNode::Side side, std::vector< TileNode* >& tnv)
{
    if( getNumChildren() < 4 ) return;

    // If the LOD of the Tiles under this group (our current LOD + 1) is less than or equal to the target LOD, then we know that one TileNode will cover the entire boundary requested.
    // If the LOD of the Tiles under this group (our current LOD + 1) is greater than the target LOD, then we will need to include all subtiles along the requested boundary.
    unsigned int thislod = _key.getLOD();

    if(thislod < lod)
    {
        // Work out the index of the subtile that will lead us to our target tile.
        // We do this by looking at the most significant bit of the provided x & y location at LOD0 to provide the x & y index of the target subtile,
        // working our way to the LSB at LOD lod.

        // Calculate which bit to use, correcting for the Tiles being 1 LOD greater than this TileGroup
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

void
TileGroup::CollectTargetTiles(unsigned int subtile,  int x, unsigned int y, unsigned int lod, MPTerrainEngineNode::Side side, std::vector< TileNode* >& tnv)
{
    // Get the subtile
    osg::Node* node = getChild(subtile);

    //If this is a TileNode and we have got this far then it is a displayed leaf, so we just add it
    TileNode* tn = dynamic_cast<TileNode*>(node);
    if(tn)
    {
        tnv.push_back(tn);
    }
    else
    {
        // If this is a TilePagedLOD, then we need to test the associated TileNode to see if it was used.
        TilePagedLOD* tpl = dynamic_cast<TilePagedLOD*>(node);
        if(tpl)
        {
            TileNode* tpltn = tpl->getTileNode();
            if(tpltn)
            {
                if(tpltn->getUsedLastFrame())
                {
                    // If it was used, then add to the list
                    tnv.push_back(tpltn);
                }
                else
                {
                    if(tpl->getNumChildren() > 1)
                    {
                        // If not used, then look to higher LODs
                        TileGroup* tpltg = dynamic_cast<TileGroup*>(tpl->getChild(1));
                        if(tpltg)
                        {
                            tpltg->GetDisplayedTilesForTarget(x, y, lod, side, tnv);
                        }
                    }
                }
            }
        }
    }
}

