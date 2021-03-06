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
#include "MPGeometry"

#include <osg/Version>
#include <osgUtil/MeshOptimizers>
#include <iterator>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>

using namespace osg;
using namespace osgEarth::Drivers::MPTerrainEngine;
using namespace osgEarth;

#define LC "[MPGeometry] "


//----------------------------------------------------------------------------

//osg::buffered_object<MPGeometry::PerGC> MPGeometry::_perGC;

MPGeometry::MPGeometry(const TileKey& key, const MapFrame& frame, int imageUnit) : 
osg::Geometry    ( ),
_frame           ( frame ),
_imageUnit       ( imageUnit )
{
    _supportsGLSL = Registry::capabilities().supportsGLSL();

    unsigned tw, th;
    key.getProfile()->getNumTiles(key.getLOD(), tw, th);
    _tileKeyValue.set( key.getTileX(), th-key.getTileY()-1.0f, key.getLOD(), -1.0f );

    _imageUnitParent = _imageUnit + 1; // temp

    // establish uniform name IDs.
    _tileKeyUniformNameID      = osg::Uniform::getNameID( "oe_tile_key" );
    _birthTimeUniformNameID    = osg::Uniform::getNameID( "oe_tile_birthtime" );
    _uidUniformNameID          = osg::Uniform::getNameID( "oe_layer_uid" );
    _orderUniformNameID        = osg::Uniform::getNameID( "oe_layer_order" );
    _opacityUniformNameID      = osg::Uniform::getNameID( "oe_layer_opacity" );
    _texMatParentUniformNameID = osg::Uniform::getNameID( "oe_layer_parent_matrix" );

    // we will set these later (in TileModelCompiler)
    this->setUseVertexBufferObjects(false);
    this->setUseDisplayList(false);
}


void
MPGeometry::renderPrimitiveSets(osg::State& state,
                                bool        renderColor,
                                bool        usingVBOs) const
{
    // check the map frame to see if it's up to date
    if ( _frame.needsSync() )
    {
        // this lock protects a MapFrame sync when we have multiple DRAW threads.
        Threading::ScopedMutexLock exclusive( _frameSyncMutex );

        if ( _frame.needsSync() && _frame.sync() ) // always double check
        {
            // This should only happen is the layer ordering changes;
            // If layers are added or removed, the Tile gets rebuilt and
            // the point is moot.
            std::vector<Layer> reordered;
            const ImageLayerVector& layers = _frame.imageLayers();
            reordered.reserve( layers.size() );
            for( ImageLayerVector::const_iterator i = layers.begin(); i != layers.end(); ++i )
            {
                std::vector<Layer>::iterator j = std::find( _layers.begin(), _layers.end(), i->get()->getUID() );
                if ( j != _layers.end() )
                    reordered.push_back( *j );
            }
            _layers.swap( reordered );
        }
    }

    unsigned layersDrawn = 0;

    // access the GL extensions interface for the current GC:
    const osg::Program::PerContextProgram* pcp = 0L;
    osg::ref_ptr<osg::GL2Extensions> ext;
    unsigned contextID;

    if (_supportsGLSL)
    {
        contextID = state.getContextID();
        ext = osg::GL2Extensions::Get( contextID, true );
        pcp = state.getLastAppliedProgramObject();
    }

    // cannot store these in the object since there could be multiple GCs (and multiple
    // PerContextPrograms) at large
    GLint tileKeyLocation       = -1;
    GLint birthTimeLocation     = -1;
    GLint opacityLocation       = -1;
    GLint uidLocation           = -1;
    GLint orderLocation         = -1;
    GLint texMatParentLocation  = -1;

    // The PCP can change (especially in a VirtualProgram environment). So we do need to
    // requery the uni locations each time unfortunately. TODO: explore optimizations.
    if ( pcp )
    {
        tileKeyLocation      = pcp->getUniformLocation( _tileKeyUniformNameID );
        birthTimeLocation    = pcp->getUniformLocation( _birthTimeUniformNameID );
        opacityLocation      = pcp->getUniformLocation( _opacityUniformNameID );
        uidLocation          = pcp->getUniformLocation( _uidUniformNameID );
        orderLocation        = pcp->getUniformLocation( _orderUniformNameID );
        texMatParentLocation = pcp->getUniformLocation( _texMatParentUniformNameID );
    }
    
    // apply the tilekey uniform once.
    if ( tileKeyLocation >= 0 )
    {
        ext->glUniform4fv( tileKeyLocation, 1, _tileKeyValue.ptr() );
    }

    // set the "birth time" - i.e. the time this tile last entered the scene in the current GC.
    if ( birthTimeLocation >= 0 )
    {
        PerContextData& pcd = _pcd[contextID];
        if ( pcd.birthTime < 0.0f )
        {
            const osg::FrameStamp* stamp = state.getFrameStamp();
            if ( stamp )
            {
                pcd.birthTime = stamp->getReferenceTime();
            }
        }
        ext->glUniform1f( birthTimeLocation, pcd.birthTime );
    }

    // activate the tile coordinate set - same for all layers
    if ( renderColor )
    {
        state.setTexCoordPointer( _imageUnit+1, _tileCoords.get() );
    }

#ifndef OSG_GLES2_AVAILABLE
    if ( renderColor )
    {
        // emit a default terrain color since we're not binding a color array:
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    }
#endif

    if ( _layers.size() > 0 )
    {
        float prev_opacity        = -1.0f;
        float prev_alphaThreshold = -1.0f;

        // first bind any shared layers. We still have to do this even if we are
        // in !renderColor mode b/c these textures could be used by vertex shaders
        // to alter the geometry.
        int sharedLayers = 0;
        if ( _supportsGLSL )
        {
            for(unsigned i=0; i<_layers.size(); ++i)
            {
                const Layer& layer = _layers[i];

                // a "shared" layer binds to a secondary texture unit so that other layers
                // can see it and use it.
                if ( layer._imageLayer->isShared() )
                {
                    ++sharedLayers;
                    int sharedUnit = layer._imageLayer->shareImageUnit().get();
                    {
                        state.setActiveTextureUnit( sharedUnit );
                        state.setTexCoordPointer( sharedUnit, layer._texCoords.get() );
                        // bind the texture for this layer to the active share unit.
                        layer._tex->apply( state );

                        // no texture LOD blending for shared layers for now. maybe later.
                    }
                }
            }
        }

        // track the active image unit.
        int activeImageUnit = -1;

        if (renderColor)
        {
            // find the first opaque layer, top-down, and start there:
            unsigned first = 0;
            for(first = _layers.size()-1; first > 0; --first)
            {
                const Layer& layer = _layers[first];
                if (layer._opaque && 
                    layer._imageLayer->getVisible() &&
                    layer._imageLayer->getOpacity() >= 1.0f)
                {
                    break;
                }
            }

            // interate over all the image layers
            for(unsigned i=first; i<_layers.size(); ++i)
            {
                const Layer& layer = _layers[i];

                if ( layer._imageLayer->getVisible() && layer._imageLayer->getOpacity() > 0.0f )
                {       
                    // activate the visible unit if necessary:
                    if ( activeImageUnit != _imageUnit )
                    {
                        state.setActiveTextureUnit( _imageUnit );
                        activeImageUnit = _imageUnit;
                    }

                    // bind the texture for this layer:
                    layer._tex->apply( state );

                    // in FFP mode, we need to enable the GL mode for texturing:
                    if (!_supportsGLSL)
                    {
                        state.applyMode(GL_TEXTURE_2D, true);
                    }

                    // if we're using a parent texture for blending, activate that now
                    if ( texMatParentLocation >= 0 && layer._texParent.valid() )
                    {
                        state.setActiveTextureUnit( _imageUnitParent );
                        activeImageUnit = _imageUnitParent;
                        layer._texParent->apply( state );
                    }

                    // bind the texture coordinates for this layer.
                    // TODO: can probably optimize this by sharing or using texture matrixes.
                    // State::setTexCoordPointer does some redundant work under the hood.
                    state.setTexCoordPointer( _imageUnit, layer._texCoords.get() );

                    // apply uniform values:
                    if ( pcp )
                    {
                        // apply opacity:
                        if ( opacityLocation >= 0 )
                        {
                            float opacity = layer._imageLayer->getOpacity();
                            if ( opacity != prev_opacity )
                            {
                                ext->glUniform1f( opacityLocation, (GLfloat)opacity );
                                prev_opacity = opacity;
                            }
                        }

                        // assign the layer UID:
                        if ( uidLocation >= 0 )
                        {
                            ext->glUniform1i( uidLocation, (GLint)layer._layerID );
                        }

                        // assign the layer order:
                        if ( orderLocation >= 0 )
                        {
                            ext->glUniform1i( orderLocation, (GLint)layersDrawn );
                        }

                        // assign the parent texture matrix
                        if ( texMatParentLocation >= 0 && layer._texParent.valid() )
                        {
                            ext->glUniformMatrix4fv( texMatParentLocation, 1, GL_FALSE, layer._texMatParent.ptr() );
                        }
                    }

                    // draw the primitive sets.
                    for(unsigned int primitiveSetNum=0; primitiveSetNum!=_primitives.size(); ++primitiveSetNum)
                    {
                        const osg::PrimitiveSet* primitiveset = _primitives[primitiveSetNum].get();
                        if ( primitiveset )
                        {
                            primitiveset->draw(state, usingVBOs);
                        }
                        else
                        {
                            OE_WARN << LC << "Strange, MPGeometry had a 0L primset" << std::endl;
                        }
                    }

                    ++layersDrawn;
                }
            }
        }
    }

    // if we didn't draw anything, draw the raw tiles anyway with no texture.
    if ( layersDrawn == 0 )
    {
        if ( opacityLocation >= 0 )
            ext->glUniform1f( opacityLocation, (GLfloat)1.0f );
        if ( uidLocation >= 0 )
            ext->glUniform1i( uidLocation, (GLint)-1 );
        if ( orderLocation >= 0 )
            ext->glUniform1i( orderLocation, (GLint)0 );

        // draw the primitives themselves.
        for(unsigned int primitiveSetNum=0; primitiveSetNum!=_primitives.size(); ++primitiveSetNum)
        {
            const osg::PrimitiveSet* primitiveset = _primitives[primitiveSetNum].get();
            primitiveset->draw(state, usingVBOs);
        }
    }

    else // at least one textured layer was drawn:
    {
        // prevent texture leakage
        // TODO: find a way to remove this to speed things up
        if ( renderColor )
        {
            glBindTexture( GL_TEXTURE_2D, 0 );
        }
    }
}

#if OSG_VERSION_GREATER_THAN(3,3,1)
#    define COMPUTE_BOUND computeBoundingBox
#else
#    define COMPUTE_BOUND computeBound
#endif

osg::BoundingBox
MPGeometry:: COMPUTE_BOUND() const
{
    osg::BoundingBox bbox = osg::Geometry:: COMPUTE_BOUND ();
    {
        // update the uniform.
        Threading::ScopedMutexLock exclusive(_frameSyncMutex);
        _tileKeyValue.w() = bbox.radius();
    }
    return bbox;
}

void
MPGeometry::validate()
{
    unsigned numVerts = getVertexArray()->getNumElements();

    for(unsigned i=0; i < _primitives.size(); ++i)
    {
        osg::DrawElements* de = static_cast<osg::DrawElements*>(_primitives[i].get());
        if ( de->getMode() != GL_TRIANGLES )
        {
            OE_WARN << LC << "Invalid primitive set - not GL_TRIANGLES" << std::endl;
            _primitives.clear();
        }

        else if ( de->getNumIndices() % 3 != 0 )
        {
            OE_WARN << LC << "Invalid primitive set - wrong number of indicies" << std::endl;
            //_primitives.clear();
            osg::DrawElementsUShort* deus = static_cast<osg::DrawElementsUShort*>(de);
            int extra = de->getNumIndices() % 3;
            deus->resize(de->getNumIndices() - extra);
            OE_WARN << LC << "   ..removed " << extra << " indices" << std::endl;
            //return;
        }
        else
        {
            for( unsigned j=0; j<de->getNumIndices(); ++j ) 
            {
                unsigned index = de->index(j);
                if ( index >= numVerts )
                {
                    OE_WARN << LC << "Invalid primitive set - index out of bounds" << std::endl;
                    _primitives.clear();
                    return;
                }
            }
        }
    }
}


void 
MPGeometry::releaseGLObjects(osg::State* state) const
{
    osg::Geometry::releaseGLObjects( state );

    for(unsigned i=0; i<_layers.size(); ++i)
    {
        const Layer& layer = _layers[i];

        //Moved to TileModel since that's where the texture is created. Releasing it
        // here could break texture sharing.
        //if ( layer._tex.valid() )
        //    layer._tex->releaseGLObjects( state );

        // Check the refcount since texcoords can be cached/shared.
        if ( layer._texCoords.valid() && layer._texCoords->referenceCount() == 1 )
            layer._texCoords->releaseGLObjects( state );
    }
}


void
MPGeometry::resizeGLObjectBuffers(unsigned maxSize)
{
    osg::Geometry::resizeGLObjectBuffers( maxSize );

    for(unsigned i=0; i<_layers.size(); ++i)
    {
        const Layer& layer = _layers[i];
        if ( layer._tex.valid() )
            layer._tex->resizeGLObjectBuffers( maxSize );
    }

    if ( _pcd.size() < maxSize )
    {
        _pcd.resize(maxSize);
    }
}


void 
MPGeometry::compileGLObjects( osg::RenderInfo& renderInfo ) const
{
    osg::Geometry::compileGLObjects( renderInfo );

    for(unsigned i=0; i<_layers.size(); ++i)
    {
        const Layer& layer = _layers[i];
        if ( layer._tex.valid() )
            layer._tex->apply( *renderInfo.getState() );
    }
}


void 
MPGeometry::drawImplementation(osg::RenderInfo& renderInfo) const
{
    //if (!_supportsGLSL)
    //{
    //    osg::Geometry::drawImplementation(renderInfo);
    //    return;
    //}

    // See if this is a pre-render depth-only camera. If so we can skip all the layers
    // and just render the primitive sets.
    osg::Camera* camera = renderInfo.getCurrentCamera();
    bool renderColor =
        (camera->getRenderOrder() != osg::Camera::PRE_RENDER) ||
        ((camera->getClearMask() & GL_COLOR_BUFFER_BIT) != 0L);

    osg::State& state = *renderInfo.getState();

    bool usingVertexBufferObjects = _useVertexBufferObjects && state.isVertexBufferObjectSupported();
    bool handleVertexAttributes = !_vertexAttribList.empty();

    osg::ArrayDispatchers& arrayDispatchers = state.getArrayDispatchers();

    arrayDispatchers.reset();
    arrayDispatchers.setUseVertexAttribAlias(state.getUseVertexAttributeAliasing());


    //Remove 
#if OSG_VERSION_LESS_THAN(3,1,8)
    arrayDispatchers.setUseGLBeginEndAdapter(false);
#endif

#if OSG_MIN_VERSION_REQUIRED(3,1,8)
    arrayDispatchers.activateNormalArray(_normalArray.get());
    if (renderColor)
    {
        arrayDispatchers.activateColorArray(_colorArray.get());
        arrayDispatchers.activateSecondaryColorArray(_secondaryColorArray.get());
        arrayDispatchers.activateFogCoordArray(_fogCoordArray.get());
    }
#else
    arrayDispatchers.activateNormalArray(_normalData.binding, _normalData.array.get(), _normalData.indices.get());
    if (renderColor)
    {
        arrayDispatchers.activateColorArray(_colorData.binding, _colorData.array.get(), _colorData.indices.get());
        arrayDispatchers.activateSecondaryColorArray(_secondaryColorData.binding, _secondaryColorData.array.get(), _secondaryColorData.indices.get());
        arrayDispatchers.activateFogCoordArray(_fogCoordData.binding, _fogCoordData.array.get(), _fogCoordData.indices.get());
    }
#endif
    

    if (handleVertexAttributes)
    {
        for(unsigned int unit=0;unit<_vertexAttribList.size();++unit)
        {
#if OSG_MIN_VERSION_REQUIRED(3,1,8)
            arrayDispatchers.activateVertexAttribArray(unit, _vertexAttribList[unit].get());
#else
            arrayDispatchers.activateVertexAttribArray(_vertexAttribList[unit].binding, unit, _vertexAttribList[unit].array.get(), _vertexAttribList[unit].indices.get());
#endif             
        }
    }

    // dispatch any attributes that are bound overall
    arrayDispatchers.dispatch(BIND_OVERALL,0);
    state.lazyDisablingOfVertexAttributes();


    // set up arrays
#if OSG_MIN_VERSION_REQUIRED( 3, 1, 8 )
    if( _vertexArray.valid() )
        state.setVertexPointer(_vertexArray.get());

    if (_normalArray.valid() && _normalArray->getBinding()==osg::Array::BIND_PER_VERTEX)
        state.setNormalPointer(_normalArray.get());

    if (renderColor && _colorArray.valid() && _colorArray->getBinding()==osg::Array::BIND_PER_VERTEX)
        state.setColorPointer(_colorArray.get());

    if (renderColor && _secondaryColorArray.valid() && _secondaryColorArray->getBinding()==osg::Array::BIND_PER_VERTEX)
        state.setSecondaryColorPointer(_secondaryColorArray.get());

    if (renderColor && _fogCoordArray.valid() && _fogCoordArray->getBinding()==osg::Array::BIND_PER_VERTEX)
        state.setFogCoordPointer(_fogCoordArray.get());
#else
    if( _vertexData.array.valid() )
        state.setVertexPointer(_vertexData.array.get());

    if (_normalData.binding==BIND_PER_VERTEX && _normalData.array.valid())
        state.setNormalPointer(_normalData.array.get());

    if (renderColor && _colorData.binding==BIND_PER_VERTEX && _colorData.array.valid())
        state.setColorPointer(_colorData.array.get());

    if (renderColor && _secondaryColorData.binding==BIND_PER_VERTEX && _secondaryColorData.array.valid())
        state.setSecondaryColorPointer(_secondaryColorData.array.get());

    if (renderColor && _fogCoordData.binding==BIND_PER_VERTEX && _fogCoordData.array.valid())
        state.setFogCoordPointer(_fogCoordData.array.get());
#endif    
        
    for(unsigned int unit=0;unit<_texCoordList.size();++unit)
    {
#if OSG_MIN_VERSION_REQUIRED( 3, 1, 8)
        const Array* array = _texCoordList[unit].get();
        if (array)
        {
            state.setTexCoordPointer(unit,array);
        }
#else
        const osg::Array* array = _texCoordList[unit].array.get();
        if (array) state.setTexCoordPointer(unit,array);
#endif        
    }

    if( handleVertexAttributes )
    {
        for(unsigned int index = 0; index < _vertexAttribList.size(); ++index )
        {
#if OSG_MIN_VERSION_REQUIRED( 3, 1, 8)
            const Array* array = _vertexAttribList[index].get();
            if (array && array->getBinding()==osg::Array::BIND_PER_VERTEX)
            {
                if (array->getPreserveDataType())
                {
                    GLenum dataType = array->getDataType();
                    if (dataType==GL_FLOAT) state.setVertexAttribPointer( index, array );
                    else if (dataType==GL_DOUBLE) state.setVertexAttribLPointer( index, array );
                    else state.setVertexAttribIPointer( index, array );
                }
                else
                {
                    state.setVertexAttribPointer( index, array );
                }
            }
#else            
            const osg::Array* array = _vertexAttribList[index].array.get();
            const AttributeBinding ab = _vertexAttribList[index].binding;
            if( ab == BIND_PER_VERTEX && array )
            {
                state.setVertexAttribPointer( index, array, _vertexAttribList[index].normalize );
            }
#endif
        }
    }

    state.applyDisablingOfVertexAttributes();

    // draw the multipass geometry.
    renderPrimitiveSets(state, renderColor, usingVertexBufferObjects);

    // unbind the VBO's if any are used.
    state.unbindVertexBufferObject();
    state.unbindElementBufferObject();
}
