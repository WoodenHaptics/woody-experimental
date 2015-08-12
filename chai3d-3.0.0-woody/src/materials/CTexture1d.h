//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2014, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.0.0 $Rev: 1264 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CTexture1dH
#define CTexture1dH
//------------------------------------------------------------------------------
#include "graphics/CColor.h"
#include "materials/CGenericTexture.h"
#include "graphics/CImage.h"
//------------------------------------------------------------------------------
#include <string>
#include <stdio.h>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*! 
    \file   CTexture1d.h

    \brief  
    <b> Materials </b> \n 
    1D Texture.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cTexture1d;
typedef std::shared_ptr<cTexture1d> cTexture1dPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cTexture1d
    \ingroup    materials

    \brief      
    1D Texture Map

    \details      
    cTexture1d describes a 1D bitmap texture used for OpenGL texture-mapping.
*/
//==============================================================================
class cTexture1d : public cGenericTexture
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cTexture1d.
    cTexture1d();

    //! Destructor of cTexture1d.
    virtual ~cTexture1d();

    //! Shared cTexture1d allocator.
    static cTexture1dPtr create() { return (std::make_shared<cTexture1d>()); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! Set texture unit where a_textureUnit is GL_TEXTUREi_ARB, where 0i<GL_MAX_TEXTURE_UNITS_ARB.
    inline void setTextureUnit(const GLenum a_textureUnit) { m_textureUnit = a_textureUnit; }

    //! Get texture unit where a_textureUnit is GL_TEXTUREi_ARB, where 0i<GL_MAX_TEXTURE_UNITS_ARB.
    inline GLenum getTextureUnit() const { return (m_textureUnit); }

    //! Create a copy of current object.
    cTexture1dPtr copy();

    //! Load a texture image file.
    virtual bool loadFromFile(const std::string& a_fileName);

    //! Save a texture image file.
    virtual bool saveToFile(const std::string& a_fileName);

    //! Enable texturing and set this texture as the current texture.
    virtual void render(cRenderOptions& a_options);

    //! Call this to force texture re-initialization.
    virtual void markForUpdate();

    //! Set the environment mode (GL_MODULATE, GL_DECAL, GL_BLEND, GL_REPLACE, or -1 for "don't set").
    void setEnvironmentMode(const GLint& a_environmentMode) { m_environmentMode = a_environmentMode; }

    //! Get the environment mode status.
    GLint getEnvironmentMode() { return (m_environmentMode); }

    //! Set the texture wrap mode.
    virtual void setWrapMode(const GLint& a_wrapMode);

    //! Get the texture wrap mode of S.
    GLint getWrapSmode() const { return (m_wrapSmode); }

    //! Set the magnification function.
    void setMagFunction(const GLint a_magFunction);

    //! Get current magnification function.
    GLint getMagFunction() const { return (m_magFunction); }

    //! Set the minification function.
    void setMinFunction(const GLint a_minFunction);

    //! Get current minification function.
    GLint getMinFunction() const { return (m_minFunction); }

    //! Set spherical mapping mode \b ON or \b OFF.
    void setSphericalMappingEnabled(const bool a_enabled) { m_useSphericalMapping = a_enabled; }

    //! Get the status of the spherical mapping mode.
    bool getSphericalMappingEnabled() const { return (m_useSphericalMapping); }

    //! Enable or disable Mipmaps.
    void setUseMipmaps(bool a_useMipmaps);

    //! Get the status of Mipmaps mode.
    bool getUseMipmaps() { return (m_useMipmaps); }

    // ! Set an existing image to use as texture
    bool setImage (cImagePtr a_image);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! Image loader (use this to get data about the texture itself).
    cImagePtr m_image;

    //! Environmental color.
    cColorf m_color;


    //--------------------------------------------------------------------------
    // PRIVATE METHODS:
    //--------------------------------------------------------------------------

private:

    //! Reset internal variables. This function should be called only by constructors.
    void reset();

    //! Initialize GL texture.
    void update(cRenderOptions& a_options);


    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! If __true__, texture bitmap has not yet been sent to video card.
    bool m_updateTextureFlag[C_MAX_DISPLAY_CONTEXTS];

    //! OpenGL texture ID number.
    GLuint m_textureID[C_MAX_DISPLAY_CONTEXTS];

    //! Texture wrap parameter along S (\e GL_REPEAT or \e GL_CLAMP).
    GLint m_wrapSmode;

    //! Texture magnification function. (\e GL_NEAREST or \e GL_LINEAR).
    GLint m_magFunction;

    //! Texture minifying function. (\e GL_NEAREST or \e GL_LINEAR).
    GLint m_minFunction;

    //! If __true__, we use GLU to build mipmaps.
    bool m_useMipmaps;

    //! If __true__, we use spherical mapping.
    bool m_useSphericalMapping;

    //! OpenGL texture mode (\e GL_MODULATE, \e GL_DECAL, \e GL_BLEND, \e GL_REPLACE).
    GLint m_environmentMode;

    //! Texture unit number
    GLenum m_textureUnit;

    //! Texture magnification function when Mipmap is OFF.
    GLint m_magFunctionMipmapsOFF;

    //! Texture mignifying function when Mipmap is OFF.
    GLint m_minFunctionMipmapsOFF;

    //! Texture magnification function when Mipmap is ON.
    GLint m_magFunctionMipmapsON;

    //! Texture mignifying function when Mipmap is ON.
    GLint m_minFunctionMipmapsON;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

