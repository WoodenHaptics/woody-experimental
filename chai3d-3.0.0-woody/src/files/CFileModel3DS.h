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
    \author    Lev Povalahev
    \author    Dan Morris
    \version   3.0.0 $Rev: 1269 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CFileModel3DSH
#define CFileModel3DSH
//------------------------------------------------------------------------------
#include "math/CMatrix3d.h"
#include "math/CVector3d.h"
#include "graphics/CTriangleArray.h"
#include "materials/CMaterial.h"
#include "materials/CTexture2d.h"
#include "world/CWorld.h"
#include "world/CMultiMesh.h"
#include "lighting/CGenericLight.h"
#include <string>
#include <vector>
#include <stdio.h>
//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CFileModel3DS.h

    \brief    
    <b> Files </b> \n 
    3DS Format 3D Model Handler.
*/
//==============================================================================

//------------------------------------------------------------------------------
// GLOBAL UTILITY FUNCTIONS:
//------------------------------------------------------------------------------ 

//------------------------------------------------------------------------------
/*!
    \ingroup    files
    \brief
    Loads and saves a 3D model of type 3DS providing a filename and multimesh 
    in which the object is loaded.
    */
//------------------------------------------------------------------------------

#ifdef C_USE_FILE_3DS

    //! Load a 3DS model file.
    bool cLoadFile3DS(cMultiMesh* a_object, const std::string& a_filename);

    //! Save a 3DS model file.
    bool cSaveFile3DS(cMultiMesh* a_object, const std::string& a_filename);

#else

    inline bool cLoadFile3DS(cMultiMesh* a_object, const std::string& a_filename) { return (false); }
    inline bool cSaveFile3DS(cMultiMesh* a_object, const std::string& a_filename) { return (false); }

#endif


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
