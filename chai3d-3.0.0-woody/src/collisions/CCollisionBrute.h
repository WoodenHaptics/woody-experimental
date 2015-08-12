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
    \author    Chris Sewell
    \author    Francois Conti
    \version   3.0.0 $Rev: 1269 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CCollisionBruteH
#define CCollisionBruteH
//------------------------------------------------------------------------------
#include "math/CMaths.h"
#include "graphics/CTriangleArray.h"
#include "graphics/CVertexArray.h"
#include "collisions/CGenericCollision.h"
//------------------------------------------------------------------------------
#include <vector>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CCollisionBrute.h

    \brief
    <b> Collision Detection </b> \n
    Brute Force Model.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cCollisionBrute
    \ingroup    collisions

    \brief
    Collision detector - Brute force

    \details
    cCollisionBrute provides methods to check for the intersection
    of a line segment with a mesh by checking all triangles in the mesh.
*/
//==============================================================================
class cCollisionBrute : public cGenericCollision
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cCollisionBrute.
    cCollisionBrute(cTriangleArrayPtr a_triangles);

    //! Destructor of cCollisionBrute.
    virtual ~cCollisionBrute() {}


    //-----------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

public:

    //! No initialization is necessary for the brute force method.
    virtual void initialize(const double a_radius = 0) {};

    //! There isn't really a useful "visualization" of "check all triangles".
    virtual void render() {};

    //! Return the triangles intersected by the given segment, if any.
    bool computeCollision(cGenericObject* a_object,
                          cVector3d& a_segmentPointA,
                          cVector3d& a_segmentPointB,
                          cCollisionRecorder& a_recorder,
                          cCollisionSettings& a_settings);


    //-----------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------

protected:

    //! Pointer to the list of triangles in the mesh.
    cTriangleArrayPtr m_triangles;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

