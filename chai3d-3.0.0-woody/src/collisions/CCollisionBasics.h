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
    \version   3.0.0 $Rev: 1289 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CCollisionBasicsH
#define CCollisionBasicsH
//------------------------------------------------------------------------------
#include "math/CVector3d.h"
#include "math/CMatrix3d.h"
#include "graphics/CTriangleArray.h"
#include "materials/CMaterial.h"
#include <vector>

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CCollisionBasics.h
    
    \brief  
    <b> Collision Detection </b> \n
    Collision Events & Recording.

    \details
    cCollisionEvent stores all information related to the
    intersection (or collision) between an segment and an object.
*/
//==============================================================================

//==============================================================================
/*!
    \struct     cCollisionEvent
    \ingroup    collisions

    \brief
    Collision invent.


*/
//==============================================================================
struct cCollisionEvent
{
    //! Pointer to the collided object.
    cGenericObject* m_object;

    //! Pointer to triangle array.
    cTriangleArrayPtr m_triangles;

    //! Index to collided triangle. This pointer may be NULL for collisions with non triangle based objects.
    int m_triangleIndex;

    //! Position of the collision point in reference to the objects coordinate frame (local coordinates).
    cVector3d m_localPos;

    //! Position of the collision point in world coordinates (global coordinates).
    cVector3d m_globalPos;

    //! Surface normal at collision point in reference to the objects coordinate frame (local coordinates).
    cVector3d m_localNormal;

    //! Surface normal at collision point in world coordinates (global coordinates).
    cVector3d m_globalNormal;

    //! Square distance between ray origin and collision point.
    double m_squareDistance;

    //! Projection of collision point onto line going from Vertex0 to Vertex1 of triangle. Value is bounded to [0.0, 1.0].
    double m_trianglePosV01;

    //! Projection of collision point onto line going from Vertex0 to Vertex2 of triangle. Value is bounded to [0.0, 1.0].
    double m_trianglePosV02;

    /*!
    If the position of segment A is modified to take into account motion
    (see m_adjustObjectMotion in cCollisionSettings), the value is stored here.
    */
    cVector3d m_adjustedSegmentAPoint;

    //! Initialize all data
    void clear()
    {
        m_object = NULL;
        m_triangles = nullptr;
        m_triangleIndex = -1;
        m_localPos.zero();
        m_globalPos.zero();
        m_localNormal.zero();
        m_globalNormal.zero();
        m_squareDistance = C_LARGE;
    }
};


//==============================================================================
/*!
    \class      cCollisionRecorder
    \ingroup    collisions

    \brief
    Recorder - Collision events

    \details
    cCollisionRecorder stores a list of collision events.
*/
//==============================================================================
class cCollisionRecorder
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cCollisionRecorder
    cCollisionRecorder() { clear(); }

    //! Destructor of cCollisionRecorder
    virtual ~cCollisionRecorder() {};


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! Clear all records.
    void clear()
    {
        m_nearestCollision.clear();
        m_collisions.clear();
    }


    //--------------------------------------------------------------------------
    // MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! nearest collision from start point of segment.
    cCollisionEvent m_nearestCollision;

    //! List of collisions.
    std::vector<cCollisionEvent> m_collisions;
};


//==============================================================================
/*!
    \struct     cCollisionSettings
    \ingroup    collisions

    \brief
    Settings for collision recorder.

    \details
    This structure contains a list of settings which are passed to the 
    collision detector when checking for a collision.
*/
//==============================================================================
struct cCollisionSettings
{
    cCollisionSettings()
    {
        m_checkForNearestCollisionOnly  = false;
        m_returnMinimalCollisionData    = false;
        m_checkVisibleObjects           = true;
        m_checkHapticObjects            = true;
        m_adjustObjectMotion            = false;
        m_collisionRadius               = 0.0;
    }

    //! If __true__, only return the nearest collision event.
    bool m_checkForNearestCollisionOnly;

    //! If __true__, return minimal amount of data about the collision.
    bool m_returnMinimalCollisionData;

    //! If __true__, then collision detector shall check for collisions on visible objects (m_showEnabled == true).
    bool m_checkVisibleObjects;

    //! If __true__, then collision detector shall check for collisions on haptic enabled objects (m_hapticEnabled == true).
    bool m_checkHapticObjects;

    //! If __true__, then adjust for object motion. (See dynamic proxy model).
    bool m_adjustObjectMotion;

    //! Radius of the virtual tool or cursor.
    double m_collisionRadius;
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

