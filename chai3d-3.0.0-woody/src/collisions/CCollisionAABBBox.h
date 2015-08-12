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
    \version   3.0.0 $Rev: 1289 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CCollisionAABBBoxH
#define CCollisionAABBBoxH
//------------------------------------------------------------------------------
#include "math/CMaths.h"
#include "graphics/CTriangleArray.h"
#include "graphics/CDraw3D.h"
#include <vector>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CCollisionAABBBox.h

    \brief    
    <b> Collision Detection </b> \n
    Axis-Aligned Bounding Box Tree (AABB) - Implementation.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cCollisionAABBBox
    \ingroup    collisions
    
    \brief    
    cCollisionAABBox contains the properties and methods of an axis-aligned 
    bounding box. These methods are used by the AABB collision detection 
    algorithm.
*/
//==============================================================================
struct cCollisionAABBBox
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Default constructor of cCollisionAABBBox.
    cCollisionAABBBox() {};

    //! Constructor of cCollisionAABBBox.
    cCollisionAABBBox(const cVector3d& a_min, 
                      const cVector3d& a_max) 
    { 
        setValue(a_min, a_max); 
    }

    //! Destructor of cCollisionAABBBox.
    virtual ~cCollisionAABBBox() {};


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

public:

    //! Return the center of the bounding box.
    inline cVector3d getCenter() const { return (m_center); }

    //! Return the extent (half the width) of the bounding box.
    inline cVector3d getExtent() const { return (m_extent); }

    //! Set the center of the bounding box.
    inline void setCenter(const cVector3d& a_center)  { m_center = a_center; }

    //! Set the extent (half the width) of the bounding box.
    inline void setExtent(const cVector3d& a_extent) { m_extent = a_extent; }

    //! Set the center and extent of the box based on two points.
    inline void setValue(const cVector3d& a_min, const cVector3d& a_max)
    {
        m_extent = cMul(0.5, cSub(a_max, a_min));
        m_center = cAdd(a_min, m_extent);
        m_min = a_min;
        m_max = a_max;
    }


    //--------------------------------------------------------------------------
    /*!
        Determine whether the current and input box passed as argument intersect
        each other.

        \param    a_box  Input box.

        \return   Return whether there is any overlap of current and 
        input box.
    */
    //--------------------------------------------------------------------------
    inline bool intersect(const cCollisionAABBBox& a_box)
    {
        // check for overlap along each axis
        if (a_box.getLowerX() > getUpperX()) return (false);
        if (a_box.getLowerY() > getUpperY()) return (false);
        if (a_box.getLowerZ() > getUpperZ()) return (false);
        if (a_box.getLowerX() > getUpperX()) return (false);
        if (a_box.getLowerY() > getUpperY()) return (false);
        if (a_box.getLowerZ() > getUpperZ()) return (false);

        // if the boxes are not separated along any axis, a collision has occurred
        return (true);
    }


    //--------------------------------------------------------------------------
    /*!
        Determine whether the given ray intersects the bounding box. Based on code
        by Andrew Woo from "Graphics Gems", Academic Press, 1990.

        \param  a_segmentPointA  Initial point of segment.
        \param  a_segmentPointB  End point of segment.

        \return Return __true__ if line segment intersects the bounding box,
                __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool intersect(cVector3d& a_segmentPointA, cVector3d& a_segmentPointB)
    {
        const int RIGHT     = 0;
        const int LEFT      = 1;
        const int MIDDLE    = 2;

        double coord[3];
        char inside = true;
        char quadrant[3];
        register int i;
        int whichPlane;
        double maxT[3];
        double candidatePlane[3];
        double dir[3];
        dir[0] = a_segmentPointB(0) - a_segmentPointA(0);
        dir[1] = a_segmentPointB(1) - a_segmentPointA(1);
        dir[2] = a_segmentPointB(2) - a_segmentPointA(2);

        // Find candidate planes; this loop can be avoided if rays cast all from 
        // the eye (assume perspective view)
        for (i=0; i<3; i++)
        {
            if(a_segmentPointA(i) < m_min(i))
            {
                quadrant[i] = LEFT;
                candidatePlane[i] = m_min(i);
                inside = false;
            }
            else if (a_segmentPointA(i) > m_max(i))
            {
                quadrant[i] = RIGHT;
                candidatePlane[i] = m_max(i);
                inside = false;
            }
            else
            {
               quadrant[i] = MIDDLE;
            }
        }

        // ray origin inside bounding box
        if (inside)
        {
            return (true);
        }

        // calculate T distances to candidate planes
        for (i = 0; i < 3; i++)
        {
            if (quadrant[i] != MIDDLE && dir[i] !=0.)
                maxT[i] = (candidatePlane[i]-a_segmentPointA(i)) / dir[i];
            else
                maxT[i] = -1.;
        }

        // get largest of the maxT's for final choice of intersection
        whichPlane = 0;
        for (i = 1; i < 3; i++)
            if (maxT[whichPlane] < maxT[i])
                whichPlane = i;

        // check final candidate actually inside box
        if (maxT[whichPlane] < 0.) return (false);
        for (i = 0; i < 3; i++)
        {
            if (whichPlane != i)
            {
                coord[i] = a_segmentPointA(i) + maxT[whichPlane] * dir[i];
                if (coord[i] < m_min(i) || coord[i] > m_max(i))
                    return (false);
            }
            else
            {
                coord[i] = candidatePlane[i];
            }
        }

        // ray hits box
        return (true);
    }


    //--------------------------------------------------------------------------
    /*!
        Test whether this box contains the given point.

        \param  a_point  Point.
        \return Returns whether this box contains this point
    */
    //--------------------------------------------------------------------------
    inline bool contains(const cVector3d& a_point) const
    {
        // check that each of the point's coordinates are within the box's range
        if ((a_point(0) > m_min(0)) && (a_point(1) > m_min(1)) && (a_point(2) > m_min(2)) &&
            (a_point(0) < m_max(0)) && (a_point(1) < m_max(1)) && (a_point(2) < m_max(2)))
        {
            return (true);
        }
        else
        {
            return (false);
        }
    }


    //--------------------------------------------------------------------------
    /*!
        Set the bounding box to bound the two given bounding boxes.

        \param  a_boxA  The first bounding box to be enclosed.
        \param  a_boxB  The other bounding box to be enclosed.
    */
    //--------------------------------------------------------------------------
    inline void enclose(const cCollisionAABBBox& a_boxA, 
                        const cCollisionAABBBox& a_boxB)
    {
        // find the minimum coordinate along each axis
        cVector3d lower(cMin(a_boxA.getLowerX(), a_boxB.getLowerX()),
                        cMin(a_boxA.getLowerY(), a_boxB.getLowerY()),
                        cMin(a_boxA.getLowerZ(), a_boxB.getLowerZ()));

        // find the maximum coordinate along each axis
        cVector3d upper(cMax(a_boxA.getUpperX(), a_boxB.getUpperX()),
                        cMax(a_boxA.getUpperY(), a_boxB.getUpperY()),
                        cMax(a_boxA.getUpperZ(), a_boxB.getUpperZ()));

        // set the center and extent of this box to enclose the two extreme points
        setValue(lower, upper);
    }


    //--------------------------------------------------------------------------
    /*!
        Modify the bounding box as needed to bound the given point.

        \param    a_point  The point to be bounded.
    */
    //--------------------------------------------------------------------------
    inline void enclose (const cVector3d& a_point)
    {
        // decrease coordinates as needed to include given point
        cVector3d lower(cMin(getLowerX(), a_point(0) ),
                        cMin(getLowerY(), a_point(1) ),
                        cMin(getLowerZ(), a_point(2)));

        // increase coordinates as needed to include given point
        cVector3d upper(cMax(getUpperX(), a_point(0) ),
                        cMax(getUpperY(), a_point(1) ),
                        cMax(getUpperZ(), a_point(2)));

        // set the center and extent of this box to enclose the given point
        setValue(lower, upper);
    }


    //! Modify the bounding box to bound another box
    inline void enclose(const cCollisionAABBBox& a_box) { enclose(*this, a_box); }

    //! Initialize a bounding box to center at origin and infinite extent.
    inline void setEmpty()
    {
        const double C_INFINITY = 1.0e50;
        m_center.zero();
        m_extent = cVector3d(-C_INFINITY, -C_INFINITY, -C_INFINITY);
        m_min.set(C_INFINITY, C_INFINITY, C_INFINITY);
        m_max.set(-C_INFINITY, -C_INFINITY, -C_INFINITY);
    }

    //! Return the smallest coordinate along X axis.
    inline double getLowerX() const  { return (m_min(0) ); }

    //! Return the largest coordinate along X axis.
    inline double getUpperX() const  { return (m_max(0) ); }

    //! Return the smallest coordinate along Y axis.
    inline double getLowerY() const  { return (m_min(1) ); }

    //! Return the largest coordinate along Y axis.
    inline double getUpperY() const  { return (m_max(1) ); }

    //! Return the smallest coordinate along Z axis.
    inline double getLowerZ() const  { return (m_min(2)); }

    //! Return the largest coordinate along Z axis.
    inline double getUpperZ() const  { return (m_max(2)); }

    //! Return the length of the longest axis of the bounding box.
    double size() const;


    //--------------------------------------------------------------------------
    /*!
        Return the index of the longest axis of the bounding box.

        \return  Return the index of the longest axis of the box.
    */
    //--------------------------------------------------------------------------
    inline int getLongestAxis() const
    {
        // if extent of x axis is greatest, return index 0
        if ((m_extent(0) >= m_extent(1)) && (m_extent(0) >= m_extent(2))) 
        {
            return (0);
        }
        else if ((m_extent(1) >= m_extent(0)) && (m_extent(1) >= m_extent(2))) 
        {
            return (1);
        }
        else
        {
            return (2);
        }
    }

    //! Draw the edges of the bounding box.
    inline void render()
    {
        cDrawWireBox(m_min(0), m_max(0), m_min(1), m_max(1), m_min(2), m_max(2));
    }


    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! The center of the bounding box.
    cVector3d m_center;

    //! The extent (half the width) of the bounding box.
    cVector3d m_extent;

    //! The minimum point (along each axis) of the bounding box.
    cVector3d m_min;

    //! The maximum point (along each axis) of the bounding box.
    cVector3d m_max;
};


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

