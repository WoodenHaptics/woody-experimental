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
    \author    Dan Morris
    \version   3.0.0 $Rev: 1304 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CGenericObjectH
#define CGenericObjectH
//------------------------------------------------------------------------------
#include "collisions/CCollisionBasics.h"
#include "effects/CGenericEffect.h"
#include "forces/CInteractionBasics.h"
#include "graphics/CDraw3D.h"
#include "graphics/CColor.h"
#include "graphics/CDisplayList.h"
#include "graphics/CRenderOptions.h"
#include "materials/CMaterial.h"
#include "materials/CNormalMap.h"
#include "math/CMaths.h"
#include "math/CTransform.h"
#include "system/CGenericType.h"
//------------------------------------------------------------------------------
#include <vector>
#include <list>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
class cGenericCollision;
class cGenericForceAlgorithm;
class cMultiMesh;
class cShaderProgram;
class cInteractionRecorder;
//------------------------------------------------------------------------------
typedef std::shared_ptr<cShaderProgram> cShaderProgramPtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CGenericObject.h

    \brief 
    <b> Scenegraph </b> \n 
    Base Class.
*/
//==============================================================================

//==============================================================================
/*!
    \class      cGenericObject
    \ingroup    scenegraph

    \brief
    Abstract class for virtual 3D objects.

    \details
    This class is the root of basically every render-able object in CHAI3D.  
    It defines a reference frame (position and rotation) and virtual methods 
    for rendering, which are overloaded by useful subclasses. \n
 
    This class also defines basic methods for maintaining a scene graph, 
    and propagating rendering passes and reference frame changes through 
    a hierarchy of cGenericObjects. \n

    Besides subclassing, a useful way to extend cGenericObject is to store 
    custom data in the m_tag and m_userData member fields, which are not 
    used by CHAI3D. \n

    The most important methods to look at here are probably the virtual 
    methods, which are listed last in CGenericObject.h. These methods 
    will be called on each cGenericObject as operations propagate through 
    the scene graph.
*/
//==============================================================================
class cGenericObject : public cGenericType
{
    friend class cMultiMesh;

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cGenericObject.
    cGenericObject();

    //! Destructor of cGenericObject.
    virtual ~cGenericObject();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL
    //--------------------------------------------------------------------------

public:

    //! Enable or Disable an object. When an object is disabled, both haptic and graphic rendering no longer occur.
    virtual void setEnabled(bool a_enabled,  const bool a_affectChildren = false);

    //! Read status of object.
    bool getEnabled() const { return (m_enabled); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - COPY:
    //-----------------------------------------------------------------------

public:

    //! Create a copy of itself.
    virtual cGenericObject* copy(const bool a_duplicateMaterialData = false,
        const bool a_duplicateTextureData = false, 
        const bool a_duplicateMeshData = false,
        const bool a_buildCollisionDetector = true) { return (NULL); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - TRANSLATION AND ORIENTATION:
    //-----------------------------------------------------------------------

public:

    //! Set the local position of this object.
    virtual void setLocalPos(const cVector3d& a_localPos)
    {
        m_localPos = a_localPos;
    }

#ifdef C_USE_EIGEN
    //! Set the local position of this object.
    void setLocalPos(const Eigen::Vector3d& a_localPos)
    {
        setLocalPos(cVector3d(a_localPos[0], a_localPos[1], a_localPos[2]));
    }
#endif

    //! Set the local position of this object.
    void setLocalPos(const double a_x = 0.0, 
        const double a_y = 0.0, 
        const double a_z = 0.0)
    {
        setLocalPos(cVector3d(a_x, a_y, a_z));
    }

    //! Get the local position of this object.
    inline cVector3d getLocalPos() const { return (m_localPos); }

    //! Get the global position of this object.
    inline cVector3d getGlobalPos() const { return (m_globalPos); }

    //! Set the local rotation matrix for this object.
    virtual void setLocalRot(const cMatrix3d& a_localRot)
    {
        m_localRot = a_localRot;
    }

#ifdef C_USE_EIGEN
    //! Set the local rotation matrix for this object.
    inline void setLocalRot(const Eigen::Matrix3d a_localRot)
    {
        cMatrix3d localRot;
        localRot.copyfrom(a_localRot);
        setLocalRot(localRot);
    }
#endif

    //! Get the local rotation matrix of this object.
    inline cMatrix3d getLocalRot() const { return (m_localRot); }

    //! Get the global rotation matrix of this object.
    inline cMatrix3d getGlobalRot() const { return (m_globalRot); }

    //! Set the local position and rotation matrix by passing a transformation matrix.
    inline void setLocalTransform(const cTransform& a_transform) 
    {
        setLocalPos(a_transform.getLocalPos());
        setLocalRot(a_transform.getLocalRot());
    }

    //! Get the local position and rotation matrix in a transformation matrix.
    inline cTransform getLocalTransform() { return (cTransform(m_localPos, m_localRot)); }

    //! Get the global position and rotation matrix in a transformation matrix.
    inline cTransform getGlobalTransform() { return (cTransform(m_globalPos, m_globalRot)); }

    //! Translate object by a specified offset.
    void translate(const cVector3d& a_translation);

    //! Translate object by a specified offset.
    void translate(const double a_x, 
        const double a_y, 
        const double a_z = 0.0);

    //! Rotate object around a local axis. Angle magnitude is defined in radians.
    void rotateAboutLocalAxisRad(const cVector3d& a_axis, const double a_angleRad);

    //! Rotate object around a local axis. Angle magnitude is defined in degrees.
    inline void rotateAboutLocalAxisDeg(const cVector3d& a_axis,
        const double a_angleDeg) 
    { 
        rotateAboutLocalAxisRad(a_axis, cDegToRad(a_angleDeg)); 
    }

    //! Rotate object around a local axis. Angle magnitude is defined in radians.
    inline void rotateAboutLocalAxisRad(const double a_axisX,
        const double a_axisY, 
        const double a_axisZ, 
        const double a_angleRad) 
    { 
        rotateAboutLocalAxisRad(cVector3d(a_axisX, a_axisY, a_axisZ), a_angleRad);
    }

    //! Rotate object around a local axis. Angle magnitude is defined in degrees.
    inline void rotateAboutLocalAxisDeg(const double a_axisX,
        const double a_axisY, 
        const double a_axisZ, 
        const double a_angleDeg) 
    { 
        rotateAboutLocalAxisRad(cVector3d(a_axisX, a_axisY, a_axisZ), cDegToRad(a_angleDeg));
    }

    //! Rotate object around a global axis. Angle magnitude is defined in radians.
    void rotateAboutGlobalAxisRad(const cVector3d& a_axis, const double a_angleRad);

    //! Rotate object around a global axis. Angle magnitude is defined in degrees.
    inline void rotateAboutGlobalAxisDeg(const cVector3d& a_axis, const double a_angleDeg) 
    { 
        rotateAboutGlobalAxisRad(a_axis, cDegToRad(a_angleDeg)); 
    }

    //! Rotate object around a local axis. Angle magnitude is defined in radians.
    inline void rotateAboutGlobalAxisRad(const double a_axisX,
        const double a_axisY, 
        const double a_axisZ, 
        const double a_angleRad) 
    { 
        rotateAboutGlobalAxisRad(cVector3d(a_axisX, a_axisY, a_axisZ), a_angleRad);
    }

    //! Rotate object around a local axis. Angle magnitude is defined in degrees.
    inline void rotateAboutGlobalAxisDeg(const double a_axisX, 
        const double a_axisY, 
        const double a_axisZ, 
        const double a_angleDeg) 
    { 
        rotateAboutGlobalAxisRad(cVector3d(a_axisX, a_axisY, a_axisZ), cDegToRad(a_angleDeg)); 
    }

    //! Rotate object using fixed Euler representation. Angles are defined in radians.
    void rotateExtrinsicEulerAnglesRad(const double& a_angleRad1,
        const double& a_angleRad2,
        const double& a_angleRad3,
        const cEulerOrder a_eulerOrder);

    //! Rotate object using fixed Euler representation. Angles are defined in radians.
    void rotateExtrinsicEulerAnglesDeg(const double& a_angleDeg1,
        const double& a_angleDeg2,
        const double& a_angleDeg3,
        const cEulerOrder a_eulerOrder) 
    { 
        rotateExtrinsicEulerAnglesRad(cDegToRad(a_angleDeg1), cDegToRad(a_angleDeg2), cDegToRad(a_angleDeg3), a_eulerOrder); 
    }

    //! Rotate object using co-moving Euler representation. Angles are defined in radians.
    void rotateIntrinsicEulerAnglesRad(const double& a_angleRad1,
        const double& a_angleRad2,
        const double& a_angleRad3,
        const cEulerOrder a_eulerOrder);

    //! Rotate object using co-moving Euler representation. Angles are defined in radians.
    void rotateIntrinsicEulerAnglesDeg(const double& a_angleDeg1,
        const double& a_angleDeg2,
        const double& a_angleDeg3,
        const cEulerOrder a_eulerOrder) 
    { 
        rotateIntrinsicEulerAnglesRad(cDegToRad(a_angleDeg1), cDegToRad(a_angleDeg2), cDegToRad(a_angleDeg3), a_eulerOrder); 
    }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - COMPUTING GLOBAL POSITIONS:
    //-----------------------------------------------------------------------

public:

    //! Compute the global position and rotation of this object and its children.
    virtual void computeGlobalPositions(const bool a_frameOnly = true,
        const cVector3d& a_globalPos = cVector3d(0.0, 0.0, 0.0),
        const cMatrix3d& a_globalRot = cIdentity3d());

    //! Compute the global position and rotation of current object only.
    void computeGlobalPositionsFromRoot(const bool a_frameOnly = true);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - HAPTIC EFFECTS:
    //-----------------------------------------------------------------------

public:

    //! Add a haptic effect to this object.
    bool addEffect(cGenericEffect* a_effect);

    //! Remove a haptic effect from this object.
    bool removeEffect(cGenericEffect* a_effect);
    
    //! Remove all haptic effects
    void deleteAllEffects();

    //! Create a magnetic haptic effect.
    bool createEffectMagnetic();

    //! Delete current magnetic haptic effect.
    bool deleteEffectMagnetic();

    //! Create a stick-and-slip haptic effect
    bool createEffectStickSlip();

    //! Delete current stick-and-slip haptic effect
    bool deleteEffectStickSlip();

    //! Create a surface haptic effect
    bool createEffectSurface();

    //! Delete current surface haptic effect
    bool deleteEffectSurface();

    //! Create a vibration haptic effect
    bool createEffectVibration();

    //! Delete current vibration haptic effect
    bool deleteEffectVibration();

    //! Create a viscous haptic effect
    bool createEffectViscosity();

    //! Delete current viscous haptic effect
    bool deleteEffectViscosity();

    
    //-----------------------------------------------------------------------
    // PUBLIC METHODS - HAPTIC PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! Allow this object to be felt (when visible), optionally propagating the change to children.
    virtual void setHapticEnabled(const bool a_hapticEnabled, const bool a_affectChildren = true);

    //! Read the haptic status of object (true means it can be felt when visible).
    inline bool getHapticEnabled() const { return (m_hapticEnabled); }

    //! Set the haptic stiffness, possibly recursively affecting children.
    virtual void setStiffness(const double a_stiffness, const bool a_affectChildren = true);

    //! Set the static and dynamic friction for this mesh, possibly recursively affecting children.
    virtual void setFriction(double a_staticFriction, 
        double a_dynamicFriction, 
        const bool a_affectChildren = true);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - GRAPHIC PROPERTIES:
    //-----------------------------------------------------------------------

public:

    //! Show or hide this object, optionally propagating the change to children.
    virtual void setShowEnabled(const bool a_show, const bool a_affectChildren = true);

    //! Read the display status of object (true means it's visible).
    inline bool getShowEnabled() const { return (m_showEnabled); }

    //! Enable or disable transparency.
    virtual void setUseTransparency(const bool a_useTransparency, const bool a_affectChildren = false);

    //! Is transparency enabled for this mesh?
    inline bool getUseTransparency() const { return m_useTransparency; }

    //! Set the alpha value at each vertex and in all of my material colors.
    virtual void setTransparencyLevel(const float a_level,
        const bool a_applyToTextures = false,
        const bool a_affectChildren = false);

    //! Enable or disable wireframe rendering, optionally propagating the operation to my children.
    virtual void setWireMode(const bool a_showWireMode, const bool a_affectChildren = false);

    //! Return whether wireframe rendering is enabled.
    inline bool getWireMode() const { return (m_triangleMode == GL_LINE); }

    //! Enable or disabling face-culling, optionally propagating the operation to my children.
    virtual void setUseCulling(const bool a_useCulling, const bool a_affectChildren = false);

    //! Is face-culling currently enabled?
    inline bool getUseCulling() const { return (m_cullingEnabled); }

    //! Enable or disable the use of per-vertex colors, optionally propagating the operation to my children.
    virtual void setUseVertexColors(const bool a_useColors, const bool a_affectChildren = false);

    //! Are per-vertex properties currently enabled?
    inline bool getUseVertexColors() const { return (m_useVertexColors); }

    //! Backup material color properties of object, optionally propagating the operation to my children.
    virtual void backupMaterialColors(const bool a_material_properties_only = true, const bool a_affectChildren = false);

    //! Restore material color properties of object, optionally propagating the operation to my children.
    virtual void restoreMaterialColors(const bool a_material_properties_only = true, const bool a_affectChildren = false);


    //! Enable or disable the use of a display list for rendering, optionally propagating the operation to my children.
    virtual void setUseDisplayList(const bool a_useDisplayList, const bool a_affectChildren = false);

    //! Ask whether I'm currently rendering with a display list.
    inline bool getUseDisplayList() const { return (m_useDisplayList); }


    //! Invalidate any existing display lists, optionally propagating the operation to my children.
    virtual void markForUpdate(const bool a_affectChildren = false);


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - MATERIAL PROPERTY:
    //-----------------------------------------------------------------------

public:

    //! Enable or disable the use of material properties, optionally propagating the operation to my children.
    virtual void setUseMaterial(const bool a_useMaterial, const bool a_affectChildren = false);

    //! Are material properties currently enabled?
    inline bool getUseMaterial() const { return (m_useMaterialProperty); }

    //! Set material to this object, possibly recursively affecting children.
    virtual void setMaterial(cMaterialPtr a_material, const bool a_affectChildren = false);

    //! Set material to this object, possibly recursively affecting children.
    virtual void setMaterial(cMaterial& a_material, const bool a_affectChildren = false);

    //! Set shader program.
    virtual void setShaderProgram(cShaderProgramPtr a_shaderProgram, const bool a_affectChildren = false);

    //! Get shader program.
    virtual cShaderProgramPtr getShaderProgram() { return (m_shaderProgram); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS - TEXTURE PROPERTY:
    //-----------------------------------------------------------------------

public:

    //! Enable or disable the use of texture-mapping, optionally propagating the operation to my children.
    virtual void setUseTexture(const bool a_useTexture, const bool a_affectChildren = false);

    //! Is texture-mapping enabled?
    inline bool getUseTexture() const { return (m_useTextureMapping); }

    //! Set texture to this object, possibly recursively affecting children.
    virtual void setTexture(cTexture1dPtr a_texture, const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - BOUNDARY BOX:
    //-----------------------------------------------------------------------

public:

    //! Color of the boundary box.
    static cColorf s_boundaryBoxColor;

    //! Show or hide the boundary box for this object, optionally propagating the change to children.
    virtual void setShowBoundaryBox(const bool a_showBoundaryBox, const bool a_affectChildren = false);

    //! Read the display status of boundary box. (true means it's visible).
    inline bool getShowBoundaryBox() const { return (m_showBoundaryBox); }

    //! Read the minimum point of this object's boundary box.
    inline cVector3d getBoundaryMin() const { return (m_boundaryBoxMin); }

    //! Read the maximum point of this object's boundary box.
    inline cVector3d getBoundaryMax() const { return (m_boundaryBoxMax); }

    //! Compute the center of this object's boundary box.
    inline cVector3d getBoundaryCenter() const { return (m_boundaryBoxMax + m_boundaryBoxMin)/2.0; }

    //! Return __true__, if bounding box is empty. Otherwise return __false__.
    inline bool getBoundaryBoxEmpty() { return (m_boundaryBoxEmpty); }

    //! Re-compute this object's bounding box, optionally forcing it to bound child objects.
    virtual void computeBoundaryBox(const bool a_includeChildren = true);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - REFERENCE FRAME REPRESENTATION:
    //-----------------------------------------------------------------------

public:

    //! Show or hide the reference frame arrows for this object, optionally propagating the change to children.
    virtual void setShowFrame(const bool a_showFrame, const bool a_affectChildren  = false);

    //! Read the display status of the reference frame (true means it's visible).
    inline bool getShowFrame(void) const { return (m_showFrame); }

    //! Set the size of the rendered reference frame, optionally propagating the change to children.
    virtual void setFrameSize(const double a_size = 1.0, const bool a_affectChildren = false);

    //! Read the size of the rendered reference frame.
    inline double getFrameSize() const { return (m_frameSize); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - COLLISION DETECTION:
    //-----------------------------------------------------------------------

public:

    //! Set a collision detector for current object.
    void setCollisionDetector(cGenericCollision* a_collisionDetector) { m_collisionDetector = a_collisionDetector; }

    //! Get pointer to this object's current collision detector.
    inline cGenericCollision* getCollisionDetector() const { return (m_collisionDetector); }

    //! Delete any existing collision detector and set the current cd to null (no collisions).
    virtual void deleteCollisionDetector(const bool a_affectChildren = false);

    //! Compute collision detection using collision detectors.
    virtual bool computeCollisionDetection(cVector3d& a_segmentPointA,
        cVector3d& a_segmentPointB,
        cCollisionRecorder& a_recorder,
        cCollisionSettings& a_settings);

    //! Show or hide the collision tree for this object, optionally propagating the change to children.
    virtual void setShowCollisionDetector(const bool a_showCollisionDetector, const bool a_affectChildren = false);

    //! Read the display status of of the collision tree for this object.
    inline bool getShowCollisionDetector() { return (m_showCollisionDetector); }

    //! Set collision rendering properties.
    virtual void setCollisionDetectorProperties(unsigned int a_displayDepth, 
        cColorf& a_color, 
        const bool a_affectChildren = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - SCENE GRAPH:
    //-----------------------------------------------------------------------

public:

    //! Set parent of current object.
    inline void setParent(cGenericObject* a_parent) { m_parent = a_parent; }

    //! Read parent of current object.
    inline cGenericObject* getParent() const { return (m_parent); }

    //! Set a link to an object which may own current object. This can be  parent object for instance.
    inline void setOwner(cGenericObject* a_owner) { m_owner = a_owner; }

    //! Get owner of current object.
    inline cGenericObject* getOwner() { return (m_owner); }

    //! Read an object from my list of children.
    inline cGenericObject* getChild(const unsigned int a_index) const { return (m_children[a_index]); }

    //! Add an object to my list of children.
    bool addChild(cGenericObject* a_object);

    //! Remove an object from my list of children, without deleting it.
    bool removeChild(cGenericObject* a_object);

    //! Remove object from parent's list of children.
    bool removeFromGraph();

    //! Does this object have the specified object as a child?
    bool containsChild(cGenericObject* a_object, bool a_includeChildren = false);

    //! Remove an object from my list of children and delete it.
    bool deleteChild(cGenericObject *a_object);

    //! Clear all objects from my list of children, without deleting them.
    void clearAllChildren();

    //! Clear and delete all objects from my list of children.
    void deleteAllChildren();

    //! Return the number of children on my list of children.
    inline unsigned int getNumChildren() { return ((unsigned int)m_children.size()); }

    //! Return my total number of descendants, optionally including this object.
    inline unsigned int getNumDescendants(bool a_includeCurrentObject = false);

    //! Fill this list with all of my descendants.
    void enumerateChildren(::std::list<cGenericObject*>& a_childList, bool a_includeCurrentObject = false);


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - GHOSTING:
    //-----------------------------------------------------------------------

public:

    //! Makes this object a ghost node.
    void setGhostEnabled(bool a_ghostEnabled) { m_ghostEnabled = a_ghostEnabled; }

    //! Read the ghost status of this object.
    bool getGhostEnabled() { return (m_ghostEnabled); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS - GEOMETRY:
    //-----------------------------------------------------------------------

public:

    //! Scale this object by a_scaleFactor (uniform scale).
    virtual void scale(const double& a_scaleFactor, const bool a_affectChildren = true);


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - PROPERTIES:
    //-----------------------------------------------------------------------

public: 

    //! Name of current object (filename).
    std::string m_name;

    //! Material property.
    cMaterialPtr m_material;

    //! Texture property.
    cTexture1dPtr m_texture;

    //! Normal map property.
    cNormalMapPtr m_normalMap;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - CUSTOM USER DATA:
    //-----------------------------------------------------------------------

public:

    //! An arbitrary tag, not used by CHAI3D.
    int m_userTag;

    //! An arbitrary data pointer, not used by CHAI3D.
    void* m_userData;

    //! Name of current object, not used by CHAI3D.
    std::string m_userName;

    //! A link to an external cGenericObject object, not used by CHAI3D.
    cGenericObject* m_userExternalObject;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - SCENEGRAPH:
    //-----------------------------------------------------------------------

protected:

    //! Parent object.
    cGenericObject* m_parent;

    /*!
        For most objects this value is initialized to point to the object itself.
        In the case of cMultiMesh, all mesh objects contained in cMultimesh are
        owned by their parent (cMultiMesh). 
    */
    cGenericObject* m_owner;

    //! List of children.
    std::vector<cGenericObject*> m_children;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - POSITION & ORIENTATION:
    //-----------------------------------------------------------------------

protected:

    //! The position of this object in my parent's reference frame.
    cVector3d m_localPos;

    //! The position of this object in the world's reference frame.
    cVector3d m_globalPos;

    //! The rotation matrix that rotates my reference frame into my parent's reference frame.
    cMatrix3d m_localRot;

    //! The rotation matrix that rotates my reference frame into the world's reference frame.
    cMatrix3d m_globalRot;

    //! Previous position since last haptic computation.
    cVector3d m_prevGlobalPos;

    //! Previous rotation since last haptic computation.
    cMatrix3d m_prevGlobalRot;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - BOUNDARY BOX
    //-----------------------------------------------------------------------

protected:

    //! Minimum position of boundary box.
    cVector3d m_boundaryBoxMin;

    //! Maximum position of boundary box.
    cVector3d m_boundaryBoxMax;

    //! If __true__, then boundary box does not include any object.
    bool m_boundaryBoxEmpty;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - FRAME REPRESENTATION [X,Y,Z]:
    //-----------------------------------------------------------------------

protected:

    //! Size of graphical representation of frame (X-Y-Z).
    double m_frameSize;

    //! Pen thickness of graphical representation of frame (X-Y-Z).
    double m_frameThicknessScale;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - GENERAL OPTIONS:
    //-----------------------------------------------------------------------

protected:

    //! If __true__, the object may be rendered graphicaly and hapticaly. 
    bool m_enabled;

    //! If __true__, this object is rendered.
    bool m_showEnabled;

    //! If __true__, this object can be felt.
    bool m_hapticEnabled;

    //! If __true__, object is enabled as ghost. 
    bool m_ghostEnabled;

    //! If __true__, this object's reference frame is rendered as a set of arrows.
    bool m_showFrame;

    //! If __true__, this object's boundary box is displayed as a set of lines.
    bool m_showBoundaryBox;

    //! If __true__, the collision detector is displayed (if available) at this node.
    bool m_showCollisionDetector;

    //! Should texture mapping be used?
    bool m_useTextureMapping;

    //! Should material properties be used?
    bool m_useMaterialProperty;

    //! Should per-vertex colors be used?
    bool m_useVertexColors;

    //! Should we use a display list to render this mesh?
    bool m_useDisplayList;

    //! Basic display list for current object.
    cDisplayList m_displayList;

    //! The polygon rendering mode (GL_FILL or GL_LINE).
    int m_triangleMode;

    /*!
        If __true__, transparency is enabled... this turns alpha on when the mesh is
        rendered, and - if multipass transparency is enabled in the rendering camera -
        uses the camera's multiple rendering passes to approximate back-to-front
        sorting via culling.
    */
    bool m_useTransparency;

    /*!
        Should culling be used when rendering triangles? \n

        Note that this option only applies when multipass transparency is
        disabled or during the non-transparent rendering pass when multipass
        transparency is enabled... \n

        Also note that currently only back-faces are culled during non-transparent
        rendering; you can't cull front-faces.
    */
    bool m_cullingEnabled;

    //! Default material property.
    static cMaterialPtr s_defaultMaterial;

    //! Shader program.
    cShaderProgramPtr m_shaderProgram;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - COLLISION DETECTION:
    //-----------------------------------------------------------------------

protected:

    //! The collision detector used to test for contact with this object.
    cGenericCollision* m_collisionDetector;


    //-----------------------------------------------------------------------
    // PROTECTED MEMBERS - HAPTIC EFFECTS AND INTERACTIONS:
    //-----------------------------------------------------------------------

protected:

    //! List of haptic effects programmed for this object.
    std::vector<cGenericEffect*> m_effects;


    //-----------------------------------------------------------------------
    // PROTECTED VIRTUAL METHODS:
    //-----------------------------------------------------------------------

protected:

    //! Render object graphically (OpenGL).
    virtual void render(cRenderOptions& a_options);

    //! Update the m_globalPos and m_globalRot properties of any members of this object (e.g. all triangles).
    virtual void updateGlobalPositions(const bool a_frameOnly) {};

    //! Update the bounding box of this object, based on object-specific data (e.g. triangle positions).
    virtual void updateBoundaryBox() {};

    //! Scale current object with scale factor.
    virtual void scaleObject(const double& a_scaleFactor) { m_boundaryBoxMin.mul(a_scaleFactor); m_boundaryBoxMax.mul(a_scaleFactor);}

    //! Update the geometric relationship between the tool and the current object.
    virtual void computeLocalInteraction(const cVector3d& a_toolPos,
        const cVector3d& a_toolVel,
        const unsigned int a_IDN);

    //! computes any other interactions between the object and the tools. 
    virtual cVector3d computeOtherInteractions(const cVector3d& a_toolPos,
        const cVector3d& a_toolVel,
        const unsigned int a_IDN,
        cInteractionRecorder& a_interactions) { return cVector3d(0,0,0); }

    //! Compute any collisions other than the ones computed by the default collision detector defined by m_collisionDetector.
    virtual bool computeOtherCollisionDetection(cVector3d& a_segmentPointA,
        cVector3d& a_segmentPointB,
        cCollisionRecorder& a_recorder,
        cCollisionSettings& a_settings) {return(false);}


    //-----------------------------------------------------------------------
    // PROTECTED METHODS:
    //-----------------------------------------------------------------------

protected:

    //! Copy properties of current generic object to another.
    void copyGenericObjectProperties(cGenericObject* a_objDest, 
        const bool a_duplicateMaterialData,
        const bool a_duplicateTextureData, 
        const bool a_duplicateMeshData,
        const bool a_buildCollisionDetector);


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - INTERACTIONS:
    //-----------------------------------------------------------------------

public: 

    //! Projection of the most recent haptic point (tool) onto the surface of the virtual object.
    cVector3d m_interactionPoint;

    //! Surface normal at the current interaction point.
    cVector3d m_interactionNormal;

    //! Was the last tool (haptic point) located inside the object?
    bool m_interactionInside;

    //! OpenGL matrix describing my position and orientation transformation.
    cTransform m_frameGL;


    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS - GENERAL
    //-----------------------------------------------------------------------

public:

    //! Render the entire scene graph, starting from this object.
    virtual void renderSceneGraph(cRenderOptions& a_options);

    //! Adjust collision segment for dynamic objects.
    virtual void adjustCollisionSegment(cVector3d& a_segmentPointA, cVector3d& a_segmentPointAadjusted);

    //! Computer haptic interaction.
    virtual cVector3d computeInteractions(const cVector3d& a_toolPos,
        const cVector3d& a_toolVel,
        const unsigned int a_IDN,
        cInteractionRecorder& a_interactions);
};

//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

