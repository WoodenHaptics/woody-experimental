//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2014, CHAI3D
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
//===========================================================================

//---------------------------------------------------------------------------
#include "CDemo4.h"
using namespace std;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cDemo4.
*/
//===========================================================================
cDemo4::cDemo4(const string a_resourceRoot,
               const int a_numDevices,
               shared_ptr<cGenericHapticDevice> a_hapticDevice0,
               shared_ptr<cGenericHapticDevice> a_hapticDevice1):cGenericDemo(a_resourceRoot, a_numDevices, a_hapticDevice0, a_hapticDevice1)
{
    cMaterial matBase;
    matBase.setGrayLevel(0.3);
    matBase.setStiffness(1500);

    m_ODEBase0 = new cODEGenericBody(m_ODEWorld);
    cMesh* base0 = new cMesh();
    cCreateCylinder(base0, 0.15, 0.01, 12, 1, true, true, cVector3d(0,0,-0.075), cIdentity3d(), cColorf(1.0, 1.0, 1.0));
    m_ODEBase0->createDynamicCylinder(0.01, 0.15, true);
    base0->createAABBCollisionDetector(m_toolRadius);
    base0->setMaterial(matBase);
    m_ODEBase0->setImageModel(base0);
    m_ODEBase0->setLocalPos(0.015, 0.0, -0.02);
  
    // create a new ODE object that is automatically added to the ODE world
    m_ODEBody0 = new cODEGenericBody(m_ODEWorld);
    cMesh* object0 = new cMesh();
    cCreateRing(object0, 0.01, 0.025, 12, 16);
    object0->createAABBCollisionDetector(m_toolRadius);

    // define some material properties for each cube
    cMaterial mat;
  	mat.setWhite();
  	mat.m_specular.set(0.0, 0.0, 0.0);
    mat.setStiffness(1000);
    mat.setDynamicFriction(0.6);
    mat.setStaticFriction(0.6);
    object0->setMaterial(mat);
    m_ODEBody0->setImageModel(object0);
    m_ODEBody0->createDynamicMesh();
    m_ODEBody0->setMass(0.02);

    // initialize
    init();
};


//===========================================================================
/*!
    Set stiffness of environment

    \param  a_stiffness  Stiffness value [N/m]
*/
//===========================================================================
void cDemo4::setStiffness(double a_stiffness)
{
    // set ground
    m_ground->setStiffness(a_stiffness, true);
    
    // set objects
    m_ODEBody0->m_imageModel->setStiffness(a_stiffness);
    m_ODEBase0->m_imageModel->setStiffness(a_stiffness);
\
};


//===========================================================================
/*!
    Initialize position of objects
*/
//===========================================================================
void cDemo4::init()
{
    m_ODEBody0->setLocalPos(0.04,-0.05, 0.04);
}
