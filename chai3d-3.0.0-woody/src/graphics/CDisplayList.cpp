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
    \version   3.0.0 $Rev: 1242 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "graphics/CDisplayList.h"
//------------------------------------------------------------------------------
#ifdef C_USE_OPENGL
#include "GL/glew.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    Constructor of cDisplayList. \n
    The display list is set to zero as it has not been created. We also 
    initialize the flag which tells us if a display list is currently being created.
*/
//==============================================================================
cDisplayList::cDisplayList()
{
    for (int i=0; i<C_MAX_DISPLAY_CONTEXTS; i++)
    {
        m_displayList[i] = 0;
        m_flagCreatingDisplayList[i] = false;
        m_flagMarkedForDeletion[i] = false;
    }
}


//==============================================================================
/*!
    Destructor of cDisplayList.
*/
//==============================================================================
cDisplayList::~cDisplayList()
{
}


//==============================================================================
/*!
    Invalidate current display list. We inform the display list that it will
    need to be update the next time the object is rendered. We free any 
    graphic card memory which contains the current display list.
*/
//==============================================================================
void cDisplayList::invalidate()
{
    for (int i=0; i<C_MAX_DISPLAY_CONTEXTS; i++)
    {
        m_flagMarkedForDeletion[i] = true;
    }
}


//==============================================================================
/*!
    Render display list. We pass a boolean from the object which will
    check if the display list should be used. If the display list is valid
    and used for rendering the object, the method returns __true__, otherwise
    __false__.

    \param  a_displayContext  Display context number.
    \param  a_useDisplayList  If __true__, then display list is rendered.

    \return Return __true__ is display list was rendered, otherwise __false__
            if display list was invalid. 
*/
//==============================================================================
bool cDisplayList::render(const int a_displayContext, const bool a_useDisplayList)
{
    // check if display list needs to be deleted
    if (m_flagMarkedForDeletion[a_displayContext])
    {
        // clear any current display list
        if (m_displayList[a_displayContext] != 0)
        {
#ifdef C_USE_OPENGL
            glDeleteLists(m_displayList[a_displayContext], 1);
#endif
            m_displayList[a_displayContext] = 0;
        }
        m_flagMarkedForDeletion[a_displayContext] = false;
    }

    // if available, render display list
    if ((a_useDisplayList) && (m_displayList[a_displayContext] != 0))
    {
        #ifdef C_USE_OPENGL
        glCallList(m_displayList[a_displayContext]);
        #endif

        return (true);
    }
    else
    {
        return (false);
    }
}


//==============================================================================
/*!
    Begin creating a display list. 

    \param  a_displayContext  Display context number.
    \param  a_useDisplayList  If __true__, then display list is created.

    \return Return __true__ if the OpenGL display list has been allocated
            successfully.
*/
//==============================================================================
bool cDisplayList::begin(const int a_displayContext, const bool a_useDisplayList)
{
    if (a_useDisplayList)
    {
        // clear any current display list
        if (m_displayList[a_displayContext] != 0)
        {
#ifdef C_USE_OPENGL
            glDeleteLists(m_displayList[a_displayContext], 1);
#endif
            m_displayList[a_displayContext] = 0;
        }
        m_flagMarkedForDeletion[a_displayContext] = false;

        // create an OpenGL display list
        #ifdef C_USE_OPENGL
        m_displayList[a_displayContext] = glGenLists(1);
        #endif

        // verify result
        if (m_displayList[a_displayContext] == 0) 
        {
            // operation failed
            return (false);
        }
        else
        {
            // On some machines, GL_COMPILE_AND_EXECUTE totally blows for some reason,
            // so even though it's more complex on the first rendering pass, we use
            // GL_COMPILE (and _repeat_ the first rendering pass)
            #ifdef C_USE_OPENGL
            glNewList(m_displayList[a_displayContext], GL_COMPILE);
            #endif

            // we are now creating a display list
            m_flagCreatingDisplayList[a_displayContext] = true;

            // success
            return (true);
        }
    }
    else
    {
        return (false);
    }
}


//==============================================================================
/*!
    Finalize and compile the display list. Optionally render the display list
    as nothing will have yet been rendered at the screen during display
    list compilation which began after calling method begin().

    \param  a_displayContext  Display context number.
    \param  a_executeDisplayList  If __true__, render display list.
*/
//==============================================================================
void cDisplayList::end(const int a_displayContext, const bool a_executeDisplayList)
{
    if (m_flagCreatingDisplayList[a_displayContext])
    {
        // finalize list
        #ifdef C_USE_OPENGL
        glEndList();
        #endif

        // display list has been finalized
        m_flagCreatingDisplayList[a_displayContext] = false;

        // execute display list if requested
        if (a_executeDisplayList)
        {
            render(a_displayContext, true);
        }
    }
}


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------
