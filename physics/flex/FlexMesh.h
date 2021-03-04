/*
    This source file is part of Rigs of Rods
    Copyright 2005-2012 Pierre-Michel Ricordel
    Copyright 2007-2012 Thomas Fischer
    Copyright 2013-2017 Petr Ohlidal & contributors

    For more information, see http://www.rigsofrods.org/

    Rigs of Rods is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3, as
    published by the Free Software Foundation.

    Rigs of Rods is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Rigs of Rods. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "RoRPrerequisites.h"

#include "Flexable.h"

<string>
 
 
 
#include <OgreSubMesh.h>
#include <OgreHardwareBuffer.h>

class FlexMesh: public Flexable
{
public:

    FlexMesh(
        std::string const& name,
        //RoR::GfxActor* gfx_actor,
        int n1,
        int n2,
        int nstart,
        int nrays,
        std::string const& face_material_name,
        std::string const& band_material_name,
        bool rimmed = false,
        float rimratio = 1.f
    );

    ~FlexMesh();

    irr::core::vector3df updateVertices();

    // Flexable
    bool flexitPrepare() { return true; };
    void flexitCompute();
    irr::core::vector3df flexitFinal();

    void setVisible(bool visible) {} // Nothing to do here

private:

    struct FlexMeshVertex
    {
        irr::core::vector3df position;
        irr::core::vector3df normal;
        irr::core::vector2df texcoord;
    };

    // Wheel
    irr::core::vector3df     m_flexit_center;
    //RoR::GfxActor*    m_gfx_actor;
    int               m_num_rays;
    bool              m_is_rimmed;

    // Meshes
    Ogre::MeshPtr     m_mesh;
    Ogre::SubMesh*    m_submesh_wheelface;
    Ogre::SubMesh*    m_submesh_tiretread;
    Ogre::VertexDeclaration* m_vertex_format;
    Ogre::HardwareVertexBufferSharedPtr m_hw_vbuf;

    // Vertices
    FlexMeshVertex*   m_vertices;
    int*              m_vertex_nodes;

    // Indices
    unsigned short*   m_wheelface_indices;
    unsigned short*   m_tiretread_indices;
};
