/*
    This source file is part of Rigs of Rods
    Copyright 2005-2012 Pierre-Michel Ricordel
    Copyright 2007-2012 Thomas Fischer

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

#include <irrlicht.h>

#include "RoRPrerequisites.h"
#include "BeamData.h" // For MAX_AEROENGINES

class FlexAirfoil : public ZeroedMemoryAllocator
{
    friend class RigInspector; // Debug utility class

public:

    FlexAirfoil(std::string const& wname, Actor* actor,
        int pnfld, int pnfrd, int pnflu, int pnfru, int pnbld, int pnbrd, int pnblu, int pnbru,
        std::string const & texname,
        irr::core::vector2df texlf, irr::core::vector2df texrf, irr::core::vector2df texlb, irr::core::vector2df texrb,
        char mtype, float controlratio, float mind, float maxd, std::string const& afname, float lift_coef, bool break_able);

    ~FlexAirfoil();

    // DEV NOTE: original `updateVertices()` updated both physics state + visuals.
    void updateVerticesPhysics();
    irr::core::vector3df updateVerticesGfx(RoR::GfxActor* gfx_actor);
    void uploadVertices();

    void setControlDeflection(float val);

    void enableInducedDrag(float span, float area, bool l);

    void addwash(int propid, float ratio);

    void updateForces();

    float aoa;
    char type;
    int nfld;
    int nfrd;
    int nflu;
    int nfru;
    int nbld;
    int nbrd;
    int nblu;
    int nbru;

    bool broken;
    bool breakable;
    float liftcoef;

private:

    float airfoilpos[90];

    typedef struct
    {
        irr::core::vector3df vertex;
        irr::core::vector3df normal;
        //	irr::core::vector3df color;
        irr::core::vector2df texcoord;
    } CoVertice_t;

    Ogre::MeshPtr msh;
    Ogre::SubMesh* subface;
    Ogre::SubMesh* subband;

    Ogre::SubMesh* subcup;
    Ogre::SubMesh* subcdn;

    Ogre::VertexDeclaration* decl;
    Ogre::HardwareVertexBufferSharedPtr vbuf;

    size_t nVertices;
    size_t vbufCount;

    union
    {
        float* vertices;
        CoVertice_t* covertices;
    };

    size_t faceibufCount;
    size_t bandibufCount;
    size_t cupibufCount;
    size_t cdnibufCount;
    unsigned short* facefaces;
    unsigned short* bandfaces;
    unsigned short* cupfaces;
    unsigned short* cdnfaces;
    node_t* nodes;

    float sref;

    float deflection;
    float chordratio;
    bool hascontrol;
    bool isstabilator;
    bool stabilleft;
    float lratio;
    float rratio;
    float mindef;
    float maxdef;
    float thickness;
    bool useInducedDrag;
    float idSpan;
    float idArea;
    bool idLeft;

    Airfoil* airfoil;
    AeroEngine** aeroengines;
    int free_wash;
    int washpropnum[MAX_AEROENGINES];
    float washpropratio[MAX_AEROENGINES];
};
