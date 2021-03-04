#pragma once

//#include "RigDef_Prerequisites.h"
#include "RoRPrerequisites.h"
#include "Flexable.h"
#include "Locator_t.h"

#include <irrlicht.h>
#include <vector>
#include <string>
#include <list>

namespace RigDef{//Originaly in RigDef file
struct Flexbody
{
    Flexbody():
        offset(irr::core::vector3df(0.f)),
        rotation(irr::core::vector3df(0.f))
    {
    }

    //Node::Ref reference_node;
    //Node::Ref x_axis_node;
    //Node::Ref y_axis_node;
    irr::core::vector3df offset;
    irr::core::vector3df rotation;
    std::string mesh_name;
    //std::list<Animation> animations;				//LUCIANO
    //std::vector<Node::Range> node_list_to_import; //< Node ranges are disallowed in fileformatversion >=450
    //std::vector<Node::Ref> node_list;
    //CameraSettings camera_settings;
};
}
/// Flexbody = A deformable mesh; updated on CPU every frame, then uploaded to video memory
class FlexBody
{
    //friend class RoR::FlexFactory;
    //friend class RoR::FlexBodyFileIO;

    FlexBody( // Private, for FlexFactory
        //RigDef::Flexbody*         ,
        //RoR::FlexBodyCacheData* preloaded_from_cache,
        //RoR::GfxActor* gfx_actor,
        //Ogre::Entity* entity,
        int ref, 
        int nx, 
        int ny,
        irr::core::quaternion const & rot, 
        std::vector<unsigned int> & node_indices
    );

public:

    ~FlexBody();

    void printMeshInfo(irr::scene::IMeshSceneNode* mesh);
    void reset();
    void updateBlend();
    void writeBlend();
    irr::scene::ISceneNode *getSceneNode() { return m_scene_node; };

    /// Visibility control 
    /// @param mode {-2 = always, -1 = 3rdPerson only, 0+ = cinecam index}
    void setCameraMode(int mode) { m_camera_mode = mode; };
    int getCameraMode() { return m_camera_mode; };

    void ComputeFlexbody(); //!< Updates mesh deformation; works on CPU using local copy of vertex data.
    void UpdateFlexbodyVertexBuffers();

    void setVisible(bool visible);

    void SetFlexbodyCastShadow(bool val);

    int size() { return static_cast<int>(m_vertex_count); };

private:

    //RoR::GfxActor*    			m_gfx_actor;
    size_t            			m_vertex_count;
    irr::core::vector3df     	m_flexit_center; ///< Updated per frame

    irr::core::vector3df*   	m_dst_pos;
    irr::core::vector3df*   	m_src_normals;
    irr::core::vector3df*   	m_dst_normals;
    irr::video::SColor*       	m_src_colors;
    Locator_t*        			m_locators; ///< 1 loc per vertex

    int               			m_node_center;
    int               			m_node_x;
    int               			m_node_y;
    irr::core::vector3df     	m_center_offset;
    irr::scene::ISceneNode*  	m_scene_node;
    //Ogre::Entity*     m_scene_entity;
    int               m_camera_mode; ///< Visibility control {-2 = always, -1 = 3rdPerson only, 0+ = cinecam index}


    int                                 m_shared_buf_num_verts;
    irr::scene::IVertexBuffer* 	m_shared_vbuf_pos;
    irr::scene::IVertexBuffer* 	m_shared_vbuf_norm;
    irr::scene::IVertexBuffer* 	m_shared_vbuf_color;

    int                         m_num_submesh_vbufs;
    int                         m_submesh_vbufs_vertex_counts[16];
    irr::scene::IVertexBuffer* 	m_submesh_vbufs_pos[16];   ///< positions
    irr::scene::IVertexBuffer* 	m_submesh_vbufs_norm[16];  ///< normals
    irr::scene::IVertexBuffer* 	m_submesh_vbufs_color[16]; ///< colors


    bool m_uses_shared_vertex_data;
    bool m_has_texture;
    bool m_has_texture_blend;
    bool m_blend_changed;
};
