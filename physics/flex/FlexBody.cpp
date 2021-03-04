#include "FlexBody.h"

#include "Application.h"
#include "ApproxMath.h"
#include "BeamData.h"

#include <irrlicht.h>
#include <vector>
#include <limits>

using namespace irr;
using namespace irr::core;

struct NodeData {
	irr::core::vector3df AbsPosition;
	bool nd_has_contact:1;
	bool nd_is_wet:1;
};

FlexBody::FlexBody(
    int ref,
    int nx,
    int ny,
    irr::core::quaternion const & rot,
    std::vector<unsigned int> & node_indices
):
      m_camera_mode(-2)
    /*m_center_offset(def->offset)
    , m_node_center(ref)*/
    , m_node_x(nx)
    , m_node_y(ny)
    , m_has_texture_blend(true)
    , m_scene_node(nullptr)
    //, m_scene_entity(ent)
    , m_shared_buf_num_verts(0)
    , m_has_texture(true)
    , m_blend_changed(false)
    , m_locators(nullptr)
    , m_src_normals(nullptr)
    , m_dst_normals(nullptr)
    , m_dst_pos(nullptr)
    , m_src_colors(nullptr)
    //, m_gfx_actor(gfx_actor)
{

    irr::core::vector3df* vertices = nullptr;

    irr::core::vector3df normal = irr::core::vector3df(0.,1.,0.);
    irr::core::vector3df position = irr::core::vector3df(0.f,0.f,0.f);
    irr::core::quaternion orientation = irr::core::quaternion();

    //RoR::GfxActor::NodeData* nodes = m_gfx_actor->GetSimNodeBuffer();
	std::vector<NodeData> nodes;
	//TODO equal nodes to something

    if (ref >= 0) {
	irr::core::vector3df diffX = nodes[nx].AbsPosition-nodes[ref].AbsPosition;
        irr::core::vector3df diffY = nodes[ny].AbsPosition-nodes[ref].AbsPosition;

        normal = (diffY.crossProduct(diffX)).normalize();

        // position
        position = nodes[ref].AbsPosition + def->offset.X * diffX + def->offset.Y * diffY;
        position = position + def->offset.Z * normal;

        // orientation
        irr::core::vector3df refX = diffX;
		refX.normalize();
        irr::core::vector3df refY = refX.crossProduct(normal);
        orientation  = irr::core::quaternion(refX, normal, refY) * rot;
    }  else {
        // special case!
        normal = irr::core::vector3df(0.,1.,0.);
        position = nodes[0].AbsPosition /*+ def->offset*/;
        orientation = rot;
    }


	irr::scene::IMeshSceneNode* mesh;
	//=ent->getMesh();
	//TODO equal mesh with something
    //int num_submeshes = static_cast<int>(mesh->getNumSubMeshes());


    // Profiler data
    double stat_manual_buffers_created_time = -1;
    double stat_transformed_time = -1;
    double stat_located_time = -1;

	vertices=(irr::core::vector3df*)malloc(sizeof(irr::core::vector3df)*m_vertex_count);
	m_dst_pos=(irr::core::vector3df*)malloc(sizeof(irr::core::vector3df)*m_vertex_count);
	m_src_normals=(irr::core::vector3df*)malloc(sizeof(irr::core::vector3df)*m_vertex_count);
	m_dst_normals=(irr::core::vector3df*)malloc(sizeof(irr::core::vector3df)*m_vertex_count);

	irr::core::vector3df* vpt=vertices;
	irr::core::vector3df* npt=m_src_normals;

	int cursubmesh=0;

	//transform
	for (int i=0; i<(int)m_vertex_count; i++) {
		vertices[i]=(orientation*vertices[i])+position;
	}

	m_locators = new Locator_t[m_vertex_count];
	for (int i=0; i<(int)m_vertex_count; i++) {
		//search nearest node as the local origin
		float closest_node_distance = std::numeric_limits<float>::max();
		int closest_node_index = -1;
		for (auto node_index : node_indices) {
			float node_distance = vertices[i].getDistanceFromSQ(nodes[node_index].AbsPosition);
			if (node_distance < closest_node_distance) {
				closest_node_distance = node_distance;
				closest_node_index = node_index;
			}
		}
		if (closest_node_index == -1) {
			//LOG("FLEXBODY ERROR on mesh "+def->mesh_name+": REF node not found");
			closest_node_index = 0;
		}
		m_locators[i].ref=closest_node_index;            

		//search the second nearest node as the X vector
		closest_node_distance = std::numeric_limits<float>::max();
		closest_node_index = -1;
		for (auto node_index : node_indices) {
			if (node_index == m_locators[i].ref)
			{
				continue;
			}
			float node_distance = vertices[i].getDistanceFromSQ(nodes[node_index].AbsPosition);
			if (node_distance < closest_node_distance)
			{
				closest_node_distance = node_distance;
				closest_node_index = node_index;
			}
		}
		if (closest_node_index == -1)
		{
			//LOG("FLEXBODY ERROR on mesh "+def->mesh_name+": VX node not found");
			closest_node_index = 0;
		}
		m_locators[i].nx=closest_node_index;

		//search another close, orthogonal node as the Y vector
		closest_node_distance = std::numeric_limits<float>::max();
		closest_node_index = -1;
		//irr::core::vector3df vx = (nodes[m_locators[i].nx].AbsPosition - nodes[m_locators[i].ref].AbsPosition).normalisedCopy();
		irr::core::vector3df vx = (nodes[m_locators[i].nx].AbsPosition - nodes[m_locators[i].ref].AbsPosition).normalize();
		for (auto node_index : node_indices)
		{
			if (node_index == m_locators[i].ref || node_index == m_locators[i].nx)
			{
				continue;
			}
			//float node_distance = vertices[i].squaredDistance(nodes[node_index].AbsPosition);
			float node_distance = vertices[i].getDistanceFromSQ(nodes[node_index].AbsPosition);
			if (node_distance < closest_node_distance) {
				irr::core::vector3df vt = (nodes[node_index].AbsPosition - nodes[m_locators[i].ref].AbsPosition).normalize();
				float cost = vx.dotProduct(vt);
				if (std::abs(cost) > std::sqrt(2.0f) / 2.0f) {
					continue; //rejection, fails the orthogonality criterion (+-45 degree)
				}
				closest_node_distance = node_distance;
				closest_node_index = node_index;
			}
		}
		if (closest_node_index == -1) {
			//LOG("FLEXBODY ERROR on mesh "+def->mesh_name+": VY node not found");
			closest_node_index = 0;
		}
		m_locators[i].ny=closest_node_index;

		//Matrix3 mat;
		irr::core::matrix4 mat;
		irr::core::vector3df diffX = nodes[m_locators[i].nx].AbsPosition-nodes[m_locators[i].ref].AbsPosition;
		irr::core::vector3df diffY = nodes[m_locators[i].ny].AbsPosition-nodes[m_locators[i].ref].AbsPosition;

		mat(0,0) = diffX.X; mat(1,0) = diffX.X; mat(2,0) = diffX.Z;
		mat(0,1) = diffY.X; mat(1,1) = diffY.X; mat(2,1) = diffY.Z;
		
		irr::core::vector3df vec=(diffX.crossProduct(diffY)).normalize();
		mat(0,2) = vec.X; mat(1,2) = vec.X; mat(2,2) = vec.Z;
		
		//mat.SetColumn(0, diffX);
		//mat.SetColumn(1, diffY);
		//mat.SetColumn(2, (diffX.crossProduct(diffY)).normalize()); // Old version: mat.SetColumn(2, nodes[loc.nz].AbsPosition-nodes[loc.ref].AbsPosition);

		mat = mat.getInverse(mat);

		//compute coordinates in the newly formed Euclidean basis
		m_locators[i].coords = mat * (vertices[i] - nodes[m_locators[i].ref].AbsPosition);

		// that's it!
	}



    //adjusting bounds
    irr::core::aabbox3df aab=mesh->getBoundingBox();
    irr::core::vector3df v=aab.getMinimum();
    float mi=v.X;
    if (v.Y<mi) mi=v.Y;
    if (v.Z<mi) mi=v.Z;
    mi=fabs(mi);
    v=aab.getMaximum();
    float ma=v.X;
    if (ma<v.Y) ma=v.Y;
    if (ma<v.Z) ma=v.Z;
    ma=fabs(ma);
    if (mi>ma) ma=mi;
    //aab.setMinimum(irr::core::vector3df(-ma,-ma,-ma));
    //aab.setMaximum(irr::core::vector3df(ma,ma,ma));
    aab=aabbox3df (vector3df(-ma,-ma,-ma), vector3df(ma,ma,ma));
    //mesh->_setBounds(aab, true
	mesh->setBoundingBox(aab);

    //okay, show the mesh now
    //m_scene_node=gEnv->sceneManager->getRootSceneNode()->createChildSceneNode();
    //m_scene_node->attachObject(ent);
    m_scene_node->setPosition(position);


	for (int i=0; i<(int)m_vertex_count; i++) {
		//Matrix3 mat;
		irr::core::matrix4 mat;

		irr::core::vector3df diffX = nodes[m_locators[i].nx].AbsPosition-nodes[m_locators[i].ref].AbsPosition;
		irr::core::vector3df diffY = nodes[m_locators[i].ny].AbsPosition-nodes[m_locators[i].ref].AbsPosition;
		
		//for (int j=0;j<3; j++) {
		    mat(0,0)=diffX.X;mat(1,0)=diffX.Y;mat(2,0)=diffX.Z;
		    mat(0,1)=diffY.X;mat(1,1)=diffY.Y;mat(2,1)=diffY.Z;
		    irr::core::vector3df vec=diffX.crossProduct(diffY).normalize();
		    mat(0,2)=vec.X;mat(1,2)=vec.Y;mat(2,2)=vec.Z;
		//}
		//mat.SetColumn(0, diffX);
		//mat.SetColumn(1, diffY);
		//mat.SetColumn(2, diffX.crossProduct(diffY).normalisedCopy()); // Old version: mat.SetColumn(2, nodes[loc.nz].AbsPosition-nodes[loc.ref].AbsPosition);

		//mat = 
		mat.getInverse(mat);

		// compute coordinates in the Euclidean basis
		m_src_normals[i] = mat*(orientation * m_src_normals[i]);
	}


    if (vertices != nullptr) { free(vertices); }

}

FlexBody::~FlexBody() {
    // Stuff using <new>
    if (m_locators != nullptr) { delete[] m_locators; }
    // Stuff using malloc()
    if (m_src_normals != nullptr) { free(m_src_normals); }
    if (m_dst_normals != nullptr) { free(m_dst_normals); }
    if (m_dst_pos     != nullptr) { free(m_dst_pos    ); }
    if (m_src_colors  != nullptr) { free(m_src_colors ); }


    m_scene_node = nullptr;

}

void FlexBody::setVisible(bool visible)
{
    if (m_scene_node)
        m_scene_node->setVisible(visible);
}

void FlexBody::SetFlexbodyCastShadow(bool val)
{
    //m_scene_entity->setCastShadows(val);
}

void FlexBody::printMeshInfo(irr::scene::IMeshSceneNode* mesh) {
}

void FlexBody::ComputeFlexbody()
{
    if (m_has_texture_blend) updateBlend();

    //RoR::GfxActor::NodeData* nodes = m_gfx_actor->GetSimNodeBuffer();
    std::vector<NodeData> nodes;
    // compute the local center
    irr::core::vector3df flexit_normal;

    if (m_node_center >= 0) {
        irr::core::vector3df diffX = nodes[m_node_x].AbsPosition - nodes[m_node_center].AbsPosition;
        irr::core::vector3df diffY = nodes[m_node_y].AbsPosition - nodes[m_node_center].AbsPosition;
        flexit_normal = fast_normalise(diffY.crossProduct(diffX));

        m_flexit_center = nodes[m_node_center].AbsPosition + m_center_offset.X * diffX + m_center_offset.Y * diffY;
        m_flexit_center += m_center_offset.Z * flexit_normal;
    } else {
        flexit_normal = irr::core::vector3df(0.,1.,0.);
        m_flexit_center = nodes[0].AbsPosition;
    }

    for (int i=0; i<(int)m_vertex_count; i++) {
        irr::core::vector3df diffX = nodes[m_locators[i].nx].AbsPosition - nodes[m_locators[i].ref].AbsPosition;
        irr::core::vector3df diffY = nodes[m_locators[i].ny].AbsPosition - nodes[m_locators[i].ref].AbsPosition;
        irr::core::vector3df nCross = fast_normalise(diffX.crossProduct(diffY)); //nCross.normalise();

        m_dst_pos[i].X = diffX.X * m_locators[i].coords.X + diffY.X * m_locators[i].coords.Y + nCross.X * m_locators[i].coords.Z;
        m_dst_pos[i].Y = diffX.Y * m_locators[i].coords.X + diffY.Y * m_locators[i].coords.Y + nCross.Y * m_locators[i].coords.Z;
        m_dst_pos[i].Z = diffX.Z * m_locators[i].coords.X + diffY.Z * m_locators[i].coords.Y + nCross.Z * m_locators[i].coords.Z;

        m_dst_pos[i] += nodes[m_locators[i].ref].AbsPosition - m_flexit_center;

        m_dst_normals[i].X = diffX.X * m_src_normals[i].X + diffY.X * m_src_normals[i].Y + nCross.X * m_src_normals[i].Z;
        m_dst_normals[i].Y = diffX.Y * m_src_normals[i].X + diffY.Y * m_src_normals[i].Y + nCross.Y * m_src_normals[i].Z;
        m_dst_normals[i].Z = diffX.Z * m_src_normals[i].X + diffY.Z * m_src_normals[i].Y + nCross.Z * m_src_normals[i].Z;

        m_dst_normals[i] = fast_normalise(m_dst_normals[i]);
    }
}

void FlexBody::UpdateFlexbodyVertexBuffers() {

    m_scene_node->setPosition(m_flexit_center);
}

void FlexBody::reset()
{

}

void FlexBody::writeBlend()
{

}

void FlexBody::updateBlend() //so easy!
{

}

