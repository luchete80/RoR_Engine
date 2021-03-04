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

#include "Beam.h"
#include "BeamData.h"
//#include "BeamFactory.h"
//#include "CmdKeyInertia.h"
#include "Collisions.h"
//#include "DashBoardManager.h"
#include "DynamicCollisions.h"
//#include "ErrorUtils.h"
#include "FlexBody.h"
#include "FlexMesh.h"
#include "FlexObj.h"
//#include "Console.h"
//#include "GfxActor.h"
//#include "InputEngine.h"
//#include "Language.h"
#include "MeshObject.h"
#include "PointColDetector.h"
//#include "Replay.h"
//#include "RigSpawner.h"
//#include "RoRFrameListener.h"
//#include "SlideNode.h"
//#include "Utils.h"


using namespace Ogre;
using namespace RoR;

static const irr::core::vector3df BOUNDING_BOX_PADDING(0.05f, 0.05f, 0.05f);

Actor::~Actor()
{
    TRIGGER_EVENT(SE_GENERIC_DELETED_TRUCK, ar_instance_id);

    // TODO: IMPROVE below: delete/destroy prop entities, etc

    this->DisjoinInterActorBeams();

    // delete all classes we might have constructed
    if (ar_dashboard != nullptr)
    {
        delete ar_dashboard;
        ar_dashboard = nullptr;
    }

    if (m_replay_handler)
        delete m_replay_handler;
    m_replay_handler = nullptr;

    if (ar_vehicle_ai)
        delete ar_vehicle_ai;
    ar_vehicle_ai = 0;

    // TODO: Make sure we catch everything here
    // remove all scene nodes
    if (m_deletion_scene_nodes.size() > 0)
    {
        for (unsigned int i = 0; i < m_deletion_scene_nodes.size(); i++)
        {
            if (!m_deletion_scene_nodes[i])
                continue;
            m_deletion_scene_nodes[i]->removeAndDestroyAllChildren();
            gEnv->sceneManager->destroySceneNode(m_deletion_scene_nodes[i]);
        }
        m_deletion_scene_nodes.clear();
    }
    // remove all entities
    if (m_deletion_entities.size() > 0)
    {
        for (unsigned int i = 0; i < m_deletion_entities.size(); i++)
        {
            if (!m_deletion_entities[i])
                continue;
            m_deletion_entities[i]->detachAllObjectsFromBone();
            gEnv->sceneManager->destroyEntity(m_deletion_entities[i]->getName());
        }
        m_deletion_entities.clear();
    }

    // delete wings
    for (int i = 0; i < ar_num_wings; i++)
    {
        // flexAirfoil, airfoil
        if (ar_wings[i].fa)
            delete ar_wings[i].fa;
        if (ar_wings[i].cnode)
        {
            ar_wings[i].cnode->removeAndDestroyAllChildren();
            gEnv->sceneManager->destroySceneNode(ar_wings[i].cnode);
        }
    }

    // delete aeroengines
    for (int i = 0; i < ar_num_aeroengines; i++)
    {
        if (ar_aeroengines[i])
            delete ar_aeroengines[i];
    }

    // delete screwprops
    for (int i = 0; i < ar_num_screwprops; i++)
    {
        if (ar_screwprops[i])
        {
            delete ar_screwprops[i];
            ar_screwprops[i] = nullptr;
        }
    }

    // delete airbrakes
    for (Airbrake* ab: ar_airbrakes)
    {
        delete ab;
    }
    ar_airbrakes.clear();

    // delete skidmarks
    for (int i = 0; i < ar_num_wheels; ++i)
    {
        delete m_skid_trails[i];
        m_skid_trails[i] = nullptr;
    }

    // delete flares
    for (size_t i = 0; i < this->ar_flares.size(); i++)
    {
        if (ar_flares[i].snode)
        {
            ar_flares[i].snode->removeAndDestroyAllChildren();
            gEnv->sceneManager->destroySceneNode(ar_flares[i].snode);
        }
        if (ar_flares[i].bbs)
            gEnv->sceneManager->destroyBillboardSet(ar_flares[i].bbs);
        if (ar_flares[i].light)
            gEnv->sceneManager->destroyLight(ar_flares[i].light);
    }
    this->ar_flares.clear();

    // delete exhausts
    for (std::vector<exhaust_t>::iterator it = exhausts.begin(); it != exhausts.end(); it++)
    {
        if (it->smokeNode)
        {
            it->smokeNode->removeAndDestroyAllChildren();
            gEnv->sceneManager->destroySceneNode(it->smokeNode);
        }
        if (it->smoker)
        {
            it->smoker->removeAllAffectors();
            it->smoker->removeAllEmitters();
            gEnv->sceneManager->destroyParticleSystem(it->smoker);
        }
    }

    // delete ar_custom_particles
    for (int i = 0; i < ar_num_custom_particles; i++)
    {
        if (ar_custom_particles[i].snode)
        {
            ar_custom_particles[i].snode->removeAndDestroyAllChildren();
            gEnv->sceneManager->destroySceneNode(ar_custom_particles[i].snode);
        }
        if (ar_custom_particles[i].psys)
        {
            ar_custom_particles[i].psys->removeAllAffectors();
            ar_custom_particles[i].psys->removeAllEmitters();
            gEnv->sceneManager->destroyParticleSystem(ar_custom_particles[i].psys);
        }
    }

    // delete Rails
    for (std::vector<RailGroup*>::iterator it = m_railgroups.begin(); it != m_railgroups.end(); it++)
    {
        delete (*it);
    }

    if (m_net_label_mt)
    {
        m_net_label_mt->setVisible(false);
        delete m_net_label_mt;
        m_net_label_mt = nullptr;
    }

    if (m_intra_point_col_detector)
    {
        delete m_intra_point_col_detector;
        m_intra_point_col_detector = nullptr;
    }

    if (m_inter_point_col_detector)
    {
        delete m_inter_point_col_detector;
        m_inter_point_col_detector = nullptr;
    }

    if (m_transfer_case)
        delete m_transfer_case;

    for (int i = 0; i < m_num_axle_diffs; ++i)
    {
        if (m_axle_diffs[i] != nullptr)
            delete m_axle_diffs[i];
    }

    for (int i = 0; i < m_num_wheel_diffs; ++i)
    {
        if (m_wheel_diffs[i] != nullptr)
            delete m_wheel_diffs[i];
    }

    delete ar_nodes;
    delete ar_beams;
    delete ar_shocks;
    delete ar_rotators;
    delete ar_wings;
}

// This method scales actors. Stresses should *NOT* be scaled, they describe
// the material type and they do not depend on length or scale.
void Actor::ScaleActor(float value)
{
    if (value < 0)
        return;
    ar_scale *= value;
    // scale beams
    for (int i = 0; i < ar_num_beams; i++)
    {
        //ar_beams[i].k *= value;
        ar_beams[i].d *= value;
        ar_beams[i].L *= value;
        ar_beams[i].refL *= value;
    }
    // scale hydros
    for (hydrobeam_t& hbeam: ar_hydros)
    {
        hbeam.hb_ref_length *= value;
        hbeam.hb_speed *= value;
    }
    // scale nodes
    Vector3 refpos = ar_nodes[0].AbsPosition;
    Vector3 relpos = ar_nodes[0].RelPosition;
    for (int i = 1; i < ar_num_nodes; i++)
    {
        ar_initial_node_positions[i] = refpos + (ar_initial_node_positions[i] - refpos) * value;
        ar_nodes[i].AbsPosition = refpos + (ar_nodes[i].AbsPosition - refpos) * value;
        ar_nodes[i].RelPosition = relpos + (ar_nodes[i].RelPosition - relpos) * value;
        ar_nodes[i].Velocity *= value;
        ar_nodes[i].Forces *= value;
        ar_nodes[i].mass *= value;
    }
    updateSlideNodePositions();

    m_gfx_actor->ScaleActor(relpos, value);

}

float Actor::getRotation()
{
    Vector3 dir = getDirection();

    return atan2(dir.dotProduct(Vector3::UNIT_X), dir.dotProduct(-Vector3::UNIT_Z));
}

Vector3 Actor::getDirection()
{
    return ar_main_camera_dir_corr * this->GetCameraDir();
}

Vector3 Actor::getPosition()
{
    return m_avg_node_position; //the position is already in absolute position
}

void Actor::RecalculateNodeMasses(Real total)
{
    //reset
    for (int i = 0; i < ar_num_nodes; i++)
    {
        if (!ar_nodes[i].nd_tyre_node)
        {
            if (!ar_nodes[i].nd_loaded_mass)
            {
                ar_nodes[i].mass = 0;
            }
            else if (!ar_nodes[i].nd_override_mass)
            {
                ar_nodes[i].mass = m_load_mass / (float)m_masscount;
            }
        }
    }
    //average linear density
    Real len = 0.0f;
    for (int i = 0; i < ar_num_beams; i++)
    {
        if (ar_beams[i].bm_type != BEAM_VIRTUAL)
        {
            Real half_newlen = ar_beams[i].L / 2.0;
            if (!ar_beams[i].p1->nd_tyre_node)
                len += half_newlen;
            if (!ar_beams[i].p2->nd_tyre_node)
                len += half_newlen;
        }
    }

    for (int i = 0; i < ar_num_beams; i++)
    {
        if (ar_beams[i].bm_type != BEAM_VIRTUAL)
        {
            Real half_mass = ar_beams[i].L * total / len / 2.0f;
            if (!ar_beams[i].p1->nd_tyre_node)
                ar_beams[i].p1->mass += half_mass;
            if (!ar_beams[i].p2->nd_tyre_node)
                ar_beams[i].p2->mass += half_mass;
        }
    }
    //fix rope masses
    for (std::vector<rope_t>::iterator it = ar_ropes.begin(); it != ar_ropes.end(); it++)
    {
        it->rp_beam->p2->mass = 100.0f;
    }

    // Apply pre-defined cinecam node mass
    for (int i = 0; i < this->ar_num_cinecams; ++i)
    {
        // TODO: this expects all cinecams to be defined in root module (i.e. outside 'section/end_section')
        ar_nodes[ar_cinecam_node[i]].mass = m_definition->root_module->cinecam[i].node_mass;
    }

    //update mass
    for (int i = 0; i < ar_num_nodes; i++)
    {
        if (!ar_nodes[i].nd_tyre_node &&
            !(m_definition->minimass_skip_loaded_nodes && ar_nodes[i].nd_loaded_mass) &&
            ar_nodes[i].mass < ar_minimass[i])
        {
            if (App::diag_truck_mass.GetActive())
            {
                char buf[300];
                snprintf(buf, 300, "Node '%d' mass (%f Kg) is too light. Resetting to 'minimass' (%f Kg)", i, ar_nodes[i].mass, ar_minimass[i]);
                LOG(buf);
            }
            ar_nodes[i].mass = ar_minimass[i];
        }
    }

    m_total_mass = 0;
    for (int i = 0; i < ar_num_nodes; i++)
    {
        if (App::diag_truck_mass.GetActive())
        {
            String msg = "Node " + TOSTRING(i) + " : " + TOSTRING((int)ar_nodes[i].mass) + " kg";
            if (ar_nodes[i].nd_loaded_mass)
            {
                if (ar_nodes[i].nd_override_mass)
                    msg += " (overriden by node mass)";
                else
                    msg += " (normal load node: " + TOSTRING(m_load_mass) + " kg / " + TOSTRING(m_masscount) + " nodes)";
            }
            LOG(msg);
        }
        m_total_mass += ar_nodes[i].mass;
    }
    LOG("TOTAL VEHICLE MASS: " + TOSTRING((int)m_total_mass) +" kg");
}

float Actor::getTotalMass(bool withLocked)
{
    if (!withLocked)
        return m_total_mass; // already computed in RecalculateNodeMasses

    float mass = m_total_mass;

    for (auto actor : m_linked_actors)
    {
        mass += actor->m_total_mass;
    }

    return mass;
}

void Actor::DetermineLinkedActors() //TODO: Refactor this - logic iterating over all actors should be in `ActorManager`! ~ only_a_ptr, 01/2018
{
    m_linked_actors.clear();

    bool found = true;
    std::map<Actor*, bool> lookup_table;
    std::pair<std::map<Actor*, bool>::iterator, bool> ret;

    lookup_table.insert(std::pair<Actor*, bool>(this, false));
    
    auto inter_actor_links = App::GetSimController()->GetBeamFactory()->inter_actor_links; // TODO: Shouldn't this have been a reference?? Also, ugly, see the TODO note above ~ only_a_ptr, 01/2018

    while (found)
    {
        found = false;

        for (std::map<Actor*, bool>::iterator it_beam = lookup_table.begin(); it_beam != lookup_table.end(); ++it_beam)
        {
            if (!it_beam->second)
            {
                auto actor = it_beam->first;
                for (auto it = inter_actor_links.begin(); it != inter_actor_links.end(); it++)
                {
                    auto actor_pair = it->second;
                    if (actor == actor_pair.first || actor == actor_pair.second)
                    {
                        auto other_actor = (actor != actor_pair.first) ? actor_pair.first : actor_pair.second;
                        ret = lookup_table.insert(std::pair<Actor*, bool>(other_actor, false));
                        if (ret.second)
                        {
                            m_linked_actors.push_back(other_actor);
                            found = true;
                        }
                    }
                }
                it_beam->second = true;
            }
        }
    }
}

int Actor::getWheelNodeCount() const
{
    return m_wheel_node_count;
}

void Actor::calcNodeConnectivityGraph()
{
    int i;

    ar_node_to_node_connections.resize(ar_num_nodes, std::vector<int>());
    ar_node_to_beam_connections.resize(ar_num_nodes, std::vector<int>());

    for (i = 0; i < ar_num_beams; i++)
    {
        if (ar_beams[i].p1 != NULL && ar_beams[i].p2 != NULL && ar_beams[i].p1->pos >= 0 && ar_beams[i].p2->pos >= 0)
        {
            ar_node_to_node_connections[ar_beams[i].p1->pos].push_back(ar_beams[i].p2->pos);
            ar_node_to_beam_connections[ar_beams[i].p1->pos].push_back(i);
            ar_node_to_node_connections[ar_beams[i].p2->pos].push_back(ar_beams[i].p1->pos);
            ar_node_to_beam_connections[ar_beams[i].p2->pos].push_back(i);
        }
    }
}

bool Actor::Intersects(Actor* actor, Vector3 offset)
{
    Vector3 bb_min = ar_bounding_box.getMinimum() + offset;
    Vector3 bb_max = ar_bounding_box.getMaximum() + offset;
    AxisAlignedBox bb = AxisAlignedBox(bb_min, bb_max);

    if (!bb.intersects(actor->ar_bounding_box))
        return false;

    // Test own (contactable) beams against others cabs
    for (int i = 0; i < ar_num_beams; i++)
    {
        if (!(ar_beams[i].p1->nd_contacter || ar_beams[i].p1->nd_contactable) ||
            !(ar_beams[i].p2->nd_contacter || ar_beams[i].p2->nd_contactable))
            continue;

        Vector3 origin = ar_beams[i].p1->AbsPosition + offset;
        Vector3 target = ar_beams[i].p2->AbsPosition + offset;

        Ray ray(origin, target - origin);

        for (int j = 0; j < actor->ar_num_collcabs; j++)
        {
            int index = actor->ar_collcabs[j] * 3;
            Vector3 a = actor->ar_nodes[actor->ar_cabs[index + 0]].AbsPosition;
            Vector3 b = actor->ar_nodes[actor->ar_cabs[index + 1]].AbsPosition;
            Vector3 c = actor->ar_nodes[actor->ar_cabs[index + 2]].AbsPosition;

            auto result = Ogre::Math::intersects(ray, a, b, c);
            if (result.first && result.second < 1.0f)
            {
                return true;
            }
        }
    }

    // Test own cabs against others (contactable) beams
    for (int i = 0; i < actor->ar_num_beams; i++)
    {
        if (!(actor->ar_beams[i].p1->nd_contacter || actor->ar_beams[i].p1->nd_contactable) ||
            !(actor->ar_beams[i].p2->nd_contacter || actor->ar_beams[i].p2->nd_contactable))
            continue;

        Vector3 origin = actor->ar_beams[i].p1->AbsPosition;
        Vector3 target = actor->ar_beams[i].p2->AbsPosition;

        Ray ray(origin, target - origin);

        for (int j = 0; j < ar_num_collcabs; j++)
        {
            int index = ar_collcabs[j] * 3;
            Vector3 a = ar_nodes[ar_cabs[index + 0]].AbsPosition + offset;
            Vector3 b = ar_nodes[ar_cabs[index + 1]].AbsPosition + offset;
            Vector3 c = ar_nodes[ar_cabs[index + 2]].AbsPosition + offset;

            auto result = Ogre::Math::intersects(ray, a, b, c);
            if (result.first && result.second < 1.0f)
            {
                return true;
            }
        }
    }

    return false;
}

Vector3 Actor::calculateCollisionOffset(Vector3 direction)
{
    if (direction == Vector3::ZERO)
        return Vector3::ZERO;

    Real max_distance = direction.normalise();

    // collision displacement
    Vector3 collision_offset = Vector3::ZERO;

    while (collision_offset.length() < max_distance)
    {
        Vector3 bb_min = ar_bounding_box.getMinimum() + collision_offset;
        Vector3 bb_max = ar_bounding_box.getMaximum() + collision_offset;
        AxisAlignedBox bb = AxisAlignedBox(bb_min, bb_max);

        bool collision = false;

        for (auto actor : App::GetSimController()->GetActors())
        {
            if (actor == this)
                continue;
            if (!bb.intersects(actor->ar_bounding_box))
                continue;

            // Test own contactables against others cabs
            if (m_intra_point_col_detector)
            {
                for (int i = 0; i < actor->ar_num_collcabs; i++)
                {
                    int tmpv = actor->ar_collcabs[i] * 3;
                    node_t* no = &actor->ar_nodes[actor->ar_cabs[tmpv]];
                    node_t* na = &actor->ar_nodes[actor->ar_cabs[tmpv + 1]];
                    node_t* nb = &actor->ar_nodes[actor->ar_cabs[tmpv + 2]];

                    m_intra_point_col_detector->query(no->AbsPosition - collision_offset,
                        na->AbsPosition - collision_offset,
                        nb->AbsPosition - collision_offset,
                        actor->ar_collision_range * 3.0f);

                    if (collision = !m_intra_point_col_detector->hit_list.empty())
                        break;
                }

                if (collision)
                    break;
            }

            float proximity = std::max(.05f, std::sqrt(std::max(m_min_camera_radius, actor->m_min_camera_radius)) / 50.f);

            // Test proximity of own nodes against others nodes
            for (int i = 0; i < ar_num_nodes; i++)
            {
                if (!ar_nodes[i].nd_contacter && !ar_nodes[i].nd_contactable)
                    continue;

                Vector3 query_position = ar_nodes[i].AbsPosition + collision_offset;

                for (int j = 0; j < actor->ar_num_nodes; j++)
                {
                    if (!actor->ar_nodes[j].nd_contacter && !actor->ar_nodes[j].nd_contactable)
                        continue;

                    if (collision = query_position.squaredDistance(actor->ar_nodes[j].AbsPosition) < proximity)
                        break;
                }

                if (collision)
                    break;
            }

            if (collision)
                break;
        }

        // Test own cabs against others contacters
        if (!collision && m_inter_point_col_detector)
        {
            for (int i = 0; i < ar_num_collcabs; i++)
            {
                int tmpv = ar_collcabs[i] * 3;
                node_t* no = &ar_nodes[ar_cabs[tmpv]];
                node_t* na = &ar_nodes[ar_cabs[tmpv + 1]];
                node_t* nb = &ar_nodes[ar_cabs[tmpv + 2]];

                m_inter_point_col_detector->query(no->AbsPosition + collision_offset,
                    na->AbsPosition + collision_offset,
                    nb->AbsPosition + collision_offset,
                    ar_collision_range * 3.0f);

                if (collision = !m_inter_point_col_detector->hit_list.empty())
                    break;
            }
        }

        // Test beams (between contactable nodes) against cabs
        if (!collision)
        {
            for (auto actor : App::GetSimController()->GetActors())
            {
                if (actor == this)
                    continue;
                if (collision = this->Intersects(actor, collision_offset))
                    break;
            }
        }

        if (!collision)
            break;

        collision_offset += direction * 0.05f;
    }

    return collision_offset;
}

void Actor::resolveCollisions(Vector3 direction)
{
    if (m_intra_point_col_detector)
        m_intra_point_col_detector->UpdateIntraPoint(true);

    if (m_inter_point_col_detector)
        m_inter_point_col_detector->UpdateInterPoint(true);

    Vector3 offset = calculateCollisionOffset(direction);

    if (offset == Vector3::ZERO)
        return;

    // Additional 20 cm safe-guard (horizontally)
    offset += 0.2f * Vector3(offset.x, 0.0f, offset.z).normalisedCopy();

    ResetPosition(ar_nodes[0].AbsPosition.x + offset.x, ar_nodes[0].AbsPosition.z + offset.z, false, this->GetMinHeight() + offset.y);
}

void Actor::resolveCollisions(float max_distance, bool consider_up)
{
    if (m_intra_point_col_detector)
        m_intra_point_col_detector->UpdateIntraPoint(true);

    if (m_inter_point_col_detector)
        m_inter_point_col_detector->UpdateInterPoint(true);

    Vector3 u = Vector3::UNIT_Y;
    Vector3 f = Vector3(getDirection().x, 0.0f, getDirection().z).normalisedCopy();
    Vector3 l = u.crossProduct(f);

    // Calculate an ideal collision avoidance direction (prefer left over right over [front / back / up])
    Vector3 left  = calculateCollisionOffset(+l * max_distance);
    Vector3 right = calculateCollisionOffset(-l * left.length());
    Vector3 lateral = left.length() < right.length() * 1.1f ? left : right;

    Vector3 front = calculateCollisionOffset(+f * lateral.length());
    Vector3 back  = calculateCollisionOffset(-f * front.length());
    Vector3 sagittal = front.length() < back.length() * 1.1f ? front : back;

    Vector3 offset = lateral.length() < sagittal.length() * 1.2f ? lateral : sagittal;

    if (consider_up)
    {
        Vector3 up = calculateCollisionOffset(+u * offset.length());
        if (up.length() * 1.2f < offset.length())
            offset = up;
    }

    if (offset == Vector3::ZERO)
        return;

    // Additional 20 cm safe-guard (horizontally)
    offset += 0.2f * Vector3(offset.x, 0.0f, offset.z).normalisedCopy();

    ResetPosition(ar_nodes[0].AbsPosition.x + offset.x, ar_nodes[0].AbsPosition.z + offset.z, true, this->GetMinHeight() + offset.y);
}

void Actor::calculateAveragePosition()
{
    // calculate average position
    if (ar_custom_camera_node >= 0)
    {
        m_avg_node_position = ar_nodes[ar_custom_camera_node].AbsPosition;
    }
    else if (ar_extern_camera_mode == 1 && ar_num_cinecams > 0)
    {
        // the new (strange) approach: reuse the cinecam node
        m_avg_node_position = ar_nodes[ar_cinecam_node[0]].AbsPosition;
    }
    else if (ar_extern_camera_mode == 2 && ar_extern_camera_node >= 0)
    {
        // the new (strange) approach #2: reuse a specified node
        m_avg_node_position = ar_nodes[ar_extern_camera_node].AbsPosition;
    }
    else
    {
        // the classic approach: average over all nodes and beams
        Vector3 aposition = Vector3::ZERO;
        for (int n = 0; n < ar_num_nodes; n++)
        {
            aposition += ar_nodes[n].AbsPosition;
        }
        m_avg_node_position = aposition / ar_num_nodes;
    }
}

inline void PadBoundingBox(Ogre::AxisAlignedBox& box) // Internal helper
{
    box.setMinimum(box.getMinimum() - BOUNDING_BOX_PADDING);
    box.setMaximum(box.getMaximum() + BOUNDING_BOX_PADDING);
}

void Actor::UpdateBoundingBoxes()
{
    // Reset
    ar_bounding_box = AxisAlignedBox::BOX_NULL;
    ar_predicted_bounding_box = AxisAlignedBox::BOX_NULL;
    for (size_t i = 0; i < ar_collision_bounding_boxes.size(); ++i)
    {
        ar_collision_bounding_boxes[i] = AxisAlignedBox::BOX_NULL;
        ar_predicted_coll_bounding_boxes[i] = AxisAlignedBox::BOX_NULL;
    }

    // Update
    for (int i = 0; i < ar_num_nodes; i++)
    {
        Vector3 vel = ar_nodes[i].Velocity;
        Vector3 pos = ar_nodes[i].AbsPosition;
        int16_t cid = ar_nodes[i].nd_coll_bbox_id;

        ar_bounding_box.merge(pos);                                  // Current box
        ar_predicted_bounding_box.merge(pos);                        // Predicted box (current position)
        ar_predicted_bounding_box.merge(pos + vel);                  // Predicted box (future position)
        if (cid != node_t::INVALID_BBOX)
        {
            ar_collision_bounding_boxes[cid].merge(pos);
            ar_predicted_coll_bounding_boxes[cid].merge(pos);
            ar_predicted_coll_bounding_boxes[cid].merge(pos + vel);
        }
    }

    // Finalize - add padding
    PadBoundingBox(ar_bounding_box);
    PadBoundingBox(ar_predicted_bounding_box);
    for (size_t i = 0; i < ar_collision_bounding_boxes.size(); ++i)
    {
        PadBoundingBox(ar_collision_bounding_boxes[i]);
        PadBoundingBox(ar_predicted_coll_bounding_boxes[i]);
    }
}

void Actor::UpdatePhysicsOrigin()
{
    if (ar_nodes[0].RelPosition.getLengthSQ() > 10000.0)
    {
        Vector3 offset = ar_nodes[0].RelPosition;
        ar_origin += offset;
        for (int i = 0; i < ar_num_nodes; i++)
        {
            ar_nodes[i].RelPosition -= offset;
        }
    }
}

void Actor::ResetAngle(float rot)
{
    // Set origin of rotation to camera node
    Vector3 origin = ar_nodes[ar_main_camera_node_pos].AbsPosition;

    // Set up matrix for yaw rotation
    Matrix3 matrix;
    matrix.FromEulerAnglesXYZ(Radian(0), Radian(-rot + m_spawn_rotation), Radian(0));

    for (int i = 0; i < ar_num_nodes; i++)
    {
        // Move node back to origin, apply rotation matrix, and move node back
        ar_nodes[i].AbsPosition -= origin;
        ar_nodes[i].AbsPosition = matrix * ar_nodes[i].AbsPosition;
        ar_nodes[i].AbsPosition += origin;
        ar_nodes[i].RelPosition = ar_nodes[i].AbsPosition - this->ar_origin;
    }

    this->UpdateBoundingBoxes();
    calculateAveragePosition();
}

void Actor::UpdateInitPosition()
{
    for (int i = 0; i < ar_num_nodes; i++)
    {
        ar_initial_node_positions[i] = ar_nodes[i].AbsPosition;
    }
}

void Actor::ResetPosition(float px, float pz, bool setInitPosition, float miny)
{
    // horizontal displacement
    Vector3 offset = Vector3(px, ar_nodes[0].AbsPosition.y, pz) - ar_nodes[0].AbsPosition;
    for (int i = 0; i < ar_num_nodes; i++)
    {
        ar_nodes[i].AbsPosition += offset;
        ar_nodes[i].RelPosition = ar_nodes[i].AbsPosition - ar_origin;
    }

    // vertical displacement
    float vertical_offset = miny - this->GetMinHeight();
    if (App::GetSimTerrain()->getWater())
    {
        vertical_offset += std::max(0.0f, App::GetSimTerrain()->getWater()->GetStaticWaterHeight() - miny);
    }
    for (int i = 1; i < ar_num_nodes; i++)
    {
        if (ar_nodes[i].nd_no_ground_contact)
            continue;
        float terrainHeight = App::GetSimTerrain()->GetHeightAt(ar_nodes[i].AbsPosition.x, ar_nodes[i].AbsPosition.z);
        vertical_offset += std::max(0.0f, terrainHeight - (ar_nodes[i].AbsPosition.y + vertical_offset));
    }
    for (int i = 0; i < ar_num_nodes; i++)
    {
        ar_nodes[i].AbsPosition.y += vertical_offset;
        ar_nodes[i].RelPosition = ar_nodes[i].AbsPosition - ar_origin;
    }

    // mesh displacement
    float mesh_offset = 0.0f;
    for (int i = 0; i < ar_num_nodes; i++)
    {
        if (mesh_offset >= 1.0f)
            break;
        if (ar_nodes[i].nd_no_ground_contact)
            continue;
        float offset = mesh_offset;
        while (offset < 1.0f)
        {
            Vector3 query = ar_nodes[i].AbsPosition + Vector3(0.0f, offset, 0.0f);
            if (!gEnv->collisions->collisionCorrect(&query, false))
            {
                mesh_offset = offset;
                break;
            }
            offset += 0.001f;
        }
    }
    for (int i = 0; i < ar_num_nodes; i++)
    {
        ar_nodes[i].AbsPosition.y += mesh_offset;
        ar_nodes[i].RelPosition = ar_nodes[i].AbsPosition - ar_origin;
    }

    ResetPosition(Vector3::ZERO, setInitPosition);
}

void Actor::ResetPosition(Vector3 translation, bool setInitPosition)
{
    // total displacement
    if (translation != Vector3::ZERO)
    {
        Vector3 offset = translation - ar_nodes[0].AbsPosition;
        for (int i = 0; i < ar_num_nodes; i++)
        {
            ar_nodes[i].AbsPosition += offset;
            ar_nodes[i].RelPosition = ar_nodes[i].AbsPosition - ar_origin;
        }
    }

    if (setInitPosition)
    {
        for (int i = 0; i < ar_num_nodes; i++)
        {
            ar_initial_node_positions[i] = ar_nodes[i].AbsPosition;
        }
    }

    this->UpdateBoundingBoxes();
    calculateAveragePosition();
}

void Actor::HandleMouseMove(int node, Vector3 pos, float force)
{
    m_mouse_grab_node = node;
    m_mouse_grab_move_force = force * std::pow(m_total_mass / 3000.0f, 0.75f);
    m_mouse_grab_pos = pos;
}

void Actor::ToggleWheelDiffMode()
{
    for (int i = 0; i < m_num_wheel_diffs; ++i)
    {
        m_wheel_diffs[i]->ToggleDifferentialMode();
    }
}

void Actor::ToggleAxleDiffMode()
{
    for (int i = 0; i < m_num_axle_diffs; ++i)
    {
        m_axle_diffs[i]->ToggleDifferentialMode();
    }
}

void Actor::DisplayAxleDiffMode()
{
    if (m_num_axle_diffs == 0)
    {
        App::GetConsole()->putMessage(Console::CONSOLE_MSGTYPE_INFO, Console::CONSOLE_SYSTEM_NOTICE,
                _L("No inter-axle differential installed on current vehicle!"), "warning.png", 3000);
    }
    else
    {
        String message = "";
        for (int i = 0; i < m_num_axle_diffs; ++i)
        {
            if (m_axle_diffs[i])
            {
                int a1 = m_axle_diffs[i]->di_idx_1 + 1;
                int a2 = m_axle_diffs[i]->di_idx_2 + 1;
                message += _L("Axle ") + TOSTRING(a1) + " <--> " + _L("Axle ") + TOSTRING(a2) + ": ";
                message += m_axle_diffs[i]->GetDifferentialTypeName();
                message += "\n";
            }
        }
        App::GetConsole()->putMessage(Console::CONSOLE_MSGTYPE_INFO, Console::CONSOLE_SYSTEM_NOTICE,
                "Inter-axle differentials:\n" + message, "cog.png", 3000);
    }
}

void Actor::DisplayWheelDiffMode()
{
    if (m_num_wheel_diffs == 0)
    {
        App::GetConsole()->putMessage(Console::CONSOLE_MSGTYPE_INFO, Console::CONSOLE_SYSTEM_NOTICE,
                _L("No inter-wheel differential installed on current vehicle!"), "warning.png", 3000);
    }
    else
    {
        String message = "";
        for (int i = 0; i < m_num_wheel_diffs; ++i)
        {
            if (m_wheel_diffs[i])
            {
                message += _L("Axle ") + TOSTRING(i + 1) + ": ";
                message += m_wheel_diffs[i]->GetDifferentialTypeName();
                message += "\n";
            }
        }
        App::GetConsole()->putMessage(Console::CONSOLE_MSGTYPE_INFO, Console::CONSOLE_SYSTEM_NOTICE,
                "Inter-wheel differentials:\n" + message, "cog.png", 3000);
    }
}

void Actor::DisplayTransferCaseMode()
{
    if (m_transfer_case)
    {
        App::GetConsole()->putMessage(Console::CONSOLE_MSGTYPE_INFO, Console::CONSOLE_SYSTEM_NOTICE,
                _L("Transfercase switched to: ") + this->GetTransferCaseName(), "cog.png", 3000);
    }
    else
    {
        App::GetConsole()->putMessage(Console::CONSOLE_MSGTYPE_INFO, Console::CONSOLE_SYSTEM_NOTICE,
                _L("No transfercase installed on current vehicle!"), "warning.png", 3000);
    }
}

void Actor::ToggleTransferCaseMode()
{
    if (!ar_engine || !m_transfer_case || m_transfer_case->tr_ax_2 < 0 || !m_transfer_case->tr_2wd)
        return;

    if (m_transfer_case->tr_4wd_mode && !m_transfer_case->tr_2wd_lo)
    {
        for (int i = 0; i < m_transfer_case->tr_gear_ratios.size(); i++)
        {
            this->ToggleTransferCaseGearRatio();
            if (m_transfer_case->tr_gear_ratios[0] == 1.0f)
                break;
        }
    }

    m_transfer_case->tr_4wd_mode = !m_transfer_case->tr_4wd_mode;

    if (m_transfer_case->tr_4wd_mode)
    {
        ar_wheels[m_wheel_diffs[m_transfer_case->tr_ax_2]->di_idx_1].wh_propulsed = true;
        ar_wheels[m_wheel_diffs[m_transfer_case->tr_ax_2]->di_idx_2].wh_propulsed = true;
        m_num_proped_wheels += 2;
    }
    else
    {
        ar_wheels[m_wheel_diffs[m_transfer_case->tr_ax_2]->di_idx_1].wh_propulsed = false;
        ar_wheels[m_wheel_diffs[m_transfer_case->tr_ax_2]->di_idx_2].wh_propulsed = false;
        m_num_proped_wheels -= 2;
    }
}

void Actor::ToggleTransferCaseGearRatio()
{
    if (!ar_engine || !m_transfer_case || m_transfer_case->tr_gear_ratios.size() < 2)
        return;

    if (m_transfer_case->tr_4wd_mode || m_transfer_case->tr_2wd_lo)
    {
        auto gear_ratios = &m_transfer_case->tr_gear_ratios;
        std::rotate(gear_ratios->begin(), gear_ratios->begin() + 1, gear_ratios->end());

        ar_engine->SetTCaseRatio(m_transfer_case->tr_gear_ratios[0]);
    }
}

String Actor::GetTransferCaseName()
{
    String name = "";
    if (m_transfer_case)
    {
        name += m_transfer_case->tr_4wd_mode ? "4WD " : "2WD ";
        if (m_transfer_case->tr_gear_ratios[0] > 1.0f)
            name += "Lo (" + TOSTRING(m_transfer_case->tr_gear_ratios[0]) + ":1)";
        else
            name += "Hi";
    }
    return name;
}

irr::core::vector3df Actor::GetRotationCenter()
{
    Vector3 sum = Vector3::ZERO;
    std::vector<Vector3> positions;
    for (int i = 0; i < ar_num_nodes; i++)
    {
        Vector3 pos = ar_nodes[i].AbsPosition;
        const auto it = std::find_if(positions.begin(), positions.end(),
            [pos](const Vector3 ref) { return pos.positionEquals(ref, 0.01f); });
        if (it == positions.end())
        {
            sum += pos;
            positions.push_back(pos);
        }
    }
    return sum / positions.size();
}

float Actor::GetMinHeight(bool skip_virtual_nodes)
{
    float height = std::numeric_limits<float>::max(); 
    for (int i = 0; i < ar_num_nodes; i++)
    {
        if (!skip_virtual_nodes || !ar_nodes[i].nd_no_ground_contact)
        {
            height = std::min(ar_nodes[i].AbsPosition.y, height);
        }
    }
    return (!skip_virtual_nodes || height < std::numeric_limits<float>::max()) ? height : GetMinHeight(false);
}

float Actor::GetMaxHeight(bool skip_virtual_nodes)
{
    float height = std::numeric_limits<float>::min(); 
    for (int i = 0; i < ar_num_nodes; i++)
    {
        if (!skip_virtual_nodes || !ar_nodes[i].nd_no_ground_contact)
        {
            height = std::max(height, ar_nodes[i].AbsPosition.y);
        }
    }
    return (!skip_virtual_nodes || height > std::numeric_limits<float>::min()) ? height : GetMaxHeight(false);
}

float Actor::GetHeightAboveGround(bool skip_virtual_nodes)
{
    float agl = std::numeric_limits<float>::max(); 
    for (int i = 0; i < ar_num_nodes; i++)
    {
        if (!skip_virtual_nodes || !ar_nodes[i].nd_no_ground_contact)
        {
            Vector3 pos = ar_nodes[i].AbsPosition;
            agl = std::min(pos.y - gEnv->collisions->getSurfaceHeight(pos.x, pos.z), agl);
        }
    }
    return (!skip_virtual_nodes || agl < std::numeric_limits<float>::max()) ? agl : GetHeightAboveGround(false);
}

float Actor::GetHeightAboveGroundBelow(float height, bool skip_virtual_nodes)
{
    float agl = std::numeric_limits<float>::max(); 
    for (int i = 0; i < ar_num_nodes; i++)
    {
        if (!skip_virtual_nodes || !ar_nodes[i].nd_no_ground_contact)
        {
            Vector3 pos = ar_nodes[i].AbsPosition;
            agl = std::min(pos.y - gEnv->collisions->getSurfaceHeightBelow(pos.x, pos.z, height), agl);
        }
    }
    return (!skip_virtual_nodes || agl < std::numeric_limits<float>::max()) ? agl : GetHeightAboveGroundBelow(height, false);
}

void Actor::SoftReset()
{
    TRIGGER_EVENT(SE_TRUCK_RESET, ar_instance_id);

    float agl = this->GetHeightAboveGroundBelow(this->GetMaxHeight(true), true);

    if (App::GetSimTerrain()->getWater())
    {
        agl = std::min(this->GetMinHeight(true) - App::GetSimTerrain()->getWater()->GetStaticWaterHeight(), agl);
    }

    if (agl < 0.0f)
    {
        Vector3 translation = -agl * Vector3::UNIT_Y;
        this->ResetPosition(ar_nodes[0].AbsPosition + translation, false);
        for (auto actor : m_linked_actors)
        {
            actor->ResetPosition(actor->ar_nodes[0].AbsPosition + translation, false);
        }
    }

    m_ongoing_reset = true;
}

void Actor::SyncReset(bool reset_position)
{
    TRIGGER_EVENT(SE_TRUCK_RESET, ar_instance_id);

    m_reset_timer.reset();

    m_camera_local_gforces_cur = Vector3::ZERO;
    m_camera_local_gforces_max = Vector3::ZERO;

    ar_hydro_dir_state = 0.0;
    ar_hydro_aileron_state = 0.0;
    ar_hydro_rudder_state = 0.0;
    ar_hydro_elevator_state = 0.0;
    ar_hydro_dir_wheel_display = 0.0;
	
    ar_fusedrag = Vector3::ZERO;
    m_blink_type = BLINK_NONE;
    ar_parking_brake = false;
    ar_trailer_parking_brake = false;
    ar_avg_wheel_speed = 0.0f;
    ar_wheel_speed = 0.0f;
    ar_wheel_spin = 0.0f;
    cc_mode = false;

    ar_origin = Vector3::ZERO;
    float cur_rot = getRotation();
    Vector3 cur_position = ar_nodes[0].AbsPosition;

    for (int i = 0; i < ar_num_nodes; i++)
    {
        ar_nodes[i].AbsPosition = ar_initial_node_positions[i];
        ar_nodes[i].RelPosition = ar_nodes[i].AbsPosition - ar_origin;
        ar_nodes[i].Velocity = Vector3::ZERO;
        ar_nodes[i].Forces = Vector3::ZERO;
    }

    for (int i = 0; i < ar_num_beams; i++)
    {
        ar_beams[i].maxposstress    = ar_beams[i].default_beam_deform;
        ar_beams[i].maxnegstress    = -ar_beams[i].default_beam_deform;
        ar_beams[i].minmaxposnegstress = ar_beams[i].default_beam_deform;
        ar_beams[i].strength        = ar_beams[i].initial_beam_strength;
        ar_beams[i].L               = ar_beams[i].refL;
        ar_beams[i].stress          = 0.0;
        ar_beams[i].bm_broken       = false;
        ar_beams[i].bm_disabled     = false;
    }

    this->ApplyNodeBeamScales();

    this->DisjoinInterActorBeams();

    for (auto& h : ar_hooks)
    {
        h.hk_locked = UNLOCKED;
        h.hk_lock_node = nullptr;
        h.hk_locked_actor = nullptr;
        h.hk_beam->p2 = &ar_nodes[0];
        h.hk_beam->bm_disabled = true;
        h.hk_beam->bm_inter_actor = false;
        h.hk_beam->L = (ar_nodes[0].AbsPosition - h.hk_hook_node->AbsPosition).length();
        this->RemoveInterActorBeam(h.hk_beam);
    }

    for (auto& r : ar_ropes)
    {
        r.rp_locked = UNLOCKED;
        r.rp_locked_ropable = nullptr;
        r.rp_locked_actor = nullptr;
        this->RemoveInterActorBeam(r.rp_beam);
    }

    for (auto& t : ar_ties)
    {
        t.ti_tied = false;
        t.ti_tying = false;
        t.ti_locked_actor = nullptr;
        t.ti_locked_ropable = nullptr;
        t.ti_beam->p2 = &ar_nodes[0];
        t.ti_beam->bm_disabled = true;
        t.ti_beam->bm_inter_actor = false;
        this->RemoveInterActorBeam(t.ti_beam);
    }

    for (auto& r : ar_ropables)
    {
        r.attached_ties = 0;
        r.attached_ropes = 0;
    }

    for (int i = 0; i < ar_num_wheels; i++)
    {
        ar_wheels[i].wh_speed = 0.0;
        ar_wheels[i].wh_torque = 0.0;
        ar_wheels[i].wh_avg_speed = 0.0;
        ar_wheels[i].wh_is_detached = false;
    }

    if (ar_engine)
    {
        if (App::sim_spawn_running.GetActive())
        {
            ar_engine->StartEngine();
        }
        ar_engine->SetWheelSpin(0.0f);
    }

    int num_axle_diffs = (m_transfer_case && m_transfer_case->tr_4wd_mode) ? m_num_axle_diffs + 1 : m_num_axle_diffs;
    for (int i = 0; i < num_axle_diffs; i++)
        m_axle_diffs[i]->di_delta_rotation = 0.0f;
    for (int i = 0; i < m_num_wheel_diffs; i++)
        m_wheel_diffs[i]->di_delta_rotation = 0.0f;
    for (int i = 0; i < ar_num_aeroengines; i++)
        ar_aeroengines[i]->reset();
    for (int i = 0; i < ar_num_screwprops; i++)
        ar_screwprops[i]->reset();
    for (int i = 0; i < ar_num_rotators; i++)
        ar_rotators[i].angle = 0.0;
    for (int i = 0; i < ar_num_wings; i++)
        ar_wings[i].fa->broken = false;
    if (ar_autopilot)
        this->ar_autopilot->reset();
    if (m_buoyance)
        m_buoyance->sink = false;

    for (hydrobeam_t& hydrobeam: ar_hydros)
    {
        hydrobeam.hb_inertia.ResetCmdKeyDelay();
    }

    this->GetGfxActor()->ResetFlexbodies();

    // reset on spot with backspace
    if (!reset_position)
    {
        this->ResetAngle(cur_rot);
        this->ResetPosition(cur_position, false);
        float agl = this->GetHeightAboveGroundBelow(this->GetMaxHeight(true), true);
        if (App::GetSimTerrain()->getWater())
        {
            agl = std::min(this->GetMinHeight(true) - App::GetSimTerrain()->getWater()->GetStaticWaterHeight(), agl);
        }
        if (agl < 0.0f)
        {
            this->ResetPosition(ar_nodes[0].AbsPosition - agl * Vector3::UNIT_Y, false);
        }
    }
    else
    {
        this->UpdateBoundingBoxes();
        this->calculateAveragePosition();
    }

    for (int i = 0; i < MAX_COMMANDS; i++)
    {
        ar_command_key[i].commandValue = 0.0;
        ar_command_key[i].triggerInputValue = 0.0f;
        ar_command_key[i].playerInputValue = 0.0f;
        for (auto& b : ar_command_key[i].beams)
        {
            b.cmb_state->auto_moving_mode = 0;
            b.cmb_state->pressed_center_mode = false;
        }
    }

    this->resetSlideNodes();
    if (m_slidenodes_locked)
    {
        this->ToggleSlideNodeLock();
    }

    m_ongoing_reset = true;
}

void Actor::ApplyNodeBeamScales()
{
    for (int i = 0; i < ar_num_nodes; i++)
    {
        ar_nodes[i].mass = ar_initial_node_masses[i] * ar_nb_mass_scale;
    }

    m_total_mass = ar_initial_total_mass * ar_nb_mass_scale;

    for (int i = 0; i < ar_num_beams; i++)
    {
        if ((ar_beams[i].p1->nd_tyre_node || ar_beams[i].p1->nd_rim_node) ||
            (ar_beams[i].p2->nd_tyre_node || ar_beams[i].p2->nd_rim_node))
        {
            ar_beams[i].k = ar_initial_beam_defaults[i].first * ar_nb_wheels_scale.first;
            ar_beams[i].d = ar_initial_beam_defaults[i].second * ar_nb_wheels_scale.second;
        }
        else if (ar_beams[i].bounded == SHOCK1 || ar_beams[i].bounded == SHOCK2 || ar_beams[i].bounded == SHOCK3)
        {
            ar_beams[i].k = ar_initial_beam_defaults[i].first * ar_nb_shocks_scale.first;;
            ar_beams[i].d = ar_initial_beam_defaults[i].second * ar_nb_shocks_scale.second;
        }
        else
        {
            ar_beams[i].k = ar_initial_beam_defaults[i].first * ar_nb_beams_scale.first;
            ar_beams[i].d = ar_initial_beam_defaults[i].second * ar_nb_beams_scale.second;
        }
    }
}

bool Actor::ReplayStep()
{
    if (!ar_replay_mode || !m_replay_handler || !m_replay_handler->isValid())
        return false;

    // no replay update needed if position was not changed
    if (ar_replay_pos != m_replay_pos_prev)
    {
        unsigned long time = 0;

        node_simple_t* nbuff = (node_simple_t *)m_replay_handler->getReadBuffer(ar_replay_pos, 0, time);
        if (nbuff)
        {
            for (int i = 0; i < ar_num_nodes; i++)
            {
                ar_nodes[i].AbsPosition = nbuff[i].position;
                ar_nodes[i].RelPosition = nbuff[i].position - ar_origin;

                ar_nodes[i].Velocity = nbuff[i].velocity;
                ar_nodes[i].Forces = Vector3::ZERO;
            }

            updateSlideNodePositions();
            this->UpdateBoundingBoxes();
            calculateAveragePosition();
        }

        beam_simple_t* bbuff = (beam_simple_t *)m_replay_handler->getReadBuffer(ar_replay_pos, 1, time);
        if (bbuff)
        {
            for (int i = 0; i < ar_num_beams; i++)
            {
                ar_beams[i].bm_broken = bbuff[i].broken;
                ar_beams[i].bm_disabled = bbuff[i].disabled;
            }
        }
        m_replay_pos_prev = ar_replay_pos;
    }

    return true;
}

void Actor::ForceFeedbackStep(int steps)
{
    m_force_sensors.out_body_forces = m_force_sensors.accu_body_forces / steps;
    if (!ar_hydros.empty()) // Vehicle has hydros?
    {
        m_force_sensors.out_hydros_forces = (m_force_sensors.accu_hydros_forces / steps) / ar_hydros.size();    
    }
}

void Actor::HandleAngelScriptEvents(float dt)
{
#ifdef USE_ANGELSCRIPT

    // TODO: restore events SE_TRUCK_LOCKED and SE_TRUCK_UNLOCKED
    if (m_water_contact && !m_water_contact_old)
    {
        m_water_contact_old = m_water_contact;
        ScriptEngine::getSingleton().triggerEvent(SE_TRUCK_TOUCHED_WATER, ar_instance_id);
    }
#endif // USE_ANGELSCRIPT
}

void Actor::SearchBeamDefaults()
{
    SyncReset(true);

    auto old_beams_scale = ar_nb_beams_scale;
    auto old_shocks_scale = ar_nb_shocks_scale;
    auto old_wheels_scale = ar_nb_wheels_scale;

    if (ar_nb_initialized)
    {
        ar_nb_beams_scale.first   = Math::RangeRandom(ar_nb_beams_k_interval.first,  ar_nb_beams_k_interval.second);
        ar_nb_beams_scale.second  = Math::RangeRandom(ar_nb_beams_d_interval.first,  ar_nb_beams_d_interval.second);
        ar_nb_shocks_scale.first  = Math::RangeRandom(ar_nb_shocks_k_interval.first, ar_nb_shocks_k_interval.second);
        ar_nb_shocks_scale.second = Math::RangeRandom(ar_nb_shocks_d_interval.first, ar_nb_shocks_d_interval.second);
        ar_nb_wheels_scale.first  = Math::RangeRandom(ar_nb_wheels_k_interval.first, ar_nb_wheels_k_interval.second);
        ar_nb_wheels_scale.second = Math::RangeRandom(ar_nb_wheels_d_interval.first, ar_nb_wheels_d_interval.second);
    }
    else
    {
        ar_nb_beams_scale.first   = Math::Clamp(1.0f, ar_nb_beams_k_interval.first,  ar_nb_beams_k_interval.second);
        ar_nb_beams_scale.second  = Math::Clamp(1.0f, ar_nb_beams_d_interval.first,  ar_nb_beams_d_interval.second);
        ar_nb_shocks_scale.first  = Math::Clamp(1.0f, ar_nb_shocks_k_interval.first, ar_nb_shocks_k_interval.second);
        ar_nb_shocks_scale.second = Math::Clamp(1.0f, ar_nb_shocks_d_interval.first, ar_nb_shocks_d_interval.second);
        ar_nb_wheels_scale.first  = Math::Clamp(1.0f, ar_nb_wheels_k_interval.first, ar_nb_wheels_k_interval.second);
        ar_nb_wheels_scale.second = Math::Clamp(1.0f, ar_nb_wheels_d_interval.first, ar_nb_wheels_d_interval.second);
        ar_nb_reference = std::vector<float>(ar_nb_reference.size(), std::numeric_limits<float>::max());
        ar_nb_optimum   = std::vector<float>(ar_nb_reference.size(), std::numeric_limits<float>::max());
    }

    this->ApplyNodeBeamScales();

    m_ongoing_reset = false;
    this->CalcForcesEulerPrepare(true);
    for (int i = 0; i < ar_nb_skip_steps; i++)
    {
        this->CalcForcesEulerCompute(i == 0, ar_nb_skip_steps);
        if (m_ongoing_reset)
            break;
    }
    m_ongoing_reset = true;

    float sum_movement = 0.0f;
    float movement = 0.0f;
    float sum_velocity = 0.0f;
    float velocity = 0.0f;
    float sum_stress = 0.0f;
    float stress = 0.0f;
    int sum_broken = 0;
    for (int k = 0; k < ar_nb_measure_steps; k++)
    {
        this->CalcForcesEulerCompute(false, ar_nb_measure_steps);
        for (int i = 0; i < ar_num_nodes; i++)
        {
            float v = ar_nodes[i].Velocity.length();
            sum_movement += v / (float)ar_nb_measure_steps;
            movement = std::max(movement, v);
        }
        for (int i = 0; i < ar_num_beams; i++)
        {
            Vector3 dis = (ar_beams[i].p1->RelPosition - ar_beams[i].p2->RelPosition).normalisedCopy();
            float v = (ar_beams[i].p1->Velocity - ar_beams[i].p2->Velocity).dotProduct(dis);
            sum_velocity += std::abs(v) / (float)ar_nb_measure_steps;
            velocity = std::max(velocity, std::abs(v));
            sum_stress += std::abs(ar_beams[i].stress) / (float)ar_nb_measure_steps;
            stress = std::max(stress, std::abs(ar_beams[i].stress));
            if (k == 0 && ar_beams[i].bm_broken)
            {
                sum_broken++;
            }
        }
        if (sum_broken > ar_nb_reference[6] ||
                stress > ar_nb_reference[0] ||     velocity > ar_nb_reference[2] ||     movement > ar_nb_optimum[4] ||
            sum_stress > ar_nb_reference[1] || sum_velocity > ar_nb_reference[3] || sum_movement > ar_nb_optimum[5] * 2.f)
        {
            ar_nb_beams_scale  = old_beams_scale;
            ar_nb_shocks_scale = old_shocks_scale;
            ar_nb_wheels_scale = old_wheels_scale;
            SyncReset(true);
            return;
        }
    }
    SyncReset(true);

    ar_nb_optimum = {stress, sum_stress, velocity, sum_velocity, movement, sum_movement, (float)sum_broken};
    if (!ar_nb_initialized)
    {
        ar_nb_reference = ar_nb_optimum;
    }
    ar_nb_initialized = true;
}

void Actor::HandleInputEvents(float dt)
{
    if (!m_ongoing_reset)
        return;

    if (m_anglesnap_request > 0)
    {
        float rotation = Radian(getRotation()).valueDegrees();
        float target_rotation = std::round(rotation / m_anglesnap_request) * m_anglesnap_request;
        m_rotation_request = -Degree(target_rotation - rotation).valueRadians();
	m_rotation_request_center = GetRotationCenter();
        m_anglesnap_request = 0;
    }

    if (m_rotation_request != 0.0f)
    {
        Quaternion rot = Quaternion(Radian(m_rotation_request), Vector3::UNIT_Y);

        for (int i = 0; i < ar_num_nodes; i++)
        {
            ar_nodes[i].AbsPosition -= m_rotation_request_center;
            ar_nodes[i].AbsPosition = rot * ar_nodes[i].AbsPosition;
            ar_nodes[i].AbsPosition += m_rotation_request_center;
            ar_nodes[i].RelPosition = ar_nodes[i].AbsPosition - ar_origin;
            ar_nodes[i].Velocity = rot * ar_nodes[i].Velocity;
            ar_nodes[i].Forces = rot * ar_nodes[i].Forces;
        }

        m_rotation_request = 0.0f;
        this->UpdateBoundingBoxes();
        calculateAveragePosition();
    }

    if (m_translation_request != Vector3::ZERO)
    {
        for (int i = 0; i < ar_num_nodes; i++)
        {
            ar_nodes[i].AbsPosition += m_translation_request;
            ar_nodes[i].RelPosition = ar_nodes[i].AbsPosition - ar_origin;
        }

        m_translation_request = Vector3::ZERO;
        UpdateBoundingBoxes();
        calculateAveragePosition();
    }
}

void Actor::CalcAnimators(const int flag_state, float& cstate, int& div, Real timer, const float lower_limit, const float upper_limit, const float option3)
{
    // ## DEV NOTE:
    // ## Until 06/2018, this function was used for both animator-beams (physics, part of softbody) and animated props (visual-only).
    // ## Animated props are now done by `GfxActor::CalcPropAnimation()`   ~ only_a_ptr

    // -- WRITES --
    // ANIM_FLAG_TORQUE: ar_anim_previous_crank
    // sequential shifting: m_previous_gear, ar_anim_shift_timer

    Real dt = timer;

    //boat rudder - read only
    if (flag_state & ANIM_FLAG_BRUDDER)
    {
        int spi;
        float ctmp = 0.0f;
        for (spi = 0; spi < ar_num_screwprops; spi++)
            if (ar_screwprops[spi])
                ctmp += ar_screwprops[spi]->getRudder();

        if (spi > 0)
            ctmp = ctmp / spi;
        cstate = ctmp;
        div++;
    }

    //boat throttle - read only
    if (flag_state & ANIM_FLAG_BTHROTTLE)
    {
        int spi;
        float ctmp = 0.0f;
        for (spi = 0; spi < ar_num_screwprops; spi++)
            if (ar_screwprops[spi])
                ctmp += ar_screwprops[spi]->getThrottle();

        if (spi > 0)
            ctmp = ctmp / spi;
        cstate = ctmp;
        div++;
    }

    //differential lock status - read only
    if (flag_state & ANIM_FLAG_DIFFLOCK)
    {
        if (m_num_wheel_diffs && m_wheel_diffs[0])
        {
            String name = m_wheel_diffs[0]->GetDifferentialTypeName();
            if (name == "Open")
                cstate = 0.0f;
            if (name == "Split")
                cstate = 0.5f;
            if (name == "Locked")
                cstate = 1.0f;
        }
        else // no axles/diffs avail, mode is split by default
            cstate = 0.5f;

        div++;
    }

    //heading - read only
    if (flag_state & ANIM_FLAG_HEADING)
    {
        float heading = getRotation();
        // rad2deg limitedrange  -1 to +1
        cstate = (heading * 57.29578f) / 360.0f;
        div++;
    }

    //torque - WRITES 
    if (ar_engine && flag_state & ANIM_FLAG_TORQUE)
    {
        float torque = ar_engine->GetCrankFactor();
        if (torque <= 0.0f)
            torque = 0.0f;
        if (torque >= ar_anim_previous_crank)
            cstate -= torque / 10.0f;
        else
            cstate = 0.0f;

        if (cstate <= -1.0f)
            cstate = -1.0f;
        ar_anim_previous_crank = torque;
        div++;
    }

    //shifterseq, to amimate sequentiell shifting
    if (ar_engine && (flag_state & ANIM_FLAG_SHIFTER) && option3 == 3.0f)
    {
        // opt1 &opt2 = 0   this is a shifter
        if (!lower_limit && !upper_limit)
        {
            int shifter = ar_engine->GetGear();
            if (shifter > m_previous_gear)
            {
                cstate = 1.0f;
                ar_anim_shift_timer = 0.2f;
            }
            if (shifter < m_previous_gear)
            {
                cstate = -1.0f;
                ar_anim_shift_timer = -0.2f;
            }
            m_previous_gear = shifter;

            if (ar_anim_shift_timer > 0.0f)
            {
                cstate = 1.0f;
                ar_anim_shift_timer -= dt;
                if (ar_anim_shift_timer < 0.0f)
                    ar_anim_shift_timer = 0.0f;
            }
            if (ar_anim_shift_timer < 0.0f)
            {
                cstate = -1.0f;
                ar_anim_shift_timer += dt;
                if (ar_anim_shift_timer > 0.0f)
                    ar_anim_shift_timer = 0.0f;
            }
        }
        else
        {
            // check if lower_limit is a valid to get commandvalue, then get commandvalue
            if (lower_limit >= 1.0f && lower_limit <= 48.0)
                if (ar_command_key[int(lower_limit)].commandValue > 0)
                    cstate += 1.0f;
            // check if upper_limit is a valid to get commandvalue, then get commandvalue
            if (upper_limit >= 1.0f && upper_limit <= 48.0)
                if (ar_command_key[int(upper_limit)].commandValue > 0)
                    cstate -= 1.0f;
        }

        div++;
    }

    //shifterman1, left/right
    if (ar_engine && (flag_state & ANIM_FLAG_SHIFTER) && option3 == 1.0f)
    {
        int shifter = ar_engine->GetGear();
        if (!shifter)
        {
            cstate = -0.5f;
        }
        else if (shifter < 0)
        {
            cstate = 1.0f;
        }
        else
        {
            cstate -= int((shifter - 1.0) / 2.0);
        }
        div++;
    }

    //shifterman2, up/down
    if (ar_engine && (flag_state & ANIM_FLAG_SHIFTER) && option3 == 2.0f)
    {
        int shifter = ar_engine->GetGear();
        cstate = 0.5f;
        if (shifter < 0)
        {
            cstate = 1.0f;
        }
        if (shifter > 0)
        {
            cstate = shifter % 2;
        }
        div++;
    }

    //shifterlinear, to amimate cockpit gearselect gauge and autotransmission stick
    if (ar_engine && (flag_state & ANIM_FLAG_SHIFTER) && option3 == 4.0f)
    {
        int shifter = ar_engine->GetGear();
        int numgears = ar_engine->getNumGears();
        cstate -= (shifter + 2.0) / (numgears + 2.0);
        div++;
    }

    //parking brake
    if (flag_state & ANIM_FLAG_PBRAKE)
    {
        float pbrake = ar_parking_brake;
        cstate -= pbrake;
        div++;
    }

    //speedo ( scales with speedomax )
    if (flag_state & ANIM_FLAG_SPEEDO)
    {
        float speedo = ar_wheel_speed / ar_speedo_max_kph;
        cstate -= speedo * 3.0f;
        div++;
    }

    //engine tacho ( scales with maxrpm, default is 3500 )
    if (ar_engine && flag_state & ANIM_FLAG_TACHO)
    {
        float tacho = ar_engine->GetEngineRpm() / ar_engine->getMaxRPM();
        cstate -= tacho;
        div++;
    }

    //turbo
    if (ar_engine && flag_state & ANIM_FLAG_TURBO)
    {
        float turbo = ar_engine->GetTurboPsi() * 3.34;
        cstate -= turbo / 67.0f;
        div++;
    }

    //brake
    if (flag_state & ANIM_FLAG_BRAKE)
    {
        cstate -= ar_brake;
        div++;
    }

    //accelerator
    if (ar_engine && flag_state & ANIM_FLAG_ACCEL)
    {
        float accel = ar_engine->GetAcceleration();
        cstate -= accel + 0.06f;
        //( small correction, get acc is nver smaller then 0.06.
        div++;
    }

    //clutch
    if (ar_engine && flag_state & ANIM_FLAG_CLUTCH)
    {
        float clutch = ar_engine->GetClutch();
        cstate -= fabs(1.0f - clutch);
        div++;
    }

    //aeroengines rpm + throttle + torque ( turboprop ) + pitch ( turboprop ) + status +  fire
    int ftp = ar_num_aeroengines;

    if (ftp > option3 - 1.0f)
    {
        int aenum = int(option3 - 1.0f);
        if (flag_state & ANIM_FLAG_RPM)
        {
            float angle;
            float pcent = ar_aeroengines[aenum]->getRPMpc();
            if (pcent < 60.0)
                angle = -5.0 + pcent * 1.9167;
            else if (pcent < 110.0)
                angle = 110.0 + (pcent - 60.0) * 4.075;
            else
                angle = 314.0;
            cstate -= angle / 314.0f;
            div++;
        }
        if (flag_state & ANIM_FLAG_THROTTLE)
        {
            float throttle = ar_aeroengines[aenum]->getThrottle();
            cstate -= throttle;
            div++;
        }

        if (flag_state & ANIM_FLAG_AETORQUE)
            if (ar_aeroengines[aenum]->getType() == AeroEngine::AEROENGINE_TYPE_TURBOPROP)
            {
                Turboprop* tp = (Turboprop*)ar_aeroengines[aenum];
                cstate = (100.0 * tp->indicated_torque / tp->max_torque) / 120.0f;
                div++;
            }

        if (flag_state & ANIM_FLAG_AEPITCH)
            if (ar_aeroengines[aenum]->getType() == AeroEngine::AEROENGINE_TYPE_TURBOPROP)
            {
                Turboprop* tp = (Turboprop*)ar_aeroengines[aenum];
                cstate = tp->pitch / 120.0f;
                div++;
            }

        if (flag_state & ANIM_FLAG_AESTATUS)
        {
            if (!ar_aeroengines[aenum]->getIgnition())
                cstate = 0.0f;
            else
                cstate = 0.5f;
            if (ar_aeroengines[aenum]->isFailed())
                cstate = 1.0f;
            div++;
        }
    }

    //airspeed indicator
    if (flag_state & ANIM_FLAG_AIRSPEED)
    {
        // TODO Unused Varaible
        //float angle=0.0;
        float ground_speed_kt = ar_nodes[0].Velocity.length() * 1.9438;
        float altitude = ar_nodes[0].AbsPosition.y;

        // TODO Unused Varaible
        //float sea_level_temperature=273.15+15.0; //in Kelvin
        float sea_level_pressure = 101325; //in Pa

        // TODO Unused Varaible
        //float airtemperature=sea_level_temperature-altitude*0.0065; //in Kelvin
        float airpressure = sea_level_pressure * pow(1.0 - 0.0065 * altitude / 288.15, 5.24947); //in Pa
        float airdensity = airpressure * 0.0000120896;//1.225 at sea level
        float kt = ground_speed_kt * sqrt(airdensity / 1.225);
        cstate -= kt / 100.0f;
        div++;
    }

    //vvi indicator
    if (flag_state & ANIM_FLAG_VVI)
    {
        float vvi = ar_nodes[0].Velocity.y * 196.85;
        // limit vvi scale to +/- 6m/s
        cstate -= vvi / 6000.0f;
        if (cstate >= 1.0f)
            cstate = 1.0f;
        if (cstate <= -1.0f)
            cstate = -1.0f;
        div++;
    }

    //altimeter
    if (flag_state & ANIM_FLAG_ALTIMETER)
    {
        //altimeter indicator 1k oscillating
        if (option3 == 3.0f)
        {
            float altimeter = (ar_nodes[0].AbsPosition.y * 1.1811) / 360.0f;
            int alti_int = int(altimeter);
            float alti_mod = (altimeter - alti_int);
            cstate -= alti_mod;
        }

        //altimeter indicator 10k oscillating
        if (option3 == 2.0f)
        {
            float alti = ar_nodes[0].AbsPosition.y * 1.1811 / 3600.0f;
            int alti_int = int(alti);
            float alti_mod = (alti - alti_int);
            cstate -= alti_mod;
            if (cstate <= -1.0f)
                cstate = -1.0f;
        }

        //altimeter indicator 100k limited
        if (option3 == 1.0f)
        {
            float alti = ar_nodes[0].AbsPosition.y * 1.1811 / 36000.0f;
            cstate -= alti;
            if (cstate <= -1.0f)
                cstate = -1.0f;
        }
        div++;
    }

    //AOA
    if (flag_state & ANIM_FLAG_AOA)
    {
        float aoa = 0;
        if (ar_num_wings > 4)
            aoa = (ar_wings[4].fa->aoa) / 25.0f;
        if ((ar_nodes[0].Velocity.length() * 1.9438) < 10.0f)
            aoa = 0;
        cstate -= aoa;
        if (cstate <= -1.0f)
            cstate = -1.0f;
        if (cstate >= 1.0f)
            cstate = 1.0f;
        div++;
    }

    // roll
    if (flag_state & ANIM_FLAG_ROLL)
    {
        Vector3 rollv = this->GetCameraRoll();
        Vector3 dirv = this->GetCameraDir();
        Vector3 upv = dirv.crossProduct(-rollv);
        float rollangle = asin(rollv.dotProduct(Vector3::UNIT_Y));
        // rad to deg
        rollangle = Math::RadiansToDegrees(rollangle);
        // flip to other side when upside down
        if (upv.y < 0)
            rollangle = 180.0f - rollangle;
        cstate = rollangle / 180.0f;
        // data output is -0.5 to 1.5, normalize to -1 to +1 without changing the zero position.
        // this is vital for the animator beams and does not effect the animated props
        if (cstate >= 1.0f)
            cstate = cstate - 2.0f;
        div++;
    }

    // pitch
    if (flag_state & ANIM_FLAG_PITCH)
    {
        Vector3 dirv = this->GetCameraDir();
        float pitchangle = asin(dirv.dotProduct(Vector3::UNIT_Y));
        // radian to degrees with a max cstate of +/- 1.0
        cstate = (Math::RadiansToDegrees(pitchangle) / 90.0f);
        div++;
    }

    // airbrake
    if (flag_state & ANIM_FLAG_AIRBRAKE)
    {
        float airbrake = ar_airbrake_intensity;
        // cstate limited to -1.0f
        cstate -= airbrake / 5.0f;
        div++;
    }

    //flaps
    if (flag_state & ANIM_FLAG_FLAP)
    {
        float flaps = flapangles[ar_aerial_flap];
        // cstate limited to -1.0f
        cstate = flaps;
        div++;
    }
}

void Actor::CalcCabCollisions()
{
    for (int i = 0; i < ar_num_nodes; i++)
    {
        ar_nodes[i].nd_has_mesh_contact = false;
    }
    if (m_intra_point_col_detector != nullptr)
    {
        m_intra_point_col_detector->UpdateIntraPoint();
        ResolveIntraActorCollisions(PHYSICS_DT,
            *m_intra_point_col_detector,
            ar_num_collcabs,
            ar_collcabs,
            ar_cabs,
            ar_intra_collcabrate,
            ar_nodes,
            ar_collision_range,
            *ar_submesh_ground_model);
    }
}

void Actor::CalcTriggers(int i, Real difftoBeamL, bool trigger_hooks)
{
    if ((ar_beams[i].shock->flags & SHOCK_FLAG_ISTRIGGER) && ar_beams[i].shock->trigger_enabled) // this is a trigger and its enabled
    {
        const float dt = PHYSICS_DT;

        if (difftoBeamL > ar_beams[i].longbound * ar_beams[i].L || difftoBeamL < -ar_beams[i].shortbound * ar_beams[i].L) // that has hit boundary
        {
            ar_beams[i].shock->trigger_switch_state -= dt;
            if (ar_beams[i].shock->trigger_switch_state <= 0.0f) // emergency release for dead-switched trigger
                ar_beams[i].shock->trigger_switch_state = 0.0f;
            if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_BLOCKER) // this is an enabled blocker and past boundary
            {
                for (int scount = i + 1; scount <= i + ar_beams[i].shock->trigger_cmdshort; scount++) // (cycle blockerbeamID +1) to (blockerbeamID + beams to lock)
                {
                    if (ar_beams[scount].shock && (ar_beams[scount].shock->flags & SHOCK_FLAG_ISTRIGGER)) // don't mess anything up if the user set the number too big
                    {
                        if (m_trigger_debug_enabled && !ar_beams[scount].shock->trigger_enabled && ar_beams[i].shock->last_debug_state != 1)
                        {
                            LOG(" Trigger disabled. Blocker BeamID " + TOSTRING(i) + " enabled trigger " + TOSTRING(scount));
                            ar_beams[i].shock->last_debug_state = 1;
                        }
                        ar_beams[scount].shock->trigger_enabled = false; // disable the trigger
                    }
                }
            }
            else if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_BLOCKER_A) // this is an enabled inverted blocker and inside boundary
            {
                for (int scount = i + 1; scount <= i + ar_beams[i].shock->trigger_cmdlong; scount++) // (cycle blockerbeamID + 1) to (blockerbeamID + beams to release)
                {
                    if (ar_beams[scount].shock && (ar_beams[scount].shock->flags & SHOCK_FLAG_ISTRIGGER)) // don't mess anything up if the user set the number too big
                    {
                        if (m_trigger_debug_enabled && ar_beams[scount].shock->trigger_enabled && ar_beams[i].shock->last_debug_state != 9)
                        {
                            LOG(" Trigger enabled. Inverted Blocker BeamID " + TOSTRING(i) + " disabled trigger " + TOSTRING(scount));
                            ar_beams[i].shock->last_debug_state = 9;
                        }
                        ar_beams[scount].shock->trigger_enabled = true; // enable the triggers
                    }
                }
            }
            else if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_CMD_BLOCKER) // this an enabled cmd-key-blocker and past a boundary
            {
                ar_command_key[ar_beams[i].shock->trigger_cmdshort].trigger_cmdkeyblock_state = false; // Release the cmdKey
                if (m_trigger_debug_enabled && ar_beams[i].shock->last_debug_state != 2)
                {
                    LOG(" F-key trigger block released. Blocker BeamID " + TOSTRING(i) + " Released F" + TOSTRING(ar_beams[i].shock->trigger_cmdshort));
                    ar_beams[i].shock->last_debug_state = 2;
                }
            }
            else if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_CMD_SWITCH) // this is an enabled cmdkey switch and past a boundary
            {
                if (!ar_beams[i].shock->trigger_switch_state)// this switch is triggered first time in this boundary
                {
                    for (int scount = 0; scount < ar_num_shocks; scount++)
                    {
                        int short1 = ar_beams[ar_shocks[scount].beamid].shock->trigger_cmdshort; // cmdshort of checked trigger beam
                        int short2 = ar_beams[i].shock->trigger_cmdshort; // cmdshort of switch beam
                        int long1 = ar_beams[ar_shocks[scount].beamid].shock->trigger_cmdlong; // cmdlong of checked trigger beam
                        int long2 = ar_beams[i].shock->trigger_cmdlong; // cmdlong of switch beam
                        int tmpi = ar_beams[ar_shocks[scount].beamid].shock->beamid; // beamID global of checked trigger beam
                        if (((short1 == short2 && long1 == long2) || (short1 == long2 && long1 == short2)) && i != tmpi) // found both command triggers then swap if its not the switching trigger
                        {
                            int tmpcmdkey = ar_beams[ar_shocks[scount].beamid].shock->trigger_cmdlong;
                            ar_beams[ar_shocks[scount].beamid].shock->trigger_cmdlong = ar_beams[ar_shocks[scount].beamid].shock->trigger_cmdshort;
                            ar_beams[ar_shocks[scount].beamid].shock->trigger_cmdshort = tmpcmdkey;
                            ar_beams[i].shock->trigger_switch_state = ar_beams[i].shock->trigger_boundary_t; //prevent trigger switching again before leaving boundaries or timeout
                            if (m_trigger_debug_enabled && ar_beams[i].shock->last_debug_state != 3)
                            {
                                LOG(" Trigger F-key commands switched. Switch BeamID " + TOSTRING(i)+ " switched commands of Trigger BeamID " + TOSTRING(ar_beams[ar_shocks[scount].beamid].shock->beamid) + " to cmdShort: F" + TOSTRING(ar_beams[ar_shocks[scount].beamid].shock->trigger_cmdshort) + ", cmdlong: F" + TOSTRING(ar_beams[ar_shocks[scount].beamid].shock->trigger_cmdlong));
                                ar_beams[i].shock->last_debug_state = 3;
                            }
                        }
                    }
                }
            }
            else
            { // just a trigger, check high/low boundary and set action
                if (difftoBeamL > ar_beams[i].longbound * ar_beams[i].L) // trigger past longbound
                {
                    if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_HOOK_UNLOCK)
                    {
                        if (trigger_hooks)
                        {
                            //autolock hooktoggle unlock
                            ToggleHooks(ar_beams[i].shock->trigger_cmdlong, HOOK_UNLOCK, -1);
                        }
                    }
                    else if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_HOOK_LOCK)
                    {
                        if (trigger_hooks)
                        {
                            //autolock hooktoggle lock
                            ToggleHooks(ar_beams[i].shock->trigger_cmdlong, HOOK_LOCK, -1);
                        }
                    }
                    else if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_ENGINE)
                    {
                        EngineTriggerHelper(ar_beams[i].shock->trigger_cmdshort, ar_beams[i].shock->trigger_cmdlong, 1.0f);
                    }
                    else
                    {
                        //just a trigger
                        if (!ar_command_key[ar_beams[i].shock->trigger_cmdlong].trigger_cmdkeyblock_state) // related cmdkey is not blocked
                        {
                            if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_CONTINUOUS)
                                ar_command_key[ar_beams[i].shock->trigger_cmdshort].triggerInputValue = 1; // continuous trigger only operates on trigger_cmdshort
                            else
                                ar_command_key[ar_beams[i].shock->trigger_cmdlong].triggerInputValue = 1;
                            if (m_trigger_debug_enabled && ar_beams[i].shock->last_debug_state != 4)
                            {
                                LOG(" Trigger Longbound activated. Trigger BeamID " + TOSTRING(i) + " Triggered F" + TOSTRING(ar_beams[i].shock->trigger_cmdlong));
                                ar_beams[i].shock->last_debug_state = 4;
                            }
                        }
                    }
                }
                else // trigger past short bound
                {
                    if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_HOOK_UNLOCK)
                    {
                        if (trigger_hooks)
                        {
                            //autolock hooktoggle unlock
                            ToggleHooks(ar_beams[i].shock->trigger_cmdshort, HOOK_UNLOCK, -1);
                        }
                    }
                    else if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_HOOK_LOCK)
                    {
                        if (trigger_hooks)
                        {
                            //autolock hooktoggle lock
                            ToggleHooks(ar_beams[i].shock->trigger_cmdshort, HOOK_LOCK, -1);
                        }
                    }
                    else if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_ENGINE)
                    {
                        bool triggerValue = !(ar_beams[i].shock->flags & SHOCK_FLAG_TRG_CONTINUOUS); // 0 if trigger is continuous, 1 otherwise

                        EngineTriggerHelper(ar_beams[i].shock->trigger_cmdshort, ar_beams[i].shock->trigger_cmdlong, triggerValue);
                    }
                    else
                    {
                        //just a trigger
                        if (!ar_command_key[ar_beams[i].shock->trigger_cmdshort].trigger_cmdkeyblock_state) // related cmdkey is not blocked
                        {
                            if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_CONTINUOUS)
                                ar_command_key[ar_beams[i].shock->trigger_cmdshort].triggerInputValue = 0; // continuous trigger only operates on trigger_cmdshort
                            else
                                ar_command_key[ar_beams[i].shock->trigger_cmdshort].triggerInputValue = 1;

                            if (m_trigger_debug_enabled && ar_beams[i].shock->last_debug_state != 5)
                            {
                                LOG(" Trigger Shortbound activated. Trigger BeamID " + TOSTRING(i) + " Triggered F" + TOSTRING(ar_beams[i].shock->trigger_cmdshort));
                                ar_beams[i].shock->last_debug_state = 5;
                            }
                        }
                    }
                }
            }
        }
        else // this is a trigger inside boundaries and its enabled
        {
            if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_CONTINUOUS) // this is an enabled continuous trigger
            {
                if (ar_beams[i].longbound - ar_beams[i].shortbound > 0.0f)
                {
                    float diffPercentage = difftoBeamL / ar_beams[i].L;
                    float triggerValue = (diffPercentage - ar_beams[i].shortbound) / (ar_beams[i].longbound - ar_beams[i].shortbound);

                    triggerValue = std::max(0.0f, triggerValue);
                    triggerValue = std::min(triggerValue, 1.0f);

                    if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_ENGINE) // this trigger controls an engine
                    {
                        EngineTriggerHelper(ar_beams[i].shock->trigger_cmdshort, ar_beams[i].shock->trigger_cmdlong, triggerValue);
                    }
                    else
                    {
                        // normal trigger
                        ar_command_key[ar_beams[i].shock->trigger_cmdshort].triggerInputValue = triggerValue;
                        ar_command_key[ar_beams[i].shock->trigger_cmdlong].triggerInputValue = triggerValue;
                    }
                }
            }
            else if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_BLOCKER) // this is an enabled blocker and inside boundary
            {
                for (int scount = i + 1; scount <= i + ar_beams[i].shock->trigger_cmdlong; scount++) // (cycle blockerbeamID + 1) to (blockerbeamID + beams to release)
                {
                    if (ar_beams[scount].shock && (ar_beams[scount].shock->flags & SHOCK_FLAG_ISTRIGGER)) // don't mess anything up if the user set the number too big
                    {
                        if (m_trigger_debug_enabled && ar_beams[scount].shock->trigger_enabled && ar_beams[i].shock->last_debug_state != 6)
                        {
                            LOG(" Trigger enabled. Blocker BeamID " + TOSTRING(i) + " disabled trigger " + TOSTRING(scount));
                            ar_beams[i].shock->last_debug_state = 6;
                        }
                        ar_beams[scount].shock->trigger_enabled = true; // enable the triggers
                    }
                }
            }
            else if (ar_beams[i].shock->flags & SHOCK_FLAG_TRG_BLOCKER_A) // this is an enabled reverse blocker and past boundary
            {
                for (int scount = i + 1; scount <= i + ar_beams[i].shock->trigger_cmdshort; scount++) // (cylce blockerbeamID +1) to (blockerbeamID + beams tob lock)
                {
                    if (ar_beams[scount].shock && (ar_beams[scount].shock->flags & SHOCK_FLAG_ISTRIGGER)) // dont mess anything up if the user set the number too big
                    {
                        if (m_trigger_debug_enabled && !ar_beams[scount].shock->trigger_enabled && ar_beams[i].shock->last_debug_state != 10)
                        {
                            LOG(" Trigger disabled. Inverted Blocker BeamID " + TOSTRING(i) + " enabled trigger " + TOSTRING(scount));
                            ar_beams[i].shock->last_debug_state = 10;
                        }
                        ar_beams[scount].shock->trigger_enabled = false; // disable the trigger
                    }
                }
            }
            else if ((ar_beams[i].shock->flags & SHOCK_FLAG_TRG_CMD_SWITCH) && ar_beams[i].shock->trigger_switch_state) // this is a switch that was activated and is back inside boundaries again
            {
                ar_beams[i].shock->trigger_switch_state = 0.0f; //trigger_switch reset
                if (m_trigger_debug_enabled && ar_beams[i].shock->last_debug_state != 7)
                {
                    LOG(" Trigger switch reset. Switch BeamID " + TOSTRING(i));
                    ar_beams[i].shock->last_debug_state = 7;
                }
            }
            else if ((ar_beams[i].shock->flags & SHOCK_FLAG_TRG_CMD_BLOCKER) && !ar_command_key[ar_beams[i].shock->trigger_cmdshort].trigger_cmdkeyblock_state) // this cmdkeyblocker is inside boundaries and cmdkeystate is diabled
            {
                ar_command_key[ar_beams[i].shock->trigger_cmdshort].trigger_cmdkeyblock_state = true; // activate trigger blocking
                if (m_trigger_debug_enabled && ar_beams[i].shock->last_debug_state != 8)
                {
                    LOG(" F-key trigger blocked. Blocker BeamID " + TOSTRING(i) + " Blocked F" + TOSTRING(ar_beams[i].shock->trigger_cmdshort));
                    ar_beams[i].shock->last_debug_state = 8;
                }
            }
        }
    }
}

void Actor::setAirbrakeIntensity(float intensity)
{
    ar_airbrake_intensity = intensity;
    for (Airbrake* ab: ar_airbrakes)
    {
        ab->updatePosition((float)ar_airbrake_intensity / 5.0);
    }
}

void Actor::prepareInside(bool inside)
{
    // TODO: this whole function belongs to GfxActor ~ 08/2018
    if (inside)
    {
        gEnv->mainCamera->setNearClipDistance(0.1f);

        // enable transparent seat
        MaterialPtr seatmat = (MaterialPtr)(MaterialManager::getSingleton().getByName("driversseat"));
        seatmat->setDepthWriteEnabled(false);
        seatmat->setSceneBlending(SBT_TRANSPARENT_ALPHA);
    }
    else
    {
        if (ar_dashboard)
        {
            ar_dashboard->setVisible(false);
        }

        gEnv->mainCamera->setNearClipDistance(0.5f);

        // disable transparent seat
        MaterialPtr seatmat = (MaterialPtr)(MaterialManager::getSingleton().getByName("driversseat"));
        seatmat->setDepthWriteEnabled(true);
        seatmat->setSceneBlending(SBT_REPLACE);
    }

  // TEMPORARY - until this function is moved to GfxActor ~ 08/2018
  //  if (m_cab_scene_node != nullptr)
  //  {
  //      m_gfx_actor->GetCabTransMaterial()->setReceiveShadows(!inside);
  //  }

    if (App::gfx_reduce_shadows.GetActive())
    {
        m_gfx_actor->SetCastShadows(!inside);
    }
}

void Actor::updateVisual(float dt)
{
    Vector3 ref(Vector3::UNIT_Y);
    autoBlinkReset();
    UpdateSoundSources();

#ifdef USE_OPENAL
    //airplane radio chatter
    if (ar_driveable == AIRPLANE && ar_sim_state != SimState::LOCAL_SLEEPING)
    {
        // play random chatter at random time
        m_avionic_chatter_timer -= dt;
        if (m_avionic_chatter_timer < 0)
        {
            SOUND_PLAY_ONCE(ar_instance_id, SS_TRIG_AVICHAT01 + Math::RangeRandom(0, 12));
            m_avionic_chatter_timer = Math::RangeRandom(11, 30);
        }
    }
#endif //openAL

    // update exhausts
    // TODO: Move to GfxActor, don't forget dt*m_simulation_speed
    if (!m_disable_smoke && ar_engine && exhausts.size() > 0)
    {
        std::vector<exhaust_t>::iterator it;
        for (it = exhausts.begin(); it != exhausts.end(); it++)
        {
            if (!it->smoker)
                continue;
            Vector3 dir = ar_nodes[it->emitterNode].AbsPosition - ar_nodes[it->directionNode].AbsPosition;
            //			dir.normalise();
            ParticleEmitter* emit = it->smoker->getEmitter(0);
            it->smokeNode->setPosition(ar_nodes[it->emitterNode].AbsPosition);
            emit->setDirection(dir);
            if (ar_engine->GetSmoke() != -1.0)
            {
                emit->setEnabled(true);
                emit->setColour(ColourValue(0.0, 0.0, 0.0, 0.02 + ar_engine->GetSmoke() * 0.06));
                emit->setTimeToLive((0.02 + ar_engine->GetSmoke() * 0.06) / 0.04);
            }
            else
            {
                emit->setEnabled(false);
            }
            emit->setParticleVelocity(1.0 + ar_engine->GetSmoke() * 2.0, 2.0 + ar_engine->GetSmoke() * 3.0);
        }
    }

    // Wings (only physics, graphics are updated in GfxActor)
    float autoaileron = 0;
    float autorudder = 0;
    float autoelevator = 0;
    if (ar_autopilot)
    {
        ar_autopilot->UpdateIls(App::GetSimTerrain()->getObjectManager()->GetLocalizers());
        autoaileron = ar_autopilot->getAilerons();
        autorudder = ar_autopilot->getRudder();
        autoelevator = ar_autopilot->getElevator();
        ar_autopilot->gpws_update(ar_posnode_spawn_height);
    }
    autoaileron += ar_aileron;
    autorudder += ar_rudder;
    autoelevator += ar_elevator;
    if (autoaileron < -1.0)
        autoaileron = -1.0;
    if (autoaileron > 1.0)
        autoaileron = 1.0;
    if (autorudder < -1.0)
        autorudder = -1.0;
    if (autorudder > 1.0)
        autorudder = 1.0;
    if (autoelevator < -1.0)
        autoelevator = -1.0;
    if (autoelevator > 1.0)
        autoelevator = 1.0;
    for (int i = 0; i < ar_num_wings; i++)
    {
        if (ar_wings[i].fa->type == 'a')
            ar_wings[i].fa->setControlDeflection(autoaileron);
        if (ar_wings[i].fa->type == 'b')
            ar_wings[i].fa->setControlDeflection(-autoaileron);
        if (ar_wings[i].fa->type == 'r')
            ar_wings[i].fa->setControlDeflection(autorudder);
        if (ar_wings[i].fa->type == 'e' || ar_wings[i].fa->type == 'S' || ar_wings[i].fa->type == 'T')
            ar_wings[i].fa->setControlDeflection(autoelevator);
        if (ar_wings[i].fa->type == 'f')
            ar_wings[i].fa->setControlDeflection(flapangles[ar_aerial_flap]);
        if (ar_wings[i].fa->type == 'c' || ar_wings[i].fa->type == 'V')
            ar_wings[i].fa->setControlDeflection((autoaileron + autoelevator) / 2.0);
        if (ar_wings[i].fa->type == 'd' || ar_wings[i].fa->type == 'U')
            ar_wings[i].fa->setControlDeflection((-autoaileron + autoelevator) / 2.0);
        if (ar_wings[i].fa->type == 'g')
            ar_wings[i].fa->setControlDeflection((autoaileron + flapangles[ar_aerial_flap]) / 2.0);
        if (ar_wings[i].fa->type == 'h')
            ar_wings[i].fa->setControlDeflection((-autoaileron + flapangles[ar_aerial_flap]) / 2.0);
        if (ar_wings[i].fa->type == 'i')
            ar_wings[i].fa->setControlDeflection((-autoelevator + autorudder) / 2.0);
        if (ar_wings[i].fa->type == 'j')
            ar_wings[i].fa->setControlDeflection((autoelevator + autorudder) / 2.0);
        ar_wings[i].fa->updateVerticesPhysics(); // Actual graphics update moved to GfxActor
    }
    //setup commands for hydros
    ar_hydro_aileron_command = autoaileron;
    ar_hydro_rudder_command = autorudder;
    ar_hydro_elevator_command = autoelevator;
}

void Actor::AddInterActorBeam(beam_t* beam, Actor* a, Actor* b)
{
    beam->bm_locked_actor = b;

    auto pos = std::find(ar_inter_beams.begin(), ar_inter_beams.end(), beam);
    if (pos == ar_inter_beams.end())
    {
        ar_inter_beams.push_back(beam);
    }

    std::pair<Actor*, Actor*> actor_pair(a, b);
    App::GetSimController()->GetBeamFactory()->inter_actor_links[beam] = actor_pair;

    a->DetermineLinkedActors();
    for (auto actor : a->m_linked_actors)
        actor->DetermineLinkedActors();

    b->DetermineLinkedActors();
    for (auto actor : b->m_linked_actors)
        actor->DetermineLinkedActors();
}

void Actor::RemoveInterActorBeam(beam_t* beam)
{
    auto pos = std::find(ar_inter_beams.begin(), ar_inter_beams.end(), beam);
    if (pos != ar_inter_beams.end())
    {
        ar_inter_beams.erase(pos);
    }

    auto it = App::GetSimController()->GetBeamFactory()->inter_actor_links.find(beam);
    if (it != App::GetSimController()->GetBeamFactory()->inter_actor_links.end())
    {
        auto actor_pair = it->second;
        App::GetSimController()->GetBeamFactory()->inter_actor_links.erase(it);

        actor_pair.first->DetermineLinkedActors();
        for (auto actor : actor_pair.first->m_linked_actors)
            actor->DetermineLinkedActors();

        actor_pair.second->DetermineLinkedActors();
        for (auto actor : actor_pair.second->m_linked_actors)
            actor->DetermineLinkedActors();
    }
}

void Actor::DisjoinInterActorBeams()
{
    ar_inter_beams.clear();
    auto inter_actor_links = &App::GetSimController()->GetBeamFactory()->inter_actor_links;
    for (auto it = inter_actor_links->begin(); it != inter_actor_links->end();)
    {
        auto actor_pair = it->second;
        if (this == actor_pair.first || this == actor_pair.second)
        {
            it->first->bm_locked_actor = nullptr;
            it->first->bm_inter_actor = false;
            it->first->bm_disabled = true;
            inter_actor_links->erase(it++);

            actor_pair.first->DetermineLinkedActors();
            for (auto actor : actor_pair.first->m_linked_actors)
                actor->DetermineLinkedActors();

            actor_pair.second->DetermineLinkedActors();
            for (auto actor : actor_pair.second->m_linked_actors)
                actor->DetermineLinkedActors();
        }
        else
        {
            ++it;
        }
    }
}

void Actor::ToggleTies(int group)
{
    Actor* player_actor = App::GetSimController()->GetPlayerActor();

    // untie all ties if one is tied
    bool istied = false;

    for (std::vector<tie_t>::iterator it = ar_ties.begin(); it != ar_ties.end(); it++)
    {
        // only handle ties with correct group
        if (group != -1 && (it->ti_group != -1 && it->ti_group != group))
            continue;

        // if tied, untie it. And the other way round
        if (it->ti_tied)
        {
            istied = !it->ti_beam->bm_disabled;

            // tie is locked and should get unlocked and stop tying
            it->ti_tied = false;
            it->ti_tying = false;
            if (it->ti_locked_ropable)
                it->ti_locked_ropable->attached_ties--;
            // disable the ties beam
            it->ti_beam->p2 = &ar_nodes[0];
            it->ti_beam->bm_inter_actor = false;
            it->ti_beam->bm_disabled = true;
            if (it->ti_locked_actor != this)
            {
                this->RemoveInterActorBeam(it->ti_beam);
                // update skeletonview on the untied actors
                auto linked_actors = it->ti_locked_actor->GetAllLinkedActors();
                if (!(std::find(linked_actors.begin(), linked_actors.end(), this) != linked_actors.end()))
                {
                    if (this == player_actor)
                    {
                        it->ti_locked_actor->GetGfxActor()->SetDebugView(GfxActor::DebugViewType::DEBUGVIEW_NONE);
                        for (auto actor : it->ti_locked_actor->GetAllLinkedActors())
                        {
                            actor->GetGfxActor()->SetDebugView(GfxActor::DebugViewType::DEBUGVIEW_NONE);
                        }
                    }
                    else if (it->ti_locked_actor == player_actor)
                    {
                        m_gfx_actor->SetDebugView(GfxActor::DebugViewType::DEBUGVIEW_NONE);
                        for (auto actor : this->GetAllLinkedActors())
                        {
                            actor->GetGfxActor()->SetDebugView(GfxActor::DebugViewType::DEBUGVIEW_NONE);
                        }
                    }
                }
            }
            it->ti_locked_actor = nullptr;
        }
    }

    // iterate over all ties
    if (!istied)
    {
        for (std::vector<tie_t>::iterator it = ar_ties.begin(); it != ar_ties.end(); it++)
        {
            // only handle ties with correct group
            if (group != -1 && (it->ti_group != -1 && it->ti_group != group))
                continue;

            if (!it->ti_tied)
            {
                // tie is unlocked and should get locked, search new remote ropable to lock to
                float mindist = it->ti_beam->refL;
                node_t* nearest_node = 0;
                Actor* nearest_actor = 0;
                ropable_t* locktedto = 0;
                // iterate over all actors
                for (auto actor : App::GetSimController()->GetActors())
                {
                    if (actor->ar_sim_state == SimState::LOCAL_SLEEPING ||
                        (actor == this && it->ti_no_self_lock))
                    {
                        continue;
                    }

                    // and their ropables
                    for (std::vector<ropable_t>::iterator itr = actor->ar_ropables.begin(); itr != actor->ar_ropables.end(); itr++)
                    {
                        // if the ropable is not multilock and used, then discard this ropable
                        if (!itr->multilock && itr->attached_ties > 0)
                            continue;

                        // skip if tienode is ropable too (no selflock)
                        if (this == actor && itr->node->pos == it->ti_beam->p1->pos)
                            continue;

                        // calculate the distance and record the nearest ropable
                        float dist = (it->ti_beam->p1->AbsPosition - itr->node->AbsPosition).length();
                        if (dist < mindist)
                        {
                            mindist = dist;
                            nearest_node = itr->node;
                            nearest_actor = actor;
                            locktedto = &(*itr);
                        }
                    }
                }
                // if we found a ropable, then tie towards it
                if (nearest_node)
                {
                    // enable the beam and visually display the beam
                    it->ti_beam->bm_disabled = false;
                    // now trigger the tying action
                    it->ti_locked_actor = nearest_actor;
                    it->ti_beam->p2 = nearest_node;
                    it->ti_beam->bm_inter_actor = nearest_actor != this;
                    it->ti_beam->stress = 0;
                    it->ti_beam->L = it->ti_beam->refL;
                    it->ti_tied = true;
                    it->ti_tying = true;
                    it->ti_locked_ropable = locktedto;
                    it->ti_locked_ropable->attached_ties++;
                    if (it->ti_beam->bm_inter_actor)
                    {
                        AddInterActorBeam(it->ti_beam, this, nearest_actor);
                        // update skeletonview on the tied actors
                        if (this == player_actor)
                        {
                            nearest_actor->GetGfxActor()->SetDebugView(m_gfx_actor->GetDebugView());
                            for (auto actor : nearest_actor->GetAllLinkedActors())
                            {
                                actor->GetGfxActor()->SetDebugView(m_gfx_actor->GetDebugView());
                            }
                        }
                        else if (nearest_actor == player_actor)
                        {
                            m_gfx_actor->SetDebugView(player_actor->GetGfxActor()->GetDebugView());
                            for (auto actor : this->GetAllLinkedActors())
                            {
                                actor->GetGfxActor()->SetDebugView(player_actor->GetGfxActor()->GetDebugView());
                            }
                        }
                    }
                }
            }
        }
    }

    //ScriptEvent - Tie toggle
    TRIGGER_EVENT(SE_TRUCK_TIE_TOGGLE, ar_instance_id);
}

void Actor::ToggleRopes(int group)
{
    Actor* player_actor = App::GetSimController()->GetPlayerActor();

    // iterate over all ropes
    for (std::vector<rope_t>::iterator it = ar_ropes.begin(); it != ar_ropes.end(); it++)
    {
        // only handle ropes with correct group
        if (group != -1 && (it->rp_group != -1 && it->rp_group != group))
            continue;

        if (it->rp_locked == LOCKED || it->rp_locked == PRELOCK)
        {
            // we unlock ropes
            it->rp_locked = UNLOCKED;
            // remove node locking
            if (it->rp_locked_ropable)
                it->rp_locked_ropable->attached_ropes--;
            if (it->rp_locked_actor != this)
            {
                this->RemoveInterActorBeam(it->rp_beam);
                // update skeletonview on the unroped actors
                auto linked_actors = it->rp_locked_actor->GetAllLinkedActors();
                if (!(std::find(linked_actors.begin(), linked_actors.end(), this) != linked_actors.end()))
                {
                    if (this == player_actor)
                    {
                        it->rp_locked_actor->GetGfxActor()->SetDebugView(GfxActor::DebugViewType::DEBUGVIEW_NONE);
                        for (auto actor : it->rp_locked_actor->GetAllLinkedActors())
                        {
                            actor->GetGfxActor()->SetDebugView(GfxActor::DebugViewType::DEBUGVIEW_NONE);
                        }
                    }
                    else if (it->rp_locked_actor == player_actor)
                    {
                        m_gfx_actor->SetDebugView(GfxActor::DebugViewType::DEBUGVIEW_NONE);
                        for (auto actor : this->GetAllLinkedActors())
                        {
                            actor->GetGfxActor()->SetDebugView(GfxActor::DebugViewType::DEBUGVIEW_NONE);
                        }
                    }
                }
            }
            it->rp_locked_actor = nullptr;
            it->rp_locked_ropable = nullptr;
        }
        else
        {
            //we lock ropes
            // search new remote ropable to lock to
            float mindist = it->rp_beam->L;
            Actor* nearest_actor = nullptr;
            ropable_t* rop = 0;
            // iterate over all actor_slots
            for (auto actor : App::GetSimController()->GetActors())
            {
                if (actor->ar_sim_state == SimState::LOCAL_SLEEPING)
                    continue;
                // and their ropables
                for (std::vector<ropable_t>::iterator itr = actor->ar_ropables.begin(); itr != actor->ar_ropables.end(); itr++)
                {
                    // if the ropable is not multilock and used, then discard this ropable
                    if (!itr->multilock && itr->attached_ropes > 0)
                        continue;

                    // calculate the distance and record the nearest ropable
                    float dist = (it->rp_beam->p1->AbsPosition - itr->node->AbsPosition).length();
                    if (dist < mindist)
                    {
                        mindist = dist;
                        nearest_actor = actor;
                        rop = &(*itr);
                    }
                }
            }
            // if we found a ropable, then lock it
            if (nearest_actor)
            {
                //okay, we have found a rope to tie
                it->rp_locked_actor = nearest_actor;
                it->rp_locked = LOCKED;
                it->rp_locked_ropable = rop;
                it->rp_locked_ropable->attached_ropes++;
                if (nearest_actor != this)
                {
                    AddInterActorBeam(it->rp_beam, this, nearest_actor);
                    // update skeletonview on the roped up actors
                    if (this == player_actor)
                    {
                        nearest_actor->GetGfxActor()->SetDebugView(m_gfx_actor->GetDebugView());
                        for (auto actor : nearest_actor->GetAllLinkedActors())
                        {
                            actor->GetGfxActor()->SetDebugView(m_gfx_actor->GetDebugView());
                        }
                    }
                    else if (nearest_actor == player_actor)
                    {
                        m_gfx_actor->SetDebugView(player_actor->GetGfxActor()->GetDebugView());
                        for (auto actor : this->GetAllLinkedActors())
                        {
                            actor->GetGfxActor()->SetDebugView(player_actor->GetGfxActor()->GetDebugView());
                        }
                    }
                }
            }
        }
    }
}

//Returns the number of active (non bounded) beams connected to a node
int Actor::GetNumActiveConnectedBeams(int nodeid)
{
    int totallivebeams = 0;
    for (unsigned int ni = 0; ni < ar_node_to_beam_connections[nodeid].size(); ++ni)
    {
        if (!ar_beams[ar_node_to_beam_connections[nodeid][ni]].bm_disabled && !ar_beams[ar_node_to_beam_connections[nodeid][ni]].bounded)
            totallivebeams++;
    }
    return totallivebeams;
}

bool Actor::isTied()
{
    for (std::vector<tie_t>::iterator it = ar_ties.begin(); it != ar_ties.end(); it++)
        if (it->ti_tied)
            return true;
    return false;
}

bool Actor::isLocked()
{
    for (std::vector<hook_t>::iterator it = ar_hooks.begin(); it != ar_hooks.end(); it++)
        if (it->hk_locked == LOCKED)
            return true;
    return false;
}

void Actor::calculateLocalGForces()
{
    Vector3 cam_dir  = this->GetCameraDir();
    Vector3 cam_roll = this->GetCameraRoll();
    Vector3 cam_up   = cam_dir.crossProduct(cam_roll);

    float gravity = App::GetSimTerrain()->getGravity();

    float vertacc = m_camera_gforces.dotProduct(cam_up) + gravity;
    float longacc = m_camera_gforces.dotProduct(cam_dir);
    float latacc  = m_camera_gforces.dotProduct(cam_roll);

    m_camera_local_gforces_cur = Vector3(vertacc, longacc, latacc) / gravity;

    // Let it settle before we start recording the maximum forces
    if (m_reset_timer.getMilliseconds() > 500)
    {
        m_camera_local_gforces_max.makeCeil(-m_camera_local_gforces_cur);
        m_camera_local_gforces_max.makeCeil(+m_camera_local_gforces_cur);
    }
}

void Actor::EngineTriggerHelper(int engineNumber, int type, float triggerValue)
{
    // engineNumber tells us which engine
    EngineSim* e = ar_engine; // placeholder: actors do not have multiple engines yet

    switch (type)
    {
    case TRG_ENGINE_CLUTCH:
        if (e)
            e->SetClutch(triggerValue);
        break;
    case TRG_ENGINE_BRAKE:
        ar_brake = triggerValue;
        break;
    case TRG_ENGINE_ACC:
        if (e)
            e->SetAcceleration(triggerValue);
        break;
    case TRG_ENGINE_RPM:
        // TODO: Implement setTargetRPM in the EngineSim.cpp
        break;
    case TRG_ENGINE_SHIFTUP:
        if (e)
            e->shift(1);
        break;
    case TRG_ENGINE_SHIFTDOWN:
        if (e)
            e->shift(-1);
        break;
    default:
        break;
    }
}

Actor::Actor(
    int actor_id,
    unsigned int vector_index,
    std::shared_ptr<RigDef::File> def,
    RoR::ActorSpawnRequest rq
) 
    : ar_nodes(nullptr), ar_num_nodes(0)
    , ar_beams(nullptr), ar_num_beams(0)
    , ar_shocks(nullptr), ar_num_shocks(0)
    , ar_has_active_shocks(false)
    , ar_rotators(nullptr), ar_num_rotators(0)
    , ar_wings(nullptr), ar_num_wings(0)
    , m_hud_features_ok(false)
    , ar_aileron(0)
    , m_avionic_chatter_timer(11.0f) // some pseudo random number,  doesn't matter
    , m_beacon_light_is_active(false)
    , m_blink_type(BLINK_NONE)
    , m_blinker_autoreset(false)
    , ar_brake(0.0)
    , m_camera_gforces_accu(irr::core::vector3df(0.f))
    , m_camera_gforces(irr::core::vector3df(0.f))
    , m_camera_local_gforces_cur(irr::core::vector3df(0.f))
    , m_camera_local_gforces_max(irr::core::vector3df(0.f))
    , ar_engine_hydraulics_ready(true)
    , m_custom_particles_enabled(false)
    , ar_scale(1)
    , ar_current_cinecam(-1) // -1 = external
    , ar_dashboard(nullptr)
    , ar_disable_aerodyn_turbulent_drag(false)
    , ar_elevator(0)
    , ar_aerial_flap(0)
    , ar_fusedrag(irr::core::vector3df(0.f))
    , ar_hydro_aileron_command(0)
    , ar_hydro_aileron_state(0)
    , ar_hydro_dir_command(0)
    , ar_hydro_dir_state(0)
    , ar_hydro_dir_wheel_display(0.0)
    , ar_hydro_elevator_command(0)
    , ar_hydro_elevator_state(0)
    , ar_hydro_rudder_command(0)
    , ar_hydro_rudder_state(0)
    , ar_nb_initialized(false)
    , ar_nb_skip_steps(0)
    , ar_nb_measure_steps(500)
    , ar_nb_optimum(7, std::numeric_limits<float>::max())
    , ar_nb_reference(7, std::numeric_limits<float>::max())
    , ar_nb_mass_scale(1.0f)
    , ar_nb_beams_scale(std::make_pair(1.0f, 1.0f))
    , ar_nb_shocks_scale(std::make_pair(1.0f, 1.0f))
    , ar_nb_wheels_scale(std::make_pair(1.0f, 1.0f))
    , ar_nb_beams_k_interval(std::make_pair(0.1f, 2.0f))
    , ar_nb_beams_d_interval(std::make_pair(0.1f, 2.0f))
    , ar_nb_shocks_k_interval(std::make_pair(0.1f, 8.0f))
    , ar_nb_shocks_d_interval(std::make_pair(0.1f, 8.0f))
    , ar_nb_wheels_k_interval(std::make_pair(1.0f, 1.0f))
    , ar_nb_wheels_d_interval(std::make_pair(1.0f, 1.0f))
    , m_inter_point_col_detector(nullptr)
    , m_intra_point_col_detector(nullptr)
    , ar_net_last_update_time(0)
    , m_avg_node_position_prev(rq.asr_position)
    , ar_left_mirror_angle(0.52)
    , ar_lights(1)
    , m_avg_node_velocity(irr::core::vector3df(0.f))
    , ar_custom_camera_node(-1)
    , ar_main_camera_dir_corr(irr::core::quaternion::IDENTITY)
    , ar_main_camera_node_pos(0)
    , ar_main_camera_node_dir(0)
    , ar_main_camera_node_roll(0)
    , m_preloaded_with_terrain(rq.asr_origin == RoR::ActorSpawnRequest::Origin::TERRN_DEF)
    , ar_net_source_id(0)
    , m_spawn_rotation(0.0)
    , ar_net_stream_id(0)
    , m_custom_light_toggle_countdown(0)
    , m_min_camera_radius(0.0f)
    , m_mouse_grab_move_force(0.0f)
    , m_mouse_grab_node(-1)
    , m_mouse_grab_pos(irr::core::vector3df(0.f))
    , m_net_initialized(false)
    , m_net_brake_light(false)
    , m_net_label_node(0)
    , m_net_label_mt(0)
    , m_net_reverse_light(false)
    , ar_initial_total_mass(0)
    , m_replay_pos_prev(-1)
    , ar_parking_brake(false)
    , ar_trailer_parking_brake(false)
    , m_avg_node_position(rq.asr_position)
    , m_previous_gear(0)
    , m_ref_tyre_pressure(50.0)
    , m_replay_handler(nullptr)
    , ar_replay_precision(0)
    , m_replay_timer(0)
    , ar_replay_length(10000)
    , ar_replay_mode(false)
    , ar_replay_pos(0)
    , m_reverse_light_active(false)
    , ar_right_mirror_angle(-0.52)
    , ar_rudder(0)
    , ar_update_physics(false)
    , ar_sleep_counter(0.0f)
    , m_stabilizer_shock_request(0)
    , m_stabilizer_shock_ratio(0.0)
    , m_stabilizer_shock_sleep(0.0)
    , m_total_mass(0)
    , m_water_contact(false)
    , m_water_contact_old(false)
    , m_num_axle_diffs(0)
    , m_axle_diffs{} // Init array to nullptr
    , m_num_wheel_diffs(0)
    , m_wheel_diffs{} // Init array to nullptr
    , m_has_command_beams(false)
    , m_num_command_beams(0)
    , m_load_mass(0.f)
    , m_dry_mass(0.f)
    , ar_gui_use_engine_max_rpm(false)
    , ar_autopilot(nullptr)
    , ar_is_police(false)
    , m_disable_default_sounds(false)
    , ar_engine(nullptr)
    , ar_driveable(NOT_DRIVEABLE)
    , m_skid_trails{} // Init array to nullptr
    , ar_collision_range(DEFAULT_COLLISION_RANGE)
    , ar_instance_id(actor_id)
    , ar_vector_index(vector_index)
    , ar_rescuer_flag(false)
    , m_antilockbrake(false)
    , m_tractioncontrol(false)
    , ar_forward_commands(false)
    , ar_import_commands(false)
    , ar_toggle_ties(false)
    , ar_airbrakes{} // Init array to nullptr
    , ar_cabs{} // Init array to 0
    , ar_num_cabs(0)
    , ar_num_contactable_nodes(0)
    , ar_num_contacters(0)
    , ar_screwprops{} // Init array to nullptr
    , ar_num_screwprops(0)
    , ar_num_camera_rails(0)
    , ar_aeroengines() // Zero-init array
    , ar_num_aeroengines() // Zero-init
    , ar_pressure_beams() // Zero-init array
    , ar_free_pressure_beam() // Zero-init
    , ar_wheels() // array
    , ar_num_wheels() // int
    , m_avg_proped_wheel_radius(0.2f)
    , ar_filename(rq.asr_filename)
    , m_section_config(rq.asr_config)
    , m_ongoing_reset(false)
    , ar_top_speed(0.0f)
    , ar_last_fuzzy_ground_model(nullptr)
    , m_transfer_case(nullptr)
{
}

float Actor::getSteeringAngle()
{
    return ar_hydro_dir_command;
}

std::vector<authorinfo_t> Actor::getAuthors()
{
    return authors;
}

std::vector<std::string> Actor::getDescription()
{
    return description;
}

void Actor::setMass(float m)
{
    m_dry_mass = m;
}

float Actor::getMinimalCameraRadius()
{
    return m_min_camera_radius;
}

Replay* Actor::getReplay()
{
    return m_replay_handler;
}

Vector3 Actor::getNodePosition(int nodeNumber)
{
    if (nodeNumber >= 0 && nodeNumber < ar_num_nodes)
    {
        return ar_nodes[nodeNumber].AbsPosition;
    }
    else
    {
        return irr::core::vector3df();
    }
}
