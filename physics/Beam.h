/*
    This source file is part of Rigs of Rods
    Copyright 2005-2012 Pierre-Michel Ricordel
    Copyright 2007-2012 Thomas Fischer
    Copyright 2016-2018 Petr Ohlidal & contributors

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

//#include "Application.h"
#include "BeamData.h"
#include "CmdKeyInertia.h"
#include <aabbox3d.h>
#include <quaternion.h>
//#include "GfxActor.h"
//#include "PerVehicleCameraContext.h"
//#include "RigDef_Prerequisites.h"

 

/// Softbody object; can be anything from soda can to a space shuttle
/// Monsterclass; contains logic related to physics, network, sound, threading, rendering.
/// NOTE: Until 01/2018, this class was named `Beam` (and was derived from `rig_t`), you may find references to this.
class Actor 
//: public ZeroedMemoryAllocator
{
    friend class ActorSpawner;
    friend class RoR::ActorManager;
    friend class RoR::GfxActor; // Temporary until all visuals are moved there. ~ only_a_ptr, 2018
public:

    enum class SimState
    {
        LOCAL_SIMULATED,  //!< simulated (local) actor
        NETWORKED_OK,     //!< not simulated (remote) actor
        LOCAL_SLEEPING,   //!< sleeping (local) actor
    };

    Actor(
          int actor_id
        , unsigned int vector_index
        //, std::shared_ptr<RigDef::File> def
        //, RoR::ActorSpawnRequest rq
        );

    ~Actor();

    void              ApplyNodeBeamScales();
    bool              AddTyrePressure(float v);
    float             GetTyrePressure();
    float             getRotation();
    irr::core::vector3df     getDirection();
    irr::core::vector3df     getPosition();
    void              UpdateInitPosition();
    /// Moves the actor.
    /// @param translation Offset to move in world coordinates
    /// @param setInitPosition Set initial positions of nodes to current position?
    void              ResetPosition(irr::core::vector3df translation, bool setInitPosition);
    void              ResetPosition(float px, float pz, bool setInitPosition, float miny);
    void              RequestRotation(float rotation, irr::core::vector3df center) { m_rotation_request += rotation; m_rotation_request_center = center; };
    void              RequestAngleSnap(int division) { m_anglesnap_request = division; };
    void              RequestTranslation(irr::core::vector3df translation) { m_translation_request += translation; };
    irr::core::vector3df     GetRotationCenter();
    float             GetMinHeight(bool skip_virtual_nodes=true);
    float             GetMaxHeight(bool skip_virtual_nodes=true);
    float             GetHeightAboveGround(bool skip_virtual_nodes=true);
    float             GetHeightAboveGroundBelow(float height, bool skip_virtual_nodes=true);
    bool              ReplayStep();
    void              ForceFeedbackStep(int steps);
    void              HandleInputEvents(float dt);
    void              HandleMouseMove(int node, irr::core::vector3df pos, float force); //!< Event handler
    void              ToggleLights();                      //!< Event handler
    void              ToggleTies(int group=-1);
    void              ToggleRopes(int group=-1);            //!< Event handler
    void              ToggleHooks(int group=-1, hook_states mode=HOOK_TOGGLE, int node_number=-1); //!< Event handler
    void              EngineTriggerHelper(int engineNumber, int type, float triggerValue);
    void              ToggleSlideNodeLock();
    void              ToggleCustomParticles();
    void              ToggleTransferCaseMode();            //! Toggles between 2WD and 4WD mode
    void              ToggleTransferCaseGearRatio();       //! Toggles between Hi and Lo mode
    std::string       GetTransferCaseName();               //! Gets the current transfer case mode name (4WD Hi, ...)
    void              DisplayTransferCaseMode();           //! Displays the current transfer case mode
    void              ToggleParkingBrake();                //!< Event handler
    void              ToggleAntiLockBrake();               //!< Event handler
    void              ToggleTractionControl();             //!< Event handler
    void              ToggleCruiseControl();               //!< Event handler
    void              ToggleBeacons();                     //!< Event handler
    void              setReplayMode(bool rm);              //!< Event handler; toggle replay mode.
    bool              Intersects(Actor* actor, irr::core::vector3df offset = irr::core::vector3df(0.f));  //!< Slow intersection test
    /// Moves the actor at most 'direction.length()' meters towards 'direction' to resolve any collisions
    void              resolveCollisions(irr::core::vector3df direction);
    /// Auto detects an ideal collision avoidance direction (front, back, left, right, up)
    /// Then moves the actor at most 'max_distance' meters towards that direction to resolve any collisions
    void              resolveCollisions(float max_distance, bool consider_up);
    void              updateSkidmarks();                   //!< Creates or updates skidmarks. No side effects.
    void              prepareInside(bool inside);          //!< Prepares vehicle for in-cabin camera use.

    void              updateVisual(float dt=0);
    void              ScaleActor(float value);
    irr::core::vector3df     GetGForcesCur() { return m_camera_local_gforces_cur; };
    irr::core::vector3df     GetGForcesMax() { return m_camera_local_gforces_max; };
    float             getSteeringAngle();
    float             getMinCameraRadius() { return m_min_camera_radius; };
    std::string       GetActorDesignName() { return ar_design_name; };
    std::string       GetActorFileName() { return ar_filename; };
    std::string       GetActorFileHash() { return ar_filehash; };
    int               GetActorType() { return ar_driveable; };
    int               GetNumActiveConnectedBeams(int nodeid);     //!< Returns the number of active (non bounded) beams connected to a node
    void              NotifyActorCameraChanged();                 //!< Logic: sound, display; Notify this vehicle that camera changed;
    void              StopAllSounds();
    void              UnmuteAllSounds();
    float             getTotalMass(bool withLocked=true);
    float             getAvgPropedWheelRadius() { return m_avg_proped_wheel_radius; };

    void              setMass(float m);
    bool              isTied();
    bool              isLocked(); 
    bool              hasSlidenodes() { return !m_slidenodes.empty(); };
    void              UpdateBoundingBoxes();
    void              calculateAveragePosition();
    void              UpdatePhysicsOrigin();
    void              SoftReset();
    void              SyncReset(bool reset_position);      //!< this one should be called only synchronously (without physics running in background)

    std::vector<authorinfo_t>     getAuthors();
    std::vector<std::string>      getDescription();
    std::string     GetSectionConfig()                 { return m_section_config; }
    std::vector<Actor*> GetAllLinkedActors()            { return m_linked_actors; }; //!< Returns a list of all connected (hooked) actors
    irr::core::vector3df     GetCameraDir()                    { return (ar_nodes[ar_main_camera_node_pos].RelPosition - ar_nodes[ar_main_camera_node_dir].RelPosition).normalisedCopy(); }
    irr::core::vector3df     GetCameraRoll()                   { return (ar_nodes[ar_main_camera_node_pos].RelPosition - ar_nodes[ar_main_camera_node_roll].RelPosition).normalisedCopy(); }
    irr::core::vector3df     GetFFbBodyForces() const          { return m_force_sensors.out_body_forces; }
    //RoR::GfxActor*    GetGfxActor()                     { return m_gfx_actor.get(); }
    
    irr::core::vector3df     getNodePosition(int nodeNumber);     //!< Returns world position of node
    float        	getMinimalCameraRadius();
    Replay*           getReplay();
    bool              isPreloadedWithTerrain() const    { return m_preloaded_with_terrain; };
    bool              isBeingReset() const              { return m_ongoing_reset; };
    int               GetNumNodes() const               { return ar_num_nodes; }
    //CacheEntry*       GetUsedSkin() const               { return m_used_skin_entry; }
    //void              SetUsedSkin(CacheEntry* skin)     { m_used_skin_entry = skin; }
    float             getSpeed()                        { return m_avg_node_velocity.length(); };
    irr::core::vector3df     getVelocity() const               { return m_avg_node_velocity; }; //!< average actor velocity, calculated using the actor positions of the last two frames
#ifdef USE_ANGELSCRIPT
    // we have to add this to be able to use the class as reference inside scripts
    void              addRef()                          {};
    void              release()                         {};
#endif

    // -------------------- Public data -------------------- //

    node_t*           ar_nodes;
    int               ar_num_nodes;
    beam_t*           ar_beams;
    int               ar_num_beams;
    std::vector<beam_t*> ar_inter_beams;    //!< Beams connecting 2 actors

    rotator_t*        ar_rotators;
    int               ar_num_rotators;


    std::vector<std::string>  description;
    std::vector<authorinfo_t> authors;

    
    irr::core::aabbox3df      ar_bounding_box;     //!< standard bounding box (surrounds all nodes of an actor)
    irr::core::aabbox3df      ar_predicted_bounding_box;
    float                     ar_initial_total_mass;
    std::vector<float>        ar_initial_node_masses;
    std::vector<irr::core::vector3df>     ar_initial_node_positions;
    std::vector<std::pair<float, float>> ar_initial_beam_defaults;
    std::vector<float>             ar_minimass; //!< minimum node mass in Kg
    std::vector<std::vector<int>>  ar_node_to_node_connections;
    std::vector<std::vector<int>>  ar_node_to_beam_connections;
    std::vector<irr::core::aabbox3df>  ar_collision_bounding_boxes; //!< smart bounding boxes, used for determining the state of an actor (every box surrounds only a subset of nodes)
    std::vector<irr::core::aabbox3df>  ar_predicted_coll_bounding_boxes;
    int               ar_num_contactable_nodes; //!< Total number of nodes which can contact ground or cabs
    int               ar_num_contacters; //!< Total number of nodes which can selfcontact cabs
    int               ar_num_wheels;
    command_t         ar_command_key[MAX_COMMANDS + 10]; // 0 for safety
    int               ar_num_custom_particles;
    soundsource_t     ar_soundsources[MAX_SOUNDSCRIPTS_PER_TRUCK];
    int               ar_num_soundsources;
    int               ar_pressure_beams[MAX_PRESSURE_BEAMS];
    int               ar_free_pressure_beam;

    int               ar_num_aeroengines;

    int               ar_num_screwprops;
    int               ar_cabs[MAX_CABS*3];
    int               ar_num_cabs;
    int               ar_collcabs[MAX_CABS];
    collcab_rate_t    ar_inter_collcabrate[MAX_CABS];
    collcab_rate_t    ar_intra_collcabrate[MAX_CABS];
    int               ar_num_collcabs;
    int               ar_buoycabs[MAX_CABS];
    int               ar_buoycab_types[MAX_CABS];
    int               ar_num_buoycabs;
    int               ar_camera_rail[MAX_CAMERARAIL]; //!< Nodes defining camera-movement spline
    int               ar_num_camera_rails;
    bool              ar_hide_in_actor_list;      //!< Hide in list of spawned actors (available in top menubar). Useful for fixed-place machinery, i.e. cranes.
    std::string      ar_design_name;             //!< Name of the vehicle/machine/object this actor represents
    float             ar_anim_previous_crank;     //!< For 'animator' with flag 'torque'
    float             alb_ratio;          //!< Anti-lock brake attribute: Regulating force
    float             alb_minspeed;       //!< Anti-lock brake attribute;
    bool              alb_mode;           //!< Anti-lock brake state; Enabled? {1/0}
    float             alb_pulse_time;     //!< Anti-lock brake attribute;
    bool              alb_pulse_state;    //!< Anti-lock brake state;
    bool              alb_nodash;         //!< Anti-lock brake attribute: Hide the dashboard indicator?
    bool              alb_notoggle;       //!< Anti-lock brake attribute: Disable in-game toggle?
    float             alb_timer;          //!< Anti-lock brake state;
    float             tc_ratio;           //!< Traction control attribute: Regulating force
    bool              tc_mode;            //!< Traction control state; Enabled? {1/0}
    float             tc_pulse_time;      //!< Traction control attribute;
    bool              tc_pulse_state;     //!< Traction control state;
    bool              tc_nodash;          //!< Traction control attribute; Hide the dashboard indicator?
    bool              tc_notoggle;        //!< Traction control attribute; Disable in-game toggle?
    float             tc_timer;           //!< Traction control state;
    float             ar_anim_shift_timer;//!< For 'animator' with flag 'shifter'
    bool              cc_mode;            //!< Cruise Control
    bool              cc_can_brake;       //!< Cruise Control
    float             cc_target_rpm;      //!< Cruise Control
    float             cc_target_speed;    //!< Cruise Control
    float             cc_target_speed_lower_limit; //!< Cruise Control
    //std::deque<float> cc_accs;            //!< Cruise Control
    bool              sl_enabled;         //!< Speed limiter;
    float             sl_speed_limit;     //!< Speed limiter;
    int               ar_extern_camera_mode;
    int               ar_extern_camera_node;
    int               ar_exhaust_pos_node; //!< Old-format exhaust (one per vehicle) emitter node
    int               ar_exhaust_dir_node; //!< Old-format exhaust (one per vehicle) backwards direction node
    int               ar_instance_id;              //!< Static attr; session-unique ID
    unsigned int      ar_vector_index;             //!< Sim attr; actor element index in std::vector<m_actors>
    int               ar_driveable;                //!< Sim attr; marks vehicle type and features
    EngineSim*        ar_engine;
    int               ar_cinecam_node[MAX_CAMERAS];//!< Sim attr; Cine-camera node indexes
    int               ar_num_cinecams;             //!< Sim attr;
    Autopilot*        ar_autopilot;
    float             ar_brake_force;              //!< Physics attr; filled at spawn
    float             ar_speedo_max_kph;           //!< GUI attr
    irr::core::vector3df     ar_origin;                   //!< Physics state; base position for softbody nodes
    int               ar_num_cameras;
    irr::core::quaternion  ar_main_camera_dir_corr;     //!< Sim attr;
    int               ar_main_camera_node_pos;     //!< Sim attr; ar_camera_node_pos[0]  >= 0 ? ar_camera_node_pos[0]  : 0
    int               ar_main_camera_node_dir;     //!< Sim attr; ar_camera_node_dir[0]  >= 0 ? ar_camera_node_dir[0]  : 0
    int               ar_main_camera_node_roll;    //!< Sim attr; ar_camera_node_roll[0] >= 0 ? ar_camera_node_roll[0] : 0
    int               ar_camera_node_pos[MAX_CAMERAS]; //!< Physics attr; 'camera' = frame of reference; origin node
    int               ar_camera_node_dir[MAX_CAMERAS]; //!< Physics attr; 'camera' = frame of reference; back node
    int               ar_camera_node_roll[MAX_CAMERAS]; //!< Physics attr; 'camera' = frame of reference; left node
    bool              ar_camera_node_roll_inv[MAX_CAMERAS]; //!< Physics attr; 'camera' = frame of reference; indicates roll node is right instead of left
    float             ar_posnode_spawn_height;
    VehicleAI*        ar_vehicle_ai;
    float             ar_scale;               //!< Physics state; scale of the actor (nominal = 1.0)
    float        ar_brake;               //!< Physics state; braking intensity
    float             ar_wheel_speed;         //!< Physics state; wheel speed in m/s
    float             ar_wheel_spin;          //!< Physics state; wheel speed in radians/s
    float             ar_avg_wheel_speed;     //!< Physics state; avg wheel speed in m/s
    float             ar_hydro_dir_command;
    float             ar_hydro_dir_state;
    float        ar_hydro_dir_wheel_display;
    float             ar_hydro_aileron_command;
    float             ar_hydro_aileron_state;
    float             ar_hydro_rudder_command;
    float             ar_hydro_rudder_state;
    float             ar_hydro_elevator_command;
    float             ar_hydro_elevator_state;
    float        ar_replay_precision;            //!< Sim attribute; determined at startup
    int               ar_replay_length;               //!< Sim attribute; clone of GVar 'sim_replay_length'
    int               ar_replay_pos;                  //!< Sim state
    float             ar_sleep_counter;               //!< Sim state; idle time counter
    ground_model_t*   ar_submesh_ground_model;
    bool              ar_parking_brake;
    bool              ar_trailer_parking_brake;
    int               ar_lights;                      //!< boolean 1/0
    float             ar_left_mirror_angle;           //!< Sim state; rear view mirror angle
    float             ar_right_mirror_angle;          //!< Sim state; rear view mirror angle
    float             ar_elevator;                    //!< Sim state; aerial controller
    float             ar_rudder;                      //!< Sim state; aerial/marine controller
    float             ar_aileron;                     //!< Sim state; aerial controller
    int               ar_aerial_flap;                 //!< Sim state; state of aircraft flaps (values: 0-5)
    irr::core::vector3df     ar_fusedrag;                    //!< Physics state
    int               ar_current_cinecam;             //!< Sim state; index of current CineCam (-1 if using 3rd-person camera)
    int               ar_custom_camera_node;          //!< Sim state; custom tracking node for 3rd-person camera
    std::string       ar_filename;                    //!< Attribute; filled at spawn
    std::string       ar_filehash;                    //!< Attribute; filled at spawn
	

    SimState          ar_sim_state;                   //!< Sim state
    float             ar_collision_range;             //!< Physics attr
    float             ar_top_speed;                   //!< Sim state
    ground_model_t*   ar_last_fuzzy_ground_model;     //!< GUI state

    // Realtime node/beam structure editing helpers
    void                    SearchBeamDefaults();     //!< Searches for more stable beam defaults
    bool                    ar_nb_initialized;
    std::vector<float>      ar_nb_optimum;            //!< Temporary storage of the optimum search result
    std::vector<float>      ar_nb_reference;          //!< Temporary storage of the reference search result
    int                     ar_nb_skip_steps;         //!< Amount of physics steps to be skipped before measuring
    int                     ar_nb_measure_steps;      //!< Amount of physics steps to be measured
    float                   ar_nb_mass_scale;         //!< Global mass scale (affects all nodes the same way)
    std::pair<float, float> ar_nb_beams_scale;        //!< Scales for springiness & damping of regular beams
    std::pair<float, float> ar_nb_shocks_scale;       //!< Scales for springiness & damping of shock beams
    std::pair<float, float> ar_nb_wheels_scale;       //!< Scales for springiness & damping of wheel / rim beams
    std::pair<float, float> ar_nb_beams_d_interval;   //!< Search interval for springiness & damping of regular beams
    std::pair<float, float> ar_nb_beams_k_interval;   //!< Search interval for springiness & damping of regular beams
    std::pair<float, float> ar_nb_shocks_d_interval;  //!< Search interval for springiness & damping of shock beams
    std::pair<float, float> ar_nb_shocks_k_interval;  //!< Search interval for springiness & damping of shock beams
    std::pair<float, float> ar_nb_wheels_d_interval;  //!< Search interval for springiness & damping of wheel / rim beams
    std::pair<float, float> ar_nb_wheels_k_interval;  //!< Search interval for springiness & damping of wheel / rim beams

    // Bit flags
    bool ar_left_blink_on:1;  //!< Gfx state; turn signals
    bool ar_right_blink_on:1; //!< Gfx state; turn signals
    bool ar_warn_blink_on:1;  //!< Gfx state; turn signals
    bool ar_update_physics:1; //!< Physics state; Should this actor be updated (locally) in the next physics step?
    bool ar_disable_aerodyn_turbulent_drag:1; //!< Physics state
    bool ar_engine_hydraulics_ready:1; //!< Sim state; does engine have enough RPM to power hydraulics?
    bool ar_gui_use_engine_max_rpm:1;  //!< Gfx attr
    bool ar_hydro_speed_coupling:1;
    bool ar_collision_relevant:1;      //!< Physics state;
    bool ar_replay_mode:1;      //!< Sim state
    bool ar_is_police:1;        //!< Gfx/sfx attr
    bool ar_rescuer_flag:1;     //!< Gameplay attr; defined in truckfile. TODO: Does anybody use this anymore?
    bool ar_forward_commands:1; //!< Sim state
    bool ar_import_commands:1;  //!< Sim state
    bool ar_toggle_ropes:1;     //!< Sim state
    bool ar_toggle_ties:1;      //!< Sim state
    bool ar_physics_paused:1;   //!< Sim state

private:

    bool              CalcForcesEulerPrepare(bool doUpdate); //!< TIGHT LOOP; Physics;
    void              CalcAircraftForces(bool doUpdate);   //!< TIGHT LOOP; Physics;
    void              CalcAnimatedProps(bool doUpdate);    //!< TIGHT LOOP; Physics;
    void              CalcForcesEulerCompute(bool doUpdate, int num_steps); //!< TIGHT LOOP; Physics;
    void              CalcAnimators(const int flag_state, float &cstate, int &div, float timer, const float lower_limit, const float upper_limit, const float option3); //!< TIGHT LOOP; Physics;
    void              CalcBeams(bool trigger_hooks);       //!< TIGHT LOOP; Physics;
    void              CalcBeamsInterActor();               //!< TIGHT LOOP; Physics;
    void              CalcBuoyance(bool doUpdate);         //!< TIGHT LOOP; Physics;
    void              CalcCommands(bool doUpdate);         //!< TIGHT LOOP; Physics;
    void              CalcCabCollisions();                 //!< TIGHT LOOP; Physics;
    void              CalcDifferentials();                 //!< TIGHT LOOP; Physics;
    void              CalcForceFeedback(bool doUpdate);    //!< TIGHT LOOP; Physics;
    void              CalcFuseDrag();                      //!< TIGHT LOOP; Physics;
    void              CalcHooks();                         //!< TIGHT LOOP; Physics;
    void              CalcHooks(bool doUpdate);            //!< TIGHT LOOP; Physics;
    void              CalcHydros();                        //!< TIGHT LOOP; Physics;
    void              CalcMouse();                         //!< TIGHT LOOP; Physics;
    void              CalcNodes();                         //!< TIGHT LOOP; Physics;
    void              CalcReplay();                        //!< TIGHT LOOP; Physics;
    void              CalcRopes();                         //!< TIGHT LOOP; Physics;
    void              CalcShocks(bool doUpdate, int num_steps); //!< TIGHT LOOP; Physics;
    void              CalcShocks2(int i, float difftoBeamL, float &k, float &d, float v);
    void              CalcShocks3(int i, float difftoBeamL, float &k, float &d, float v);
    void              CalcTriggers(int i, float difftoBeamL, bool update_hooks);
    void              CalcSlideNodes();                    //!< TIGHT LOOP; Physics;
    void              CalcTies();                          //!< TIGHT LOOP; Physics;
    void              CalcTruckEngine(bool doUpdate);      //!< TIGHT LOOP; Physics;
    void              CalcAxles();                         //!< TIGHT LOOP; Physics;
    void              CalcWheels(bool doUpdate, int num_steps); //!< TIGHT LOOP; Physics;

    void              DetermineLinkedActors();
    void              RecalculateNodeMasses(float total); //!< Previously 'calc_masses2()'
    void              calcNodeConnectivityGraph();
    void              AddInterActorBeam(beam_t* beam, Actor* a, Actor* b);
    void              RemoveInterActorBeam(beam_t* beam);
    void              DisjoinInterActorBeams();            //!< Destroys all inter-actor beams which are connected with this actor
    void              autoBlinkReset();                    //!< Resets the turn signal when the steering wheel is turned back.
    void              sendStreamSetup();
    void              UpdateSlideNodeForces(const float delta_time_sec); //!< calculate and apply Corrective forces
    void              resetSlideNodePositions();           //!< Recalculate SlideNode positions
    void              resetSlideNodes();                   //!< Reset all the SlideNodes
    void              updateSlideNodePositions();          //!< incrementally update the position of all SlideNodes
    void              ResetAngle(float rot);
    void              calculateLocalGForces();             //!< Derive the truck local g-forces from the global ones
    /// Virtually moves the actor at most 'direction.length()' meters towards 'direction' trying to resolve any collisions
    /// Returns a minimal offset by which the actor needs to be moved to resolve any collisions
    //  Both PointColDetectors need to be updated accordingly before calling this
    irr::core::vector3df     calculateCollisionOffset(irr::core::vector3df direction);
    /// @param actor which actor to retrieve the closest Rail from
    /// @param node which SlideNode is being checked against
    /// @return a pair containing the rail, and the distant to the SlideNode
    std::pair<RailGroup*, float> GetClosestRailOnActor( Actor* actor, const SlideNode& node);

    // -------------------- data -------------------- //

    std::vector<std::shared_ptr<Task>> m_flexbody_tasks;   //!< Gfx state
    //std::shared_ptr<RigDef::File>      m_definition;
    //std::unique_ptr<RoR::GfxActor>     m_gfx_actor;
    //RoR::PerVehicleCameraContext       m_camera_context;
    std::string                       m_section_config;
    std::vector<SlideNode>             m_slidenodes;       //!< all the SlideNodes available on this actor
    std::vector<RailGroup*>            m_railgroups;       //!< all the available RailGroups for this actor
    //std::vector<Ogre::Entity*>         m_deletion_entities;    //!< For unloading vehicle; filled at spawn.
    
	//std::vector<irr::scene::ISceneNode*>      m_deletion_scene_nodes; //!< For unloading vehicle; filled at spawn.
    
	
	int               m_proped_wheel_pairs[MAX_WHEELS];    //!< Physics attr; For inter-differential locking
    int               m_num_proped_wheels;          //!< Physics attr, filled at spawn - Number of propelled wheels.
    float             m_avg_proped_wheel_radius;    //!< Physics attr, filled at spawn - Average proped wheel radius.
    float             m_avionic_chatter_timer;      //!< Sound fx state
    PointColDetector* m_inter_point_col_detector;   //!< Physics
    PointColDetector* m_intra_point_col_detector;   //!< Physics
    std::vector<Actor*>  m_linked_actors;           //!< Sim state; other actors linked using 'hooks'
    irr::core::vector3df     m_avg_node_position;          //!< average node position
    float        m_min_camera_radius;
    irr::core::vector3df     m_avg_node_position_prev;
    irr::core::vector3df     m_avg_node_velocity;          //!< average node velocity (compared to the previous frame step)
    float        m_replay_timer;               //!< Sim state

    //Replay*           m_replay_handler;
    float             m_total_mass;            //!< Physics state; total mass in Kg
    int               m_mouse_grab_node;       //!< Sim state; node currently being dragged by user
    irr::core::vector3df     m_mouse_grab_pos;
    float             m_mouse_grab_move_force;
    float             m_spawn_rotation;
    std::string   m_net_username;
    //Ogre::Timer       m_reset_timer;
    float             m_custom_light_toggle_countdown; //!< Input system helper status
    irr::core::vector3df     m_rotation_request_center;
    float             m_rotation_request;         //!< Accumulator
    int               m_anglesnap_request;        //!< Accumulator
    irr::core::vector3df     m_translation_request;      //!< Accumulator
    irr::core::vector3df     m_camera_gforces_accu;      //!< Accumulator for 'camera' G-forces
    irr::core::vector3df     m_camera_gforces;           //!< Physics state (global)
    irr::core::vector3df     m_camera_local_gforces_cur; //!< Physics state (camera local)
    irr::core::vector3df     m_camera_local_gforces_max; //!< Physics state (camera local)
    float             m_ref_tyre_pressure;        //!< Physics state
    float             m_stabilizer_shock_ratio;   //!< Physics state
    int               m_stabilizer_shock_request; //!< Physics state; values: { -1, 0, 1 }
    Differential*     m_axle_diffs[1+MAX_WHEELS/2];//!< Physics
    int               m_num_axle_diffs;           //!< Physics attr
    Differential*     m_wheel_diffs[MAX_WHEELS/2];//!< Physics
    int               m_num_wheel_diffs;          //!< Physics attr
    TransferCase*     m_transfer_case;            //!< Physics
    float             m_net_node_compression;  //!< Sim attr;
    int               m_net_first_wheel_node;  //!< Network attr; Determines data buffer layout
    int               m_net_node_buf_size;     //!< Network attr; buffer size
    int               m_net_buffer_size;       //!< Network attr; buffer size
    int               m_wheel_node_count;      //!< Static attr; filled at spawn
    int               m_replay_pos_prev;       //!< Sim state
    int               m_previous_gear;         //!< Sim state; land vehicle shifting
    float             m_handbrake_force;       //!< Physics attr; defined in truckfile
    node_t*           m_fusealge_front;        //!< Physics attr; defined in truckfile
    node_t*           m_fusealge_back;         //!< Physics attr; defined in truckfile
    float             m_fusealge_width;        //!< Physics attr; defined in truckfile
    float             m_odometer_total;        //!< GUI state
    float             m_odometer_user;         //!< GUI state
    int               m_num_command_beams;     //!< TODO: Remove! Spawner context only; likely unused feature
    float             m_load_mass;             //!< Physics attr; predefined load mass in Kg
    int               m_masscount;             //!< Physics attr; Number of nodes loaded with l option
    float             m_dry_mass;              //!< Physics attr;



    //CacheEntry*       m_used_skin_entry;       //!< Graphics

    bool              m_ongoing_reset;         //!< Hack to prevent position/rotation creep during interactive truck reset
    bool              m_has_axles_section;     //!< Temporary (legacy parsing helper) until central diffs are implemented

    bool m_hud_features_ok:1;      //!< Gfx state; Are HUD features matching actor's capabilities?
    bool m_slidenodes_locked:1;    //!< Physics state; Are SlideNodes locked?
    bool m_blinker_autoreset:1;    //!< Gfx state; We're steering - when we finish, the blinker should turn off


    bool m_has_command_beams:1;    //!< Physics attr;

    bool m_preloaded_with_terrain:1;        //!< Spawn context (TODO: remove!)
    bool m_beam_break_debug_enabled:1;  //!< Logging state
    bool m_beam_deform_debug_enabled:1; //!< Logging state
    bool m_trigger_debug_enabled:1;     //!< Logging state


    struct VehicleForceSensors
    {
        inline void Reset()
        {
            accu_body_forces    = irr::core::vector3df(0.f);
            accu_hydros_forces  = 0;
            out_body_forces     = irr::core::vector3df(0.f);
            out_hydros_forces   = 0;
        };

        irr::core::vector3df accu_body_forces;
        float         accu_hydros_forces;
        irr::core::vector3df out_body_forces;
        float         out_hydros_forces;
    } m_force_sensors; //!< Data for ForceFeedback devices

    
    
};
