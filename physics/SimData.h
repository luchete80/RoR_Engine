/*
    This source file is part of Rigs of Rods
    Copyright 2005-2012 Pierre-Michel Ricordel
    Copyright 2007-2012 Thomas Fischer
    Copyright 2013-2020 Petr Ohlidal

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

/// @file
/// @author Thomas Fischer
/// @date   30th of April 2010
/// @brief Core data structures for simulation.

#pragma once

#include "ForwardDeclarations.h"
#include "SimConstants.h"
//#include "BitFlags.h"
#include "CmdKeyInertia.h"

#include <vector3d.h>

#include <memory>
// #include <Ogre.h>
// #include <OgreUTFString.h>
// #include <rapidjson/document.h>

namespace RoR {

enum CollisionEventFilter: short
{
    EVENT_NONE = 0,
    EVENT_ALL,
    EVENT_AVATAR,
    EVENT_TRUCK,
    EVENT_AIRPLANE,
    EVENT_BOAT,
    EVENT_DELETE
};

enum HookAction
{
    HOOK_LOCK=0,
    HOOK_UNLOCK,
    HOOK_TOGGLE,
    MOUSE_HOOK_TOGGLE,
};

enum BeamType: short
{
    BEAM_NORMAL,
    BEAM_HYDRO,
    BEAM_VIRTUAL,         //!< Excluded from mass calculations, visuals permanently disabled
};

enum HookState
{
    UNLOCKED,       //!< lock not locked
    PREUNLOCK,      //!< preunlocking, inter-actor beam deletion in progress
    PRELOCK,        //!< prelocking, attraction forces in action
    LOCKED          //!< lock locked.
};

enum ActorType //!< Aka 'Driveable'
{
    NOT_DRIVEABLE,  //!< not drivable at all
    TRUCK,          //!< its a truck (or other land vehicle)
    AIRPLANE,       //!< its an airplane
    BOAT,           //!< its a boat
    MACHINE,        //!< its a machine
    AI,             //!< machine controlled by an Artificial Intelligence
};

enum SpecialBeam: short
{
    NOSHOCK,        //!< not a shock
    SHOCK1,         //!< shock1
    SHOCK2,         //!< shock2
    SHOCK3,         //!< shock3
    TRIGGER,        //!< trigger
    SUPPORTBEAM,    //!<
    ROPE            //!<
};

enum BlinkType //!< Turn signal
{
    BLINK_NONE,     //!<
    BLINK_LEFT,     //!<
    BLINK_RIGHT,    //!<
    BLINK_WARN      //!<
};

// enum HydroFlags
// {
    // HYDRO_FLAG_SPEED        = BITMASK(1),
    // HYDRO_FLAG_DIR          = BITMASK(2),
    // HYDRO_FLAG_AILERON      = BITMASK(3),
    // HYDRO_FLAG_RUDDER       = BITMASK(4),
    // HYDRO_FLAG_ELEVATOR     = BITMASK(5),
    // HYDRO_FLAG_REV_AILERON  = BITMASK(6),
    // HYDRO_FLAG_REV_RUDDER   = BITMASK(7),
    // HYDRO_FLAG_REV_ELEVATOR = BITMASK(8),
// };

// enum AnimFlags
// {
    // ANIM_FLAG_AIRSPEED      = BITMASK(1),
    // ANIM_FLAG_VVI           = BITMASK(2),
    // ANIM_FLAG_ALTIMETER     = BITMASK(3),
    // ANIM_FLAG_AOA           = BITMASK(4),
    // ANIM_FLAG_FLAP          = BITMASK(5),
    // ANIM_FLAG_AIRBRAKE      = BITMASK(6),
    // ANIM_FLAG_ROLL          = BITMASK(7),
    // ANIM_FLAG_PITCH         = BITMASK(8),
    // ANIM_FLAG_THROTTLE      = BITMASK(9),
    // ANIM_FLAG_RPM           = BITMASK(10),
    // ANIM_FLAG_ACCEL         = BITMASK(11),
    // ANIM_FLAG_BRAKE         = BITMASK(12),
    // ANIM_FLAG_CLUTCH        = BITMASK(13),
    // ANIM_FLAG_TACHO         = BITMASK(14),
    // ANIM_FLAG_SPEEDO        = BITMASK(15),
    // ANIM_FLAG_PBRAKE        = BITMASK(16),
    // ANIM_FLAG_TURBO         = BITMASK(17),
    // ANIM_FLAG_SHIFTER       = BITMASK(18),
    // ANIM_FLAG_AETORQUE      = BITMASK(19),
    // ANIM_FLAG_AEPITCH       = BITMASK(20),
    // ANIM_FLAG_AESTATUS      = BITMASK(21),
    // ANIM_FLAG_TORQUE        = BITMASK(22),
    // ANIM_FLAG_HEADING       = BITMASK(23),
    // ANIM_FLAG_DIFFLOCK      = BITMASK(24),
    // ANIM_FLAG_STEERING      = BITMASK(25),
    // ANIM_FLAG_EVENT         = BITMASK(26),
    // ANIM_FLAG_AILERONS      = BITMASK(27),
    // ANIM_FLAG_ARUDDER       = BITMASK(28),
    // ANIM_FLAG_BRUDDER       = BITMASK(29),
    // ANIM_FLAG_BTHROTTLE     = BITMASK(30),
    // ANIM_FLAG_PERMANENT     = BITMASK(31),
    // ANIM_FLAG_ELEVATORS     = BITMASK(32),
// };

// enum AnimModes
// {
    // ANIM_MODE_ROTA_X        = BITMASK(1),
    // ANIM_MODE_ROTA_Y        = BITMASK(2),
    // ANIM_MODE_ROTA_Z        = BITMASK(3),
    // ANIM_MODE_OFFSET_X      = BITMASK(4),
    // ANIM_MODE_OFFSET_Y      = BITMASK(5),
    // ANIM_MODE_OFFSET_Z      = BITMASK(6),
    // ANIM_MODE_AUTOANIMATE   = BITMASK(7),
    // ANIM_MODE_NOFLIP        = BITMASK(8),
    // ANIM_MODE_BOUNCE        = BITMASK(9),
// };

// enum ShockFlags
// {
    // SHOCK_FLAG_NORMAL           = BITMASK(1),
    // SHOCK_FLAG_LACTIVE          = BITMASK(3),
    // SHOCK_FLAG_RACTIVE          = BITMASK(4),
    // SHOCK_FLAG_ISSHOCK2         = BITMASK(5),
    // SHOCK_FLAG_ISSHOCK3         = BITMASK(6),
    // SHOCK_FLAG_SOFTBUMP         = BITMASK(7),
    // SHOCK_FLAG_ISTRIGGER        = BITMASK(8),
    // SHOCK_FLAG_TRG_BLOCKER      = BITMASK(9),
    // SHOCK_FLAG_TRG_CMD_SWITCH   = BITMASK(10),
    // SHOCK_FLAG_TRG_CMD_BLOCKER  = BITMASK(11),
    // SHOCK_FLAG_TRG_BLOCKER_A    = BITMASK(12),
    // SHOCK_FLAG_TRG_HOOK_UNLOCK  = BITMASK(13),
    // SHOCK_FLAG_TRG_HOOK_LOCK    = BITMASK(14),
    // SHOCK_FLAG_TRG_CONTINUOUS   = BITMASK(15),
    // SHOCK_FLAG_TRG_ENGINE       = BITMASK(16)
// };

enum EngineTriggerType
{
    TRG_ENGINE_CLUTCH    = 0,
    TRG_ENGINE_BRAKE     = 1,
    TRG_ENGINE_ACC       = 2,
    TRG_ENGINE_RPM       = 3,
    TRG_ENGINE_SHIFTUP   = 4,
    TRG_ENGINE_SHIFTDOWN = 5 
};

enum class FlareType: char
{
    NONE           = 0,
    HEADLIGHT      = 'f',
    BRAKE_LIGHT    = 'b',
    REVERSE_LIGHT  = 'R',
    BLINKER_LEFT   = 'l',
    BLINKER_RIGHT  = 'r',
    USER           = 'u'
};

enum LocalizerType
{
    LOCALIZER_VERTICAL,
    LOCALIZER_HORIZONTAL,
    LOCALIZER_NDB,
    LOCALIZER_VOR
};


// --------------------------------
// Soft body physics

/// Physics: A vertex in the softbody structure
struct node_t
{
    // REFACTOR IN PROGRESS: Currently nodes are adressed mostly by pointers or int32_t indices,
    //     although there was always a hidden soft limit of 2^16 nodes (because of `short node_t::pos`).
    //     Let's use `uint16_t` indices everywhere to be clear.      ~ only_a_ptr, 04/2018
    static const uint16_t INVALID_IDX = std::numeric_limits<uint16_t>::max();
    static const int8_t   INVALID_BBOX = -1;

    node_t()               { memset(this, 0, sizeof(node_t)); nd_coll_bbox_id = INVALID_BBOX; }
    node_t(size_t _pos)    { memset(this, 0, sizeof(node_t)); nd_coll_bbox_id = INVALID_BBOX; pos = static_cast<short>(_pos); }

    irr::core::vector3df   RelPosition;             //!< relative to the local physics origin (one origin per actor) (shaky)
    irr::core::vector3df   AbsPosition;             //!< absolute position in the world (shaky)
    irr::core::vector3df   Velocity;
    irr::core::vector3df   Forces;

    float      mass;
    float      buoyancy;
    float      friction_coef;
    float      surface_coef;
    float      volume_coef;

    int16_t         pos;                     //!< This node's index in Actor::ar_nodes array.
    int16_t         nd_coll_bbox_id;         //!< Optional attribute (-1 = none) - multiple collision bounding boxes defined in truckfile
    int16_t         nd_lockgroup;            //!< Optional attribute (-1 = default, 9999 = deny lock) - used in the hook lock logic

    // Bit flags
    bool            nd_cab_node:1;           //!< Attr; This node is part of collision triangle
    bool            nd_rim_node:1;           //!< Attr; This node is part of a rim
    bool            nd_tyre_node:1;          //!< Attr; This node is part of a tyre
    bool            nd_contacter:1;          //!< Attr; User-defined
    bool            nd_contactable:1;        //!< Attr; This node will be treated as contacter on inter truck collisions
    bool            nd_has_ground_contact:1; //!< Physics state
    bool            nd_has_mesh_contact:1;   //!< Physics state
    bool            nd_immovable:1;          //!< Attr; User-defined
    bool            nd_loaded_mass:1;        //!< User defined attr; mass is calculated from 'globals/loaded-mass' rather than 'globals/dry-mass'
    bool            nd_no_ground_contact:1;  //!< User-defined attr; node ignores contact with ground
    bool            nd_override_mass:1;      //!< User defined attr; mass is user-specified rather than calculated (override the calculation)
    bool            nd_under_water:1;        //!< State; GFX hint
    bool            nd_no_mouse_grab:1;      //!< Attr; User-defined

    float      nd_avg_collision_slip;   //!< Physics state; average slip velocity across the last few physics frames
    irr::core::vector3df   nd_last_collision_slip;  //!< Physics state; last collision slip vector
    irr::core::vector3df   nd_last_collision_force; //!< Physics state; last collision force
    ground_model_t* nd_last_collision_gm;    //!< Physics state; last collision 'ground model' (surface definition)
};

/// Simulation: An edge in the softbody structure
struct beam_t
{
    beam_t() { memset(this, 0, sizeof(beam_t)); }

    node_t*         p1;
    node_t*         p2;
    float      k;                     //!< tensile spring
    float      d;                     //!< damping factor
    float      L;                     //!< length
    float      minmaxposnegstress;
    float      maxposstress;
    float      maxnegstress;
    float      strength;
    float      stress;
    float      plastic_coef;
    int             detacher_group;        //!< Attribute: detacher group number (integer)
    SpecialBeam     bounded;
    BeamType        bm_type;
    bool            bm_inter_actor;        //!< in case p2 is on another actor
    Actor*          bm_locked_actor;       //!< in case p2 is on another actor
    bool            bm_disabled;
    bool            bm_broken;

    float      shortbound;
    float      longbound;
    float      refL;                  //!< reference length

    shock_t*        shock;

    float      initial_beam_strength; //!< for reset
    float      default_beam_deform;   //!< for reset

    float      debug_k;               //!< debug shock spring_rate
    float      debug_d;               //!< debug shock damping
    float      debug_v;               //!< debug shock velocity
};

struct shock_t
{
    shock_t() { memset(this, 0, sizeof(shock_t)); }

    int beamid;
    int flags;

    bool trigger_enabled;       //!< general trigger,switch and blocker state
    float trigger_switch_state; //!< needed to avoid doubleswitch, bool and timer in one
    float trigger_boundary_t;   //!< optional value to tune trigger_switch_state autorelease
    int trigger_cmdlong;        //!< F-key for trigger injection longbound-check
    int trigger_cmdshort;       //!< F-key for trigger injection shortbound-check
    int last_debug_state;       //!< smart debug output

    float springin;  //!< shocks2 & shocks3
    float dampin;    //!< shocks2 & shocks3
    float springout; //!< shocks2 & shocks3
    float dampout;   //!< shocks2 & shocks3

    float sprogin;   //!< shocks2
    float dprogin;   //!< shocks2
    float sprogout;  //!< shocks2
    float dprogout;  //!< shocks2

    float splitin;   //!< shocks3
    float dslowin;   //!< shocks3
    float dfastin;   //!< shocks3
    float splitout;  //!< shocks3
    float dslowout;  //!< shocks3
    float dfastout;  //!< shocks3

    float sbd_spring;           //!< set beam default for spring
    float sbd_damp;             //!< set beam default for damping
};

struct collcab_rate_t
{
    int rate;     // remaining amount of physics cycles to be skipped
    int distance; // distance (in physics cycles) to the previous collision check
};

struct soundsource_t
{
    SoundScriptInstance* ssi;
    int nodenum;
    int type;
};

struct commandbeam_state_t
{
    commandbeam_state_t() { memset(this, 0, sizeof(commandbeam_state_t)); }

    int8_t   auto_moving_mode;      //!< State

    // Bit flags
    bool     pressed_center_mode:1; //!< State
    bool     auto_move_lock:1;      //!< State
};

struct commandbeam_t
{
    uint16_t cmb_beam_index;            //!< Index to Actor::ar_beams array
    float    cmb_engine_coupling;       //!< Attr from truckfile
    float    cmb_center_length;         //!< Attr computed at spawn
    float    cmb_speed;                 //!< Attr; Rate of contraction/extension
    float    cmb_boundary_length;       //!< Attr; Maximum/minimum length proportional to orig. len.

    // Bit flags
    bool     cmb_is_contraction:1;      //!< Attribute defined at spawn
    bool     cmb_is_force_restricted:1; //!< Attribute defined in truckfile
    bool     cmb_needs_engine:1;        //!< Attribute defined in truckfile
    bool     cmb_is_autocentering:1;    //!< Attribute defined in truckfile
    bool     cmb_plays_sound:1;         //!< Attribute defined in truckfile
    bool     cmb_is_1press:1;           //!< Attribute defined in truckfile
    bool     cmb_is_1press_center:1;    //!< Attribute defined in truckfile

    std::shared_ptr<commandbeam_state_t> cmb_state;
};

struct command_t
{
    int commandValueState;
    float commandValue;
    float triggerInputValue;
    float playerInputValue;
    bool trigger_cmdkeyblock_state;  //!< identifies blocked F-commands for triggers
    std::vector<commandbeam_t> beams;
    std::vector<int> rotators;
    Ogre::String description;
    RoR::CmdKeyInertia rotator_inertia;
    RoR::CmdKeyInertia command_inertia;
};

struct rotator_t
{
    bool needs_engine;
    int nodes1[4];
    int nodes2[4];
    int axis1; //!< rot axis
    int axis2;
    float angle;
    float rate;
    float force;
    float tolerance;
    float engine_coupling;
    float debug_rate;
    float debug_aerror;
};

// --------------------------------
// some non-actor structs


struct collision_box_t
{
    bool virt;
    bool refined;
    bool selfrotated;
    bool camforced;
    bool enabled;
    CollisionEventFilter event_filter;
    short eventsourcenum;
    irr::core::vector3df lo;           //!< absolute collision box
    irr::core::vector3df hi;           //!< absolute collision box
    irr::core::vector3df center;       //!< center of rotation
    Ogre::Quaternion rot;       //!< rotation
    Ogre::Quaternion unrot;     //!< rotation
    irr::core::vector3df selfcenter;   //!< center of self rotation
    Ogre::Quaternion selfrot;   //!< self rotation
    Ogre::Quaternion selfunrot; //!< self rotation
    irr::core::vector3df relo;         //!< relative collision box
    irr::core::vector3df rehi;         //!< relative collision box
    irr::core::vector3df campos;       //!< camera position
};

struct ground_model_t
{
    float va;                       //!< adhesion velocity
    float ms;                       //!< static friction coefficient
    float mc;                       //!< sliding friction coefficient
    float t2;                       //!< hydrodynamic friction (s/m)
    float vs;                       //!< stribeck velocity (m/s)
    float alpha;                    //!< steady-steady
    float strength;                 //!< ground strength

    float fluid_density;            //!< Density of liquid
    float flow_consistency_index;   //!< general drag coefficient

    //! if flow_behavior_index<1 then liquid is Pseudoplastic (ketchup, whipped cream, paint)
    //! if =1 then liquid is Newtoni'an fluid
    //! if >1 then liquid is Dilatant fluid (less common)
    float flow_behavior_index;

    
    float solid_ground_level;       //!< how deep the solid ground is
    float drag_anisotropy;          //!< Upwards/Downwards drag anisotropy

    int fx_type;
    Ogre::ColourValue fx_colour;
    char name[256];
    char basename[256];
    char particle_name[256];

    int fx_particle_amount;         //!< amount of particles

    float fx_particle_min_velo;     //!< minimum velocity to display sparks
    float fx_particle_max_velo;     //!< maximum velocity to display sparks
    float fx_particle_fade;         //!< fade coefficient
    float fx_particle_timedelta;    //!< delta for particle animation
    float fx_particle_velo_factor;  //!< velocity factor
    float fx_particle_ttl;
};

struct authorinfo_t
{
    int id;
    Ogre::String type;
    Ogre::String name;
    Ogre::String email;
};

struct ActorSpawnRequest
{
    enum class Origin //!< Enables special processing
    {
        UNKNOWN,
        CONFIG_FILE,  //!< 'Preselected vehicle' in RoR.cfg or command line
        TERRN_DEF,    //!< Preloaded with terrain
        USER,         //!< Direct selection by user via GUI
        SAVEGAME,     //!< User spawned and part of a savegame
        NETWORK       //!< Remote controlled
    };

    CacheEntry*         asr_cache_entry = nullptr; //!< Optional, overrides 'asr_filename' and 'asr_cache_entry_num'
    std::string         asr_filename;
    Ogre::String        asr_config;
    irr::core::vector3df       asr_position = irr::core::vector3df::ZERO;
    Ogre::Quaternion    asr_rotation = Ogre::Quaternion::ZERO;
    collision_box_t*    asr_spawnbox = nullptr;
    CacheEntry*         asr_skin_entry = nullptr;
    Origin              asr_origin = Origin::UNKNOWN;
    int                 asr_debugview = 0; //(int)GfxActor::DebugViewType::DEBUGVIEW_NONE;
    Ogre::UTFString     asr_net_username;
    int                 asr_net_color = 0;
    int                 net_source_id = 0;
    int                 net_stream_id = 0;
    bool                asr_free_position = false;   //!< Disables the automatic spawn position adjustment
    bool                asr_terrn_machine = false;   //!< This is a fixed machinery
    std::shared_ptr<rapidjson::Document>
                        asr_saved_state;             //!< Pushes msg MODIFY_ACTOR (type RESTORE_SAVED) after spawn.
};

struct ActorModifyRequest
{
    enum class Type
    {
        INVALID,
        RELOAD,               //!< Full reload from filesystem, requested by user
        RESET_ON_INIT_POS,
        RESET_ON_SPOT,
        SOFT_RESET,
        RESTORE_SAVED
    };

    Actor*              amr_actor;
    Type                amr_type;
    std::shared_ptr<rapidjson::Document>
                        amr_saved_state;
};

} // namespace RoR
