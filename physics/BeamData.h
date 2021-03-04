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

/// @file
/// @author Thomas Fischer
/// @date   30th of April 2010
/// @brief Core data structures for simulation.

#pragma once

/*
 A word of warning:
 RoR's performance is very sensitive to the ordering of the parameters in this
 structure (due to cache reasons). You can easily destroy RoR's performance if you put
 something in the wrong place. Unless you know what you are doing (do you come armed
 with a cache usage tracker?), add what you wish to the bottom of the structure.
*/

// The RoR required includes (should be included already)
#include "ForwardDeclarations.h"
#include "BeamConstants.h"
//#include "BitFlags.h"
#include "CmdKeyInertia.h"
#include <vector>
#include <memory>

enum event_types {
    EVENT_NONE=0,
    EVENT_ALL,
    EVENT_AVATAR,
    EVENT_TRUCK,
    EVENT_AIRPLANE,
    EVENT_BOAT,
    EVENT_DELETE
};

enum hook_states {
    HOOK_LOCK=0,
    HOOK_UNLOCK,
    HOOK_TOGGLE,
    MOUSE_HOOK_TOGGLE,
};

/* Enumerations */
enum {
    BEAM_NORMAL,
    BEAM_HYDRO,
    BEAM_VIRTUAL,         //!< Excluded from mass calculations, visuals permanently disabled
};

enum {
    UNLOCKED,       //!< lock not locked
    PREUNLOCK,      //!< preunlocking, inter-actor beam deletion in progress
    PRELOCK,        //!< prelocking, attraction forces in action
    LOCKED          //!< lock locked.
};
enum {
    NOT_DRIVEABLE,  //!< not drivable at all
    TRUCK,          //!< its a truck (or other land vehicle)
    AIRPLANE,       //!< its an airplane
    BOAT,           //!< its a boat
    MACHINE,        //!< its a machine
    AI,             //!< machine controlled by an Artificial Intelligence
};

enum {
    NOSHOCK,        //!< not a shock
    SHOCK1,         //!< shock1
    SHOCK2,         //!< shock2
    SHOCK3,         //!< shock3
    TRIGGER,        //!< trigger
    SUPPORTBEAM,    //!<
    ROPE            //!<
};


enum {
    DEFAULT_DETACHER_GROUP  = 0, // default for detaching beam group
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

/* some info holding arrays */
static const float flapangles[6] = {0.f, -0.07f, -0.17f, -0.33f, -0.67f, -1.f};

/* basic structures */

#include "datatypes/node_t.h"

struct collcab_rate_t
{
    int rate;     // remaining amount of physics cycles to be skipped
    int distance; // distance (in physics cycles) to the previous collision check
};

#include "datatypes/beam_t.h"

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
    std::string description;
    RoR::CmdKeyInertia rotator_inertia;
    RoR::CmdKeyInertia command_inertia;
};

struct hydrobeam_t
{
    uint16_t hb_beam_index; //!< Index to Actor::ar_beams array
    float    hb_ref_length; //!< Idle length in meters
    float    hb_speed;      //!< Rate of change
    int      hb_flags;
    int      hb_anim_flags; //!< Animators (beams updating length based on simulation variables)
    float    hb_anim_param; //!< Animators (beams updating length based on simulation variables)
    RoR::CmdKeyInertia  hb_inertia;
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


// some non-beam structs


// struct collision_box_t
// {
    // bool virt;
    // bool refined;
    // bool selfrotated;
    // bool camforced;
    // bool enabled;
    // short event_filter;
    // short eventsourcenum;
    // irr::core::vector3df lo;           //!< absolute collision box
    // irr::core::vector3df hi;           //!< absolute collision box
    // irr::core::vector3df center;       //!< center of rotation
    // irr::core::quaternion rot;       //!< rotation
    // irr::core::quaternion unrot;     //!< rotation
    // irr::core::vector3df selfcenter;   //!< center of self rotation
    // irr::core::quaternion selfrot;   //!< self rotation
    // irr::core::quaternion selfunrot; //!< self rotation
    // irr::core::vector3df relo;         //!< relative collision box
    // irr::core::vector3df rehi;         //!< relative collision box
    // irr::core::vector3df campos;       //!< camera position
// };

// struct ground_model_t
// {
    // float va;                       //!< adhesion velocity
    // float ms;                       //!< static friction coefficient
    // float mc;                       //!< sliding friction coefficient
    // float t2;                       //!< hydrodynamic friction (s/m)
    // float vs;                       //!< stribeck velocity (m/s)
    // float alpha;                    //!< steady-steady
    // float strength;                 //!< ground strength

    // float fluid_density;            //!< Density of liquid
    // float flow_consistency_index;   //!< general drag coefficient

    // //! if flow_behavior_index<1 then liquid is Pseudoplastic (ketchup, whipped cream, paint)
    // //! if =1 then liquid is Newtoni'an fluid
    // //! if >1 then liquid is Dilatant fluid (less common)
    // float flow_behavior_index;

    
    // float solid_ground_level;       //!< how deep the solid ground is
    // float drag_anisotropy;          //!< Upwards/Downwards drag anisotropy

    // int fx_type;
    // irr::video::SColor fx_colour;
    // char name[256];
    // char basename[256];
    // char particle_name[256];

    // int fx_particle_amount;         //!< amount of particles

    // float fx_particle_min_velo;     //!< minimum velocity to display sparks
    // float fx_particle_max_velo;     //!< maximum velocity to display sparks
    // float fx_particle_fade;         //!< fade coefficient
    // float fx_particle_timedelta;    //!< delta for particle animation
    // float fx_particle_velo_factor;  //!< velocity factor
    // float fx_particle_ttl;
// };

// struct authorinfo_t
// {
    // int id;
    // std::string type;
    // std::string name;
    // std::string email;
// };

namespace RoR {

// struct ActorSpawnRequest
// {
    // enum class Origin //!< Enables special processing
    // {
        // UNKNOWN,
        // CONFIG_FILE,  //!< 'Preselected vehicle' in RoR.cfg
        // TERRN_DEF,    //!< Preloaded with terrain
        // USER,         //!< Direct selection by user via GUI
        // SAVEGAME,     //!< User spawned and part of a savegame
        // NETWORK       //!< Remote controlled
    // };

    // ActorSpawnRequest();

    // CacheEntry*       asr_cache_entry; //!< Optional, overrides 'asr_filename' and 'asr_cache_entry_num'
    // std::string       asr_filename;
    // std::string      asr_config;
    // irr::core::vector3df     asr_position;
    // irr::core::quaternion  asr_rotation;
    // collision_box_t*  asr_spawnbox;
    // CacheEntry*       asr_skin_entry;
    // Origin            asr_origin;
    // //std::string   asr_net_username; //LUCIANO,TODO: switch back to UTFString 
	// std::string   asr_net_username;
    // int               asr_net_color;
    // bool              asr_free_position:1;   //!< Disables the automatic spawn position adjustment
    // bool              asr_terrn_machine:1;   //!< This is a fixed machinery
// };

struct ActorModifyRequest
{
    enum class Type
    {
        INVALID,
        RELOAD,               //!< Full reload from filesystem, requested by user
        RESET_ON_INIT_POS,
        RESET_ON_SPOT,
        SOFT_RESET
    };

    Actor* amr_actor;
    Type   amr_type;
};

} // namespace RoR
