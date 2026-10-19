# Explicitly define fSim and bind it directly to the global context
globals.fSim = {
    myObjectState: nil,

    set_targets: func(hdg = nil, spd = nil, alt = nil, roll = nil, pitch = nil, thr = nil) {
        var state = globals.fSim.myObjectState;
        if (state == nil) {
            print("Error: fSim.myObjectState is not initialized.");
            return;
        }
        if (hdg != nil)   state.tgt_heading = hdg;
        if (spd != nil)   state.tgt_speed = spd;
        if (alt != nil)   state.tgt_altitude_ft = alt;
        if (roll != nil)  state.tgt_roll = roll;
        if (pitch != nil) state.tgt_pitch = pitch;
        if (thr != nil)   state.throttle = thr;
        
        print(sprintf("fSim Target Update -> Hdg: %.1f° | Spd: %.1f kts | Alt: %.1f ft | Thr: %.2f", 
            state.tgt_heading, state.tgt_speed, state.tgt_altitude_ft, state.throttle));
    },
    
    get_state: func() {
        var state = globals.fSim.myObjectState;
        if (state == nil) return;
        print(sprintf("fSim Current State -> Hdg: %.1f (Tgt: %.1f) | Spd: %.1f kts | Alt: %.1f ft (Tgt: %.1f) | Thr: %.2f", 
            state.hdg, state.tgt_heading, state.speed, state.alt_ft, state.tgt_altitude_ft, state.throttle));
    }
};



###########################################################
# fSim_force.nas - 4-Force Vector FDM Integrated with fSim
###########################################################

# Physical & Conversion Constants
var g = 9.80665;             # Acceleration due to gravity (m/s^2)
var rho0 = 1.225;            # Sea level air density (kg/m^3)
var lbf_to_N = 4.44822;
var kts_to_mps = 0.514444;

var m_per_deg_lat = 111699.7;
var m_per_deg_lon = 111321.5;

var update_m_per_deg_latlon = func() {
    var alat_deg = getprop("/position/latitude-deg") or 37.61899;
    var aLat_rad = alat_deg * D2R;
    m_per_deg_lat = 111699.7 - 1132.978 * math.cos(aLat_rad);
    m_per_deg_lon = 111321.5 * math.cos(aLat_rad);
};
update_m_per_deg_latlon();


########################## update_roll_target ###########################
# Heading error -> desired turn rate -> bank angle.
# With lift trimmed to W/cos(phi) (see update_pitch_target), turn rate = g*tan(phi)/V,
# so phi = atan(V * turn_rate / g).

var update_roll_target = func(self, dt) {
    var diff = self.tgt_heading - self.hdg;
    while (diff >  180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;

    var K_HDG         = 0.3;   # 1/s, heading-error gain
    var MAX_TURN_RATE = 3.0;   # deg/s (standard rate)

    var rate_cmd = diff * K_HDG;
    if (rate_cmd >  MAX_TURN_RATE) rate_cmd =  MAX_TURN_RATE;
    if (rate_cmd < -MAX_TURN_RATE) rate_cmd = -MAX_TURN_RATE;

    var v_h = math.sqrt(self.v_east*self.v_east + self.v_north*self.v_north);
    var phi = math.atan2(v_h * rate_cmd * D2R, g) * R2D;

    self.tgt_roll = math.clamp(phi, -self.perf.max_bank_angle, self.perf.max_bank_angle);
};

########################## update_pitch_target ###########################
# Altitude error -> vertical speed -> flight-path angle (gamma).
# Pitch attitude = gamma + alpha_required, where alpha_required is the angle of attack
# that makes lift = W*cos(gamma)/cos(roll) at the current speed (includes bank compensation).

var update_pitch_target = func(self, dt) {
    var v_h   = math.sqrt(self.v_east*self.v_east + self.v_north*self.v_north);
    var speed = math.sqrt(v_h*v_h + self.v_up*self.v_up);
    if (speed < 20.0) speed = 20.0;   # guard only the controller, not the physics

    var alt_err_m = (self.tgt_altitude_ft - self.alt_ft) / 3.28084;
    var vs_cmd = alt_err_m * 0.1;                       # m/s per metre of error
    var vs_max =  self.perf.climb_rate   * 0.00508;     # ft/min -> m/s
    var vs_min = -self.perf.descent_rate * 0.00508;
    if (vs_cmd > vs_max) vs_cmd = vs_max;
    if (vs_cmd < vs_min) vs_cmd = vs_min;

    var gamma_cmd = math.atan2(vs_cmd, math.max(v_h, 1.0));

    var qS = 0.5 * rho0 * speed * speed * self.perf.wing_area_m2;
    var cphi = math.cos(self.roll * D2R);
    if (cphi < 0.3) cphi = 0.3;

    var CL_req = weight_N * math.cos(gamma_cmd) / (qS * cphi);
    var alpha_req = (CL_req - C_L_vmax) / CL_ALPHA;
    alpha_req = math.clamp(alpha_req, -0.2, 0.2);       # +/- ~11 deg

    var tgt = (gamma_cmd + alpha_req) * R2D;
    self.tgt_pitch = math.clamp(tgt, -self.perf.max_pitch_angle, self.perf.max_pitch_angle);
};

########################## update_orientation ###########################
var update_orientation = func(self, dt) {
    # --- PITCH: Rate-limiting & State Integration ---
    var pitch_diff = self.tgt_pitch - self.pitch;
    var max_pitch_step = self.perf.max_pitch_rate * dt;
    var actual_pitch_change = math.clamp(pitch_diff, -max_pitch_step, max_pitch_step);
    self.pitch += actual_pitch_change;
    self.dPitch = (dt > 0) ? (actual_pitch_change / dt) : 0.0;

    # --- ROLL: Rate-limiting & State Integration ---
    var roll_diff = self.tgt_roll - self.roll;
    var max_roll_step = self.perf.max_roll_rate * dt;
    var actual_roll_change = math.clamp(roll_diff, -max_roll_step, max_roll_step);
    self.roll += actual_roll_change;
    self.dRoll = (dt > 0) ? (actual_roll_change / dt) : 0.0;
};

########################## update_force_physics ###########################

var update_force_physics = func(self, dt) {
    if (dt <= 0.0) return;

    var v_h       = math.sqrt(self.v_east*self.v_east + self.v_north*self.v_north);
    var speed_mps = math.sqrt(v_h*v_h + self.v_up*self.v_up);

    var q  = 0.5 * rho0 * speed_mps * speed_mps;
    var qS = q * self.perf.wing_area_m2;

    # 1. Attitude trig (heading, pitch, roll)
    var psi = self.hdg   * D2R;
    var th  = self.pitch * D2R;
    var phi = self.roll  * D2R;
    var sps = math.sin(psi);  var cps = math.cos(psi);
    var sth = math.sin(th);   var cth = math.cos(th);
    var sph = math.sin(phi);  var cph = math.cos(phi);

    # 2. Aircraft axis (forward) unit vector in world frame
    var f_e = cth * sps;
    var f_n = cth * cps;
    var f_u = sth;

    # 3. Right-wing direction with wings level (horizontal, 90 deg right of heading)
    var r_e =  cps;
    var r_n = -sps;

    # 4. Wings-level "up" (lift) direction = r0 x f  (perpendicular to axis, in vertical plane)
    var u_e = -sth * sps;
    var u_n = -sth * cps;
    var u_u =  cth;

    # 5. Bank about the axis: lift direction = cos(phi)*u0 + sin(phi)*r0
    var l_e = cph * u_e + sph * r_e;
    var l_n = cph * u_n + sph * r_n;
    var l_u = cph * u_u;

    # 6. Angle of attack = axis pitch minus flight-path angle
    var gamma = math.atan2(self.v_up, v_h);
    var alpha = th - gamma;
    var CL = C_L_vmax + CL_ALPHA * alpha;
    if (CL >  CL_MAX) CL =  CL_MAX;
    if (CL < -CL_MAX) CL = -CL_MAX;

    # 7. Force magnitudes
    var lift_mag   = qS * CL;
    var drag_mag   = qS * C_D;
    var thrust_mag = self.throttle * self.perf.max_thrust_N;
    var axial      = thrust_mag - drag_mag;   # thrust and drag both act along the axis

    # 8. Accelerations (m/s^2)
    var m  = self.perf.mass_kg;
    var ax = (axial * f_e + lift_mag * l_e) / m;
    var ay = (axial * f_n + lift_mag * l_n) / m;
    var az = (axial * f_u + lift_mag * l_u - weight_N) / m;

    # 9. Integrate velocity
    self.v_east  += ax * dt;
    self.v_north += ay * dt;
    self.v_up    += az * dt;

    self.speed = math.sqrt(self.v_east*self.v_east + self.v_north*self.v_north + self.v_up*self.v_up) / kts_to_mps;

    # 10. Heading from velocity (guard against near-vertical flight)
    var v_h_new = math.sqrt(self.v_east*self.v_east + self.v_north*self.v_north);
    if (v_h_new > 1.0) {
        var current_hdg = math.atan2(self.v_east, self.v_north) * R2D;
        if (current_hdg < 0) current_hdg += 360.0;

        var hdg_diff = current_hdg - self.hdg;
        if (hdg_diff > 180) hdg_diff -= 360;
        else if (hdg_diff < -180) hdg_diff += 360;

        self.dHdg = hdg_diff / dt;
        self.hdg = current_hdg;
    }

    # 11. Position rates for animationManager
    var v_up_fps = self.v_up * 3.28084;
    self.dLat = self.v_north / m_per_deg_lat;
    self.dLon = self.v_east / m_per_deg_lon;   # m_per_deg_lon must be scaled by cos(latitude)
    self.dAlt = v_up_fps;
    self.alt_ft += v_up_fps * dt;
};


########################## run_flight_physics ###########################

var run_flight_physics = func(mName) {
    var dt = physics_update_interval; 
    var state = myObjectState;

    # 1. Update Desired Targets (What does the AI want to do?)
    update_roll_target(state, dt);
    update_pitch_target(state, dt);

    # 2. Update Orientation State (Rate-limit roll/pitch toward targets)
    update_orientation(state, dt);

    # 3. Apply 4-Force Vector Physics (Compute forces using current roll/pitch, integrate velocity & heading)
    update_force_physics(state, dt);

    # 4. Push Computed Rates to animationManager
    var anim = attributes[mName].controls.animation;
    if (anim != nil and anim.isActive) {
        anim.dLat   = state.dLat;
        anim.dLon   = state.dLon;
        anim.dAlt   = state.dAlt;
        anim.dHdg   = state.dHdg;
        anim.dPitch = state.dPitch;
        anim.dRoll  = state.dRoll;
        anim.hdgNode.setDoubleValue(state.hdg);
        anim.pitchNode.setDoubleValue(state.pitch);
        anim.rollNode.setDoubleValue(state.roll);  
        anim.speedNode.setDoubleValue(state.speed);  
    }
};

########################## animationManager ###########################
# Custom Animation Manager integrated into Bombable

var animationManager = {
    register: func(myNodeName) {
        var ats = attributes[myNodeName];
        if (ats == nil) return;

        ats.controls = ats.controls or {};
        ats.controls.animation = {
            isActive: false,
            latNode: props.globals.getNode(myNodeName ~ "/position/latitude-deg", 1),
            lonNode: props.globals.getNode(myNodeName ~ "/position/longitude-deg", 1),
            altNode: props.globals.getNode(myNodeName ~ "/position/altitude-ft", 1),
            hdgNode: props.globals.getNode(myNodeName ~ "/orientation/true-heading-deg", 1),
            pitchNode: props.globals.getNode(myNodeName ~ "/orientation/pitch-deg", 1),
            rollNode: props.globals.getNode(myNodeName ~ "/orientation/roll-deg", 1),
            speedNode: props.globals.getNode(myNodeName ~ "/velocities/true-airspeed-kts", 1),

            tNode: props.globals.getNode("/sim/time/delta-sec", 1),

            dLat: 0.0, dLon: 0.0, dAlt: 0.0,
            dHdg: 0.0, dPitch: 0.0, dRoll: 0.0,
            timer: nil
        };

        var animData = ats.controls.animation;
        var callback = func { me.updateObject(myNodeName); };
        animData.timer = maketimer(0.0, callback);
        animData.timer.simulatedTime = 1;
    },

    updateObject: func(myNodeName) {
        var ats = attributes[myNodeName];
        if (ats == nil or ats.damage == 1) {
            if (ats != nil and ats.damage == 1) me.stopObject(myNodeName);
            return;
        }

        var anim = ats.controls.animation;
        if (anim == nil or !anim.isActive) return;

        var lat = anim.latNode.getValue() or 0.0;
        var lon = anim.lonNode.getValue() or 0.0;
        var alt = anim.altNode.getValue() or 0.0;
        var hdg = anim.hdgNode.getValue() or 0.0;
        var pitch = anim.pitchNode.getValue() or 0.0; # Fetch current pitch
        var roll = anim.rollNode.getValue() or 0.0;   # Fetch current roll
        var dt = anim.tNode.getValue() or 0.016;

        var new_hdg = hdg + anim.dHdg * dt;
        if (new_hdg >= 360.0) new_hdg -= 360.0;
        else if (new_hdg < 0.0) new_hdg += 360.0;

        # Apply positional and orientational updates per frame
        anim.latNode.setDoubleValue(lat + anim.dLat * dt);
        anim.lonNode.setDoubleValue(lon + anim.dLon * dt);
        anim.altNode.setDoubleValue(alt + anim.dAlt * dt);
        anim.hdgNode.setDoubleValue(new_hdg);
        anim.pitchNode.setDoubleValue(pitch + anim.dPitch * dt);
        anim.rollNode.setDoubleValue(roll + anim.dRoll * dt);  
    },

    startObject: func(myNodeName) {
        var ats = attributes[myNodeName];
        if (ats != nil and ats.controls != nil and ats.controls.animation != nil) {
            ats.controls.animation.isActive = true;
            ats.controls.animation.timer.start();
        }
    },

    stopObject: func(myNodeName) {
        var ats = attributes[myNodeName];
        if (ats != nil and ats.controls != nil and ats.controls.animation != nil) {
            ats.controls.animation.isActive = false;
            ats.controls.animation.timer.stop();
        }
    }
};


########################## Initialize aircraft ###########################
# Conventions assumed: world frame = East, North, Up (m/s).
#   self.hdg   : degrees, clockwise from north (taken from velocity heading, i.e. zero sideslip)
#   self.pitch : degrees, nose-up positive, angle of the aircraft AXIS above horizontal
#   self.roll  : degrees, right-wing-down positive



# 1. Performance Profile & Force Coefficients
var modelName = "/spitfire";


###########################################################
# F-15C Eagle Performance Profile (Clean, Full Internal Fuel)
###########################################################

if (modelName == "/F15") {
    var modelPath = "/home/michael-wilkinson/.fgfs/fgdata_2024_1/AI/Aircraft/F-15/Models/F-15.xml";
    var perf_profile = {
        climb_rate: 40000.0, # ft per min
        descent_rate: 30000.0, #ft per min
        max_bank_angle: 90.0, #deg
        max_roll_rate: 175.0, #deg/sec
        max_pitch_angle: 90.0, #deg
        max_pitch_rate: 20.0, #deg/sec
        
        # Force-Vector Parameters
        mass_kg: 18800.0,
        wing_area_m2: 56.5,
        max_thrust_N: 58000.0 * lbf_to_N, # 2x Upgraded -229/GE engines
        max_speed_mps: 790.0 * kts_to_mps,
    };
}

###########################################################
# Supermarine Spitfire Mk IX Performance Profile (Clean, Full Internal Fuel)
###########################################################

elsif (modelName == "/spitfire") {
    var modelPath = "/home/michael-wilkinson/Desktop/Bombable 5.0/Bombable/FGData/AI/Aircraft/spitfireIIa-Bombable/Models/spitfire_model.xml";
    var perf_profile = {
        climb_rate: 4000.0,            # ft per min (~66.7 ft/sec, ~4,000 ft/min combat climb)
        descent_rate: 3000.0,          # ft per min (~500.0 ft/sec)
        max_bank_angle: 90.0,          # deg (Fully aerobatic fighter agility)
        max_roll_rate: 100.0,          # deg/sec (Brisk manual roll response)
        max_pitch_angle: 90.0,         # deg (Vertical loop capability)
        max_pitch_rate: 15.0,          # deg/sec
        
        # Force-Vector Parameters
        mass_kg: 3450.0,               # kg (Empty ~2,600 kg + ~850 kg internal fuel/ammo)
        wing_area_m2: 24.2,            # m^2 (Elliptical wing planform area)
        max_thrust_N: 2800.0 * lbf_to_N,         # Equivalent propeller thrust (~12,000 N at combat speed)
        max_speed_mps: 300.0 * kts_to_mps, # Maximum low-altitude TAS (~300 kts at sea level / 500 ft ASL)
    };
}

print(modelName ~ " initialised");

# Derive aerodynamic coefficients from performance limits
var weight_N = perf_profile.mass_kg * g;
var C_D = (2.0 * perf_profile.max_thrust_N) / (rho0 * perf_profile.max_speed_mps * perf_profile.max_speed_mps * perf_profile.wing_area_m2);
# ---- constants (replace C_L_cruise) ----
var CL_ALPHA = 5.0;    # lift-curve slope per radian (~0.09 per degree)
var CL_MAX   = 1.6;    # simple stall clamp (symmetric)

# CL needed for L = W at max speed, sea level.
# This is the CL at alpha = 0, i.e. at the attitude where theta == gamma at vmax.
var C_L_vmax = (2.0 * weight_N) / (rho0 * perf_profile.max_speed_mps * perf_profile.max_speed_mps * perf_profile.wing_area_m2);


# 2. Aircraft State Factory
var create_aircraft_state = func(callsign, initial_lat, initial_lon, initial_alt_ft, initial_hdg, initial_speed_kts, initial_throttle) {
    var initial_spd_mps = initial_speed_kts * kts_to_mps;
    var hdg_rad = initial_hdg * D2R;

    return {
        callsign: callsign,
        # Position state
        lat: initial_lat,
        lon: initial_lon,
        alt_ft: initial_alt_ft,
        
        # Orientation state
        hdg: initial_hdg,
        pitch: 0.0,
        roll: 0.0,
        
        # Speeds & 3D Velocity Vector (East, North, Up in m/s)
        speed: initial_speed_kts,
        v_east: initial_spd_mps * math.sin(hdg_rad),
        v_north: initial_spd_mps * math.cos(hdg_rad),
        v_up: 0.0,
        
        # Target controls
        tgt_heading: initial_hdg,
        tgt_speed: initial_speed_kts,
        tgt_altitude_ft: initial_alt_ft,
        tgt_roll: 0.0,
        tgt_pitch: 0.0,
        throttle: initial_throttle,
        
        # Rates for animationManager compatibility (per second)
        dLat: 0.0,
        dLon: 0.0,
        dAlt: 0.0,
        dHdg: 0.0,
        dPitch: 0.0,
        dRoll: 0.0,

        perf: perf_profile
    };
};


########################## Initialize location ###########################
# 3. Initialize Airport & Runway Position Data
# Using EGOD as default reference airport (or change to your preferred airport ID e.g. "EGLL")
var airport_id = "EGOD"; 
var apt = airportinfo(airport_id);

var init_lat = 37.61899; # Fallback latitude
var init_lon = -122.3749; # Fallback longitude
var init_alt_ft = 5013.0; # Fallback altitude (+5000ft over sea level)

if (apt != nil and size(apt.runways) > 0) {
    var runways_list = values(apt.runways);
    var first_rwy = runways_list[0];
    
    init_lat = first_rwy.lat;
    init_lon = first_rwy.lon;

    var apt_elev_ft = apt.elevation * 3.28084 or 0.0;
    init_alt_ft = apt_elev_ft + 5000.0;
    print("fly.nas: Initialized position from " ~ airport_id ~ " Runway threshold at Lat: " ~ init_lat ~ ", Lon: " ~ init_lon);
} else {
    print("fly.nas: Warning - Could not load airport " ~ airport_id ~ ", using default fallback coordinates.");
}

# Initial flight parameters for level flight northwards (Heading 0°, Speed 250 kts)
var init_hdg = 0.0;
var init_speed = 400.0;
var init_throttle = 0.7;

########################## Initialize model ###########################

# 4. Define Property Nodes & Add Model via fgcommand

# Create /model (or nodeName) property tree structure and set initial values
var latNode = props.globals.getNode(modelName ~ "/position/latitude-deg", 1);
var lonNode = props.globals.getNode(modelName ~ "/position/longitude-deg", 1);
var altNode = props.globals.getNode(modelName ~ "/position/altitude-ft", 1);
var hdgNode = props.globals.getNode(modelName ~ "/orientation/true-heading-deg", 1);
var pitchNode = props.globals.getNode(modelName ~ "/orientation/pitch-deg", 1);
var rollNode = props.globals.getNode(modelName ~ "/orientation/roll-deg", 1);
var spdNode = props.globals.getNode(modelName ~ "/velocities/true-airspeed-kts", 1);
var engineNode = props.globals.getNode(modelName ~ "/engines/engine[0]/rpm", 1);


latNode.setDoubleValue(init_lat);
lonNode.setDoubleValue(init_lon);
altNode.setDoubleValue(init_alt_ft);
hdgNode.setDoubleValue(init_hdg);
pitchNode.setDoubleValue(0.0);
rollNode.setDoubleValue(0.0);
spdNode.setDoubleValue(init_speed);
engineNode.setDoubleValue(init_throttle);

# 4. Load model via fgcommand (allocates next model index under target_weapon_path)
var spawn_config = props.Node.new({
    "path": modelPath,
    "latitude-deg-prop":   modelName ~ "/position/latitude-deg",
    "longitude-deg-prop":  modelName ~ "/position/longitude-deg",
    "elevation-ft-prop":   modelName ~ "/position/altitude-ft",
    "heading-deg-prop":    modelName ~ "/orientation/true-heading-deg",
    "pitch-deg-prop":      modelName ~ "/orientation/pitch-deg",
    "roll-deg-prop":       modelName ~ "/orientation/roll-deg",
});

fgcommand("add-model", spawn_config);

print("fly.nas: Spawned model from " ~ modelPath);


########################## monitorValues ###########################
var monitorValues = func(count, countmax = 500, dt = 0.1) {
    print(sprintf("Current Heading: %.2f deg Roll: %.2f deg Pitch: %.2f deg",
    fSim.myObjectState.hdg,
    fSim.myObjectState.roll,
    fSim.myObjectState.pitch
    ));
    count += 1;
    if (count < countmax) 
    {
        settimer(func {monitorValues(count); }, dt);
    }
};
########################## Start flying ###########################

# 5. Create Aircraft State Instance & Register with animationManager
# Initialize the state and bind it to the global fSim namespace
globals.fSim.myObjectState = create_aircraft_state(modelName, init_lat, init_lon, init_alt_ft, init_hdg, init_speed, init_throttle);

# Create a local alias so internal script loops can access it instantly without typing globals
var myObjectState = globals.fSim.myObjectState;

var attributes = {};
attributes[modelName] = {
    damage: 0,
    controls: {},
};

# Register object with animationManager
animationManager.register(modelName);


# Main Physics Tick Function (Runs at a slower interval, e.g., 10 Hz / 0.1s)
var physics_update_interval = 0.1; 

# 2. Wrap it in a closure for maketimer so it evaluates per tick with modelName
var physics_timer = maketimer(physics_update_interval, func { run_flight_physics(modelName); });

physics_timer.simulatedTime = 1;

# Start both systems:
animationManager.startObject(modelName); # Keeps the 3D model moving smoothly per-frame
physics_timer.start();                   # Updates the physics rates 10x per second

monitorValues(0,100,0.1);

########################## Start camera ###########################
# Configure and activate a dynamic chase view slot for F15 matching Bombable's architecture
var setup_fsim_chase_view = func(slot_idx, mName) {
    # 1. Ensure the view slot configuration path exists
    var raw_slot_prop = "/sim/views/ai-targets/slot-" ~ slot_idx;

    setprop(raw_slot_prop, mName);

    setprop("/sim/view[" ~ slot_idx ~ "]/config/x-offset-m", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/y-offset-m", -5);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/z-offset-m", -35);

    setprop("/sim/view[" ~ slot_idx ~ "]/config/eye-lat-deg", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/eye-lon-deg", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/eye-alt-ft", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/target-y-offset-m", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/target-z-offset-m", 0);

    # 6. Switch active camera to this slot
    view.setViewByIndex(slot_idx);

    # Defer root path assignment past FGView::init() reset cycle
    settimer(func {
        setprop("/sim/view[" ~ slot_idx ~ "]/config/root", mName);
    }, 0.1);
    print("fSim: Chase camera successfully bound to " ~ mName ~ " on View Slot " ~ slot_idx);
};

setup_fsim_chase_view(101, modelName);