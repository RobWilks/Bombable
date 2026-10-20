# Explicitly define fSimL and bind it directly to the global context
globals.fSimL = {
    myObjectState: nil,

    set_targets: func(hdg = nil, spd = nil, alt = nil, roll = nil, pitch = nil, thr = nil) {
        var state = globals.fSimL.myObjectState;
        if (state == nil) {
            print("Error: fSimL.myObjectState is not initialized.");
            return;
        }
        if (hdg != nil)   state.tgt_heading = hdg;
        if (spd != nil)   state.tgt_speed = spd;
        if (alt != nil)   state.tgt_altitude_ft = alt;
        if (roll != nil)  state.tgt_roll = roll;
        if (pitch != nil) state.tgt_pitch = pitch;
        if (thr != nil)   state.throttle = thr;
        
        print(sprintf("fSimL Target Update -> Hdg: %.1f° | Spd: %.1f kts | Alt: %.1f ft | Thr: %.2f", 
            state.tgt_heading, state.tgt_speed, state.tgt_altitude_ft, state.throttle));
    },
    
    get_state: func() {
        var state = globals.fSimL.myObjectState;
        if (state == nil) return;
        print(sprintf("fSimL Current State -> Hdg: %.1f (Tgt: %.1f) | Spd: %.1f kts | Alt: %.1f ft (Tgt: %.1f) | Thr: %.2f", 
            state.hdg, state.tgt_heading, state.speed, state.alt_ft, state.tgt_altitude_ft, state.throttle));
    }
};



###########################################################
# fSimL_force.nas - 4-Force Vector FDM Integrated with fSimL
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

########################## helpers ###########################

var wrap180 = func(x) {
    x = math.fmod(x + 180.0, 360.0);
    if (x < 0.0) x += 360.0;
    return x - 180.0;
};

var wrap360 = func(x) {
    x = math.fmod(x, 360.0);
    if (x < 0.0) x += 360.0;
    return x;
};

# ISA troposphere density (kg/m^3); a loop changes altitude by thousands of feet
var air_density = func(alt_m) {
    var h = math.max(alt_m, 0.0);
    return rho0 * math.pow(1.0 - 2.25577e-5 * h, 4.25588);
};

########################## lift_curve ###########################
# Lift coefficient vs angle of attack (rad, wrapped to +/- pi): linear, then stall
var lift_curve = func(a) {
    var cl = C_L_vmax + CL_ALPHA * a;
    if (a > A_STALL_POS)
        cl = CL_MAX * math.max(0.3, 1.0 - 1.5 * (a - A_STALL_POS));
    else if (a < A_STALL_NEG)
        cl = -CL_MAX * math.max(0.3, 1.0 - 1.5 * (A_STALL_NEG - a));
    return cl;
};

########################## manoeuvre support ###########################
# Throttle to hold a true airspeed (m/s): feed-forward for drag and gravity along the path,
# plus proportional correction.
var hold_speed = func(self, v_tgt) {
    var v   = self.speed * kts_to_mps;
    var gam = (self.gamma_deg or 0.0) * D2R;
    var req = ((self.drag_N or 0.0) + weight_N * math.sin(gam)
               + self.perf.mass_kg * 0.5 * (v_tgt - v)) / self.perf.max_thrust_N;
    self.throttle = math.clamp(req, 0.0, 1.0);
};

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
# Wrap-aware, rate-limited approach to tgt_pitch / tgt_roll over the full +/-180 range.

var update_orientation = func(self, dt) {
    if (self.aerobatic) {
        var pr = (!contains(self.perf, "max_pitch_rate_aero")) ? 30.0 : self.perf.max_pitch_rate_aero;
        var rr = (!contains(self.perf, "max_roll_rate_aero")) ? 90.0 : self.perf.max_roll_rate_aero;

    }
    else {
        var pr = self.perf.max_pitch_rate;
        var rr = self.perf.max_roll_rate;
    }

    var dp = math.clamp(wrap180(self.tgt_pitch - self.pitch), -pr * dt, pr * dt);
    self.pitch = wrap180(self.pitch + dp);
    self.dPitch = (dt > 0.0) ? dp / dt : 0.0;

    var dr = math.clamp(wrap180(self.tgt_roll - self.roll), -rr * dt, rr * dt);
    self.roll = wrap180(self.roll + dr);
    self.dRoll = (dt > 0.0) ? dr / dt : 0.0;
};




########## update_force_physics ###########################

var update_force_physics = func(self, dt) {
    if (dt <= 0.0) return;

    var ve = self.v_east;  var vn = self.v_north;  var vu = self.v_up;
    var v_h   = math.sqrt(ve*ve + vn*vn);
    var speed = math.sqrt(v_h*v_h + vu*vu);

    var rho = air_density(self.alt_ft * 0.3048);
    var qS  = 0.5 * rho * speed * speed * self.perf.wing_area_m2;

    # Attitude trig
    var psi = self.hdg   * D2R;
    var th  = self.pitch * D2R;
    var phi = self.roll  * D2R;
    var sps = math.sin(psi);  var cps = math.cos(psi);
    var sth = math.sin(th);   var cth = math.cos(th);
    var sph = math.sin(phi);  var cph = math.cos(phi);

    # Axis, wings-level lift direction, right wing (all valid for any pitch and roll)
    var f_e = cth * sps;   var f_n = cth * cps;   var f_u = sth;
    # Right-wing direction with wings level (horizontal, 90 deg right of heading)
    var r_e = cps;         var r_n = -sps;
    # Wings-level "up" (lift) direction = r0 x f  (perpendicular to axis, in vertical plane)
    var u_e = -sth * sps;  var u_n = -sth * cps;  var u_u = cth;

    # Banked lift direction
    var l_e = cph * u_e + sph * r_e;
    var l_n = cph * u_n + sph * r_n;
    var l_u = cph * u_u;

    # Flight-path angle in the plane of flight, signed: valid over the top of a loop
    var v_pl  = ve * sps + vn * cps;
    var gamma = math.atan2(vu, v_pl);                      # (-pi, pi]
    var alpha = wrap180((th - gamma) * R2D) * D2R;         # wrapped angle of attack

    var CL = lift_curve(alpha);
    var CD = C_D0 + K_IND * CL * CL;

    var lift_mag   = qS * CL;
    var drag_mag   = qS * CD;
    var thrust_mag = self.throttle * self.perf.max_thrust_N;
    var axial      = thrust_mag - drag_mag;

    # Diagnostics for the manoeuvre controllers
    self.gamma_deg = gamma * R2D;
    self.alpha_deg = alpha * R2D;
    self.drag_N    = drag_mag;
    self.n_load    = (lift_mag + axial * math.sin(alpha)) / weight_N;
    self.stalled   = (alpha > A_STALL_POS or alpha < A_STALL_NEG);

    var m  = self.perf.mass_kg;
    var ax = (axial * f_e + lift_mag * l_e) / m;
    var ay = (axial * f_n + lift_mag * l_n) / m;
    var az = (axial * f_u + lift_mag * l_u - weight_N) / m;

    self.v_east  += ax * dt;
    self.v_north += ay * dt;
    self.v_up    += az * dt;

    var spd2 = math.sqrt(self.v_east*self.v_east + self.v_north*self.v_north + self.v_up*self.v_up);
    var vh2  = math.sqrt(self.v_east*self.v_east + self.v_north*self.v_north);
    self.speed = spd2 / kts_to_mps;

    # Plane heading follows the horizontal velocity, kept continuous: the velocity azimuth
    # flips by 180 degrees when the path passes over the top, so choose the candidate
    # (azimuth or azimuth+180) nearest the current heading. Hold when nearly vertical.
    if (spd2 > 1.0 and vh2 > 0.05 * spd2) {
        var d = wrap180(math.atan2(self.v_east, self.v_north) * R2D - self.hdg);
        if (d > 90.0) d -= 180.0;
        else if (d < -90.0) d += 180.0;
        self.hdg  = wrap360(self.hdg + d);
        self.dHdg = d / dt;
    } else {
        self.dHdg = 0.0;
    }

    var v_up_fps = self.v_up * 3.28084;
    self.dLat = self.v_north / m_per_deg_lat;
    self.dLon = self.v_east / m_per_deg_lon;
    self.dAlt = v_up_fps;
    self.alt_ft += v_up_fps * dt;
};

########################## start_loop ###########################
# Phases: entry (normal flight to entry speed) -> pull (loop) -> recover (normal hold) -> done.
#
# Pull law: choose a constant pitch rate q so that the radius stays roughly constant.
# In the vertical plane  V * dgamma/dt = g * (n - cos(gamma)),  so for dgamma/dt = q:
#     n_cmd = V*q/g + cos(gamma)         (high at the bottom, lower at the top)
# The lift needed for n_cmd gives the angle of attack, and the attitude target is
#     pitch_target = gamma + alpha_required
# so attitude follows the flight path rather than running ahead of it.

var start_loop = func(self, opts = nil) {
    opts = opts or {};
    self.manoeuvre = {
        name: "loop", phase: "entry", t: 0.0,
        entry_speed: opts["entry_speed"] or 220.0,           # m/s
        n_bottom:    opts["n_bottom"]    or 4.0,             # design load factor at the bottom
        exit_deg:    opts["exit_deg"]    or 3.0,             # hand back to normal control this far from 360
        floor_ft:    opts["floor_ft"]    or (self.alt_ft - 1000.0),
        swept: 0.0, gamma_prev: 0.0, stall_t: 0.0, q_loop: 0.0,
        update: update_loop_target
    };
};

########################## update_loop_target ###########################
var update_loop_target = func(self, dt) {
    var m   = self.manoeuvre;
    var v   = self.speed * kts_to_mps;
    var gam = (self.gamma_deg or 0.0) * D2R;
    m.t += dt;

    if (m.phase == "entry") {
        update_roll_target(self, dt);
        update_pitch_target(self, dt);
        hold_speed(self, m.entry_speed);
        var ready = math.abs(v - m.entry_speed) < 5.0
                and math.abs(self.gamma_deg or 0.0) < 2.0
                and math.abs(self.roll) < 3.0;
        if (ready) {
            m.phase = "pull";
            m.t = 0.0;
            m.swept = 0.0;
            m.gamma_prev = self.gamma_deg;
            m.q_loop = g * (m.n_bottom - 1.0) / v;           # rad/s
            self.aerobatic = 1;
        } else if (m.t > 120.0) {
            self.manoeuvre = nil;                              # could not reach entry conditions
        }
        return;
    }

    if (m.phase == "pull") {
        self.tgt_roll = 0.0;                                   # wings level in the loop plane
        hold_speed(self, m.entry_speed);

        var rho = air_density(self.alt_ft * 0.3048);
        var qS  = 0.5 * rho * v * v * self.perf.wing_area_m2;
        var n_stall = 0.85 * qS * CL_MAX / weight_N;
        var n_allow = math.min(0.9 * (self.perf.n_max_pos or 6.0), n_stall);

        var n_cmd = v * m.q_loop / g + math.cos(gam);
        n_cmd = math.min(n_cmd, n_allow);
        if (self.alt_ft < m.floor_ft and m.swept > 180.0) n_cmd = n_allow;   # low on the way down

        # lift needed, allowing for the thrust and drag component normal to the path
        var a_prev = (self.alpha_deg or 0.0) * D2R;
        var axial  = self.throttle * self.perf.max_thrust_N - (self.drag_N or 0.0);
        var CL_req = (n_cmd * weight_N - axial * math.sin(a_prev)) / math.max(qS, 1.0);
        var a_req  = math.clamp((CL_req - C_L_vmax) / CL_ALPHA, -0.1, 0.9 * A_STALL_POS);

        self.tgt_pitch = wrap180(self.gamma_deg + a_req * R2D);

        m.swept += wrap180(self.gamma_deg - m.gamma_prev);
        m.gamma_prev = self.gamma_deg;
        if (self.stalled) m.stall_t += dt; else m.stall_t = 0.0;

        if (m.swept >= 360.0 - m.exit_deg or m.stall_t > 1.0) {
            m.phase = "recover";
            m.t = 0.0;
        }
        return;
    }

    if (m.phase == "recover") {
        update_roll_target(self, dt);
        update_pitch_target(self, dt);
        hold_speed(self, m.entry_speed);
        if (m.t > 3.0
            and math.abs(wrap180(self.gamma_deg or 0.0)) < 3.0
            and math.abs(wrap180(self.pitch)) < 10.0) {
            self.manoeuvre = nil;
            self.aerobatic = 0;
        }
        return;
    }
};

########################## canonical_euler ###########################
# Equivalent canonical Euler angles with |pitch| <= 90 (pitch > 90 equals pitch 180-p with
# heading+180 and roll+180). Use for the property tree if the model misbehaves beyond 90.

var canonical_euler = func(s) {
    var p = s.pitch;  var h = s.hdg;  var r = s.roll;  var sgn = 1.0;
    if (math.abs(p) > 90.0) {
        p = wrap180(180.0 - p);
        h = wrap360(h + 180.0);
        r = wrap180(r + 180.0);
        sgn = -1.0;
    }
    return { hdg: h, pitch: p, roll: r, sgn: sgn };
};
########################## run_flight_physics ###########################

var run_flight_physics = func(mName) {
    var dt = physics_update_interval;
    var state = myObjectState;

    # 1. Targets: a running manoeuvre owns the targets, otherwise normal holds
    if (state["manoeuvre"] != nil) {
        state.manoeuvre.update(state, dt);
    } else {
        update_roll_target(state, dt);
        update_pitch_target(state, dt);
    }

    # 2. Orientation, 3. forces
    update_orientation(state, dt);
    update_force_physics(state, dt);

    # 4. Push to the animation manager
    var anim = attributes[mName].controls["animation"];
    if (anim != nil and anim.isActive) {
        var o = canonical_euler(state);
        anim.dLat   = state.dLat;
        anim.dLon   = state.dLon;
        anim.dAlt   = state.dAlt;
        anim.dHdg   = state.dHdg;
        anim.dPitch = o.sgn * state.dPitch;
        anim.dRoll  = state.dRoll;
        anim.hdgNode.setDoubleValue(o.hdg);
        anim.pitchNode.setDoubleValue(o.pitch);
        anim.rollNode.setDoubleValue(o.roll);
        anim.altNode.setDoubleValue(state.alt_ft);
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

        # Roll spans +/-180, so wrap it
        var new_roll = roll + anim.dRoll * dt;
        if (new_roll > 180.0) new_roll -= 360.0;
        else if (new_roll < -180.0) new_roll += 360.0;

        # Apply positional and orientational updates per frame
        anim.latNode.setDoubleValue(lat + anim.dLat * dt);
        anim.lonNode.setDoubleValue(lon + anim.dLon * dt);
        anim.altNode.setDoubleValue(alt + anim.dAlt * dt);
        anim.hdgNode.setDoubleValue(new_hdg);
        anim.pitchNode.setDoubleValue(pitch + anim.dPitch * dt);
        anim.rollNode.setDoubleValue(new_roll);        
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
# state.hdg   : degrees, clockwise from north (taken from velocity heading, i.e. zero sideslip)
# state.pitch : degrees, nose-up positive, angle of the aircraft AXIS above horizontal
# state.roll  : degrees, right-wing-down positive

# Loop manoeuvre. Relies on globals defined earlier: rho0, weight_N, C_D, C_L_vmax, CL_ALPHA,
# CL_MAX, g, D2R, R2D, kts_to_mps, m_per_deg_lat, m_per_deg_lon,
# update_roll_target, update_pitch_target (from targetsOrientation_corrected.nas).
#
# state.hdg is the heading of the PLANE of flight
# (kept continuous through a loop). 
# state.pitch is the axis pitch in (-180, 180]: it runs
# straight through 90 degrees over the top. state.roll is in (-180, 180].



# 1. Performance Profile & Force Coefficients
var modelName = "/spitfireXVI";


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

        max_pitch_rate_aero: 30.0, # 30 deg/s
        max_roll_rate_aero:  90.0, # deg/s
        n_max_pos: 6.0, # g factor

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

        max_pitch_rate_aero: 30.0, # 30 deg/s
        max_roll_rate_aero:  90.0, # deg/s
        n_max_pos: 6.0, # g factor

    };
}
###########################################################
# Supermarine Spitfire Mk IX Performance Profile (Clean, Full Internal Fuel)
###########################################################

elsif (modelName == "/spitfireXVI") {
    var modelPath = "/home/michael-wilkinson/Desktop/Bombable 5.0/Bombable/FGData/AI/Aircraft/spitfireIIa-Bombable/Models/spitfire_model.xml";
    var perf_profile = {
        climb_rate: 4745.0,            # ft per min (Max rate of climb at ~10,000 ft)
        descent_rate: 5000.0,          # ft per min (Standard steep tactical descent; max dive limit went much higher)
        max_bank_angle: 180.0,         # deg (Fully aerobatic fighter capable of sustained inverted flight)
        max_roll_rate: 175.0,          # deg/sec (Significantly enhanced at 200 mph due to clipped wing tips)
        max_pitch_angle: 180.0,        # deg (Full vertical looping and looping-maneuver capability)
        max_pitch_rate: 28.0,          # deg/sec (Highly sensitive, light elevator authority typical of late-mark Spitfires)
        
        # Force-Vector Parameters
        mass_kg: 3400.0,               # kg (Standard operational loaded combat weight)
        wing_area_m2: 21.46,           # m^2 (231 sq ft due to the clipped wing tips, down from the standard 22.48 m²)
        max_thrust_N: 12000.0,         # N (Peak propeller thrust derived from the 1,720 hp Packard Merlin 266 engine)
        max_speed_mps: 154.2,          # m/s (Maximum level speed at sea level, equal to ~345 mph / 300 knots)

        max_pitch_rate_aero: 35.0,     # deg/s (Maximum transient pitch rate before high-speed stall or G-lock limit)
        max_roll_rate_aero:  175.0,    # deg/s (Aerodynamic maximum roll rate achieved under optimal speed/airspeed pressures)
        n_max_pos: 6.0,                # g factor (Official design limit load factor for structural airframe integrity)
    };
}

print(modelName ~ " initialised");

########################## aerodynamics ###########################
# Derive aerodynamic coefficients from performance limits
var weight_N = perf_profile.mass_kg * g;
var C_D = (2.0 * perf_profile.max_thrust_N) / (rho0 * perf_profile.max_speed_mps * perf_profile.max_speed_mps * perf_profile.wing_area_m2);
# ---- constants (replace C_L_cruise) ----
var CL_ALPHA = 5.0;    # lift-curve slope per radian (~0.09 per degree)
var CL_MAX   = 1.6;    # simple stall clamp (symmetric)

# CL needed for L = W at max speed, sea level.
# This is the CL at alpha = 0, i.e. at the attitude where theta == gamma at vmax.
var C_L_vmax = (2.0 * weight_N) / (rho0 * perf_profile.max_speed_mps * perf_profile.max_speed_mps * perf_profile.wing_area_m2);
var K_IND = 0.13;                                   # induced drag factor k in CD = CD0 + k*CL^2
var C_D0  = C_D - K_IND * C_L_vmax * C_L_vmax;      # C_D was derived at max speed, where CL is tiny
var A_STALL_POS = (CL_MAX - C_L_vmax) / CL_ALPHA;   # rad
var A_STALL_NEG = -(CL_MAX + C_L_vmax) / CL_ALPHA;  # rad


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

        aerobatic: 0, # flag to enable aerobgatic performance

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
var startLoopNode = props.globals.getNode(modelName ~ "/controls/startLoop", 1);


latNode.setDoubleValue(init_lat);
lonNode.setDoubleValue(init_lon);
altNode.setDoubleValue(init_alt_ft);
hdgNode.setDoubleValue(init_hdg);
pitchNode.setDoubleValue(0.0);
rollNode.setDoubleValue(0.0);
spdNode.setDoubleValue(init_speed);
engineNode.setDoubleValue(init_throttle);
startLoopNode.setBoolValue(0);

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

print("fSimL.nas: Spawned model from " ~ modelPath);


########################## monitorValues ###########################

var monitorValues = func(count, countmax = 500, dt = 0.1) {
    if (fSimL.myObjectState.manoeuvre == nil) return;
    print(sprintf("Phase: %s Gamma: %.2f Pitch %.2f Alpha: %.2f Drag: %.2f Load: %.2f Stalled %d Speed_kts: %.1f Alt_ft: %.1f",
    fSimL.myObjectState.manoeuvre.phase,
    fSimL.myObjectState.gamma_deg,
    fSimL.myObjectState.pitch,
    fSimL.myObjectState.alpha_deg,
    fSimL.myObjectState.drag_N,
    fSimL.myObjectState.n_load,
    fSimL.myObjectState.stalled,
    fSimL.myObjectState.speed,
    fSimL.myObjectState.alt_ft
    ));
    count += 1;
    if (count < countmax) 
    {
        settimer(func { monitorValues(count, countmax, dt); }, dt);
    }
};
########################## Start flying ###########################

# 5. Create Aircraft State Instance & Register with animationManager
# Initialize the state and bind it to the global fSimL namespace
globals.fSimL.myObjectState = create_aircraft_state(modelName, init_lat, init_lon, init_alt_ft, init_hdg, init_speed, init_throttle);

# Create a local alias so internal script loops can access it instantly without typing globals
var myObjectState = globals.fSimL.myObjectState;

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


########################## Start camera ###########################
# Configure and activate a dynamic chase view slot for F15 matching Bombable's architecture
var setup_fSimL_chase_view = func(slot_idx, mName) {
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
    print("fSimL: Chase camera successfully bound to " ~ mName ~ " on View Slot " ~ slot_idx);
};

setup_fSimL_chase_view(101, modelName);

########################## loop usage ###########################
# 
# Create a property listener for startLoopNode
setlistener(startLoopNode, func(node) {
    # Only trigger if the node value becomes true (1)
    if (node.getValue()) {
        var startSpd = 160; nBottom = 4.0;
        start_loop(myObjectState, {entry_speed: startSpd, n_bottom: nBottom});
        print("Starting loop for ", modelName, " with entry spd: ", startSpd, "mps   n bottom: ", nBottom);
        print("Dumping aircraft performance parameters ...");
        debug.dump(myObjectState.perf);
        print("Monitoring flight parameters ...");
        monitorValues(0, 500, 1);
        
        # Reset the node back to 0 after 1 second (1.0 seconds)
        settimer(func() {
            startLoopNode.setBoolValue(0);
        }, 1.0);
    }
}, 0, 0); # runtime = 0 (don't fire immediately on load), async = 0