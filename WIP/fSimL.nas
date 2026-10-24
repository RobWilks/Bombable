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
var PI = 3.14159265358979;


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

########################## air_density ###########################
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

########################## thrust_available ###########################
# Thrust available at full throttle at the current altitude (N)
var thrust_available = func(self) {
    var rho = air_density(self.alt_ft * 0.3048);
    return self.perf.max_thrust_N * math.pow(rho / rho0, THRUST_LAPSE);
};

########################## hold_speed ###########################
# Throttle to hold a true airspeed (m/s): feed-forward for drag and gravity along the path,
# plus proportional correction.
var hold_speed = func(self, v_tgt) {
    var v   = self.speed * kts_to_mps;
    var gam = (self.gamma_deg or 0.0) * D2R;
    var req = ((self.drag_N or 0.0) + weight_N * math.sin(gam)
               + self.perf.mass_kg * 0.5 * (v_tgt - v)) / thrust_available(self);
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

    var qS = 0.5 * air_density(self.alt_ft * 0.3048) * speed * speed * self.perf.wing_area_m2;
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




########################### update_force_physics ###########################
#  1. Lift acts perpendicular to the VELOCITY (rotated from body-up about the right wing),
#     with an explicit induced drag K_IND*CL^2. Thrust and drag still act along the aircraft
#     axis, as you specified. Before, lift perpendicular to the axis made the lift tilt an
#     induced drag with an effective k of 1/CL_ALPHA = 0.2 (2-3 times a real Spitfire), about
#     20 kN at 4 g, which is why the loop top speeds and Immelmann stall margins were thin.
#  2. Thrust falls with air density: T = T0 * (rho/rho0)^THRUST_LAPSE. Before, level speed
#     rose with altitude (353 kt at 15,000 ft against max_speed_mps of 300 kt).
#     Adopt 1 and 2 together: lapse alone makes the Immelmann stall at altitude.
#  3. Integration uses the measured time between physics ticks (your tNode) and sub-steps of
#     at most 25 ms, running the whole control + orientation + force loop each sub-step. The
#     explicit Euler step adds energy in proportion to dt (top speed +5 kt at 0.1 s, +14 kt at
#     0.25 s, and a variable tick of 0.100-0.117 s changed the dynamics rate by 17 percent).

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

    # Axis, wings-level body-up, right wing
    var f_e = cth * sps;   var f_n = cth * cps;   var f_u = sth;
    var r_e = cps;         var r_n = -sps;
    var u_e = -sth * sps;  var u_n = -sth * cps;  var u_u = cth;

    # Body-up direction after bank
    var l_e = cph * u_e + sph * r_e;
    var l_n = cph * u_n + sph * r_n;
    var l_u = cph * u_u;

    # Path angle in the plane of flight (signed, valid over the top) and aircraft-frame alpha
    var v_pl  = ve * sps + vn * cps;
    var gamma = math.atan2(vu, v_pl);                      # (-pi, pi]
    var v_dot_l = ve * l_e + vn * l_n + vu * l_u;

    var alpha = math.atan2(-v_dot_l, ve * f_e + vn * f_n + vu * f_u);

    # Lift direction: body-up with its component along the velocity removed, so lift is
    # perpendicular to the velocity (standard wind-axes lift).
    var lp_e = l_e;  var lp_n = l_n;  var lp_u = l_u;
    if (speed > 1.0) {
        var d = v_dot_l / speed;
        var a_e = l_e - d * ve / speed;
        var a_n = l_n - d * vn / speed;
        var a_u = l_u - d * vu / speed;
        var nrm = math.sqrt(a_e*a_e + a_n*a_n + a_u*a_u);
        if (nrm > 0.01) {
            lp_e = a_e / nrm;  lp_n = a_n / nrm;  lp_u = a_u / nrm;
        }
    }


    var CL = lift_curve(alpha);
    var CD = C_D0 + K_IND * CL * CL;

    var lift_mag   = qS * CL;
    var drag_mag   = qS * CD;
    var thrust_mag = self.throttle * thrust_available(self);
    var axial      = thrust_mag - drag_mag;           # thrust and drag act along the axis

    # Diagnostics for the manoeuvre controllers and monitors
    self.gamma_deg = gamma * R2D;
    self.alpha_deg = alpha * R2D;
    self.drag_N    = drag_mag;
    self.n_load    = (lift_mag + axial * math.sin(alpha)) / weight_N;   # load normal to the path
    self.stalled   = (alpha > A_STALL_POS or alpha < A_STALL_NEG);

    var m  = self.perf.mass_kg;
    var ax = (axial * f_e + lift_mag * lp_e) / m;
    var ay = (axial * f_n + lift_mag * lp_n) / m;
    var az = (axial * f_u + lift_mag * lp_u - weight_N) / m;

    self.v_east  += ax * dt;
    self.v_north += ay * dt;
    self.v_up    += az * dt;

    var spd2 = math.sqrt(self.v_east*self.v_east + self.v_north*self.v_north + self.v_up*self.v_up);
    var vh2  = math.sqrt(self.v_east*self.v_east + self.v_north*self.v_north);
    self.speed = spd2 / kts_to_mps;

    # Plane heading follows the horizontal velocity, kept continuous through the vertical
    if (spd2 > 1.0 and vh2 > 0.05 * spd2) {
        var dh = wrap180(math.atan2(self.v_east, self.v_north) * R2D - self.hdg);
        if (dh > 90.0) dh -= 180.0;
        else if (dh < -90.0) dh += 180.0;
        self.hdg  = wrap360(self.hdg + dh);
        self.dHdg = dh / dt;
    } else {
        self.dHdg = 0.0;
    }

    var v_up_fps = self.v_up * 3.28084;
    self.dLat = self.v_north / m_per_deg_lat;
    self.dLon = self.v_east / m_per_deg_lon;
    self.dAlt = v_up_fps;
    self.alt_ft += v_up_fps * dt;
};

########################## acrobatic manoeuvres ###########################



# Immelmann turn, 360 degree aileron roll and split-S, each a complete state machine in its own
# update function (self.manoeuvre.update), in the same style as update_loop_target.
# Only small primitives are shared (load-factor -> attitude target, roll stepper, entry and
# recovery helpers); no manoeuvre is built by chaining another.
#
# Load after loop_manoeuvre.nas and loop_control.nas. Uses from them:
#   wrap180, wrap360, air_density, hold_speed, lift_curve constants (C_L_vmax, CL_ALPHA, CL_MAX,
#   A_STALL_POS, A_STALL_NEG, K_IND, C_D0), weight_N, g, D2R, R2D, kts_to_mps,
#   update_roll_target, start_loop.
#
# ---- REQUIRED CHANGES TO EXISTING CODE ------------------------------------------------------
#
# 1. update_force_physics: angle of attack must be measured in the aircraft frame, not as
#    (pitch - path angle). They are identical while roll = 0 (so the loop is unchanged) but the
#    old form is wrong for any rolled or inverted attitude. Replace
#        var alpha = wrap180((th - gamma) * R2D) * D2R;
#    with (it must come after l_e, l_n, l_u and f_e, f_n, f_u are defined):
#        var alpha = math.atan2(-(ve * l_e + vn * l_n + vu * l_u),
#                                 ve * f_e + vn * f_n + vu * f_u);
#
# 2. K_IND: because lift acts perpendicular to the AXIS, the tilt of the lift vector from the
#    flight path creates an induced drag
#
# 3. Allow for decrease of air_density with altitude.
#
# 4. Keep physics_update_interval at 0.1 s or less: a 175 deg/s roll moves 17 degrees per tick.

########################## shared primitives ###########################

# Put the attitude in canonical form (|pitch| <= 90) and make the normal holds continue from it.
# The property-tree output (canonical_euler) is identical before and after.
var normalise_attitude = func(self) {
    if (math.abs(self.pitch) > 90.0) {
        self.pitch = wrap180(180.0 - self.pitch);
        self.hdg   = wrap360(self.hdg + 180.0);
        self.roll  = wrap180(self.roll + 180.0);
    }
    self.tgt_pitch = self.pitch;
    self.tgt_roll  = self.roll;
};

# For manoeuvres that reverse heading: normalise, then hold the new heading and altitude.
var finish_reversal = func(self) {
    normalise_attitude(self);
    self.tgt_heading     = self.hdg;
    self.tgt_altitude_ft = self.alt_ft;
};

var v_stall_1g = func(self) {
    var rho = air_density(self.alt_ft * 0.3048);
    return math.sqrt(2.0 * weight_N / (rho * self.perf.wing_area_m2 * CL_MAX));
};

# Angle of attack (rad) giving load factor n_cmd along body-up, allowing for the thrust and
# drag component normal to the flight path (using last tick's alpha).
var alpha_for_load = func(self, n_cmd, qS) {
    var a_prev = (self.alpha_deg or 0.0) * D2R;
    var axial  = self.throttle * thrust_available(self) - (self.drag_N or 0.0);
    var CL_req = (n_cmd * weight_N - axial * math.sin(a_prev)) / math.max(qS, 1.0);
    return math.clamp((CL_req - C_L_vmax) / CL_ALPHA, 0.9 * A_STALL_NEG, 0.9 * A_STALL_POS);
};

# The aircraft-frame alpha is about (pitch - path angle)*cos(roll). Invert that for the
# pitch offset to command: tan(delta) = tan(alpha) / cos(roll), with |cos| floored near knife edge.
var delta_for_alpha = func(a, phi) {
    var c  = math.cos(phi);
    var sg = (c < 0.0) ? -1.0 : 1.0;
    return math.atan2(sg * math.sin(a), math.cos(a) * math.max(math.abs(c), 0.25));
};

# Command a body-axis load factor: sets tgt_pitch = path angle + offset + rate feed-forward.
var command_load = func(self, v, n_cmd, lead_deg) {
    var rho = air_density(self.alt_ft * 0.3048);
    var qS  = 0.5 * rho * v * v * self.perf.wing_area_m2;
    var n_stall = 0.85 * qS * CL_MAX / weight_N;
    var n_pos = math.min(0.9 * (self.perf.n_max_pos or 6.0), n_stall);
    var n_neg = math.min(0.9 * math.abs(self.perf.n_max_neg or 3.0), n_stall);
    n_cmd = math.clamp(n_cmd, -n_neg, n_pos);

    var a_req = alpha_for_load(self, n_cmd, qS);
    var delta = delta_for_alpha(a_req, self.roll * D2R);
    self.tgt_pitch = wrap180((self.gamma_deg or 0.0) + delta * R2D + lead_deg);
    return n_cmd;
};

# Command a path curvature rate gdot (rad/s, + = path rotating upward, in the vertical plane).
# Required lift along the path normal is W*cos(gamma) + m*V*gdot.
#   steady (roll 0 or 180): n = X / cos(roll)
#   rolling:                n = 2 cos(roll) X   (barrel schedule: net vertical impulse over the
#                                                roll equals that of 1 g steady flight)
var command_path = func(self, v, gdot, rolling, dt) {
    var c = math.cos(self.roll * D2R);
    var X = v * gdot / g + math.cos((self.gamma_deg or 0.0) * D2R);
    var n = 0.0;
    if (rolling) {
        n = 2.0 * c * X;
    } else {
        if (math.abs(c) < 0.3) c = (c < 0.0) ? -0.3 : 0.3;
        n = X / c;
    }
    return command_load(self, v, n, gdot * dt * R2D);
};

# Normal flight until the entry conditions are met. Returns 1 ready, 0 waiting, -1 timed out.
var run_entry = func(self, m, dt) {
    update_roll_target(self, dt);
    update_pitch_target(self, dt);
    hold_speed(self, m.entry_speed);
    var v = self.speed * kts_to_mps;
    if (math.abs(v - m.entry_speed) < 5.0
        and math.abs(self.gamma_deg or 0.0) < 3.0
        and math.abs(self.roll) < 3.0) return 1;
    if (m.t > 120.0) {
        print(sprintf("%s cancelled: entry conditions not met in 120 s (speed %.0f m/s, entry_speed %.0f m/s)",
                      m.name, v, m.entry_speed));
        return -1;
    }
    return 0;
};

# Flight-path recovery to level. Unlike the altitude-hold pitch law (which commands an attitude
# and so deepens a stall as the path falls away) this commands path curvature with the angle of
# attack capped, so a slow aircraft trades height for speed.
var recover_level = func(self, dt, v_hold) {
    update_roll_target(self, dt);
    hold_speed(self, v_hold);
    var v   = math.max(self.speed * kts_to_mps, 20.0);
    var v_h = math.sqrt(self.v_east * self.v_east + self.v_north * self.v_north);
    var vs  = (self.tgt_altitude_ft - self.alt_ft) / 3.28084 * 0.1;
    vs = math.clamp(vs, -self.perf.descent_rate * 0.00508, self.perf.climb_rate * 0.00508);
    var gcmd = math.atan2(vs, math.max(v_h, 1.0));
    if (v < 1.25 * v_stall_1g(self)) gcmd = math.min(gcmd, -8.0 * D2R);   # slow: unload and dive
    var gdot = math.clamp(0.8 * (gcmd - (self.gamma_deg or 0.0) * D2R), -0.5, 0.5);
    return command_path(self, v, gdot, 0, dt);
};

var recovered = func(self, m) {
    var ok = math.abs(self.gamma_deg or 0.0) < 3.0
         and math.abs(wrap180(self.pitch)) < 10.0
         and self.speed * kts_to_mps > math.max(1.25 * v_stall_1g(self), 0.8 * m.entry_speed);
    return (m.t > 3.0 and ok) or m.t > 45.0;
};

# Roll stepper A: roll through `total` degrees from where it started (any amount up to 360).
# Returns degrees remaining.
var step_roll = func(self, m, dt, total) {
    m.roll_swept += m.dir * wrap180(self.roll - m.roll_prev);
    m.roll_prev = self.roll;
    var remaining = total - m.roll_swept;
    var rr   = self.aerobatic ? (self.perf.max_roll_rate_aero or 90.0) : self.perf.max_roll_rate;
    var step = math.min(math.min(m.roll_rate, rr) * dt, math.max(remaining, 0.0));
    self.tgt_roll = wrap180(self.roll + m.dir * step);
    return remaining;
};

# Roll stepper B: roll in direction m.dir to an absolute roll angle (e.g. 180 = inverted),
# whatever the starting bank. Returns degrees remaining.
var step_roll_to = func(self, m, dt, target) {
    var remaining = wrap360(m.dir * (target - self.roll));
    if (remaining > 350.0) remaining = 0.0;
    var rr   = self.aerobatic ? (self.perf.max_roll_rate_aero or 90.0) : self.perf.max_roll_rate;
    var step = math.min(math.min(m.roll_rate, rr) * dt, remaining);
    self.tgt_roll = wrap180(self.roll + m.dir * step);
    return remaining;
};

####################### 2. aileron roll ###########################
# From steady flight: roll 360 degrees about the aircraft axis at the commanded rate, holding
# the path roughly straight with the barrel load schedule n = 2 cos(roll) cos(gamma) (positive
# load while upright, pushing while inverted). Heading and altitude holds then resume.
# Phases: entry -> roll -> recover.
#
# opts: direction ("right"/"left", default right), roll_rate (deg/s, default max_roll_rate_aero),
#       entry_speed (m/s, default current speed), immediate (1 = skip the entry phase)

var start_aileron_roll = func(self, opts = nil) {
    opts = opts or {};
    var v   = self.speed * kts_to_mps;
    var imm = opts["immediate"] or 0;
    self.manoeuvre = {
        name: "aileron_roll", phase: imm ? "roll" : "entry", t: 0.0,
        entry_speed: opts["entry_speed"] or v,
        dir: (opts["direction"] == "left") ? -1.0 : 1.0,
        roll_rate: opts["roll_rate"] or (self.perf.max_roll_rate_aero or 90.0),
        roll_swept: 0.0, roll_prev: self.roll, stall_t: 0.0,
        update: update_aileron_roll_target
    };
    if (imm) self.aerobatic = 1;
    return self.manoeuvre;
};

var update_aileron_roll_target = func(self, dt) {
    var m = self.manoeuvre;
    var v = self.speed * kts_to_mps;
    m.t += dt;

    if (m.phase == "entry") {
        var r = run_entry(self, m, dt);
        if (r == 1) {
            m.phase = "roll";  m.t = 0.0;
            m.roll_swept = 0.0;  m.roll_prev = self.roll;  m.stall_t = 0.0;
            self.aerobatic = 1;
        } else if (r < 0) {
            self.manoeuvre = nil;
        }
        return;
    }

    if (m.phase == "roll") {
        hold_speed(self, m.entry_speed);
        command_path(self, v, 0.0, 1, dt);
        var remaining = step_roll(self, m, dt, 360.0);
        if (self.stalled) m.stall_t += dt; else m.stall_t = 0.0;
        if (remaining <= 1.0 or m.stall_t > 1.0 or m.t > 30.0) {
            m.phase = "recover";
            m.t = 0.0;
        }
        return;
    }

    if (m.phase == "recover") {
        update_roll_target(self, dt);
        update_pitch_target(self, dt);
        hold_speed(self, m.entry_speed);
        if (m.t > 2.0 and math.abs(wrap180(self.roll)) < 3.0
            and math.abs(self.gamma_deg or 0.0) < 3.0) {
            self.manoeuvre = nil;
            self.aerobatic = 0;
        }
        return;
    }
};

############# 1. Immelmann ###########################
# Half loop up (constant pitch rate, as the loop), then a 180 degree roll to upright started
# roll_lead_deg before the path reaches inverted, ending level and upright with the heading
# reversed and the height gained. Phases: entry -> pull -> roll -> recover.
#
# The roll is made at the top, near stall speed, so energy decides feasibility. start_immelmann
# predicts the top speed and refuses if it would be below the 1 g stall speed there
# (opts.force = 1 overrides).
#
# opts: entry_speed (m/s, default 0.97*max_speed), n_bottom (default min(5, 0.9*n_max_pos):
#       a tighter loop reaches the top with more speed), roll_lead_deg (default 20),
#       direction ("right"/"left"), roll_rate, force

var predict_immelmann_top = func(self, v0, n_bottom) {
    var S = self.perf.wing_area_m2;
    var T = thrust_available(self);
    var n_cap = 0.9 * (self.perf.n_max_pos or 6.0);
    var q = g * (n_bottom - 1.0) / v0;
    var dt = 0.05;
    var v = v0;  var gm = 0.0;  var h = 0.0;  var t = 0.0;
    while (gm < PI - 0.35 and t < 90.0) {
        var rho = air_density(self.alt_ft * 0.3048 + h);
        var qS  = 0.5 * rho * v * v * S;
        var n   = math.min(v * q / g + math.cos(gm),
                           math.min(n_cap, 0.85 * qS * CL_MAX / weight_N));
        var CL  = n * weight_N / math.max(qS, 1.0);
        var al  = (CL - C_L_vmax) / CL_ALPHA;
        var D   = qS * (C_D0 + K_IND * CL * CL);
        var L   = qS * CL;
        var a_ax = (T - D) * math.cos(al) - L * math.sin(al);
        v += (a_ax / self.perf.mass_kg - g * math.sin(gm)) * dt;
        v  = math.max(v, 5.0);
        h += v * math.sin(gm) * dt;
        gm += g * (n - math.cos(gm)) / math.max(v, 20.0) * dt;
        t += dt;
    }
    return { v: v, h: h };
};

var start_immelmann = func(self, opts = nil) {
    opts = opts or {};
    var v_e = opts["entry_speed"] or (0.97 * self.perf.max_speed_mps);
    var n_b = opts["n_bottom"] or math.min(5.0, 0.9 * (self.perf.n_max_pos or 6.0));

    if (!opts["force"]) {
        var pr    = predict_immelmann_top(self, v_e, n_b);
        var probe = { alt_ft: self.alt_ft + pr.h / 0.3048, perf: self.perf };
        var vs    = v_stall_1g(probe);
        if (pr.v < vs) {
            print(sprintf("Immelmann refused: predicted top speed %.0f kt is below the %.0f kt stall speed there",
                          pr.v / kts_to_mps, vs / kts_to_mps));
            return nil;
        }
    }

    self.manoeuvre = {
        name: "immelmann", phase: "entry", t: 0.0,
        entry_speed: v_e, n_bottom: n_b,
        roll_lead_deg: opts["roll_lead_deg"] or 20.0,
        dir: (opts["direction"] == "left") ? -1.0 : 1.0,
        roll_rate: opts["roll_rate"] or (self.perf.max_roll_rate_aero or 90.0),
        swept: 0.0, gamma_prev: 0.0, q_loop: 0.0,
        roll_swept: 0.0, roll_prev: 0.0, stall_t: 0.0,
        update: update_immelmann_target
    };
    return self.manoeuvre;
};

var update_immelmann_target = func(self, dt) {
    var m = self.manoeuvre;
    var v = self.speed * kts_to_mps;
    m.t += dt;

    if (m.phase == "entry") {
        var r = run_entry(self, m, dt);
        if (r == 1) {
            m.phase = "pull";  m.t = 0.0;
            m.swept = 0.0;  m.gamma_prev = self.gamma_deg;
            m.q_loop = g * (m.n_bottom - 1.0) / v;                 # rad/s, held constant
            self.aerobatic = 1;
        } else if (r < 0) {
            self.manoeuvre = nil;
        }
        return;
    }

    if (m.phase == "pull") {
        self.tgt_roll = 0.0;
        hold_speed(self, m.entry_speed);
        command_path(self, v, m.q_loop, 0, dt);
        m.swept += wrap180(self.gamma_deg - m.gamma_prev);
        m.gamma_prev = self.gamma_deg;
        if (m.swept >= 180.0 - m.roll_lead_deg or m.t > 60.0) {
            m.phase = "roll";  m.t = 0.0;
            m.roll_swept = 0.0;  m.roll_prev = self.roll;
        }
        return;
    }

    if (m.phase == "roll") {
        hold_speed(self, m.entry_speed);
        # keep curving toward inverted level (gamma = 180) while rolling
        var err  = wrap180(180.0 - self.gamma_deg) * D2R;
        var gdot = math.clamp(0.8 * err, -m.q_loop, m.q_loop);
        command_path(self, v, gdot, 1, dt);
        var remaining = step_roll_to(self, m, dt, 180.0);
        if (remaining <= 1.0 or m.t > 30.0) {
            finish_reversal(self);
            m.phase = "recover";
            m.t = 0.0;
        }
        return;
    }

    if (m.phase == "recover") {
        recover_level(self, dt, m.entry_speed);
        if (recovered(self, m)) {
            self.manoeuvre = nil;
            self.aerobatic = 0;
        }
        return;
    }
};

########################## 3. split-S ###########################
# Half roll to inverted, then a half loop downward (constant load factor, with the onset
# ramped and the path angle captured smoothly at the end), ending level and upright with the
# heading reversed and the height lost. Phases: entry -> roll -> pull -> recover.
#
# The height loss is large, so start_split_s predicts it with a planar integration of the pull
# and refuses if the aircraft is too low (checked against the Spitfire log: predicted 3640 ft
# raw, 4370 ft with the 1.2 margin, against 3440 ft flown). opts.force = 1 overrides. The pull law pulls at maximum load once below floor_ft.
#
# opts: n_pull (g, default 4), entry_speed (m/s, default current speed), direction
#       ("right"/"left", default = the shorter way to inverted), immediate (1 = start the
#       roll at once from any attitude, e.g. a banked turn), ground_ft (default 0),
#       clearance_ft (default 1000), onset (g/s, default 6), roll_rate, force

var predict_split_s_loss = func(self, v0, n_pull) {
    var S = self.perf.wing_area_m2;
    var m = self.perf.mass_kg;
    var n_cap = 0.9 * (self.perf.n_max_pos or 6.0);
    var dt = 0.05;
    var v = v0;  var gp = 0.0;  var loss = 0.0;  var t = 0.0;
    while (gp < PI - 0.05 and t < 90.0) {
        var rho = air_density(self.alt_ft * 0.3048 - loss);
        var qS  = 0.5 * rho * v * v * S;
        var n   = math.min(n_pull, math.min(n_cap, 0.85 * qS * CL_MAX / weight_N));
        var CL  = n * weight_N / math.max(qS, 1.0);
        var al  = (CL - C_L_vmax) / CL_ALPHA;
        var D   = qS * (C_D0 + K_IND * CL * CL);
        var L   = qS * CL;
        gp   += g * (n + math.cos(gp)) / math.max(v, 20.0) * dt;
        # drag acts along the axis, lift is normal to the axis: the tilt of lift from the
        # flight path is the model's main induced drag (about 20 kN at 4 g for the Spitfire)
        v    += ((-D * math.cos(al) - L * math.sin(al)) / m + g * math.sin(gp)) * dt;
        loss += v * math.sin(gp) * dt;
        t += dt;
    }
    var rr = self.perf.max_roll_rate_aero or 90.0;
    return 1.2 * loss + 0.25 * g * (180.0 / rr) * (180.0 / rr);     # margin + half-roll sink
};

var start_split_s = func(self, opts = nil) {
    opts = opts or {};
    var v0     = self.speed * kts_to_mps;
    var n_pull = opts["n_pull"] or 4.0;
    var ground_ft    = opts["ground_ft"] or 0.0;
    var clearance_ft = opts["clearance_ft"] or 1000.0;

    if (!opts["force"]) {
        var loss  = predict_split_s_loss(self, v0, n_pull);
        var avail = (self.alt_ft - ground_ft - clearance_ft) * 0.3048;
        if (loss > avail) {
            print(sprintf("Split-S refused: needs about %.0f ft, only %.0f ft available",
                          loss / 0.3048, avail / 0.3048));
            return nil;
        }
    }

    var d = opts["direction"];
    var dir = (d == "left") ? -1.0 : ((d == "right") ? 1.0 : ((self.roll < 0.0) ? -1.0 : 1.0));
    var imm = opts["immediate"] or 0;

    self.manoeuvre = {
        name: "split_s", phase: imm ? "roll" : "entry", t: 0.0,
        entry_speed: opts["entry_speed"] or v0,
        n_pull: n_pull, onset: opts["onset"] or 6.0,
        exit_deg: 3.0, k_capture: 1.2,
        floor_ft: ground_ft + clearance_ft,
        dir: dir,
        roll_rate: opts["roll_rate"] or (self.perf.max_roll_rate_aero or 90.0),
        roll_swept: 0.0, roll_prev: self.roll,
        swept: 0.0, gamma_prev: 0.0, stall_t: 0.0,
        update: update_split_s_target
    };
    if (imm) self.aerobatic = 1;
    return self.manoeuvre;
};

var update_split_s_target = func(self, dt) {
    var m   = self.manoeuvre;
    var v   = self.speed * kts_to_mps;
    var gam = (self.gamma_deg or 0.0) * D2R;
    m.t += dt;

    if (m.phase == "entry") {
        var r = run_entry(self, m, dt);
        if (r == 1) {
            m.phase = "roll";  m.t = 0.0;
            m.roll_swept = 0.0;  m.roll_prev = self.roll;
            self.aerobatic = 1;
        } else if (r < 0) {
            self.manoeuvre = nil;
        }
        return;
    }

    if (m.phase == "roll") {
        hold_speed(self, m.entry_speed);
        command_path(self, v, 0.0, 1, dt);
        var remaining = step_roll_to(self, m, dt, 180.0);
        if (remaining <= 1.0 or m.t > 30.0) {
            m.phase = "pull";  m.t = 0.0;
            m.swept = 0.0;  m.gamma_prev = self.gamma_deg;
        }
        return;
    }

    if (m.phase == "pull") {
        self.tgt_roll = (self.roll < 0.0) ? -180.0 : 180.0;      # hold inverted
        hold_speed(self, m.entry_speed);

        # Target load factor, ramped up from the -1 g needed to hold inverted level flight.
        var c    = math.cos(self.roll * D2R);                    # about -1
        var rem  = (180.0 - m.swept) * D2R;                      # path rotation still to go
        var n_t  = math.min(m.n_pull, -1.0 + m.onset * m.t);
        if (self.alt_ft < m.floor_ft) n_t = 99.0;                # low: pull as hard as allowed
        # path rotation rate that gives n_t (negative: path rotating downward), captured smoothly
        var gdot = g * (n_t * c - math.cos(gam)) / math.max(v, 20.0);
        gdot = math.max(gdot, -m.k_capture * rem);
        command_path(self, v, gdot, 0, dt);

        m.swept += wrap180(m.gamma_prev - self.gamma_deg);       # path angle decreasing
        m.gamma_prev = self.gamma_deg;
        if (180.0 - m.swept <= m.exit_deg or m.t > 60.0) {
            finish_reversal(self);
            m.phase = "recover";
            m.t = 0.0;
        }
        return;
    }

    if (m.phase == "recover") {
        recover_level(self, dt, m.entry_speed);
        if (recovered(self, m)) {
            self.manoeuvre = nil;
            self.aerobatic = 0;
        }
        return;
    }
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
        var axial  = self.throttle * thrust_available(self) - (self.drag_N or 0.0);
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
    var timeNow = state.tNode.getValue() or 0.0;
    var lastTime = state.time;
    state.dTime = timeNow - lastTime;
    state.time = timeNow;

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

    var throttleFactor = (contains(state.perf, "max_rpm")) ? state.perf.max_rpm : 0.0; 

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
        anim.engineNode.setDoubleValue(state.throttle * throttleFactor);

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
            engineNode: props.globals.getNode(modelName ~ "/engines/engine[0]/rpm", 1),

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
var modelName = "/fkdr1";


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
        n_max_neg: 3.5, 

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
        n_max_neg: 3.5,    
        max_rpm: 3500,              # Merlin engine            

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
        n_max_neg: 3.5,
        max_rpm: 3500,              # Merlin engine            
                
    };
}

elsif (modelName == "/fkdr1") {
    var modelPath = "/home/michael-wilkinson/Desktop/Bombable 5.0/Bombable/FGData/AI/Aircraft/fkdr1-Bombable/Models/dr1.xml";
    var perf_profile = {
        climb_rate: 1122.0,           # ft per min (Max rate of climb ~5.7 m/s)
        descent_rate: 3500.0,          # ft per min (Typical WWI combat dive limit)
        max_bank_angle: 180.0,         # deg (Fully aerobatic maneuverability)
        max_roll_rate: 90.0,           # deg/sec (Moderate aileron authority via interplane struts/warp characteristics)
        max_pitch_angle: 180.0,        # deg (Full loop capability)
        max_pitch_rate: 20.0,          # deg/sec (Highly responsive pitch control due to short tail moment)
        
        # Force-Vector Parameters
        mass_kg: 586.0,                # kg (Standard operational gross weight)
        wing_area_m2: 18.7,            # m^2 (Combined area across all three wings, ~201 sq ft)
        max_thrust_N: 1400.0,          # N (Approximate static thrust derived from the 110 hp Oberursel Ur.II rotary engine)
        max_speed_mps: 50.0,           # m/s (Maximum level speed, ~110–115 mph / 97–100 knots)

        max_pitch_rate_aero: 25.0,     # deg/s (Transient aerodynamic pitch limit)
        max_roll_rate_aero:  90.0,     # deg/s (Maximum aerodynamic roll rate)
        n_max_pos: 5.0,                # g factor (WWI structural design limit load factor)
        n_max_neg: -2.5,
        max_rpm: 1200,                 # engine (Typical maximum RPM for the Oberursel Ur.II rotary engine)
                    
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
var K_IND        = 0.09;     # induced drag factor (real Spitfire about 0.07-0.10)
var THRUST_LAPSE = 0.6;     # thrust ~ density^0.6 (piston engine and propeller - reduced from 0.75)
var C_D0  = C_D - K_IND * C_L_vmax * C_L_vmax;      # C_D was derived at max speed, where CL is tiny
var A_STALL_POS = (CL_MAX - C_L_vmax) / CL_ALPHA;   # rad
var A_STALL_NEG = -(CL_MAX + C_L_vmax) / CL_ALPHA;  # rad


# 2. Aircraft State Factory
var create_aircraft_state = func(callsign, initial_lat, initial_lon, initial_alt_ft, initial_hdg, initial_speed_kts, initial_throttle) {
    var initial_spd_mps = initial_speed_kts * kts_to_mps;
    var hdg_rad = initial_hdg * D2R;
    var tNode = props.globals.getNode("/sim/time/elapsed-sec", 1);

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
        manoeuvre: nil,

        perf: perf_profile,

        tNode: tNode,
        time: tNode.getValue(),
        dTime: 0.1,
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
    init_alt_ft = apt_elev_ft + 6000.0;
    print("fly.nas: Initialized position from " ~ airport_id ~ " Runway threshold at Lat: " ~ init_lat ~ ", Lon: " ~ init_lon);
} else {
    print("fly.nas: Warning - Could not load airport " ~ airport_id ~ ", using default fallback coordinates.");
}

# Initial flight parameters for level flight northwards (Heading 0°, Speed 250 kts)
var init_hdg = 0.0;
var init_speed = 400.0;
var init_throttle = 0.7;
var init_rpm = 1000;

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
engineNode.setDoubleValue(1000);

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
    print(sprintf("Phase: %s Gamma: %.2f Pitch %.2f Alpha: %.2f Drag: %.2f Load: %.2f Stalled %d Speed_kts: %.1f Alt_ft: %.1f Heading %.1f Roll %.1f Throttle %.1f Tick_interval %.3f",
    fSimL.myObjectState.manoeuvre.phase,
    fSimL.myObjectState.gamma_deg,
    fSimL.myObjectState.pitch,
    fSimL.myObjectState.alpha_deg,
    fSimL.myObjectState.drag_N,
    fSimL.myObjectState.n_load,
    fSimL.myObjectState.stalled,
    fSimL.myObjectState.speed,
    fSimL.myObjectState.alt_ft,
    fSimL.myObjectState.hdg,
    fSimL.myObjectState.roll,
    fSimL.myObjectState.throttle,
    fSimL.myObjectState.dTime
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

########################## register_manoeuvre_listeners ###########################
# Example listener registration (adapt the node path to your controls node):

  var register_manoeuvre_listeners = func(mName, self) {
      foreach (var name; ["loop", "immelmann", "aileron_roll", "split_s"]) {
          (func(name) {
              var node = props.globals.getNode(mName ~ "/controls/manoeuvres/" ~ name, 1);
              node.setBoolValue(0);
              setlistener(node, func(n) {
                  if (n.getBoolValue()) {
                    var opts = {};
                    if (contains(self, "manoeuvre_opts")) opts = self.manoeuvre_opts;
                    start_manoeuvre(mName, self, name, opts);
                    n.setBoolValue(0);
                  }
              }, 0, 0);
          })(name);
      }
  };

########################## dispatcher ###########################
# Call from the listeners on the model's controls node. One manoeuvre at a time.
#   start_manoeuvre(myObjectState, "immelmann");
#   start_manoeuvre(myObjectState, "split_s", {n_pull: 4.5, clearance_ft: 1500});
# Returns the manoeuvre hash, or nil if it was refused or one is already running.

var start_manoeuvre = func(mname, self, name, opts = nil) {
    if (self.manoeuvre != nil) {
        print("Manoeuvre " ~ self.manoeuvre.name ~ " is already running");
        return nil;
    }
    if (name == "loop")              start_loop(self, opts);
    else if (name == "immelmann")    start_immelmann(self, opts);
    else if (name == "aileron_roll") start_aileron_roll(self, opts);
    else if (name == "split_s")      start_split_s(self, opts);
    else {
        print("Unknown manoeuvre: " ~ name);
        return nil;
    }
    print("Starting ", name, " for ", mname, " manoeuvre options ...");
    if (opts != nil) debug.dump(opts) else print("No options defined using defaults");
    print("Dumping aircraft performance parameters ...");
    debug.dump(self.perf);
    print("Monitoring flight parameters ...");
    monitorValues(0, 300, 1);    
    return self.manoeuvre;
};

register_manoeuvre_listeners(modelName, myObjectState);
