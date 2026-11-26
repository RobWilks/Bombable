###########################################################
# fSim.nas - 4-Force Vector FDM Integrated
###########################################################
# Change log (newest first):
#   0.2.0  cleanup of items 1-7 agreed in discussion: start_loop defaults now match its guard; speed floors scale with stall speed;
#          monitor no longer reads a nil climb_cache; unknown model name stops with a clear message; unused variable removed;
#          optional perf keys read with perf["key"] so a profile can omit them; stale comments tagged; version number added
#   0.1.0  baseline as uploaded (fSim_1.nas)
# 
# 
# 
#  1. Lift acts perpendicular to the VELOCITY (rotated from body-up about the right wing),
#     with an explicit induced drag K_IND*CL^2. Thrust and drag act along the aircraft
#     axis. 
#  2. Thrust falls with air density: T = T0 * (rho/rho0)^THRUST_LAPSE. Before, level speed
#     rose with altitude (353 kt at 15,000 ft against max_speed_mps of 300 kt).
#     Adopt 1 and 2 together: lapse alone makes the Immelmann stall at altitude.
#  3. Timing: each tick advances the state by the MEASURED elapsed sim time (your tNode), not the
#     nominal 0.1 s. The timer fires on frame boundaries, so a nominal 0.1 s arrives after 0.1167 s
#     at 60 fps (7 frames) and 0.100 s at 120 fps: with a nominal dt the dynamics ran 15 percent
#     slow and lat/lon (integrated by the animation loop in real time) outran the state.
#     Compute is unchanged: one step per tick. Optional sub-stepping is set by MAX_SUBSTEP.
#  4. Energy-consistent velocity update (see update_force_physics): removes the explicit-Euler
#     energy error without sub-stepping.
#  5. Optional profiling: run prof.start(), run a manoeuvre, then prof.stop() and prof.report().
#  6. Keep physics_update_interval at 0.1 s or less: a 175 deg/s roll moves 17 degrees per tick.


#CLAUDE+ Change log (newest first):
#CLAUDE+   0.7.3.4 approach: leaving the funnel is no longer an abort while the closed form check says the gate can still be reached, the aircraft drops back to the intercept phase;
#CLAUDE+          the heading of a turning reference is smoothed before it turns the line; the climb rate of the reference is fed forward into the height command
#CLAUDE+   0.7.3.3 approach: an error far outside the funnel aborts at once, and the go-around holds the present heading 
#          unless the aircraft is already close to the course
#CLAUDE+   0.7.3.2 approach: the refusal for an aircraft already past the gate says when reversing the course would fit; 
#          the course_deg and course_rel comment states the directions explicitly
#CLAUDE+   0.7.3.1 phase 0 takes the closing speed from the speed flown now when that is below the approach speed; the approach local variable geo (a FlightGear namespace) is now nl; an on_abort function is called with no manoeuvre running so that it can start one (it was refused before)
#CLAUDE+   0.7.3  approach: a guidance layer above cruise() that flies the aircraft onto a line (reference point or moving object, course, glide angle,
#CLAUDE+          gate), phase 0 feasibility check, phase 1 intercept, phase 2 tracking inside a funnel, the gate check against the entry conditions of
#CLAUDE+          an optional successor manoeuvre (then), abort with a reason and a default go-around, result in state.last_approach. Geometry (turn
#CLAUDE+          radius, intercept and settle times) comes from the aircraft profile. New optional perf keys max_turn_rate_dps (also read by
#CLAUDE+          update_roll_target, default 3), approach_speed_mps, goaround_margin_ft.
#CLAUDE+          Series runner: start accepts lat and lon, rows show the approach result.
#CLAUDE+          No change to the other manoeuvres
#CLAUDE+   0.7.2.1 series runner returns the aircraft to its starting position, altitude, heading and speed when the series ends (option home, default 1;
#CLAUDE+          run_series_stop(1) does the same); profiler also keeps osum so the report shows omean, the mean cost of the samples over 1 ms
#CLAUDE+   0.7.2  profiler keeps the maximum over ALL samples and a separate outlier maximum (omax); the dive pull-out starts when the altitude is
#CLAUDE+          expected to cross the pull-out height within half a tick (was: the first tick below it, so on average half a tick late);
#CLAUDE+          series runner (run_series, series_cases, hook series_step in run_flight_physics) for repeatable batches of any manoeuvre:
#CLAUDE+          test code, idle unless started. Target and release code is unchanged and is not being extended
#CLAUDE+   0.7.1  start_dive takes an optional target (hash or AI property path): check_reference gate (speed limit), an approach phase that
#CLAUDE+          flies level until the predicted landing point reaches the reference, then push-over; projectile released at the start of the pull-out
#CLAUDE+          if the predicted miss is within release_tol_m, otherwise a no-release pull-out. predict_dive also returns the ground distance and
#CLAUDE+          time to the pull-out point. New: dive_aim_prediction, dive_aim_error, dive_approach, dive_release, option preview and monitor_rows.
#CLAUDE+          Heading is held (no steering onto the reference yet); dives without a reference are unchanged
#CLAUDE+   0.7.0  reference record and tracker (hash or AI property path, dead reckoning, validity check), local east/north helpers,
#CLAUDE+          ballistic projectile with drag, predicted landing point and miss, release_projectile/update_projectiles;
#CLAUDE+          start_dive n_push default -1. No change to the control laws: aiming itself comes in 0.7.1 and later
#CLAUDE+   0.6.1  monitorValues runs on a maketimer (created next to the physics timer) instead of a chain of settimer calls: one sample
#CLAUDE+          per interval, no duplicate rows, restarting it cannot create a second chain
#CLAUDE+   0.6.0  new dive manoeuvre (start_dive) for profiles with terminal_speed_mps (Ju87 only so far): air brakes raise CD0 (value from the terminal
#CLAUDE+          speed, 1 s ramp each way), straight dive at dive_angle_deg, pull-out started at pullout_agl_ft at the load limit, zoom to an
#CLAUDE+          altitude set by the excess speed at the base, then cruise. predict_dive refuses unsafe dives.
#CLAUDE+   0.5.0  set_targets starts a climb (start_climb) when the new altitude needs one, optional fifth argument sched
#CLAUDE+          (max_rate default, max_angle, economy); start_climb stays quiet for descents
#CLAUDE+   0.4.0  new climb mode: start_climb(opts) with schedule max_rate (Vy), max_angle (Vx) or economy (max (T-D)/T); it reverts
#CLAUDE+          to cruise when cruise can handle the altitude change; best_climb_speed(self, mode) with a per-mode cache;
#CLAUDE+          update_pitch_target is altitude capture plus protection (reference speed 1.2 x stall unless the caller passes one);
#CLAUDE+          v_climb perf key and its override deleted
#CLAUDE+   0.3.0  loop guard also refuses when the predicted top speed is below LOOP_MIN_TOP_RATIO x the stall speed; per-aircraft loop defaults
#CLAUDE+          (perf loop_entry_speed_mps, loop_n_bottom, loop_min_top_ratio); new cruise() base mode (heading, altitude and, with
#CLAUDE+          auto_throttle, speed hold) runs after every manoeuvre and restores the pre-manoeuvre throttle mode; set_targets gains spd
#CLAUDE+   0.2.0  cleanup of start_loop defaults, speed floors, monitor nil guard, unknown-model message, optional perf keys, version number
#CLAUDE+   0.1.0  baseline as uploaded
var FSIM_VERSION = "0.7.3.4";
# Explicitly define fS
var fS = {
    version: FSIM_VERSION,
    state: nil,

    # note roll and pitch targets are not set here.  They are outputs, calculated by functions to update altitude and heading
    # The rule is that manoeuvres don’t touch the user targets below until their documented hand-back.
    set_targets: func(hdg = nil, alt = nil, thr = nil, spd = nil, sched = nil) {   #CLAUDE~ 0.5.0: sched = climb schedule   #CLAUDE~ 0.3.0: spd added (target speed, kts)
        if (me.state == nil) {
            print("Error: fS.state is not initialized.");
            return;
        }
        if (hdg != nil)   me.state.tgt_heading = hdg;
        if (alt != nil)   me.state.tgt_altitude_ft = alt;
        if (thr != nil) {   #CLAUDE~ 0.3.0: a throttle set by hand turns the cruise auto throttle off
            me.state.throttle = thr;
            me.state.auto_throttle = 0;
        }
        if (spd != nil) {   #CLAUDE~ 0.3.0: a target speed turns it on: cruise then holds tgt_speed with the throttle (if both are given, spd wins)
            me.state.tgt_speed = spd;
            me.state.auto_throttle = 1;
        }
        #CLAUDE+ 0.5.0: a new altitude is handed to start_climb, which decides whether cruise can fly it. Not while a manoeuvre is
        #CLAUDE+ running: the target is stored and the manoeuvre hands back to cruise (or the running climb flies to it).
        if (alt != nil and me.state.manoeuvre == nil) {
            if (start_climb(me.state, { schedule: sched or "max_rate" }) != nil)
                me.state.pre_manoeuvre = { auto: me.state.auto_throttle, thr: me.state.throttle };
        }
        
        print(sprintf("fSim: Target Update -> Hdg: %.1f° | Alt: %.1f ft | Thr: %.2f", 
            me.state.tgt_heading, me.state.tgt_altitude_ft, me.state.throttle));
    },
    
    get_state: func() {
        if (me.state == nil) return;
        print(sprintf("fSim: Current State -> Hdg: %.1f (Tgt: %.1f) | Spd: %.1f kts | Alt: %.1f ft (Tgt: %.1f) | Thr: %.2f", 
            me.state.hdg, me.state.tgt_heading, me.state.speed, me.state.alt_ft, me.state.tgt_altitude_ft, me.state.throttle));
    },

    ########################## profiling ###########################
    # Set prof.enable = 1, run for a while, then call prof.report() from the Nasal console.
    # systime() is wall-clock seconds with sub-microsecond fraction; max >> mean means GC or
    # a frame hitch landed inside the timed section.
    # What the profile terms measure:
    # All timings come from run_flight_physics and the animation manager:
    # anim is the animation manager's per-frame update. It runs about five times per physics tick, reads the model's position and attitude nodes, advances them by the last rates, and writes them back.
    # control is the controller part of a physics tick: the manoeuvre's update (or cruise()) plus update_orientation. 
    # All of the approach re-prediction lands here.
    # force is update_force_physics, the four-force physics: lift, drag, thrust and weight, the velocity update, and the position state.
    # push is everything after the physics step: copying the rates and attitude into the animation manager and writing the model's nodes.
    # tick is the whole run_flight_physics call. It also includes the projectile update and the series hook, which is why it is slightly more than the sum of the other three.

    prof: {
        enable: 0,
        stats: {},
        timestamps_1: [],
        timestamps_5: [],
        add: func(name, sec) {
            var st = me.stats[name];
            if (st == nil) {
                st = { n: 0, sum: 0.0, max: 0.0, omax: 0.0, osum: 0.0, over1: 0, over5: 0 };   #CLAUDE~ 0.7.2: omax = largest outlier (over 1 ms); 0.7.2.1: osum = their total
                me.stats[name] = st;
            }
            if (sec > st.max) st.max = sec;                         #CLAUDE+ 0.7.2: max over ALL samples, outliers included
            if (sec > 0.001 and sec > st.omax) st.omax = sec;       #CLAUDE+ 0.7.2: largest outlier
            if (sec > 0.001) st.osum += sec;                        #CLAUDE+ 0.7.2.1: total time of the outliers
            if (sec > 0.005) {
                st.over5 += 1;
                append(me.timestamps_5, systime());
            }
            elsif (sec > 0.001) {
                st.over1 += 1;
                append(me.timestamps_1, systime());
            }
            else
            {
                st.n += 1;
                st.sum += sec;
            }
            # st.n += 1;
            # st.sum += sec;
            # if (sec > st.max) st.max = sec;
            # if (sec > 0.001) st.over1 += 1;
            # if (sec > 0.005) st.over5 += 1;
        },
        start: func() {
            me.stats = {};
            me.timestamps_1 = [];
            me.timestamps_5 = [];
            me.t0 = systime();
            me.enable = 1;
        },
        stop: func() {
            me.enable = 0;
            me.report();
        },        
        report: func() {
            foreach (var k; keys(me.stats)) {
                var st = me.stats[k];
                print(sprintf("%-9s n=%6d  mean=%7.1f us  max=%8.1f us  omax=%8.1f us  omean=%8.1f us  >1ms: %d  >5ms: %d",
                            k, st.n, 1.0e6 * st.sum / math.max(st.n, 1), 1.0e6 * st.max, 1.0e6 * st.omax,
                            1.0e6 * st.osum / math.max(st.over1 + st.over5, 1), st.over1, st.over5));   #CLAUDE~ 0.7.2: n and mean cover the samples under 1 ms only; max covers all; omax is the largest sample over 1 ms
            }
            print("--- Timestamps of outliers (>5ms) ---");
            foreach (var val; me.timestamps_5) print(sprintf("%.5f", val));
            print("--- Timestamps of outliers (>1ms) ---");
            foreach (var val; me.timestamps_1) print(sprintf("%.5f", val));
        },
        reset: func() { 
            me.stats = {}; 
            me.timestamps_1 = [];
            me.timestamps_5 = [];
        },
    },
};


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


########################## thrust_at ###########################
# Thrust at full throttle for air density rho and speed v (m/s).
# perf.max_thrust_N is the thrust at max_speed_mps (where thrust = drag), so it is the LOW end of
# what a propeller gives. A propeller's thrust falls with speed, roughly T ~ 1/V^p with p between
# 0 (constant thrust, a jet-like model) and 1 (constant power). perf.thrust_speed_exp sets p
# (default 0 = unchanged behaviour; 0.53 reproduces the published 5.7 m/s sea-level climb of the
# fkdr1). Below 0.4 x max speed the thrust is held at its 0.4 x max speed value (static thrust).
var thrust_at = func(self, v, rho) {
    var T = self.perf.max_thrust_N * math.pow(rho / rho0, THRUST_LAPSE);
    var p = self.perf["thrust_speed_exp"] or 0.0;
    if (p > 0.0) {
        var vmax = self.perf.max_speed_mps;
        T *= math.pow(vmax / math.max(v, 0.4 * vmax), p);
    }
    return T;
};




########################## thrust_available ###########################
# Thrust available at full throttle at the current speed and altitude (N)
var thrust_available = func(self) {
    return thrust_at(self, self.speed * kts_to_mps, air_density(self.alt_ft * 0.3048));
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
    var MAX_TURN_RATE = self.perf["max_turn_rate_dps"] or 3.0;   # deg/s (standard rate)   #CLAUDE~ 0.7.3: optional perf key, default unchanged

    var rate_cmd = diff * K_HDG;
    if (rate_cmd >  MAX_TURN_RATE) rate_cmd =  MAX_TURN_RATE;
    if (rate_cmd < -MAX_TURN_RATE) rate_cmd = -MAX_TURN_RATE;

    var v_h = math.sqrt(self.v_east*self.v_east + self.v_north*self.v_north);
    var phi = math.atan2(v_h * rate_cmd * D2R, g) * R2D;

    self.tgt_roll = math.clamp(phi, -self.perf.max_bank_angle, self.perf.max_bank_angle);
};


########################## steady_drag ###########################
# Drag in steady flight at speed v and 1 g (lift = weight * cos(path angle)). The climb limit
# must use this, not the measured drag: measured drag includes the induced drag of the load being
# flown RIGHT NOW (about 16 kN at 4 g against 11.8 kN of thrust for the Spitfire), so a high
# load collapses the allowed climb angle on the next tick, which lowers the load, which reopens
# the limit: a two-tick limit cycle in pitch.
var steady_drag = func(self, v) {
    var rho = air_density(self.alt_ft * 0.3048);
    var qS  = 0.5 * rho * v * v * self.perf.wing_area_m2;
    var CL  = weight_N * math.cos((self.gamma_deg or 0.0) * D2R) / math.max(qS, 1.0);
    return qS * (cd0_now(self) + K_IND * CL * CL);   #CLAUDE~ 0.6.0: was C_D0
};

########################## best_climb_speed ###########################
# Speed that maximises the climb power (T - D) * v for level-ish flight at the current altitude,
# searched on a 24-step grid with a parabolic refinement of the best point.
#CLAUDE+ 0.4.0: mode selects the score maximised over speed:
#CLAUDE+   "rate"    (T - D) * v   best rate of climb, Vy
#CLAUDE+   "angle"   (T - D)       best angle of climb, Vx (the rate score without its final * v)
#CLAUDE+   "economy" (T - D) / T   least mechanical work per unit height (work = W * T / (T - D))
var best_climb_speed = func(self, mode = "rate") {
    var vs1  = v_stall_1g(self);
    var rho  = air_density(self.alt_ft * 0.3048);
    var S    = self.perf.wing_area_m2;
    var v_lo = 1.15 * vs1;
    var v_hi = 1.1 * self.perf.max_speed_mps;
    var n    = 24;
    var h    = (v_hi - v_lo) / n;
    var r    = [];
    var best = 0;
    for (var i = 0; i <= n; i += 1) {
        var v    = v_lo + h * i;
        var qS   = 0.5 * rho * v * v * S;
        var CL   = weight_N / qS;
        var rate = -1.0e9;
        if (CL <= 0.9 * CL_MAX) {
            var T  = thrust_at(self, v, rho);
            var ex = T - qS * (C_D0 + K_IND * CL * CL);
            if (mode == "angle")        rate = ex;
            else if (mode == "economy") rate = ex / math.max(T, 1.0);
            else                        rate = ex * v;
        }
        append(r, rate);
        if (rate > r[best]) best = i;
    }
    var v_best = v_lo + h * best;
    if (best > 0 and best < n and r[best - 1] > -1.0e8) {
        var den = r[best - 1] - 2.0 * r[best] + r[best + 1];
        if (den < 0.0) v_best += 0.5 * h * (r[best - 1] - r[best + 1]) / den;
    }
    return v_best;
};

########################## climb_speed ###########################
# Reference climb speed in metres per sec, recomputed only when the altitude has changed by 500 ft, never below
# 1.2 x the 1 g stall speed.
#CLAUDE~ 0.4.0: mode argument; one cache entry per mode (c[mode] = {alt_ft, v}); c["v"] is the last speed returned (read by monitorValues)
var climb_speed = func(self, mode = "rate") {
    var vs1 = v_stall_1g(self);
    var c = self.climb_cache;
    if (c == nil) c = {};
    var e = c[mode];
    if (e == nil or math.abs(self.alt_ft - e.alt_ft) > 500.0) {
        e = { alt_ft: self.alt_ft, v: best_climb_speed(self, mode) };
        c[mode] = e;
    }
    c["v"] = math.max(e.v, 1.2 * vs1);
    self.climb_cache = c;
    return c["v"];
};

########################## climb control parameters ###########################

var CLIMB_K_SPEED = 0.5;     # 1/s   speed error -> desired acceleration
var CLIMB_A_MAX   = 1.5;     # m/s^2 limit on that acceleration
var CLIMB_K_GAMMA = 0.8;     # 1/s   path-angle error -> path curvature rate
var CLIMB_TAU     = 0.3;     # s     time constant of the low-pass filter on the commanded path angle

# Filter state. This is anything a function remembers between ticks. 
# In update_pitch_target, for example, gamma_cmd_f holds the smoothed climb angle. 
# Any filter state remembered carries a timestamp or is reset when its mode starts.

########################## update_pitch_target ###########################
# Needs:
#   command_path, v_stall_1g, thrust_available, thrust_at, air_density, K_IND, C_D0, CL_MAX, weight_N, g, D2R
#
# The method for setting pitch has three steps; pitch itself is not capped (acrobatics need +/-180), the angle
# of attack is:
#   1. altitude error -> wanted path angle (as before)
#   2. a climb is limited to the path angle the excess thrust can sustain while the speed is held
#      near the best-climb speed:  sin(gamma_max) = (T - D)/W - a_des/g,  a_des = K (v_ref - v)
#      (when slow, a_des > 0 and the climb flattens; when fast, speed may be traded for height)
#   3. fly the path angle: path curvature -> load factor -> angle of attack -> pitch target
#      (command_path), which clamps alpha to 0.9 x the stall angle and the load factor to the
#      structural and stall limits, using the MEASURED path angle.
#
#CLAUDE+ 0.4.0: v_ref (m/s) is the speed the climb limit in step 2 tries to hold. Default 1.2 x the 1 g stall speed, which makes this
#CLAUDE+ function altitude capture plus protection. update_climb_target passes the schedule speed instead.
# vs is vertical speed in m/s, v_h is horizontal speed in m/s, speed is total speed in m/s.

var update_pitch_target = func(self, dt, v_ref = nil) {
    var v_h   = math.sqrt(self.v_east * self.v_east + self.v_north * self.v_north);
    var speed = math.sqrt(v_h * v_h + self.v_up * self.v_up);
    var vs1   = v_stall_1g(self);
    var v     = math.max(speed, 0.5 * vs1);       # floor scales with the aircraft, not a fixed 20 m/s
    if (v_ref == nil) v_ref = 1.2 * vs1;              #CLAUDE+ 0.4.0

    # 1. path angle wanted for the altitude error
    var alt_err_m = (self.tgt_altitude_ft - self.alt_ft) / 3.28084;
    var vs_cmd = math.clamp(alt_err_m * 0.1,
                            -self.perf.descent_rate * 0.00508,
                             self.perf.climb_rate   * 0.00508);
    var gamma_cmd = math.atan2(vs_cmd, math.max(v_h, 1.0));

    # 2. climb (or level) no steeper than the thrust can sustain at the speed we want to hold
    if (gamma_cmd >= 0.0) {
        #CLAUDE- var v_ref  = climb_speed(self);
        var excess = (thrust_available(self) - steady_drag(self, v)) / weight_N; # excess force - dimensionless
        var a_des  = math.clamp(CLIMB_K_SPEED * (v_ref - v), -CLIMB_A_MAX, CLIMB_A_MAX); # calculate the desired acceleration from the speed error
        var s      = math.clamp(excess - a_des / g, -0.95, 0.95);
        gamma_cmd  = math.min(gamma_cmd, math.atan2(s, math.sqrt(1.0 - s * s)));
    }

    # Low-pass the COMMANDED path angle (outside the feedback loop, so it adds no lag to the
    # control itself). The coefficient follows the time step, so the smoothing is the same
    # whether the physics ticks at 6 Hz or 20 Hz. Restart the filter after a gap, e.g. when a
    # manoeuvre has been running instead of this law.
    var now = self.time or 0.0;
    if (self.gamma_cmd_f == nil or now - (self.gamma_cmd_t or 0.0) > 1.0) {
        self.gamma_cmd_f = gamma_cmd;
    } else {
        var kf = 1.0 - math.exp(-dt / CLIMB_TAU);
        self.gamma_cmd_f += kf * (gamma_cmd - self.gamma_cmd_f);
    }
    self.gamma_cmd_t = now;
    gamma_cmd = self.gamma_cmd_f;


    # 3. fly the path angle with the angle of attack limited (command_path sets tgt_pitch)

    # No pitch-rate lead term here: it is meant for a steadily rotating path (loops), and at a
    # step in the altitude demand it adds a spurious +1 g kick.
    var gdot = math.clamp(CLIMB_K_GAMMA * (gamma_cmd - (self.gamma_deg or 0.0) * D2R), -0.5, 0.5); # gdot is change in gamma per second, limited to 0.5 rad/s
    command_path(self, v, gdot, 0, dt, 0);
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
#
var update_force_physics = func(self, dt) {
    if (dt <= 0.0) return;

    #CLAUDE+ 0.6.0: air brakes move towards brake_cmd (0 or 1) at 1/DIVE_BRAKE_TIME per second
    var db = dt / DIVE_BRAKE_TIME;
    self.brake = math.clamp(self.brake_cmd, self.brake - db, self.brake + db);

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
    var CD = cd0_now(self) + K_IND * CL * CL;   #CLAUDE~ 0.6.0: was C_D0

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

    # Velocity update. A plain Euler step adds speed whenever the force has a component
    # perpendicular to the velocity (a = 4 g of lift: +5 kt at 0.1 s, +16 kt at 0.25 s at the top
    # of an Immelmann). Keep the direction from the Euler step but take the speed from the
    # along-track acceleration alone; error at a 0.117 s step drops to about 1 kt, at no extra cost.
    var ve1 = ve + ax * dt;
    var vn1 = vn + ay * dt;
    var vu1 = vu + az * dt;
    if (speed > 1.0) {
        var a_track = (ax * ve + ay * vn + az * vu) / speed;
        var sp1 = math.sqrt(ve1*ve1 + vn1*vn1 + vu1*vu1);
        var kc  = (speed + a_track * dt) / math.max(sp1, 0.001);
        ve1 *= kc;  vn1 *= kc;  vu1 *= kc;
    }
    self.v_east  = ve1;
    self.v_north = vn1;
    self.v_up    = vu1;

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
#

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

# For manoeuvres that reverse heading: normalise, then steer to the exact reversed heading
# (entry heading + 180; the roll leaves the path 1-4 degrees off) and hold the new altitude.
var finish_reversal = func(self, hdg_target) {
    normalise_attitude(self);
    self.tgt_heading     = hdg_target;
    self.tgt_altitude_ft = self.alt_ft;
};

########################## v_stall_1g ###########################
var v_stall_1g = func(self) {
    var rho = air_density(self.alt_ft * 0.3048);
    return math.sqrt(2.0 * weight_N / (rho * self.perf.wing_area_m2 * CL_MAX)); # in mps
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
    var n_pos = math.min(0.9 * (self.perf["n_max_pos"] or 6.0), n_stall);
    var n_neg = math.min(0.9 * math.abs(self.perf["n_max_neg"] or 3.0), n_stall);
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
var command_path = func(self, v, gdot, rolling, dt, use_lead = 1) {
    var c = math.cos(self.roll * D2R);
    var X = v * gdot / g + math.cos((self.gamma_deg or 0.0) * D2R);
    var n = 0.0;
    if (rolling) {
        n = 2.0 * c * X;
    } else {
        if (math.abs(c) < 0.3) c = (c < 0.0) ? -0.3 : 0.3;
        n = X / c;
    }
    return command_load(self, v, n, use_lead ? gdot * dt * R2D : 0.0);
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
    var v   = math.max(self.speed * kts_to_mps, 0.5 * v_stall_1g(self));
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
    var rr   = self.aerobatic ? (self.perf["max_roll_rate_aero"] or 90.0) : self.perf.max_roll_rate;
    var step = math.min(math.min(m.roll_rate, rr) * dt, math.max(remaining, 0.0));
    self.tgt_roll = wrap180(self.roll + m.dir * step);
    return remaining;
};

# Roll stepper B: roll in direction m.dir to an absolute roll angle (e.g. 180 = inverted),
# whatever the starting bank. Returns degrees remaining.
var step_roll_to = func(self, m, dt, target) {
    var remaining = wrap360(m.dir * (target - self.roll));
    if (remaining > 350.0) remaining = 0.0;
    var rr   = self.aerobatic ? (self.perf["max_roll_rate_aero"] or 90.0) : self.perf.max_roll_rate;
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
        roll_rate: opts["roll_rate"] or (self.perf["max_roll_rate_aero"] or 90.0),
        roll_swept: 0.0, roll_prev: self.roll, stall_t: 0.0,
        update: update_aileron_roll_target
    };
    if (imm) self.aerobatic = 1;
    return self.manoeuvre;
};

####################### update_aileron_roll_target ###########################
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

########################### 1. Immelmann ###########################
# Half loop up (constant pitch rate, as the loop), then a 180 degree roll to upright started
# roll_lead_deg before the path reaches inverted, ending level and upright with the heading
# reversed and the height gained. Phases: entry -> pull -> roll -> recover.
#
# The roll is made at the top, near stall speed, so energy decides feasibility. start_immelmann
# predicts the top speed (within about 5 kt against the logs) and refuses if it would be below
# 1.1 x the 1 g stall speed there (opts.force = 1 overrides).
#
# opts: entry_speed (m/s, default 0.97*max_speed), n_bottom (default min(5, 0.9*n_max_pos):
#       a tighter loop reaches the top with more speed), roll_lead_deg (default 20),
#       direction ("right"/"left"), roll_rate, force

########################### predict_loop_top ###########################
# Predicts the top-of-loop state of the constant-pitch-rate half loop (same law and force model
# as the manoeuvre). Returns { v, h, ok }: speed (m/s) and height gain (m) where the path angle
# reaches gm_end (default 160 degrees, the Immelmann roll point; pass PI for the top of a full
# loop), and ok = 1 only if it got there. If the speed falls below half the 1 g stall speed
# on the way, the aircraft stalls out of the manoeuvre: ok = 0 and the values are not a top state.

var predict_loop_top = func(self, v0, n_bottom, gm_end = nil) {
    var S = self.perf.wing_area_m2;
    var n_cap = 0.9 * (self.perf["n_max_pos"] or 6.0);
    var q = g * (n_bottom - 1.0) / v0;
    var goal = gm_end or (PI - 0.35);
    var v_floor = 0.5 * v_stall_1g(self);
    var dt = 0.05;
    var v = v0;  var gm = 0.0;  var h = 0.0;  var t = 0.0;  var ok = 0;
    while (t < 90.0) {
        var rho = air_density(self.alt_ft * 0.3048 + h);
        var qS  = 0.5 * rho * v * v * S;
        var n   = math.min(v * q / g + math.cos(gm),
                           math.min(n_cap, 0.85 * qS * CL_MAX / weight_N));
        var CL  = n * weight_N / math.max(qS, 1.0);
        var al  = (CL - C_L_vmax) / CL_ALPHA;
        var D   = qS * (C_D0 + K_IND * CL * CL);
        var Ta  = thrust_at(self, v, rho);
        # lift is perpendicular to the velocity (physics v2): only drag and thrust act along it
        v += ((Ta - D) * math.cos(al) / self.perf.mass_kg - g * math.sin(gm)) * dt;
        if (v < v_floor) {
            v = v_floor;
            break;
        }
        h += v * math.sin(gm) * dt;
        gm += g * (n - math.cos(gm)) / math.max(v, v_floor) * dt;
        t += dt;
        if (gm >= goal) {
            ok = 1;
            break;
        }
    }
    return { v: v, h: h, ok: ok };
};


########################### start_immelmann ###########################

var start_immelmann = func(self, opts = nil) {
    opts = opts or {};
    var v_e = opts["entry_speed"] or (0.97 * self.perf.max_speed_mps);
    var n_b = opts["n_bottom"] or math.min(5.0, 0.9 * (self.perf["n_max_pos"] or 6.0));

    if (!opts["force"]) {
        var pr    = predict_loop_top(self, v_e, n_b, PI - 0.35);
        var probe = { alt_ft: self.alt_ft + pr.h / 0.3048, perf: self.perf };
        var vs    = v_stall_1g(probe);
        if (!pr.ok) {
            print(sprintf("Immelmann refused: the predicted half loop stalls before the top (speed falls to %.0f kt)",
                          pr.v / kts_to_mps));
            return nil;
        }
        if (pr.v < 1.1 * vs) {
            print(sprintf("Immelmann refused: predicted top speed %.0f kt is below 1.1 x the %.0f kt stall speed there",
                          pr.v / kts_to_mps, vs / kts_to_mps));
            return nil;
        }
    }

    self.manoeuvre = {
        name: "immelmann", phase: "entry", t: 0.0,
        entry_speed: v_e, n_bottom: n_b,
        roll_lead_deg: opts["roll_lead_deg"] or 20.0,
        dir: (opts["direction"] == "left") ? -1.0 : 1.0,
        roll_rate: opts["roll_rate"] or (self.perf["max_roll_rate_aero"] or 90.0),
        swept: 0.0, gamma_prev: 0.0, q_loop: 0.0, hdg0: self.hdg,
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
            m.hdg0 = self.hdg;
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
            finish_reversal(self, wrap360(m.hdg0 + 180.0));
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

########################## predict_split_s_loss ###########################
var predict_split_s_loss = func(self, v0, n_pull) {
    var S = self.perf.wing_area_m2;
    var m = self.perf.mass_kg;
    var n_cap = 0.9 * (self.perf["n_max_pos"] or 6.0);
    var dt = 0.05;
    var v_floor = 0.5 * v_stall_1g(self);
    var v = v0;  var gp = 0.0;  var loss = 0.0;  var t = 0.0;
    while (gp < PI - 0.05 and t < 90.0) {
        var rho = air_density(self.alt_ft * 0.3048 - loss);
        var qS  = 0.5 * rho * v * v * S;
        var n   = math.min(n_pull, math.min(n_cap, 0.85 * qS * CL_MAX / weight_N));
        var CL  = n * weight_N / math.max(qS, 1.0);
        var al  = (CL - C_L_vmax) / CL_ALPHA;
        var D   = qS * (C_D0 + K_IND * CL * CL);
        gp   += g * (n + math.cos(gp)) / math.max(v, v_floor) * dt;
        v    += (-D * math.cos(al) / m + g * math.sin(gp)) * dt;       # idle thrust in the dive
        loss += v * math.sin(gp) * dt;
        t += dt;
    }
    var rr = self.perf["max_roll_rate_aero"] or 90.0;
    return 1.2 * loss + 0.25 * g * (180.0 / rr) * (180.0 / rr);     # margin + half-roll sink
};

########################## start_split_s ###########################
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
        roll_rate: opts["roll_rate"] or (self.perf["max_roll_rate_aero"] or 90.0),
        roll_swept: 0.0, roll_prev: self.roll,
        swept: 0.0, gamma_prev: 0.0, stall_t: 0.0, hdg0: self.hdg,
        update: update_split_s_target
    };
    if (imm) self.aerobatic = 1;
    return self.manoeuvre;
};

########################## update_split_s_target ###########################
var update_split_s_target = func(self, dt) {
    var m   = self.manoeuvre;
    var v   = self.speed * kts_to_mps;
    var gam = (self.gamma_deg or 0.0) * D2R;
    m.t += dt;

    if (m.phase == "entry") {
        var r = run_entry(self, m, dt);
        if (r == 1) {
            m.phase = "roll";  m.t = 0.0;
            m.hdg0 = self.hdg;
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
        var gdot = g * (n_t * c - math.cos(gam)) / math.max(v, 0.5 * v_stall_1g(self));
        gdot = math.max(gdot, -m.k_capture * rem);
        command_path(self, v, gdot, 0, dt);

        m.swept += wrap180(m.gamma_prev - self.gamma_deg);       # path angle decreasing
        m.gamma_prev = self.gamma_deg;
        if (180.0 - m.swept <= m.exit_deg or m.t > 60.0) {
            finish_reversal(self, wrap360(m.hdg0 + 180.0));
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

#CLAUDE~ 0.3.0: new. Loop entry speed (m/s) and design load factor at the bottom: the aircraft profile if it sets
#CLAUDE+ loop_entry_speed_mps / loop_n_bottom, otherwise 0.97 x max speed and min(5, 0.9 x n_max_pos).
var loop_defaults = func(self) {
    return {
        v: self.perf["loop_entry_speed_mps"] or (0.97 * self.perf.max_speed_mps),
        n: self.perf["loop_n_bottom"] or math.min(5.0, 0.9 * (self.perf["n_max_pos"] or 6.0))
    };
};

#CLAUDE~ 0.3.0: new. A loop passes the top at low g, so its top speed may be below the 1 g stall speed (Spitfire loop: 0.86 x, fine);
#CLAUDE+ well below that the aircraft stalls out (fkdr1 loop: 0.19 x, tail slide). A profile can override with loop_min_top_ratio.
var LOOP_MIN_TOP_RATIO = 0.7;

var start_loop = func(self, opts = nil) {
    opts = opts or {};
    var d   = loop_defaults(self);   #CLAUDE~ 0.3.0: profile defaults, see loop_defaults
    var v_e = opts["entry_speed"] or d.v;
    var n_b = opts["n_bottom"] or d.n;

    if (!opts["force"]) {
        var pr    = predict_loop_top(self, v_e, n_b, PI);
        var probe = { alt_ft: self.alt_ft + pr.h / 0.3048, perf: self.perf };
        var vs    = v_stall_1g(probe);
        if (!pr.ok) {
            print(sprintf("Loop refused: predicted top speed %.0f kt is below the %.0f kt stall speed there",
                          pr.v / kts_to_mps, vs / kts_to_mps));
            return nil;
        }
        var min_ratio = self.perf["loop_min_top_ratio"] or LOOP_MIN_TOP_RATIO;   #CLAUDE~ 0.3.0: a profile can override the default
        if (pr.v < min_ratio * vs) {   #CLAUDE~ 0.3.0: new rule
            print(sprintf("Loop refused: predicted top speed %.0f kt is below %.2f x the %.0f kt stall speed there",
                          pr.v / kts_to_mps, min_ratio, vs / kts_to_mps));
            return nil;
        }
    }

    self.manoeuvre = {
        name: "loop", phase: "entry", t: 0.0,
        entry_speed: v_e,                                    # m/s
        n_bottom:    n_b,                                    # design load factor at the bottom
        exit_deg:    opts["exit_deg"] or 3.0,             # hand back to normal control this far from 360
        floor_ft:    opts["floor_ft"] or (self.alt_ft - 1000.0),
        swept: 0.0, gamma_prev: 0.0, stall_t: 0.0, q_loop: 0.0,
        update: update_loop_target
    };
    return self.manoeuvre;
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
        var n_allow = math.min(0.9 * (self.perf["n_max_pos"] or 6.0), n_stall);

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
# One step per tick of the measured elapsed sim time. Late ticks are capped at MAX_DT (after a
# hitch or pause the state does not try to catch up). MAX_SUBSTEP splits a tick into equal
# sub-steps of at most that size, running the whole control + orientation + force loop each
# time; 0.25 means never (one step). With the energy-consistent velocity update this is not
# needed for accuracy; lower it (0.05) only if profiling shows spare time and you want the
# controller discretisation error smaller too.

var MAX_SUBSTEP = 0.25;
var MAX_DT      = 0.25;

########################## reference tracking and projectile (0.7.0) ###########################
#CLAUDE+ Target: a hash { lat, lon, alt_ft, hdg, spd_kt } (a snapshot, moved at constant velocity by dead reckoning) or the property path of an
#CLAUDE+ AI object, e.g. "/ai/models/ship[0]" (read each call). make_reference turns either into a record; reference_state(t) gives the present state.
#CLAUDE+ All distances below are metres east (e) / north (n) in a flat local frame, using m_per_deg_lat / m_per_deg_lon.
#CLAUDE+ Projectile: point mass with quadratic drag (spec = { mass_kg, diameter_m, cd }), x east, y north, z up (MSL, m).
var REFERENCE_MAX_KT        = 40.0;     # references faster than this are refused by check_reference (option target_max_kt)
var REFERENCE_STATIONARY_KT = 2.0;      # at or below this the target counts as stationary (option stationary_kt)
var PROJECTILE_DT        = 0.05;     # s, integration step
var FUEL_TANK_500 = { mass_kg: 500.0, diameter_m: 0.47, cd: 0.3 };   # approximate Cd on frontal area: not from a source

var sim_time = func { return getprop("/sim/time/elapsed-sec") or 0.0; };

var local_en = func(ref_lat, ref_lon, lat, lon) {
    return { e: (lon - ref_lon) * m_per_deg_lon, n: (lat - ref_lat) * m_per_deg_lat };
};
var lat_lon_at = func(ref_lat, ref_lon, e, n) {
    return { lat: ref_lat + n / m_per_deg_lat, lon: ref_lon + e / m_per_deg_lon };
};

# Own position: the model's position nodes (integrated by the animation manager), altitude from the state.
var own_position = func(self) {
    return { lat: latNode.getValue() or self.lat, lon: lonNode.getValue() or self.lon, alt_ft: self.alt_ft };
};

var make_reference = func(spec) {
    if (spec == nil) return nil;
    if (typeof(spec) == "hash") {
        if (spec["lat"] == nil or spec["lon"] == nil) {
            print("Target needs lat and lon");
            return nil;
        }
        return { path: nil, lat: spec["lat"], lon: spec["lon"], alt_ft: spec["alt_ft"] or 0.0,
                 hdg: spec["hdg"] or 0.0, spd_kt: spec["spd_kt"] or 0.0, t0: sim_time() };
    }
    return { path: spec, t0: sim_time() };
};

# Present reference state { lat, lon, alt_ft, hdg, spd_kt }, or nil when an AI object is missing or not valid.
var reference_state = func(t, now = nil) {
    if (t == nil) return nil;
    if (t.path != nil) {
        var p = t.path;
        var valid = getprop(p ~ "/valid");
        var lat = getprop(p ~ "/position/latitude-deg");
        var lon = getprop(p ~ "/position/longitude-deg");
        if (lat == nil or lon == nil or (valid != nil and !valid)) return nil;
        var spd = getprop(p ~ "/velocities/true-airspeed-kt");
        if (spd == nil) spd = getprop(p ~ "/velocities/speed-kts");
        return { lat: lat, lon: lon, alt_ft: getprop(p ~ "/position/altitude-ft") or 0.0,
                 hdg: getprop(p ~ "/orientation/true-heading-deg") or 0.0, spd_kt: spd or 0.0 };
    }
    if (now == nil) now = sim_time();
    var d  = (now - t.t0) * t.spd_kt * kts_to_mps;
    var h  = t.hdg * D2R;
    var ll = lat_lon_at(t.lat, t.lon, d * math.sin(h), d * math.cos(h));
    return { lat: ll.lat, lon: ll.lon, alt_ft: t.alt_ft, hdg: t.hdg, spd_kt: t.spd_kt };
};

# Target in the local frame centred on (ref_lat, ref_lon): position, velocity (m/s), altitude.
var reference_local = func(t, now, ref_lat, ref_lon) {
    var s = reference_state(t, now);
    if (s == nil) return nil;
    var en = local_en(ref_lat, ref_lon, s.lat, s.lon);
    var v  = s.spd_kt * kts_to_mps;
    return { e: en.e, n: en.n, vE: v * math.sin(s.hdg * D2R), vN: v * math.cos(s.hdg * D2R),
             alt_ft: s.alt_ft, hdg: s.hdg, spd_kt: s.spd_kt };
};

# Phase 0 check: { ok, why, stationary, state }. opts: reference_max_kt, stationary_kt.
var check_reference = func(t, opts = nil) {
    opts = opts or {};
    var s = reference_state(t);
    if (s == nil) return { ok: 0, why: "reference missing or not valid", stationary: 0, state: nil };
    var vmax  = (opts["reference_max_kt"] != nil) ? opts["reference_max_kt"] : REFERENCE_MAX_KT;
    var vstat = (opts["stationary_kt"] != nil) ? opts["stationary_kt"] : REFERENCE_STATIONARY_KT;
    if (s.spd_kt > vmax)
        return { ok: 0, why: sprintf("reference speed %.0f kt is above the %.0f kt limit", s.spd_kt, vmax), stationary: 0, state: s };
    return { ok: 1, why: "", stationary: (s.spd_kt <= vstat), state: s };
};

var projectile_accel = func(spec, z, vx, vy, vz) {
    var s = math.sqrt(vx * vx + vy * vy + vz * vz);
    var k = 0.5 * air_density(z) * spec.cd * PI * 0.25 * spec.diameter_m * spec.diameter_m / spec.mass_kg * s;
    return [-k * vx, -k * vy, -g - k * vz];
};

# One midpoint (RK2) step. p = { x, y, z, vx, vy, vz, t }.
var step_projectile = func(p, spec, dt) {
    var a  = projectile_accel(spec, p.z, p.vx, p.vy, p.vz);
    var hx = p.vx + 0.5 * dt * a[0];
    var hy = p.vy + 0.5 * dt * a[1];
    var hz = p.vz + 0.5 * dt * a[2];
    var b  = projectile_accel(spec, p.z + 0.5 * dt * p.vz, hx, hy, hz);
    p.x += dt * hx;  p.y += dt * hy;  p.z += dt * hz;
    p.vx += dt * b[0];  p.vy += dt * b[1];  p.vz += dt * b[2];
    p.t += dt;
};

# Step, and if the ground was crossed put the projectile on it (linear interpolation within the step). Returns 1 on landing.
var advance_projectile = func(p, spec, dt, ground_m) {
    var x0 = p.x;  var y0 = p.y;  var z0 = p.z;  var t0 = p.t;
    step_projectile(p, spec, dt);
    if (p.z > ground_m) return 0;
    var f = math.clamp((z0 - ground_m) / math.max(z0 - p.z, 1.0e-9), 0.0, 1.0);
    p.x = x0 + f * (p.x - x0);  p.y = y0 + f * (p.y - y0);  p.t = t0 + f * dt;  p.z = ground_m;
    return 1;
};

# Where a projectile released now (position x,y,z in m, velocity vx,vy,vz in m/s) lands: { x, y, t, speed, hit }.
var predict_projectile = func(x, y, z, vx, vy, vz, ground_m, spec = nil, dt = nil) {
    spec = spec or FUEL_TANK_500;
    if (dt == nil) dt = PROJECTILE_DT;
    var p = { x: x, y: y, z: z, vx: vx, vy: vy, vz: vz, t: 0.0 };
    var hit = (z <= ground_m);
    while (!hit and p.t < 120.0) hit = advance_projectile(p, spec, dt, ground_m);
    return { x: p.x, y: p.y, t: p.t, speed: math.sqrt(p.vx * p.vx + p.vy * p.vy + p.vz * p.vz), hit: hit };
};

# Landing point if released now from the aircraft: e, n are metres from the aircraft, plus lat/lon and fall time.
var landing_point = func(self, spec, ground_ft) {
    var pos = own_position(self);
    var r  = predict_projectile(0.0, 0.0, pos.alt_ft * 0.3048, self.v_east, self.v_north, self.v_up, ground_ft * 0.3048, spec);
    var ll = lat_lon_at(pos.lat, pos.lon, r.x, r.y);
    return { e: r.x, n: r.y, lat: ll.lat, lon: ll.lon, t: r.t, speed: r.speed, hit: r.hit };
};

# Miss if released now: landing point against the reference position at the landing time (reference moved at constant velocity).
# along is positive when the landing is beyond the reference along the aircraft ground track, cross positive to the right of it.
var landing_miss = func(self, tgt, spec = nil, ground_ft = nil) {
    var pos = own_position(self);
    var tl  = reference_local(tgt, sim_time(), pos.lat, pos.lon);
    if (tl == nil) return nil;
    if (ground_ft == nil) ground_ft = tl.alt_ft;
    var ip  = landing_point(self, spec, ground_ft);
    var dx  = ip.e - (tl.e + tl.vE * ip.t);
    var dy  = ip.n - (tl.n + tl.vN * ip.t);
    var trk = math.atan2(self.v_east, self.v_north);
    return { miss: math.sqrt(dx * dx + dy * dy), along: dx * math.sin(trk) + dy * math.cos(trk),
             cross: dx * math.cos(trk) - dy * math.sin(trk), t: ip.t, landing: ip, reference: tl };
};

# Release a projectile from the aircraft's present position and velocity. update_projectiles flies it to the ground.
var release_projectile = func(self, spec = nil, tgt = nil, ground_ft = nil) {
    var pos = own_position(self);
    if (ground_ft == nil) {
        var ts = reference_state(tgt);
        ground_ft = (ts != nil) ? ts.alt_ft : 0.0;
    }
    var pr = { x: 0.0, y: 0.0, z: pos.alt_ft * 0.3048, vx: self.v_east, vy: self.v_north, vz: self.v_up, t: 0.0,
               lat0: pos.lat, lon0: pos.lon, spec: spec or FUEL_TANK_500, tgt: tgt, ground_m: ground_ft * 0.3048,
               trk: math.atan2(self.v_east, self.v_north) };
    append(self.projectiles, pr);
    print(sprintf("Projectile released at %.0f ft, %.0f ft AGL, %.0f m/s", pos.alt_ft, pos.alt_ft - ground_ft, math.sqrt(pr.vx * pr.vx + pr.vy * pr.vy + pr.vz * pr.vz)));
    return pr;
};

var update_projectiles = func(self, dt) {
    var keep = [];
    foreach (var pr; self.projectiles) {
        var n = math.ceil(dt / PROJECTILE_DT);
        var h = dt / n;
        var hit = 0;
        for (var i = 0; i < n and !hit; i += 1) hit = advance_projectile(pr, pr.spec, h, pr.ground_m);
        if (!hit) {
            append(keep, pr);
            continue;
        }
        var ll = lat_lon_at(pr.lat0, pr.lon0, pr.x, pr.y);
        var res = { lat: ll.lat, lon: ll.lon, fall_s: pr.t, miss: nil, along: nil, cross: nil };
        var tl = reference_local(pr.tgt, sim_time(), pr.lat0, pr.lon0);
        if (tl != nil) {
            var dx = pr.x - tl.e;
            var dy = pr.y - tl.n;
            res.miss  = math.sqrt(dx * dx + dy * dy);
            res.along = dx * math.sin(pr.trk) + dy * math.cos(pr.trk);
            res.cross = dx * math.cos(pr.trk) - dy * math.sin(pr.trk);
            print(sprintf("Landing after %.1f s at %.5f, %.5f: miss %.1f m (along %+.1f, cross %+.1f)", pr.t, ll.lat, ll.lon, res.miss, res.along, res.cross));
        } else {
            print(sprintf("Landing after %.1f s at %.5f, %.5f (no reference)", pr.t, ll.lat, ll.lon));
        }
        self.last_landing = res;
    }
    self.projectiles = keep;
};

########################## dive mode (0.6.0) ###########################
#CLAUDE+ start_dive(self, opts): straight dive for aircraft whose profile has terminal_speed_mps.
#CLAUDE+   opts: pullout_agl_ft (height above ground where the pull-out starts, default 2000)
#CLAUDE+         dive_angle_deg (default 70), ground_ft (ground height, default 0), clearance_ft (least predicted base height AGL, default 300)
#CLAUDE+         n_push (load factor while pushing over, default -1), n_pull (default: the limit, 0.9 x n_max_pos), eta (zoom energy efficiency, 0.7),
#CLAUDE+         force (1 = skip the predictor)
#CLAUDE+ Phases: pushover (brakes out, idle, path rotates down) -> dive (hold the angle) -> pullout (brakes in, full throttle, load limit, level off)
#CLAUDE+         -> zoom (spend the excess speed on height; the cruise target altitude is set from it) -> cruise.

var DIVE_ZOOM_A_MAX = 5.0;      # m/s^2 deceleration the zoom may use to trade speed for height

# Point-mass prediction of the whole dive from the present state: pushover, dive to the pull-out height, pull-out.
# Returns { ok, why, v_trig, g_trig_deg, loss (m), base_agl (m), v_base, t }.
var predict_dive = func(self, m) {
    var S = self.perf.wing_area_m2;
    var mass = self.perf.mass_kg;
    var n_cap = math.min(m.n_pull, 0.9 * (self.perf["n_max_pos"] or 6.0));
    var rate  = (self.perf["max_pitch_rate_aero"] or 30.0) * D2R;
    var floor_v = 0.5 * v_stall_1g(self);
    var dt = 0.05;
    var v  = math.max(self.speed * kts_to_mps, floor_v);
    var gm = (self.gamma_deg or 0.0) * D2R;
    var z  = self.alt_ft * 0.3048;
    var h  = (self.alt_ft - m.ground_ft) * 0.3048;
    var h_pull = m.h_pull_ft * 0.3048;
    var brake = 0.0;  var pulling = 0;  var t = 0.0;  var done = 0;
    var v_trig = v;  var g_trig = gm;  var h_trig = h;
    var x = 0.0;  var x_trig = 0.0;  var t_trig = 0.0;  var z_trig = z;   #CLAUDE+ 0.7.1: ground distance flown, and its value at the pull-out point
    while (t < 240.0 and !done) {
        var rho = air_density(z);
        var qS  = 0.5 * rho * v * v * S;
        if (!pulling and h <= h_pull) { pulling = 1; v_trig = v; g_trig = gm; h_trig = h; x_trig = x; t_trig = t; z_trig = z; }   #CLAUDE~ 0.7.1: also x_trig, t_trig, z_trig
        brake = math.clamp(pulling ? 0.0 : 1.0, brake - dt / DIVE_BRAKE_TIME, brake + dt / DIVE_BRAKE_TIME);
        var T = 0.0;  var gdot = 0.0;
        if (pulling) {
            var n_t = math.min(n_cap, 0.85 * qS * CL_MAX / weight_N);
            gdot = math.min(g * (n_t - math.cos(gm)) / v, rate);
            gdot = math.min(gdot, 1.2 * (-gm));                       # level-off capture
            T = thrust_at(self, v, rho);
        } else {
            gdot = math.clamp(0.8 * (-m.theta - gm), -0.35, 0.35);
            gdot = math.max(gdot, g * (m.n_push - math.cos(gm)) / v);
        }
        var n  = v * gdot / g + math.cos(gm);
        var CL = n * weight_N / math.max(qS, 1.0);
        var D  = qS * (C_D0 + brake * (C_D0_BRAKE - C_D0) + K_IND * CL * CL);
        gm += gdot * dt;
        v   = math.max(v + ((T - D) / mass - g * math.sin(gm)) * dt, floor_v);
        z  += v * math.sin(gm) * dt;
        h  += v * math.sin(gm) * dt;
        x  += v * math.cos(gm) * dt;   #CLAUDE+ 0.7.1
        t  += dt;
        if (pulling and gm > -0.035) done = 1;
    }
    var r = { ok: 1, why: "", v_trig: v_trig, g_trig_deg: g_trig * R2D, loss: h_trig - h, base_agl: h, v_base: v, t: t,
              trig: pulling, x_trig: x_trig, t_trig: t_trig, z_trig: z_trig };   #CLAUDE~ 0.7.1: trig = 1 if the pull-out point was reached; x_trig (m) and t_trig (s) from now to it, z_trig its height MSL (m)
    if (!pulling) {
        r.ok = 0;  r.why = "never reaches the pull-out height";
    } else if (math.abs(g_trig + m.theta) > 5.0 * D2R) {
        r.ok = 0;  r.why = sprintf("dive angle not reached before the pull-out height (path %.0f deg of %.0f)", -g_trig * R2D, m.theta * R2D);
    } else if (h_trig - 1.2 * (h_trig - h) < m.clearance_ft * 0.3048) {
        r.ok = 0;  r.why = sprintf("base only %.0f ft AGL with a 20%% margin (need %.0f ft)", (h_trig - 1.2 * (h_trig - h)) / 0.3048, m.clearance_ft);
    }
    return r;
};

#CLAUDE+ 0.7.1: aiming a dive at a reference. The projectile is released where the pull-out starts, so a dive begun now lands the projectile
#CLAUDE+   X = (ground distance to the pull-out point) + (range of the projectile) along the ground track, T seconds from now (T = dive time + fall time).
#CLAUDE+ The reference is dead-reckoned T seconds ahead, which gives the along-track and cross-track error of a dive begun now (same signs as
#CLAUDE+ landing_miss: along > 0 means beyond the reference, cross > 0 means to the right of it). The approach flies level and starts the push-over
#CLAUDE+ when the along-track error reaches zero. Heading is held: the cross-track error is not corrected (no steering is planned).
var DIVE_AIM_DT      = 0.5;     # s   how often the approach re-predicts the dive (X and T change slowly; each prediction takes a few ms)
var DIVE_RELEASE_TOL = 100.0;   # m   default release_tol_m: the projectile is not released if the predicted miss is larger. A first guess, not a requirement
var DIVE_LOG_DT      = 5.0;     # s   spacing of the approach log lines
var DIVE_APPROACH_MAX = 900.0;  # s   an approach that has not reached the push-over point by then is cancelled

# Where a dive begun now would land: { ok, why, X (m along the ground track), T (s), pred, projectile }. p = predict_dive result if already known.
var dive_aim_prediction = func(self, m, p = nil) {
    if (p == nil) p = predict_dive(self, m);
    if (!p.trig) return { ok: 0, why: "never reaches the pull-out height" };
    if (!p.ok and !m.force) return { ok: 0, why: p.why };
    var vh = p.v_trig * math.cos(p.g_trig_deg * D2R);
    var vz = p.v_trig * math.sin(p.g_trig_deg * D2R);
    var rb = predict_projectile(0.0, 0.0, p.z_trig, vh, 0.0, vz, m.ground_ft * 0.3048, m.spec);
    return { ok: 1, why: "", X: p.x_trig + rb.x, T: p.t_trig + rb.t, pred: p, projectile: rb };
};

# Error of a dive begun now (ap from dive_aim_prediction, tl from reference_local, centred on the aircraft): { along, cross, closing }.
# closing (m/s) is how fast the along-track error grows: the aircraft ground speed less the reference speed along the track.
var dive_aim_error = func(self, ap, tl) {
    var trk = math.atan2(self.v_east, self.v_north);
    var s = math.sin(trk);  var c = math.cos(trk);
    var dx = ap.X * s - (tl.e + tl.vE * ap.T);
    var dy = ap.X * c - (tl.n + tl.vN * ap.T);
    var vh = math.sqrt(self.v_east * self.v_east + self.v_north * self.v_north);
    return { along: dx * s + dy * c, cross: dx * c - dy * s, closing: vh - (tl.vE * s + tl.vN * c) };
};

# Approach phase. Returns 1 = start the push-over now, 0 = keep flying, -1 = cancel the dive.
var dive_approach = func(self, m, dt) {
    var pos = own_position(self);
    var tl  = reference_local(m.reference, sim_time(), pos.lat, pos.lon);
    if (tl == nil) {
        print("Dive cancelled: reference lost");
        return -1;
    }
    if (m.t > DIVE_APPROACH_MAX) {
        print("Dive cancelled: approach took longer than " ~ DIVE_APPROACH_MAX ~ " s");
        return -1;
    }
    if (m.aim == nil or m.t - m.aim_t >= DIVE_AIM_DT) {
        var ap = dive_aim_prediction(self, m);
        if (!ap.ok) {
            print("Dive cancelled: " ~ ap.why);
            return -1;
        }
        m.aim = ap;  m.aim_t = m.t;
    }
    var e = dive_aim_error(self, m.aim, tl);
    if (e.closing < 1.0) {
        print(sprintf("Dive cancelled: not closing on the reference (closing speed %.1f m/s)", e.closing));
        return -1;
    }
    if (m.t - m.log_t >= DIVE_LOG_DT) {
        m.log_t = m.t;
        print(sprintf("Dive approach: range %.0f m, landing would be %+.0f m along, %+.0f m cross; push-over in about %.0f s",
                      math.sqrt(tl.e * tl.e + tl.n * tl.n), e.along, e.cross, math.max(-e.along, 0.0) / e.closing));
    }
    if (e.along + 0.5 * e.closing * dt < 0.0) return 0;      # half a tick of lead: the error is zero on average at the push-over
    m.aim_start = { range: math.sqrt(tl.e * tl.e + tl.n * tl.n), along: e.along, cross: e.cross, X: m.aim.X, T: m.aim.T };
    m.phase = "pushover";  m.t = 0.0;
    m.entry_speed = self.speed * kts_to_mps;                  # the zoom energy is measured from the speed the dive really starts at
    self.aerobatic = 1;
    print(sprintf("Dive: push-over at %.0f m from the reference, predicted landing %+.0f m along, %+.0f m cross, in %.1f s",
                  m.aim_start.range, e.along, e.cross, m.aim.T));
    return 1;
};

# Decision gate at the start of the pull-out: release if the predicted miss is within release_tol_m, otherwise pull out without releasing.
var dive_release = func(self, m) {
    var ms = landing_miss(self, m.reference, m.spec, m.ground_ft);
    if (ms == nil) {
        print("Dive: no release, reference lost");
        return;
    }
    m.release = { released: 0, miss: ms.miss, along: ms.along, cross: ms.cross, alt_ft: self.alt_ft, v: self.speed * kts_to_mps };
    if (ms.miss > m.release_tol_m) {
        print(sprintf("Dive: NO RELEASE, predicted miss %.0f m (along %+.0f, cross %+.0f) is above the %.0f m limit", ms.miss, ms.along, ms.cross, m.release_tol_m));
        return;
    }
    m.release.released = 1;
    print(sprintf("Dive: release, predicted miss %.1f m (along %+.1f, cross %+.1f)", ms.miss, ms.along, ms.cross));
    release_projectile(self, m.spec, m.reference, m.ground_ft);
};

var start_dive = func(self, opts = nil) {
    opts = opts or {};
    if (C_D0_BRAKE == nil) {
        print("Dive refused: this aircraft's profile has no terminal_speed_mps");
        return nil;
    }
    var m = {
        name: "dive", phase: "pushover", t: 0.0,
        theta: (opts["dive_angle_deg"] or 70.0) * D2R,
        ground_ft: opts["ground_ft"] or 0.0,
        h_pull_ft: opts["pullout_agl_ft"] or 2000.0,
        clearance_ft: opts["clearance_ft"] or 300.0,
        n_push: (opts["n_push"] != nil) ? opts["n_push"] : -1.0,   #CLAUDE~ 0.7.0: default -1 g (was 0)
        n_pull: opts["n_pull"] or 99.0,
        eta: opts["eta"] or 0.7,
        reference: nil, spec: nil, release_tol_m: 0.0, force: opts["force"] or 0,   #CLAUDE+ 0.7.1: reference fields (see below), release: result of dive_release
        aim: nil, aim_t: 0.0, log_t: 0.0, aim_start: nil, release: nil,
        entry_speed: self.speed * kts_to_mps,
        zoom_alt_ft: self.alt_ft, v_end: 0.0,
        update: update_dive_reference
    };
    #CLAUDE+ 0.7.1: optional reference { reference: hash or "/ai/models/ship[0]", projectile: spec (default FUEL_TANK_500), release_tol_m, reference_max_kt, stationary_kt, ground_ft }
    #CLAUDE+ The ground height defaults to the reference altitude. check_reference is the phase 0 gate: a reference above reference_max_kt is refused (force does not override it).
    if (opts["reference"] != nil) {
        var tgt = make_reference(opts["reference"]);
        var ck  = check_reference(tgt, opts);
        if (!ck.ok) {
            print("Dive refused: " ~ ck.why);
            return nil;
        }
        m.reference = tgt;
        m.spec   = opts["projectile"] or FUEL_TANK_500;
        m.release_tol_m = (opts["release_tol_m"] != nil) ? opts["release_tol_m"] : DIVE_RELEASE_TOL;
        if (opts["ground_ft"] == nil) m.ground_ft = ck.state.alt_ft;
        m.phase = "approach";
    }

    var p = predict_dive(self, m);
    print(sprintf("Dive prediction: pull-out at %.0f m/s on a %.0f deg path, loses %.0f m, base %.0f m AGL at %.0f m/s (%.1f s) %s",
                  p.v_trig, -p.g_trig_deg, p.loss, p.base_agl, p.v_base, p.t, p.ok ? "OK" : "REFUSED: " ~ p.why));
    if (!p.ok and !opts["force"]) return nil;

    #CLAUDE+ 0.7.1: where a dive begun now would land, and how far the aircraft still has to fly before the push-over
    if (m.reference != nil) {
        var ap = dive_aim_prediction(self, m, p);
        if (!ap.ok) {
            print("Dive refused: " ~ ap.why);
            return nil;
        }
        var pos = own_position(self);
        var tl  = reference_local(m.reference, sim_time(), pos.lat, pos.lon);
        var e   = dive_aim_error(self, ap, tl);
        print(sprintf("Dive aim: reference %.0f m away; a dive begun now would land %.0f m %s the reference and %.0f m to the %s of it; push-over in about %.0f s",
                      math.sqrt(tl.e * tl.e + tl.n * tl.n), math.abs(e.along), (e.along < 0) ? "short of" : "beyond", math.abs(e.cross),
                      (e.cross < 0) ? "left" : "right", math.max(-e.along, 0.0) / math.max(e.closing, 1.0)));
        if (!opts["force"] and e.closing < 1.0) {
            print("Dive refused: not closing on the reference");
            return nil;
        }
        if (!opts["force"] and e.along > m.release_tol_m) {
            print(sprintf("Dive refused: a dive begun now would land %.0f m beyond the reference (the reference is too close for this dive)", e.along));
            return nil;
        }
        if (!opts["force"] and math.abs(e.cross) > m.release_tol_m) {
            print(sprintf("Dive refused: the reference is %.0f m off the track (limit %.0f m): turn onto it first (there is no steering)", math.abs(e.cross), m.release_tol_m));
            return nil;
        }
    }
    if (opts["preview"]) return nil;   #CLAUDE+ 0.7.1: preview = 1 prints the predictions above and starts nothing

    self.aerobatic = (m.reference == nil) ? 1 : 0;   #CLAUDE~ 0.7.1: was 1; with a reference the approach flies normally and sets it at the push-over
    self.manoeuvre = m;
    return m;
};

var update_dive_reference = func(self, dt) {
    var m   = self.manoeuvre;
    var vs1 = v_stall_1g(self);
    var v   = math.max(self.speed * kts_to_mps, 0.5 * vs1);
    var v_h = math.sqrt(self.v_east * self.v_east + self.v_north * self.v_north);
    var gam = (self.gamma_deg or 0.0) * D2R;
    m.t += dt;
    update_roll_target(self, dt);

    #CLAUDE+ 0.7.1: approach (only with a reference): altitude and heading hold until dive_approach starts the push-over (this same tick)
    if (m.phase == "approach") {
        update_pitch_target(self, dt);
        if (self.auto_throttle) hold_speed(self, self.tgt_speed * kts_to_mps);
        var ra = dive_approach(self, m, dt);
        if (ra < 0) {
            self.manoeuvre = nil;
            self.aerobatic = 0;
            return;
        }
        if (ra == 0) return;
    }

    if (m.phase == "pushover" or m.phase == "dive") {
        self.brake_cmd = 1;
        self.throttle  = 0.0;
        if (self.alt_ft - m.ground_ft + 0.5 * (self.v_up or 0.0) * 3.28084 * dt <= m.h_pull_ft) {
    #CLAUDE~ 0.7.2: was alt <= h_pull, which is up to a tick late. With half a tick of lead (v_up is m/s and negative in a dive, 
    # altitude is in ft) the pull-out starts within half a tick either side of the crossing
            m.phase = "pullout";  m.t = 0.0;
            if (m.reference != nil) dive_release(self, m);   #CLAUDE+ 0.7.1: the decision gate, then the projectile leaves at the start of the pull-out
        } else {
            var gdot = math.clamp(0.8 * (-m.theta - gam), -0.35, 0.35);
            gdot = math.max(gdot, g * (m.n_push - math.cos(gam)) / v);        # no more than n_push g while pushing over
            command_path(self, v, gdot, 0, dt, 1);   #CLAUDE~ 0.7.1: was use_lead 0. tgt_pitch is built from last tick's path angle, so without the lead the pitch lags by gdot*dt
            if (m.phase == "pushover" and math.abs(gam + m.theta) < 3.0 * D2R) m.phase = "dive";
            return;
        }
    }

    if (m.phase == "pullout") {
        self.brake_cmd = 0;                                    # brakes in: the pull-out starts here
        self.throttle  = 1.0;
        var n_t  = math.min(m.n_pull, 0.9 * (self.perf["n_max_pos"] or 6.0));
        var gdot = math.min(g * (n_t - math.cos(gam)) / v, 1.2 * (-gam));      # at the load limit, captured smoothly at level
        command_path(self, v, gdot, 0, dt, 1);                 # with the rate lead: the path is rotating steadily
        if (gam > -2.0 * D2R or m.t > 20.0) {
            # level at the base: the excess speed over the pre-dive speed buys height
            var dE = math.max(0.0, m.eta * (v * v - m.entry_speed * m.entry_speed) / (2.0 * g));
            m.zoom_alt_ft = self.alt_ft + dE / 0.3048;
            m.v_end = math.max(1.15 * m.entry_speed, 1.3 * vs1);
            m.phase = "zoom";  m.t = 0.0;
        }
        return;
    }

    if (m.phase == "zoom") {
        self.throttle = 1.0;
        # climb no steeper than the speed can be spent: sin(gamma) = (T-D)/W - a_des/g, a_des < 0 while fast
        var excess = (thrust_available(self) - steady_drag(self, v)) / weight_N;
        var a_des  = math.clamp(CLIMB_K_SPEED * (m.v_end - v), -DIVE_ZOOM_A_MAX, DIVE_ZOOM_A_MAX);
        var s      = math.clamp(excess - a_des / g, -0.95, 0.95);
        var gcmd   = math.atan2(s, math.sqrt(1.0 - s * s));
        var err_m  = (m.zoom_alt_ft - self.alt_ft) / 3.28084;
        gcmd = math.min(gcmd, math.atan2(0.25 * err_m, math.max(v_h, 1.0)));   # flatten as the height is reached
        var gdot = math.clamp(CLIMB_K_GAMMA * (gcmd - gam), -0.15, 0.5);
        command_path(self, v, gdot, 0, dt, 0);
        if (err_m < 30.0 or v < m.v_end + 2.0 or m.t > 40.0) {
            self.tgt_altitude_ft = m.zoom_alt_ft;              # documented hand-back: cruise target set from the base energy
            self.manoeuvre = nil;
            self.aerobatic = 0;
            cruise(self, dt);
        }
        return;
    }
};

########################## climb mode (0.4.0) ###########################
#CLAUDE+ start_climb(self, opts): opts = { schedule: "max_rate" | "max_angle" | "economy" (default max_rate),
#CLAUDE+                                   alt_ft: target altitude (default tgt_altitude_ft) }
#CLAUDE+ Sets tgt_altitude_ft. If cruise can handle the altitude change (the path angle it needs is at most CLIMB_CRUISE_FRACTION of
#CLAUDE+ the angle the excess thrust can sustain at the current speed) nothing is started and cruise does the climb. Otherwise the
#CLAUDE+ climb runs at full throttle on the schedule speed until the altitude error is small, then hands over to cruise, which
#CLAUDE+ captures the altitude and restores the pre-manoeuvre throttle mode.
var CLIMB_SCHEDULES       = { max_rate: "rate", max_angle: "angle", economy: "economy" };
var CLIMB_CRUISE_FRACTION = 0.5;
var CLIMB_CAPTURE_MIN_M   = 20.0;

var cruise_can_climb = func(self, dh_m) {
    var vs1 = v_stall_1g(self);
    var v   = math.max(self.speed * kts_to_mps, 0.5 * vs1);
    var v_h = math.sqrt(self.v_east * self.v_east + self.v_north * self.v_north);
    var vs  = math.clamp(dh_m * 0.1, 0.0, self.perf.climb_rate * 0.00508);
    var g_req = math.atan2(vs, math.max(v_h, 1.0));
    var s = math.clamp((thrust_available(self) - steady_drag(self, v)) / weight_N, -0.95, 0.95);
    var g_av = math.atan2(s, math.sqrt(1.0 - s * s));
    return (v >= 1.2 * vs1 and g_av > 0.0 and g_req <= CLIMB_CRUISE_FRACTION * g_av);
};

var start_climb = func(self, opts = nil) {
    opts = opts or {};
    var sched = opts["schedule"] or "max_rate";
    var mode  = CLIMB_SCHEDULES[sched];
    if (mode == nil) {
        print("Unknown climb schedule: " ~ sched ~ " (use max_rate, max_angle or economy), using max_rate");   #CLAUDE~ 0.5.0: was: refuse
        sched = "max_rate";  mode = "rate";
    }
    if (opts["alt_ft"] != nil) self.tgt_altitude_ft = opts["alt_ft"];
    var dh = (self.tgt_altitude_ft - self.alt_ft) / 3.28084;
    if (dh <= CLIMB_CAPTURE_MIN_M or cruise_can_climb(self, dh)) {
        if (dh > CLIMB_CAPTURE_MIN_M) print(sprintf("climb: %.0f m to go, cruise handles it", dh));   #CLAUDE~ 0.5.0: quiet for descents and small changes
        return nil;
    }
    self.manoeuvre = { name: "climb", phase: "climb", t: 0.0, schedule: sched, mode: mode, update: update_climb_target };
    return self.manoeuvre;
};

var update_climb_target = func(self, dt) {
    var m = self.manoeuvre;
    m.t += dt;
    var alt_err_m = (self.tgt_altitude_ft - self.alt_ft) / 3.28084;
    if (alt_err_m < math.max(CLIMB_CAPTURE_MIN_M, 10.0 * self.v_up) or m.t > 3600.0) {
        self.manoeuvre = nil;
        cruise(self, dt);          # hand over now: altitude capture, throttle restored
        return;
    }
    update_roll_target(self, dt);
    self.throttle = 1.0;
    update_pitch_target(self, dt, climb_speed(self, m.mode));
};

#CLAUDE~ 0.3.0: new. The base mode, run whenever no manoeuvre is active: heading hold, altitude hold and, if auto_throttle is on,
#CLAUDE+ speed hold. On the first tick after a manoeuvre it hands back the throttle: auto mode carries on holding tgt_speed;
#CLAUDE+ in manual mode the throttle the pilot had before the manoeuvre is restored (a manoeuvre can leave it at 0, e.g. after a dive).
var cruise = func(self, dt) {
    var pre = self.pre_manoeuvre;
    if (pre != nil) {
        if (!pre["auto"]) self.throttle = pre["thr"];
        self.pre_manoeuvre = nil;
    }
    self.brake_cmd = 0;   #CLAUDE+ 0.6.0: brakes are in whenever no manoeuvre is running
    update_roll_target(self, dt);
    update_pitch_target(self, dt);
    if (self.auto_throttle) hold_speed(self, self.tgt_speed * kts_to_mps);
};

########################## approach (0.7.3) ###########################
#CLAUDE+ Approach: guidance above cruise(). Guidance only produces a heading, an altitude and a speed; the existing laws fly them
#CLAUDE+ (update_roll_target, update_pitch_target, hold_speed), so the user targets tgt_heading and tgt_altitude_ft are not touched until the
#CLAUDE+ documented hand-back (the same rule as every other manoeuvre). Nothing here is specific to one aircraft or one use: the line, the
#CLAUDE+ funnel and the gate are plain geometry, and what happens at the gate is another manoeuvre (then).
#CLAUDE+
#CLAUDE+ The line: a reference (hash { lat, lon, alt_ft, hdg, spd_kt } or an AI property path such as "/ai/models/carrier[0]": fixed if its
#CLAUDE+ speed is 0, otherwise dead-reckoned), an inbound course, a glide angle and a gate. Everything is worked in the frame of the
#CLAUDE+ reference, so a moving reference only changes the closing speed and adds a crab angle.
#CLAUDE+   x   distance along the course from the reference (negative before it); the gate is at x = -gate_m
#CLAUDE+   xt  cross-track error, positive to the right of the course;  dtg = distance to the gate along the course
#CLAUDE+
#CLAUDE+ Phases: 0 check (start_approach: is the line reachable in time?) -> 1 intercept (turn onto the line, capture the height) ->
#CLAUDE+         2 track (inside the funnel, which narrows towards the gate) -> gate (entry conditions checked) -> hand-over to then, or cruise.
#CLAUDE+ Abort (reference lost, not closing, no longer reachable, left the funnel, too slow, gate conditions not met, timeout): on_abort =
#CLAUDE+ "goaround" (default: full power, climb to the safe height on the course, 
#CLAUDE+ or on the present heading if that is more than NAV_GA_COURSE_DEG from the course, 
#CLAUDE+ then cruise), "cruise" (hand back now) or a function(self, m, reason)
#CLAUDE+ The on_abort function is called with no manoeuvre running, so it can start one with start_manoeuvre. If it starts none, the aircraft is handed back to cruise.
#CLAUDE+ Leaving the funnel is not an abort in itself. A line that has moved from under the aircraft, for example when the reference turns, is flown again from the
#CLAUDE+ intercept phase for as long as the phase 0 check on the present state says that the gate can still be reached with a margin, up to NAV_MAX_REINT times.
#CLAUDE+ The heading of a turning reference is smoothed before it turns the line, so that the far end of the line does not sweep sideways faster than the aircraft can follow.
#CLAUDE+ What follows an abort (reposition, a second attempt) is left to later versions: m.attempt and m.max_attempts are already there.
#CLAUDE+ The result is left in state.last_approach { outcome ("gate" | "abort"), reason, phase, t, dtg, xt, dh_ft, dv, dpsi, gamma, roll, attempt }.
#CLAUDE+
#CLAUDE+ start_approach(self, opts). opts:
#CLAUDE+   ref (required), course_deg (the direction of flight along the line, default: the bearing to the reference, i.e. straight in), course_rel,
#CLAUDE+   course_rel = 1 measures course_deg from the heading of the reference: 0 is the same direction as the reference, a landing from astern, and
#CLAUDE+   180 is the opposite direction, a head-on pass from ahead of it. The gate lies gate_m before the reference along the course, so the aircraft
#CLAUDE+   must start on the side of the reference that the course comes from. Phase 0 says when the course looks reversed.
#CLAUDE+   glide_deg (0 = level; positive: the path descends towards the reference), gate_m (distance before the reference where the approach ends, 0),
#CLAUDE+   gate_dh_ft (gate height above the reference; default: the glide path height, or for a level line the present height above it) or gate_alt_ft (MSL),
#CLAUDE+   speed_mps / speed_kt (default perf approach_speed_mps, else the present speed; never below 1.25 x the 1 g stall speed),
#CLAUDE+   then (name of the manoeuvre to start at the gate) with then_opts, on_abort, ga_margin_ft (default perf goaround_margin_ft, else 1000),
#CLAUDE+   gate_xt_m, gate_dh_tol_ft, gate_hdg_deg (override the gate tolerances), force (1 = skip the phase 0 check), preview (1 = print it and stop).
#CLAUDE+ Gate tolerances: the entry conditions of the successor if it has any (NAV_ENTRY), otherwise the NAV_GATE_* defaults.
var NAV_LOOKAHEAD_S  = 8.0;     # s    intercept look-ahead: the track angle is atan(cross-track / (speed x this)), a damped (zeta about 0.8) capture
var NAV_MAX_INTERCEPT = 45.0;   # deg  steepest intercept angle to the line
var NAV_SETTLE_S     = 10.0;    # s    time the aircraft needs on the line before the gate (also the margin in the phase 0 check)
var NAV_FUNNEL_DEG   = 3.0;     # deg  half-angle of the cross-track funnel (width = gate width + distance to the gate x tan of this)
var NAV_FUNNEL_V_DEG = 1.5;     # deg  the same for height
var NAV_GATE_XT_M    = 25.0;    # m    gate: largest cross-track error
var NAV_GATE_DH_FT   = 80.0;    # ft   gate: largest height error
var NAV_GATE_HDG_DEG = 5.0;     # deg  gate: largest heading error against the commanded heading
var NAV_GATE_V_FRAC  = 0.10;    #      gate: speed within this fraction of the approach speed
var NAV_GATE_GAMMA_DEG = 3.0;   # deg  gate: path angle within this of the glide path
var NAV_GATE_ROLL_DEG  = 8.0;   # deg  gate: largest roll angle
var NAV_OUT_S        = 1.0;     # s    outside the funnel for this long aborts
var NAV_MAX_REINT    = 5;       #      how many times the aircraft may drop back to the intercept phase
var NAV_REINT_MARGIN = 1.0;     #      a re-intercept needs the time required to be below this fraction of the time to the gate, which already includes the settle time
var NAV_REF_HDG_TAU_S = 6.0;    # s    smoothing of a turning reference heading, shortened to a quarter of the time to the gate
var NAV_REF_VS_TAU_S = 3.0;     # s    smoothing of the estimated climb rate of the reference
var NAV_GROSS        = 4.0;     #      a cross-track or height error beyond this many funnel widths aborts at once
var NAV_GA_COURSE_DEG = 20.0;   # deg  go-around: fly the course only if the heading is within this of it, otherwise hold the present heading
var NAV_TIMEOUT_S    = 900.0;   # s    an approach not at the gate by then aborts
var NAV_LOG_DT       = 5.0;     # s    spacing of the log lines
var NAV_GA_MARGIN_FT = 1000.0;  # ft   go-around: climb this far above the abort height (or the gate height)
var NAV_GA_TIMEOUT_S = 300.0;   # s    go-around: hand back to cruise after this whatever the height

var nav_tan = func(deg) { var r = deg * D2R; return math.sin(r) / math.cos(r); };

# The entry conditions a successor needs at the gate: name -> func(self, opts, spec) that adjusts spec (see nav_entry_spec for its keys).
var NAV_ENTRY = {
    dive: func(self, opts, spec) { spec.v_min = 1.3 * v_stall_1g(self); },   # the push-over needs speed in hand
};

# Gate tolerances for a successor: { xt_m, dh_ft, hdg_deg, gamma_deg, roll_deg, v_min, v_max } (v_min and v_max nil = no requirement of its own).
var nav_entry_spec = func(self, name, opts) {
    var spec = { xt_m: NAV_GATE_XT_M, dh_ft: NAV_GATE_DH_FT, hdg_deg: NAV_GATE_HDG_DEG, gamma_deg: NAV_GATE_GAMMA_DEG,
                 roll_deg: NAV_GATE_ROLL_DEG, v_min: nil, v_max: nil };
    var f = (name == nil) ? nil : NAV_ENTRY[name];
    if (f != nil) f(self, opts, spec);
    return spec;
};

# Highest turn rate (rad/s) the heading law can command at speed v: its rate limit or the bank limit, whichever is lower.
var turn_rate_max = func(self, v) {
    var rate = (self.perf["max_turn_rate_dps"] or 3.0) * D2R;
    var bank = g * nav_tan(math.min(self.perf.max_bank_angle, 89.0)) / math.max(v, 1.0);
    return math.min(rate, bank);
};

# Fly the commanded heading and altitude with the existing laws without overwriting the user targets.
var nav_steer = func(self, hdg, alt_ft, dt, v_ref = nil) {
    var h0 = self.tgt_heading;  var a0 = self.tgt_altitude_ft;
    self.tgt_heading = hdg;  self.tgt_altitude_ft = alt_ft;
    update_roll_target(self, dt);
    update_pitch_target(self, dt, v_ref);
    self.tgt_heading = h0;  self.tgt_altitude_ft = a0;
};

# The line at this instant, from the aircraft's position and the reference's present state (closed form, a few microseconds):
# { x, xt, dtg, xdot (closing speed along the course), xdot_cmd (the same on the commanded heading), xtdot, course, us, uc, gate_alt_ft, path_alt_ft, psi_deg, alt_cmd_ft, xt_tol, v_tol_ft, ref_alt_ft, w_al, tl }
var nav_geometry = func(self, m, dt = 0.0) {
    var pos = own_position(self);
    var tl  = reference_local(m.ref, sim_time(), pos.lat, pos.lon);
    if (tl == nil) return nil;
    var rh = tl.hdg;
    if (m.course_rel) {                                 #CLAUDE+ 0.7.3.4: smoothed heading of the reference
        if (m.ref_hdg_f == nil) m.ref_hdg_f = tl.hdg;
        else if (dt > 0.0) {
            var tau = NAV_REF_HDG_TAU_S;
            var lg = m.last_geo;
            if (lg != nil and lg.xdot > 1.0) tau = math.min(tau, 0.25 * math.max(lg.dtg, 0.0) / lg.xdot);
            m.ref_hdg_f = wrap360(m.ref_hdg_f + wrap180(tl.hdg - m.ref_hdg_f) * dt / (tau + dt));
        }
        rh = m.ref_hdg_f;
    }
    if (dt > 0.0 and m.ref_alt_prev != nil) {           #CLAUDE+ 0.7.3.4: climb rate of the reference in ft/s
        var vs = math.clamp((tl.alt_ft - m.ref_alt_prev) / dt, -60.0, 60.0);
        m.ref_vs += (vs - m.ref_vs) * dt / (NAV_REF_VS_TAU_S + dt);
    }
    m.ref_alt_prev = tl.alt_ft;
    var course = wrap360((m.course_rel ? rh : 0.0) + m.course_deg);
    var us = math.sin(course * D2R);  var uc = math.cos(course * D2R);
    var px = -tl.e;  var py = -tl.n;                    # the aircraft relative to the reference
    var x  = px * us + py * uc;
    var xt = px * uc - py * us;
    var rvx = self.v_east - tl.vE;  var rvy = self.v_north - tl.vN;   # velocity relative to the reference
    var xdot  = rvx * us + rvy * uc;
    var xtdot = rvx * uc - rvy * us;
    var dtg = -m.gate_m - x;
    var gate_alt = (m.gate_alt_ft != nil) ? m.gate_alt_ft : tl.alt_ft + m.gate_dh_ft;
    var tg = nav_tan(m.glide);
    var dpos = math.max(dtg, 0.0);
    var path_alt = gate_alt + dpos * tg * 3.28084;
    var xt_tol = m.gate.xt_m + dpos * nav_tan(NAV_FUNNEL_DEG);
    var v_tol  = m.gate.dh_ft + dpos * nav_tan(NAV_FUNNEL_V_DEG) * 3.28084;
    # heading: track angle from the cross-track error (clamped), then the crab that cancels the reference's own motion
    var vh  = math.max(math.sqrt(self.v_east * self.v_east + self.v_north * self.v_north), 10.0);
    var ang = math.clamp(math.atan2(xt, vh * NAV_LOOKAHEAD_S), -NAV_MAX_INTERCEPT * D2R, NAV_MAX_INTERCEPT * D2R);
    var theta = course * D2R - ang;
    var sc = math.clamp((tl.vE * math.cos(theta) - tl.vN * math.sin(theta)) / vh, -0.9, 0.9);
    var psi = theta + math.atan2(sc, math.sqrt(1.0 - sc * sc));
    # height: the path height plus 10 s of its rate, which is what update_pitch_target's 0.1/s law needs for no steady lag
    var alt_cmd = path_alt + 32.8084 * (-xdot * tg) + 10.0 * m.ref_vs;     #CLAUDE~ 0.7.3.4: plus 10 s of the climb rate of the reference, the same lag removal
    if (m.glide > 0.0 and self.alt_ft < path_alt - v_tol) alt_cmd = self.alt_ft;   # below the glide path: hold until it comes down to us
    var xdot_cmd = vh * math.cos(psi - course * D2R) - (tl.vE * us + tl.vN * uc);      # closing speed once the aircraft is on the commanded heading
    return { x: x, xt: xt, dtg: dtg, xdot: xdot, xdot_cmd: xdot_cmd, xtdot: xtdot, course: course, us: us, uc: uc, gate_alt_ft: gate_alt, path_alt_ft: path_alt,
             psi_deg: wrap360(psi * R2D), alt_cmd_ft: alt_cmd, xt_tol: xt_tol, v_tol_ft: v_tol, ref_alt_ft: tl.alt_ft,
             w_al: tl.vE * us + tl.vN * uc, tl: tl };
};

# Phase 0: can the line be reached in time? Returns { ok, why, t_req, t_avail }. All times from the aircraft profile.
var nav_check = func(self, m, nl) {
    var v = math.max(m.v_app, 1.0);
    if (nl.dtg < 0.0) {
        var why = sprintf("the aircraft is already %.0f m past the gate", -nl.dtg);
        if (-nl.dtg > 2.0 * m.gate_m)       #CLAUDE+ 0.7.3.2: reversing the course would put the aircraft before the gate
            why = why ~ sprintf("; the course may be reversed, try course_deg %.0f", wrap360(m.course_deg + 180.0));
        return { ok: 0, why: why, t_req: 0.0, t_avail: 0.0 };
    }
    var vn = math.sqrt(self.v_east * self.v_east + self.v_north * self.v_north);   #CLAUDE~ 0.7.3.1: closing speed from the speed flown now if that is lower than the approach speed (it was the approach speed: a slow aircraft was accepted, then aborted "not closing" at once)
    var closure = math.min(v, vn) - nl.w_al;
    if (closure < 5.0) return { ok: 0, why: sprintf("cannot close on the reference at the present speed (closing speed %.1f m/s at %.0f m/s; speed up first)", closure, math.min(v, vn)), t_req: 0.0, t_avail: 0.0 };
    var om  = turn_rate_max(self, v);
    var th  = math.clamp(math.atan2(math.abs(nl.xt), v * NAV_LOOKAHEAD_S), 10.0 * D2R, NAV_MAX_INTERCEPT * D2R);
    var dps = math.abs(wrap180(nl.psi_deg - self.hdg)) * D2R;
    var t_lat = (dps + th) / om + math.abs(nl.xt) / (v * math.sin(th)) + NAV_SETTLE_S;      # turn onto the intercept, fly it, turn back, settle
    var dh = math.abs(nl.path_alt_ft - self.alt_ft) / 3.28084;
    var tg = nav_tan(m.glide);
    var t_vert = NAV_SETTLE_S;
    if (m.glide > 0.0 and self.alt_ft < nl.path_alt_ft) t_vert += dh / math.max(closure * tg, 0.1);        # level until the glide path comes down
    else t_vert += dh / math.max(0.5 * math.min(self.perf.climb_rate, self.perf.descent_rate) * 0.00508, 0.5);
    var t_req = math.max(t_lat, t_vert);
    var t_av  = nl.dtg / closure;
    if (t_req > t_av)
        return { ok: 0, why: sprintf("the line cannot be reached in time: needs %.0f s (turn and intercept %.0f s, height %.0f s), %.0f s to the gate", t_req, t_lat, t_vert, t_av), t_req: t_req, t_avail: t_av };
    return { ok: 1, why: "", t_req: t_req, t_avail: t_av };
};

var start_approach = func(self, opts = nil) {
    opts = opts or {};
    var ref = make_reference(opts["ref"]);
    if (ref == nil) {
        print("Approach refused: needs a reference (opts.ref)");
        return nil;
    }
    var pos = own_position(self);
    var tl0 = reference_local(ref, sim_time(), pos.lat, pos.lon);
    if (tl0 == nil) {
        print("Approach refused: the reference is missing or not valid");
        return nil;
    }
    var vs1 = v_stall_1g(self);
    var v_app = opts["speed_mps"];
    if (v_app == nil and opts["speed_kt"] != nil) v_app = opts["speed_kt"] * kts_to_mps;
    if (v_app == nil) v_app = self.perf["approach_speed_mps"];
    if (v_app == nil) v_app = self.speed * kts_to_mps;
    v_app = math.max(v_app, 1.25 * vs1);
    var rel = opts["course_rel"] or 0;
    var m = {
        name: "approach", phase: "intercept", t: 0.0,
        ref: ref, course_rel: rel,
        course_deg: (opts["course_deg"] != nil) ? opts["course_deg"] : (rel ? 0.0 : wrap360(math.atan2(tl0.e, tl0.n) * R2D)),
        glide: opts["glide_deg"] or 0.0,
        gate_m: opts["gate_m"] or 0.0,
        gate_alt_ft: opts["gate_alt_ft"], gate_dh_ft: 0.0,
        v_app: v_app, then: opts["then"], then_opts: opts["then_opts"],
        on_abort: opts["on_abort"] or "goaround",
        ga_margin_ft: opts["ga_margin_ft"] or self.perf["goaround_margin_ft"] or NAV_GA_MARGIN_FT,
        attempt: 1, max_attempts: 1,
        out_t: 0.0, log_t: 0.0, last_geo: nil, gate: nil, reint: 0, ref_hdg_f: nil, ref_alt_prev: nil, ref_vs: 0.0,
        ga_alt_ft: 0.0, ga_hdg: 0.0,
        update: update_approach_reference
    };
    if (m.gate_alt_ft == nil) {
        m.gate_dh_ft = (opts["gate_dh_ft"] != nil) ? opts["gate_dh_ft"]
                     : ((m.glide > 0.0) ? m.gate_m * nav_tan(m.glide) * 3.28084 : self.alt_ft - tl0.alt_ft);
    }
    var gt = nav_entry_spec(self, m.then, m.then_opts);
    if (opts["gate_xt_m"] != nil)     gt.xt_m = opts["gate_xt_m"];
    if (opts["gate_dh_tol_ft"] != nil) gt.dh_ft = opts["gate_dh_tol_ft"];
    if (opts["gate_hdg_deg"] != nil)  gt.hdg_deg = opts["gate_hdg_deg"];
    gt.v_min = math.max((1.0 - NAV_GATE_V_FRAC) * v_app, gt.v_min or 0.0);
    gt.v_max = math.min((1.0 + NAV_GATE_V_FRAC) * v_app, gt.v_max or 1.0e9);
    m.gate = gt;

    var nl = nav_geometry(self, m);
    var ck  = nav_check(self, m, nl);
    print(sprintf("Approach phase 0: gate %.0f m from the reference on course %.0f, %.0f m to go, cross-track %+.0f m, height error %+.0f ft, speed %.0f m/s; %s",
                  m.gate_m, nl.course, nl.dtg, nl.xt, self.alt_ft - nl.path_alt_ft, m.v_app,
                  ck.ok ? sprintf("reachable (needs %.0f s, %.0f s available)", ck.t_req, ck.t_avail) : "REFUSED: " ~ ck.why));
    if (!ck.ok and !opts["force"]) {
        self.last_approach = { outcome: "refused", reason: ck.why, phase: "check", t: 0.0, dtg: nl.dtg, xt: nl.xt, dh_ft: self.alt_ft - nl.path_alt_ft,
                               dv: 0.0, dpsi: 0.0, gamma: self.gamma_deg or 0.0, roll: self.roll, attempt: 1 };
        return nil;
    }
    if (opts["preview"]) return nil;
    self.last_approach = nil;
    self.manoeuvre = m;
    return m;
};

# Record the errors at the gate or at the abort in state.last_approach.
var nav_record = func(self, m, nl, outcome, reason) {
    var r = { outcome: outcome, reason: reason, phase: m.phase, t: m.t, attempt: m.attempt, reint: m.reint,
              dtg: 0.0, xt: 0.0, dh_ft: 0.0, dv: self.speed * kts_to_mps - m.v_app, dpsi: 0.0,
              gamma: self.gamma_deg or 0.0, roll: self.roll };
    if (nl != nil) {
        r.dtg = nl.dtg;  r.xt = nl.xt;  r.dh_ft = self.alt_ft - nl.path_alt_ft;  r.dpsi = wrap180(self.hdg - nl.psi_deg);
    }
    self.last_approach = r;
    return r;
};

# Hand the aircraft back to cruise with a documented heading and altitude.
var nav_handback = func(self, hdg, alt_ft, dt) {
    self.tgt_heading = hdg;
    self.tgt_altitude_ft = alt_ft;
    self.manoeuvre = nil;
    cruise(self, dt);
};

var nav_abort = func(self, m, nl, dt, reason) {
    nav_record(self, m, nl, "abort", reason);
    print(sprintf("Approach ABORT in phase %s after %.1f s: %s", m.phase, m.t, reason));
    var g2 = (nl != nil) ? nl : m.last_geo;
    m.ga_hdg    = self.hdg;                    #CLAUDE~ 0.7.3.3: the course only when the heading is already close to it
    if (g2 != nil and math.abs(wrap180(g2.course - self.hdg)) < NAV_GA_COURSE_DEG) m.ga_hdg = g2.course;
    m.ga_alt_ft = math.max(self.alt_ft, (g2 != nil) ? g2.gate_alt_ft : self.alt_ft) + m.ga_margin_ft;
    var oa = m.on_abort;
    if (typeof(oa) == "func") {
        self.manoeuvre = nil;                              #CLAUDE~ 0.7.3.1: cleared first, so the handler can start a manoeuvre (start_manoeuvre refuses while one is running)
        oa(self, m, reason);
        if (self.manoeuvre != nil) return;                 # the handler started something else
        nav_handback(self, self.hdg, self.alt_ft, dt);
        return;
    }
    if (oa == "cruise") {
        nav_handback(self, m.ga_hdg, self.alt_ft, dt);
        return;
    }
    m.phase = "goaround";  m.t = 0.0;
    print(sprintf("Approach go-around: full power, climb to %.0f ft on course %.0f", m.ga_alt_ft, m.ga_hdg));
    nav_goaround(self, m, dt);
};

var nav_goaround = func(self, m, dt) {
    self.throttle = 1.0;
    nav_steer(self, m.ga_hdg, m.ga_alt_ft, dt, climb_speed(self, "rate"));
    var err_m = (m.ga_alt_ft - self.alt_ft) / 3.28084;
    if (err_m < math.max(CLIMB_CAPTURE_MIN_M, 10.0 * self.v_up) or m.t > NAV_GA_TIMEOUT_S) {
        print(sprintf("Approach go-around complete at %.0f ft: back to cruise", self.alt_ft));
        nav_handback(self, m.ga_hdg, m.ga_alt_ft, dt);
    }
};

# Gate: compare the aircraft with the entry conditions. Returns "" when they are met, otherwise the reason.
var nav_gate_check = func(self, m, nl) {
    var gt = m.gate;
    var v  = self.speed * kts_to_mps;
    var dh = self.alt_ft - nl.path_alt_ft;
    if (math.abs(nl.xt) > gt.xt_m) return sprintf("gate: cross-track %+.0f m (limit %.0f)", nl.xt, gt.xt_m);
    if (math.abs(dh) > gt.dh_ft) return sprintf("gate: height %+.0f ft (limit %.0f)", dh, gt.dh_ft);
    if (v < gt.v_min or v > gt.v_max) return sprintf("gate: speed %.0f m/s (allowed %.0f to %.0f)", v, gt.v_min, gt.v_max);
    var dp = wrap180(self.hdg - nl.psi_deg);
    if (math.abs(dp) > gt.hdg_deg) return sprintf("gate: heading %+.1f deg off (limit %.1f)", dp, gt.hdg_deg);
    var ga = (self.gamma_deg or 0.0) + m.glide - math.atan2(m.ref_vs * 0.3048, math.max(v, 1.0)) * R2D;     #CLAUDE~ 0.7.3.4: the path angle expected over the ground includes the climb of the reference
    if (math.abs(ga) > gt.gamma_deg) return sprintf("gate: path angle %+.1f deg off the path (limit %.1f)", ga, gt.gamma_deg);
    if (math.abs(self.roll) > gt.roll_deg) return sprintf("gate: roll %.1f deg (limit %.1f)", self.roll, gt.roll_deg);
    return "";
};

var update_approach_reference = func(self, dt) {
    var m = self.manoeuvre;
    m.t += dt;
    if (m.phase == "goaround") {
        nav_goaround(self, m, dt);
        return;
    }
    var nl = nav_geometry(self, m, dt);
    if (nl == nil) {
        nav_abort(self, m, nil, dt, "the reference was lost");
        return;
    }
    m.last_geo = nl;
    var v  = self.speed * kts_to_mps;
    var dh = self.alt_ft - nl.path_alt_ft;
    nav_steer(self, nl.psi_deg, nl.alt_cmd_ft, dt);
    hold_speed(self, m.v_app);

    if (m.t - m.log_t >= NAV_LOG_DT) {
        m.log_t = m.t;
        print(sprintf("Approach %s: %.0f m to the gate, cross-track %+.0f m (funnel %.0f), height %+.0f ft (funnel %.0f), %.0f m/s",
                      m.phase, nl.dtg, nl.xt, nl.xt_tol, dh, nl.v_tol_ft, v));
    }
    if (v < 1.1 * v_stall_1g(self)) { nav_abort(self, m, nl, dt, sprintf("too slow (%.0f m/s)", v)); return; }
    if (m.t > NAV_TIMEOUT_S)        { nav_abort(self, m, nl, dt, "timeout"); return; }
    if (nl.xdot_cmd < 1.0)         { nav_abort(self, m, nl, dt, sprintf("not closing (%.1f m/s even on the commanded heading)", nl.xdot_cmd)); return; }

    # the gate: crossed within half a tick (the same lead as the dive's pull-out start)
    if (nl.dtg - 0.5 * nl.xdot * dt <= 0.0) {
        var why = nav_gate_check(self, m, nl);
        if (why != "") { nav_abort(self, m, nl, dt, why); return; }
        var r = nav_record(self, m, nl, "gate", "");
        print(sprintf("Approach: at the gate after %.1f s: cross-track %+.1f m, height %+.1f ft, speed %+.1f m/s, heading %+.1f deg%s",
                      m.t, r.xt, r.dh_ft, r.dv, r.dpsi, (m.then != nil) ? ", starting " ~ m.then : ""));
        var pre = self.pre_manoeuvre;
        nav_handback(self, nl.psi_deg, nl.path_alt_ft, dt);          # documented hand-back values, whether or not a successor follows
        if (m.then != nil) {
            var ok = start_manoeuvre(modelName, self, m.then, m.then_opts);
            if (ok == nil) {
                print("Approach: the successor " ~ m.then ~ " did not start (refused, or cruise can already do it); flying on in cruise");   #CLAUDE~ 0.7.3.1: wording (start_climb returns nil when cruise handles the climb)
                self.last_approach.then_started = 0;
            } else {
                self.last_approach.then_started = 1;
            }
            if (pre != nil) self.pre_manoeuvre = pre;                   # the throttle mode from before the approach, not the approach's
        }
        return;
    }

    var xo = math.abs(nl.xt) > nl.xt_tol or math.abs(dh) > nl.v_tol_ft;
    if (m.phase == "intercept") {
        var inside = math.abs(nl.xt) < 0.8 * nl.xt_tol and math.abs(dh) < 0.8 * nl.v_tol_ft and math.abs(wrap180(self.hdg - nl.psi_deg)) < 10.0;
        if (inside) {
            m.phase = "track";  m.out_t = 0.0;
            print(sprintf("Approach established (phase 2): %.0f m to the gate, cross-track %+.0f m, height %+.0f ft", nl.dtg, nl.xt, dh));
        } else if (nl.dtg < nl.xdot_cmd * NAV_SETTLE_S) {
            nav_abort(self, m, nl, dt, sprintf("not established %.0f m from the gate (cross-track %+.0f m, height %+.0f ft)", nl.dtg, nl.xt, dh));
        }
        return;
    }
    if (xo) m.out_t += dt; else m.out_t = 0.0;
    var gross = math.abs(nl.xt) > NAV_GROSS * nl.xt_tol or math.abs(dh) > NAV_GROSS * nl.v_tol_ft;
    if (gross or m.out_t > NAV_OUT_S) {                 #CLAUDE~ 0.7.3.4: re-intercept when the gate can still be reached, abort when it cannot
        var ck = nav_check(self, m, nl);
        if (ck.ok and ck.t_req < NAV_REINT_MARGIN * ck.t_avail and m.reint < NAV_MAX_REINT) {
            m.reint += 1;  m.phase = "intercept";  m.out_t = 0.0;
            print(sprintf("Approach left the funnel (cross-track %+.0f m of %.0f, height %+.0f ft of %.0f): intercepting again, %d of %d, needs %.0f s, %.0f s to the gate",
                          nl.xt, nl.xt_tol, dh, nl.v_tol_ft, m.reint, NAV_MAX_REINT, ck.t_req, ck.t_avail));
            return;
        }
        nav_abort(self, m, nl, dt, sprintf("%s (cross-track %+.0f m of %.0f, height %+.0f ft of %.0f), %s",
                  gross ? "far outside the funnel" : "left the funnel", nl.xt, nl.xt_tol, dh, nl.v_tol_ft,
                  ck.ok ? "intercepted again too often or too little time in hand" : "cannot recover: " ~ ck.why));
    }
};

var run_flight_physics = func(mName) {
    var prof = fS.prof;
    var state = fS.state;
    var now   = state.tNode.getValue() or 0.0;
    var last  = state.time or now;                  # first call: dt = 0
    var dt_real = now - last;
    state.time  = now;
    state.dTime = dt_real;                          # for the monitor
    if (dt_real <= 0.0) return;                     # sim paused, or first call
    dt_real = math.min(dt_real, MAX_DT);

    var t0 = prof.enable ? systime() : 0.0;

    var n = math.ceil(dt_real / MAX_SUBSTEP);
    var h = dt_real / n;
    var t_ctl = 0.0;
    var t_frc = 0.0;
    for (var i = 0; i < n; i += 1) {
        var a0 = prof.enable ? systime() : 0.0;
        if (state.manoeuvre != nil) {
            state.manoeuvre.update(state, h);
        } else {
            cruise(state, h);   #CLAUDE~ 0.3.0: was update_roll_target(state, h); update_pitch_target(state, h);
        }

        # 2. Orientation, 3. forces
        update_orientation(state, h);
        var a1 = prof.enable ? systime() : 0.0;
        update_force_physics(state, h);
        if (prof.enable) {
            t_ctl += a1 - a0;
            t_frc += systime() - a1;
        }
    }


    if (size(state.projectiles) > 0) update_projectiles(state, dt_real);   #CLAUDE+ 0.7.0
    if (series.running) series_step(state, dt_real);   #CLAUDE+ 0.7.2: series runner (test code, see the end of the file); one flag test per tick when idle

    var throttleFactor = (contains(state.perf, "max_rpm")) ? state.perf.max_rpm : 0.0; 

    var t1 = prof.enable ? systime() : 0.0;

    # Push to the animation manager (rates are per second, from the last step)
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
        anim.tLast = now;                           # re-base the animation clock: frames integrate from this tick
    }

    if (prof.enable) {
        var t2 = systime();
        prof.add("control", t_ctl);
        prof.add("force", t_frc);
        prof.add("push", t2 - t1);
        prof.add("tick", t2 - t0);
    }
};




########################## animationManager ###########################
# Custom Animation Manager integrated into Flightgear

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

            tNode: props.globals.getNode("/sim/time/delta-sec", 1),         # this frame only
            clockNode: props.globals.getNode("/sim/time/elapsed-sec", 1),   # running sim time

            dLat: 0.0, dLon: 0.0, dAlt: 0.0,
            dHdg: 0.0, dPitch: 0.0, dRoll: 0.0,
            tLast: nil,
            frames: 0, missed: 0, maxDt: 0.0,
            timer: nil
        };

        var animData = ats.controls.animation;
        var callback = func { me.updateObject(myNodeName); };
        animData.timer = maketimer(0.0, callback);
        animData.timer.simulatedTime = 1;
    },

    updateObject: func(myNodeName) {
        var prof = fS.prof;
        var ats = attributes[myNodeName];
        if (ats == nil or ats.damage == 1) {
            if (ats != nil and ats.damage == 1) me.stopObject(myNodeName);
            return;
        }

        var anim = ats.controls.animation;
        if (anim == nil or !anim.isActive) return;

        var t0 = prof.enable ? systime() : 0.0;

        # Own clock: elapsed sim time since this object was last advanced.
        var now = anim.clockNode.getValue() or 0.0;
        var last = anim.tLast;
        anim.tLast = now;
        if (last == nil) return;                    # first frame: just start the clock
        var dt = now - last;
        if (dt <= 0.0) return;                      # paused
        if (dt > 0.25) dt = 0.25;                   # after a hitch do not extrapolate far

        anim.frames += 1;
        if (dt > anim.maxDt) anim.maxDt = dt;
        var fd = anim.tNode.getValue() or 0.0;
        if (fd > 0.0 and dt > 1.5 * fd) anim.missed += 1;

        var lat = anim.latNode.getValue() or 0.0;
        var lon = anim.lonNode.getValue() or 0.0;
        var alt = anim.altNode.getValue() or 0.0;
        var hdg = anim.hdgNode.getValue() or 0.0;
        var pitch = anim.pitchNode.getValue() or 0.0; # Fetch current pitch
        var roll = anim.rollNode.getValue() or 0.0;   # Fetch current roll

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

        if (prof.enable) prof.add("anim", systime() - t0);
    },

    report: func(myNodeName) {
        var anim = attributes[myNodeName].controls.animation;
        print(sprintf("animation %s: %d frames, %d covered more than 1.5 frame-times, longest interval %.1f ms",
                      myNodeName, anim.frames, anim.missed, 1000.0 * anim.maxDt));
    },

    startObject: func(myNodeName) {
        var ats = attributes[myNodeName];
        if (ats != nil and ats.controls != nil and ats.controls.animation != nil) {
            ats.controls.animation.tLast = nil;     # restart the clock
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
        climb_rate: 40000.0,            # ft per min
        descent_rate: 30000.0,          # ft per min
        max_bank_angle: 90.0,           # deg
        max_roll_rate: 175.0,           # deg/sec
        max_pitch_angle: 90.0,          # deg
        max_pitch_rate: 20.0,           # deg/sec
        
        # Force-Vector Parameters
        mass_kg: 18800.0,
        wing_area_m2: 56.5,
        max_thrust_N: 58000.0 * lbf_to_N, # 2x Upgraded -229/GE engines
        max_speed_mps: 790.0 * kts_to_mps,
        thrust_speed_exp: ,

        max_pitch_rate_aero: 30.0, # 30 deg/s
        max_roll_rate_aero:  90.0, # deg/s
        n_max_pos: 6.0, # g factor
        n_max_neg: 3.5, 
        loop_entry_speed_mps: 220.0,   # m/s  entry speed for a loop (blank = 0.97 x max_speed_mps)   #CLAUDE~ 0.3.0: keeps the F-15 default it had before
        loop_n_bottom: 4.0,            # g    design load factor at the bottom of a loop (blank = min(5, 0.9 x n_max_pos))
        loop_min_top_ratio: ,         #      predicted loop top speed / 1 g stall speed below which a loop is refused (blank = 0.7)   #CLAUDE+ new optional key

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
        thrust_speed_exp: ,

        max_pitch_rate_aero: 30.0, # 30 deg/s
        max_roll_rate_aero:  90.0, # deg/s
        n_max_pos: 6.0, # g factor
        n_max_neg: 3.5,    
        loop_entry_speed_mps: ,        # m/s  entry speed for a loop (blank = 0.97 x max_speed_mps)   #CLAUDE+ new optional key
        loop_n_bottom: ,               # g    design load factor at the bottom of a loop (blank = min(5, 0.9 x n_max_pos))
        loop_min_top_ratio: ,         #      predicted loop top speed / 1 g stall speed below which a loop is refused (blank = 0.7)   #CLAUDE+ new optional key
        max_rpm: 3500,              # Merlin engine            

    };
}
###########################################################
# Supermarine Spitfire Mk XVI Performance Profile (Clean, Full Internal Fuel)
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
        thrust_speed_exp: ,

        max_pitch_rate_aero: 35.0,     # deg/s (Maximum transient pitch rate before high-speed stall or G-lock limit)
        max_roll_rate_aero:  175.0,    # deg/s (Aerodynamic maximum roll rate achieved under optimal speed/airspeed pressures)
        n_max_pos: 6.0,                # g factor (Official design limit load factor for structural airframe integrity)
        n_max_neg: 3.5,
        loop_entry_speed_mps: ,        # m/s  entry speed for a loop (blank = 0.97 x max_speed_mps)   #CLAUDE+ new optional key
        loop_n_bottom: ,               # g    design load factor at the bottom of a loop (blank = min(5, 0.9 x n_max_pos))
        loop_min_top_ratio: ,         #      predicted loop top speed / 1 g stall speed below which a loop is refused (blank = 0.7)   #CLAUDE+ new optional key
        max_rpm: 3500,              # Merlin engine            
                
    };
}

elsif (modelName == "/fkdr1") {
    var modelPath = "/home/michael-wilkinson/Desktop/Bombable 5.0/Bombable/FGData/AI/Aircraft/fkdr1-Bombable/Models/dr1.xml";
    var perf_profile = {
        # https://www.gruppofalchi.com/files/Profile-Publications-Aircraft-055---Fokker-DR-1.pdf
        climb_rate: 1078.4,            # ft per min (Max rate of climb ~5.7 m/s)
        descent_rate: 3500.0,          # ft per min (Typical WWI combat dive limit)
        max_bank_angle: 180.0,         # deg (Fully aerobatic maneuverability)
        max_roll_rate: 90.0,           # deg/sec (Moderate aileron authority via interplane struts/warp characteristics)
        max_pitch_angle: 180.0,        # deg (Full loop capability)
        max_pitch_rate: 20.0,          # deg/sec (Highly responsive pitch control due to short tail moment)
        
        # Force-Vector Parameters
        mass_kg: 586.0,                # kg (Standard operational gross weight, pilot, fuel, military load)
        wing_area_m2: 17.48,           # m^2 (Combined area across all three wings, ~201 sq ft)
        max_thrust_N: 1900.0,          # N (Climb Thrust ~60–70 mph / 100–110 km/h, full throttle for the 110 hp Oberursel Ur.II rotary engine)
        max_speed_mps: 51.4,           # m/s (Maximum level speed, ~110–115 mph / 97–100 knots)
        thrust_speed_exp: ,

        max_pitch_rate_aero: 25.0,     # deg/s (Transient aerodynamic pitch limit)
        max_roll_rate_aero:  90.0,     # deg/s (Maximum aerodynamic roll rate)
        n_max_pos: 5.0,                # g factor (WWI structural design limit load factor)
        n_max_neg: 2.5,
        loop_entry_speed_mps: ,        # m/s  entry speed for a loop (blank = 0.97 x max_speed_mps)   #CLAUDE+ new optional key
        loop_n_bottom: ,               # g    design load factor at the bottom of a loop (blank = min(5, 0.9 x n_max_pos))
        loop_min_top_ratio: ,         #      predicted loop top speed / 1 g stall speed below which a loop is refused (blank = 0.7)   #CLAUDE+ new optional key
        max_rpm: 1200,                 # engine (Typical maximum RPM for the Oberursel Ur.II rotary engine)
                    
    };
}

elsif (modelName == "/a6m5") {
    # The A6M5b (Model 52b Otsu) presents a contrast to the late-mark Spitfire XVI. 
    # Where the Spitfire rely on sheer power (1,700+ hp Rolls-Royce Merlin 66) and high-speed elevator authority, 
    # the A6M5b relies on low wing loading, a light airframe, and lower power (~1,130 hp Nakajima Sakae 21 engine).
    # The key aerodynamic hallmark of the Zero—its extreme low-speed roll authority and high-speed aileron stiffening—
    # needs careful representation in your flight dynamics model. 
    # At airspeeds above ~200 kts (100 m/s), the Zero’s mechanical ailerons stiffened drastically, 
    # dropping its roll rate from top-tier agility to a sluggish rate.

    var modelPath = "/home/michael-wilkinson/Desktop/Bombable 5.0/Bombable/FGData/AI/Aircraft/A6M5-Bombable/Models/a6m5b-green-fSim.xml";
    var perf_profile = {
        climb_rate: 3100.0,           # ft/min (Max initial rate of climb near sea level; ~15.7 m/s)
        descent_rate: 4500.0,         # ft/min (Standard operational descent; max dive IAS limit was 740 km/h / 400 kts)
        max_bank_angle: 180.0,        # deg (Fully aerobatic, though inverted fuel supply limits sustained inverted flight)
        max_roll_rate: 135.0,         # deg/sec (Peak roll rate at low IAS (~130–150 kts); stiffens severely above 200 kts)
        max_pitch_angle: 180.0,       # deg (Full vertical loop capability)
        max_pitch_rate: 22.0,         # deg/sec (Light, highly responsive elevator at low/medium speeds)
        
        # Force-Vector Parameters
        mass_kg: 2730.0,              # kg (Normal combat loaded weight; significantly lighter than Spitfire's 3,400 kg)
        wing_area_m2: 21.30,          # m^2 (Redesigned non-folding clipped wing for the Model 52 series)
        max_thrust_N: 9500.0,         # N (Peak propeller static thrust from the 1,130 hp Nakajima Sakae 21 engine)
        max_speed_mps: 156.9,         # m/s (565 km/h / 305 kts max level speed at rated altitude of 6,000 m / 19,680 ft)
        thrust_speed_exp: 0.85,       # Exponent for propeller thrust decay with forward velocity (typical fixed-pitch/constant-speed prop curve)

        max_pitch_rate_aero: 30.0,    # deg/s (Instantaneous pitch rate at low IAS before accelerated stall)
        max_roll_rate_aero: 135.0,    # deg/s (Peak aerodynamic roll rate; note severe aeroelastic/stiffening penalty at VNE)
        n_max_pos: 7.0,               # g factor (A6M5 reinforced wing skin increased safe load factor over early A6M2)
        n_max_neg: 3.5,               # g factor
        loop_entry_speed_mps: ,        # m/s  entry speed for a loop (blank = 0.97 x max_speed_mps)   #CLAUDE+ new optional key
        loop_n_bottom: ,               # g    design load factor at the bottom of a loop (blank = min(5, 0.9 x n_max_pos))
        loop_min_top_ratio: ,         #      predicted loop top speed / 1 g stall speed below which a loop is refused (blank = 0.7)   #CLAUDE+ new optional key
        max_rpm: 2750,                # RPM (Redline for the Sakae 21 two-stage supercharged radial engine)
    };
}

elsif (modelName == "/ju87") {

    var modelPath = "/home/michael-wilkinson/Desktop/Bombable 5.0/Bombable/FGData/AI/Aircraft/Ju87/Models/ju87.xml";
    var perf_profile = {
        climb_rate: 1450.0,           # ft/min (Max initial rate of climb near sea level; ~7.4 m/s)
        descent_rate: 5500.0,         # ft/min (Steep dive operational descent; terminal dive velocity ~550-600 km/h with dive brakes)
        max_bank_angle: 180.0,        # deg (Fully aerobatic within structural and fuel feed limits)
        max_roll_rate: 75.0,          # deg/sec (Sluggish roll rate due to high span and heavy fabric-covered ailerons)
        max_pitch_angle: 180.0,       # deg (Full vertical loop capability)
        max_pitch_rate: 18.0,         # deg/sec (Responsive elevator at low/medium speeds, stiffening in high-speed dives)
        
        # Force-Vector Parameters
        mass_kg: 4320.0,              # kg (Normal combat loaded weight for Ju 87B-2)
        wing_area_m2: 31.90,          # m^2 (Large inverted gull wing providing low wing loading and stable low-speed handling)
        max_thrust_N: 5200.0,         # N (Static thrust from the Junkers Jumo 211D liquid-cooled V12 engine, ~1,200 PS)
        max_speed_mps: 108.3,         # m/s (390 km/h / 210 kts max level speed at rated altitude of 4,400 m / 14,400 ft)
        thrust_speed_exp: 0.85,       # Exponent for propeller thrust decay with forward velocity

        max_pitch_rate_aero: 22.0,    # deg/s (Instantaneous pitch rate before G-limiting during dive recovery)
        max_roll_rate_aero: 75.0,     # deg/s (Aerodynamic roll rate; limited by heavy wing inertia and aileron control forces)
        n_max_pos: 7.0,               # g factor (Robust structural limit designed to handle violent 6g–7g dive pullouts)
        n_max_neg: 3.0,               # g factor
        loop_entry_speed_mps: ,       # m/s  entry speed for a loop (blank = 0.97 x max_speed_mps)   #CLAUDE+ new optional key
        loop_n_bottom: ,              # g    design load factor at the bottom of a loop (blank = min(5, 0.9 x n_max_pos))
        loop_min_top_ratio: 0.5,      # predicted loop top speed / 1 g stall speed below which a loop is refused (blank = 0.7)   #CLAUDE+ new optional key
        terminal_speed_mps: 159.7,    # m/s  575 km/h terminal dive speed with dive brakes out, sea level, vertical   #CLAUDE+ 0.6.0 new optional key: enables start_dive
        max_rpm: 2400,                # RPM (Rated combat redline for the Jumo 211 inverted V12 engine)
    };
}

elsif (modelName == "/bf109g14") {

    var modelPath = "/home/michael-wilkinson/Desktop/Bombable 5.0/Bombable/FGData/AI/Aircraft/BF109G14_late_war/Models/bf109g14_new.xml";
    var perf_profile = {
        climb_rate: 3770.0,            # ft per min (~19.2 m/s max initial rate of climb near sea level under emergency boost)
        descent_rate: 6000.0,          # ft per min (High-speed tactical push-over; VNE dive limit reached rapidly up to 850 km/h IAS)
        max_bank_angle: 180.0,         # deg (Fully aerobatic, though inverted fuel supply/carburetor float limits negative-G maneuvers)
        max_roll_rate: 120.0,          # deg/sec (Good low-to-medium speed roll authority via Frise ailerons, but stiffens severely at high IAS)
        max_pitch_angle: 180.0,        # deg (Full vertical loop capability, excels in vertical energy retention/zoom climbs)
        max_pitch_rate: 22.0,          # deg/sec (Heavier elevator control forces at high speeds due to rigid stabilizer linkage)
        
        # Force-Vector Parameters
        mass_kg: 3266.0,               # kg (Standard operational combat weight with full internal fuel and MW-50 system)
        wing_area_m2: 16.10,           # m^2 (Compact, highly loaded wing area yielding ~173 sq ft)
        max_thrust_N: 11500.0,         # N (Peak static propeller thrust derived from the 1,800 hp DB 605AM emergency power rating)
        max_speed_mps: 185.5,          # m/s (Maximum true air speed at altitude, equal to ~668 km/h / 360 knots at ~5,500 m with MW-50)
        thrust_speed_exp: 0.82,        # Exponent for Daimler-Benz constant-speed variable pitch propeller efficiency curve

        max_pitch_rate_aero: 28.0,     # deg/s (Transient pitch rate limit before compressibility or high elevator hinge moments)
        max_roll_rate_aero:  120.0,    # deg/s (Aerodynamic maximum roll rate achieved under optimal dynamic pressure windows)
        n_max_pos: 7.0,                # g factor (Sturdy structural design load factor favored by German operational criteria)
        n_max_neg: 3.0,                # g factor
        loop_entry_speed_mps: ,        # m/s  entry speed for a loop (blank = 0.97 x max_speed_mps)   #CLAUDE+ new optional key
        loop_n_bottom: ,               # g    design load factor at the bottom of a loop (blank = min(5, 0.9 x n_max_pos))
        loop_min_top_ratio: ,         #      predicted loop top speed / 1 g stall speed below which a loop is refused (blank = 0.7)   #CLAUDE+ new optional key
        max_rpm: 2800,                 # RPM (Redline for the DB 605AM engine under 1.7 ata emergency MW-50 boost)
    };
}


else {
    die("fSim: no performance profile for model " ~ modelName);   # call error instead of an undefined perf_profile at load
}
print(modelName ~ " initialised");
print("fSim version " ~ FSIM_VERSION ~ " loaded");

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

#CLAUDE+ 0.6.0: air-brake drag. A vertical dive at terminal speed has no lift and idle thrust, so drag = weight:
#CLAUDE+   CD0_brake = W / (0.5 * rho0 * vt^2 * S) - K_IND * C_L_vmax^2, vt = perf terminal_speed_mps (a sea-level value). The last term is the
#CLAUDE+ induced drag the model still has at zero angle of attack (CL = C_L_vmax). nil when the profile has no terminal_speed_mps.
var C_D0_BRAKE = (perf_profile["terminal_speed_mps"] != nil)
    ? (2.0 * weight_N / (rho0 * perf_profile["terminal_speed_mps"] * perf_profile["terminal_speed_mps"] * perf_profile.wing_area_m2)
       - K_IND * C_L_vmax * C_L_vmax)
    : nil;
var DIVE_BRAKE_TIME = 1.0;     # s  brakes take this long to extend and to retract

# Zero-lift drag now: clean, or blended towards the brake value by the deployment fraction self.brake (0..1).
var cd0_now = func(self) {
    if (C_D0_BRAKE == nil) return C_D0;
    return C_D0 + (self.brake or 0.0) * (C_D0_BRAKE - C_D0);
};
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
        gamma_deg: ,
        alpha_deg: ,
        
        # Speeds & 3D Velocity Vector (East, North, Up in m/s)
        speed: initial_speed_kts,
        v_east: initial_spd_mps * math.sin(hdg_rad),
        v_north: initial_spd_mps * math.cos(hdg_rad),
        v_up: 0.0,
        
        # Target controls
        tgt_heading: initial_hdg,
        tgt_speed: initial_speed_kts,
        auto_throttle: 0,   #CLAUDE~ 0.3.0: 1 = cruise holds tgt_speed with the throttle, 0 = throttle set by hand
        pre_manoeuvre: ,   #CLAUDE~ 0.3.0: throttle mode saved when a manoeuvre starts, applied by cruise() when it ends
        tgt_altitude_ft: initial_alt_ft,
        tgt_roll: 0.0,
        tgt_pitch: 0.0,
        throttle: initial_throttle,

        # climb rate control (internal)
        climb_cache: ,
        drag_N: ,
        gamma_cmd_f: ,
        gamma_cmd_t: ,
        
        # Rates for animationManager compatibility (per second)
        dLat: 0.0,
        dLon: 0.0,
        dAlt: 0.0,
        dHdg: 0.0,
        dPitch: 0.0,
        dRoll: 0.0,

        brake: 0.0,        #CLAUDE+ 0.6.0: air-brake deployment 0..1 (moves towards brake_cmd)
        brake_cmd: 0,      #CLAUDE+ 0.6.0: 1 = brakes commanded out; cruise() resets it to 0
        aerobatic: 0, # flag to enable aerobgatic performance
        manoeuvre: nil,
        projectiles: [],   #CLAUDE+ 0.7.0: projectiles in flight (release_projectile)
        last_landing: nil,  #CLAUDE+ 0.7.0: result of the last landing (position, miss)
        last_approach: nil,   #CLAUDE+ 0.7.3: result of the last approach (outcome, reason and the errors at the gate or at the abort)
        monitor: nil,
        n_load: ,        

        perf: perf_profile,

        tNode: tNode,
        time: tNode.getValue(),
        dTime: 0.1,
    };
};


########################## Initialize location ###########################
# 3. Initialize Airport & Runway Position Data
# Using EGOD as default reference airport (or change to your preferred airport ID e.g. "EGLL")
var airport_id = "LFML"; 
var apt = airportinfo(airport_id);

var init_lat = 37.61899; # Fallback latitude
var init_lon = -122.3749; # Fallback longitude
var init_alt_ft = 2013.0; # Fallback altitude (+5000ft over sea level)

if (apt != nil and size(apt.runways) > 0) {
    var runways_list = values(apt.runways);
    var first_rwy = runways_list[0];
    
    init_lat = first_rwy.lat;
    init_lon = first_rwy.lon;

    var apt_elev_ft = apt.elevation * 3.28084 or 0.0;
    init_alt_ft = apt_elev_ft + 2000.0;
    print("fSim.nas: Initialized position from " ~ airport_id ~ " Runway threshold at Lat: " ~ init_lat ~ ", Lon: " ~ init_lon);
} else {
    print("fSim.nas: Warning - Could not load airport " ~ airport_id ~ ", using default fallback coordinates.");
}

# Initial flight parameters for level flight northwards (Heading 0°, Speed 250 kts)
var init_hdg = 200.0;
var init_speed = 300.0; # knots
var init_throttle = 1.0;
var init_rpm = 3500;

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

# 4. Load model via fgcommand
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

print("fSim.nas: Spawned model from " ~ modelPath);


########################## monitorValues ###########################

#CLAUDE~ 0.6.1: was a settimer chain inside monitorValues; now the callback of monitor_timer (one print per call)
var monitorTick = func {
    if (fS.state.monitor == nil) {
        monitor_timer.stop();
        return;
    }
    if (fS.state["manoeuvre"] != nil) var mPhase = fS.state.manoeuvre["phase"] else var mPhase = "";
    # print(sprintf("Phase: %s Gamma: %.2f Pitch %.2f Alpha: %.2f Drag: %.2f Load: %.2f Stalled %d Speed_mps: %.1f Alt_ft: %.1f Heading %.1f Roll %.1f Throttle %.1f Tick_interval %.3f",
    print(sprintf("Phase: %s Gamma: %.2f Pitch %.2f Alpha: %.2f Drag: %.2f Load: %.2f v_stall_1g %.1f Speed_mps: %.1f Alt_ft: %.1f v_up %.1f best_climb_speed %.1f Throttle %.1f Brake %.1f Tick_interval %.3f",
    mPhase,
    fS.state.gamma_deg,
    fS.state.pitch,
    fS.state.alpha_deg,
    fS.state.drag_N,
    fS.state.n_load,
    # fS.state.stalled,
    v_stall_1g(fS.state),
    fS.state.speed * kts_to_mps,
    fS.state.alt_ft,
    # fS.state.hdg,
    fS.state.v_up,
    # fS.state.roll,
    (fS.state.climb_cache != nil) ? fS.state.climb_cache["v"] : 0.0,
    fS.state.throttle,
    fS.state.brake,
    fS.state.dTime
    ));
    monitor_n += 1;
    if (monitor_n >= monitor_max) {
        monitor_timer.stop();
        fS.state.monitor = nil;
    }
};

#CLAUDE+ 0.6.1: monitorValues(count, countmax, dt) starts (or restarts) the monitor: one row now, then one per dt sim seconds until countmax
#CLAUDE+ rows have been printed. state.monitor must be non-nil, as before; it is reset to nil when the run ends.
var MONITOR_INTERVAL = 0.2;   # s, default sample interval used by start_manoeuvre
var monitor_n = 0;
var monitor_max = 0;
var monitorValues = func(count = 0, countmax = 500, dt = 1) {
    if (fS.state.monitor == nil) return;
    monitor_n = count;
    monitor_max = countmax;
    monitorTick();
    if (fS.state.monitor != nil) monitor_timer.restart(dt);
};
########################## Start flying ###########################

# 5. Create Aircraft State Instance & Register with animationManager
# Initialize the state
fS.state = create_aircraft_state(modelName, init_lat, init_lon, init_alt_ft, init_hdg, init_speed, init_throttle);

var attributes = {};
attributes[modelName] = {
    damage: 0,
    controls: {},
};

# Register object with animationManager
animationManager.register(modelName);


# Main Physics Tick Function (Runs at a slower interval, e.g., 10 Hz / 0.1s)
var physics_update_interval = 0.16; 

# 2. Wrap it in a closure for maketimer so it evaluates per tick with modelName
var physics_timer = maketimer(physics_update_interval, func { run_flight_physics(modelName); });

physics_timer.simulatedTime = 1;

# Start both systems:
animationManager.startObject(modelName); # Keeps the 3D model moving smoothly per-frame
physics_timer.start();                   # Updates the physics rates 10x per second

#CLAUDE+ 0.6.1: the monitor timer (started by monitorValues). Created here, beside the physics timer, so there is exactly one.
var monitor_timer = maketimer(MONITOR_INTERVAL, monitorTick);
monitor_timer.simulatedTime = 1;


########################## Start camera ###########################
# Configure and activate a dynamic chase view slot for F15 matching Bombable's architecture
var setup_fSim_chase_view = func(slot_idx, mName) {
    # 1. Ensure the view slot configuration path exists
    var raw_slot_prop = "/sim/views/ai-targets/slot-" ~ slot_idx;

    setprop(raw_slot_prop, mName);

    setprop("/sim/view[" ~ slot_idx ~ "]/config/x-offset-m", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/y-offset-m", -3);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/z-offset-m", 2);

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

setup_fSim_chase_view(101, modelName);

# Automatically load a nasal console
settimer(func {
    fgcommand("dialog-show", props.Node.new({"dialog-name": "nasal-console"}));

# Open multiple property browsers at specific paths
var paths = [
    "/position",
    "/orientation",
    "/velocities",
    "/controls/manoeuvres",
];

foreach (var path; paths) {
    # Fetch the Property Node object for the given path
    var node = props.globals.getNode(modelName ~ path);
    
    if (node != nil) {
        # Opens a new property browser dialog focused on the specified node
        gui.property_browser(node);
        }
    }
}, 10);

########################## register_manoeuvre_listeners ###########################
# Example listener registration (adapt the node path to your controls node):

  var register_manoeuvre_listeners = func(mName, self) {
      foreach (var name; ["loop", "immelmann", "aileron_roll", "split_s", "climb", "dive", "approach"]) {
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
# Returns the manoeuvre hash, or nil if it was refused or one is already running.

var start_manoeuvre = func(mname, self, name, opts = nil) {
    if (self.manoeuvre != nil) {
        print("Manoeuvre " ~ self.manoeuvre.name ~ " is already running");
        return nil;
    }
    var ret = nil;
    if (name == "loop")              ret = start_loop(self, opts);
    else if (name == "immelmann")    ret = start_immelmann(self, opts);
    else if (name == "aileron_roll") ret = start_aileron_roll(self, opts);
    else if (name == "split_s")      ret = start_split_s(self, opts);
    else if (name == "climb")        ret = start_climb(self, opts);   #CLAUDE+ 0.4.0
    else if (name == "dive")         ret = start_dive(self, opts);    #CLAUDE+ 0.6.0
    else if (name == "approach")     ret = start_approach(self, opts);   #CLAUDE+ 0.7.3
    else {
        print("Unknown manoeuvre: " ~ name);
        return nil;
    }
    if (ret == nil) return ret;
    self.pre_manoeuvre = { auto: self.auto_throttle, thr: self.throttle };   #CLAUDE~ 0.3.0: cruise() restores this when the manoeuvre ends
    print("Starting ", name, " for ", mname, " manoeuvre options ...");
    if (opts != nil) debug.dump(opts) else print("No options defined using defaults");
    print("Dumping aircraft performance parameters ...");
    debug.dump(self.perf);
    print("Monitoring flight parameters ...");
    monitorValues(0, (opts != nil and opts["monitor_rows"] != nil) ? opts["monitor_rows"] : 500, MONITOR_INTERVAL);   #CLAUDE~ 0.7.1: option monitor_rows, default 500 (rows at 0.2 s: an approach plus a dive can need more); was monitorValues(0, 300, 1) in 0.6.0
    return self.manoeuvre;
};

########################## series runner (0.7.2) ###########################
#CLAUDE+ TEST CODE, not part of the flight model. run_series(name, cases, sopts) runs ONE kind of manoeuvre again and again from a repeatable start
#CLAUDE+ and prints one row per case, so a batch can run unattended (speed-up works: it runs on simulated physics ticks, no timer).
#CLAUDE+   name  : any manoeuvre start_manoeuvre knows: dive, loop, immelmann, aileron_roll, split_s, climb
#CLAUDE+   cases : vector of { label, opts, start }. opts = the manoeuvre's own options. start = { lat, lon, alt_ft, speed_kt, hdg, throttle }; 
#CLAUDE+   a missing value is the value the aircraft had when run_series was called
#CLAUDE+   sopts : { settle (time in sec of level flight before each manoeuvre, default 3), max_s (time limit per case, default 180) }
#CLAUDE+ series_cases(key, values, base_opts, base_start, where) builds the cases for a one-parameter sweep of an option (where "opts", the default)
#CLAUDE+ or of a start value (where "start"), e.g. series_cases("dive_angle_deg", [50, 60, 70], { pullout_agl_ft: 2000 }).
#CLAUDE+ Each case puts the aircraft back at the start, settles, starts the manoeuvre and waits for it to end. The row gives the outcome, the
#CLAUDE+ duration, altitudes, speeds, peak load and, for each phase, the altitude, speed and time of the tick on which the phase began
#CLAUDE+ (the state the controller saw when it changed phase). run_series_stop() abandons a series (a manoeuvre already running carries on).
#CLAUDE+ Results are also kept in series.results. Throttle mode (manual or auto) is the one in force when run_series was called.
var series = {
    running: 0, name: "", cases: [], i: 0, settle: 3.0, max_s: 180.0, home: 1, results: [],
    base: nil, start: nil, phase: "idle", t: 0.0, t_run: 0.0, last_phase: nil, phases: "",
    alt0: 0.0, prev_alt: 0.0, prev_v: 0.0, min_alt: 0.0, max_v: 0.0, max_n: 0.0
};

var series_cases = func(key, values, base_opts = nil, base_start = nil, where = "opts") {
    var out = [];
    foreach (var val; values) {
        var o = {};  var st = {};
        if (base_opts != nil)  foreach (var k; keys(base_opts))  o[k]  = base_opts[k];
        if (base_start != nil) foreach (var k; keys(base_start)) st[k] = base_start[k];
        if (where == "start") st[key] = val; else o[key] = val;
        append(out, { label: key ~ "=" ~ val, opts: o, start: st });
    }
    return out;
};

# Put the aircraft back at the start of a case, in place (the listeners and the monitor keep their reference to the state hash).
var series_reset_state = func(base, start) {
    var st = fS.state;
    var fresh = create_aircraft_state(modelName, start.lat, start.lon, start.alt_ft, start.hdg, start.speed_kt, start.throttle);   #CLAUDE~ 0.7.3: start.lat and lon (were base.lat, base.lon)
    foreach (var k; keys(fresh)) {
        if (k == "perf" or k == "tNode" or k == "monitor" or k == "callsign") continue;
        st[k] = fresh[k];
    }
    st.auto_throttle = base.auto_throttle;
    latNode.setDoubleValue(start.lat);
    lonNode.setDoubleValue(start.lon);
    altNode.setDoubleValue(start.alt_ft);
    hdgNode.setDoubleValue(start.hdg);
    pitchNode.setDoubleValue(0.0);
    rollNode.setDoubleValue(0.0);
    spdNode.setDoubleValue(start.speed_kt);
    var anim = attributes[modelName].controls["animation"];
    if (anim != nil) {
        anim.dLat = 0.0;  anim.dLon = 0.0;  anim.dAlt = 0.0;
        anim.dHdg = 0.0;  anim.dPitch = 0.0;  anim.dRoll = 0.0;
    }
    return st;
};

var series_next = func(self) {
    var s = series;
    var c = s.cases[s.i];
    var b = s.base;
    var st = c["start"] or {};
    s.start = { lat:      (st["lat"]      != nil) ? st["lat"]      : b.lat,    #CLAUDE+ 0.7.3: start lat and lon (default: where run_series was called)
                lon:      (st["lon"]      != nil) ? st["lon"]      : b.lon,
                alt_ft:   (st["alt_ft"]   != nil) ? st["alt_ft"]   : b.alt_ft,
                speed_kt: (st["speed_kt"] != nil) ? st["speed_kt"] : b.speed_kt,
                hdg:      (st["hdg"]      != nil) ? st["hdg"]      : b.hdg,
                throttle: (st["throttle"] != nil) ? st["throttle"] : b.throttle };
    series_reset_state(b, s.start);
    s.phase = "settle";  s.t = 0.0;
};

var series_row = func(self, outcome) {
    var s = series;
    var c = s.cases[s.i];
    var v = self.speed * kts_to_mps;
    var la = self["last_approach"];
    var ap = (la == nil) ? "" : sprintf(" | approach %s%s: xt %+.0f m, dh %+.0f ft, dv %+.1f m/s, dpsi %+.1f deg", la.outcome, (la.reason == "") ? "" : " (" ~ la.reason ~ ")", la.xt, la.dh_ft, la.dv, la.dpsi);   #CLAUDE+ 0.7.3
    print(sprintf("SERIES %2d/%d %-18s %-8s %6.1f s | alt %6.0f > min %6.0f > end %6.0f ft | speed max %5.1f end %5.1f m/s | load max %4.1f | %s%s",
                  s.i + 1, size(s.cases), c["label"] or "", outcome, s.t_run, s.alt0, s.min_alt, self.alt_ft, s.max_v, v, s.max_n, s.phases, ap));
    append(s.results, { label: c["label"], outcome: outcome, t: s.t_run, alt0: s.alt0, min_alt: s.min_alt, end_alt: self.alt_ft,
                        max_v: s.max_v, end_v: v, max_n: s.max_n, phases: s.phases, approach: la });
    s.i += 1;
    if (s.i >= size(s.cases)) {
        s.running = 0;  s.phase = "idle";
        print("SERIES done: " ~ size(s.cases) ~ " cases");
        if (s.home) {
            series_reset_state(s.base, s.base);   # base has the same keys as a case start
            print("SERIES: aircraft back at the start position");
        }
    } else {
        series_next(self);
    }
};

var series_launch = func(self) {
    var s = series;
    var c = s.cases[s.i];
    s.t_run = 0.0;  s.last_phase = nil;  s.phases = "";
    s.alt0 = self.alt_ft;  s.prev_alt = self.alt_ft;  s.prev_v = self.speed * kts_to_mps;
    s.min_alt = self.alt_ft;  s.max_v = s.prev_v;  s.max_n = 0.0;
    var m = start_manoeuvre(modelName, self, s.name, c["opts"]);
    if (m == nil) {
        series_row(self, "REFUSED");
        return;
    }
    s.phase = "run";
};

# Called by run_flight_physics after every physics tick while a series is running.
var series_step = func(self, dt) {
    var s = series;
    if (s.phase == "settle") {
        s.t += dt;
        if (s.t >= s.settle) series_launch(self);
        return;
    }
    if (s.phase != "run") return;
    s.t_run += dt;
    var m = self.manoeuvre;
    if (m != nil) {
        var ph = (m["phase"] == nil) ? "-" : m["phase"];
        if (ph != s.last_phase) {
            s.phases = s.phases ~ (s.phases == "" ? "" : " > ") ~ sprintf("%s %.0f ft %.0f m/s %.1f s", ph, s.prev_alt, s.prev_v, s.t_run - dt);
            s.last_phase = ph;
        }
    }
    var v = self.speed * kts_to_mps;
    if (self.alt_ft < s.min_alt) s.min_alt = self.alt_ft;
    if (v > s.max_v) s.max_v = v;
    if ((self.n_load or 0.0) > s.max_n) s.max_n = self.n_load;
    s.prev_alt = self.alt_ft;  s.prev_v = v;
    if (m == nil) {
        series_row(self, "ended");
    } else if (s.t_run > s.max_s) {
        self.manoeuvre = nil;  self.aerobatic = 0;  self.brake_cmd = 0;
        series_row(self, "TIMEOUT");
    }
};

var run_series = func(name, cases, sopts = nil) {
    var s = series;
    sopts = sopts or {};
    var st = fS.state;
    if (s.running) {
        print("A series is already running (run_series_stop() abandons it)");
        return nil;
    }
    if (st.manoeuvre != nil) {
        print("Series not started: manoeuvre " ~ st.manoeuvre.name ~ " is running");
        return nil;
    }
    if (cases == nil or size(cases) == 0) {
        print("Series not started: no cases");
        return nil;
    }
    var pos = own_position(st);
    s.base = { lat: pos.lat, lon: pos.lon, alt_ft: st.alt_ft, speed_kt: st.speed, hdg: st.hdg, throttle: st.throttle, auto_throttle: st.auto_throttle };
    s.name = name;  s.cases = cases;  s.i = 0;  s.results = [];
    s.settle = (sopts["settle"] != nil) ? sopts["settle"] : 3.0;
    s.max_s  = (sopts["max_s"]  != nil) ? sopts["max_s"]  : 180.0;
    s.home   = (sopts["home"]   != nil) ? sopts["home"]   : 1;
    print(sprintf("SERIES start: %d cases of %s from %.5f, %.5f at %.0f ft, %.0f kt, heading %.0f, settle %.1f s", size(cases), name,
                  pos.lat, pos.lon, st.alt_ft, st.speed, st.hdg, s.settle));
    series_next(st);
    s.running = 1;
    return size(cases);
};

var run_series_stop = func(home = 0) {
    series.running = 0;  series.phase = "idle";
    print("SERIES stopped");
    if (home and series.base != nil) {
        series_reset_state(series.base, series.base);   # also ends any manoeuvre: the state is rebuilt
        print("SERIES: aircraft back at the start position");
    }
};

register_manoeuvre_listeners(modelName, fS.state);
