# Abort and go-around demonstrations for the approach manoeuvre (fSim 0.7.3.1).
# Load this file in the Nasal console (or as a second file), then call ONE variant at a time from the console, e.g.   approach_tests.v1_gate_abort_ship();
# Each variant starts the approach, prints what the guidance reports, and when the approach is over prints one RESULT line from state.last_approach.
# Fly the aircraft level at a steady speed first (this is the start state the variants assume). 2x simulation speed is fine.
# v1 to v3 reuse your own ship test: start them from the position where your own test worked (on the 3 deg path to the gate, or phase 0 refuses).
# Variants that need a reference you can switch off or turn (a ship cannot be) use a test reference in the property tree, /sim/fsim-test/ref,
# which this file moves by dead reckoning every 0.5 s of simulated time; it is placed ahead of the aircraft, sized by its speed.
#
#   v1_gate_abort_ship()     the ship and your options, but a gate tolerance that can never be met: ABORT at the gate, default go-around
#   v2_abort_cruise_ship()   the same, on_abort "cruise": handed back at once, no climb
#   v3_abort_handler_ship()  the same, on_abort = a function that starts a max-rate climb itself
#   v4_reference_lost()      the reference is switched off 20 s into the approach: ABORT "reference was lost", go-around
#   v5_ship_turns()          a moving reference turns 25 deg, the line turns with it (course_rel): ABORT "left the funnel", go-around
#   v6_not_established()     forced start from a bad position (offset line, little distance): ABORT "not established", go-around
#   v7_entry_speed()         a successor dive needs 1.3 x stall speed, the approach flies 1.25 x: ABORT at the gate on speed
#   v8_refused_preview()     phase 0 only (preview): reference behind, reference outrunning the aircraft, a line that is too close
#   v9_fast_ref_join(path)   join astern of a faster moving object (path = an AI aircraft such as "/ai/models/aircraft[0]"; default: a test reference at 250 kt)
var approach_tests = {};

var _d2r = 0.017453292519943295;
var _kt = 0.514444;

# Test reference in the property tree: ahead of the aircraft by dist_m along its heading and cross_m to the right.
approach_tests.make_ref = func(dist_m, cross_m, hdg, kt, alt_ft) {
    var st  = fSim.fS.state;
    var pos = fSim.own_position(st);
    var h   = st.hdg * _d2r;
    var ll  = fSim.lat_lon_at(pos.lat, pos.lon, dist_m * math.sin(h) + cross_m * math.cos(h), dist_m * math.cos(h) - cross_m * math.sin(h));
    var path = "/sim/fsim-test/ref";
    setprop(path ~ "/position/latitude-deg", ll.lat);
    setprop(path ~ "/position/longitude-deg", ll.lon);
    setprop(path ~ "/position/altitude-ft", alt_ft);
    setprop(path ~ "/orientation/true-heading-deg", hdg);
    setprop(path ~ "/velocities/true-airspeed-kt", kt);
    setprop(path ~ "/valid", 1);
    var r = { path: path, t: fSim.sim_time(), timer: nil };
    r.step = func {
        var now = fSim.sim_time();
        var dt  = now - r.t;
        r.t = now;
        var v = getprop(path ~ "/velocities/true-airspeed-kt") * _kt;
        var hd = getprop(path ~ "/orientation/true-heading-deg") * _d2r;
        var la = getprop(path ~ "/position/latitude-deg");
        var lo = getprop(path ~ "/position/longitude-deg");
        var n = fSim.lat_lon_at(la, lo, v * dt * math.sin(hd), v * dt * math.cos(hd));
        setprop(path ~ "/position/latitude-deg", n.lat);
        setprop(path ~ "/position/longitude-deg", n.lon);
    };
    r.turn = func(deg) {
        var hd = getprop(path ~ "/orientation/true-heading-deg") + deg;
        setprop(path ~ "/orientation/true-heading-deg", hd - 360.0 * math.floor(hd / 360.0));
        print(sprintf("TEST reference turned by %+.0f deg", deg));
    };
    r.lose = func { setprop(path ~ "/valid", 0); print("TEST reference switched off"); };
    r.stop = func { if (r.timer != nil) r.timer.stop(); };
    r.timer = maketimer(0.5, func { r.step(); });
    r.timer.simulatedTime = 1;
    r.timer.start();
    return r;
};

# Run something once, after s seconds of simulated time.
approach_tests.after = func(s, f) {
    var t = maketimer(s, func { t.stop(); f(); });
    t.simulatedTime = 1;
    t.start();
};

# Start the approach and print one RESULT line when it is over (gate reached and the successor started, or abort and go-around finished).
approach_tests.go = func(label, opts, ref = nil) {
    var st = fSim.fS.state;
    print("TEST ===== " ~ label ~ " =====");
    var m = fSim.start_manoeuvre(fSim.modelName, st, "approach", opts);
    if (m == nil) {
        var la = st.last_approach;
        print("TEST RESULT not started" ~ ((la != nil) ? ": " ~ la.reason : ""));
        if (ref != nil) ref.stop();
        return nil;
    }
    var w = maketimer(2.0, func {
        var s2 = fSim.fS.state;
        if (s2.manoeuvre != nil and s2.manoeuvre.name == "approach") return;
        w.stop();
        if (ref != nil) ref.stop();
        var la = s2.last_approach;
        if (la == nil) { print("TEST RESULT no result recorded"); return; }
        print(sprintf("TEST RESULT %s%s after %.0f s (phase %s): cross-track %+.1f m, height %+.0f ft, speed %+.1f m/s, heading %+.1f deg; now %s, %.0f ft",
              la.outcome, (la.reason == "") ? "" : " [" ~ la.reason ~ "]", la.t, la.phase, la.xt, la.dh_ft, la.dv, la.dpsi,
              (s2.manoeuvre == nil) ? "cruise" : s2.manoeuvre.name, s2.alt_ft));
    });
    w.simulatedTime = 1;
    w.start();
    return m;
};

# speed (m/s) and the distance flown in s seconds
approach_tests.v = func { return fSim.fS.state.speed * _kt; };

# ---- variants that use the ship exactly as in your own test --------------------------------------------------------------------------------
var _ship_opts = func(extra) {
    var o = { ref: "/ai/models/ship[0]", course_deg: 180, course_rel: 1, glide_deg: 3.0, gate_m: 1000, gate_dh_ft: 500,
              then: "climb", then_opts: { alt_ft: 2000, schedule: "max_rate" }, ga_margin_ft: 1000 };
    foreach (var k; keys(extra)) o[k] = extra[k];
    return o;
};
# a negative tolerance can never be met, so the gate check fails when the gate is reached, however well the aircraft is flying
approach_tests.v1_gate_abort_ship = func {
    approach_tests.go("v1: the ship, gate cross-track tolerance -1 m (cannot be met), on_abort goaround", _ship_opts({ gate_xt_m: -1.0, on_abort: "goaround" }));
};
approach_tests.v2_abort_cruise_ship = func {
    approach_tests.go("v2: the ship, impossible gate tolerance, on_abort cruise", _ship_opts({ gate_xt_m: -1.0, on_abort: "cruise" }));
};
approach_tests.v3_abort_handler_ship = func {
    approach_tests.go("v3: the ship, impossible gate tolerance, on_abort = a function that starts a climb", _ship_opts({ gate_xt_m: -1.0,
        on_abort: func(self, m, reason) {
            print("TEST handler called: " ~ reason);
            fSim.start_manoeuvre(fSim.modelName, self, "climb", { alt_ft: self.alt_ft + 1500.0, schedule: "max_rate" });
        } }));
};

# ---- variants with a test reference ---------------------------------------------------------------------------------------------------------
# A stationary reference 60 s ahead, level line at the present height above it (gate 300 ft above the reference), gate 500 m before it.
approach_tests.level_opts = func(ref, extra = nil) {
    var o = { ref: ref.path, gate_m: 500.0, gate_dh_ft: 300.0, ga_margin_ft: 1000 };
    if (extra != nil) foreach (var k; keys(extra)) o[k] = extra[k];
    return o;
};
approach_tests.v4_reference_lost = func {
    var st = fSim.fS.state;
    var r = approach_tests.make_ref(60.0 * approach_tests.v(), 0.0, st.hdg, 0.0, st.alt_ft - 300.0);
    approach_tests.go("v4: reference lost 20 s into the approach", approach_tests.level_opts(r, { course_deg: st.hdg }), r);
    approach_tests.after(20.0, func { r.lose(); });
};
# the reference moves away at 20 kt along the aircraft's heading; the course is relative to the reference's heading, so when it turns the line turns
approach_tests.v5_ship_turns = func {
    var st = fSim.fS.state;
    var r = approach_tests.make_ref(60.0 * approach_tests.v(), 0.0, st.hdg, 20.0, st.alt_ft - 300.0);
    approach_tests.go("v5: the reference turns 25 deg 25 s into the approach", approach_tests.level_opts(r, { course_deg: 0.0, course_rel: 1 }), r);
    approach_tests.after(25.0, func { r.turn(25.0); });
};
approach_tests.v6_not_established = func {
    var st = fSim.fS.state;
    var v = approach_tests.v();
    var r = approach_tests.make_ref(25.0 * v, 15.0 * v, st.hdg, 0.0, st.alt_ft - 300.0);
    # the line runs along the aircraft's heading through a reference 15 s of flight to the side and 25 s ahead: phase 0 refuses, force: 1 starts it anyway
    approach_tests.go("v6: forced start from a bad position", approach_tests.level_opts(r, { course_deg: st.hdg, force: 1 }), r);
};
approach_tests.v7_entry_speed = func {
    var st = fSim.fS.state;
    var r = approach_tests.make_ref(60.0 * approach_tests.v(), 0.0, st.hdg, 0.0, st.alt_ft - 300.0);
    # the dive wants at least 1.3 x the 1 g stall speed at the gate; the approach is asked to fly 1.25 x, the lowest it accepts
    var vs1 = fSim.v_stall_1g(st);
    approach_tests.go("v7: then = dive at 1.25 x stall speed", approach_tests.level_opts(r, { course_deg: st.hdg, speed_mps: 1.25 * vs1, then: "dive", then_opts: { dive_angle_deg: 45.0 } }), r);
};
approach_tests.v8_refused_preview = func {
    var st = fSim.fS.state;
    var v = approach_tests.v();
    print("TEST ===== v8a: reference behind the gate =====");
    var r1 = approach_tests.make_ref(-3.0 * v, 0.0, st.hdg, 0.0, st.alt_ft - 300.0);
    fSim.start_approach(st, { ref: r1.path, course_deg: st.hdg, gate_m: 500.0, gate_dh_ft: 300.0, preview: 1 });
    r1.stop();
    print("TEST ===== v8b: reference moving away faster than the aircraft flies =====");
    var r2 = approach_tests.make_ref(60.0 * v, 0.0, st.hdg, 1.2 * st.speed, st.alt_ft - 300.0);
    fSim.start_approach(st, { ref: r2.path, course_deg: 0.0, course_rel: 1, gate_m: 500.0, gate_dh_ft: 300.0, preview: 1 });
    r2.stop();
    print("TEST ===== v8c: a line that is too close to reach =====");
    var r3 = approach_tests.make_ref(500.0 + 6.0 * v, 0.0, st.hdg, 0.0, st.alt_ft - 300.0);
    fSim.start_approach(st, { ref: r3.path, course_deg: st.hdg, gate_m: 500.0, gate_dh_ft: 300.0, preview: 1 });
    r3.stop();
};

# ---- a faster moving object ------------------------------------------------------------------------------------------------------------------
# Join astern of it: the line runs along its heading, the gate is gate_m behind it and level with it, the approach speed is its speed plus 20 m/s
# (or your present speed if that is higher). Phase 0 takes the closing speed from the speed you are flying NOW, so be flying faster than the object
# before you start. path: an AI aircraft ("/ai/models/aircraft[0]"). Without a path a test reference 60 s ahead, flying at 70% of your speed, is used.
approach_tests.v9_fast_ref_join = func(path = nil) {
    var st = fSim.fS.state;
    var r = nil;
    if (path == nil) {
        r = approach_tests.make_ref(60.0 * approach_tests.v(), 0.0, st.hdg, 0.7 * st.speed, st.alt_ft);
        path = r.path;
    }
    var kt = getprop(path ~ "/velocities/true-airspeed-kt") or getprop(path ~ "/velocities/speed-kts") or 0.0;
    var v_ref = kt * _kt;
    var v_app = math.max(v_ref + 20.0, approach_tests.v());
    approach_tests.go(sprintf("v9: join astern of an object at %.0f kt, approach speed %.0f m/s", kt, v_app),
        { ref: path, course_deg: 0.0, course_rel: 1, gate_m: 600.0, gate_dh_ft: 0.0, speed_mps: v_app, ga_margin_ft: 1000 }, r);
};
