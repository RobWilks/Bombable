# gui_ai_views_new.nas
# Handles dynamic target smoothing and menu generation for custom AI view slots.

# Global hash store for smoother timer instances per slot
var active_smoothers = {};

# Helper to completely halt and remove an active smoother instance for a slot
var stop_slot_smoother = func(slot_idx) {
    if (contains(active_smoothers, slot_idx) and active_smoothers[slot_idx] != nil) {
        active_smoothers[slot_idx].stop();
        active_smoothers[slot_idx] = nil;
    }
};

# Ensure proxy property nodes exist before FGView binds to them
var ensure_proxy_nodes = func(proxy_path) {
    var out_node = props.globals.getNode(proxy_path, 1);
    var pos = out_node.getChild("position", 0, 1);
    var ori = out_node.getChild("orientation", 0, 1);

    if (pos.getChild("latitude-deg", 0, 0) == nil) pos.getChild("latitude-deg", 0, 1).setDoubleValue(0.0);
    if (pos.getChild("longitude-deg", 0, 0) == nil) pos.getChild("longitude-deg", 0, 1).setDoubleValue(0.0);
    if (pos.getChild("altitude-ft", 0, 0) == nil) pos.getChild("altitude-ft", 0, 1).setDoubleValue(0.0);

    if (ori.getChild("heading-deg", 0, 0) == nil) ori.getChild("heading-deg", 0, 1).setDoubleValue(0.0);
    if (ori.getChild("true-heading-deg", 0, 0) == nil) ori.getChild("true-heading-deg", 0, 1).setDoubleValue(0.0);
    if (ori.getChild("pitch-deg", 0, 0) == nil) ori.getChild("pitch-deg", 0, 1).setDoubleValue(0.0);
    if (ori.getChild("roll-deg", 0, 0) == nil) ori.getChild("roll-deg", 0, 1).setDoubleValue(0.0);
};

# Hermite/Catmull-Rom Smoother Controller
var start_slot_smoother = func(slot_idx, raw_prop, proxy_path, delay_sec) {
    stop_slot_smoother(slot_idx);
    ensure_proxy_nodes(proxy_path);

    var smoother = {
        raw_prop: raw_prop,
        proxy_path: proxy_path,
        delay_sec: delay_sec,
        buffer: [],
        timer: nil,

        start: func {
            me.buffer = [];
            me.timer = maketimer(0, func me.update());
            me.timer.start();
        },

        stop: func {
            if (me.timer != nil) me.timer.stop();
        },

        update: func {
            var path = getprop(me.raw_prop);
            if (path == nil or path == "") return;

            var target_node = props.globals.getNode(path);
            if (target_node == nil) return;

            var now = getprop("/sim/time/elapsed-sec");

            var lat_n = target_node.getNode("position/latitude-deg");
            var lon_n = target_node.getNode("position/longitude-deg");
            var alt_n = target_node.getNode("position/altitude-ft");

            var hdg_n = target_node.getNode("orientation/true-heading-deg");
            if (hdg_n == nil) hdg_n = target_node.getNode("orientation/heading-deg");
            var pitch_n = target_node.getNode("orientation/pitch-deg");

            var current_sample = {
                time: now,
                lat:   (lat_n != nil) ? lat_n.getValue() : 0.0,
                lon:   (lon_n != nil) ? lon_n.getValue() : 0.0,
                alt:   (alt_n != nil) ? alt_n.getValue() : 0.0,
                hdg:   (hdg_n != nil) ? hdg_n.getValue() : 0.0,
                pitch: (pitch_n != nil) ? pitch_n.getValue() : 0.0
            };

            append(me.buffer, current_sample);

            while (size(me.buffer) > 4 and (now - me.buffer[0].time) > (me.delay_sec + 0.5)) {
                me.buffer = me.buffer[1:];
            }

            if (size(me.buffer) < 4) {
                me.write_output(current_sample);
                return;
            }

            var render_time = now - me.delay_sec;

            var idx = -1;
            for (var i = 1; i < size(me.buffer) - 2; i += 1) {
                if (me.buffer[i].time <= render_time and render_time <= me.buffer[i+1].time) {
                    idx = i;
                    break;
                }
            }

            if (idx == -1) {
                me.write_output(me.buffer[size(me.buffer) - 1]);
                return;
            }

            var p0 = me.buffer[idx - 1];
            var p1 = me.buffer[idx];
            var p2 = me.buffer[idx + 1];
            var p3 = me.buffer[idx + 2];

            var dt = p2.time - p1.time;
            if (dt <= 0.0001) return;

            var t = (render_time - p1.time) / dt;
            var t2 = t * t;
            var t3 = t2 * t;

            var h00 = 2*t3 - 3*t2 + 1;
            var h10 = t3 - 2*t2 + t;
            var h01 = -2*t3 + 3*t2;
            var h11 = t3 - t2;

            var v1_lat = (p2.lat - p0.lat) / (p2.time - p0.time);
            var v2_lat = (p3.lat - p1.lat) / (p3.time - p1.time);

            var v1_lon = (p2.lon - p0.lon) / (p2.time - p0.time);
            var v2_lon = (p3.lon - p1.lon) / (p3.time - p1.time);

            var v1_alt = (p2.alt - p0.alt) / (p2.time - p0.time);
            var v2_alt = (p3.alt - p1.alt) / (p3.time - p1.time);

            var d_hdg_p2_p1 = math.mod(p2.hdg - p1.hdg + 540, 360) - 180;
            var d_hdg_p2_p0 = math.mod(p2.hdg - p0.hdg + 540, 360) - 180;
            var d_hdg_p3_p1 = math.mod(p3.hdg - p1.hdg + 540, 360) - 180;

            var v1_hdg = d_hdg_p2_p0 / (p2.time - p0.time);
            var v2_hdg = d_hdg_p3_p1 / (p3.time - p1.time);

            var interp_state = {
                lat:   h00 * p1.lat + h10 * dt * v1_lat + h01 * p2.lat + h11 * dt * v2_lat,
                lon:   h00 * p1.lon + h10 * dt * v1_lon + h01 * p2.lon + h11 * dt * v2_lon,
                alt:   h00 * p1.alt + h10 * dt * v1_alt + h01 * p2.alt + h11 * dt * v2_alt,
                hdg:   p1.hdg + (h00 * 0 + h10 * dt * v1_hdg + h01 * d_hdg_p2_p1 + h11 * dt * v2_hdg),
                pitch: h00 * p1.pitch + h10 * dt * ((p2.pitch - p0.pitch)/(p2.time - p0.time)) + 
                       h01 * p2.pitch + h11 * dt * ((p3.pitch - p1.pitch)/(p3.time - p1.time))
            };

            me.write_output(interp_state);
        },

        write_output: func(state) {
            var out_node = props.globals.getNode(me.proxy_path, 1);
            var pos = out_node.getChild("position", 0, 1);
            var ori = out_node.getChild("orientation", 0, 1);

            pos.getChild("latitude-deg", 0, 1).setDoubleValue(state.lat);
            pos.getChild("longitude-deg", 0, 1).setDoubleValue(state.lon);
            pos.getChild("altitude-ft", 0, 1).setDoubleValue(state.alt);

            ori.getChild("heading-deg", 0, 1).setDoubleValue(state.hdg);
            ori.getChild("true-heading-deg", 0, 1).setDoubleValue(state.hdg);
            ori.getChild("pitch-deg", 0, 1).setDoubleValue(state.pitch);
        }
    };

    smoother.start();
    active_smoothers[slot_idx] = smoother;
};

var select_ai_view_slot = func(slot_idx, ai_path, x_off, y_off, z_off) {
    if (ai_path == nil or ai_path == "") return;

    var raw_slot_prop = "/sim/views/ai-targets/slot-" ~ slot_idx;
    var smooth_proxy_path = "/sim/views/ai-targets/smooth-slot-" ~ slot_idx;

    setprop(raw_slot_prop, ai_path);

    setprop("/sim/view[" ~ slot_idx ~ "]/config/x-offset-m", x_off);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/y-offset-m", y_off);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/z-offset-m", z_off);

    setprop("/sim/view[" ~ slot_idx ~ "]/config/eye-lat-deg", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/eye-lon-deg", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/eye-alt-ft", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/target-y-offset-m", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/target-z-offset-m", 0);

    var smooth_enabled = getprop("/sim/gui/dialogs/ai-view/smoothing-enabled");
    if (smooth_enabled == nil) smooth_enabled = 1;

    # Conditional Execution: Only spawn timer loop if smoothing is active
    if (smooth_enabled) {
        start_slot_smoother(slot_idx, raw_slot_prop, smooth_proxy_path, .25);
    } else {
        stop_slot_smoother(slot_idx);
    }

    # Switch active camera view
    view.setViewByIndex(slot_idx);

    # Defer root path assignment past FGView::init() reset cycle
    settimer(func {
        var target_root = (smooth_enabled) ? smooth_proxy_path : ai_path;
        setprop("/sim/view[" ~ slot_idx ~ "]/config/root", target_root);
    }, 0.05);
};

# Toggle tracking root path between raw AI path and smoothed proxy node
var toggle_smoothing = func {
    var slot_str = getprop("/sim/gui/dialogs/ai-view/selected-slot");
    if (slot_str == nil) return;

    var slot_parts = split(" ", slot_str);
    var slot_idx   = num(slot_parts[size(slot_parts) - 1]);
    if (slot_idx == nil) return;

    var raw_path = getprop("/sim/views/ai-targets/slot-" ~ slot_idx);
    if (raw_path == nil or raw_path == "") return;

    var smooth_enabled = getprop("/sim/gui/dialogs/ai-view/smoothing-enabled");
    var raw_slot_prop = "/sim/views/ai-targets/slot-" ~ slot_idx;
    var smooth_proxy_path = "/sim/views/ai-targets/smooth-slot-" ~ slot_idx;

    # Manage calculation loop lifecycle based on toggle state
    if (smooth_enabled) {
        start_slot_smoother(slot_idx, raw_slot_prop, smooth_proxy_path, 0.25);
    } else {
        stop_slot_smoother(slot_idx);
    }

    # Defer root path update to allow view update loop to conclude clean frame
    settimer(func {
        var target_root = (smooth_enabled) ? smooth_proxy_path : raw_path;
        setprop("/sim/view[" ~ slot_idx ~ "]/config/root", target_root);

        var mode_str = (smooth_enabled) ? "Enabled (Hermite)" : "Disabled (Raw AI)";
        gui.popupTip("Slot " ~ slot_idx ~ " Smoothing: " ~ mode_str, 2);
    }, 0.05);
};

var apply_selected_ai_view = func {
    var selection = getprop("/sim/gui/dialogs/ai-view/selected-target");
    if (selection == nil) return;

    var idx_sep = find("  -  ", selection);
    var ai_path = (idx_sep != -1) ? substr(selection, idx_sep + 5) : selection;
    var slot_str  = getprop("/sim/gui/dialogs/ai-view/selected-slot");

    if (slot_str == nil or ai_path == nil or ai_path == "") {
        gui.popupTip("Error: Invalid slot or AI target selection.");
        return;
    }

    var slot_parts = split(" ", slot_str);
    var slot_idx   = num(slot_parts[size(slot_parts) - 1]);

    if (slot_idx == nil) {
        gui.popupTip("Error: Failed to parse view slot index.");
        return;
    }

    var x_off = num(getprop("/sim/gui/dialogs/ai-view/x-offset-m"));
    var y_off = num(getprop("/sim/gui/dialogs/ai-view/y-offset-m"));
    var z_off = num(getprop("/sim/gui/dialogs/ai-view/z-offset-m"));

    select_ai_view_slot(
        slot_idx, 
        ai_path, 
        (x_off != nil) ? x_off : 0.0, 
        (y_off != nil) ? y_off : 0.0, 
        (z_off != nil) ? z_off : 0.0
    );

    gui.popupTip("Assigned " ~ ai_path ~ " to Slot " ~ slot_idx, 3);
};

var show_ai_view_menu_in_memory = func {
    var dialog_name = "ai_view_selector";

    var ai_paths = [];
    var models_root = props.globals.getNode("/ai/models");

    if (models_root != nil) {
        foreach (var category; ["aircraft", "ship", "static", "multiplayer"]) {
            var children = models_root.getChildren(category);
            foreach (var child; children) {
                var valid_node = child.getNode("valid");
                var lat_node = child.getNode("position/latitude-deg");
                var lat = (lat_node != nil) ? lat_node.getValue() : 0;

                if ((valid_node == nil or valid_node.getBoolValue()) and lat != nil and lat != 0) {
                    append(ai_paths, child.getPath());
                }
            }
        }
    }

    if (size(ai_paths) == 0) {
        gui.popupTip("No active AI models found under /ai/models/");
        return;
    }

    var ai_callSign = [];
    foreach (var path; ai_paths) {
        var callsign = nil;
        if (contains(globals, "bombable") and contains(bombable, "getCallSign")) {
            callsign = bombable.getCallSign(path);
        }
        if (callsign == nil or callsign == "") {
            callsign = "***";
        }
        append(ai_callSign, callsign);
    }

    var slot_prop   = "/sim/gui/dialogs/ai-view/selected-slot";
    var target_prop = "/sim/gui/dialogs/ai-view/selected-target";
    var x_off_prop  = "/sim/gui/dialogs/ai-view/x-offset-m";
    var y_off_prop  = "/sim/gui/dialogs/ai-view/y-offset-m";
    var z_off_prop  = "/sim/gui/dialogs/ai-view/z-offset-m";
    var smooth_prop = "/sim/gui/dialogs/ai-view/smoothing-enabled";

    if (getprop(slot_prop) == nil) setprop(slot_prop, "101");
    if (getprop(smooth_prop) == nil) setprop(smooth_prop, 1);

    var current_slot = num(getprop(slot_prop));
    if (current_slot != nil) {
        var cur_x = getprop("/sim/view[" ~ current_slot ~ "]/config/x-offset-m");
        var cur_y = getprop("/sim/view[" ~ current_slot ~ "]/config/y-offset-m");
        var cur_z = getprop("/sim/view[" ~ current_slot ~ "]/config/z-offset-m");

        setprop(x_off_prop, (cur_x != nil) ? cur_x : 0.0);
        setprop(y_off_prop, (cur_y != nil) ? cur_y : 0.0);
        setprop(z_off_prop, (cur_z != nil) ? cur_z : 0.0);
    }

    setprop(target_prop, ai_callSign[0] ~ "  -  " ~ ai_paths[0]);

    var dlg_tree = props.Node.new();
    dlg_tree.getChild("name", 0, 1).setValue(dialog_name);
    dlg_tree.getChild("layout", 0, 1).setValue("vbox");

    var header = dlg_tree.addChild("text");
    header.getChild("label", 0, 1).setValue("AI View Target Selector");

    dlg_tree.addChild("hrule");

    var main_group = dlg_tree.addChild("group");
    main_group.getChild("layout", 0, 1).setValue("vbox");

    # Slot & Offsets controls
    var slot_grp = main_group.addChild("group");
    slot_grp.getChild("layout", 0, 1).setValue("hbox");
    
    slot_grp.addChild("text").getChild("label", 0, 1).setValue("View Slot:");
    var slot_combo = slot_grp.addChild("combo");
    slot_combo.getChild("property", 0, 1).setValue(slot_prop);
    slot_combo.getChild("pref-width", 0, 1).setIntValue(60);
    slot_combo.addChild("value").setValue("101");
    slot_combo.addChild("value").setValue("102");
    slot_combo.addChild("value").setValue("103");
    
    var slot_binding = slot_combo.addChild("binding");
    slot_binding.getChild("command", 0, 1).setValue("dialog-apply");

    slot_grp.addChild("text").getChild("label", 0, 1).setValue(" X:");
    var x_input = slot_grp.addChild("input");
    x_input.getChild("property", 0, 1).setValue(x_off_prop);
    x_input.getChild("pref-width", 0, 1).setIntValue(50);

    slot_grp.addChild("text").getChild("label", 0, 1).setValue(" Y:");
    var y_input = slot_grp.addChild("input");
    y_input.getChild("property", 0, 1).setValue(y_off_prop);
    y_input.getChild("pref-width", 0, 1).setIntValue(50);

    slot_grp.addChild("text").getChild("label", 0, 1).setValue(" Z:");
    var z_input = slot_grp.addChild("input");
    z_input.getChild("property", 0, 1).setValue(z_off_prop);
    z_input.getChild("pref-width", 0, 1).setIntValue(50);

    # Smoothing Toggle Option
    var smooth_grp = main_group.addChild("group");
    smooth_grp.getChild("layout", 0, 1).setValue("hbox");
    
    var smooth_chk = smooth_grp.addChild("checkbox");
    smooth_chk.getChild("property", 0, 1).setValue(smooth_prop);
    smooth_chk.getChild("label", 0, 1).setValue(" Enable Catmull-Rom Smoothing");

    var chk_apply = smooth_chk.addChild("binding");
    chk_apply.getChild("command", 0, 1).setValue("dialog-apply");

    var chk_script = smooth_chk.addChild("binding");
    chk_script.getChild("command", 0, 1).setValue("nasal");
    chk_script.getChild("script", 0, 1).setValue("gui_ai_views.toggle_smoothing();");

    # AI Target Combo
    var target_grp = main_group.addChild("group");
    target_grp.getChild("layout", 0, 1).setValue("hbox");
    target_grp.addChild("text").getChild("label", 0, 1).setValue("AI Target:");

    var target_combo = target_grp.addChild("combo");
    target_combo.getChild("property", 0, 1).setValue(target_prop);
    target_combo.getChild("pref-width", 0, 1).setIntValue(300);

    forindex (var i; ai_paths) {
        target_combo.addChild("value").setValue(ai_callSign[i] ~ "  -  " ~ ai_paths[i]);
    }

    var target_binding = target_combo.addChild("binding");
    target_binding.getChild("command", 0, 1).setValue("dialog-apply");

    dlg_tree.addChild("hrule");

    var btn_grp = dlg_tree.addChild("group");
    btn_grp.getChild("layout", 0, 1).setValue("hbox");

    var set_btn = btn_grp.addChild("button");
    set_btn.getChild("legend", 0, 1).setValue("Set & View Target");
    set_btn.getChild("pref-width", 0, 1).setIntValue(160);
    
    var apply_binding = set_btn.addChild("binding");
    apply_binding.getChild("command", 0, 1).setValue("dialog-apply");
    
    var script_binding = set_btn.addChild("binding");
    script_binding.getChild("command", 0, 1).setValue("nasal");
    script_binding.getChild("script", 0, 1).setValue("gui_ai_views.apply_selected_ai_view();");

    var close_btn = btn_grp.addChild("button");
    close_btn.getChild("legend", 0, 1).setValue("Close");
    close_btn.getChild("pref-width", 0, 1).setIntValue(80);
    var close_binding = close_btn.addChild("binding");
    close_binding.getChild("command", 0, 1).setValue("dialog-close");

    fgcommand("dialog-close", props.Node.new({ "dialog-name": dialog_name }));
    fgcommand("dialog-new", dlg_tree);
    fgcommand("dialog-show", props.Node.new({ "dialog-name": dialog_name }));
};