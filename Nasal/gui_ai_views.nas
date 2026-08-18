# Switch camera view to a specific AI model in the sim, using a dialog GUI to select the target and view slot.
# Assume view slot 101, 102, 103 are available for AI tracking. 
# The user can select an AI model from the /ai/models tree and assign it to a view slot with optional x/y/z offsets.

var show_ai_view_menu_in_memory = func {
    var dialog_name = "ai_view_selector";

    # Gather active AI paths
    var ai_paths = [];
    var models_root = props.globals.getNode("/ai/models");

    if (models_root != nil) {
        foreach (var category; ["aircraft", "ship", "static"]) {
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

    # Get call signs for each AI path for display in the combo box
    var ai_callSign = [];
    foreach (var path; ai_paths) {
        var callsign = bombable.getCallSign(path);
        # Fallback if callsign is completely missing
        if (callsign == nil or callsign == "") {
            callsign = "***";
        }
        append(ai_callSign, callsign);
    }

    # Set initial default properties BEFORE building/showing dialog
    var slot_prop   = "/sim/gui/dialogs/ai-view/selected-slot";
    var target_prop = "/sim/gui/dialogs/ai-view/selected-target";
    var x_off_prop  = "/sim/gui/dialogs/ai-view/x-offset-m";
    var y_off_prop  = "/sim/gui/dialogs/ai-view/y-offset-m";
    var z_off_prop  = "/sim/gui/dialogs/ai-view/z-offset-m";

    if (getprop(slot_prop) == nil) {
        setprop(slot_prop, "101");
    }

    # Populate current offset values from property tree based on selected slot
    var current_slot = num(getprop(slot_prop));
    var cur_x = getprop("/sim/view[" ~ current_slot ~ "]/config/x-offset-m");
    var cur_y = getprop("/sim/view[" ~ current_slot ~ "]/config/y-offset-m");
    var cur_z = getprop("/sim/view[" ~ current_slot ~ "]/config/z-offset-m");

    setprop(x_off_prop, (cur_x != nil) ? cur_x : 0.0);
    setprop(y_off_prop, (cur_y != nil) ? cur_y : 0.0);
    setprop(z_off_prop, (cur_z != nil) ? cur_z : 0.0);

    # Set default target selection
    setprop(target_prop, ai_callSign[0] ~ "  -  " ~ ai_paths[0]);

    # 1. Create root property node for the dialog
    var dlg_tree = props.Node.new();
    dlg_tree.getChild("name", 0, 1).setValue(dialog_name);
    dlg_tree.getChild("layout", 0, 1).setValue("vbox");

    # Text Header
    var header = dlg_tree.addChild("text");
    header.getChild("label", 0, 1).setValue("AI View Target Selector");

    dlg_tree.addChild("hrule");

    # Group container
    var main_group = dlg_tree.addChild("group");
    main_group.getChild("layout", 0, 1).setValue("vbox");

    # Slot & Offsets Combined Group (Single horizontal row)
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

    # X Offset input
    slot_grp.addChild("text").getChild("label", 0, 1).setValue(" X:");
    var x_input = slot_grp.addChild("input");
    x_input.getChild("property", 0, 1).setValue(x_off_prop);
    x_input.getChild("pref-width", 0, 1).setIntValue(50);

    # Y Offset input
    slot_grp.addChild("text").getChild("label", 0, 1).setValue(" Y:");
    var y_input = slot_grp.addChild("input");
    y_input.getChild("property", 0, 1).setValue(y_off_prop);
    y_input.getChild("pref-width", 0, 1).setIntValue(50);

    # Z Offset input
    slot_grp.addChild("text").getChild("label", 0, 1).setValue(" Z:");
    var z_input = slot_grp.addChild("input");
    z_input.getChild("property", 0, 1).setValue(z_off_prop);
    z_input.getChild("pref-width", 0, 1).setIntValue(50);

    # AI Target Combo Group
    var target_grp = main_group.addChild("group");
    target_grp.getChild("layout", 0, 1).setValue("hbox");
    target_grp.addChild("text").getChild("label", 0, 1).setValue("AI Target:");

    var target_combo = target_grp.addChild("combo");
    target_combo.getChild("property", 0, 1).setValue(target_prop);
    target_combo.getChild("pref-width", 0, 1).setIntValue(300);

    # Dynamically inject <value> child nodes for each AI path
    forindex (var i; ai_paths) {
        target_combo.addChild("value").setValue(ai_callSign[i] ~ "  -  " ~ ai_paths[i]);
    }

    # Binding to commit combo selection immediately to the property tree
    var target_binding = target_combo.addChild("binding");
    target_binding.getChild("command", 0, 1).setValue("dialog-apply");

    dlg_tree.addChild("hrule");

    # Buttons Group
    var btn_grp = dlg_tree.addChild("group");
    btn_grp.getChild("layout", 0, 1).setValue("hbox");

    var set_btn = btn_grp.addChild("button");
    set_btn.getChild("legend", 0, 1).setValue("Set & View Target");
    set_btn.getChild("pref-width", 0, 1).setIntValue(160);
    
    # 1. Apply dialog choices to properties
    var apply_binding = set_btn.addChild("binding");
    apply_binding.getChild("command", 0, 1).setValue("dialog-apply");
    
    # 2. Trigger view slot assignment logic
    var script_binding = set_btn.addChild("binding");
    script_binding.getChild("command", 0, 1).setValue("nasal");
    script_binding.getChild("script", 0, 1).setValue("gui_ai_views.apply_selected_ai_view();");

    var close_btn = btn_grp.addChild("button");
    close_btn.getChild("legend", 0, 1).setValue("Close");
    close_btn.getChild("pref-width", 0, 1).setIntValue(80);
    var close_binding = close_btn.addChild("binding");
    close_binding.getChild("command", 0, 1).setValue("dialog-close");

    # 2. Instantiate dialog
    fgcommand("dialog-close", props.Node.new({ "dialog-name": dialog_name }));
    fgcommand("dialog-new", dlg_tree);
    fgcommand("dialog-show", props.Node.new({ "dialog-name": dialog_name }));
};

var apply_selected_ai_view = func {
    # Read selection from property tree
    var selection = getprop("/sim/gui/dialogs/ai-view/selected-target");
    var ai_path   = substr(selection, find("  -  ", selection) + 5);
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

    # Fetch offset values from dialog GUI inputs
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

# Set up target view slot to track an AI path and apply offsets
var select_ai_view_slot = func(slot_idx, ai_path, x_off, y_off, z_off) {
    if (ai_path == nil or ai_path == "") return;

    # Record selection
    setprop("/sim/views/ai-targets/slot-" ~ slot_idx, ai_path);

    # Apply x, y, z offset positions to configured slot
    setprop("/sim/view[" ~ slot_idx ~ "]/config/x-offset-m", x_off);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/y-offset-m", y_off);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/z-offset-m", z_off);

    # Zero these view controls so that no pilot offset is applied
    setprop("/sim/view[" ~ slot_idx ~ "]/config/eye-lat-deg", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/eye-lon-deg", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/eye-alt-ft", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/target-y-offset-m", 0);
    setprop("/sim/view[" ~ slot_idx ~ "]/config/target-z-offset-m", 0);


    # Switch view slot
    view.setViewByIndex(slot_idx);

    # Defer root path assignment past FGView::init() reset cycle
    settimer(func {
        setprop("/sim/view[" ~ slot_idx ~ "]/config/root", ai_path);
    }, 0.05);
};