var show_ai_view_menu_in_memory = func {
    var dialog_name = "ai_view_selector";

    # Gather active AI paths
    var ai_paths = [];
    var models_root = props.globals.getNode("/ai/models");

    if (models_root != nil) {
        foreach (var category; ["aircraft", "ship", "multiplayer", "ballistic"]) {
            var children = models_root.getChildren(category);
            foreach (var child; children) {
                var valid_node = child.getNode("valid");
                if (valid_node == nil or valid_node.getBoolValue()) {
                    append(ai_paths, child.getPath());
                }
            }
        }
    }

    if (size(ai_paths) == 0) {
        gui.popupTip("No active AI models found under /ai/models/");
        return;
    }

    # Set initial default properties BEFORE building/showing dialog
    var slot_prop = "/sim/gui/dialogs/ai-view/selected-slot";
    var target_prop = "/sim/gui/dialogs/ai-view/selected-target";

    if (getprop(slot_prop) == nil) {
        setprop(slot_prop, "101");
    }

    # Set default target selection
    setprop(target_prop, ai_paths[0]);

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

    # Slot Combo Group
    var slot_grp = main_group.addChild("group");
    slot_grp.getChild("layout", 0, 1).setValue("hbox");
    slot_grp.addChild("text").getChild("label", 0, 1).setValue("View Slot:");
    
    var slot_combo = slot_grp.addChild("combo");
    slot_combo.getChild("property", 0, 1).setValue(slot_prop);
    slot_combo.addChild("value").setValue("101");
    slot_combo.addChild("value").setValue("102");
    slot_combo.addChild("value").setValue("103");
    
    var slot_binding = slot_combo.addChild("binding");
    slot_binding.getChild("command", 0, 1).setValue("dialog-apply");

    # AI Target Combo Group
    var target_grp = main_group.addChild("group");
    target_grp.getChild("layout", 0, 1).setValue("hbox");
    target_grp.addChild("text").getChild("label", 0, 1).setValue("AI Target:");

    var target_combo = target_grp.addChild("combo");
    target_combo.getChild("property", 0, 1).setValue(target_prop);
    target_combo.getChild("pref-width", 0, 1).setIntValue(300);

    # Dynamically inject <value> child nodes for each AI path
    foreach (var path; ai_paths) {
        target_combo.addChild("value").setValue(path);
    }

    # Binding to commit combo selection immediately to the property tree
    var target_binding = target_combo.addChild("binding");
    target_binding.getChild("command", 0, 1).setValue("dialog-apply");

    dlg_tree.addChild("hrule");

    # Buttons Group
    var btn_grp = dlg_tree.addChild("group");
    btn_grp.getChild("layout", 0, 1).setValue("hbox");

    var set_btn = btn_grp.addChild("button");
    set_btn.getChild("legend", 0, 1).setValue("Set & View Target");  # <-- Changed from 'label' to 'legend'
    set_btn.getChild("pref-width", 0, 1).setIntValue(160);            # Ensures enough width for text
    
    # 1. Apply dialog choices to properties
    var apply_binding = set_btn.addChild("binding");
    apply_binding.getChild("command", 0, 1).setValue("dialog-apply");
    
    # 2. Trigger view slot assignment logic
    var script_binding = set_btn.addChild("binding");
    script_binding.getChild("command", 0, 1).setValue("nasal");
    script_binding.getChild("script", 0, 1).setValue("gui_ai_views.apply_selected_ai_view();");

    var close_btn = btn_grp.addChild("button");
    close_btn.getChild("legend", 0, 1).setValue("Close");            # <-- Changed from 'label' to 'legend'
    close_btn.getChild("pref-width", 0, 1).setIntValue(80);
    var close_binding = close_btn.addChild("binding");
    close_binding.getChild("command", 0, 1).setValue("dialog-close");

    # 2. Instantiate dialog
    fgcommand("dialog-close", props.Node.new({ "dialog-name": dialog_name }));
    fgcommand("dialog-new", dlg_tree);
    fgcommand("dialog-show", props.Node.new({ "dialog-name": dialog_name }));
};



var apply_selected_ai_view = func {
    # Read selection from the value property
    var ai_path  = getprop("/sim/gui/dialogs/ai-view/selected-target");
    var slot_str = getprop("/sim/gui/dialogs/ai-view/selected-slot");

    if (slot_str == nil or ai_path == nil or ai_path == "") {
        gui.popupTip("Error: Invalid slot or AI target selection.");
        return;
    }

    var slot_parts = split(" ", slot_str);
    var slot_idx = num(slot_parts[size(slot_parts) - 1]);

    if (slot_idx == nil) {
        gui.popupTip("Error: Failed to parse view slot index.");
        return;
    }

    select_ai_view_slot(slot_idx, ai_path);
    gui.popupTip("Assigned " ~ ai_path ~ " to Slot " ~ slot_idx, 3);
};

# Set up target view slot to track an AI path
var select_ai_view_slot = func(slot_idx, ai_path) {
    if (ai_path == nil or ai_path == "") return;

    # Record selection
    setprop("/sim/views/ai-targets/slot-" ~ slot_idx, ai_path);

    # Switch view slot
    view.setViewByIndex(slot_idx);

    # Defer root path assignment past FGView::init() reset cycle
    settimer(func {
        setprop("/sim/view[" ~ slot_idx ~ "]/config/root", ai_path);
    }, 0.05);
};


