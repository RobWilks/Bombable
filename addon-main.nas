#
# Bombable addon
#
# Started by Brent Hugh
# Started in 2009
#
# Converted to a FlightGear addon by
# Brendan Black, Feb 2021

var main = func( addon ) {
    var root = addon.basePath;
    var myAddonId  = addon.id;
    var mySettingsRootPath = "/addons/by-id/" ~ myAddonId;
    # setting root path to addon
    setprop("/sim/bombable/root_path", root);

    # Scripts to load: [filename, namespace]
    var scripts = [
        ["fast_trig.nas",     "fast_trig"],
        ["gui_ai_views.nas",  "gui_ai_views"],
        ["bombable.nas",      "bombable"]
    ];

    # Load scripts sequentially on addon init
    foreach(var script; scripts) {
        var file_path = root ~ "/Nasal/" ~ script[0];
        var namespace = script[1];
        io.load_nasal(file_path, namespace);
    }
}