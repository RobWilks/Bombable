# Custom Animation Manager integrated into Bombable
var animationManager = {

    # 1. Register and initialize animation controls on a Bombable object
    register: func(myNodeName) {
        var ats = bombable.attributes[myNodeName];
        if (ats == nil) {
            print("animationManager: Target " ~ myNodeName ~ " not found in bombable.attributes");
            return;
        }

        # Initialize controls.animation sub-hash on the Bombable object
        ats.controls = ats.controls or {};
        ats.controls.animation = {
            # Active status flag
            isActive: false,

            # Pre-cached property nodes for high-frequency execution
            latNode: props.globals.getNode(myNodeName ~ "/position/latitude-deg", 1),
            lonNode: props.globals.getNode(myNodeName ~ "/position/longitude-deg", 1),
            altNode: props.globals.getNode(myNodeName ~ "/position/altitude-ft", 1),
            hdgNode: props.globals.getNode(myNodeName ~ "/orientation/heading-deg", 1),
            pitchNode: props.globals.getNode(myNodeName ~ "/orientation/pitch-deg", 1),
            rollNode: props.globals.getNode(myNodeName ~ "/orientation/roll-deg", 1),

            # Kinematic / State variables
            dLat: 0.0,
            dLon: 0.0,
            dAlt: 0.0, # ft
            dHdg: 0.0,
            dPitch: 0.0,
            dRoll: 0.0,
            
            # Timer instance pointer
            timer: nil
        };

        # Bind myNodeName to updateObject using a Nasal closure
        var animData = ats.controls.animation;
        var callback = func { me.updateObject(myNodeName); };
        
        # Frame-rate synchronized timer (0.0 sec interval)
        animData.timer = maketimer(0.0, callback);
    },

    # 2. Unified per-frame update routine
    updateObject: func(myNodeName) {
        var ats = bombable.attributes[myNodeName];
        if (ats == nil) return;

        # Safety check: stop animation if object is damaged, dead, or inactive
        if (ats.damage == 1) {
            me.stopObject(myNodeName);
            return;
        }

        var anim = ats.controls.animation;
        if (anim == nil or !anim.isActive) return;

        var dt = getprop("/sim/time/delta-sec") or 0.016;

        # Fetch current position
        var lat = anim.latNode.getValue() or 0.0;
        var lon = anim.lonNode.getValue() or 0.0;

        # --- KINEMATICS / ANIMATION LOGIC ---
        # Example displacement calculation:
        var dist_nm = (anim.speed_kts / 3600.0) * dt;
        var delta_lat = dist_nm / 60.0; # 1 NM ≈ 1/60th deg latitude

        # Update cached property nodes directly
        anim.latNode.setDoubleValue(lat + delta_lat);
        # -----------------------------------
    },

    # 3. Control Operations
    startObject: func(myNodeName) {
        var ats = bombable.attributes[myNodeName];
        if (ats != nil and ats.controls != nil and ats.controls.animation != nil) {
            ats.controls.animation.isActive = true;
            ats.controls.animation.timer.start();
        }
    },

    stopObject: func(myNodeName) {
        var ats = bombable.attributes[myNodeName];
        if (ats != nil and ats.controls != nil and ats.controls.animation != nil) {
            ats.controls.animation.isActive = false;
            ats.controls.animation.timer.stop();
        }
    },

    startAll: func {
        foreach (var myNodeName; keys(bombable.attributes)) {
            me.startObject(myNodeName);
        }
    },

    stopAll: func {
        foreach (var myNodeName; keys(bombable.attributes)) {
            me.stopObject(myNodeName);
        }
    },

    # Teardown and deallocate animation pointers for an object
    clearObject: func(myNodeName) {
        me.stopObject(myNodeName);
        var ats = bombable.attributes[myNodeName];
        if (ats != nil and ats.controls != nil) {
            ats.controls.animation = nil;
        }
    },

    clearAll: func {
        foreach (var myNodeName; keys(bombable.attributes)) {
            me.clearObject(myNodeName);
        }
    }
};




# Example: Spawn/Initialize Bombable scenario objects
var initScenario = func {
    # Assuming Bombable has populated bombable.attributes["ai/models/static[0]"]
    var targetName = "ai/models/static[0]";

    # Register animation framework onto the target
    animationManager.register(targetName);

    # Set custom target dLon, dLat etc.
    bombable.attributes[targetName].controls.animation.dLat = 1e-5;

    # Start loop
    animationManager.startObject(targetName);
};

# End of Epoch / Reset
var endScenarioEpoch = func {
    animationManager.stopAll();
    animationManager.clearAll();
};


