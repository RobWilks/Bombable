# Custom Animation Manager integrated into Bombable
var ObjectManager = {

    # 1. Register and initialize animation controls on a Bombable object
    register: func(myNodeName) {
        var bObj = bombable.object[myNodeName];
        if (bObj == nil) {
            print("ObjectManager: Target " ~ myNodeName ~ " not found in bombable.object");
            return;
        }

        # Initialize controls.animation sub-hash on the Bombable object
        bObj.controls = bObj.controls or {};
        bObj.controls.animation = {
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
            speed_kts: 100.0,
            heading_deg: 0.0,
            
            # Timer instance pointer
            timer: nil
        };

        # Bind myNodeName to updateObject using a Nasal closure
        var animData = bObj.controls.animation;
        var callback = func { me.updateObject(myNodeName); };
        
        # Frame-rate synchronized timer (0.0 sec interval)
        animData.timer = maketimer(0.0, callback);
    },

    # 2. Unified per-frame update routine
    updateObject: func(myNodeName) {
        var bObj = bombable.object[myNodeName];
        if (bObj == nil) return;

        # Safety check: stop animation if object is damaged, dead, or inactive
        if (bObj.isDead or bObj.health <= 0) {
            me.stopObject(myNodeName);
            return;
        }

        var anim = bObj.controls.animation;
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
        var bObj = bombable.object[myNodeName];
        if (bObj != nil and bObj.controls != nil and bObj.controls.animation != nil) {
            bObj.controls.animation.isActive = true;
            bObj.controls.animation.timer.start();
        }
    },

    stopObject: func(myNodeName) {
        var bObj = bombable.object[myNodeName];
        if (bObj != nil and bObj.controls != nil and bObj.controls.animation != nil) {
            bObj.controls.animation.isActive = false;
            bObj.controls.animation.timer.stop();
        }
    },

    startAll: func {
        foreach (var myNodeName; keys(bombable.object)) {
            me.startObject(myNodeName);
        }
    },

    stopAll: func {
        foreach (var myNodeName; keys(bombable.object)) {
            me.stopObject(myNodeName);
        }
    },

    # Teardown and deallocate animation pointers for an object
    clearObject: func(myNodeName) {
        me.stopObject(myNodeName);
        var bObj = bombable.object[myNodeName];
        if (bObj != nil and bObj.controls != nil) {
            bObj.controls.animation = nil;
        }
    },

    clearAll: func {
        foreach (var myNodeName; keys(bombable.object)) {
            me.clearObject(myNodeName);
        }
    }
};




# Example: Spawn/Initialize Bombable scenario objects
var initScenario = func {
    # Assuming Bombable has populated bombable.object["ai/models/static[0]"]
    var targetName = "ai/models/static[0]";

    # Register animation framework onto the target
    ObjectManager.register(targetName);

    # Set custom target speed
    bombable.object[targetName].controls.animation.speed_kts = 150.0;

    # Start loop
    ObjectManager.startObject(targetName);
};

# End of Epoch / Reset
var endScenarioEpoch = func {
    ObjectManager.stopAll();
    ObjectManager.clearAll();
};