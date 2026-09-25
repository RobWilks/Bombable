# Set your property
setprop("/sim/ai/scenario", "BOMB-MarinCountyNineA6M5ThreeB17");

# Tell FlightGear to save this property to autosave.xml on exit
var node = props.globals.getNode("/sim/ai/scenario", 1);
node.setAttribute("archive", 1);

debug.dump(teams);

debug.dump(bombable.targetData);

var myNodeName1 = "/ai/models/aircraft[1]";
var ats = bombable.attributes[myNodeName1];
debug.dump(ats.weapons);
debug.dump(ats.evasions);
debug.dump(ats.attacks);

debug.dump(ats.index);
debug.dump(ats.shooterIndex);
debug.dump(ats.targetIndex);

debug.dump(bombable.bombableMenu);

bombable.dodge("/ai/models/aircraft", 1, 1);
bombable.dodge("/ai/models/aircraft[1]", 1, 1);
bombable.dodge("/ai/models/aircraft[2]", 1, 1);



print(fast_trig.sin(math.pi/6));
print(math.sin(math.pi / 6));

var myNodeName1 = "/ai/models/aircraft";
var ats = bombable.attributes[myNodeName1];
debug.dump(ats.loopids);

var myNodeName1 = "/ai/models/aircraft[4]";
var ats = bombable.attributes[myNodeName1];
debug.dump(ats.loopids);

print(bombable.un_variable_safe(""));

var dumpLoopId = func(nodeName) {
		var ats = bombable.attributes[nodeName];
        print(nodeName);
		# Iterate over all dynamic loop keys present in ats
        var s = "";
		foreach (var raw_key; keys(ats.loopids))
		{
           s = s ~ raw_key ~ ", "; 
        }
        print(s);
}
var myNodeName1 = "/ai/models/aircraft";
dumpLoopId(myNodeName1);
dumpLoopId("/ai/models/aircraft[4]");
dumpLoopId("");

debug.dump(bombable.nodeNames);

print(bombable.bombable_epoch);
bombable.resetTerrainFires();
print(bombable.bombable_epoch);



setprop("/sim/presets/elevation-ft", 1000);
var myNodeName = "/ai/models/aircraft";
            var distHdg = bombable.courseToAirport(myNodeName); # returns a hash or nil
        if (distHdg != nil) {
            var dist = distHdg.distance;
            var courseToTarget_deg = distHdg.heading; # absolute bearing
            
            # Fix: closing parenthesis on getprop
            var oldHdg = getprop("" ~ myNodeName ~ "/controls/flight/target-hdg") ?? 0;
            
            setprop("" ~ myNodeName ~ "/controls/flight/target-hdg", courseToTarget_deg);
            print(sprintf("Bombable: updated target heading for %s from %.1f to %.1f", myNodeName, oldHdg, courseToTarget_deg));
        }

	var ats = bombable.attributes[myNodeName];
	# skill ranges 0-6
	var skill = bombable.calcPilotSkill (myNodeName);
	print (rand() < skill / 6 * (1.0 - ats.damage));





##################### find_closest_runway_details ##########################
# Queries airportinfo(icao) and finds the runway best aligned with mainAC_heading.
# Returns a hash containing ID, heading (deg), length (m/ft), and threshold coordinates (lat/lon).
# Returns nil if the airport ICAO code is not found in apt.dat.

var find_closest_runway_details = func(icao, mainAC_heading) {
    # 1. Fetch and validate airport existence
    var apt = airportinfo(icao);
    if (apt == nil) {
        print("Error: Airport '" ~ icao ~ "' not found in apt.dat database.");
        return nil;
    }

    # Normalize target heading to [0, 360) range using geo.nas
    var target_hdg = geo.normdeg(mainAC_heading);
    
    var best_rwy = nil;
    var min_diff = 999.0;

    # 2. Iterate through available runways and find the closest heading match
    var rwy_keys = keys(apt.runways);
    foreach (var rwy_id; rwy_keys) {
        var rwy = apt.runways[rwy_id];
        
        # Shortest arc difference across 0/360 boundary
        var diff = abs(geo.normdeg180(rwy.heading - target_hdg));

        if (diff < min_diff) {
            min_diff = diff;
            best_rwy = rwy;
        }
    }

    # 3. Construct and return result hash
    if (best_rwy != nil) {
        var result = {
            id: best_rwy.id,
            heading: best_rwy.heading,
            length_m: best_rwy.length,
            length_ft: best_rwy.length * 3.28084,
            lat: best_rwy.lat,
            lon: best_rwy.lon
        };

        var elev = geo.elevation(result.lat, result.lon);
        var elev_ft = elev * M2FT;

        # Print summary to console
        print(sprintf("\n--- Selected Runway Details for %s ---", icao));
        print(sprintf("Runway ID  : %s", result.id));
        print(sprintf("Heading    : %.2f deg (Off by %.2f deg)", result.heading, min_diff));
        print(sprintf("Length     : %.0f ft (%.1f m)", result.length_ft, result.length_m));
        print(sprintf("Threshold  : Lat %.6f, Lon %.6f", result.lat, result.lon));
        print(sprintf("Elevation  : %.2f m %.2f ft", elev, elev_ft));

        return result;
    }

    return nil;
}
# Fetch airport object by ICAO
var airport_name = "ROAH";
find_closest_runway_details(airport_name, 180.0);


##################### check teams ##########################


			var teamName = "W";
            var count = bombable.teams[teamName].count;
			if (count < size(bombable.teams[teamName].indices)) {
                print("error: " ~ count);
            } # check to ensure scenario definition and extension files are consistent

##################### process_offsets ##########################
var process_offsets = func(icao, offsets, is_runway_aligned = 0) {
    # Find the airport object
    var apt = airportinfo(icao);
    if (apt == nil) {
        print("Airport " ~ icao ~ " not found.");
        return [];
    }
    
    # Get the main runway
    var rwys = apt.runways;
    var rwy_keys = keys(rwys);
    if (size(rwy_keys) == 0) {
        print("No runways found for airport " ~ icao);
        return [];
    }
    
    var main_rwy = rwys[rwy_keys[0]];
    
    # Extract runway properties
    var base_lat  = main_rwy.lat;
    var base_lon  = main_rwy.lon;
    var rwy_heading = main_rwy.heading;

    var result_points = [];

    foreach (var pair; offsets) {
        # Construct the geo.Coord object
        var target_pos = geo.Coord.new();
        target_pos.set_latlon(base_lat, base_lon);

        if (is_runway_aligned) {
            var d_parallel = pair[0]; # Parallel along runway heading (m)
            var d_perp     = pair[1]; # Perpendicular right of runway heading (m)

            if (d_parallel != 0) {
                var hdg_p = (d_parallel >= 0) ? rwy_heading : math.fmod(rwy_heading + 180.0, 360.0);
                target_pos.apply_course_distance(hdg_p, math.abs(d_parallel));
            }

            if (d_perp != 0) {
                var hdg_perp = (d_perp >= 0) ? math.fmod(rwy_heading + 90.0, 360.0) : math.fmod(rwy_heading + 270.0, 360.0);
                target_pos.apply_course_distance(hdg_perp, math.abs(d_perp));
            }

        } else {
            var dx = pair[0]; # Easting (m)
            var dy = pair[1]; # Northing (m)

            if (dx != 0) {
                var hdg_x = (dx >= 0) ? 90.0 : 270.0;
                target_pos.apply_course_distance(hdg_x, math.abs(dx));
            }

            if (dy != 0) {
                var hdg_y = (dy >= 0) ? 0.0 : 180.0;
                target_pos.apply_course_distance(hdg_y, math.abs(dy));
            }
        }

        var new_lat = target_pos.lat();
        var new_lon = target_pos.lon();

        # Calculate elevation dynamically for the target coordinate:
        # Option 1: Use geo.elevation(lat, lon)
        var elev = geo.elevation(new_lat, new_lon);

        # Fallback to main runway elevation if scenery terrain at point is unloaded (nil)
        if (elev == nil) {
            var info = geodinfo(new_lat, new_lon);
            if (info != nil and info[0] != nil) {
                elev = info[0];
            } else {
                elev = main_rwy.elevation;
            }
        }

        var elev_ft = elev * M2FT;
        append(result_points, [new_lat, new_lon, elev]);

        # Output formatted for FlightGear scenario XML files
        print('<latitude type="double">' ~ sprintf("%.8f", new_lat) ~ '</latitude>');
        print('<longitude type="double">' ~ sprintf("%.8f", new_lon) ~ '</longitude>');
        print('<altitude>' ~ sprintf("%.2f", elev_ft) ~ '</altitude>');
    }

    return result_points;
};

var airportName = "EGOD";
var offsets =[
    [0, 0],
    [4, 0],
    [8, 0],
    [12, 0],
    [16, 0],
    [20, 0],
    [-0, 0],
    [-4, 0],
    [-8, 0],
    [-12, 0],
    [-16, 0],
    [-20, 0],
];
# var offsets =[
#     [25, 12],
#     [-25,-12],
#     [50,25],
#     [-50,-25]
# ];
var result = process_offsets(airportName, offsets, 1);
debug.dump(result);

##################### dump attributes of ai model ##########################

var myNodeName = "/ai/models/static";
var ats = bombable.attributes[myNodeName];
debug.dump(ats);

debug.dump(bombable.nodeNames);

##################### fireAIWeapon (working) ##########################
# AA gun
var myNodeName = "/ai/models/static";
var ats = bombable.attributes[myNodeName];
var weaps=ats.weapons;
var time_sec = 2.0;
var speed=700;
var elem="flak_gun_88mm_left";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);
var elem="flak_gun_88mm_centre";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);
var elem="flak_gun_88mm_right";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);

#b17
var myNodeName = "/ai/models/aircraft";
var ats = bombable.attributes[myNodeName];
var weaps=ats.weapons;
var time_sec = 2.0;
var speed=700;
var elem="rear_gun";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);
var elem="chin_turret_gun";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);
var elem="belly_gun";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);
var elem="top_turret_gun";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);

#f6f
var myNodeName = "/ai/models/aircraft[1]";
var ats = bombable.attributes[myNodeName];
var weaps=ats.weapons;
var time_sec = 2.0;
var speed=700;
var elem="front_gun_left";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);
var elem="front_gun_right";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);

#a62m
var myNodeName = "/ai/models/aircraft[2]";
var ats = bombable.attributes[myNodeName];
var weaps=ats.weapons;
var time_sec = 3.0;
var speed=700;
var elem="front_gun";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);
var elem="front_cannon";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);

#WW2jeep
var myNodeName = "/ai/models/ship";
var ats = bombable.attributes[myNodeName];
var weaps=ats.weapons;
var time_sec = 3.0;
var speed=700;
var elem="jeep_MG";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);

#Leopard
var myNodeName = "/ai/models/ship";
var ats = bombable.attributes[myNodeName];
var weaps=ats.weapons;
var time_sec = 3.0;
var speed=700;
var elem="main_gun";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);
var elem="turret_MG";
bombable.fireAIWeapon (time_sec, myNodeName, elem, speed);
##################### add model ##########################
# var weapon_node = ai_node.getNode("models/model[13]", 1);

# Populate values
# weapon_node.setDoubleValue("offset-x", 0.0);
# weapon_node.setDoubleValue("offset-y", 0.0);
# weapon_node.setDoubleValue("offset-z", 0.0);
# weapon_node.setDoubleValue("speed", 750.0);
# weapon_node.setDoubleValue("projectile-startsize", 0.3); # Larger size for high visibility testing
# weapon_node.setDoubleValue("projectile-endsize", 0.1);
# weapon_node.setBoolValue("ai-weapon-firing", 1);
# setprop("/bombable/menusettings/fire-particles/ai-weapon-fire-visual-trigger", 1);

var ai_node = props.globals.getNode("ai/models").getChildren("static")[0];

fgcommand("add-model", props.Node.new({
    "path": "AI/Aircraft/Fire-Particles/myTracer.xml",
    "latitude-deg-prop":  ai_node.getPath() ~ "/position/latitude-deg",
    "longitude-deg-prop": ai_node.getPath() ~ "/position/longitude-deg",
    "elevation-ft-prop":   ai_node.getPath() ~ "/position/altitude-ft",
    "heading-deg-prop":    ai_node.getPath() ~ "/orientation/true-heading-deg",
    "pitch-deg-prop":      ai_node.getPath() ~ "/orientation/pitch-deg",
    "roll-deg-prop":       ai_node.getPath() ~ "/orientation/roll-deg"
}));
##################### dump branch of property tree ##########################

var myNodeName = "/ai/models/aircraft";
var ats = bombable.attributes[myNodeName];
var key = "weapons";
if (contains(ats, key)) {
debug.dump(ats[key]);
}
else
{
    print(key ~ " is not a key");
}
var weaps=ats.weapons;
debug.dump(weaps.top_turret_gun);
##################### test updateWptHeading ##########################

# skill ranges 0-6
var skill = bombable.calcPilotSkill (myNodeName);
var threshold1 = 5000;
var threshold2 = 500;
var myHeading_deg = ats.heading; # if further away than threshold 1 stasy on heading in initial scenario
if (rand() < skill / 6 * (1.0 - ats.damage)) {
    if (!ats.jobDone) {
    var distHdg = bombable.courseToAirport (myNodeName); # returns a hash
    var dist = distHdg.distance;
        if (dist[0] < threshold2) {
            ats.jobDone = 1;
            var msg = bombable.getCallSign(myNodeName) ~ " reached target.  Mission accomplished.  Returning to base.";
            bombable.dodge(myNodeName);
            gui.popupTip(msg, 5);				
            mainStatusPopupTip (msg, 5);
            debprint ("Bombable: "~msg);
            myHeading_deg = math.fmod(myHeading_deg + 180, 360);
            ats.heading = myHeading_deg; # update the default heading so as to return to base
        }
        elsif (dist[0] < threshold1) {
            myHeading_deg = distHdg.heading; # set course heading for target
        }
    } 
    var oldHdg = getprop("" ~ myNodeName ~ "/controls/flight/target-hdg");
    if (abs(oldHdg - myHeading_deg) > 2) {
        setprop("" ~ myNodeName ~ "/controls/flight/target-hdg", myHeading_deg);
        debprint(sprintf("Bombable: updated target heading for %s from %.1f to %.1f", myNodeName, oldHdg, myHeading_deg));
    }
}


###################### get scenario info ####################

	var scenarioName = getprop("/sim/ai/scenario");
	if (scenarioName == nil) scenarioName = "BOMB-Llandbehr_Type45_F15_rocket";
	bombable.debprint("Bombable: starting scenario "~scenarioName);

	# Construct file path relative to addon path
	var scenarioFilePath = getprop("/sim/fg-aircraft") ~ "/../../Scenarios/Extensions/" ~ scenarioName ~ ".xml";
	
	# Load XML into a temporary property branch
	var targetTree = props.globals.getNode("/sim/ai/bombable-temp", 1);
	if (!io.read_properties(scenarioFilePath, targetTree)) {
		debprint("Bombable: startScenario: Error loading file " ~ scenarioFilePath);
		return;
	}

	# Dynamically construct the scenario hash from loaded properties
	var scenario = [];
	var groupNodes = targetTree.getChildren("group");
	
	foreach (var gNode; groupNodes) {
		var offsetList = [];
		
		var offsetsNode = gNode.getNode("offsets");
		if (offsetsNode != nil) {
			foreach (var oNode; offsetsNode.getChildren("offset")) {
				append(offsetList, [
					oNode.getNode("behind", 1).getValue() or 0,
					oNode.getNode("right", 1).getValue() or 0,
					oNode.getNode("up", 1).getValue() or 0
				]);
			}
		}

		append(scenario, {
			team        : gNode.getNode("team", 1).getValue(),
			target      : gNode.getNode("target", 1).getValue(),
			arrivalTime : gNode.getNode("arrivalTime", 1).getValue(),
			airSpeed    : gNode.getNode("airSpeed", 1).getValue() * KT2MPS,
			airportName : gNode.getNode("airportName", 1).getValue(),
			heading     : gNode.getNode("heading", 1).getValue(),
			alt         : gNode.getNode("alt", 1).getValue(),
			offsets     : offsetList
		});
	}

	# Clear the temporary property tree
	targetTree.remove();

###################### test geocord navigation ####################    

    var GeoCoord = geo.Coord.new();
    var GeoCoord2 = geo.Coord.new();    

    var from = airportinfo("ROAH"); # provides lat, lon, alt of airport
    var o = [-1500,0,0];


    GeoCoord.set_latlon(from.lat, from.lon);
    var dist = 0;
    var heading = 177.59;
    GeoCoord.apply_course_distance(heading + 180, dist);
    GeoCoord2.set_latlon ( GeoCoord.lat(), GeoCoord.lon());
    var myHeading = math.atan2(o[1], -o[0]) * R2D;
    var deltaHeading = heading + myHeading ;
    dist2me = math.sqrt(o[0]*o[0] + o[1]*o[1]); 
    GeoCoord2.apply_course_distance(deltaHeading, dist2me);    #frontreardist in meters
    var lat = GeoCoord.lat();
    var lon = GeoCoord.lon();
    var lat2 = GeoCoord2.lat();
    var lon2 = GeoCoord2.lon();
    print(GeoCoord.lat() ~ " " ~ GeoCoord.lon());
    print(GeoCoord2.lat() ~ " " ~ GeoCoord2.lon());

###################### add scene marker ####################    
# 1. Define target coordinates and elevation (in meters)

var addMarker = func(lat, lon) {
        var elev_m = geo.elevation(lat, lon); # Get ground level automatically

        if (elev_m == nil) {
            elev_m = 0; # Fallback if terrain is not yet loaded into memory
        }

        # 2. Define path to your 3D model relative to $FG_ROOT or absolute
        # e.g., "Models/Cursor/cursor.ac" or an absolute OS path
        # var model_path = "Models/Autopush/cursor.ac"; 
        var model_path = "Models/Buildings/water-tower.ac"; 

        # 3. Spawn the model on the terrain
        # geo.put_model(path, lat, lon, [elevation_m, heading_deg, pitch_deg, roll_deg]);
        var cursor_node = geo.put_model(model_path, lat, lon, elev_m);

        if (cursor_node != nil) {
            print("Marker successfully placed at: ", lat, ", ", lon);
        } else {
            print("Failed to place marker. Check model path.");
        }
    }

    addMarker(lat, lon);
    addMarker(lat2, lon2);


############### print out FG paths ###############        

print("=== FlightGear Standard Paths ===");
print("FG_ROOT:     ", getprop("/sim/fg-root"));
print("FG_HOME:     ", getprop("/sim/fg-home"));
print("FG_AIRCRAFT: ", getprop("/sim/fg-aircraft"));

# Optional: Print current user scenery and download paths
print("FG_SCENERY:  ", getprop("/sim/fg-scenery"));
print("FG_DOWNLOAD: ", getprop("/sim/terrasync/default-home"));

################ remove all ufo models ##############      

# Fetch the root property node where UFO stores placed models
var models_node = props.globals.getNode("/models");

if (models_node != nil) {
    # Remove all spawned child model nodes from the scene
    models_node.removeChildren("model");
    print("All UFO models removed from scene.");
}

################ print out targetting data ##############      
var myNodeName = "/ai/models/static";
var ats = bombable.attributes[myNodeName];
var myTargets = ats.targetIndex;
var nTargets = size(myTargets);
debug.dump(bombable.nodeNames);
debug.dump(myTargets);
foreach (elem; keys (ats.weapons) ) 
{	
    var thisWeapon = ats.weapons[elem];
    if (thisWeapon.destroyed == 1) 
    {
        print("" ~ elem ~ " destroyed");
        continue; #skip this weapon if destroyed
    }
    var ind = thisWeapon.aim.target; # index of object to shoot at
    var pos = vecindex(myTargets, ind);
    print (elem ~ " is targetting " ~ ind);
    if ( bombable.stores.checkWeaponsReadiness ( myNodeName, elem ) == 0) 
    {
        print("" ~ elem ~ " out of ammo");
        continue; # can only shoot if ammo left!
    }
}

var myNodeName = "/ai/models/aircraft";
var ats = bombable.attributes[myNodeName];
var myTargets = ats.targetIndex;
var nTargets = size(myTargets);
debug.dump(bombable.nodeNames);
debug.dump(myTargets);
foreach (elem; keys (ats.weapons) ) 
{	
    var thisWeapon = ats.weapons[elem];
    if (thisWeapon.destroyed == 1) 
    {
        print("" ~ elem ~ " destroyed");
        continue; #skip this weapon if destroyed
    }
    var ind = thisWeapon.aim.target; # index of object to shoot at
    var pos = vecindex(myTargets, ind);
    print (elem ~ " is targetting " ~ ind);
    if ( bombable.stores.checkWeaponsReadiness ( myNodeName, elem ) == 0) 
    {
        print("" ~ elem ~ " out of ammo");
        continue; # can only shoot if ammo left!
    }
}

var myNodeName = "/ai/models/aircraft[1]";
var ats = bombable.attributes[myNodeName];
var myTargets = ats.targetIndex;
var nTargets = size(myTargets);
debug.dump(bombable.nodeNames);
debug.dump(myTargets);
foreach (elem; keys (ats.weapons) ) 
{	
    var thisWeapon = ats.weapons[elem];
    if (thisWeapon.destroyed == 1) 
    {
        print("" ~ elem ~ " destroyed");
        continue; #skip this weapon if destroyed
    }
    var ind = thisWeapon.aim.target; # index of object to shoot at
    var pos = vecindex(myTargets, ind);
    print (elem ~ " is targetting " ~ ind);
    if ( bombable.stores.checkWeaponsReadiness ( myNodeName, elem ) == 0) 
    {
        print("" ~ elem ~ " out of ammo");
        continue; # can only shoot if ammo left!
    }
}

var myNodeName = "/ai/models/aircraft[2]";
var ats = bombable.attributes[myNodeName];
var myTargets = ats.targetIndex;
var nTargets = size(myTargets);
debug.dump(bombable.nodeNames);
debug.dump(myTargets);
foreach (elem; keys (ats.weapons) ) 
{	
    var thisWeapon = ats.weapons[elem];
    if (thisWeapon.destroyed == 1) 
    {
        print("" ~ elem ~ " destroyed");
        continue; #skip this weapon if destroyed
    }
    var ind = thisWeapon.aim.target; # index of object to shoot at
    var pos = vecindex(myTargets, ind);
    print (elem ~ " is targetting " ~ ind);
    if ( bombable.stores.checkWeaponsReadiness ( myNodeName, elem ) == 0) 
    {
        print("" ~ elem ~ " out of ammo");
        continue; # can only shoot if ammo left!
    }
}

# ==============================================================================
# replace_placeholders_2 - NOT TESTED
# ==============================================================================

var replace_placeholders_2 = func(str_, target, replacement) {
    if (str_ == nil or target == nil or target == "") return str_;
    
    var res = "";
    var pos = 0;
    var target_len = size(target);
    var str_len = size(str_);
    
    while (pos < str_len) {
        var idx = search(substr(str_, pos), target);
        if (idx == -1) {
            res ~= substr(str_, pos);
            break;
        }
        res ~= substr(str_, pos, idx) ~ replacement;
        pos += idx + target_len;
    }
    
    return res;
};
###############################################################################
# rotate_zxy Test & Benchmark Script
###############################################################################

# 1. Original Implementation
var rotate_zxy_old = func (vector, alpha, beta, gamma) {
    var alpha_rad = alpha * D2R;
    var beta_rad = beta * D2R;
    var gamma_rad = gamma * D2R;
 
    var c_alpha = math.cos(alpha_rad);
    var s_alpha = math.sin(alpha_rad);
    var c_beta = math.cos(beta_rad);
    var s_beta = math.sin(beta_rad);
    var c_gamma = math.cos(gamma_rad);
    var s_gamma = math.sin(gamma_rad);

    var matrix = [
        [
           c_gamma * c_beta + s_gamma * s_alpha * s_beta,
           -s_gamma * c_beta + c_gamma * s_alpha * s_beta,
           c_alpha * s_beta
        ],

        [
            s_gamma * c_alpha,
            c_gamma * c_alpha,
            -s_alpha
        ],

        [
          -c_gamma * s_beta + s_gamma * s_alpha * c_beta,
          s_gamma * s_beta + c_gamma * s_alpha * c_beta,
          c_alpha * c_beta
        ]
    ];

    var x2 = vector[0] * matrix[0][0] + vector[1] * matrix[1][0] + vector[2] * matrix[2][0];
    var y2 = vector[0] * matrix[0][1] + vector[1] * matrix[1][1] + vector[2] * matrix[2][1];
    var z2 = vector[0] * matrix[0][2] + vector[1] * matrix[1][2] + vector[2] * matrix[2][2];

    return [x2, y2, z2];
};

# 2. Optimized Implementation (Method 1: Direct Inlined Math)
var rotate_zxy_new = func (vector, alpha, beta, gamma) {
    var a_rad = alpha * D2R;
    var b_rad = beta * D2R;
    var g_rad = gamma * D2R;
 
    var ca = math.cos(a_rad); var sa = math.sin(a_rad);
    var cb = math.cos(b_rad); var sb = math.sin(b_rad);
    var cg = math.cos(g_rad); var sg = math.sin(g_rad);

    var vx = vector[0]; var vy = vector[1]; var vz = vector[2];

    return [
        vx * (cg * cb + sg * sa * sb) + vy * (sg * ca) + vz * (-cg * sb + sg * sa * cb),
        vx * (-sg * cb + cg * sa * sb) + vy * (cg * ca) + vz * (sg * sb + cg * sa * cb),
        vx * (ca * sb) - vy * sa + vz * (ca * cb)
    ];
};

# 3. Helper to test numerical equality within tolerance
var approx_equal = func(v1, v2, tol = 1e-6) {
    return (math.abs(v1[0] - v2[0]) < tol) and 
           (math.abs(v1[1] - v2[1]) < tol) and 
           (math.abs(v1[2] - v2[2]) < tol);
};

###############################################################################
# Test Suite Execution
###############################################################################

print("\n=======================================================");
print("       RUNNING ROTATE_ZXY ACCURACY & SPEED TEST        ");
print("=======================================================\n");

# Define diverse test cases: [vector, alpha, beta, gamma, description]
var test_cases = [
    [[1.0, 0.0, 0.0],    0.0,   0.0,   0.0, "Identity (Zero rotation)"],
    [[1.0, 0.0, 0.0],    0.0,   0.0,  90.0, "Pure Z rotation (90 deg)"],
    [[0.0, 1.0, 0.0],   45.0,   0.0,   0.0, "Pure X rotation (45 deg)"],
    [[0.0, 0.0, 1.0],    0.0,  30.0,   0.0, "Pure Y rotation (30 deg)"],
    [[1.0, 2.0, 3.0],   15.0,  30.0,  45.0, "Arbitrary positive angles"],
    [[-0.83, 1.11, 2.83], -20.0, 35.0, -110.0, "MG Pivot offset vector with negative angles"],
    [[12.5, -3.4, 0.2],  180.0, -90.0,  270.0, "Extreme/Gimbal angles"]
];

var all_passed = 1;

foreach (var tc; test_cases) {
    var vec   = tc[0];
    var a     = tc[1];
    var b     = tc[2];
    var g     = tc[3];
    var desc  = tc[4];

    var res_old = rotate_zxy_old(vec, a, b, g);
    var res_new = rotate_zxy_new(vec, a, b, g);
    var passed  = approx_equal(res_old, res_new);

    if (!passed) { all_passed = 0; }

    print("Test Case: " ~ desc);
    print("  Input Vector : [" ~ vec[0] ~ ", " ~ vec[1] ~ ", " ~ vec[2] ~ "]");
    print("  Angles (a,b,g): " ~ a ~ ", " ~ b ~ ", " ~ g);
    print("  OLD Output   : [" ~ sprintf("%.6f", res_old[0]) ~ ", " ~ sprintf("%.6f", res_old[1]) ~ ", " ~ sprintf("%.6f", res_old[2]) ~ "]");
    print("  NEW Output   : [" ~ sprintf("%.6f", res_new[0]) ~ ", " ~ sprintf("%.6f", res_new[1]) ~ ", " ~ sprintf("%.6f", res_new[2]) ~ "]");
    print("  Status       : " ~ (passed ? "PASS" : "FAIL *** MISMATCH ***"));
    print("-------------------------------------------------------");
}

print("\nAccuracy Verification Result: " ~ (all_passed ? "ALL TESTS PASSED!" : "SOME TESTS FAILED!"));

###############################################################################
# Performance / Compute Time Benchmark
###############################################################################

print("\n=======================================================");
print("               BENCHMARKING EXECUTION TIME             ");
print("=======================================================");

var iterations = 100000;
print("Running " ~ iterations ~ " iterations per method...\n");

# Benchmark Old Method
var t_start_old = systime();
for (var i = 0; i < iterations; i += 1) {
    var dummy1 = rotate_zxy_old([-0.83, 1.11, 2.83], 15.4, -8.2, 42.1);
}
var t_end_old = systime();
var time_old = t_end_old - t_start_old;

# Benchmark New Method
var t_start_new = systime();
for (var i = 0; i < iterations; i += 1) {
    var dummy2 = rotate_zxy_new([-0.83, 1.11, 2.83], 15.4, -8.2, 42.1);
}
var t_end_new = systime();
var time_new = t_end_new - t_start_new;

# Performance Summary
var speedup = (time_old > 0 and time_new > 0) ? (time_old / time_new) : 0.0;

print("RESULTS:");
print("  OLD Method Total Time: " ~ sprintf("%.4f", time_old) ~ " seconds");
print("  NEW Method Total Time: " ~ sprintf("%.4f", time_new) ~ " seconds");
print("  Performance Improvement: " ~ sprintf("%.2f", speedup) ~ "x faster");
print("=======================================================\n");