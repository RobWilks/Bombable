# Set your property
setprop("/sim/ai/scenario", "BOMB-MarinCountyNineA6M5ThreeB17");

# Tell FlightGear to save this property to autosave.xml on exit
var node = props.globals.getNode("/sim/ai/scenario", 1);
node.setAttribute("archive", 1);

debug.dump(teams);

debug.dump(bombable.targetData);

var myNodeName1 = "/ai/models/aircraft[7]";
var ats = bombable.attributes[myNodeName1];
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

debug.dump(bombable.nodes);

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



##################### dump attributes of ai model ##########################

var myNodeName = "/ai/models/static";
var ats = bombable.attributes[myNodeName];
debug.dump(ats);

debug.dump(bombable.nodes);

##################### dump branch of property tree ##########################

var myNodeName = "/ai/models/static";
var ats = bombable.attributes[myNodeName];
var key = "type";
if (contains(ats, key)) {
debug.dump(ats[key]);
}
else
{
    print(key ~ " is not a key");
}
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

##############################        

