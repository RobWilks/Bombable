#####################################################
# Bombable
# Brent Hugh, brent@brenthugh.com
var bombableVersion = "4.6";
#
# Copyright (C) 2009 - 2011  Brent Hugh  (brent@brenthugh.com)
# This file is licensed under the GPL license version 2 or later.
#
# The Bombable module implements several different but interrelated functions
# that can be used by, for example, AI objects and scenery objects.  The
# functions allow shooting and bombing of AI and multiplayer objects, explosions
# and disabling of objects that are hit, and even multiplayer communications to
# allow dogfighting:
#
# 1. BOMBABLE: Makes objects bombable.  They will detect hits, change livery according to damage, and finally start on fire and smoke when sufficiently damaged. There is also a function to change the livery colors according to damage level.
#
# Bombable also works for multiplayer objects and allows multiplayer dogfighting.
#
# In addition, it creates an explosion whenever the main aircraft hits the ground and crashes.
#
# 2. GROUND: Makes objects stay at ground level, adjusting pitch to match any slope they are on.  So, for instance, cars, trucks, or tanks can move and always stay on the ground, drive up slopes somewhat realistically, etc. Ships can be placed in any lake and will automatically find their correct altitude, etc.
#
# 3. LOCATE: Usually AI objects return to their initial start positions when FG re-inits (ie, file/reset). This function saves and maintains their previous position prior to the reset
#
# 4. ATTACK: Makes AI Aircraft (and conceivable, other AI objects as well) swarm and attack the main aircraft
#
# 5. WEAPONS: Makes AI Aircraft shoot at the main aircraft
#
#TYPICAL USAGE--ADDING BOMBABILITY TO AN AI OR SCENERY MODEL
#
# Required:
#  1. The Fire-Particles subdirectory (included in this package)
#     must be installed in the FG/data/AI/Aircraft/Fire-Particles subdirectory
#  2. This file, bombable.nas, must be installed in the FG/data/Nasal
#     subdirectory
#
# To make any particular object "bombable", simply include code similar to that
# included in the AI aircraft XML files in this distribution.
#
# This approach generally should work with any AI objects or scenery objects.
#
# You then typically create an AI scenario that includes these "bombable
# objects" and then, to see and bomb the objects, load the scenario using
# the command line or fgrun when you start FlightGear.
#
# Or two (or more) players can choose aircraft set up for dogfighting (see readme in accompanying documentation) and dogfight via multiplayer.  Damage, smoke, fire, and explosions are all transmitted via multiplayer channels.
#
# Notes:
#  - The object's node name can be found using cmdarg().getPath()
#  - You can make slight damage & high damage livery quite easily by modifying
#    any existing livery a model may have.  Note, however, that many objects
#    do not use liveries, but simply include color in the model itself. You
#    won't be able to change liveries on such models unless you alter the
#    model (.ac file) to use external textures.
#
#
# See file bombable-modding-aircraft-for-dogfighting.txt included in this
# package for more details about adding bombable to aircraft or other objects.
#
# See m1-abrams/m1.xml and other AI object XML files in this package for
# sample implementations.
#
#
#AUTHORS
# 	Base code for M1 Abrams tank by Emmanuel BARANGER - David BASTIEN
#   Modded heavily by Brent HUGH to add re-location and ground altitude
#   functionality, crashes for ships and aircraft, evasive maneuvers when
#   under attack, multiplayer functionality, other functionality,
#   and to abstract the code to a unit that can be included in most
#   any AI or scenery object.
#
#   Many snippets of code and examples of implemention were borrowed from other
#   FlightGear projects--thanks to all those contributors!
#


#################################
# prints values to console only if 'debug' flag is set in props
var debprint = func {

	setprop ("/sim/startup/terminal-ansi-colors",0);
	
	if (bombableMenu["debug"]) {
		outputs = "";
		foreach (var elem;arg) {
			if (elem != nil) {
				if (typeof(elem) == "scalar") outputs = string.trim(outputs) ~ " " ~ elem;
				else debug.dump(elem);
			}
		};
		outputs = outputs ~ " (Line #";
		var call1 = caller();
		var call2 = caller("2");
		var call3 = caller ("3");
		var call4 = caller ("4");
		if (typeof(call1) == "vector")  outputs = outputs ~ call1["3"] ~ " ";
		if (typeof(call2) == "vector")  outputs = outputs ~ call2["3"] ~ " ";
		if (typeof(call3) == "vector")  outputs = outputs ~ call3["3"] ~ " ";
		if (typeof(call4) == "vector")  outputs = outputs ~ call4["3"] ~ " ";
		outputs = outputs ~ ")";
		
		print (outputs);
	}
}


############## round #################
# returns round to nearest whole number
var round = func (a ) return int (a+0.5);

############## normdeg180 #################
# normalize degree to -180 < angle <= 180
# (for checking aim)
#
var normdeg180 = func(angle) {
	while (angle <= - 180)
	angle  +=  360;
	while (angle > 180)
	angle  -=  360;
	return angle;
}

############################ check_overall_initialized ###############################
# Checks whether nodeName has been overall-initialized yet
# if so, returns 1
# if not, returns 0 and sets nodeName ~ /bombable/overall-initialized to true
#
var check_overall_initialized = func(nodeName) 
{
	nodeName = cmdarg().getPath();
	#only allow initialization for ai & multiplayer objects
	# in FG 2.4.0 we're having trouble with strange(!?) init requests from
	# joysticks & the like
	var init_allowed = 0;
	if (find ("/ai/models/", nodeName ) != -1 ) init_allowed = 1;
	if (find ("/multiplayer/", nodeName ) != -1 ) init_allowed = 1;
	
	if (init_allowed != 1) 
	{
		bombable.debprint ("Bombable: Attempt to initialize a Bombable subroutine on an object that is not AI or Multiplayer; aborting initialization. ", nodeName);
		return 1; #1 means abort; it's initialized already or can't/shouldn't be initialized,
	}
	
	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	# if it 1 and we're trying to initialize, something has gone wrong and we abort with a message.
	var inited = getprop(""~nodeName~"/bombable/overall-initialized");
	
	if (inited == 1) 
	{
		bombable.debprint ("Bombable: Attempt to re-initialize AI aircraft when it has not been de-initialized; aborting re-initialization. ", nodeName);
		return 1; #1 means abort; it's initialized already or can't/shouldn't be initialized,
	}
	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	setprop(""~nodeName~"/bombable/overall-initialized", 1);
	return 0;
}

var de_overall_initialize = func(nodeName) 
{
	setprop(""~nodeName~"/bombable/overall-initialized", 0);
}

var mpprocesssendqueue = func {
	
	#we do the settimer first so any runtime errors, etc., below, don't stop the
	# future instances of the timer from being started
	settimer (func {mpprocesssendqueue()}, mpTimeDelaySend );  # This was experimental: mpTimeDelaySend * (97.48 + rand()/20 )
	
	if (!getprop(MP_share_pp)) return "";
	if (!getprop (MP_broadcast_exists_pp)) return "";
	if (!bombableMenu["bombable-enabled"] ) return;

	if (size(mpsendqueue) > 0) {
		setprop (MP_message_pp, mpsendqueue[0]);
		mpsendqueue = subvec(mpsendqueue,1);
	}

}

var mpsend = func (msg) {
	#adding systime to the end of the message ensures that each message is unique--otherwise messages that happen to be the same twice in a row will
	# be ignored.  The system() at the end is ignored by the parser.
	#
	if (!getprop(MP_share_pp)) return "";
	if (!getprop (MP_broadcast_exists_pp)) return "";
	if (!bombableMenu["bombable-enabled"] ) return;
	
	append(mpsendqueue, msg ~ systime());
}

var mpreceive = func (mpMessageNode) {
	
	if (!getprop(MP_share_pp)) return "";
	if (!getprop (MP_broadcast_exists_pp)) return "";
	if (!bombableMenu["bombable-enabled"]) return;
	
	msg = mpMessageNode.getValue();
	mpMessageNodeName = mpMessageNode.getPath();
	mpNodeName = string.replace (mpMessageNodeName, MP_message_pp, "");
	if (msg != nil and msg != "") {
		debprint("Bombable: Message received from ", mpNodeName,": ", msg);
		parse_msg (mpNodeName, msg);
	}
	
	

}

####################################### put_ballistic_model #########################################
# put_ballistic_model places a new model that starts at another AI model's
# position but moves independently, like a bullet, bomb, etc
# this is still not working/experimental
# update: The best way to do this appears to be to include a weapons/tracer
# submodel in the main aircraft.  Then have Bombable place
# it in the right location, speed, direction, and trigger it.
# Somewhat similar to: http://wiki.flightgear.org/Howto:_Add_contrails#Persistent_Contrails
# rjw: FUNCTION NOT USED

var put_ballistic_model = func(myNodeName = "/ai/models/aircraft", path = "AI/Aircraft/Fire-Particles/fast-particles.xml") {

	# "environment" means the main aircraft
	#if (myNodeName == "/environment" or myNodeName == "environment") myNodeName = "";

	fgcommand("add-model", ballisticNode = props.Node.new({
		"path": path,
		"latitude-deg-prop": myNodeName ~ "/position/latitude-deg",
		"longitude-deg-prop":myNodeName ~ "/position/longitude-deg",
		"elevation-ft-prop": myNodeName ~ "/position/altitude-ft",
		"heading-deg-prop": myNodeName ~ "/orientation/true-heading-deg",
		"pitch-deg-prop": myNodeName ~ "/orientation/pitch-deg",
		"roll-deg-prop": myNodeName ~ "/orientation/roll-deg",
	}));
	
	
	print (ballisticNode.getName());
	print (ballisticNode.getName().getNode("property").getName());
	print (props.globals.getNode(ballisticNode.getNode("property").getValue()));
	print (ballisticNode.getNode("property").getValue());
	return props.globals.getNode(ballisticNode.getNode("property").getValue());

}

######################################### put_remove_model #######################################
#put_remove_model places a new model at the location specified and then removes
# it time_sec later
#it puts out 12 models/sec so normally time_sec = .4 or thereabouts it plenty of time to let it run
# If time_sec is too short then no particles will be emitted.  Typical problem is
# many rounds from a gun slow FG's framerate to a crawl just as it is time to emit the
# particles.  If time_sec is slower than the frame length then you get zero particle.
# Smallest safe value for time_sec is maybe .3 .4 or .5 seconds.
#
var put_remove_model = func(lat_deg = nil, lon_deg = nil, elev_m = nil, time_sec = nil, 
	startSize_m = nil, 	endSize_m = 1, path = "AI/Aircraft/Fire-Particles/flack-impact.xml" ) 
{

	if (lat_deg == nil or lon_deg == nil or elev_m == nil) { return; }
	
	var delay_sec = 0.1; #particles/models seem to cause FG crash * sometimes * when appearing within a model
	#we try to reduce this by making the smoke appear a fraction of a second later, after
	# the a/c model has moved out of the way. (possibly moved, anyway--depending on its speed)

	# debprint ("Bombable: Placing flack");
	
	settimer ( func 
	{
		#start & end size in particle system appear to be in feet
		if (startSize_m != nil) setprop ("/bombable/fire-particles/flack-startsize", startSize_m);
		if (endSize_m != nil) setprop ("/bombable/fire-particles/flack-endsize", endSize_m);

		fgcommand("add-model", flackNode = props.Node.new(
			{
			"path": path,
			"latitude-deg": lat_deg,
			"longitude-deg":lon_deg,
			"elevation-ft": elev_m * M2FT,
			"heading-deg"  : 0,
			"pitch-deg"    : 0,
			"roll-deg"     : 0,
			"enable-hot"   : 0,
			}
			));
		
		var flackModelNodeName = flackNode.getNode("property").getValue();
		
		#add the -prop property in /models/model[X] for each of lat, long, elev, etc
		foreach (name; ["latitude-deg","longitude-deg","elevation-ft", "heading-deg", "pitch-deg", "roll-deg"])
		{
			setprop(  flackModelNodeName ~"/"~ name ~ "-prop",flackModelNodeName ~ "/" ~ name );
		}
		
		# debprint ("Bombable: Placed flack, ", flackModelNodeName);
		
		settimer ( func { props.globals.getNode(flackModelNodeName).remove();}, time_sec);

	}, 
	delay_sec);
}

############################# start_terrain_fire #################################
#Start a fire on terrain, size depending on ballisticMass_lb
#location at lat/lon
#
var start_terrain_fire = func ( lat_deg, lon_deg, alt_m = 0, ballisticMass_lb = 1.2 ) {

	var info = geodinfo(lat_deg, lon_deg);
	
	
	
	debprint ("Bombable: Starting terrain fire at ", lat_deg, " ", lon_deg, " ", alt_m, " ", ballisticMass_lb);
	
	#get the altitude of the terrain
	if (info != nil) {
		#debprint ("Bombable: Starting terrain fire at ", lat_deg, " ", lon_deg, " ", info[0]," ", info[1].solid );
		
		#if it's water we don't set a fire . . . TODO make a different explosion or fire effect for water
		if (typeof(info[1]) == "hash" and contains(info[1], "solid") and info[1].solid == 0) return;
		else debprint (info);
		
		#we go with explosion point if possible, otherwise the height of terrain at this point
		if (alt_m == nil) alt_m = info[0];
		if (alt_m == nil) alt_m = 0;
		
	}
	
	if (ballisticMass_lb == nil or ballisticMass_lb < 0) ballisticMass_lb = 1.2;
	if (ballisticMass_lb < 3 ) { var time_sec = 20; var fp = "AI/Aircraft/Fire-Particles/fire-particles-very-very-small.xml"; }
	elsif (ballisticMass_lb < 20 ) { var time_sec = 60; var fp = "AI/Aircraft/Fire-Particles/fire-particles-very-very-small.xml"; }
	elsif (ballisticMass_lb < 50 ) { var time_sec = 120; var fp = "AI/Aircraft/Fire-Particles/fire-particles-very-small.xml"; }
	elsif (ballisticMass_lb > 450 ) { var time_sec = 600; var fp = "AI/Aircraft/Fire-Particles/fire-particles.xml"; }
	elsif (ballisticMass_lb > 1000 ) { var time_sec = 900; var fp = "AI/Aircraft/Fire-Particles/fire-particles-large.xml"; }
	else { var time_sec = 300; var fp = "AI/Aircraft/Fire-Particles/fire-particles-small.xml";}

	debprint ({lat_deg:lat_deg, lon_deg:lon_deg, elev_m:alt_m, time_sec:time_sec, startSize_m: nil, endSize_m:nil, path:fp });
	put_remove_model(lat_deg:lat_deg, lon_deg:lon_deg, elev_m:alt_m, time_sec:time_sec, startSize_m: nil, endSize_m:nil, path:fp );
	
	#making the fire bigger for bigger bombs
	if (ballisticMass_lb >= 1000 ) put_remove_model(lat_deg:lat_deg, lon_deg:lon_deg, elev_m:alt_m+1, time_sec:time_sec * .9, startSize_m: nil, endSize_m:nil, path:fp );
	if (ballisticMass_lb >= 1500 ) put_remove_model(lat_deg:lat_deg, lon_deg:lon_deg, elev_m:alt_m+2, time_sec:time_sec * .8, startSize_m: nil, endSize_m:nil, path:fp );
	if (ballisticMass_lb >= 2000 ) put_remove_model(lat_deg:lat_deg, lon_deg:lon_deg, elev_m:alt_m+3, time_sec:time_sec * .7, startSize_m: nil, endSize_m:nil, path:fp );
	
	##put it out, but slowly, for large impacts
	if (ballisticMass_lb > 50) {
		var time_sec2 = 120; var fp2 = "AI/Aircraft/Fire-Particles/fire-particles-very-small.xml";
		settimer (func { put_remove_model(lat_deg:lat_deg, lon_deg:lon_deg, elev_m:alt_m, time_sec:time_sec2, startSize_m: nil, endSize_m:nil, path:fp2 )} , time_sec);
		
		var time_sec3 = 120; var fp3 = "AI/Aircraft/Fire-Particles/fire-particles-very-very-small.xml";
		settimer (func { put_remove_model(lat_deg:lat_deg, lon_deg:lon_deg, elev_m:alt_m, time_sec:time_sec3, startSize_m: nil, endSize_m:nil, path:fp3 )} , time_sec + time_sec2);
		
		var time_sec4 = 120; var fp4 = "AI/Aircraft/Fire-Particles/fire-particles-very-very-very-small.xml";
		settimer (func { put_remove_model(lat_deg:lat_deg, lon_deg:lon_deg, elev_m:alt_m, time_sec:time_sec4, startSize_m: nil, endSize_m:nil, path:fp4 )} , time_sec + time_sec2 + time_sec3);
		
	}

}



####################################### put_tied_model #########################################
#put_tied_model places a new model that is tied to another AI model
# (given by myNodeName) and will move with it in lon, lat, & alt
# rjw called by startFire

var put_tied_model = func(myNodeName = "", path = "AI/Aircraft/Fire-Particles/Fire-Particles.xml ") {

	# "environment" means the main aircraft
	#if (myNodeName == "/environment" or myNodeName == "environment") myNodeName = "";

	fgcommand("add-model", fireNode = props.Node.new({
		"path": path,
		"latitude-deg-prop": myNodeName ~ "/position/latitude-deg",
		"longitude-deg-prop":myNodeName ~ "/position/longitude-deg",
		"elevation-ft-prop": myNodeName ~ "/position/altitude-ft",
		"heading-deg-prop": myNodeName ~ "/orientation/true-heading-deg",
		"pitch-deg-prop": myNodeName ~ "/orientation/pitch-deg",
		"roll-deg-prop": myNodeName ~ "/orientation/roll-deg",
	}));
	
	return props.globals.getNode(fireNode.getNode("property").getValue());

}

####################################### put_tied_weapon #########################################
# put_tied_weapon places a new model that is tied to another AI model
# (given by myNodeName) and will move with it in lon, lat, & alt
# and have the delta heading, pitch, lat, long, alt, as specified in weapons_init
#

var put_tied_weapon = func(myNodeName = "", elem = "", path = "AI/Aircraft/Fire-Particles/Fire-Particles.xml ") {

	# "environment" means the main aircraft
	# if (myNodeName == "/environment" or myNodeName == "environment") myNodeName = "";


	fgcommand("add-model", fireNode = props.Node.new({
		"path": path,
		"latitude-deg-prop": myNodeName ~ "/" ~ elem ~ "/position/latitude-deg",
		"longitude-deg-prop": myNodeName ~ "/" ~ elem ~ "/position/longitude-deg",
		"elevation-ft-prop": myNodeName ~ "/" ~ elem ~ "/position/altitude-ft",
		"heading-deg-prop": myNodeName ~ "/" ~ elem ~ "/orientation/true-heading-deg",
		"pitch-deg-prop": myNodeName ~ "/" ~ elem ~ "/orientation/pitch-deg",
		"roll-deg-prop": myNodeName ~ "/" ~ elem ~ "/orientation/roll-deg",
	}));
	
	return props.globals.getNode(fireNode.getNode("property").getValue());

}



######################### deleteFire ###########################
#Delete a fire object (model) created earlier, turn off the fire triggers
#and unlink the fire from the parent object.
#This sets the object up so it can actually start on fire again if
#wanted (or hit again by ballistics . . . though damage is already to max if
#it has been on fire for a while, and damage is not re-set)
var deleteFire = func (myNodeName = "",fireNode = "") {

	if (fireNode == "") 
	{
		fireNodeName = getprop(""~myNodeName~"/bombable/fire-particles/fire-particles-model");
		if (fireNodeName == nil) return;
		fireNode = props.globals.getNode(fireNodeName);
	}
	
	#remove the fire node/model altogether
	if (fireNode != nil) fireNode.remove();
	
	#turn off the object's fire trigger & unlink it from its fire model
	setprop(""~myNodeName~"/bombable/fire-particles/fire-burning", 0);
	setprop(""~myNodeName~"/bombable/fire-particles/fire-particles-model", "");

}

########################### speedDamage #########################
# For main AC
# Check current speed & add damage due to excessive speed
#
var speedDamage = func 
{
	if (!bombableMenu["bombable-enabled"] ) return;
	var damage_enabled = getprop (""~GF_damage_menu_pp~"damage_enabled");
	var warning_enabled = getprop (""~GF_damage_menu_pp~"warning_enabled");
	
	if (!  damage_enabled and ! warning_enabled ) return;
	
	var currSpeed_kt = getprop("/velocities/airspeed-kt");
	if (currSpeed_kt == 0 or currSpeed_kt == nil) return;
	
	var speedDamageThreshold_kt = getprop(""~vulnerabilities_pp~"airspeed_damage/damage_threshold_kt/");
	var speedWarningThreshold_kt = getprop(""~vulnerabilities_pp~"airspeed_damage/warning_threshold_kt/");
	
	if (speedDamageThreshold_kt == 0 or speedDamageThreshold_kt == nil) speedDamageThreshold_kt = 7000;
	if (speedWarningThreshold_kt == 0 or speedWarningThreshold_kt == nil) speedWarningThreshold_kt = 7000;
	
	var speedDamageMultiplier_PercentPerSecond = getprop(""~vulnerabilities_pp~"airspeed_damage/damage_multiplier_percentpersecond/");
	
	if (speedDamageMultiplier_PercentPerSecond == nil) speedDamageMultiplier_PercentPerSecond = 1;
	
	#debprint ("Bombable: Speed checking ", currSpeed_kt, " ", speedDamageThreshold_kt, " ", speedWarningThreshold_kt," ", speedDamageMultiplier_PercentPerSecond);

	if (warning_enabled and currSpeed_kt > speedWarningThreshold_kt ) {
		var msg = "Overspeed warning: "~ round ( currSpeed_kt ) ~" kts";
		debprint(msg);
		#only put the message on the screen if damage is less than 100%
		# after that there is little point AND it will overwrite
		# any "you're out of commission" message
		if ( attributes[""].damage < 1)
		mainStatusPopupTip (msg, 5 );
	}

	if (damage_enabled and currSpeed_kt > speedDamageThreshold_kt ) 
	{
		mainAC_add_damage
		(
			speedDamageMultiplier_PercentPerSecond * (currSpeed_kt - speedDamageThreshold_kt) / 100,
			0, "speed", 
			"" ~ round( currSpeed_kt ) ~ " kts (overspeed) damaged airframe!" 
		);
	}
}


####################################################
#For main AC
#Check current accelerations & add damage due to excessive acceleration
#
var accelerationDamage = func {

	if (!bombableMenu["bombable-enabled"] ) return;
	var damage_enabled = getprop (""~GF_damage_menu_pp~"damage_enabled");
	var warning_enabled = getprop (""~GF_damage_menu_pp~"warning_enabled");
	
	if (! damage_enabled and ! warning_enabled ) return;
	if (!bombableMenu["bombable-enabled"] ) return;
	
	#debprint ("Bombable: Checking acceleration");
	#The acceleration nodes are updated once per second
	
	
	var currAccel_g = getprop("/accelerations/pilot-gdamped");
	if (currAccel_g == 0 or currAccel_g == nil) return;
	
	if (currAccel_g > 0 ) a = "positive";
	else a = "negative";
	
	currAccel_fg = math.abs(currAccel_g);
	
	
	var accelDamageThreshold_g = getprop(""~GF_damage_pp~"damage_threshold_g/"~a);
	var accelWarningThreshold_g = getprop(""~GF_damage_pp~"warning_threshold_g/"~a);
	
	if (accelDamageThreshold_g == 0 or accelDamageThreshold_g == nil) accelDamageThreshold_g = 50;
	if (accelWarningThreshold_g == 0 or accelWarningThreshold_g == nil) accelWarningThreshold_g = 10;
	
	var accelDamageMultiplier_PercentPerSecond = getprop(""~GF_damage_pp~"damage_multiplier_percentpersecond/"~a);
	
	if (accelDamageMultiplier_PercentPerSecond == nil) accelDamageMultiplier_PercentPerSecond = 8;
	
	# debprint ("Bombable: Accel checking ", a, " ", currAccel_g, " ", accelDamageThreshold_g, " ", accelWarningThreshold_g," ", accelDamageMultiplier_PercentPerSecond);

	if (warning_enabled and currAccel_g > accelWarningThreshold_g ) {
		var msg = "G-force warning: "~ round( currAccel_g ) ~"g";
		debprint(msg);
		#only put the message on the screen if damage is less than 100%
		# after that there is little point AND it will overwrite
		# any "you're out of commission" message
		if ( attributes[""].damage < 1)
		mainStatusPopupTip (msg, 5 );
	}

	if (damage_enabled and currAccel_g > accelDamageThreshold_g ) {
		mainAC_add_damage( accelDamageMultiplier_PercentPerSecond * (currAccel_g -accelDamageThreshold_g)/100,
		0, "gforce", "" ~ sprintf("%1.2f", currAccel_g ) ~ "g force damaged airframe!" );
		
	}
}

#########################################################################
# timer for accel  & speed damage checks
#
var damageCheck = func () {
	settimer (func {damageCheck (); }, damageCheckTime);
	if (!bombableMenu["bombable-enabled"] ) return;
	#debprint ("Bombable: Checking damage.");
	accelerationDamage();
	speedDamage();

}

#Notes on g-force:
# Max G tolerated by a person for periods of c. 1 sec or more
# is about 30-50g even in fairly ideal circumstances.  So even if the aircraft
# survives your 30+g maneuver--you won't!
# F22 has max g of 9.5 and Su-47 has max g of 9, so those numbers
# are probably pretty typical for modern fighter jets.
# See http://www.thespacereview.com/article/410/1
# http://www.airforce-technology.com/projects/s37/
# http://www.airforce-technology.com/projects/s37/
# Max G tolerated routinely by fighter or acrobatic
#   pilots etc seems to be about 9-12g
# 12-17g is tolerated long term, depending on the direction of the
# acceleration  See http://en.wikipedia.org/wiki/G-force
# #max g for WWI era aircraft was 4-5g (best guess).  Repeat high gs do
# weaken the structure.
# In modern aircraft, F-16 has a maximum G of 9, F-18: 9.6, Mirage M.III/V: 7, A-4:6.
# http://www.ww2aircraft.net/forum/technical/strongest-aircraft-7494-3.html :
# WW2 aircraft sometimes has higher max G, but it is interesting because pilot did not have G-suit, and trained pilots could not resist 5g for more than some seconds without G-suit.
# the strongest aircraft of WWII were the Italian monoplane fighters. They were built to withstand 8g normal load with 12g failure load. The same spec for German aircraft was 6g - 8.33g. For the late war P51s it was 5.33g
# Spitfire VIII can pull about 9 and dive to about 570 mph before ripping apart while the F4U will only dive to about 560 mph and pull a similar load.
# at normal weight the designed limit load was 7.5 g positive and 3.5 g negative for the Corsair.
#  FIAT G.50 had an ultimate factor of 14 g. According to Dottore Eng. Gianni Cattaneo�s Profile booklet on the Macchi C.202, it had an ultimate factor of no less than 15.8 g! That would make it virtually indestructible. Also the Hawker Tempest was strong with its 14+ G strength.
# http://www.aviastar.org/air/japan/mitsubishi_a6m.php :
# Most Japanese fighters were designed to withstand a force of 7g. From 1932 all Japanese warplanes were required to meet a safety load factor of 1.8 so the limit for the A6M had to be 12.6g (1.8x7g).
#
# One flaw with FG's blackout/redout is that it instantaneous rather than
# delayed.  IE, even a WW1 pilot could probably withstand a short period of
# 8-10 Gs and blackout would happen gradually rather than instantly.

########################## setAttributes ##############################
#Set attributes for main aircraft
#  You can set vulnerabilities for any aircraft by
#  simply creating a file 'vulnerabilities.nas',
#  defining vulsObject as below, and including the line
#  bombable.setAttributes (attsObject);
# otherwsie, does not get called
var setAttributes = func (attsObject = nil) {
	debprint ("Bombable: Loading main aircraft vulnerability settings.");
	if (attsObject == nil) {
		attsObject = {
			
			# TODO: Update all below to be actual dimensions of that aircraft
			#########################################
			# DIMENSION DEFINITIONS
			#
			# All dimensions are in meters
			# source: http://en.wikipedia.org/wiki/Fairchild_Republic_A-10_Thunderbolt_II
			#
			dimensions : {
				width_m : 17.53,  #width of your object, ie, for aircraft, wingspan
				length_m : 16.26, #length of your object, ie, for aircraft, distance nose to tail
				height_m : 4.47, #height of your object, ie, for aircraft ground to highest point when sitting on runway
				
				damageRadius_m : 8, #typically 1/2 the longest dimension of the object. Hits within this distance of the
				#center of object have some possibility of damage
				vitalDamageRadius_m : 2, #typically the radius of the fuselage or cockpit or other most
				# vital area at the center of the object.  Always smaller than damageRadius_m
				
				crashRadius_m : 6, #It's a crash if the main aircraft hits in this area.
				
			},

			
			vulnerabilities: {
				engineDamageVulnerability_percent : 3,
				fireVulnerability_percent:15,
				damageVulnerability:20,
				fireExtinguishSuccess_percentage:10,
				fireExtinguishMaxTime_seconds:100,
				fireDamageRate_percentpersecond : .4,
				explosiveMass_kg : 20000,
				airspeed_damage:{
					#If we don't know the aircraft it could be anything, even the UFO.
					damage_threshold_kt: 200000,
					warning_threshold_kt: 8500,
					#1kt over the threshold for 1 second will add 1% damage:
					damage_multiplier_percentpersecond: 0.07
				},
				gforce_damage: {
					damage_enabled: 0,
					warning_enabled: 1,
					damage_threshold_g: {positive:30, negative:20},
					warning_threshold_g: {positive:12, negative:7},
					#1g over the threshold for 1 second will add 8% damage:
					damage_multiplier_percentpersecond: {positive:8, negative:8 }
				},
				redout: {
					enabled: 0,
					parameters: {
						blackout_onset_g: 4,
						blackout_complete_g: 7,
						redout_onset_g: -2,
						redout_complete_g: -5
					}
				}
			}
		}
	};
	
	
	
	#predefined values for a few aircraft we have set up for
	# dogfighting
	var aircraftname = getprop("sim/aircraft");
	if (string.match(aircraftname,"A6M2 * " ))
	{
		debprint ("Bombable: Loading A6M2 main aircraft vulnerabilities");
		attsObject = 
		{
			
			#########################################
			# DIMENSION DEFINITIONS
			#
			# All dimensions are in meters
			# source: http://en.wikipedia.org/wiki/Fairchild_Republic_A-10_Thunderbolt_II
			#
			dimensions : {
				width_m : 17.53,  #width of your object, ie, for aircraft, wingspan
				length_m : 16.26, #length of your object, ie, for aircraft, distance nose to tail
				height_m : 4.47, #height of your object, ie, for aircraft ground to highest point when sitting on runway
				
				damageRadius_m : 8, #typically 1/2 the longest dimension of the object. Hits within this distance of the
				#center of object have some possibility of damage
				vitalDamageRadius_m : 2, #typically the radius of the fuselage or cockpit or other most
				# vital area at the center of the object.  Always smaller than damageRadius_m
				
				crashRadius_m : 6, #It's a crash if the main aircraft hits in this area.
				
			},
			
			vulnerabilities: {
				engineDamageVulnerability_percent : 6,
				fireVulnerability_percent:34,
				fireDamageRate_percentpersecond : .4,
				damageVulnerability:90,
				fireExtinguishSuccess_percentage:50,
				fireExtinguishMaxTime_seconds:50,
				explosiveMass_kg : 27772,
				airspeed_damage:{
					damage_threshold_kt: 356, #http://en.wikipedia.org/wiki/A6M_Zero
					warning_threshold_kt: 325,
					#1 kt over the threshold for 1 second will add 1% damage:
					damage_multiplier_percentpersecond: 0.07
				},
				gforce_damage: {
					damage_enabled: 1,  #boolean yes/no
					warning_enabled: 1, #boolean yes/no
					damage_threshold_g: {positive:12.6, negative:9},
					warning_threshold_g: {positive:7, negative:6},
					damage_multiplier_percentpersecond: {positive:12, negative:12 }
				},
				redout: {
					enabled: 1,
					parameters: {
						blackout_onset_g: 5, #no g-suit in WWI so really 6gs is pushing it
						blackout_complete_g: 9,
						redout_onset_g: -2.5,
						redout_complete_g: -5
					}
				}
			}
		}
	
	} elsif ( string.match(aircraftname,"A-10 * " ) ) 
	{
		debprint ("Bombable: Loading A-10 main aircraft vulnerabilities");
		attsObject = 
		{
			#########################################
			# DIMENSION DEFINITIONS
			#
			# All dimensions are in meters
			# source: http://en.wikipedia.org/wiki/Fairchild_Republic_A-10_Thunderbolt_II
			#
			dimensions : {
				width_m : 17.53,  #width of your object, ie, for aircraft, wingspan
				length_m : 16.26, #length of your object, ie, for aircraft, distance nose to tail
				height_m : 4.47, #height of your object, ie, for aircraft ground to highest point when sitting on runway
				
				damageRadius_m : 8, #typically 1/2 the longest dimension of the object. Hits within this distance of the
				#center of object have some possibility of damage
				vitalDamageRadius_m : 2, #typically the radius of the fuselage or cockpit or other most
				# vital area at the center of the object.  Always smaller than damageRadius_m
				
				crashRadius_m : 6, #It's a crash if the main aircraft hits in this area.
				
			},
			
			vulnerabilities: {
				
				engineDamageVulnerability_percent : 6,
				fireVulnerability_percent:7,
				fireDamageRate_percentpersecond : .1,
				damageVulnerability:6,
				fireExtinguishSuccess_percentage:65,
				fireExtinguishMaxTime_seconds:80,
				explosiveMass_kg : 27772,
				airspeed_damage:{
					damage_threshold_kt: 480, # Never exceed speed, http://en.wikipedia.org/wiki/Fairchild_Republic_A-10_Thunderbolt_II
					warning_threshold_kt: 450,
					#1 kt over the threshold for 1 second will add 1% damage:
					damage_multiplier_percentpersecond: 0.5
				},
				gforce_damage: {
					damage_enabled: 1,
					warning_enabled: 1,
					damage_threshold_g: {positive:9, negative:9},
					warning_threshold_g: {positive:8, negative:8},
					damage_multiplier_percentpersecond: {positive:3, negative:3 }  # higher = weaker aircraft
					
				},
				redout: {
					enabled: 1,
					parameters: {
						blackout_onset_g: 7, #g-suit allows up to 9Gs, http://en.wikipedia.org/wiki/G-LOC
						blackout_complete_g: 12, #or even 10-12.  Maybe. http://forum.acewings.com/pop_printer_friendly.asp?ARCHIVE = true&TOPIC_ID = 3588
						redout_onset_g: -2,  #however, g-suit doesn't help with red-out.  Source: http://en.wikipedia.org/wiki/Greyout_(medical)
						redout_complete_g: -5
					}
				}
			}
		}
		
	} elsif ( string.match(aircraftname,"f6f * " ) ) 
	{
		debprint ("Bombable: Loading F6F Hellcat main aircraft vulnerabilities");
		attsObject = 
		{
			#########################################
			# DIMENSION DEFINITIONS
			#
			# All dimensions are in meters
			# source: http://en.wikipedia.org/wiki/Fairchild_Republic_A-10_Thunderbolt_II
			#
			dimensions : {
				width_m : 17.53,  #width of your object, ie, for aircraft, wingspan
				length_m : 16.26, #length of your object, ie, for aircraft, distance nose to tail
				height_m : 4.47, #height of your object, ie, for aircraft ground to highest point when sitting on runway
				
				damageRadius_m : 8, #typically 1/2 the longest dimension of the object. Hits within this distance of the
				#center of object have some possibility of damage
				vitalDamageRadius_m : 2, #typically the radius of the fuselage or cockpit or other most
				# vital area at the center of the object.  Always smaller than damageRadius_m
				
				crashRadius_m : 6, #It's a crash if the main aircraft hits in this area.
				
			},
			vulnerabilities: {

				engineDamageVulnerability_percent : 7,
				fireVulnerability_percent:15,
				fireDamageRate_percentpersecond : .5,
				damageVulnerability:3.5,
				fireExtinguishSuccess_percentage:23,
				fireExtinguishMaxTime_seconds:30,
				explosiveMass_kg : 735,
				airspeed_damage:{
					damage_threshold_kt: 450, #VNE, http://forums.ubi.com/eve/forums/a/tpc/f/23110283/m/46710245
					warning_threshold_kt: 420,
					#1 kt over the threshold for 1 second will add 1% damage:
					damage_multiplier_percentpersecond: 0.5
				},
				gforce_damage: {
					#data: http://www.amazon.com/Grumman-Hellcat-Pilots-Operating-Instructions/dp/1935327291/ref = sr_1_1?s = books&ie = UTF8&qid = 1319249394&sr = 1-1
					#see particularly p. 59
					#accel 'never exceed' limits are +7 and -3 Gs in all situations, and less in some situations
					damage_enabled: 1,  #boolean yes/no
					warning_enabled: 1, #boolean yes/no
					damage_threshold_g: {positive:15.6, negative:10}, # it's somewhat stronger built than the A6M2
					warning_threshold_g: {positive:12, negative:8},
					damage_multiplier_percentpersecond: {positive:12, negative:12 }
				},
				redout: {
					enabled: 1,
					parameters: {
						blackout_onset_g: 5, #no g-suit in WWI so really 6gs is pushing it
						blackout_complete_g: 9,
						redout_onset_g: -2.5,
						redout_complete_g: -5
					}
				}
			}
		}
		
	} elsif ( string.match(aircraftname," * sopwithCamel * " ) ) 
	{
		debprint ("Bombable: Loading SopwithCamel main aircraft vulnerabilities");
		attsObject = 
		{
			
			#########################################
			# DIMENSION DEFINITIONS
			#
			# All dimensions are in meters
			# source: http://en.wikipedia.org/wiki/Fairchild_Republic_A-10_Thunderbolt_II
			#
			dimensions : {
				width_m : 17.53,  #width of your object, ie, for aircraft, wingspan
				length_m : 16.26, #length of your object, ie, for aircraft, distance nose to tail
				height_m : 4.47, #height of your object, ie, for aircraft ground to highest point when sitting on runway
				
				damageRadius_m : 8, #typically 1/2 the longest dimension of the object. Hits within this distance of the
				#center of object have some possibility of damage
				vitalDamageRadius_m : 2, #typically the radius of the fuselage or cockpit or other most
				# vital area at the center of the object.  Always smaller than damageRadius_m
				
				crashRadius_m : 6, #It's a crash if the main aircraft hits in this area.
				
			},
			vulnerabilities: {
				engineDamageVulnerability_percent : 7,
				fireVulnerability_percent:15,
				fireDamageRate_percentpersecond : .5,
				damageVulnerability:3.5,
				fireExtinguishSuccess_percentage:23,
				fireExtinguishMaxTime_seconds:30,
				explosiveMass_kg : 735,
				airspeed_damage:{
					damage_threshold_kt: 240, #max speed, level flight is 100 kt, so this is a guess
					warning_threshold_kt: 210,
					#1 kt over the threshold for 1 second will add 1% damage:
					damage_multiplier_percentpersecond: 0.5
				},
				gforce_damage: {
					damage_enabled: 1,
					warning_enabled: 1,
					damage_threshold_g: {positive:4, negative:3},
					warning_threshold_g: {positive:3, negative:2.5},
					damage_multiplier_percentpersecond: {positive:12, negative:12 }
					
				},
				redout: {
					enabled: 1,
					parameters: {
						blackout_onset_g: 3,
						blackout_complete_g: 7,
						redout_onset_g: -2,
						redout_complete_g: -5
					}
				}
			}
		}
	} elsif ( string.match(aircraftname, " * spadvii * " )  ) 
	{
		
		debprint ("Bombable: Loading SPAD VII main aircraft vulnerabilities");
		attsObject = 
		{
			#########################################
			# DIMENSION DEFINITIONS
			#
			# All dimensions are in meters
			# source: http://en.wikipedia.org/wiki/Fairchild_Republic_A-10_Thunderbolt_II
			#
			dimensions : {
				width_m : 17.53,  #width of your object, ie, for aircraft, wingspan
				length_m : 16.26, #length of your object, ie, for aircraft, distance nose to tail
				height_m : 4.47, #height of your object, ie, for aircraft ground to highest point when sitting on runway
				
				damageRadius_m : 8, #typically 1/2 the longest dimension of the object. Hits within this distance of the
				#center of object have some possibility of damage
				vitalDamageRadius_m : 2, #typically the radius of the fuselage or cockpit or other most
				# vital area at the center of the object.  Always smaller than damageRadius_m
				
				crashRadius_m : 6, #It's a crash if the main aircraft hits in this area.
				
			},
			
			vulnerabilities: {
				engineDamageVulnerability_percent : 3,
				fireVulnerability_percent:20,
				fireDamageRate_percentpersecond : .2,
				damageVulnerability:4,
				fireExtinguishSuccess_percentage:10,
				fireExtinguishMaxTime_seconds:100,
				explosiveMass_kg : 735,
				airspeed_damage:{
					#The Spads and SE5's were quoted to dive "well in excess of 200mph" (I'can try to dig up the reference if you like) The Alb's and Nieuports were notorious for shedding lower wings in anything other than a normal dive.  http://www.theaerodrome.com/forum/2000/8286-maximum-dive-speed.html
					damage_threshold_kt: 290, #max speed, level flight is 103 kt, so this is a guess based on that plus Spad's rep as able to hold together in "swift dives" better than most
					warning_threshold_kt: 260,
					#1 kt over the threshold for 1 second will add 1% damage:
					damage_multiplier_percentpersecond: 0.4
				},
				gforce_damage: {
					damage_enabled: 1,
					warning_enabled: 1,
					#"swift dive" capability must mean it is a bit more structurally
					#   sound than camel/DR1
					damage_threshold_g: {positive:4.5, negative:3},
					warning_threshold_g: {positive:3, negative:2.5},
					damage_multiplier_percentpersecond: {positive:9, negative:9 }
					
				},
				redout: {
					enabled: 1,
					parameters: {
						blackout_onset_g: 3,
						blackout_complete_g: 7,
						redout_onset_g: -2,
						redout_complete_g:-5
					}
				}
			}
		}
	} elsif ( string.match(aircraftname," * fkdr * " ) ) 
	{
		debprint ("Bombable: Loading Fokker DR.1 main aircraft vulnerabilities");
		attsObject = 
		{
			#########################################
			# DIMENSION DEFINITIONS
			#
			# All dimensions are in meters
			# source: http://en.wikipedia.org/wiki/Fairchild_Republic_A-10_Thunderbolt_II
			#
			dimensions : {
				width_m : 17.53,  #width of your object, ie, for aircraft, wingspan
				length_m : 16.26, #length of your object, ie, for aircraft, distance nose to tail
				height_m : 4.47, #height of your object, ie, for aircraft ground to highest point when sitting on runway
				
				damageRadius_m : 8, #typically 1/2 the longest dimension of the object. Hits within this distance of the
				#center of object have some possibility of damage
				vitalDamageRadius_m : 2, #typically the radius of the fuselage or cockpit or other most
				# vital area at the center of the object.  Always smaller than damageRadius_m
				
				crashRadius_m : 6, #It's a crash if the main aircraft hits in this area.
				
			},
			
			vulnerabilities: {
				
				engineDamageVulnerability_percent : 3,
				fireVulnerability_percent:20,
				fireDamageRate_percentpersecond : .2,
				damageVulnerability:4,
				fireExtinguishSuccess_percentage:10,
				fireExtinguishMaxTime_seconds:100,
				explosiveMass_kg : 735,
				airspeed_damage:{
					damage_threshold_kt: 170, #max speed, level flight is 100 kt, so this is a guess based on that plus the DR1's reputation for wing damage at high speeds
					warning_threshold_kt: 155,
					#1 kt over the threshold for 1 second will add 1% damage:
					damage_multiplier_percentpersecond: 0.8
				},
				
				gforce_damage: {
					damage_enabled: 1,
					warning_enabled: 1,
					#wing breakage problems indicate weaker construction
					#    than SPAD VII, Sopwith Camel
					damage_threshold_g: {positive:3.8, negative:2.8},
					warning_threshold_g: {positive:3, negative:2.2},
					damage_multiplier_percentpersecond: {positive:14, negative:14 }
				},
				redout: {
					enabled: 1,
					parameters: {
						blackout_onset_g: 3,
						blackout_complete_g: 7,
						redout_onset_g: -2,
						redout_complete_g:-5
					}
				}
			}
		}
	}
	
	# rjw removed - now using attributes hash
	# props.globals.getNode(""~attributes_pp, 1).setValues(attsObject);

	attributes[""] = attsObject;
	# hash used by inc_loopid for loop counters
	attributes[""].loopids = { update_m_per_deg_latlon_loopid : 0 };
	attributes[""].damage = 0;
	attributes[""].exploded = 0;
	attributes[""].team = "A";
	attributes[""].side = 0;
	attributes[""].index = 0;
	attributes[""].targetIndex = [];
	attributes[""].shooterIndex = [];
	attributes[""].nFixed = 0; # not used
	attributes[""].nRockets = 0; # not used
	attributes[""].maxTargets = 0; # not used
	
	# We setmaxlatlon here so that it is re-done on reinit--otherwise we
	#get errors about maxlat being nil
	settimer (func { setMaxLatLon("", 500);}, 6.2398471);

	#put the redout properties in place, too; wait a couple of
	# seconds so we aren't overwritten by the redout.nas subroutine:
	settimer ( func {
		
		props.globals.getNode("/sim/rendering/redout/enabled", 1).setValue(attsObject.vulnerabilities.redout.enabled);
		props.globals.getNode("/sim/rendering/redout/parameters/blackout-onset-g", 1).setValue(attsObject.vulnerabilities.redout.parameters.blackout_onset_g);
		props.globals.getNode("/sim/rendering/redout/parameters/blackout-complete-g", 1).setValue(attsObject.vulnerabilities.redout.parameters.blackout_complete_g);
		props.globals.getNode("/sim/rendering/redout/parameters/redout-onset-g", 1).setValue(attsObject.vulnerabilities.redout.parameters.redout_onset_g);
		props.globals.getNode("/sim/rendering/redout/parameters/redout-complete-g", 1).setValue(attsObject.vulnerabilities.redout.parameters.redout_complete_g);
		
	}, 3);


	#reset the vulnerabilities for the main object whenever FG
	# reinits.
	# Important especially for setting redout/blackout, which otherwise
	# reverts to FG's defaults on reset.
	# We need to do it here so that if some outside aircraft
	# calls setVulnerabilities with its own attsObject
	# we will be able to use that here & reinit with that attsObject
	#
	attsSet = getprop (""~attributes_pp~"/attributes-set");
	if (attsSet == nil) attsSet = 0;
	if (attsSet == 0)
	{ 
		setlistener("/sim/signals/reinit", func 
		{
			setAttributes(attsObject)
		} 
		);
		
		#also set the default gforce/speed damage/warning enabled/disabled
		# but only on initial startup, not on reset
		if (getprop (GF_damage_menu_pp ~"/damage_enabled") == nil)
		props.globals.getNode(GF_damage_menu_pp ~"/damage_enabled", 1).setValue(attsObject.vulnerabilities.gforce_damage.damage_enabled);
		if (getprop (GF_damage_menu_pp ~"/warning_enabled") == nil)
		props.globals.getNode(GF_damage_menu_pp ~"/warning_enabled", 1).setValue(attsObject.vulnerabilities.gforce_damage.warning_enabled);
	}
	props.globals.getNode(""~attributes_pp~"/attributes-set", 1).setValue(1);
}


######################### startFire ###########################
#start a fire in a given location & associated with a given object
#
#
#A fire is different than the smoke, contrails, and flares below because
#when the fire is burning it adds damage to the object and eventually
#destroys it.
#
#object is given by "myNodeName" and directory path to the model in "model"
#Also sets the fire trigger on the object itself so it knows it is on fire
#and saves the name of the fire (model) node so the object can find
#the fire (model) it is associated with to update it etc.
#Returns name of the node with the newly started fire object (model)

var startFire = func (myNodeName = "", model = "") {
	#if (myNodeName == "") myNodeName = "/environment";
	#if there is already a fire going/associated with this object
	# then we don't want to start another
	var currFire = getprop(""~myNodeName~"/bombable/fire-particles/fire-particles-model");
	if ((currFire != nil) and (currFire != "")) {
		setprop(""~myNodeName~"/bombable/fire-particles/fire-burning", 1);
		return currFire;
	}
	
	
	if (model == nil or model == "") model = "AI/Aircraft/Fire-Particles/fire-particles.xml";
	var fireNode = put_tied_model(myNodeName, model);
	
	# if (myNodeName != "") type = props.globals.getNode(myNodeName).getName();
	#else type = "";
	#if (type == "multiplayer") mp_send_damage(myNodeName, 0);
	
	
	
	#var fire_node = geo.put_model("Models/Effects/Wildfire/wildfire.xml", lat, lon, alt * FT2M);
	#print ("started fire! ", myNodeName);
	
	#turn off the fire after user-set amount of time (default 1800 seconds)
	var burnTime = getprop ("/bombable/fire-particles/fire-burn-time");
	if (burnTime == 0 or burnTime == nil) burnTime = 1800;
	settimer (func {deleteFire(myNodeName,fireNode)}, burnTime);

	#name of this prop is "/models" + getname() + [ getindex() ]
	fireNodeName = "/models/" ~ fireNode.getName() ~ "[" ~ fireNode.getIndex() ~ "]";
	

	setprop(""~myNodeName~"/bombable/fire-particles/fire-burning", 1);
	setprop(""~myNodeName~"/bombable/fire-particles/fire-particles-model", fireNodeName);
	
	return fireNodeName; #we usually start with the name & then use props.globals.getNode(nodeName) to get the node object if necessary.
	#you can also use cmdarg().getPath() to get the full path from the node
	
}




########################## deleteSmoke ##########################
#Delete any of the various smoke, contrail, flare, etc. objects
#and unlink the fire from the smoke object.
#

var deleteSmoke = func (smokeType, myNodeName = "",fireNode = "") {
	
	#if (myNodeName == "") myNodeName = "/environment";
	
	if (fireNode == "") {

		fireNodeName = getprop(""~myNodeName~"/bombable/fire-particles/"~smokeType~"-particles-model");
		if (fireNodeName == nil) return;
		fireNode = props.globals.getNode(fireNodeName);
	}
	#remove the fire node/model altogether
	if (fireNode != nil) fireNode.remove();
	
	#turn off the object's fire trigger & unlink it from its fire model
	setprop(""~myNodeName~"/bombable/fire-particles/"~smokeType~"-burning", 0);
	setprop(""~myNodeName~"/bombable/fire-particles/"~smokeType~"-particles-model", "");

	#if (myNodeName != "") type = props.globals.getNode(myNodeName).getName();
	#else type = "";
	#if (type == "multiplayer") mp_send_damage(myNodeName, 0);
	
	
	

}


########################## startSmoke ##########################
# Smoke is like a fire, but doesn't cause damage & can use one of
# several different models to create different effects.
#
# smokeTypes are flare, smoketrail, pistonexhaust, contrail, damagedengine
#
# This func starts a flare in a given location & associated with a given object
# object is given by "myNodeName" and directory path to the model in "model"
# Also sets the fire burning flag on the object itself so it knows it is on fire
# and saves the name of the fire (model) node so the object can find
# the fire (model) it is associated with to update it etc.
# Returns name of the node with the newly started fire object (model)


var startSmoke = func (smokeType, myNodeName = "", model = "") {
	if (myNodeName == "") myNodeName = "";
	# if there is already smoke of this type going/associated with this object
	# then we don't want to start another
	var currFire = getprop(""~myNodeName~"/bombable/fire-particles/"~smokeType~"-particles-model");
	if ((currFire != nil) and (currFire != "")) return currFire;
	
	
	if (model == nil or model == "") model = "AI/Aircraft/Fire-Particles/"~smokeType~"-particles.xml";
	var fireNode = put_tied_model(myNodeName, model);
	
	
	#var fire_node = geo.put_model("Models/bombable/Wildfire/wildfire.xml", lat, lon, alt * FT2M);
	#debprint ("started fire! "~ myNodeName);
	
	# turn off the flare after user-set amount of time (default 1800 seconds)
	var burnTime = getprop (burntime1_pp~smokeType~burntime2_pp);
	if (burnTime == 0 or burnTime == nil) burnTime = 1800;
	# burnTime = -1 means leave it on indefinitely
	if (burnTime >= 0) settimer (func {deleteSmoke(smokeType, myNodeName,fireNode)}, burnTime);
	# rjw debug
	#debprint("smokeType = ",smokeType,"myNodeName = ",myNodeName,"fireNode = ",fireNode,"burnTime = ",burnTime);

	# name of this prop is "/models" + getname() + [ getindex() ]
	fireNodeName = "/models/" ~ fireNode.getName() ~ "[" ~ fireNode.getIndex() ~ "]";
	
	#if (myNodeName != "") type = props.globals.getNode(myNodeName).getName();
	#else type = "";
	#if (type == "multiplayer") mp_send_damage(myNodeName, 0);
	
	

	setprop(""~myNodeName~"/bombable/fire-particles/"~smokeType~"-burning", 1);
	setprop(""~myNodeName~"/bombable/fire-particles/"~smokeType~"-particles-model", fireNodeName);
	
	
	
	return fireNodeName; #we usually pass around the name & then use props.globals.getNode(nodeName) to get the node object if necessary.
}


####################################################
#reset damage & fires for main object
#
#
var reset_damage_fires = func  {
	
	deleteFire("");
	deleteSmoke("damagedengine", "");
	attributes[""].damage = 0;
	attributes[""].exploded = 0;
	# blow away the locks for MP communication--shouldn't really
	# be needed--but just a little belt & braces here
	# to make sure that no old damage (prior to the reset) is sent
	# to other aircraft again after the reset, and that none of the
	# locks are stuck.
	props.globals.getNode("/bombable").removeChild("locks",0);
	
	var msg_add = "";
	var msg = reset_msg();
	if (msg != "" and getprop(MP_share_pp) and getprop (MP_broadcast_exists_pp) ) {
		debprint ("Bombable RESET: MP sending: "~msg);
		mpsend(msg);
		msg_add = " and broadcast via multi-player";
	}
	
	
	debprint ("Bombable: Damage level & smoke reset for main object"~msg_add);

	var msg = "Your damage reset to 0%";
	mainStatusPopupTip (msg, 30);
	
}

########################## resetAllAIDamage ##########################
# 
# reset the damage, smoke & fires from all AI object with bombable operative
# TODO if an aircraft is crashing, it stays crashing despite this.
#

var revitalizeAllAIObjects = func (revitType = "aircraft", preservePosSpeed = 0) {

	ai = props.globals.getNode ("/ai/models").getChildren();
	
	#var m_per_deg_lat = getprop ("/bombable/sharedconstants/m_per_deg_lat");
	#var m_per_deg_lon = getprop ("/bombable/sharedconstants/m_per_deg_lon");
	
	# This will put the AI objects on a circle with 5000 meters radius
	# from the main a/c, at angle relocAngle_deg from the main a/c
	var relocAngle_deg = rand() * 360;
	var latPlusMinus = math.sin (relocAngle_deg * D2R) * (5000)/m_per_deg_lat;
	var lonPlusMinus = math.cos (relocAngle_deg * D2R) * (5000)/m_per_deg_lat;
	
	
	var min_dist_km = getprop(bomb_menu_pp~"dispersal-dist-min_km");
	if (typeof(min_dist_km) == "nil" or min_dist_km == "" or min_dist_km == 0) min_dist_km = 1;

	var max_dist_km = getprop(bomb_menu_pp~"dispersal-dist-max_km");
	if (typeof(max_dist_km) == "nil" or max_dist_km == "" or max_dist_km == 0 or max_dist_km < min_dist_km) max_dist_km = 16;
	
	setprop(bomb_menu_pp~"dispersal-dist-min_km",min_dist_km);
	setprop(bomb_menu_pp~"dispersal-dist-max_km",max_dist_km);
	
	var min_dist_m = 1000 * min_dist_km;
	var max_dist_m = 1000 * max_dist_km;
	
	#var latPlusMinus = 1; if (rand() > .5) latPlusMinus = -1;
	#var lonPlusMinus = 1; if (rand() > .5) lonPlusMinus = -1;
	var referenceLat = 0;
	var referenceLon = 0;
	var heading_deg = rand() * 360; #it's helpful to have them all going in the same
	#direction, in case AI piloting is turned off (they stay together rather than dispersing)
	var waitTime_sec = 0;
	var numRespawned = 0;
	var myNodeName = "";
	foreach (elem;ai) 
	{
		#only do this for the type named in the function call
		var type = elem.getName();
		if (type != revitType) continue;
		
		# aiName = type ~ "[" ~ elem.getIndex() ~ "]";
		myNodeName = "ai/models/" ~ type ~ "[" ~ elem.getIndex() ~ "]";
		
		#Disperse within a given radius
		if (preservePosSpeed == 2) 
		{
			# This will put the AI objects within a circle with 15000 meters radius
			# from the main a/c

			var dist = math.sqrt(rand()) * (max_dist_m - min_dist_m) + min_dist_m;
			var relocAngle_deg = rand() * 360;
			var latPlusMinus = math.sin (relocAngle_deg * D2R) * (dist)/m_per_deg_lat;
			var lonPlusMinus = math.cos (relocAngle_deg * D2R) * (dist)/m_per_deg_lat;

			var heading_deg = rand() * 360;
		}

		
		#only if bombable initialized
		#experimental: disable the next line to do this for ALL aircraft/objects regardless of bombable status.
		#OK, scenarios are only initialized (and the bombable routines started) if they are within a certain distance of the main a/c. So we DO need to remark
		#out the following line--otherwise distant scenarios are not respawned near
		#to the main a/c because they have not had bombable initialized yet
		#if (props.globals.getNode ( "/ai/models/"~aiName~"/bombable" ) == nil) continue;
		
		numRespawned += 1;
		#reset damage, smoke, fires for all objects that have bombable initialized
		#even does it for multiplayer objects, which is not completely proper (the MP bombable
		#keeps their 'real' damage total remotely), but might help in case of MP malfunction of some sort, and doesn't hurt in the meanwhile
		resetBombableDamageFuelWeapons ("/ai/models/" ~ aiName);
		
		setprop (myNodeName~"/controls/flight/target-pitch", 0);
		setprop (myNodeName~"/controls/flight/target-roll", 0);
		setprop (myNodeName~"/orientation/roll-deg", 0);

		#settimer & increased waittime helps avoid segfault that seems to happen
		#to FG too often when many models appear all at once
		#settimer ( func {
			
			
		
		newlat_deg = getprop ("/position/latitude-deg") + latPlusMinus;
		newlon_deg = getprop ("/position/longitude-deg") + lonPlusMinus;
		
		if (preservePosSpeed == 1)
		{
			var currLat = getprop (myNodeName~"/position/latitude-deg");
			var currLon = getprop (myNodeName~"/position/longitude-deg");
			var old_elev_ft = elev (currLat,currLon);
			
			if (referenceLat == 0 and referenceLon == 0) 
			{
				referenceLat = currLat;
				referenceLon = currLon;
			}
			newlat_deg = newlat_deg + currLat-referenceLat ;
			newlon_deg = newlon_deg + currLon-referenceLon;
		}
		else
		{
			newlat_deg = newlat_deg + (rand() - .5) * 500/m_per_deg_lat ;
			newlon_deg = newlon_deg + (rand() - .5) * 500/m_per_deg_lon;
		}
		
		setprop (myNodeName~"/position/latitude-deg",  newlat_deg );
		setprop (myNodeName~"/position/longitude-deg",  newlon_deg );
		var elev_ft = elev (newlat_deg,newlon_deg);
		var currAlt_ft = getprop (myNodeName~"/position/altitude-ft");
		
		if (type == "aircraft") 
		{
			if (preservePosSpeed == 1) 
			{
				alt_ft = currAlt_ft-old_elev_ft + elev_ft;
				if (alt_ft-500 < elev_ft) alt_ft = elev_ft+500;
			}
			else if (preservePosSpeed == 2)
			{
				var min_alt_ft = elev_ft+500;
				var main_alt_ft = getprop ("/position/altitude-ft");
				var max_alt_ft = main_alt_ft * 2;
				if (max_alt_ft < 2 * min_alt_ft) max_alt_ft = 2 * min_alt_ft;
				if (max_alt_ft < 10000) max_alt_ft = 16000;
				if (max_alt_ft > 45000) max_alt_ft = 45000;
				alt_ft = rand() * (max_alt_ft-min_alt_ft) + min_alt_ft;
			}
			else
			{
				alt_ft = getprop ("/position/altitude-ft")+100;
				if (alt_ft-500 < elev_ft) alt_ft = elev_ft+500;
			}
		}
		else
		{
			alt_ft = elev_ft;
		}
			
		setprop (myNodeName~"/position/altitude-ft", alt_ft);
		setprop (myNodeName~"/controls/flight/target-alt", alt_ft);

		if (preservePosSpeed == 0 or preservePosSpeed == 2) 
		{
			# rjw not needed since AI model flight lateral-mode control in "roll" - not "hdg" ?
			# setprop (myNodeName~"/controls/flight/target-hdg", heading_deg);
			setprop (myNodeName~"/orientation/true-heading-deg", heading_deg);
		}
		
		#setting these stops the relocate function from relocating them back
		setprop(myNodeName~"/position/previous/latitude-deg", newlat_deg);
		setprop(myNodeName~"/position/previous/longitude-deg", newlon_deg);
		setprop(myNodeName~"/position/previous/altitude-ft", alt_ft);
		
		var cart = geodtocart(newlat_deg, newlon_deg, alt_ft * FT2M); # lat/lon/alt(m)
		
		
		setprop(myNodeName~"/position/previous/global-x", cart[0]);
		setprop(myNodeName~"/position/previous/global-y", cart[1]);
		setprop(myNodeName~"/position/previous/global-z", cart[2]);
		
		# set the speed--if not preserving speed/position OR if speed is 0 (due to crashing etc)
		var currSpeed_kt = getprop (myNodeName~"/velocities/true-airspeed-kt");

		if (preservePosSpeed == 0 or currSpeed_kt == 0 ) 
		{
			var vels = attributes[myNodeName].velocities;
			var min_vel_kt = vels.minSpeed_kt;
			var cruise_vel_kt = vels.cruiseSpeed_kt;
			var attack_vel_kt = vels.attackSpeed_kt;
			var max_vel_kt = vels.maxSpeed_kt;
			
			#defaults
			if (type == "aircraft") 
			{
				if (min_vel_kt == nil or min_vel_kt < 1) min_vel_kt = 50;
				if (cruise_vel_kt == nil or cruise_vel_kt < 1) 
				{
					cruise_vel_kt = 2 * min_vel_kt;
					#they're at 82% to 102% of your current airspeed
					var vel = getprop ("/velocities/airspeed-kt") * (.82 + rand() * .2);
				}
				else
				{ 
					var vel = 0;
				}
				if (attack_vel_kt == nil or attack_vel_kt <= cruise_vel_kt) attack_vel_kt = 1.5 * cruise_vel_kt;
				if (max_vel_kt == nil or max_vel_kt <= attack_vel_kt) max_vel_kt = 1.5 * attack_vel_kt;
			}
			else
			{
				if (min_vel_kt == nil or min_vel_kt < 1) min_vel_kt = 10;
				if (cruise_vel_kt == nil or cruise_vel_kt < 1) 
				{
					cruise_vel_kt = 2 * min_vel_kt;
					var vel = 15;
				}
				else
				{
					var vel = 0;
				}
				if (attack_vel_kt == nil or attack_vel_kt <= cruise_vel_kt) attack_vel_kt = 1.5 * cruise_vel_kt;
				if (max_vel_kt == nil or max_vel_kt <= attack_vel_kt) max_vel_kt = 1.5 * attack_vel_kt;
			}
			debprint ("vel1:", vel);
			
			if (vel < min_vel_kt or vel == 0) vel = (attack_vel_kt-cruise_vel_kt) * rand() + cruise_vel_kt;
			if (vel > max_vel_kt) vel = max_vel_kt;
			
			debprint ("vel2:", vel);

			setprop (myNodeName~"/velocities/true-airspeed-kt", vel);
			setprop (myNodeName~"/controls/flight/target-spd", vel);
		}
	}
			
	if ( preservePosSpeed == 1) 
	{

		if (revitType == "aircraft") 
		{
			var msg = numRespawned ~ " AI Aircraft have damage reset and are about 5000 meters off, with their existing speed, direction, and altitude above ground level preserved";
			} else {
			var msg = numRespawned ~ " AI ground/water craft have damage reset and are about 5000 meters off";

		}
	} 
	else if ( preservePosSpeed == 2)
	{
		if (revitType == "aircraft") 
		{
			var msg = numRespawned ~ " AI Aircraft have damage reset and are at various locations and altitudes within about 15,000 meters";
			} else {
			var msg = numRespawned ~ " AI ground/water craft have damage reset and are at various locations within about 15,000 meters";
		}
	
	} 
	else
	{
		if (revitType == "aircraft") 
		{
			var msg = numRespawned ~ " AI Aircraft have damage reset and are at your altitude about 5000 meters off";
			} else {
			var msg = numRespawned ~ " AI ground/water craft have damage reset and are about 5000 meters off";
		}
	}
		
		#many times when the objects are relocated they initialize and
		# in doing so call reinit GUI.  This can cause a segfault if
		# we are in the middle of popping up our message.  So best to wait a while
		# before doing it . . .
		settimer ( func { targetStatusPopupTip (msg, 2);}, 13);
		debprint ("Bombable: " ~ msg);

}

######################## resetBombableDamageFuelWeapons ############################
# resetBombableDamageFuelWeapons
# reset the damage, smoke & fires from an AI aircraft, or the main aircraft
#  myNodeName = the AI node to reset, or set myNodeName = "" for the main
# #aircraft.

var resetBombableDamageFuelWeapons = func (myNodeName) {
	debprint ("Bombable: Resetting damage level and fires for ", myNodeName);
			
	#don't do this for objects that don't even have bombable initialized
	if (props.globals.getNode ( ""~myNodeName~"/bombable" ) == nil) return;
			
	if (myNodeName == "") 
	{
		#main aircraft
		reset_damage_fires();
	}
	else
	{
		#ai objects
		#refill fuel & weapons
		stores.fillFuel(myNodeName, 1);
		stores.fillWeapons (myNodeName, 1);
		deleteFire(myNodeName);
		deleteSmoke("damagedengine", myNodeName);
		startEngines(myNodeName);
		var ats = attributes[myNodeName];
		var ctrls = ats.controls;	
		ats.damage = 0;
		ats.exploded = 0;
		ctrls.damageAltAddCurrent_ft = 0;
		ctrls.damageAltAddCumulative_ft = 0;
		ctrls.onGround = 0;
		ctrls.stayInFormation = 1;
		ctrls.attackInProgress = 0;
		ctrls.dodgeInProgress = 0;
		ctrls.attackClimbDiveInProgress = 0;
		ctrls.attackClimbDiveTargetAGL_m = 0;
		ctrls.stalling = 0;
		ctrls.avoidCliffInProgress = 0;
		ctrls.groundLoopCounter = 0;
		if (ctrls.kamikase != 0) ctrls.kamikase = 1;
				
		# reset the pilot's abilities, giving them
		# a new personality when they come back alive
		var ability = math.pow (rand(), 1.5); 
		if (rand() > .5) ability = -ability;
		ctrls.pilotAbility = ability;

		setWeaponPowerSkill (myNodeName);							
		msg = "Damage reset to 0 for " ~ myNodeName;
		targetStatusPopupTip (msg, 2);
	}

}




####################################################
# resetAllAIDamage
# reset the damage, smoke & fires from all AI object with bombable operative

var resetAllAIDamage = func {

	ai = props.globals.getNode ("/ai/models").getChildren();
	foreach (elem;ai) {
		aiName = elem.getName() ~ "[" ~ elem.getIndex() ~ "]";
				
		#reset damage, smoke, fires for all objects that have bombable initialized
		#even does it for multiplayer objects, which is not completely proper (the MP bombable
		#keeps their 'real' damage total remotely), but might help in case of MP malfunction of some sort, and doesn't hurt in the meanwhile
		if (props.globals.getNode ( "/ai/models/"~aiName~"/bombable" ) != nil) {
			resetBombableDamageFuelWeapons ("/ai/models/" ~ aiName);
		}
				
	}
			
	msg = "Damage reset to 0 for all AI objects";
	targetStatusPopupTip (msg, 2);
	debprint ("Bombable: "~msg);
			

}

####################################################
# resetMainAircraftDamage
# reset the damage, smoke & fires from the main aircraft/object

var resetMainAircraftDamage = func {

	resetBombableDamageFuelWeapons ("");
			
	msg = "Damage reset to 0 for main aircraft - you'll need to turn on your magnetos/restart your engines";
	mainStatusPopupTip (msg, 2);
	debprint ("Bombable: "~msg);

}


############################################################

####################################################
#Add a new menu item to turn smoke on/off
#todo: need to integrate this into the menus rather than just
#arbitrarily adding it to menu[97]
#
#This function adds the dialog object to an actual GUI menubar item


var init_bombable_dialog = func () {
	#return; #gui prob
			
	#we set bomb_menuNum to -1 at initialization time.
	#On reinit & some other times, this routine will be called again
	#so if bomb_menuNum != -1 we know not to seek out another new menu number
	#Without this check, we'd get a new Bombable menu added each time FG reinits
	#or re-positions.
	if (bomb_menuNum == nil or bomb_menuNum == -1) {
		#find the next open menu number/kludge
		bomb_menuNum = 97; #the default
		for (var i = 0;i < 300;i += 1) {
			p = props.globals.getNode("/sim/menubar/default/menu["~i~"]");
			if ( typeof(p) == "nil" ) {
				bomb_menuNum = i;
				print ("Bombable: Found empty menu: " ~ i);
				break;
				} else {
				# var l = props.globals.getNode("/sim/menubar/default/menu["~i~"]/name");
				var n = p.getChild("name");
				if (typeof(n) != "nil" ) var l = n.getValue();
				print ("Bombable: Looking at menu found a " ~ typeof(l));
						
				#p = records.create_printable_summary(l);
				if (typeof(l) != "nil") mss = l else mss = "nothing at " ~ i;
				print ("Bombable: Looking @ menu found: " ~ mss);
				if ( typeof(l) != "nil" and l == "Bombable") { # aha, we've already set up the menu once before.  So just re-use it. This happens in FG 2016.x etc when the user re-inits.
					bomb_menuNum = i;
					print ("Bombable: Found existing Bombable menu; re-initing: " ~ i);
					break;
				}
			}
		}
	}
			
	#init the main bombable options menu
	#todo: figure out how to position it in the center of the screen or somewhere better
	dialog.init(0,0);
			
	#make the GUI menubar item to select the options menu
	props.globals.getNode ("/sim/menubar/default/menu["~bomb_menuNum~"]/enabled", 1).setBoolValue(1);
	props.globals.getNode ("/sim/menubar/default/menu["~bomb_menuNum~"]/name", 1).setValue("Bombable");
	props.globals.getNode ("/sim/menubar/default/menu["~bomb_menuNum~"]/item/enabled", 1).setBoolValue(1);
	#Note: the label must be distinct from all other labels in the menubar
	#or you will get duplicate functionality with the other menu item
	#sharing the same label
	props.globals.getNode ("/sim/menubar/default/menu["~bomb_menuNum~"]/item/label", 1).setValue("Bombable Options"); #must be unique name from all others in the menubar or they both pop up together
	props.globals.getNode ("/sim/menubar/default/menu["~bomb_menuNum~"]/item/binding/command", 1).setValue("nasal");
	props.globals.getNode ("/sim/menubar/default/menu["~bomb_menuNum~"]/item/binding/script", 1).setValue("bombable.dialog.create()");

	props.globals.getNode ("/sim/menubar/default/menu["~bomb_menuNum~"]/item[1]/label", 1).setValue("Bombable Statistics"); #must be unique name from all others in the menubar or they both pop up together
	props.globals.getNode ("/sim/menubar/default/menu["~bomb_menuNum~"]/item[1]/binding/command", 1).setValue("nasal");
	props.globals.getNode ("/sim/menubar/default/menu["~bomb_menuNum~"]/item[1]/binding/script", 1).setValue("bombable.records.display_results()");
			
			
			
	#reinit makes the property changes to both the GUI & input become active
	#the delay is to avoid a segfault under dev version of FlightGear, 2010/09/07
	#This just a workaround, a real fix would like:
	#  overwriting preferences.xml with a new one including a line like < menubar include = "Dialogs/bombable.xml"/ > '
	#Thx goes to user Citronnier for tracking this down
	#settimer (func {fgcommand("reinit")}, 15);
	#As of FG 2.4.0, a straight "reinit" leads to FG crash or the dreaded NAN issue
	#at least with some aircraft.  Reinit/gui (as below) gets around this problem.
	#fgcommand("reinit", props.Node.new({subsystem : "gui"}));
			
	#OK . . . per gui.nas line 63, this appears to be the right way to do this:
	fgcommand ("gui-redraw");
}
###################################### targetStatusPopupTip #########################################
		
var targetStatusPopupTip = func (msg, delay = 5, override = nil) {
	# remove oldest line from buffer
	var line2 = find("\n", tipMessageAI) + 1;
	tipMessageAI = substr(tipMessageAI, line2) ~ "\n" ~ msg;
	var tmpl = props.Node.new
	(
		{
		name : "PopTipTarget", 
		modal : 0,
		draggable : 1,
        width : 768,
        height : 84,
		y: 70,
		text : { x : 6, y: 60, label : tipMessageAI,},
		}
	);
	if (override != nil) tmpl.setValues(override);
			
	popdown(tipArgTarget);
	fgcommand("dialog-new", tmpl);
	fgcommand("dialog-show", tipArgTarget);

	currTimerTarget  +=  1;
	var thisTimerTarget = currTimerTarget;

	# Final argument is a flag to use "real" time, not simulated time
	settimer(func { if(currTimerTarget == thisTimerTarget) { popdown(tipArgTarget) } }, delay, 1);
}

###################################### mainStatusPopupTip #########################################

var mainStatusPopupTip = func (msg, delay = 10, override = nil) {
	# remove oldest line from buffer
	var line2 = find("\n", tipMessageMain) + 1;
	tipMessageMain = substr(tipMessageMain, line2) ~ "\n" ~ msg;
	var tmpl = props.Node.new({
		name : "PopTipSelf", 
		modal : 0, 
        width : 768,
        height : 84,
		y: 160,
		text : { x : 6, y: 60, label : tipMessageMain, padding : 6 }
	});
	if (override != nil) tmpl.setValues(override);
			
	popdown(tipArgSelf);
	fgcommand("dialog-new", tmpl);
	fgcommand("dialog-show", tipArgSelf);

	currTimerSelf  +=  1;
	var thisTimerSelf = currTimerSelf;

	# Final argument is a flag to use "real" time, not simulated time
	settimer(func { if(currTimerSelf == thisTimerSelf) { popdown(tipArgSelf) } }, delay, 1);
}

var popdown = func ( tipArg ) {
	#return; #gui prob
	fgcommand("dialog-close", tipArg);
}
		


###############################################################################
## Set up Bombable Menu to turn on/off contrails etc.
## Based on the WildFire configuration dialog,
## which is partly based on Till Bush's multiplayer dialog
## to start, do dialog.init(30,30); dialog.create();

var CONFIG_DLG = 0;

var dialog = {
	#################################################################
	init : func (x = nil, y = nil) {
		me.x = x;
		me.y = y;
		me.bg = [0, 0, 0, 0.3];    # background color
		me.fg = [[1.0, 1.0, 1.0, 1.0]];
		#
		# "private"
		me.title = "Bombable";
		me.basenode = props.globals.getNode("/bombable/fire-particles");
		me.dialog = nil;
		me.namenode = props.Node.new({"dialog-name" : me.title });
		me.listeners = [];
	},
	#################################################################
	create : func {
		if (me.dialog != nil)
		me.close();
		#return; #gui prob
		me.dialog = gui.Widget.new();
		me.dialog.set("name", me.title);
		if (me.x != nil)
		me.dialog.set("x", me.x);
		if (me.y != nil)
		me.dialog.set("y", me.y);

		me.dialog.set("layout", "vbox");
		me.dialog.set("default-padding", 0);
		var titlebar = me.dialog.addChild("group");
		titlebar.set("layout", "hbox");
		titlebar.addChild("empty").set("stretch", 1);
		titlebar.addChild("text").set("label", "Bombable Objects Settings");
		var w = titlebar.addChild("button");
		w.set("pref-width", 16);
		w.set("pref-height", 16);
		w.set("legend", "");
		w.set("default", 0);
		w.set("key", "esc");
		w.setBinding("nasal", "bombable.dialog.destroy(); ");
		w.setBinding("dialog-close");
		me.dialog.addChild("hrule");

		var buttonBar1 = me.dialog.addChild("group");
		buttonBar1.set("layout", "hbox");
		buttonBar1.set("default-padding", 10);
				
		lresetSelf = buttonBar1.addChild("button");
		lresetSelf.set("legend", "Reset Main Aircraft Damage");
		lresetSelf.set("equal", 1);
		lresetSelf.prop().getNode("binding[0]/command", 1).setValue("nasal");
		lresetSelf.prop().getNode("binding[0]/script", 1).setValue("bombable.resetMainAircraftDamage();");
		lresetSelf.prop().getNode("binding[1]/command", 1).setValue("nasal");
		lresetSelf.prop().getNode("binding[1]/script", 1).setValue("bombable.bombable_dialog_save();");
		lresetSelf.prop().getNode("binding[2]/command", 1).setValue("dialog-apply");
		lresetSelf.prop().getNode("binding[3]/command", 1).setValue("dialog-close");


		lresetAI = buttonBar1.addChild("button");
		lresetAI.set("legend", "Reset AI Objects Damage");
		lresetAI.prop().getNode("binding[0]/command", 1).setValue("nasal");
		lresetAI.prop().getNode("binding[0]/script", 1).setValue("bombable.resetAllAIDamage();");
		lresetAI.prop().getNode("binding[1]/command", 1).setValue("nasal");
		lresetAI.prop().getNode("binding[1]/script", 1).setValue("bombable.bombable_dialog_save();");
		lresetAI.prop().getNode("binding[2]/command", 1).setValue("dialog-apply");
		lresetAI.prop().getNode("binding[3]/command", 1).setValue("dialog-close");

		lresetAI = buttonBar1.addChild("button");
		lresetAI.set("legend", "Reset Scenario");
		lresetAI.prop().getNode("binding[0]/command", 1).setValue("nasal");
		lresetAI.prop().getNode("binding[0]/script", 1).setValue("bombable.resetScenario();");
		lresetAI.prop().getNode("binding[1]/command", 1).setValue("nasal");
		lresetAI.prop().getNode("binding[1]/script", 1).setValue("bombable.bombable_dialog_save();");
		lresetAI.prop().getNode("binding[2]/command", 1).setValue("dialog-apply");
		lresetAI.prop().getNode("binding[3]/command", 1).setValue("dialog-close");
				

		var buttonBar2 = me.dialog.addChild("group");
		buttonBar2.set("layout", "hbox");
		buttonBar2.set("default-padding", 10);

		#respawning often makes AI objects init or reinit, which sometimes
		# includes GUI reinit.  So we need to save/close the dialogue first
		# thing; otherwise segfault is likely
		lrevitAIAir = buttonBar2.addChild("button");
		lrevitAIAir.set("legend", "Respawn AI Aircraft Grouped Near You");
		lrevitAIAir.set("tooltip", "Place all AI Aircraft in a group near your location.");
				
		lrevitAIAir.prop().getNode("binding[0]/command", 1).setValue("nasal");
		lrevitAIAir.prop().getNode("binding[0]/script", 1).setValue("bombable.revitalizeAllAIObjects(\"aircraft\",0);");
		lrevitAIAir.prop().getNode("binding[1]/command", 1).setValue("nasal");
		lrevitAIAir.prop().getNode("binding[1]/script", 1).setValue("bombable.bombable_dialog_save();");
		lrevitAIAir.prop().getNode("binding[2]/command", 1).setValue("dialog-apply");
		lrevitAIAir.prop().getNode("binding[3]/command", 1).setValue("dialog-close");

		lrevitAIObj = buttonBar2.addChild("button");
		lrevitAIObj.prop().getNode("binding[0]/command", 1).setValue("nasal");
		lrevitAIObj.prop().getNode("binding[0]/script", 1).setValue("bombable.revitalizeAllAIObjects(\"ship\", 0);");
		lrevitAIObj.prop().getNode("binding[1]/command", 1).setValue("nasal");
		lrevitAIObj.prop().getNode("binding[1]/script", 1).setValue("bombable.bombable_dialog_save();");
		lrevitAIObj.set("legend", "Respawn AI Ground/Water Craft Grouped Near You");
		lrevitAIObj.prop().getNode("binding[2]/command", 1).setValue("dialog-apply");
		lrevitAIObj.prop().getNode("binding[3]/command", 1).setValue("dialog-close");

		var buttonBar3 = me.dialog.addChild("group");
		buttonBar3.set("layout", "hbox");
		buttonBar3.set("default-padding", 10);

		#respawning often makes AI objects init or reinit, which sometimes
		# includes GUI reinit.  So we need to save/close the dialogue first
		# thing; otherwise segfault is likely
		lrevitPAIAir = buttonBar3.addChild("button");
		lrevitPAIAir.set("legend", "Respawn AI Aircraft Near You, Preserve Relative Position");
				
		lrevitPAIAir.prop().getNode("binding[0]/command", 1).setValue("nasal");
		lrevitPAIAir.prop().getNode("binding[0]/script", 1).setValue("bombable.revitalizeAllAIObjects(\"aircraft\",1);");
		lrevitPAIAir.prop().getNode("binding[1]/command", 1).setValue("nasal");
		lrevitPAIAir.prop().getNode("binding[1]/script", 1).setValue("bombable.bombable_dialog_save();");
		lrevitPAIAir.prop().getNode("binding[2]/command", 1).setValue("dialog-apply");
		lrevitPAIAir.prop().getNode("binding[3]/command", 1).setValue("dialog-close");

		lrevitPAIObj = buttonBar3.addChild("button");
		lrevitPAIObj.prop().getNode("binding[0]/command", 1).setValue("nasal");
		lrevitPAIObj.prop().getNode("binding[0]/script", 1).setValue("bombable.revitalizeAllAIObjects(\"ship\", 1);");
		lrevitPAIObj.prop().getNode("binding[1]/command", 1).setValue("nasal");
		lrevitPAIObj.prop().getNode("binding[1]/script", 1).setValue("bombable.bombable_dialog_save();");

		lrevitPAIObj.set("legend", "Respawn AI Ground/Water Craft Near You, Preserve Relative Position");
		lrevitPAIObj.set("tooltip", "Respawn AI Ground/Water Craft Preserving Relative Position");
		lrevitPAIObj.prop().getNode("binding[2]/command", 1).setValue("dialog-apply");
		lrevitPAIObj.prop().getNode("binding[3]/command", 1).setValue("dialog-close");

		var buttonBar4 = me.dialog.addChild("group");
		buttonBar4.set("layout", "hbox");
		buttonBar4.set("default-padding", 10);
		lrevitDISAir = buttonBar4.addChild("button");
		lrevitDISAir.set("legend", "Respawn AI Aircraft Near You, Dispersed");
				
		lrevitDISAir.prop().getNode("binding[0]/command", 1).setValue("nasal");
		lrevitDISAir.prop().getNode("binding[0]/script", 1).setValue("settimer ( func {bombable.revitalizeAllAIObjects(\"aircraft\",2); }, 1);"); #settimer so there is time for the distance values to be put on the prop. tree
		lrevitDISAir.prop().getNode("binding[1]/command", 1).setValue("nasal");
		lrevitDISAir.prop().getNode("binding[1]/script", 1).setValue("bombable.bombable_dialog_save();");
		lrevitDISAir.prop().getNode("binding[2]/command", 1).setValue("dialog-apply");
		lrevitDISAir.prop().getNode("binding[3]/command", 1).setValue("dialog-close");

		lrevitDISObj = buttonBar4.addChild("button");
		lrevitDISObj.prop().getNode("binding[0]/command", 1).setValue("nasal");
		lrevitDISObj.prop().getNode("binding[0]/script", 1).setValue("settimer ( func {bombable.revitalizeAllAIObjects(\"ship\",2); }, 1);");
		lrevitDISObj.prop().getNode("binding[1]/command", 1).setValue("nasal");
		lrevitDISObj.prop().getNode("binding[1]/script", 1).setValue("bombable.bombable_dialog_save();");

		lrevitDISObj.set("legend", "Respawn AI Ground/Water Craft Near You, Dispersed");
		lrevitDISObj.set("tooltip", "Respawn AI Ground/Water Craft Preserving Relative Position");
		lrevitDISObj.prop().getNode("binding[2]/command", 1).setValue("dialog-apply");
		lrevitDISObj.prop().getNode("binding[3]/command", 1).setValue("dialog-close");
				
		var buttonBar5 = me.dialog.addChild("group");
		buttonBar5.set("layout", "hbox");
		buttonBar5.set("default-padding", 10);
		lrevitDISmin = buttonBar5.addChild("input");
		lrevitDISmin.set("label", "Min dispersal distance (km)");
		lrevitDISmin.set("property", bomb_menu_pp~"dispersal-dist-min_km");
		lrevitDISmin.set("default", "1");
		lrevitDISmax = buttonBar5.addChild("input");
		lrevitDISmax.set("label", "Max dispersal distance (km)");
		lrevitDISmax.set("property", bomb_menu_pp~"dispersal-dist-max_km");
		lrevitDISmax.set("default", "16");


		#        lresetAI = buttonBar1.addChild("button");
		#        lresetAI.set("legend", "Reset All Damage (Main & AI)");
		#        lresetAI.prop().getNode("binding[0]/command", 1).setValue("nasal");
		#        lresetAI.prop().getNode("binding[0]/script", 1).setValue("bombable.resetAllAIDamage();bombable.resetMainAircraftDamage();");

		me.dialog.addChild("hrule");


		var content = me.dialog.addChild("group");
		content.set("layout", "vbox");
		content.set("halign", "center");
		content.set("default-padding", 5);
				
				
		#triggers (-trigger) are the overall on/off flag for that type of fire/smoke globally in Bombable
		# burning (-burning) is the local flag telling whether the type of
		# fire/smoke is burning on that particle node/aircraft
		#
		foreach (var b; [["Bombable module enabled", bomb_menu_pp~"bombable-enabled", "checkbox"],
		["", "", "hrule"],
		["Weapon realism (your weapons)", bomb_menu_pp~"main-weapon-realism-combo", "combo", 300, ["Ultra-realistic", "Normal", "Easier", "Dead easy"]],
		#["AI aircraft can shoot at you", bomb_menu_pp~"ai-aircraft-weapons-enabled", "checkbox"],
		["AI Weapon effectiveness (AI aircraft's weapons)", bomb_menu_pp~"ai-weapon-power-combo", "combo", 300, ["Much more effective", "More effective", "Normal", "Less effective", "Much less effective", "Disabled (they can't shoot at you)"]],

		#["AI fighter aircraft maneuver and attack", bomb_menu_pp~"ai-aircraft-attack-enabled", "checkbox"],
		["AI aircraft flying/dogfighting skill", bomb_menu_pp~"ai-aircraft-skill-combo", "combo", 300, ["Very skilled", "Above average", "Normal", "Below average", "Unskilled", "Disabled (AI aircraft can't maneuver)"]],
		["Bombable-via-multiplayer enabled", MP_share_pp, "checkbox"],
		["Excessive acceleration/speed warnings", GF_damage_menu_pp~"warning_enabled", "checkbox"],
		["Excessive acceleration/speed damages aircraft", GF_damage_menu_pp~"damage_enabled", "checkbox"],
		["Weapon impact flack enabled", trigger1_pp~"flack"~trigger2_pp, "checkbox"],
		["AI weapon fire visual effect", trigger1_pp~"ai-weapon-fire-visual"~trigger2_pp, "checkbox"],
		["Fires/Explosions enabled", trigger1_pp~"fire"~trigger2_pp, "checkbox"],
		["Jet Contrails enabled", trigger1_pp~"jetcontrail"~trigger2_pp, "checkbox"],
		["Smoke Trails enabled", trigger1_pp~"smoketrail"~trigger2_pp, "checkbox"],
		["Piston engine exhaust enabled", trigger1_pp~"pistonexhaust"~trigger2_pp, "checkbox"],
		["Damaged engine smoke enabled", trigger1_pp~"damagedengine"~trigger2_pp, "checkbox"],
		["Flares enabled", trigger1_pp~"flare"~trigger2_pp, "checkbox"],
		["Skywriting enabled", trigger1_pp~"skywriting"~trigger2_pp, "checkbox"],
		#["Easy mode enabled (twice as easy to hit targets; AI aircraft do easier manuevers; may combine w/Super Easy)", bomb_menu_pp~"easy-mode", "checkbox"],
		#["Super Easy Mode (3X as easy to hit targets; damaged tripled; AI aircraft do yet easier manuevers)", bomb_menu_pp~"super-easy-mode", "checkbox"],
		["AI ground detection: Can be disabled to improve framerate when your AI scenarios are far above the ground", bomb_menu_pp~"ai-ground-loop-enabled", "checkbox"],

				
		#["AI Weapon Effectiveness", bomb_menu_pp~"ai-weapon-power", "slider", 200, 0, 100 ],
				
		["Print debug messages to console", bomb_menu_pp~"debug", "checkbox"]
		]
		) {
			var w = content.addChild(b[2]);
			w.node.setValues({"label"    : b[0],
				"halign"   : "left",
				"property" : b[1],
				# "width"    : "200",
						
			});
					
			if (b[2] == "select" or b[2] == "combo" or b[2] == "list" ){
						
				w.node.setValues({"pref-width"    : b[3],
				});
				foreach (var r; b[4]) {
					var newentry = w.addChild("value");
					newentry.node.setValue(r);
				}
			}
					
			if (b[2] == "slider"){
						
				w.node.setValues({"pref-width"    : b[3],
					"min" : b[4],
					"max" : b[5],
				});
						
						
			}
					

		}
		me.dialog.addChild("hrule");
				
		var buttonBar = me.dialog.addChild("group");
		buttonBar.set("layout", "hbox");
		buttonBar.set("default-padding", 10);
				
		lsave = buttonBar.addChild("button");
		lsave.set("legend", "Save");
		lsave.set("default", 1);
		lsave.set("equal", 1);
		lsave.prop().getNode("binding[0]/command", 1).setValue("dialog-apply");
		lsave.prop().getNode("binding[1]/command", 1).setValue("nasal");
		lsave.prop().getNode("binding[1]/script", 1).setValue("bombable.bombable_dialog_save();");
		lsave.prop().getNode("binding[2]/command", 1).setValue("dialog-close");

		lcancel = buttonBar.addChild("button");
		lcancel.set("legend", "Cancel");
		lcancel.set("equal", 1);
		lcancel.prop().getNode("binding[0]/command", 1).setValue("dialog-close");

		# Load button.
		#var load = me.dialog.addChild("button");
		#load.node.setValues({"legend"    : "Load Wildfire log",
		#                      "halign"   : "center"});
		#load.setBinding("nasal",
		#                "wildfire.dialog.select_and_load()");

		fgcommand("dialog-new", me.dialog.prop());
		fgcommand("dialog-show", me.namenode);
	},
	#################################################################
	close : func {
		#return; #gui prob
		fgcommand("dialog-close", me.namenode);
	},
	#################################################################
	destroy : func {
		CONFIG_DLG = 0;
		me.close();
		foreach(var l; me.listeners)
		removelistener(l);
		delete(gui.dialog, "\"" ~ me.title ~ "\"");
	},
	#################################################################
	show : func {
		#return; #gui prob
		if (!CONFIG_DLG) {
			CONFIG_DLG = 1;
			me.init();
			me.create();
		}
	},
	#################################################################
	select_and_load : func {
		var selector = gui.FileSelector.new
		(func (n) { CAFire.load_event_log(n.getValue()); },
		"Load Wildfire log",                    # dialog title
		"Load",                                 # button text
		[" * .xml"],                              # pattern for files
		SAVEDIR,                                # start dir
		"fire_log.xml");                        # default file name
		selector.open();
	}


};  
##################################### bombable_dialog_save ##########################################
# save button triggers the listeners on the menu prop tree

var bombable_dialog_save = func 
{
	debprint ("Bombable: iowriting, writing . . . ");
	io.write_properties(bombable_settings_file, ""~bomb_menu_pp);
	mirrorMenu(); #write to hash
}

##################################### init_bombable_dialog_listeners ##########################################

var init_bombable_dialog_listeners = func {
	#We replaced this scheme for writing the menu selections whenever they
	#are changed, to just using the 'save' button

	#what to do when any bombable setting is changed
	#setlistener(""~bomb_menu_pp, func {
				
		#the lock prevents the file from being written if we are setting/
		# changing menu values internally or setting menu defaults
		# We only want to save the menu properties when the *  * user *  * 
		# makes changes.
		#  debprint ("Bombable: iowriting, checking lock . . . ");
		#  if (!getprop(bomb_menu_save_lock)) {
			#      debprint ("Bombable: iowriting, writing . . . ");
			#      io.write_properties(bombable_settings_file, ""~bomb_menu_pp);
		#  }
				
	#},0,2);#0,0 means (0) don't do on initial startup and (2) call listener func
	# on change of any child value

	#set listener function for main weapon power menu item
	setlistener(""~bomb_menu_pp~"main-weapon-realism-combo", func {
				
		var weap_pow = ""~bomb_menu_pp~"main-weapon-realism-combo";
		var val = getprop(weap_pow);
				
		debprint ("Updating main weapon power combo . . . ");
				
		#"Realistic", "Easy", "Super Easy", "Super-Duper Easy"
		if (val == "Ultra-realistic") {
			setprop (bomb_menu_pp~"easy-mode", 0);
			setprop (bomb_menu_pp~"super-easy-mode", 0);
			} elsif (val == "Normal") {
			setprop (bomb_menu_pp~"easy-mode", 1);
			setprop (bomb_menu_pp~"super-easy-mode", 0);
			} elsif (val == "Dead easy") {
			setprop (bomb_menu_pp~"easy-mode", 1);
			setprop (bomb_menu_pp~"super-easy-mode", 1);
			} else { #value "Easier" is the default
			setprop (bomb_menu_pp~"easy-mode", 0);
			setprop (bomb_menu_pp~"super-easy-mode", 1);
		}
				
	},1,1);#0,0 means (1) do on initial startup and (1) call listener func only when value is changed
			
			
			
	#set listener function for main weapon power menu item
	setlistener(""~bomb_menu_pp~"ai-weapon-power-combo", func {
				
		debprint ("Updating ai weapon power combo . . . ");
				
				
		var weap_pow = ""~bomb_menu_pp~"ai-weapon-power-combo";
		var val = getprop(weap_pow);
				
		if (val == "Much more effective") {
			setprop (bomb_menu_pp~"ai-weapon-power", 1);
			setprop (bomb_menu_pp~"ai-aircraft-weapons-enabled", 1);
			} elsif (val == "More effective") {
			setprop (bomb_menu_pp~"ai-weapon-power", .8);
			setprop (bomb_menu_pp~"ai-aircraft-weapons-enabled", 1);
			} elsif (val == "Normal") {
			setprop (bomb_menu_pp~"ai-weapon-power", .6);
			setprop (bomb_menu_pp~"ai-aircraft-weapons-enabled", 1);
			} elsif (val == "Less effective") {
			setprop (bomb_menu_pp~"ai-weapon-power", .4);
			setprop (bomb_menu_pp~"ai-aircraft-weapons-enabled", 1);
			} elsif (val == "Disabled (they can't shoot at you)") {
			setprop (bomb_menu_pp~"ai-weapon-power", 0);
			setprop (bomb_menu_pp~"ai-aircraft-weapons-enabled", 0);
			} else { #value "Much less effective" is the default
			setprop (bomb_menu_pp~"ai-weapon-power", .2);
			setprop (bomb_menu_pp~"ai-aircraft-weapons-enabled", 1);
		}
				
				
	},1,1);#0,0 means (1) do on initial startup and (1) call listener func only when value is changed


	#set listener function for AI aircraft fighting skill menu item
	setlistener(""~bomb_menu_pp~"ai-aircraft-skill-combo", func {

		debprint ("Updating ai aircraft skill combo . . . ");
		var maneuv = ""~bomb_menu_pp~"ai-aircraft-skill-combo";
		var val = getprop(maneuv);
				
		#"Realistic", "Easy", "Super Easy", "Super-Duper Easy"
		if (val == "Very skilled") {
			setprop (bomb_menu_pp~"ai-aircraft-skill-level", 5);
			setprop (bomb_menu_pp~"ai-aircraft-attack-enabled", 1);
			} elsif (val == "Above average") {
			setprop (bomb_menu_pp~"ai-aircraft-skill-level", 4);
			setprop (bomb_menu_pp~"ai-aircraft-attack-enabled", 1);
			} elsif (val == "Below average") {
			setprop (bomb_menu_pp~"ai-aircraft-skill-level", 2);
			setprop (bomb_menu_pp~"ai-aircraft-attack-enabled", 1);
			} elsif (val == "Normal") {
			setprop (bomb_menu_pp~"ai-aircraft-skill-level", 3);
			setprop (bomb_menu_pp~"ai-aircraft-attack-enabled", 1);
			} elsif (val == "Disabled (AI aircraft can't maneuver)") {
			setprop (bomb_menu_pp~"ai-aircraft-skill-level", 0);
			setprop (bomb_menu_pp~"ai-aircraft-attack-enabled", 0);
			} else { #value "Unskilled" is the default
			setprop (bomb_menu_pp~"ai-aircraft-skill-level", 1);
			setprop (bomb_menu_pp~"ai-aircraft-attack-enabled", 1);
		}
				
	},1,1);#0,0 means (1) do on initial startup and (1) call listener func only when value is changed

}


##################################### setupBombableMenu ##########################################


var setupBombableMenu = func {
			
			
	init_bombable_dialog_listeners ();
			
	#main bombable module is enabled by default
	if (getprop (bomb_menu_pp~"bombable-enabled") == nil )
	props.globals.getNode(bomb_menu_pp~"bombable-enabled", 1).setBoolValue(1);
			
	#multiplayer mode enabled by default
	if (getprop (MP_share_pp) == nil )
	props.globals.getNode(MP_share_pp, 1).setBoolValue(1);
			
			
	#fighter attack turned on by default
	if (getprop (""~bomb_menu_pp~"ai-aircraft-attack-enabled") == nil )
	props.globals.getNode(""~bomb_menu_pp~"ai-aircraft-attack-enabled", 1).setIntValue(1);

			
	if (getprop (""~bomb_menu_pp~"ai-ground-loop-enabled") == nil )
	props.globals.getNode(""~bomb_menu_pp~"ai-ground-loop-enabled", 1).setIntValue(1);

			
	#set these defaults
	if (getprop (""~bomb_menu_pp~"main-weapon-realism-combo") == nil )
	props.globals.getNode(""~bomb_menu_pp~"main-weapon-realism-combo", 1).setValue("Much easier");
	if (getprop (""~bomb_menu_pp~"ai-weapon-power-combo") == nil )
	props.globals.getNode(""~bomb_menu_pp~"ai-weapon-power-combo", 1).setValue("Less effective");
	if (getprop (""~bomb_menu_pp~"ai-aircraft-skill-combo") == nil )
	props.globals.getNode(""~bomb_menu_pp~"ai-aircraft-skill-combo", 1).setValue("Unskilled");
			
	#debug default
	if (getprop (bomb_menu_pp~"debug") == nil )
	props.globals.getNode(bomb_menu_pp~"debug", 1).setIntValue(0);

	#flack is default off because it seems to sometimes cause FG crashes
	#Update now default on because it seems fine
	if (getprop (""~trigger1_pp~"flack"~trigger2_pp) == nil )
	props.globals.getNode(""~trigger1_pp~"flack"~trigger2_pp, 1).setBoolValue(1);

	#rjw skywriting to show rocket trajectory
	if (getprop (""~trigger1_pp~"skywriting"~trigger2_pp) == nil )
	props.globals.getNode(""~trigger1_pp~"skywriting"~trigger2_pp, 1).setBoolValue(1);

	if (getprop (""~trigger1_pp~"ai-weapon-fire-visual"~trigger2_pp) == nil )
	props.globals.getNode(""~trigger1_pp~"ai-weapon-fire-visual"~trigger2_pp, 1).setBoolValue(1);


			

	foreach (var smokeType; [
	["fire",88, 3600],
	["jetcontrail", 77, -1],
	["smoketrail", 55, -1],
	["pistonexhaust", 5, -1],
	["damagedengine",  55, -1],
	["flare",66,3600],
	["skywriting",55,-1],
	["blaze",13,-1],
	] ) 
	{
				
		# trigger is the overall flag for that type of smoke/fire
		# for Bombable as a whole
		# burning is the flag as to whether that effect is turned on
		# for the main aircraft
				
				
		props.globals.getNode(""~life1_pp~smokeType[0]~burning_pp, 1).setBoolValue(0);
		if (getprop (""~trigger1_pp~smokeType[0]~trigger2_pp) == nil )
		props.globals.getNode(""~trigger1_pp~smokeType[0]~trigger2_pp, 1).setBoolValue(1);
		props.globals.getNode(""~life1_pp~smokeType[0]~life2_pp, 1).setDoubleValue(smokeType[1]);
		props.globals.getNode(""~burntime1_pp~smokeType[0]~burntime2_pp, 1).setDoubleValue(smokeType[2]);
				
				
	}
			
	init_bombable_dialog();
	# the previously attempted " == nil" trick doesn't work because this io.read routine
	# leaves unchecked values as 'nil'
	# so we set our defaults first & then load the file.  Anything that wasn't set by
	# the file just remains as our default.
	#
	# Now, read the menu default file:
	debprint ("Bombable: ioreading . . . ");
	var target = props.globals.getNode("" ~ bomb_menu_pp);
	io.read_properties(bombable_settings_file, target);
	mirrorMenu();
}
######################## mirrorMenu #############################
# creates a mirror of the propTree in bombableMenu hash

var mirrorMenu = func()
{
	bombableMenu = props.globals.getNode(bomb_menu_pp, 0).getValues();
}

######################## calcPilotSkill #############################
# FUNCTION calcPilotSkill
# returns the skill level of the AI pilot
# adjusted for the pilot individual skill level,
# the current level of fuel and damage and whether part of
# defending or attacking team

var calcPilotSkill = func ( myNodeName ) {
	var ats = attributes[myNodeName];
	var ctrls = ats.controls;	
	#skill ranges 0-5; 0 = disabled, so 1-5;
	var skill = bombableMenu["ai-aircraft-skill-level"];
	if (skill == nil) skill = 1;

	# pilotAbility is a rand +/-1 in skill level per individual pilot
	# so skill ranges 0-6
	skill += ctrls.pilotAbility;
			
	#ability to manoeuvre goes down as attack fuel reserves are depleted
	var fuelLevel = stores.fuelLevel (myNodeName);
	if (fuelLevel < .2) skill  *=  fuelLevel / 0.2;
			
	#skill goes down to 0 as damage goes from 80% to 100%
	if (ats.damage > 0.8) skill  *=  (1 - ats.damage)/ 0.2;

	# give team B, the defending team, higher skills than team R and team W
	# if ((ats.team == "B") and (skill < 6)) skill += (6 - skill) / 2;
	# if (ats.team == "R") skill /= 2;

	return skill;
}

########################### trueAirspeed2indicatedAirspeed ###############################
# FUNCTION trueAirspeed2indicatedAirspeed
# Give a node name & true airspeed, returns the indicated airspeed
# (using the elevation of the AI object for the calculation)
#
# The formula IAS = TAS * (1 + .02 * alt/1000) is a rule-of-thumb
# approximation for IAS but about the best we can do in simple terms
# since we don't have the temperature or pressure of the AI aircraft
# current altitude easily available.
#
# TODO: We should really use IAS for more of the AI aircraft speed limits
# & calculations, but stall speed is likely most crucial.  For instance,
# VNE (max allowed speed) seems more related to TAS for most AC.

var trueAirspeed2indicatedAirspeed = func (myNodeName = "", trueAirspeed_kt = 0 ) {

	currAlt_ft = getprop(""~myNodeName~"/position/altitude-ft");
	return trueAirspeed_kt * ( 1 + .02 * currAlt_ft/1000);

}

######################### elev ###########################
#return altitude (in feet) of given lat/lon
# rjw: The first entry in the vector is the elevation (in meters) for the given point, and the second is a hash with information about the assigned material - very CPU-intensive
var elev = func (lat, lon) {

	var info = geodinfo(lat, lon);
			
	if (info != nil) {
		var alt_m = info[0];
		if (alt_m == nil) alt_m = 0;
		return alt_m * M2FT; #return the altitude in feet
	} else  return 0;
			
}
		
##################################### damage_msg ##########################################
# MP messages
# directly based on similar functions in wildfire.nas
#

var damage_msg = func (callsign, damageAdd, damageTotal, smoke = 0, fire = 0, messageType = 1) {
	if (!getprop(MP_share_pp)) return;
	if (!getprop (MP_broadcast_exists_pp)) return;
	if (!bombableMenu["bombable-enabled"] ) return;
			
	n = 0;
			
	#bits.switch(n,1, checkRange(smoke,0,1,0 ));  # !! makes sure it's a boolean value
	#bits.switch(n,2, checkRange(fire,0,1,0 )); #can send up to 7 bits in a byte this way

			
	msg = sprintf ("%6s", callsign) ~
	Binary.encodeByte(messageType) ~
	Binary.encodeDouble(damageAdd) ~
	Binary.encodeDouble(damageTotal) ~
	Binary.encodeByte(smoke) ~
	Binary.encodeByte(fire);
			
	#too many messages overwhelm the system.  So we set a lock & only send messages
	# every 5 seconds or so (lockWaitTime); at the end we send the final message
	# (which has the final damage percentage)
	# of any that were skipped in the meanwhile
	lockName = ""~callsign~messageType;
			
	lock = props.globals.getNode("/bombable/locks/"~lockName~"/lock", 1).getValue();
	if (lock == nil or lock == "") lock = 0;
			
	masterLock = props.globals.getNode("/bombable/locks/masterLock", 1).getValue();
	if (masterLock == nil or masterLock == "") masterLock = 0;
			
	currTime = systime();
			
	#We can send 1 message per callsign & per message type, per lockWaitTime
	# seconds.  It sets a lock to prevent messages being sent in the meanwhile.
	# It sets a timer to send a cumulative damage message at the end of the
	# lock time to give a single update for damage in the meanwhile.
	# As a failsafe it also saves system time in the lock & any new
	# damage messages coming through after that lockWaitTime seconds are
	# allowed to go forward.
	# For vitally important messages (like master reset) we can set a masterLock
	# and no other messages can go out during that time.
	# This is abit of a kludge.   For real we should queue up messages &
	# send them out at a rate no faster than say 1/2 as fast as the rate
	# mpreceive checks for new messages.
	if ((currTime - masterLock > masterLockWaitTime) and (lock == nil or lock == "" or lock == 0 or currTime - lock > lockWaitTime)) {
				
		lockNum = lockNum+1;
		props.globals.getNode("/bombable/locks/"~lockName~"/lock", 1).setDoubleValue(currTime);
		settimer (func {
			lock = getprop ("/bombable/locks/"~lockName~"/lock");
			msg2 = getprop ("/bombable/locks/"~lockName~"/msg");
			setprop ("/bombable/locks/"~lockName~"/lock", 0);
			setprop ("/bombable/locks/masterLock", 0);
			setprop ("/bombable/locks/"~lockName~"/msg", "");
			if (msg2 != nil and msg2 != ""){
				mpsend(msg2);
				debprint ("Bombable: Sending delayed message "~msg);
			}
		}, lockWaitTime);
				
		return msg
				
				
		} else {
		setprop ("/bombable/locks/"~lockName~"/msg", msg);
		return nil;
	}

}

######################################## reset_msg #######################################
# reset_msg - part of MP messages
#

var reset_msg = func () {
	if (!getprop(MP_share_pp)) return "";
	if (!getprop (MP_broadcast_exists_pp)) return "";
	if (!bombableMenu["bombable-enabled"] ) return;
			
	n = 0;
			
	#bits.switch(n,1, checkRange(smoke,0,1,0 ));  # !! makes sure it's a boolean value
	#bits.switch(n,2, checkRange(fire,0,1,0 )); #can send up to 7 bits in a byte this way
			
	callsign = getprop ("/sim/multiplay/callsign");
	props.globals.getNode("/bombable/locks/masterLock", 1).setDoubleValue(systime());
	#messageType = 2 is the reset message
	return sprintf ("%6s", callsign) ~
	Binary.encodeByte(2);
}

######################################## parse_msg #######################################
var parse_msg = func (source, msg) {
	if (!getprop(MP_share_pp)) return;
	if (!getprop (MP_broadcast_exists_pp)) return;
	if (!bombableMenu["bombable-enabled"] ) return;
	debprint("Bombable: typeof source: ", typeof(source));
	debprint ("Bombable: source: ", source, " msg: ",msg);
	var ourcallsign = getprop ("/sim/multiplay/callsign");
	var p = 0;
	var msgcallsign = substr(msg, 0, 6);
	p = 6;
			
	var type = Binary.decodeByte(substr(msg, p));
	p  +=  Binary.sizeOf["byte"];
	#debprint ("msgcallsign:"~ msgcallsign," type:"~ type);
			
	#not our callsign and type != 2, we ignore it & return (type = 2 broadcasts to
	# * everyone * that their callsign is re-setting, so we always listen to that)
	if ((sprintf ("%6s", msgcallsign) != sprintf ("%6s", ourcallsign)) and
	type != 2 and type != 3 ) return;
			
			
			
	#damage message
	if (type == 1) {
		var damageAdd = Binary.decodeDouble(substr(msg, p));
		p  +=  Binary.sizeOf["double"];
		var damageTotal = Binary.decodeDouble(substr(msg, p));
		p  +=  Binary.sizeOf["double"];
		var smokeStart = Binary.decodeByte(substr(msg, p));
		p  +=  Binary.sizeOf["byte"];
		var fireStart = Binary.decodeByte(substr(msg, p));
		p  +=  Binary.sizeOf["byte"];
				
		debprint ("damageAdd:",damageAdd," damageTotal:",damageTotal," smoke:",smokeStart," fire:", fireStart);
				
		mainAC_add_damage (damageAdd, damageTotal, "weapons", "Hit by weapons!" );
				
	}
			
	#reset message for callsign
	elsif (type == 2) {
				
		#ai_loc = "/ai/models";
		#var mp_aircraft = props.globals.getNode(ai_loc).getChildren("multiplayer");
		#foreach (mp;mp_aircraft) { #mp is the node of a multiplayer AI aircraft
					
			#    mp_callsign = mp.getNode("callsign").getValue();
			#    mp_childname = mp.getName();
			#    mp_index = mp.getIndex();
			#    mp_name = ai_loc~"/"~mp_childname~"["~mp_index~"]";
			#    mp_path = cmdarg().getPath(mp);
			#    debprint ("Bombable: mp_path = " ~mp_path);
					
			mp_name = source;
			debprint ("Bombable: Resetting fire/damage for - name: ", source, " callsign: "~string.trim(msgcallsign) );
					
			#    if (sprintf ("%6s", mp_callsign) == sprintf ("%6s", msgcallsign)) {
						
				#blow away the locks for MP communication--shouldn't really
				# be needed--just a little belt & suspendors things here
				# to make sure that no old damage (prior to the reset) is sent
				# to the aircraft again after the reset, and that none of the
				# locks are stuck.
				props.globals.getNode("/bombable").removeChild("locks",0);
				resetBombableDamageFuelWeapons(source);
				msg = string.trim(msgcallsign)~" is resetting; damage reset to 0% for "~string.trim(msgcallsign);
				debprint ("Bombable: "~msg);
				targetStatusPopupTip (msg, 30);
						
						
			#  }
					
		#}
				


	}
	#update of callsign's current damage, smoke, fire situation
	elsif (type == 3) {

		#  ai_loc = "/ai/models";
		#var mp_aircraft = props.globals.getNode(ai_loc).getChildren("multiplayer");
		#foreach (mp;mp_aircraft) { #mp is the node of a multiplayer AI aircraft
					
			#    mp_callsign = mp.getNode("callsign").getValue();
			#    mp_childname = mp.getName();
			#   mp_index = mp.getIndex();
			#    mp_name = ai_loc~"/"~mp_childname~"["~mp_index~"]";
			#    mp_path = cmdarg().getPath(mp);
					
					
			#    if (sprintf ("%6s", mp_callsign) == sprintf ("%6s", msgcallsign)) {
				debprint ("Bombable: Updating fire/damage from - name: ", source ," callsign: "~string.trim(msgcallsign) );
				var damageAdd = Binary.decodeDouble(substr(msg, p));
				p  +=  Binary.sizeOf["double"];
				var damageTotal = Binary.decodeDouble(substr(msg, p));
				p  +=  Binary.sizeOf["double"];
				var smokeStart = Binary.decodeByte(substr(msg, p));
				p  +=  Binary.sizeOf["byte"];
				var fireStart = Binary.decodeByte(substr(msg, p));
				p  +=  Binary.sizeOf["byte"];
						
				mp_update_damage (source, damageAdd, damageTotal, smokeStart, fireStart, msgcallsign );
						
			#}
					
		# }



	}
	elsif (type == 4) {
		var pos   = Binary.decodeCoord(substr(msg, 6));
		var radius = Binary.decodeDouble(substr(msg, 36));
		resolve_foam_drop(pos, radius, 0, 0);
	}
}

######################### fire_loop ###########################
# timer function, every 1.5 to 2.5 seconds, adds damage if on fire
# TODO: This seems to be causing stutters.  We can separate out a separate
# loop to update the fire sizes and probably do some simplification of the
# add_damage routines.
#
var fire_loop = func(id, myNodeName = "") {
	if (myNodeName == "") myNodeName = "";
	id == attributes[myNodeName].loopids.fire_loopid or return;
			
	#Set the timer function here at the top
	#   so if there is some runtime error in the code
	#   below the timer function still continues to run
	var fireLoopUpdateTime_sec = 3;
	# add rand() so that all objects don't do this function simultaneously
	#debprint ("fire_loop starting");
	settimer(func { fire_loop(id, myNodeName); }, fireLoopUpdateTime_sec - 0.5 + rand());
			
	node = props.globals.getNode(myNodeName);
	type = node.getName();
			
	if(getprop(""~myNodeName~"/bombable/fire-particles/fire-burning")) {
		var myFireNodeName = getprop(""~myNodeName~"/bombable/fire-particles/fire-particles-model");
				

		# One single property controls the startsize & endsize
		# of ALL fire-particles active at one time. This is a bit fakey but saves on processor time.
		# The idea here is to change
		# the values of the start/endsize randomly and fairly quickly so the
		# various smoke columns don't all look like clones of each other
		# each smoke column only puts out particles 2X per second so
		# if the sizes are changed more often than that they can affect only
		# some of the smoke columns independently.
		var smokeEndsize = rand() * 100 + 50;
		setprop ("/bombable/fire-particles/smoke-endsize", smokeEndsize);
				
		var smokeEndsize = rand() * 125 + 60;
		setprop ("/bombable/fire-particles/smoke-endsize-large", smokeEndsize);
				
		var smokeEndsize = rand() * 75 + 33;
		setprop ("/bombable/fire-particles/smoke-endsize-small", smokeEndsize);
				
		var smokeEndsize = rand() * 25 + 9;
		setprop ("/bombable/fire-particles/smoke-endsize-very-small", smokeEndsize);
				
				
		var smokeStartsize = rand() * 10 + 5;
				
		#occasionally make a really BIG explosion
		if (rand() < .02/fireLoopUpdateTime_sec)  {
					
			settimer (func {setprop ("/bombable/fire-particles/smoke-startsize", smokeStartsize); }, 0.1);#turn the big explosion off quickly so it only affects a few of the fires for a moment--they put out smoke particles 4X/second
			smokeStartsize = smokeStartsize * rand() * 15 + 100; #make the occasional really big explosion
		}
				
		setprop ("/bombable/fire-particles/smoke-startsize", smokeStartsize);
		setprop ("/bombable/fire-particles/smoke-startsize-small", smokeStartsize * (rand()/2 + 0.5));
		setprop ("/bombable/fire-particles/smoke-startsize-very-small", smokeStartsize * (rand()/8 + 0.2));
		setprop ("/bombable/fire-particles/smoke-startsize-large", smokeStartsize * (rand() * 4 + 1));
				
		damageRate_percentpersecond = attributes[myNodeName].vulnerabilities.fireDamageRate_percentpersecond;
				
		if (damageRate_percentpersecond == nil) damageRate_percentpersecond = 0;
		if (damageRate_percentpersecond == 0) damageRate_percentpersecond = 0.1;
				
		# The object is burning, so we regularly add damage.
		# Have to do it differently if it is the main aircraft ("")
		if (myNodeName == "") {
			mainAC_add_damage( damageRate_percentpersecond / 100 * fireLoopUpdateTime_sec,0, "fire", "Fire damage!" );
		}
		#we don't add damage to multiplayer--we let the remote object do it & send
		#  it back to us
		else {
			if (type != "multiplayer") add_damage( damageRate_percentpersecond/100 * fireLoopUpdateTime_sec, "nonweapon", myNodeName );
		}
	}
			

}

############################### hitground_stop_explode ###########################
# Puts myNodeName right at ground level, explodes, sets up
# for full damage & onGround trigger to make it stop real fast now
# rjw in original code this function was only called for aircraft. var onGround is only set for aircraft 
# function will be called several times until exploded flag set

var hitground_stop_explode = func (myNodeName, alt) {
	var ats = attributes[myNodeName];
	var vuls = ats.vulnerabilities;
	var ctrls = ats.controls;
			

	startFire( myNodeName ); #if it wasn't on fire before it is now
	setprop (""~myNodeName~"/position/altitude-ft",  alt  );
	ctrls.onGround = 1; #this affects the slow-down system which is handled by add-damage, and will stop any forward movement very quickly
	add_damage(1, "nonweapon", myNodeName);  #and once we have buried ourselves in the ground we are surely dead; this also will stop any & all forward movement
	killEngines(myNodeName);
	stopDodgeAttack(myNodeName);	

			
	# check if this object has exploded already
			
	# if not, explode for ~3 seconds
	if ( !ats.exploded )
	{
		# and we cover our tracks by making a really big explosion momentarily
		# if it hit the ground that hard it's justified, right?
		if (vuls.explosiveMass_kg < 0) vuls.explosiveMass_kg = 1;
		lnexpl = math.ln (vuls.explosiveMass_kg / 10);
		var smokeStartsize = rand() * lnexpl * 20 + 30;
		setprop ("/bombable/fire-particles/smoke-startsize", smokeStartsize);
		setprop ("/bombable/fire-particles/smoke-startsize-small", smokeStartsize * (rand() / 2 + 0.5));
		setprop ("/bombable/fire-particles/smoke-startsize-very-small", smokeStartsize * (rand() / 8 + 0.2));
		setprop ("/bombable/fire-particles/smoke-startsize-large", smokeStartsize * (rand() * 4 + 1));
				
		# explode for, say, 3 seconds but then we're done for this object
		settimer (   func { ats.exploded = 1; }, 3 + rand() );
	}

}

############################### addAltitude_ft ###########################

var addAltitude_ft = func  (myNodeName, altAdd_ft = 40 , time = 1 ) {
			
	var loopTime = 0.033;
			
	var elapsed = getprop(""~myNodeName~"/position/addAltitude_elapsed");
	if (elapsed == nil) elapsed = 0;
	elapsed +=  loopTime;
	setprop(""~myNodeName~"/position/addAltitude_elapsed", elapsed);
			
			
			
	currAlt_ft = getprop (""~myNodeName~"/position/altitude-ft");
			
	#if (elapsed == 0) setprop (""~myNodeName~"/position/addAltitude_starting_alt_ft", currAlt_ft )
	#else var startAlt_ft = getprop (""~myNodeName~"/position/addAltitude_starting_alt_ft");
			
	setprop (""~myNodeName~"/position/altitude-ft", currAlt_ft+altAdd_ft * loopTime/time);
			
	if (elapsed < time) settimer (func { addAltitude_ft (myNodeName,altAdd_ft,time)}, loopTime);
			
	else setprop(""~myNodeName~"/position/addAltitude_elapsed", 0 );

}


################## setVerticalSpeed ####################
# FUNCTION setVerticalSpeed
# Changes to the new target vert speed but gradually over a few steps
# using settimer
#
var setVerticalSpeed = func (myNodeName, targetVertSpeed_fps = 70, maxChange_fps = 25, iterations = 4, time = .05, targetAirSpeed_kt = 0, maxChangeAirSpeed_kt = 0) {

	#give the vertical speed a boost
	var curr_vertical_speed_fps = getprop (""~myNodeName~"/velocities/vertical-speed-fps");
	var new_vertical_speed_fps = checkRange (targetVertSpeed_fps, curr_vertical_speed_fps-maxChange_fps, curr_vertical_speed_fps+maxChange_fps, targetVertSpeed_fps);
	setprop (""~myNodeName~"/velocities/vertical-speed-fps",  new_vertical_speed_fps);
			
	# now do the same to the airspeed
	if (targetAirSpeed_kt > 0 ) {
		var curr_airspeed_kt = getprop (""~myNodeName~"/velocities/true-airspeed-kt");
		var new_airspeed_kt = checkRange (targetAirSpeed_kt, curr_airspeed_kt, curr_airspeed_kt + maxChangeAirSpeed_kt, targetAirSpeed_kt);
		setprop (""~myNodeName~"/velocities/true-airspeed-kt",  new_airspeed_kt);
	}
			
	iterations  -= 1;
			
	if (iterations > 0) {
		settimer (func {
			setVerticalSpeed (myNodeName, targetVertSpeed_fps, maxChange_fps, iterations, time);
		} , time);
	}

}

######################### ground_loop ##########################
# ground_loop
# timer function, every (0.5 to 1.5 * updateTime_s) seconds, to keep object at
# ground level
# or other specified altitude above/below ground level, and at a
# reasonable-looking pitch. length_m & width_m are distances (in meters)
# needed to clear the object and find open earth on either side and front/back.
# damagealtadd is the total amount to add to the altitude above ground level (in meters) as
# the object becomes damaged, usually it is negative -- say a sinking ship or tires flattening on a
# vehicle.
# damageAltMaxRate is the max rate to allow the object to rise or sink
# as it becomes disabled
# TODO: This is one of the biggest framerate sucks in Bombable.  It can probably
# be optimized in many ways.

# rjw: the ground_loop affects the descent of aircraft. It initialises slowly, 
# e.g. possible for aircraft to enter a crash sequence before ground_loop is first called
# the ground_loop attempts to control descent of aircraft for high damage values, so does aircraftCrashControl

var ground_loop = func( id, myNodeName ) {
	id == attributes[myNodeName].loopids.ground_loopid or return;

	var updateTime_s = attributes[myNodeName].updateTime_s * (0.9 + 0.2 * rand());
	var ats = attributes[myNodeName];
	var ctrls = ats.controls;	
	if (ats.exploded == 1) return();
			
	# reset the timer loop first so we don't lose it entirely in case of a runtime
	# error or such
	# add rand() so that all objects don't do this function simultaneously
	settimer(func { ground_loop(id, myNodeName)}, updateTime_s );

	# Allow this function to be disabled via menu since it can kill framerate at times
	if (! bombableMenu["ai-ground-loop-enabled"] or ! bombableMenu["bombable-enabled"] ) return;

	var type = ats.type;

	var alts = ats.altitudes;
	var dims = ats.dimensions;
	var vels = ats.velocities;

	ctrls.groundLoopCounter += 1;	
	# rjw used to control debug printing and roll calculation
	var thorough = (math.fmod(ctrls.groundLoopCounter , 5) == 0); # to save FR we only do it thoroughly sometimes
	if (ctrls.onGround) thorough = 0; #never need thorough when crashed
	
	
			
	# If you get too close in to the object, FG detects the elevation of the top of the object itself
	# rather than the underlying ground elevation. So we go an extra FGAltObjectPerimeterBuffer_m
	# meters out from the object
	# just to be safe.  Otherwise objects climb indefinitely, always trying to get on top of themselves
	# Sometimes needed in _m, sometimes _ft, so we need both . . .
	var FGAltObjectPerimeterBuffer_m = 0.5 * dims.length_m;
	var FGAltObjectPerimeterBuffer_ft = FGAltObjectPerimeterBuffer_m * M2FT;
			
			
	# Update altitude to keep moving objects at the local ground level
	var currAlt_ft = getprop(""~myNodeName~"/position/altitude-ft"); #where the object is, in feet
	var lat = getprop(""~myNodeName~"/position/latitude-deg");
	var lon = getprop(""~myNodeName~"/position/longitude-deg");
	var heading = getprop(""~myNodeName~"/orientation/true-heading-deg");
	var speed_kt = getprop(""~myNodeName~"/velocities/true-airspeed-kt");
	var distance_til_update_ft = speed_kt * KT2FPS * updateTime_s;
	var damageValue = ats.damage;
	var damageAltAddPrev_ft = ctrls.damageAltAddCurrent_ft;
			
	if (lat == nil) {
		lat = 0;
		debprint ("Bombable: Lat = NIL, ground_loop ", myNodeName);
	}
	if (lon == nil) {
		lon = 0;
		debprint ("Bombable: Lon = NIL, ground_loop ", myNodeName);
	}

	var pitchangle_deg = 0;
	var rollangle_deg = 0;
	var pitchangle1_deg = 0;
	var frontBack_m = dims.length_m / 2 + FGAltObjectPerimeterBuffer_m;
	var leftRight_m = dims.width_m / 2 + FGAltObjectPerimeterBuffer_m;		
	var frontBack_ft = frontBack_m * M2FT;
	var leftRight_ft = leftRight_m * M2FT;	
			
	# calculate the altitude behind & ahead of the object, this determines the pitch angle and helps determine the overall ground level at this spot
	# Go that extra amount, FGAltObjectPerimeterBuffer_m, out from the actual length to keep FG from detecting the top of the
	# object as the altitude.  We need ground altitude here.
	# You can't just ask for elev at the object's current position or you'll get
	# the elev at the top of the object itself, not the ground . . .
	# rjw true for scenery objects - check for AI

	var GeoCoord = geo.Coord.new();
	GeoCoord.set_latlon(lat, lon);
	var alt_ft = elev (GeoCoord.lat(), GeoCoord.lon()  ); #in feet
	# assume lat, lon are at the centre of the object
	#debprint ("Bombable: GeoCoord.apply_course_distance(heading, dims.length_m/2); ",heading, " ", dims.length_m/2 );
	GeoCoord.apply_course_distance(heading, frontBack_m);    #frontreardist in meters
	var toFrontAlt_ft = elev ( GeoCoord.lat(), GeoCoord.lon() ); #in feet
			
	# This loop is one of our biggest framerate sucks and so if we're an undamaged
	# aircraft way above our minimum AGL we're just going to skip it entirely.
	if (type == "aircraft" and damageValue < 0.95 and (currAlt_ft - toFrontAlt_ft) > 3 * alts.minimumAGL_ft) return;
			
			

	
	
	# if it's damaged we always get the pitch angle etc as that is how we force it down.
	# but if it's on the ground, we don't care and all these geo.Coords & elevs really kill FR.
	# if (thorough or damageValue > 0.8 ) {	

	if (type == "groundvehicle" or ctrls.onGround) 
	{	#only get roll for ground vehicle or AC that has just crashed

	# find the slope of the ground in the direction we are heading

		GeoCoord.apply_course_distance(heading + 180, 2 * frontBack_m );
		var toRearAlt_ft = elev (GeoCoord.lat(), GeoCoord.lon()  );
		pitchangle1_deg = math.atan2( toFrontAlt_ft - toRearAlt_ft, 2 * frontBack_ft ) * R2D;
		pitchangle_deg = pitchangle1_deg; 
		# rjw: the slope of ground.  The buffer is to ensure that we don't measure altitude at the top of the object		

		var distAhead_m = speed_kt * KT2MPS * 5;
		GeoCoord.apply_course_distance( heading, distAhead_m + 2 * frontBack_m);
		var gradientAhead = (elev ( GeoCoord.lat(), GeoCoord.lon() ) - toFrontAlt_ft) * FT2M / distAhead_m;

		# find altitude of ground to left & right of object to determine roll &
		# to help in determining altitude
		# go that extra amount out from the actual width to keep FG from detecting the top of the
		# object as the altitude.  We need ground altitude here. FGAltObjectPerimeterBuffer_m

		var GeoCoord2 = geo.Coord.new();
		GeoCoord2.set_latlon(lat, lon);
		GeoCoord2.apply_course_distance( heading - 90, leftRight_m );  #sidedist in meters
		var toLeftAlt_ft = elev (GeoCoord2.lat(), GeoCoord2.lon()  ); #in feet
		GeoCoord2.apply_course_distance( heading + 90, 2 * leftRight_m );  #sidedist in meters
		var toRightAlt_ft = elev (GeoCoord2.lat(), GeoCoord2.lon()  ); #in feet
		var rollangle_rad = math.atan2( toLeftAlt_ft - toRightAlt_ft, 2 * leftRight_ft ); 
		rollangle_deg = R2D * rollangle_rad; 

		# in CVS, taking the alt of an object's position actually finds the top
		# of that particular object.  So to find the alt of the actual landscape
		# we do ahead, behind, to left, to right of object & take the average.
		# luckily this also helps us calculate the pitch of the slope,
		# which we need to set pitch & roll,  so little is lost

		alt_ft = (toFrontAlt_ft + toRearAlt_ft + toLeftAlt_ft + toRightAlt_ft) / 4; #in feet
		
	}
	else
	{
		toLeftAlt_ft = alt_ft;
		toRightAlt_ft = alt_ft;
	}

	
	#The first time this is called just initializes all the altitudes and exit
	#rjw are these altitudes ever used again?
	if ( alts.initialized != 1 ) 
	{
		var initial_altitude_ft = getprop (""~myNodeName~"/position/altitude-ft");
		if (initial_altitude_ft < alt_ft + alts.wheelsOnGroundAGL_ft + alts.minimumAGL_ft) {
			initial_altitude_ft = alt_ft + alts.wheelsOnGroundAGL_ft + alts.minimumAGL_ft;
		}
		if (initial_altitude_ft > alt_ft + alts.wheelsOnGroundAGL_ft + alts.maximumAGL_ft) {
			initial_altitude_ft = alt_ft + alts.wheelsOnGroundAGL_ft + alts.maximumAGL_ft;
		}
				
		var target_alt_AGL_ft = initial_altitude_ft - alt_ft - alts.wheelsOnGroundAGL_ft; 
				
		debprint (sprintf("Bombable: Initial Altitude:%6.0f Target AGL:%6.0f Object = %s", initial_altitude_ft, target_alt_AGL_ft, myNodeName));
		# debprint ("Bombable: ", alt_ft, " ", toRightAlt_ft, " ",toLeftAlt_ft, " ",toFrontAlt_ft," ", toLeftAlt_ft, " ", alts.wheelsOnGroundAGL_ft);
		
		if (type != "aircraft") 
		{
			setprop (""~myNodeName~"/position/altitude-ft", alt_ft ); # ships and groundvehicles are set to altitude of ground in their initial location
			# setprop (""~myNodeName~"/controls/flight/target-alt",  alt_ft); # sets the target height of groundvehicles which are AI model type aircraft - confusing!
			alts.targetAGL_ft = 0; 
			alts.initialAlt_ft = alt_ft;  # rjw mod to check for grounded ships
		}
		else
		{
			setprop (""~myNodeName~"/position/altitude-ft", initial_altitude_ft );
			setprop (""~myNodeName~"/controls/flight/target-alt",  initial_altitude_ft);
			alts.targetAGL_ft = target_alt_AGL_ft;  # allows aircraft to fly at constant height AGL
			alts.initialAlt_ft = initial_altitude_ft; 
		}
		vels.speedOnFlat = speed_kt; # rjw used for groundVehicles which slow down and speed up according to gradient; used in add_damage
		alts.initialized = 1;
				
		return;
	}

	var objectsLowestAllowedAlt_ft = alt_ft + alts.wheelsOnGroundAGL_ft + alts.crashedAGL_ft;
	# if (thorough) debprint (" objectsLowestAllowedAlt_ft = ", objectsLowestAllowedAlt_ft);
			
	# If the object is as low as allowed by crashedAGL_m
	# we consider it "on the ground" (for an airplane) or
	# completely sunk (for a ship) etc.
	# If it is going there at any speed we consider it crashed
	# into the ground. When this
	# property is set to true then the speed will slow quite dramatically.
	# This allows for example airplanes to continue forward movement
	# in the air but skid to a sudden halt when hitting the ground.
	#
	# alts.wheelsOnGroundAGL_ft + damageAltAdd = the altitude (AGL) the object should be at when
	# finished crashing, sinking, etc.
	# It's not that easy to determine if an object crashes--if an airplane
	# hits the ground it crashes but tanks etc are always on the ground
	

	# end of life:  damaged ships and ground vehicles grind to a halt; aircraft explode and flag onGround
	# speed is adjusted by add_damage
	if ((type == "groundvehicle") or (type == "ship")) 
	{
		if (speed_kt <= 1) 
		{
			debprint(sprintf
				(
				"Bombable: Ground loop terminated for %s speed_kt=%6.2f tgt_speed=%6.2f",
				myNodeName,
				speed_kt,
				getprop(""~myNodeName~"/controls/tgt-speed-kts")
				)
			);
			ats.exploded = 1;
			ats.damage = 1; # could continue fighting even though immobilised
			setprop(""~myNodeName~"/controls/tgt-speed-kts", 0);
			setprop(""~myNodeName~"/velocities/true-airspeed-kt", 0);
			setprop(""~myNodeName~"/velocities/vertical-speed-fps", 0);
			if (type == "groundvehicle") deleteSmoke("pistonexhaust", myNodeName); # could set a timer here; smoke from ship?
			return;
		}
	}

	# use the lowest allowed altitude and current altitude to check for aircraft crash
	# onGround check to avoid multiple calls to hitground_stop_explode
	if (
		type == "aircraft" and !ctrls.onGround and 
		(
			(damageValue > 0.8 and ( currAlt_ft <= objectsLowestAllowedAlt_ft and speed_kt > 20 ) or ( currAlt_ft <= objectsLowestAllowedAlt_ft - 5))
			or 
			(damageValue == 1 and currAlt_ft <= objectsLowestAllowedAlt_ft)
		)
	)
	{
		debprint ("Bombable: Aircraft below lowest allowed altitude");
		hitground_stop_explode(myNodeName, alt_ft); 
		return;
	}
	
	# rjw bring crashed aircraft to a stop
	if (ctrls.onGround)
	{
		#go to object's resting altitude
		#rjw onGround is set by hitground_stop_explode
		# debprint("Bombable: ", myNodeName, " on ground. Exploded = ", ats.exploded);
		
		setprop (""~myNodeName~"/position/altitude-ft", objectsLowestAllowedAlt_ft );
		setprop (""~myNodeName~"/controls/flight/target-alt",  objectsLowestAllowedAlt_ft);
		setprop (""~myNodeName~"/controls/flight/target-roll",  rollangle_deg);
		setprop (""~myNodeName~"/controls/flight/target-pitch",  pitchangle1_deg);
				
		#bring all to a complete stop
		setprop(""~myNodeName~"/controls/tgt-speed-kt", 0);
		setprop(""~myNodeName~"/controls/flight/target-spd", 0);
		setprop(""~myNodeName~"/velocities/true-airspeed-kt", 0);
		setprop(""~myNodeName~"/velocities/vertical-speed-fps", 0);
				
		#we don't even really need the timer any more, since this object
		#is now exploded and stopped.		
		#the ground_loop is terminated if exploded == 1
		return;		
	}

	# rjw mod: the descent of a destroyed (damage == 1) aircraft is managed by aircraftCrashControl 
	if (type == "aircraft" and damageValue == 1) return;
	# the flight of a partially damaged aircraft is managed by the following code which includes ground avoidance
			

	#poor man's look-ahead radar
	var lookingAheadAlt_ft = toFrontAlt_ft;
	if (type == "aircraft" and !ctrls.onGround ) 
	{
		GeoCoord.apply_course_distance( heading, speed_kt * KT2MPS * 10 );
		# 10sec look ahead - 120s below?
				
		var radarAheadAlt_ft = elev ( GeoCoord.lat(), GeoCoord.lon() ); #in feet
				
				
		# our target altitude (for aircraft purposes) is the greater of the
		# altitude immediately in front and the altitude from our
		# poor man's lookahead radar. (i.e. up to 2 min out at current
		# speed).  If the terrain is rising we add 300 to our target
		# alt just to be on the safe side.
		# But if we're crashing, we don't care about what is ahead.

		# Use the radar lookahead altitude if
		#  1. higher than elevation of current location and
		#  2. not damaged and
		#  3. we'll end up below our minimumAGL if we continue at current altitude

		if (damageValue < 0.8 )
		{
			if ( (radarAheadAlt_ft > toFrontAlt_ft)  
			and (radarAheadAlt_ft + alts.minimumAGL_ft > currAlt_ft )  )
			lookingAheadAlt_ft = radarAheadAlt_ft;
					
			#if we're low to the ground we add this extra 500 ft just to be safe
			if (currAlt_ft - radarAheadAlt_ft < 500)
			lookingAheadAlt_ft  +=  500;
		}
	} 


	

			
	# set speed, pitch and roll of ground vehicle according to terrain
	# rjw might use thorough if the number of calls to measure terrain altitude use too many clock cycles
	if (type == "groundvehicle") 
	{
		var gradient = (toFrontAlt_ft - alt_ft ) / frontBack_ft;
		# here can change speed according to gradient ahead
		# true-airspeed-kt for a ship is the horizontal speed
		# horizontal speed maintained up to the gradient at which the max climb rate is exceeded 
		var slope_rad = math.atan(gradient);

		# set vert-speed not pitch for ground craft
		var vert_speed = gradient * speed_kt * KT2FPS;
		vert_speed += (alts.wheelsOnGroundAGL_ft / math.cos(slope_rad) / math.cos(rollangle_rad) + alt_ft - currAlt_ft) / updateTime_s; # correction if above or below ground
		var speedFactor = vert_speed / vels.maxClimbRate_fps;  # this parm is only set for a groundvehicle
		if (speedFactor > 1) 
		{
			vert_speed = vels.maxClimbRate_fps;
			setprop (""~myNodeName~"/velocities/true-airspeed-kt", speed_kt / speedFactor); # rather than set target-speed try direct change which the AI will then adjust out
		}
		elsif (speedFactor < -2) 
		{
			vert_speed = -2 * vels.maxClimbRate_fps; # could also compare with cruise speed here
			setprop (""~myNodeName~"/velocities/true-airspeed-kt", -speed_kt * 2 / speedFactor);
		}


		var delta_t = updateTime_s / N_STEPS;
		var delta_alt = vert_speed * delta_t;
		altitude_adjust(myNodeName, currAlt_ft, 0, delta_alt, delta_t, N_STEPS);

		if (!ctrls.dodgeInProgress)
		{
			# avoid steep terrain
			var targetHeading = getprop (""~myNodeName~"/controls/tgt-heading-degs");

			if (math.abs(gradientAhead) > 0.9) # turn if at top or bottom of cliff
			{
				if (!ctrls.avoidCliffInProgress)
				{
					# var newTargetHeading = (rand() > 0.5 ? 90 : -90); # could choose minimum grad
					var newTargetHeading = (math.abs(toLeftAlt_ft - alt_ft) > math.abs(toRightAlt_ft - alt_ft) ) ? 90 : -90; # turn toward level ground
					newTargetHeading = math.fmod ( newTargetHeading + 3600, 360);
					if (newTargetHeading > 180) newTargetHeading -= 360;
					setprop (""~myNodeName~"/controls/tgt-heading-degs", newTargetHeading);
					settimer
					(
						func
						{
						setprop (""~myNodeName~"/controls/tgt-heading-degs", targetHeading);
						ctrls.avoidCliffInProgress = 0;
						},
						2 + rand() * 5
					);
					ctrls.avoidCliffInProgress = 1;
					targetHeading = newTargetHeading;
					debprint
					(
						sprintf(
							"Bombable: avoiding cliff, new target hdg = %5.1f, slope = %5.1f", 
							newTargetHeading, slope_rad * R2D
						)
					);
				}
			}
			
			# steer toward target heading
			var delta_heading_deg = math.fmod ( targetHeading - heading + 3600, 360);
			if (delta_heading_deg > 180) delta_heading_deg -= 360;
			var sign = 1;
			var rudder = 0;
			if (delta_heading_deg < 0)
			{
				delta_heading_deg = - delta_heading_deg;
				sign = -1;
			}
			if (delta_heading_deg > 81)
				rudder = 30;
			elsif (delta_heading_deg > 27)
				rudder = 15;
			elsif (delta_heading_deg > 9)
				rudder = 10;
			elsif (delta_heading_deg > 3)
				rudder = 7;
			elsif (delta_heading_deg > 1)
				rudder = 4;
			setprop (""~myNodeName~"/surface-positions/rudder-pos-deg", rudder * sign);
		}
		

		# pitch and roll controlled by model animation
		setprop (""~myNodeName~"/orientation/roll-animation", rollangle_deg ); 
		setprop (""~myNodeName~"/orientation/pitch-animation", pitchangle_deg ); 
		
		# if (thorough) debprint(
		# "Bombable: Ground_loop: ",
		# sprintf("vertSpeed-fps = %4.1f", vert_speed),
		# sprintf("pitchangle_deg = %4.1f", pitchangle_deg),
		# sprintf("slopeAhead_deg = %4.1f", slope_rad * R2D),	
		# sprintf("alt_ft - currAlt_ft = %4.1f", alt_ft - currAlt_ft)
		# );

		# if (thorough and alts.initialized == 1) debprint(
		# "Bombable: Ground_loop: ",
		# "vels.speedOnFlat = ", vels.speedOnFlat
		# );

		return;
	}	

	# our target altitude for normal/undamaged forward movement
	# this isn't based on our current altitude but the results of our
	# "lookahead radar" to provide the base altitude
	# However as the craft is more damaged it loses its ability to do this
	# (see above: lookingAheadAlt just becomes the same as toFrontAlt)

	var targetAlt_ft = lookingAheadAlt_ft + alts.targetAGL_ft + alts.wheelsOnGroundAGL_ft;  # allows aircraft to fly at constant height AGL

	#debprint ("laa ", lookingAheadAlt_ft, " tagl ", alts.targetAGL_ft, " awog ", alts.wheelsOnGroundAGL_ft);
			
			
	var fullDamageAltAdd_ft = (alt_ft + alts.crashedAGL_ft + alts.wheelsOnGroundAGL_ft) - currAlt_ft; 
	# Amount we should add to our current altitude when fully crashed.  
	# This is to get the object to "full crashed position", i.e. on the ground for an aircraft, fully sunk for a ship, etc.
			
			
	# now calculate how far to force the thing down if it is crashing/damaged
	# rjw ships and aircraft will sink/fall when damaged; some ground vehicles are classed as ships!

	var damageAltAddCurrent = 0; #local value of variable in attributes hash
	var damageAltMaxPerCycle_ft = 0;
	if ( damageValue > 0.8)  
	{
		var damageAltAddMax_ft = damageValue * fullDamageAltAdd_ft; 
		#max amount to add to the altitude of this object based on its current damage.
		#Like fullDamageAltAdd & damageAltAddPrev this should always be zero
		#or negative as everything on earth falls or sinks when it loses
		#power. And assuming that simplifies calculations immensely.
				
		#The altitude the object should be at, based on damagealtAddMax & the ground level:
		#currAlt_ft + damageAltAddMax_ft

		
		#limit amount of sinkage to damageAltMaxRate in one hit/loop--otherwise it just goes down too fast, not realistic.  
		#Analogous to the terminal velocity
		damageAltMaxPerCycle_ft = -abs(vels.damagedAltitudeChangeMaxRate_meterspersecond * updateTime_s * M2FT);
		#rjw might change this amount if crashing at the terminal velocity; probably no need for abs unless error in input data		
				
				
		#rjw: descent rate increases at 10% per second; initialised at 1% the max rate
		#rjw: 48sec to reach max descent rate
		#making sure to move in the right direction! (using sgn of damageAltAdd)
		if (damageAltAddPrev_ft != 0) damageAltAddCurrent = -abs((1 + 0.1 * updateTime_s) * damageAltAddPrev_ft);
		else damageAltAddCurrent = - abs(0.01 * damageAltMaxPerCycle_ft);
				
		#Ensure this is not bigger than the max rate, if so only change
		#it by the max amount allowed per cycle
		if (abs( damageAltAddCurrent ) > abs(damageAltMaxPerCycle_ft )) damageAltAddCurrent = damageAltMaxPerCycle_ft;
				
		#Make sure we're not above the max allowed altitude change for this damage level; if so, cut it off
		if (abs(damageAltAddCurrent) > abs(damageAltAddMax_ft)) {
			damageAltAddCurrent = damageAltAddMax_ft;
		}
	} 



			
	# if we are dropping faster than the current slope (typically because
	# we are an aircraft diving to the ground because of damage) we
	# make the pitch match that angle, even if it more acute than the
	# regular slope of the underlying ground
	
	# rjw we do not sink ships simply crash them a few metres below ground level

	
	if (type == "aircraft" and damageValue > 0.8 and 0) 
	# rjw removed since not realistic:  Badly damaged diving aircraft do not match their pitch to the gradient of ground.  
	# Causes errors since pitchangle1_deg only calculated when reach ground
	{ 
		var pitchangle2_deg = R2D * math.asin(damageAltAddCurrent, distance_til_update_ft );
		if (damageAltAddCurrent == 0 and distance_til_update_ft > 0) pitchangle2_deg = 0; #forward
		if (damageAltAddCurrent < 0 and distance_til_update_ft == 0) pitchangle2_deg = -90; #straight down
		#Straight up won't happen here because we are (on purpose) forcing
		#the object down as we crash.  So we ignore the case.
		#if (distance_til_update_ft == 0 and damageAltAddCurrent > 0 ) pitchangle2 = 90; straight up
				
		#if no movement at all then we leave the pitch alone
		#if movement is less than 0.5 feet for pitch purposes we consider it
		#no movement at all--just a bit of wiggling

		if ( (abs(damageAltAddCurrent) < 0.5) and ( abs(distance_til_update_ft) < 0.5)) pitchangle2_deg = 0;
				
		if (abs(pitchangle2_deg) > abs(pitchangle1_deg)) pitchangle_deg = pitchangle2_deg;
		#pitchangle1_deg is the slope of the land at the location of the object
		#pitchangle2_deg is the slope of the glide path of the object
	}

	if (type == "ship" and thorough )
	{
		# if (math.fmod(ctrls.groundLoopCounter , 10) == 0) debprint(
		# "Bombable: Ground_loop: ",
		# "vels.maxSpeedReduce_percent = ", vels.maxSpeedReduce_percent,
		# "alts.initialAlt_ft = ", alts.initialAlt_ft
		# );		

		# pitch and roll controlled by model animation
		rollangle_deg = getprop (""~myNodeName~"/orientation/roll-animation");
		pitchangle_deg = getprop (""~myNodeName~"/orientation/pitch-animation");
		if (rollangle_deg == nil) rollangle_deg = 0;
		if (pitchangle_deg == nil) pitchangle_deg = 0;
		if (ctrls["target_roll"] != nil) # initialised by add_damage
		{
			rollangle_deg = 0.95 * rollangle_deg + 0.05 * ctrls.target_roll;
			pitchangle_deg = 0.95 * pitchangle_deg + 0.05 * ctrls.target_pitch;
		}
		setprop (""~myNodeName~"/orientation/roll-animation", rollangle_deg );
		setprop (""~myNodeName~"/orientation/pitch-animation", pitchangle_deg );

		# rjw check if grounded
		if ((targetAlt_ft - alts.initialAlt_ft) > 3) 
		{
			vels.maxSpeedReduce_percent = 20;
			add_damage(1, "nonweapon", myNodeName);
			return(); # no need to sink now			
		}
	}

	var currTgtAlt_ft = getprop (""~myNodeName~"/controls/flight/target-alt");#in ft
	if (currTgtAlt_ft == nil) currTgtAlt_ft = 0;
			
	if ( (damageValue <= 0.8 ) or targetAlt_ft < currTgtAlt_ft ) 
	{
		setprop (""~myNodeName~"/controls/flight/target-alt", targetAlt_ft);   #target altitude--this is 10 feet or so in front of us for a ship or up to 1 minute in front for an aircraft
		# rjw a damaged land-, air- or sea-craft loses the ability to climb
	}
			
	#if going uphill base the altitude on the front of the vehicle (targetAlt).
	#This keeps the vehicle from sinking into the
	#hillside when climbing.  This is a bit of a kludge that is simple/fast
	#because we have already calculated targetAlt in calculating the pitch.
	#To make this precise, calculate the correct position forward
	#based on the current speed of the current object and updateTime_s
	#and find the altitude of that spot.
	#For aircraft the targetAlt is the altitude 1 minute out IF that is higher
	#than the ground level.
	#rjw_mod based on above
	var calcAlt_ft = targetAlt_ft +  ctrls.damageAltAddCumulative_ft + damageAltAddCurrent;
	if (calcAlt_ft < objectsLowestAllowedAlt_ft) calcAlt_ft = objectsLowestAllowedAlt_ft;
			
			
	#calcAlt_ft = where the object should be, in feet
	#if it is an aircraft we try to control strictly via setting the target
	# altitude etc. (above).  If a ship etc. we just have to force it to that altitude (below).  However if an aircraft gets too close to the ground
	#the AI aircraft controls just won't react quickly enough so we "rescue"
	#it by simply moving it up a bit (see below).
	#debprint ("type = ", type);
	#rjw:  changing target altitude does not bring the aircraft down to the ground quickly. The AI system does not respond in the way intended
	#rjw:  with pitch-target at -70 continue gliding with pitch at -4
	#rjw:  potentially might improve the response by reducing the airspeed

	
	
	# for an aircraft, if it is within feet of the ground (and not forced
	# there because of damage etc.) then we "rescue" it be putting it 25 feet
	# above ground again.
	

	if ((type == "aircraft") and ( currAlt_ft < toFrontAlt_ft + 75) and (damageValue <= 0.8 ))   
	{
		#debprint ("correcting!", myNodeName, " ", toFrontAlt_ft, " ", currAlt_ft, " ", currAlt_ft-toFrontAlt_ft, " ", toFrontAlt_ft+40, " ", currAlt_ft+20 );
		#set the pitch to try to make it look like we're climbing real
		#fast here, not just making an emergency correction . . .
		#for some reason the pitch is always aiming down when we
		#need to make a correction up, using pitchangle1.
		#Kludge, we just always put pitch @30 degrees
				
		#vert-speed prob
		setprop (""~myNodeName~"/orientation/pitch-deg", 30 );
		setprop (""~myNodeName~"/controls/flight/target-pitch", 30);
				
		if (currAlt_ft < toFrontAlt_ft + 25 ) 
			{ #dramatic correction
			debprint ("Bombable: Avoiding ground collision, "~ myNodeName);
					
			setprop (""~myNodeName~"/position/altitude-ft", toFrontAlt_ft + 40 );
			setprop (""~myNodeName~"/controls/flight/target-alt",  toFrontAlt_ft + 40);
					
			# vert-speed prob
			# 250 fps is achieved by a Zero in a normal barrel roll, so 300 fps is
			# a pretty extreme/edge of reality maneuver for most aircraft
			#
			# We are trying to set the vert spd to 300 fps but do it in
			# increments of 70 fps at most to try to maintain realism

			setVerticalSpeed (myNodeName, 300, 75, 4, .1, 80, 35);
					
			} 
			else 
			{   #more minor correction
					
			setprop (""~myNodeName~"/controls/flight/target-alt",  currAlt_ft + 20);
					
			#vert-speed prob
			# 250 fps is achieved by a Zero in a normal barrel roll, so 70 fps is
			# a very hard pull back on the stick in most aircraft, but not impossible

			setVerticalSpeed (myNodeName, 100, 45, 4, .2, 70, 35);
					
			}
	}
			

	if ( damageValue > 0.8 ) 
	{
		if ( type == "aircraft") 
		{
			#If crashing we just force it to the right altitude, even if an aircraft
			#but we move it a maximum of damageAltMaxRate
			#if it's an airplane & it's crashing, we take it down as far as
			#needed OR by the maximum allowed rate.
					
			#when it hits this altitude it is (or most very soon become)
			#completely kaput
			#For many objects, depending on how the model is set up, this
			#may be somewhat higher or lower than actual ground level
					
					
			if ( damageAltMaxPerCycle_ft < damageAltAddCurrent )  
			{
				setprop (""~myNodeName~"/controls/flight/target-alt",  currAlt_ft - 500);
				setprop (""~myNodeName~"/controls/flight/target-pitch", -45);
						
				#vert-speed prob
				var orientPitch_deg = getprop (""~myNodeName~"/orientation/pitch-deg");
				if ( orientPitch_deg > -10)
				{
					setprop (""~myNodeName~"/orientation/pitch-deg", orientPitch_deg - 1 );
					debprint ("Bombable: Changed pitch mild");
				}
						
			}
			elsif (currAlt_ft + damageAltMaxPerCycle_ft > objectsLowestAllowedAlt_ft ) 
			{
				#put it down by the max allowed rate
				setprop (""~myNodeName~"/controls/flight/target-alt",  currAlt_ft - 10000);
				setprop (""~myNodeName~"/controls/flight/target-pitch", -70);
									
				#vert-speed prob
				var orientPitch_deg = getprop (""~myNodeName~"/orientation/pitch-deg");
				if (orientPitch_deg > -20) 
				{
					setprop (""~myNodeName~"/orientation/pitch-deg", orientPitch_deg - 1 );
					debprint ("Bombable: Changed pitch severe");
				}
			} 
			else
			{ 
				#closer to the ground than MaxPerCycle so terminate and explode
				debprint ("Bombable: Aircraft hit ground");
				hitground_stop_explode(myNodeName, objectsLowestAllowedAlt_ft);
			}

			
			#somehow the aircraft are getting below ground sometimes
			#sometimes it's just because they hit into a mountain or something
			#else in the way.
			#kludgy fix, just check for it & put them back on the surface
			#if necessary.  And explode & stuff.
						
			if ( currAlt_ft < alt_ft - 5 )  
			{
				debprint ("Bombable: Aircraft below ground! Terminated.");
				hitground_stop_explode(myNodeName, objectsLowestAllowedAlt_ft );
			}
					
		}
		elsif (type == "ship") 
		{
			setprop(""~myNodeName~"/position/altitude-ft", calcAlt_ft );
			# if ( damageAltAddCurrent > damageAltMaxPerCycle_ft )  
			# {
			# 	setprop(""~myNodeName~"/position/altitude-ft", currAlt_ft + damageAltMaxPerCycle_ft );
			# }
			# else
			# {
			# 	setprop(""~myNodeName~"/position/altitude-ft", currAlt_ft + damageAltAddCurrent);
			# }
		}
	}		
	#Whatever else, we don't let aircraft go below their lowest allowed altitude
	#Maybe they are skidding along on the ground, but they are not allowed
	#to skid along UNDER the ground . . .

	if (currAlt_ft < objectsLowestAllowedAlt_ft)
	{
		setprop(""~myNodeName~"/position/altitude-ft", objectsLowestAllowedAlt_ft); #where the object is, in feet
	}
	ctrls.damageAltAddCurrent_ft = damageAltAddCurrent; # store local value in global hash
	ctrls.damageAltAddCumulative_ft += damageAltAddCurrent;
			
	#debprint ("alt = ", alt, " currAlt_ft = ", currAlt_ft, " deltaAlt = ", deltaAlt, " altAdjust = ", alts.wheelsOnGroundAGL_ft, " calcAlt_ft = ", calcAlt_ft, "damageAltAddCurrent = ", damageAltAddCurrent, " ", myNodeName);
}



#######################################################
# location-check loop, a timer function, every 15-16 seconds to check if the object has been relocated
# (this will happen if the object is set up as an AI ship or aircraft and FG is reset).  
# If so it restores the object to its position before the reset.
# This solves an annoying problem in FG, where using file/reset (which
# you might do if you crash the aircraft, but also if you run out of ammo
# and need to re-load or for other reasons) will also reset the objects to
# their original positions.
# With moving objects (set up as AI ships or aircraft with velocities,
# rudders, and/or flight plans) the objects are often just getting to
# interesting/difficult positions, so we want to preserve those positions
# rather than letting them reset back to where they started.
# TODO: Some of this could be done better using a listener on /sim/signals/reinit
#
# rjw: the logic to detect a reset current delta_dist > 4x previous delta_dist is flawed
# some objects such as tanks will go round in circles or make slow progress over steep terrain
# another approach is to track the average location or average displacement since last called but 
# still will not work for objects that dwell and then move therefore omit this loop

var location_loop = func(id, myNodeName) {
	id == attributes[myNodeName].loopids.location_loopid or return;

	#debprint ("location_loop starting");
	# reset the timer so we will check this again in 15 seconds +/-
	# add rand() so that all objects don't do this function simultaneously
	# when 15-20 objects are all doing this simultaneously it can lead to jerkiness in FG
	settimer(func {location_loop(id, myNodeName); }, 15 + rand() );
			
	#get out of here if Bombable is disabled
	if (! bombableMenu["bombable-enabled"] ) return;
			
	var node = props.globals.getNode(myNodeName);
			
			
	var started = getprop (""~myNodeName~"/position/previous/initialized");
			
	var lat = getprop(""~myNodeName~"/position/latitude-deg");
	var lon = getprop(""~myNodeName~"/position/longitude-deg");
	var alt_ft = getprop(""~myNodeName~"/position/altitude-ft");
			
	if (lat == nil) {
		lat = 0;
		debprint ("Bombable: Lat = NIL, location_loop", myNodeName);
	}
	if (lon == nil) {
		lon = 0;
		debprint ("Bombable: Lon = NIL, location_loop", myNodeName);
	}


			
	# getting the global_x,y,z seems to stop strange behavior from the smoke
	# when we do a relocate of the objects
	# rjw probably delete
	var global_x = getprop(""~myNodeName~"/position/global-x");
	var global_y = getprop(""~myNodeName~"/position/global-y");
	var global_z = getprop(""~myNodeName~"/position/global-z");
			
			
	var prev_distance = 0;
	var directDistance = 200; # this will be set as previous/distance if we are initializing
			
	# if we have previously recorded the position we check if it has moved too far
	# if it has moved too far it is because FG has reset and we
	# then restore the object's position to where it was before the reset
	if ( started ) 
	{
		var prevlat = getprop(""~myNodeName~"/position/previous/latitude-deg");
		var prevlon = getprop(""~myNodeName~"/position/previous/longitude-deg");
		var prevalt_ft = getprop(""~myNodeName~"/position/previous/altitude-ft");
		var prev_global_x = getprop(""~myNodeName~"/position/previous/global-x");
		var prev_global_y = getprop(""~myNodeName~"/position/previous/global-y");
		var prev_global_z = getprop(""~myNodeName~"/position/previous/global-z");
				
				
		var prev_distance = getprop(""~myNodeName~"/position/previous/distance");
				
		var GeoCoord = geo.Coord.new();
		GeoCoord.set_latlon(lat, lon, alt_ft * FT2M);

		var GeoCoordprev = geo.Coord.new();
		GeoCoordprev.set_latlon(prevlat, prevlon, prevalt_ft * FT2M);

		var directDistance = GeoCoord.distance_to(GeoCoordprev); # following earth's curvature, ignoring altitude
				
		debprint ("Object  ", myNodeName, ", distance: ", directDistance, ", prev distance: ", prev_distance);
				
		# 4X the previously traveled distance is our cutoff
		# so if our object is moving faster/further than this we assume it has
		# been reset by FG and put it back where it was before the reset.
		# Luckily, this same scheme works in the case this subroutine has moved the
		# object--then the previous distance exactly equals the distance traveled--
		# so even though that is a much larger than usual distance (which would
		# usually trigger this subroutine to think an init had happened) since
		# the object moved that large distance on the *  * previous step *  * (due to the
		# reset) the move back is less than 4X the previous move and so it is OK.

		# A bit kludgy . . . but it works.
		# rjw why 5?
		if ( directDistance > 5 and directDistance > 4 * prev_distance ) 
		{
			node.getNode("position/latitude-deg", 1).setDoubleValue(prevlat);
			node.getNode("position/longitude-deg", 1).setDoubleValue(prevlon);
			node.getNode("position/altitude-ft", 1).setDoubleValue(prevalt_ft);
			# now we want to show the previous location as this newly relocated position and distance traveled = 0;
			lat = prevlat;
			lon = prevlon;
			alt_ft = prevalt_ft;
					
			debprint ("Bombable: Repositioned object "~ myNodeName~ " to lat: "~ prevlat~ " long: "~ prevlon~ " altitude: "~ prevalt_ft~" ft.");
		}
	}
	# now we save the current position
	node.getNode("position/previous/initialized", 1).setBoolValue(1);
	node.getNode("position/previous/latitude-deg", 1).setDoubleValue(lat);
	node.getNode("position/previous/longitude-deg", 1).setDoubleValue(lon);
	node.getNode("position/previous/altitude-ft", 1).setDoubleValue(alt_ft);
	node.getNode("position/previous/global-x", 1).setDoubleValue(global_x);
	node.getNode("position/previous/global-y", 1).setDoubleValue(global_y);
	node.getNode("position/previous/global-z", 1).setDoubleValue(global_z);			
	node.getNode("position/previous/distance", 1).setDoubleValue(directDistance);

}
#################################################################
# This is the old way of calculating the closest impact distance
# This approach uses more of the geo.Coord functions from geo.nas
# The other approach is more vector based and uses a local XYZ
# coordinate system based on lat/lon/altitude.
# I'm not sure which is the most accurate but I believe this one is slower,
# with multiple geo.Coord calls plus some trig.
var altClosestApproachCalc = func {

	# figure how close the impact and terrain it's on
	var objectGeoCoord = geo.Coord.new();
	objectGeoCoord.set_latlon(oLat_deg,oLon_deg,oAlt_m );
	var impactGeoCoord = geo.Coord.new();
	impactGeoCoord.set_latlon(iLat_deg, iLon_deg, iAlt_m);
			
	#impact point as though at the same altitude as the object - for figuring impact distance on the XY plane
	var impactSameAltGeoCoord = geo.Coord.new();
	impactSameAltGeoCoord.set_latlon(iLat_deg, iLon_deg, oAlt_m);


	var impactDistanceXY_m = objectGeoCoord.direct_distance_to(impactSameAltGeoCoord);

	if (impactDistanceXY_m > 200 ) {
		#debprint ("Not close in surface distance. ", impactDistanceXY_m);
		#return;
	}
			
	var impactDistance_m = objectGeoCoord.direct_distance_to(impactGeoCoord);

	#debprint ("impactDistance ", impactDistance_m);
			
	var impactHeadingDelta_deg = math.abs ( impactGeoCoord.course_to(objectGeoCoord) -  impactorHeading_deg );
			
			
			

	#the pitch angle from the impactor to the main object
	var impact2ObjectPitch_deg = R2D * math.asin ( deltaAlt_m/impactDistance_m);
			
	var impactPitchDelta_deg = impactorPitch_deg - impact2ObjectPitch_deg;
			
	#Closest approach of the impactor to the center of the object along the direction of pitch
	var closestApproachPitch_m = impactDistance_m * math.sin (impactPitchDelta_deg * D2R);

	# This formula calcs the closest distance the object would have passed from the exact center of the target object, where 0 = a direct hit through the center of the object; on the XY plane
	var closestApproachXY_m = math.sin (impactHeadingDelta_deg* D2R) * impactDistanceXY_m * math.cos (impactPitchDelta_deg * D2R);;

			
	#combine closest approach in XY and closest approach along the pitch angle to get the
	# overall point of closest approach
	var closestApproachOLDWAY_m = math.sqrt (
	closestApproachXY_m * closestApproachXY_m +
	closestApproachPitch_m * closestApproachPitch_m);
			
	#debprint ("Bombable: Projected closest impact distance : ", closestApproachOLDWAY_m, "FG Impact Detection Point: ", impactDistance_m, " XY: ", closestApproachXY_m, " Pitch: ", closestApproachPitch_m, " impactDistance_m = ",impactDistance_m, " impactDistanceXY_m = ",impactDistanceXY_m, " ballisticMass_lb = ", ballisticMass_lb);
			
	if (impactDistance_m < closestApproach_m) debprint ("#########CLOSEST APPROACH CALC ERROR########");

}

########################################
# put_splash puts the impact splash from test_impact
#
var put_splash = func (nodeName, iLat_deg,iLon_deg, iAlt_m, ballisticMass_lb, impactTerrain = "terrain", refinedSplash = 0, myNodeName = "" ){
	#This check to avoid duplicate splashes is not quite working in some cases
	# perhaps because the lat is repeating exactly for different impacts, or
	# because some weapon impacts and collisions are reported a little differently?
	var impactSplashPlaced = getprop (""~nodeName~"/impact/bombable-impact-splash-placed");
	var impactObjectLat_deg = getprop (""~nodeName~"/impact/latitude-deg");
			
	if ((impactSplashPlaced == nil or impactSplashPlaced != impactObjectLat_deg) and iLat_deg != nil and iLon_deg != nil and iAlt_m != nil){
				
		records.record_impact ( myNodeName: myNodeName, damageRise:0, damageIncrease:0, damageValue:0, impactNodeName: nodeName, ballisticMass_lb: ballisticMass_lb, lat_deg: iLat_deg, lon_deg: iLon_deg, alt_m: iAlt_m );

		if (ballisticMass_lb < 1.2) {
			var startSize_m = 0.25 + ballisticMass_lb/3;
			var endSize_m = 1 + ballisticMass_lb;
			} else {
			var startSize_m = 0.25 + ballisticMass_lb/1000;
			var endSize_m = 2 + ballisticMass_lb/4;
		}
				
		impLength_sec = 0.75+ ballisticMass_lb/1.2;
		if (impLength_sec > 20) impLength_sec = 20;
				
		#The idea is that if the impact hits earth it throws up a bunch of
		#dirt & dust & stuff for a longer time.  But only for smaller/projectile
		#weapons where the dirt/dust is the main visual.
		# Based on observing actual weapons impacts on Youtube etc.
		#
		if (impactTerrain == "terrain" and ballisticMass_lb <= 1.2) {
			endSize_m  *=  5;
			impLength_sec  *=  5;
		}

		#debprint ("Bombable: Drawing impact, ", nodeName, " ", iLat_deg, " ", iLon_deg, " ",  iAlt_m, " refined:", refinedSplash );
		put_remove_model(iLat_deg,iLon_deg, iAlt_m, impLength_sec, startSize_m, endSize_m);
		#for larger explosives (or a slight chance with smaller rounds, which
		# all have some incindiary content) start a fire
		if (ballisticMass_lb > 1.2 or
		(ballisticMass_lb <= 1.2 and rand() < ballisticMass_lb/10) ) settimer ( func {start_terrain_fire( iLat_deg,iLon_deg,iAlt_m, ballisticMass_lb )}, impLength_sec/1.5);
		setprop (""~nodeName~"/impact/bombable-impact-splash-placed", impactObjectLat_deg);
	}
			
	if  (refinedSplash)
	setprop (""~nodeName~"/impact/bombable-impact-refined-splash-placed", impactObjectLat_deg);


}


########################################
# exit_test_impact(nodeName)
# draws the impact splash for the nodeName
#
var exit_test_impact = func(nodeName, myNodeName){


	#if impact on a ship etc we're assuming that one of the other test_impact
	# instances will pick it up & we don't need to worry about it.
	var impactTerrain = getprop(""~nodeName~"/impact/type");
	if (impactTerrain != "terrain") {
		#debprint ("Bombable: Not drawing impact; object impact");
		return;
	}
			
	var iLat_deg = getprop(""~nodeName~"/impact/latitude-deg");
	var iLon_deg = getprop(""~nodeName~"/impact/longitude-deg");
	var iAlt_m = getprop(""~nodeName~"/impact/elevation-m");
			
			
	var ballisticMass_lb = getBallisticMass_lb(nodeName);

	#debprint ("Bombable: Exiting test_impact with a splash, ", nodeName, " ", ballisticMass_lb, " ", impactTerrain," ", iLat_deg, " ", iLon_deg, " ", iAlt_m);
			
	put_splash (nodeName, iLat_deg, iLon_deg, iAlt_m, ballisticMass_lb, impactTerrain, 0, myNodeName );


}

var getBallisticMass_lb = func (impactNodeName) {

	#weight/mass of the ballistic object, in lbs
	# rjw 'ballistic mass' is not a recognised term - assume it is the mass of the bullet (i.e. excluding cartridge and explosive)
	#var ballisticMass_lb = impactNode.getNode("mass-slug").getValue() * 32.174049;
			
	var ballisticMass_lb = 0;
	var ballisticMass_slug = getprop (""~impactNodeName~"/mass-slug");

	#ok, FG 2.4.0 leaves out the /mass-slug property, so we have to improvise.
	# We basically need to list or guess the mass of each & every type of ordinance
	# that might exist or be used.  Not good.
	if (ballisticMass_slug != nil ) ballisticMass_lb = ballisticMass_slug * 32.174049
	else {
		ballisticMass_lb = .25;
		var impactType = getprop (""~impactNodeName~"/name");
		#debprint ("Bombable: ImpactNodeType = ", impactType);
		if (impactType == nil) impactType = "bullet";

				
				
		#we start with specific & end with generic, so the specific info will take
		# precedence (if we have it)
		if (find ("MK-81", impactType ) != -1 ) ballisticMass_lb = 250;
		elsif (find ("MK-82", impactType ) != -1 ) ballisticMass_lb = 500;
		elsif (find ("MK82", impactType ) != -1 ) ballisticMass_lb = 500;
		elsif (find ("MK-83", impactType ) != -1 ) ballisticMass_lb = 1000;
		elsif (find ("MK-84", impactType ) != -1 ) ballisticMass_lb = 2000;
		elsif (find ("25 lb", impactType ) != -1 ) ballisticMass_lb = 25;
		elsif (find ("5 lb", impactType ) != -1 ) ballisticMass_lb = 5;
		elsif (find ("100 lb", impactType ) != -1 ) ballisticMass_lb = 100;
		elsif (find ("150 lb", impactType ) != -1 ) ballisticMass_lb = 150;
		elsif (find ("250 lb", impactType ) != -1 ) ballisticMass_lb = 250;
		elsif (find ("500 lb", impactType ) != -1 ) ballisticMass_lb = 500;
		elsif (find ("1000 lb", impactType ) != -1 ) ballisticMass_lb = 1000;
		elsif (find ("2000 lb", impactType ) != -1 ) ballisticMass_lb = 2000;
		elsif (find ("M830", impactType ) != -1 ) ballisticMass_lb = 25; # https://en.wikipedia.org/wiki/M830
		elsif (find ("aim-9", impactType ) != -1 ) ballisticMass_lb = 20.8;
		elsif (find ("AIM", impactType ) != -1 ) ballisticMass_lb = 20.8;
		elsif (find ("WP-1", impactType ) != -1 ) ballisticMass_lb = 23.9;
		elsif (find ("GAU-8", impactType ) != -1 ) ballisticMass_lb = 0.9369635;
		elsif (find ("M-61", impactType ) != -1 ) ballisticMass_lb = 0.2249;
		elsif (find ("M61", impactType ) != -1 ) ballisticMass_lb = 0.2249;
		elsif (find ("LAU", impactType ) != -1 ) ballisticMass_lb = 86; #http://www.dtic.mil/dticasd/sbir/sbir041/srch/af276.pdf
		elsif (find ("smoke", impactType ) != -1 ) ballisticMass_lb = 0.0;
		elsif (find (".50 BMG", impactType ) != -1 ) ballisticMass_lb = 0.130072735;
		elsif (find (".50", impactType ) != -1 ) ballisticMass_lb = 0.1; #https://en.wikipedia.org/wiki/M2_Browning
		elsif (find ("303", impactType ) != -1 ) ballisticMass_lb = 0.08125; #http://en.wikipedia.org/wiki/Vickers_machine_gun
		elsif (find ("gun", impactType ) != -1 ) ballisticMass_lb = .025;
		elsif (find ("bullet", impactType) != -1 ) ballisticMass_lb = 0.0249122356;
		elsif (find ("tracer", impactType) != -1 ) ballisticMass_lb = 0.0249122356;
		elsif (find ("round", impactType) != -1 ) ballisticMass_lb = 0.9369635;
		elsif (find ("cannon", impactType ) != -1 ) ballisticMass_lb = 0.127; #https://en.wikipedia.org/wiki/Oerlikon_20_mm_cannon
		elsif (find ("bomb", impactType ) != -1 ) ballisticMass_lb = 250;
		elsif (find ("heavy-bomb", impactType ) != -1 ) ballisticMass_lb = 750;
		elsif (find ("rocket", impactType ) != -1 ) ballisticMass_lb = 50;
		elsif (find ("missile", impactType ) != -1 ) ballisticMass_lb = 185;
				
	}

	return ballisticMass_lb;
}

var getImpactVelocity_mps = func (impactNodeName = nil,ballisticMass_lb = .25) {

	var impactVelocity_mps = getprop (""~impactNodeName~"/impact/speed-mps");
			
	#if perchance impact velocity isn't available we'll estimate it from
	# projectile size
	# These are rough approximations/guesses based on http://en.wikipedia.org/wiki/Muzzle_velocity
	if (impactVelocity_mps == nil or impactVelocity_mps == 0) {
		if (ballisticMass_lb < 0.1) impactVelocity_mps = 1200;
		elsif (ballisticMass_lb < 0.5) impactVelocity_mps = 900;
		elsif (ballisticMass_lb < 2) impactVelocity_mps = 500;
		elsif (ballisticMass_lb < 50) impactVelocity_mps = 250;
		elsif (ballisticMass_lb < 500) impactVelocity_mps = 150;
		elsif (ballisticMass_lb < 2000) impactVelocity_mps = 125;
		else impactVelocity_mps = 100;
	}
	return impactVelocity_mps;
}

################### cartesianDistance #####################
# cartesianDistance (x,y,z, . . . )
# returns the cartesian distance of any number of elements
var cartesianDistance = func  (elem...){
	var dist = 0;
	foreach (e; elem ) dist += e * e;
	return math.sqrt(dist);
}

####################### test_impact #########################
# FUNCTION test_impact
#
# listener function on ballistic impacts
# checks if the impact has hit our object and if so, adds the damage
# damageMult can be set high (for easy to damage things) or low (for
# hard to damage things).  Default/normal value (M1 tank) should be 1.

# FG uses a very basic collision detection algorithm that assumes a standard
# height and length for each type of AI object.  These are actually 'radius'
# type measurements--ie for the 2nd object, if the ballistic obj strikes 50 ft
# above OR 50 ft below, and within a circle of radius 100 ft of the lat/lon,
# then we get a hit.  From the C code:
# // we specify tgt extent (ft) according to the AIObject type
#    double tgt_ht[]    = {0,  50, 100, 250, 0, 100, 0, 0,  50,  50, 20, 100,  50};
#    double tgt_length[] = {0, 100, 200, 750, 0,  50, 0, 0, 200, 100, 40, 200, 100};
# http://gitorious.org/fg/flightgear/blobs/next/src/AIModel/AIManager.cxx

# In order, those are:
# enum object_type { otNull = 0, otAircraft, otShip, otCarrier, otBallistic,
	#  otRocket, otStorm, otThermal, otStatic, otWingman, otGroundVehicle,
	#  otEscort, otMultiplayer,
#  MAX_OBJECTS };
# http://gitorious.org/fg/flightgear/blobs/next/src/AIModel/AIBase.hxx

# So Aircraft is assumed to be 50 feet high, 100 ft long; multiplayer the same, etc.
# That is where any ballistic objects are detected and stopped by FG.
#
# A main point of the function below is to improve on this impact detection
# by projecting the point of closest approach of the impactor, then assigning
# a damage value based on that.
#


		
var test_impact = func(changedNode, myNodeName) {

	# Allow this function to be disabled via bombable menu
	if ( ! bombableMenu["bombable-enabled"] ) return;

	var impactNodeName = changedNode.getValue();
	#var impactNode = props.globals.getNode(impactNodeName);
			
			
	debprint ("Bombable: test_impact, ", myNodeName," ", impactNodeName);

	var oLat_deg = getprop (""~myNodeName~"/position/latitude-deg");
	var iLat_deg = getprop (""~impactNodeName~"/impact/latitude-deg");

	debprint ("Bombable: test_impact oLat, iLat: ", oLat_deg, " ", iLat_deg );

	# bhugh, 3/28/2013, not sure why this error is happening sometimes in 2.10:
	# Nasal runtime error: No such member: maxLat
	#  at E:/FlightGear 2.10.0/FlightGear/data/Nasal/bombable.nas, line 3405
	#  called from: E:/FlightGear 2.10.0/FlightGear/data/Nasal/bombable.nas, line 8350
	#  called from: E:/FlightGear 2.10.0/FlightGear/data/Nasal/globals.nas, line 100
			
	#debug.dump (attributes[myNodeName].dimensions);
			
	var maxLat_deg = attributes[myNodeName].dimensions['maxLat'];
	var maxLon_deg = attributes[myNodeName].dimensions['maxLon'];

	#attributes[myNodeName].dimensions.maxLon;
			
			
	# quick-n-dirty way to tell if an impact is close to our object at all
	# without processor-intensive calculations
	# we do this first and then exit if not close, so as to minimise
	# processing time
	#
	#
	var deltaLat_deg = (oLat_deg - iLat_deg);
	if (abs(deltaLat_deg) > maxLat_deg * 1.5 ) {
		#debprint ("Not close in lat. ", deltaLat_deg);
		exit_test_impact(impactNodeName, myNodeName);
		return;
	}
			
	var oLon_deg = getprop (""~myNodeName~"/position/longitude-deg");
	var iLon_deg = getprop (""~impactNodeName~"/impact/longitude-deg");

	var deltaLon_deg = (oLon_deg - iLon_deg);
	if (abs(deltaLon_deg) > maxLon_deg * 1.5 )  {
		#debprint ("Not close in lon. ", deltaLon_deg);
		exit_test_impact(impactNodeName, myNodeName);
		return;
	}

	var oAlt_m = getprop (""~myNodeName~"/position/altitude-ft") * FT2M;
	var iAlt_m = getprop (""~impactNodeName~"/impact/elevation-m");
	var deltaAlt_m = (oAlt_m - iAlt_m);
			
	if (abs(deltaAlt_m) > 300 ) {
		#debprint ("Not close in Alt. ", deltaAlt);
		exit_test_impact(impactNodeName, myNodeName);
		return;
	}
			
	#debprint ("Impactor: ", impactNodeName, ", Object: ", myNodeName);
	if (impactNodeName == "" or impactNodeName == nil) {
		#debprint ("impactNode doesn't seem to exist, exiting");
		return;
	}

	# Since FG kindly intercepts collisions along a fairly large target cylinder surrounding the
	# object, we simply project the last known heading of the ballistic object along
	# its path to determine how close to the center of the object it would have struck,
	# if it continued along its present heading in a straight line.

	# we do this for both terrain & ship/aircraft hits, because if the aircraft or ship is on
	# or very close to the ground, FG often lets the ai submodel go 'right through' the main
	# object and the only impact detected is with the ground.  This gets worse as the framerate
	# gets slow, because FG can only check for impacts at each frame - so with a projectile
	# going 1000 MPS and framerate of 10, that is only once every hundred meters.
			
	# Formula here:
	# http://mathforum.org/library/drmath/view/54731.html (a more vector-based
	# approach).
	#
	# ft_per_deg_lat = 366468.96 - 3717.12 * math.cos(pos.getLatitudeRad());
	# ft_per_deg_lon = 365228.16 * math.cos(pos.getLatitudeRad());
	# per FG c code, http://gitorious.org/fg/flightgear/blobs/next/src/AIModel/AIBase.cxx line 178
	# We could speed this up by leaving out the math.cos term in deg_lat and/or calculating these
	# occasionally as the main A/C flies around and storing them (they don't change that)
	# much from one mile to the next)
	# var iLat_rad = iLat_deg* D2R;
	# m_per_deg_lat = 111699.7 - 1132.978 * math.cos (iLat_rad);
	# m_per_deg_lon = 111321.5 * math.cos (iLat_rad);
			
	# m_per_deg_lat = getprop ("/bombable/sharedconstants/m_per_deg_lat");
	# m_per_deg_lon = getprop ("/bombable/sharedconstants/m_per_deg_lon");
			
	# the following plus deltaAlt_m make a < vector > where impactor is at < 0,0,0 > 
	# and target object is at < deltaX,deltaY,deltaAlt > in relation to it.
	var deltaY_m = deltaLat_deg * m_per_deg_lat;
	var deltaX_m = deltaLon_deg * m_per_deg_lon;
			
	# calculate point & distance of closest approach.
	# if the main aircraft (myNodeName == "") then we just
	# use FG's impact detection point.  If an AI or MP
	# aircraft, we project it into actual point of closest approach.
	if (myNodeName == "") {

		closestApproach_m = cartesianDistance(deltaX_m,deltaY_m,deltaAlt_m );
				
		} else {
				
		# debprint ("MPDL:", m_per_deg_lat, " MPDLon: ", m_per_deg_lon, " dL:", deltaLat_deg, " dLon:", deltaLon_deg);
				
		impactorHeading_deg = getprop (""~impactNodeName~"/impact/heading-deg");
		# if perchance this doesn't exist we'll just randomize it; it must be -90 to 90 or it wouldn't have hit.
		if (impactorHeading_deg == nil ) impactorHeading_deg = rand() * 180 - 90;
				
		impactorPitch_deg = getprop (""~impactNodeName~"/impact/pitch-deg");
		# if perchance this doesn't exist we'll just randomize it; it must be -90 to 90 or it wouldn't have hit.
		if (impactorPitch_deg == nil ) impactorPitch_deg = rand() * 180 - 90;
				
				
		# the following make a unit vector in the direction the impactor is moving
		# this could all be saved in the prop tree so as to avoid re-calcing in
		# case of repeated AI objects checking the same impactor
		var impactorPitch_rad = impactorPitch_deg* D2R;
		var impactorHeading_rad = impactorHeading_deg* D2R;
		var impactordirectionZcos = math.cos(impactorPitch_rad);
		var impactorDirectionX = math.sin(impactorHeading_rad) * impactordirectionZcos; #heading
		var impactorDirectionY = math.cos(impactorHeading_rad) * impactordirectionZcos; #heading
		var impactorDirectionZ = math.sin(impactorPitch_rad); #pitch
				
		# now we have a simple vector algebra problem: the impactor is at < 0,0,0 > moving
		# in the direction of the < impactorDirection > vector and the object is
		# at point < deltaX,deltaY,deltaAlt > .
		# So the closest approach of the line through < 0,0,0 > in the direction of < impactorDirection > 
		# to point < deltaX,deltaY,deltaAlt > is the length of the cross product  vector
		# < impactorDirection > X < deltaX,deltaY,deltaAlt > divided by the length of
		# < impactorDirection >.  We have cleverly chosen < impactDirection > so as to always
		# have length one (unit vector), so we can skip that calculation.
		# So the cross product vector:
				
		var crossProdX_m = impactorDirectionY * deltaAlt_m - impactorDirectionZ * deltaY_m;
		var crossProdY_m = impactorDirectionZ * deltaX_m   - impactorDirectionX * deltaAlt_m;
		var crossProdZ_m = impactorDirectionX * deltaY_m   - impactorDirectionY * deltaX_m;
				
		#the length of the cross-product vector divided by the length of the line/direction
		# vector is the distance we want (and the line/direction vector = 1 in our
		# setup:
		closestApproach_m = cartesianDistance(crossProdX_m,crossProdY_m,crossProdZ_m );
				
				
		#debprint( "closestApproach_m = ", closestApproach_m, " impactorDirectionX = ", impactorDirectionX,
		#" impactorDirectionY = ", impactorDirectionY,
		#" impactorDirectionZ = ", impactorDirectionZ,
		#" crossProdX_m = ", crossProdX_m,
		#" crossProdY_m = ", crossProdY_m,
		#" crossProdZ_m = ", crossProdZ_m,
		#" deltaX_m = ", deltaX_m,
		#" deltaY_m = ", deltaY_m,
		#" deltaAlt_m = ", deltaAlt_m,
		#" impactDist (lat/long) ", cartesianDistance(deltaX_m,deltaY_m,deltaAlt_m),
		#" shouldbeOne: ", cartesianDistance(impactorDirectionX,impactorDirectionY,impactorDirectionZ),
		#);
				
		#var impactSurfaceDistance_m = objectGeoCoord.distance_to(impactGeoCoord);
		#var heightDifference_m = math.abs(getprop (""~impactNodeName~"/impact/elevation-m") - getprop (""~nodeName~"/impact/altitude-ft") * FT2M);
	}

	var damAdd = 0; #total amount of damage actually added as the result of the impact
	var impactTerrain = getprop (""~impactNodeName~"/impact/type");
			
	#debprint ("Bombable: Possible hit - calculating . . . ", impactTerrain);

	#Potential for adding serious damage increases the closer we are to the center
	#of the object.  We'll say more than damageRadius meters away, no potential for increased damage
			
	var damageRadius_m = attributes[myNodeName].dimensions.damageRadius_m;
	var vitalDamageRadius_m = attributes[myNodeName].dimensions.vitalDamageRadius_m;
	# if it doesn't exist we assume it is 1/3 the damage radius
	if (!vitalDamageRadius_m) vitalDamageRadius_m = damageRadius_m/3;
			
	var vuls = attributes[myNodeName].vulnerabilities;
			
	ballisticMass_lb = getBallisticMass_lb(impactNodeName);
	var ballisticMass_kg = ballisticMass_lb/2.2;
			
	# Only worry about small arms/small cannon fire if it is a direct hit on the object;
	# if it hits terrain, then no damage.
	if (impactTerrain == "terrain" and ballisticMass_lb <= 1.2) {
		#debprint ("hit on terrain & mass < 1.2 lbs, exiting ");
		exit_test_impact(impactNodeName, myNodeName);
		return;
	}

	var impactVelocity_mps = getImpactVelocity_mps (impactNodeName, ballisticMass_lb);

	# How many shots does it take to down an object?  Supposedly the Red Baron
	# at times put in as many as 500 machine-gun rounds into a target to * make
	# sure * it really went down.
			
	var easyMode = 1;
	var easyModeProbability = 1;
			
	if (myNodeName != "" ) 
	{
		# Easy Mode increases the damage radius (2X), making it easier to score hits,
		# but doesn't increase the damage done by armament
		if (bombableMenu["easy-mode"]) {
			#easyMode *= 2;
			damageRadius_m *= 2;
			vitalDamageRadius_m *= 2;
		}
				
		# Super Easy mode increases both the damage radius AND the damage done
		# by 3X
		if (bombableMenu["super-easy-mode"]) {
			easyMode *= 3;
			easyModeProbability *= 3;
			damageRadius_m *= 3;
			vitalDamageRadius_m *= 3;
		}
	}

	#debprint ("Bombable: Projected closest impact distance delta : ", closestApproachOLDWAY_m-closestApproach_m, "FG Impact Detection Point delta: ", impactDistance_m - cartesianDistance(deltaX_m,deltaY_m,deltaAlt_m), " ballisticMass_lb = ", ballisticMass_lb);

	#var tgt_ht_m = 50/.3042 + 5; # AIManager.cxx it is 50 ft for aircraft & multiplayer;extra 5 m is fudge factor
	#var tgt_length_m = 100/.3024 + 5; # AIManager.cxx it is 100 ft for aircraft & multiplayer; extra 5 m is fudge factor
			
	# if impactterrain is aircraft or MP and the impact is within the tgt_alt and tgt_height, we're going to assume it is a direct impact on this object.
	# it would be much easier if FG would just pass us the node name of the object that has been hit,
	# but lacking that vital bit of info, we do it the hard way . . .
	
	#if(abs(iAlt_m-oAlt_m) < tgt_ht_m   and impactDistanceXY_m < tgt_length_m   and impactTerrain != "terrain") {

		#OK, it's within the damage radius - direct hit
		if (closestApproach_m < damageRadius_m) 
		{
										
			damagePotential = 0;
			outsideIDdamagePotential = 0;
			# Kinetic energy ranges from about 1500 joules (Vickers machine gun round) to
			# 200,000 joules (GAU-8 gatling gun round .8 lbs at 1000 MPS typical impact speed)
			# to 220,000 joules (GAU-8 at muzzle velocity)
			# to 330,000 joules (1.2 lb projectile at 1500 MPS muzzle velocity)
			# GAU-8 can penetrate an M-1 tank at impact.  But even there it would take a number of rounds,
			# perhaps a large number, to disable a tank reliably.  So let's say 20 rounds, and
			# our 100% damage amount is 20 GAU hits.
			#
			# Kinetic Energy (joules) = 1/2 * mass * velocity^2  (mass in kg, velocity in mps)
			# See http://en.wikipedia.org/wiki/Kinetic_energy
			#var kineticEnergy_joules = ballisticMass_kg * impactVelocity_mps * impactVelocity_mps /2;
					
			# According to this, weapon effectiveness isn't well correlated to kinetic energy, but
			# is better estimated in proportion to momentum
			# plus a factor for the chemical explosiveness of the round:
			#             http://eaw.wikispaces.com/Technical+Tools--Gun+Power
			#
			# We don't have a good way to estimate the chemical energy of particular rounds
			# (though it can be looked up) but momentum is easy: mass X velocity.
			#
			# Momentum ranges from 500 kg * m/s for a typical Vickers machine gun round
			# to 180 for a GAU-8 round at impact, 360 for  GAU-8 round at muzzle, 800 for
			# at 1.2 lb slug at 1500 mps
			#
			momentum_kgmps = ballisticMass_kg * impactVelocity_mps;
					
			weaponDamageCapability = momentum_kgmps / (60 * 360);
			#debprint ("mass = ", ballisticMass_lb, " vel = ", impactVelocity_mps, " Ek = ", kineticEnergy_joules, " damageCapability = ", weaponDamageCapability);
					
					
					
			# likelihood of damage goes up the closer we are to the center; it becomes 1 at vitalDamageRadius
					
			if (closestApproach_m <= vitalDamageRadius_m )impactLikelihood = 1;
			else impactLikelihood = (damageRadius_m - closestApproach_m)/(damageRadius_m -vitalDamageRadius_m);
					
					
					
			# It's within vitalDamageRadius, this is the core of the object--engines pilot, fuel tanks,
			# etc.  #So, some chance of doing high damage and near certainty of doing some damage
			if (closestApproach_m <= vitalDamageRadius_m )  {
				#damagePotential = (damageRadius_m - closestApproach_m)/damageRadius_m;
				damagePotential = impactLikelihood * vuls.damageVulnerability / 200; #possibility of causing a high amount of damage
				outsideIDdamagePotential = impactLikelihood; #possibility of causing a routine amount of damage
						
				#          debprint ("Bombable: Direct hit, "~ impactNodeName~ " on ", myNodeName, " Distance = ", closestApproach_m, " heightDiff = ", deltaAlt_m, " terrain = ", impactTerrain, " radius = ", damageRadius_m, " dP:", damagePotential, " oIdP:", outsideIDdamagePotential, " bM:", ballisticMass_lb);
						
						
				} else {
				#It's within damage radius but not vital damage Radius: VERY slim chance
				# of doing serious damage, like hitting a wing fuel tank or destroying a wing strut, and
				#some chance of doing routine damage
						
				damagePotential = impactLikelihood * vuls.damageVulnerability / 2000;
						
				# Think of a typical aircraft projected onto the 2D plane with damage radius &
				# vital damage radius superimposed over them.  For vital damage radius, it's right near
				# the center and  most of the area enclosed would be a hit.
				# But for the area between vital damage radius & damage
				# radius, there is much empty space--and the more so, the more outwards we go
				# towards the damage radius.  Squaring the oIdP takes this geometrical fact into
				# account--there is more and more area the further you go out, but more and more of
				# it is empty.  So there is less chance (approximately proportionate to
				# square of distance from center) of hitting something vital the further
				# you go out.
				#
				outsideIDdamagePotential = math.pow (impactLikelihood, 1.5) ;# ^2 makes it a bit too difficult to get a hit/let's try ^1.5 instead
						
				#           debprint ("Bombable: Near hit, "~ impactNodeName~ " on ", myNodeName, " Distance = ", closestApproach_m, " heightDiff = ", deltaAlt_m, " terrain = ", impactTerrain, " radius = ", damageRadius_m, " dP ", damagePotential, " OIdP ", outsideIDdamagePotential, " vitalHitchance% ", damagePotential * vuls.damageVulnerability * easyModeProbability * ballisticMass_lb / 5);
						
			}
					
					
					
					
			var damageCaused = 0;
			if (ballisticMass_lb < 1.2) 
				{
					# gun/small ammo
					# Guarantee of some damage, maybe big damage if it hits some vital parts
					# (the 'if' is a model for the percentage chance of it hitting some vital part,
					# which should happen only occasionally--and less occasionally for well-armored targets)
					# it always does at least 100% of weaponDamageCapability and up to 300%
					if ( rand() < damagePotential * easyModeProbability) 
					{
						damageCaused = (weaponDamageCapability + rand() * weaponDamageCapability * 2) * vuls.damageVulnerability * easyMode;
						#debprint ("Bombable: Direct Hit/Vital hit. ballisticMass: ", ballisticMass_lb," damPotent: ", damagePotential, " weaponDamageCapab:", weaponDamageCapability);

						debprint ("Bombable: Small weapons, direct hit, very damaging");
								
						#Otherwise the possibility of damage
					}
					elsif (rand() < outsideIDdamagePotential) 
					{
						damageCaused = rand () * weaponDamageCapability * vuls.damageVulnerability * easyMode * outsideIDdamagePotential;
						#debprint ("Bombable: Direct Hit/Nonvital hit. ballisticMass: ", ballisticMass_lb," outsideIDDamPotent: ", outsideIDdamagePotential, " weaponDamageCapab:", weaponDamageCapability  );

						debprint ("Bombable: Small weapons, direct hit, damaging");
					}
				}
				else 
				{
				# anything larger than 1.2 lbs making a direct hit, e.g.some kind of bomb or exploding ordinance
				# debprint ("larger than 1.2 lbs, making direct hit");
						
				var damagePoss = .6 + ballisticMass_lb / 250;

				if (damagePoss > 1) damagePoss = 1;
				# if it hits a vital spot (which becomes more likely, the larger the bomb)
				if ( rand() < damagePotential * vuls.damageVulnerability * easyModeProbability * ballisticMass_lb / 5  ) 
				damageCaused = damagePoss * vuls.damageVulnerability * ballisticMass_lb * easyMode/2;
				else  #if it hits a regular or less vital spot
				damageCaused = rand () * ballisticMass_lb * vuls.damageVulnerability * easyMode * outsideIDdamagePotential;

				debprint ("Bombable: Heavy weapon or bomb, direct hit, damaging");
						
				}

			#debprint ("Bombable: Damaging hit, "~ " Distance = ", closestApproach_m, "by ", impactNodeName~ " on ", myNodeName," terrain = ", impactTerrain, " damageRadius = ", damageRadius_m," weaponDamageCapability ", weaponDamageCapability, " damagePotential ", damagePotential, " OIdP ", outsideIDdamagePotential, " Par damage: ", weaponDamageCapability * vuls.damageVulnerability);

			damAdd = add_damage( damageCaused, "weapon", myNodeName, , impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m  );

					
			#checking/setting this prevents the same splash from being repeatedly re-drawn
			# as we check the impact from different AI objects

					
				if (damageCaused > 0 ) 
				{
					#places a gun flack at the hit location
							
					if (myNodeName == "") 
					{
						#case of MainAC, we just draw the impact where FG has detected it

						exit_test_impact(impactNodeName, myNodeName);
								
						} else {
						#case of AI or MP Aircraft, we draw it at point of closest impact
								
						# Code below calculates < crossProdObj_Imp > , the vector from the
						# object location to the closest approach/impact point
						# Vector < crossProd > has the right magnitude but is perpendicular to the plane
						# containing the impact detection point, the closest approach point, and the
						# object location.  Doing the cross product of < crossProd > with < impactorDirection > (which
						# is a unit vector in the direction of impactor travel) gives the vector
						# in the direction from object location to impact closest approach point, and (since < impactorDirection > is the unit vector and < crossProd > 's magnitude is the distance from
						# the object location to the closest approach point, that vector's magnitude is the
						# distance from object location to closest approach point.
						#
						# Between this and < impactorDirection > we have the exact location and the direction
						# of the closest impact point.  These two items together could be used to calculate specific damage,
						# systems affected, etc., by damage coming at a specific angle in a specific area.
								
						var crossProdObj_ImpX_m = impactorDirectionY * crossProdZ_m - impactorDirectionZ * crossProdY_m;
						var crossProdObj_ImpY_m = impactorDirectionZ * crossProdX_m - impactorDirectionX * crossProdZ_m;
						var crossProdObj_ImpZ_m = impactorDirectionX * crossProdY_m - impactorDirectionY * crossProdX_m;
								
						debprint ("Bombable: Put splash direct hit");
						put_splash (impactNodeName, oLat_deg+crossProdObj_ImpY_m/m_per_deg_lat, oLon_deg+crossProdObj_ImpX_m/m_per_deg_lon,
						oAlt_m+crossProdObj_ImpZ_m, ballisticMass_lb, impactTerrain, 1, myNodeName);
					}
				}
					
			# end, case of direct hit
			}
			else 
			{
			# case of a near hit, on terrain, if it's a bomb we'll add damage
			# Some of the below is a bit forward thinking--it includes some damage elements to 1000 m
			# or even more distance for very large bombs.  But up above via the quick lat/long calc
			#  (for performance reasons) we're exiting immediately for impacts > 300 meters or so away.

			#debprint ("near hit, not direct");
			if (myNodeName == "") 
				{
				# In case of MainAC, we just draw the impact where FG has detected it,
				# not calculating any refinements, which just case problems in case of the
				# mainAC, anyway.

				exit_test_impact(impactNodeName, myNodeName);
						
				}
				else
				{
				#case of AI or MP aircraft, we draw the impact at point of closest approach
						

				var impactSplashPlaced = getprop (""~impactNodeName~"/impact/bombable-impact-splash-placed");
				var impactRefinedSplashPlaced = getprop (""~impactNodeName~"/impact/bombable-impact-refined-splash-placed");
				#debprint("iSP = ",impactSplashPlaced, " iLat = ", iLat_deg);
				if ( (impactSplashPlaced == nil or impactSplashPlaced != iLat_deg)
				and (impactRefinedSplashPlaced == nil or impactRefinedSplashPlaced != iLat_deg)
				and ballisticMass_lb > 1.2) 
					{
						var crossProdObj_ImpX_m = impactorDirectionY * crossProdZ_m - impactorDirectionZ * crossProdY_m;
						var crossProdObj_ImpY_m = impactorDirectionZ * crossProdX_m - impactorDirectionX * crossProdZ_m;
						var crossProdObj_ImpZ_m = impactorDirectionX * crossProdY_m - impactorDirectionY * crossProdX_m;
								
						debprint ("Bombable: Put splash near hit > 1.2 ", ballisticMass_lb, " ", impactNodeName);
						put_splash (impactNodeName,
						oLat_deg+crossProdObj_ImpY_m/m_per_deg_lat,
						oLon_deg+crossProdObj_ImpX_m/m_per_deg_lon,
						oAlt_m+crossProdObj_ImpZ_m, ballisticMass_lb,
						impactTerrain, 1, myNodeName );
					}
				}
					
				if (ballisticMass_lb > 1.2) {
							
					debprint ("Bombable: Close hit by bomb, "~ impactNodeName~ " on "~ myNodeName~ " Distance = "~ closestApproach_m ~ " terrain = "~ impactTerrain~ " radius = "~ damageRadius_m~" mass = "~ballisticMass_lb);
				}

						
						

				# check submodel blast effect distance.
				# different cases for each size of ordnance and distance of hit
				if (ballisticMass_lb < 1.2 )
				{
					#do nothing, just a small round hitting on terrain nearby
							
				}
				elsif (ballisticMass_lb < 10 and ballisticMass_lb >= 1.2 )  
				{
					if(closestApproach_m <= 10 + damageRadius_m)
					damAdd = add_damage(.1 * vuls.damageVulnerability * ballisticMass_lb / 10 * easyMode, "weapon", myNodeName, , impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);

					elsif((closestApproach_m > 10 + damageRadius_m) and (closestApproach_m < 30 + damageRadius_m)){
						var damFactor = (30 - closestApproach_m)/30;
						if (damFactor < 0) damFactor = 0;
						if (rand() < damFactor) damAdd = add_damage(0.0002 * vuls.damageVulnerability * ballisticMass_lb/10 * easyMode, "weapon", , myNodeName, impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					}
							
					} elsif  (ballisticMass_lb < 50 and ballisticMass_lb >= 10 ) {
					if(closestApproach_m <= .75 + damageRadius_m)
					damAdd = add_damage(.3 * vuls.damageVulnerability * ballisticMass_lb /50 * easyMode, "weapon", myNodeName, , impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);

					elsif((closestApproach_m > .75 + damageRadius_m) and (closestApproach_m <= 10 + damageRadius_m))
					damAdd = add_damage(.0001 * vuls.damageVulnerability * ballisticMass_lb /50 * easyMode, "weapon",  myNodeName, , impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);

					elsif((closestApproach_m > 10 + damageRadius_m) and (closestApproach_m < 30 + damageRadius_m))
					damAdd = add_damage(0.00005 * vuls.damageVulnerability * ballisticMass_lb /50 * easyMode, "weapon", myNodeName, , impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);

					elsif((closestApproach_m > 30 + damageRadius_m) and (closestApproach_m < 60 + damageRadius_m)){
						var damFactor = (60 - closestApproach_m)/60;
						if (damFactor < 0) damFactor = 0;
						if (rand() < damFactor) damAdd = add_damage(0.0002 * vuls.damageVulnerability * ballisticMass_lb/50 * easyMode, "weapon", myNodeName, , impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					}

					else{
						var damFactor = (100 - closestApproach_m)/100;
						if (damFactor < 0) damFactor = 0;
						if (rand() < damFactor) damAdd = add_damage(0.0001 * vuls.damageVulnerability * ballisticMass_lb/350 * easyMode, "weapon", myNodeName, , impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					}

					} elsif (ballisticMass_lb < 200 and ballisticMass_lb >= 50 ) {
					if(closestApproach_m <= 1.5 + damageRadius_m)
					damAdd = add_damage(1 * vuls.damageVulnerability * ballisticMass_lb/200 * easyMode, "weapon", myNodeName, , impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					elsif((closestApproach_m > 1.5 + damageRadius_m) and (closestApproach_m <= 10 + damageRadius_m))
					damAdd = add_damage(.01 * vuls.damageVulnerability * ballisticMass_lb /200 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
							
					elsif((closestApproach_m > 10 + damageRadius_m) and (closestApproach_m < 30 + damageRadius_m))
					damAdd = add_damage(0.0001 * vuls.damageVulnerability * ballisticMass_lb/200 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					elsif((closestApproach_m > 30 + damageRadius_m) and (closestApproach_m < 60 + damageRadius_m)){
						var damFactor = (75-closestApproach_m)/75;
						if (damFactor < 0) damFactor = 0;

						if (rand() < damFactor) damAdd = add_damage(0.0002 * vuls.damageVulnerability * ballisticMass_lb/200 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					}
							
					else{
						var damFactor = (100 - closestApproach_m)/100;
						if (damFactor < 0) damFactor = 0;
						if (rand() < damFactor) damAdd = add_damage(0.0001 * vuls.damageVulnerability * ballisticMass_lb/350 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					}

					} elsif ((ballisticMass_lb >= 200) and (ballisticMass_lb < 350)) {
					# Mk-81 class
					# Source: http://en.wikipedia.org/wiki/General-purpose_bomb
					# Estimated: crater = 2 m, lethal blast = 12 m, casualty radius (50%) = 25 m, blast shrapnel ~70m, fragmentation  ~=  250 m
					# All bombs adjusted downwards outside of crater/lethal blast distance,
					# based on flight testing plus:
					# http://www.f-16.net/f-16_forum_viewtopic-t-10801.html

					if(closestApproach_m <= 2 + damageRadius_m)
					damAdd = add_damage(2 * vuls.damageVulnerability * ballisticMass_lb/350 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					elsif((closestApproach_m > 2 + damageRadius_m) and (closestApproach_m <= 12 + damageRadius_m))
					damAdd = add_damage(.015 * vuls.damageVulnerability * ballisticMass_lb /350 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					elsif((closestApproach_m > 12 + damageRadius_m) and (closestApproach_m < 25 + damageRadius_m))
					damAdd = add_damage(0.0005 * vuls.damageVulnerability * ballisticMass_lb/350 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					elsif((closestApproach_m > 25 + damageRadius_m) and (closestApproach_m < 70 + damageRadius_m))  {
						var damFactor = (90-closestApproach_m)/90;
						if (damFactor < 0) damFactor = 0;

						if (rand() < damFactor) damAdd = add_damage(0.0002 * vuls.damageVulnerability * ballisticMass_lb/350 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					}
					else{
						var damFactor = (250-closestApproach_m)/250;
						if (damFactor < 0) damFactor = 0;
						if (rand() < damFactor) damAdd = add_damage(0.0001 * vuls.damageVulnerability * ballisticMass_lb/350 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					}

					} elsif((ballisticMass_lb >= 350) and (ballisticMass_lb < 750)) {
					# Mk-82 class  (500 lb)
					# crater = 4 m, lethal blast = 20 m, casualty radius (50%) = 60 m, blast shrapnel ~100m, fragmentation  ~=  500 m
					# http://www.khyber.org/publications/006-010/usbombing.shtml
					if(closestApproach_m <= 4 + damageRadius_m )
					damAdd = add_damage(4 * vuls.damageVulnerability * ballisticMass_lb /750 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					elsif((closestApproach_m > 4 + damageRadius_m) and (closestApproach_m <= 20 + damageRadius_m))
					damAdd = add_damage(.02 * vuls.damageVulnerability * ballisticMass_lb /750 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					elsif((closestApproach_m > 20 + damageRadius_m) and (closestApproach_m <= 60 + damageRadius_m))
					damAdd = add_damage(0.001 * vuls.damageVulnerability * ballisticMass_lb /750 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					elsif((closestApproach_m > 60 + damageRadius_m) and (closestApproach_m <= 100 + damageRadius_m)) {
						var damFactor = (120-closestApproach_m)/120;
						if (damFactor < 0) damFactor = 0;

						if (rand() < damFactor) damAdd = add_damage(0.0002 * vuls.damageVulnerability * ballisticMass_lb/350 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					}
							
					else{
						var damFactor = (500-closestApproach_m)/500;
						if (damFactor < 0) damFactor = 0;
						if (rand() < damFactor) damAdd = add_damage(0.0001 * vuls.damageVulnerability * ballisticMass_lb/350 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					}
							
					} elsif((ballisticMass_lb >= 750) and (ballisticMass_lb < 1500)) {
					# Mk-83 class (1000 lb)
					# crater = 11 m, lethal blast ~=  27 m, casualty radius (50%) ~=  230 m, blast shrapnel 190m, fragmentation 1000 m
					# http://www.khyber.org/publications/006-010/usbombing.shtml

					if(closestApproach_m <= 11 + damageRadius_m )
					damAdd = add_damage(8 * vuls.damageVulnerability * ballisticMass_lb/1500 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					elsif((closestApproach_m > 11 + damageRadius_m) and (closestApproach_m <= 27 + damageRadius_m))
					damAdd = add_damage(.02 * vuls.damageVulnerability * ballisticMass_lb /1500 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);

					elsif((closestApproach_m > 27 + damageRadius_m) and (closestApproach_m <= 190 + damageRadius_m))
					damAdd = add_damage(0.001 * vuls.damageVulnerability * ballisticMass_lb/1500 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					elsif((closestApproach_m > 190 + damageRadius_m) and (closestApproach_m <= 230 + damageRadius_m)){
						var damFactor = (230-closestApproach_m)/230;
						if (damFactor < 0) damFactor = 0;

						if (rand() < damFactor) damAdd = add_damage(0.0002 * vuls.damageVulnerability * ballisticMass_lb/350 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					}
							
					else {
						var damFactor = (1000-closestApproach_m)/1000;
						if (damFactor < 0) damFactor = 0;
						if (rand() < damFactor) damAdd = add_damage(0.0001 * vuls.damageVulnerability * ballisticMass_lb/350 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					}

					} elsif(ballisticMass_lb >= 1500 ) {
					# Mk-84 class (2000 lb) and upper
					# crater = 18 m, lethal blast = 34 m, casualty radius (50%) = 400 m, blast shrapnel 380m, fragmentation = 1000 m
					# http://www.khyber.org/publications/006-010/usbombing.shtml

					if(closestApproach_m <= 18 + damageRadius_m )
					damAdd = add_damage(16 * vuls.damageVulnerability * ballisticMass_lb/3000 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					elsif((closestApproach_m > 18 + damageRadius_m) and (closestApproach_m <= 34 + damageRadius_m))
					damAdd = add_damage(.02 * vuls.damageVulnerability * ballisticMass_lb /3000 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);

					elsif((closestApproach_m > 34 + damageRadius_m) and (closestApproach_m <= 380 + damageRadius_m))
					damAdd = add_damage(0.001 * vuls.damageVulnerability * ballisticMass_lb/3000 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					elsif((closestApproach_m > 380 + damageRadius_m) and (closestApproach_m <= 500 + damageRadius_m)){
						var damFactor = (500-closestApproach_m)/500;
						if (damFactor < 0) damFactor = 0;

						if (rand() < damFactor) damAdd = add_damage(0.0002 * vuls.damageVulnerability * ballisticMass_lb/350 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					}
							
					else {
						var damFactor = (1500-closestApproach_m)/1500;
						if (damFactor < 0) damFactor = 0;
						if (rand() < damFactor) damAdd = add_damage(0.0001 * vuls.damageVulnerability * ballisticMass_lb/350 * easyMode, "weapon", myNodeName, ,  impactNodeName, ballisticMass_lb, iLat_deg, iLon_deg, iAlt_m);
					}
							
				}
						
			}

		var node = props.globals.getNode(myNodeName);
  		var type=node.getName();

		if ( type != "multiplayer" and myNodeName != "" ) 
		{
			#any impacts somewhat close to us, we start dodging - if we're a good pilot.
					
			var skill = calcPilotSkill (myNodeName);
			if ( closestApproach_m < 500 and rand() < skill/14 ) dodge (myNodeName);
					
			# but even numbskull pilots start dodging if there is a direct hit!
			# Unless distracted ((rand() < .20 - 2 * skill/100)) is a formula for
			# distraction, assumed to be lower for more skilled pilots
					
			elsif ( damAdd > 0 and (rand() < .20 - 2 * skill/100) ) dodge (myNodeName);
		}
				
	}


############################# speed_adjust ############################
# FUNCTION speed_adjust
#
# adjusts airspeed of an AI object depending on whether it is climbing, diving,
# or ~level flight
#
# TODO: We could also adjust speed based on the roll angle or turn rate (turning
# too sharp reduces speed and this is one of the primary constraints on sharp
# turns in fighter aircraft)
#
var speed_adjust = func (myNodeName, time_sec ) {
	var ctrls = attributes[myNodeName].controls;
	if (ctrls.onGround) return;
				
	var stalling = 0;
	var vels = attributes[myNodeName].velocities;
				
	var airspeed_kt = getprop (""~myNodeName~"/velocities/true-airspeed-kt");
	if (airspeed_kt <= 0) airspeed_kt = .000001; #avoid the div by zero issue
	airspeed_fps = airspeed_kt * KT2FPS;
	var vertical_speed_fps = getprop (""~myNodeName~"/velocities/vertical-speed-fps");
	
	var maxSpeed_kt = vels.maxSpeed_kt;
	if (maxSpeed_kt <= 0) maxSpeed_kt = 90;

	# The AI airspeed_kt is true airspeed (TAS) which is quite different from
	# indicated airspeed (IAS) at altitude.
	# Stall speed is (approximately) constant in IAS, regardless of altitude,
	# so it is best to use IAS to determine the real stall speed.
	# By contrast, max Speed (Vne) seems more independent of altitude

	var minSpeed_kt = trueAirspeed2indicatedAirspeed (myNodeName, vels.minSpeed_kt);
	if (minSpeed_kt <= 0) minSpeed_kt = 40;
	var sin_pitch = vertical_speed_fps/airspeed_fps;

	if (sin_pitch > 1) sin_pitch = 1;
	if (sin_pitch < -1) sin_pitch = -1;
				
	var add_velocity_fps = 0;
	var termVel_kt = 0;
				
	if (ctrls.attackInProgress or ctrls.dodgeInProgress ) 
	{
		targetSpeed_kt = vels.attackSpeed_kt;
	}
	elsif ( stores.fuelLevel (myNodeName) < .2 ) 
	{
		#reduced speed if low on fuel
		targetSpeed_kt = (vels.cruiseSpeed_kt + vels.minSpeed_kt )/2;
	}
	else
	{
		targetSpeed_kt = vels.cruiseSpeed_kt;
	}
				
	# some failsafe defaults; if we don't have min < target < max
	# our formulas below can fail horribly
	if (targetSpeed_kt <= minSpeed_kt) targetSpeed_kt = minSpeed_kt + 20;
	if (maxSpeed_kt <= targetSpeed_kt) maxSpeed_kt = targetSpeed_kt * 1.5;
				
	#reduce A/C speed when turning at a high roll rate
	#this is a bit of a kludge, but reduces target speed from attack
	#to cruise speed as roll degrees goes from 70 to 80, which about
	#matches the performance of Zero & F6F in FG.
	#this probably needs to be set/individualized per AC
	var sustainRollLimit_deg = 70;
	var sustainRollLimitTransition_deg = 10;
	var currRoll_deg = getprop(""~myNodeName~"/orientation/roll-deg");
	if (math.abs(currRoll_deg) > sustainRollLimit_deg) 
	{
		if (math.abs(currRoll_deg) > sustainRollLimit_deg + sustainRollLimitTransition_deg)
		{
			targetSpeed_kt = vels.cruiseSpeed_kt;
		}
		else 
		{
			targetSpeed_kt = (vels.attackSpeed_kt - vels.cruiseSpeed_kt )
			 * (currRoll_deg - sustainRollLimit_deg)
			+ vels.cruiseSpeed_kt;
		}
	}
				
	#level flight, we tend towards our cruise or attack speed
	# we're calling less then 5 in 128 climb or dive, level flight
	if (math.abs(sin_pitch) < 5/128 ) 
	{
		if (targetSpeed_kt <= 0) targetSpeed_kt = 50;
		var calcspeed_kt = airspeed_kt;
		if (airspeed_kt < targetSpeed_kt) 
		{
			if (calcspeed_kt < minSpeed_kt) calcspeed_kt = minSpeed_kt;
			var fact = 1-(calcspeed_kt-minSpeed_kt)/(targetSpeed_kt-minSpeed_kt);
		} 
		else
		{
			if (calcspeed_kt > maxSpeed_kt) calcspeed_kt = maxSpeed_kt;
			var fact = 1-(maxSpeed_kt-calcspeed_kt)/(maxSpeed_kt-targetSpeed_kt);
		}
					
		# the / 70 may require tweaking or customization. This basically goes to how
		# much acceleration the AC has.   / 70 matches closely the A6M2 Zero's
		# acceleration during level flight
		add_velocity_fps = math.sgn (targetSpeed_kt - airspeed_kt) * math.pow(math.abs(fact),0.5) * targetSpeed_kt * time_sec * KT2FPS / 70 ;
		termVel_kt = targetSpeed_kt;
		#debprint ("Bombable: Speed Adjust, level:", add_velocity_fps * fps2knots, " airspeed: ", airspeed_kt, " termVel: ", termVel_kt, " ", myNodeName );
	} 
	elsif (sin_pitch > 0 ) 
	{
		# climbing, so we reduce our airspeed, tending towards V (s)
		var deltaSpeed_kt = airspeed_kt-minSpeed_kt;
					
		# debprint ("Bombable: deltaS",deltaSpeed_kt, " maxS:", maxSpeed_kt, " minS:", minSpeed_kt," grav:",  grav_fpss, " timeS:", time_sec," sinP",  sin_pitch   );
		# add_velocity_fps = -(deltaSpeed_kt/(maxSpeed_kt-minSpeed_kt)) * grav_fpss * time_sec * sin_pitch * 10;
		#
					
		# termVel_kt is the terminal velocity for this particular angle of attack
		# if we could get a more accurate formula for the terminal velocity for
		# each angle of attack this would be even more realistic
		# cal ranges 0-1 (though cal 1 . . . infinity is possible)
		# and generally smaller cal makes the terminal velocity
		# slower for lower angles of attack.  so if your aircraft is going too
		# fast when climbing (compared with the similar 'real' aircraft in bombable)
		# make cal smaller.  cal = .13 seems about right for
		# Sopwith Camel, with vel1^2/vel2^2  for Zero, cal = .09 and ^3/^3
		# var cal = .09;
		# termVel_kt = targetSpeed_kt - math.pow(math.abs(sin_pitch),cal) * (targetSpeed_kt-minSpeed_kt);
					
		termVel_kt = targetSpeed_kt - vels.climbTerminalVelocityFactor * math.abs(sin_pitch);
					
		#In the case of diving, we're going to assume that the pilot will cut
		# power, add slats, add flaps, or whatever to keep the speed below
		# Vne.  However in the case of climbing, there is no such limit.
		# If you keep climbing you will eventually reach vel = 0 and even negative
		# velocity.
		#if (termVel_kt < minSpeed_kt) termVel_kt = minSpeed_kt;
					
					
		# This formula approaches 0 add_velocity as airspeed approaches termVel
					
		vel1 = maxSpeed_kt - airspeed_kt;
		vel2 = maxSpeed_kt - termVel_kt;
					
		add_velocity_fps = - (1 - (vel1/vel2)) * grav_fpss * time_sec * 1.5;
					
					
					
		# debprint ("Bombable: Speed Adjust, climbing:", add_velocity_fps * fps2knots, " airspeed: ", airspeed_kt, " termVel: ", termVel_kt, " ", myNodeName );
	} 
	elsif (sin_pitch < 0 )
	{
		# diving, so we increase our airspeed, tending towards the V(ne)
					
		# termVel_kt is the terminal velocity for this particular angle of attack
		# if we could get a more accurate formula for the terminal velocity for
		# each angle of attack this would be even more realistic
		#
		# cal generally ranges from 0 to infinity and the higher cal the slower
		# terminal velocity for low angles of attack.  If your aircraft don't
		# gain enough speed on dive, make cal smaller, down to 1 or possibly
		# even below. cal = 1.5 seems about right for Sopwith Camel.a^2/Vt^2 and g * t * 1
		# For Zero, cal = 1.0, a^3/Vt^3 and g * t * 2 is a better fit.
		# var cal = 1.0;
		# termVel_kt = math.pow (math.abs(sin_pitch), cal) * (maxSpeed_kt-targetSpeed_kt) + targetSpeed_kt;
					
		termVel_kt = targetSpeed_kt + vels.diveTerminalVelocityFactor * math.abs(sin_pitch);
					
		# We're assuming the pilot will take action to keep it below maxSpeed_kt,
		# such as reducing engine, slats, flaps, etc etc etc.  In some cases this
		# may not be realistic but
		if (termVel_kt > maxSpeed_kt) termVel_kt = maxSpeed_kt;
					
		add_velocity_fps = (1 - math.abs(airspeed_kt/termVel_kt)) * grav_fpss * time_sec/1.5;
		# debprint
		# (
		# 	sprintf
		# 	(
		# 		"Bombable: Speed Adjust, diving: %6.1f airspeed: %6.1f termVel: %6.1f %s",
		# 		add_velocity_fps * fps2knots,
		# 		airspeed_kt,
		# 		termVel_kt,
		# 		myNodeName 
		# 	)
		# );
	}
					
	# if we're above maxSpeed we make a fairly large/quick correction
	# but only if it is larger (in negative direction) than the regular correction
	if (airspeed_kt > maxSpeed_kt) 
	{
		maxS_add_velocity_fps = (maxSpeed_kt-airspeed_kt)/10 * time_sec * KT2FPS;
		if ( maxS_add_velocity_fps < add_velocity_fps)
		add_velocity_fps = maxS_add_velocity_fps;
	}
		
				
				
	#debprint ("Bombable: Speed Adjust:", add_velocity_fps * fps2knots, " TermVel:", termVel_kt, "sinPitch:", sin_pitch );
	var finalSpeed_kt = airspeed_kt + add_velocity_fps * fps2knots;
	#Zero/negative airspeed causes problems . . .
	if (finalSpeed_kt < minSpeed_kt / 3) finalSpeed_kt = minSpeed_kt / 3;
	setprop (""~myNodeName~"/controls/flight/target-spd", finalSpeed_kt);
	setprop (""~myNodeName~"/velocities/true-airspeed-kt", finalSpeed_kt);
				
	if (finalSpeed_kt < minSpeed_kt)
	{
		stalling = 1;
					
		# When we stall & both vertical speed & airspeed go to zero, FG just flips
		# out.  If we're stalling then gravity takes over, no lift, so we make
		# that happen here.
		vertical_speed_fps  -=  grav_fpss * time_sec;
		setprop ( ""~myNodeName~"/velocities/vertical-speed-fps", vertical_speed_fps );
					
	}
				
	#The vertical speed should never be greater than the airspeed, otherwise
	#   something (ie one of the bombable routines) is adding in extra
	#   energy to the AC.
	finalSpeed_fps = finalSpeed_kt * KT2FPS;
	if (math.abs(vertical_speed_fps) > math.abs(finalSpeed_fps))
	{
		setprop (""~myNodeName~"/velocities/vertical-speed-fps",math.sgn (vertical_speed_fps) * math.abs(finalSpeed_fps));
	}
				
	ctrls.stalling = stalling;
	#make the aircraft's pitch match it's vertical velocity; otherwise it looks fake
	setprop (""~myNodeName~"/orientation/pitch-deg", math.asin(sin_pitch) * R2D);


}

#################################### speed_adjust_loop ##################################
var speed_adjust_loop = func ( id, myNodeName, looptime_sec) {
	id == attributes[myNodeName].loopids.speed_adjust_loopid or return;
				
	settimer (  func { speed_adjust_loop (id, myNodeName, looptime_sec)}, looptime_sec);

	#debprint ("speed_adjust_loop starting");

	if (! getprop (bomb_menu_pp~"ai-aircraft-attack-enabled") or ! bombableMenu["bombable-enabled"] ) return;
				
	speed_adjust (myNodeName, looptime_sec);

}

############################# altitude_adjust ############################
# adjusts altitude of a groundvehicle
#
var altitude_adjust = func (myNodeName, alt_ft, count, delta_alt, delta_t, N_STEPS) {
	var new_alt = alt_ft + delta_alt;

	count += 1; 
	if (count < N_STEPS)
	{
		settimer(func
		{
			setprop (""~myNodeName~"/position/altitude-ft", new_alt);
			altitude_adjust(myNodeName, new_alt, count, delta_alt, delta_t, N_STEPS);
		},
		delta_t
		);
	}
}

################################## do_acrobatic_loop_loop ####################################
# FUNCTION do_acrobatic_loop_loop
# The settimer loop to do an acrobatic loop, up or down, or part of a loop
#

var do_acrobatic_loop_loop = func 
(
	id, myNodeName, loop_time = 20, full_loop_steps = 100, exit_steps = 100, direction = "up", 
	rolldirenter = "cc", rolldirexit = "ccw", vert_speed_add_kt = 225, loop_count = 0  
)
{
	#same loopid as roll so one can interrupt the other
	id == attributes[myNodeName].loopids.roll_loopid or return;
				
	if (direction == "up") var dir = 1;
	else var dir = -1;
				
	var vert_speed_add_fps = vert_speed_add_kt * KT2FPS;
	# we want to accelerate vertically by vert_speed_add over the first 1/4 of the loop; then back to 0 over the next 1/4 of the loop, then to - vert_speed_add
	# over the next 1/4 of the loop, then back to 0 over the last 1/4.
	var vert_speed_add_per_step_fps = vert_speed_add_fps * 4 / full_loop_steps;
				
	#we'll never put something greater than the AC's maxSpeed into the vertical
	# velocity
	var vels = attributes[myNodeName].velocities;
	var alts = attributes[myNodeName].altitudes;

				
	maxSpeed_fps = vels.maxSpeed_kt * KT2FPS;
				
	#or greater than the current speed
	currSpeed_kt = getprop (""~myNodeName~"/velocities/true-airspeed-kt");
	currSpeed_fps = currSpeed_kt * KT2FPS;
				
	currAlt_ft = getprop(""~myNodeName~"/position/altitude-ft");
	currAlt_m = currAlt_ft * FT2M;
				
				
	#we use main AC elev as a stand-in for our own elevation, since the elev
	# function is so slow.  A bit of a kludge.
	var mainACElev_m = getprop ("/position/ground-elev-m");
				
	var stalling = attributes[myNodeName].controls.stalling;
				
	#if we stall out or exceed the maxSpeed or lower than minimum allowed altitude
	#    then we terminate the loop & the dodge
	if (stalling or currSpeed_kt > vels.maxSpeed_kt or currSpeed_kt < vels.minSpeed_kt * 1.1 ) 
	{
		debprint ("Bombable: Exiting loop " ~myNodeName ~ ": ", stalling, " ", currSpeed_kt, "currAlt: ", currAlt_m );
		attributes[myNodeName].controls.dodgeInProgress = 0;
		return;
	}
	# debprint("Bombable:  do_acrobatic_loop_loop ", loop_time, " ", full_loop_steps, " ", exit_steps, " ", direction, " ", rolldirenter, " ", rolldirexit, " ", vert_speed_add_kt, " ", loop_count);
	loop_count += 1;
	if (loop_count <= exit_steps ) settimer (func { do_acrobatic_loop_loop(id, myNodeName, loop_time, full_loop_steps, exit_steps, direction, rolldirenter, rolldirexit,vert_speed_add_kt, loop_count);}, loop_time/full_loop_steps);
				
	var curr_vertical_speed_fps = getprop ("" ~ myNodeName ~ "/velocities/vertical-speed-fps");
	var curr_acrobat_vertical_speed_fps = vels.curr_acrobat_vertical_speed_fps;
				
				
				
	if (loop_count < full_loop_steps/4 or loop_count >= full_loop_steps * 3/4) var localdir = 1;
	else  var localdir = -1;
				
	curr_acrobat_vertical_speed_fps += localdir * dir * vert_speed_add_per_step_fps;
	vels.curr_acrobat_vertical_speed_fps = curr_acrobat_vertical_speed_fps;

	# EXPERIMENT			
	# var proposed_vertical_speed_fps = curr_vertical_speed_fps + localdir * dir * vert_speed_add_per_step_fps;
	# we only add the adjustments to the vertical speed when the amount
	# it 'should be' is greater (in magnitude) than the current vertical speed
	#var sgn = math.sgn (curr_acrobat_vertical_speed_fps);
	#if ( sgn * curr_acrobat_vertical_speed_fps >= sgn * proposed_vertical_speed_fps) setprop ("" ~ myNodeName ~ "/velocities/vertical-speed-fps", proposed_vertical_speed_fps);
				
	# debprint ("Bombable: Acrobatic loop, ideal vertfps: ", curr_acrobat_vertical_speed_fps );
				
	#The FG vert-speed prop sort of wiggles around for various reasons,
	# so we are just basically going to force it where we want it, no
	# matter what.
	# However, with these limits:
	#The vert speed should never be set larger than the maxSpeed

	curr_acrobat_vertical_speed_fps = checkRange (curr_acrobat_vertical_speed_fps, -maxSpeed_fps, maxSpeed_fps, curr_acrobat_vertical_speed_fps);
				
	# The vert speed should never be set larger than the current speed
	# We're just changing the direction of the motion here, not adding any
	# new speed or energy.
	curr_acrobat_vertical_speed_fps = checkRange (curr_acrobat_vertical_speed_fps, -currSpeed_fps, currSpeed_fps, curr_acrobat_vertical_speed_fps);
				
	# To avoid weird looking bumpiness, we're never going to change the current vert speed by more than 2X vert_speed_add_fps at a time.
	curr_acrobat_vertical_speed_fps = checkRange (curr_acrobat_vertical_speed_fps,
	curr_vertical_speed_fps - 2 * vert_speed_add_per_step_fps,
	curr_vertical_speed_fps + 2 * vert_speed_add_per_step_fps,
	curr_acrobat_vertical_speed_fps);
				
	# If we are below the minimumAGL for this a/c we avoid putting
	# any more negative vertical speed into the a/c than it already has
	if (currAlt_m - mainACElev_m < alts.minimumAGL_m) 
	{
		curr_acrobat_vertical_speed_fps = checkRange (curr_acrobat_vertical_speed_fps,
		curr_vertical_speed_fps,
		curr_vertical_speed_fps + 2 * vert_speed_add_per_step_fps,
		curr_acrobat_vertical_speed_fps);
	}
				
	# If we are getting close to the minimumAGL for this a/c we limit putting
	# more negative vertical speed into the a/c than it already has
	if (currAlt_m - mainACElev_m < alts.minimumAGL_m + 200) 
	{
		curr_acrobat_vertical_speed_fps = checkRange 
		(
			curr_acrobat_vertical_speed_fps,
			curr_vertical_speed_fps - vert_speed_add_per_step_fps/2,
			curr_vertical_speed_fps + 2 * vert_speed_add_per_step_fps,
			curr_acrobat_vertical_speed_fps
		);
	}
				
	setprop ("" ~ myNodeName ~ "/velocities/vertical-speed-fps", curr_acrobat_vertical_speed_fps);
	# debprint ("Bombable: Acrobatic loop, actual vertfps: ", curr_acrobat_vertical_speed_fps, "previous vertspd:",  curr_vertical_speed_fps);
				
	#target-alt will affect the vert speed unless we keep it close to current alt
	setprop (""~myNodeName~"/controls/flight/target-alt", currAlt_ft);
				
	# AI aircraft don't take kindly to flying upside down
	# so we just change their heading angle to roll them right-side up instead.
	# However, instead of just suddenly flipping by 180 degrees we do it
	# gradually over a number of steps.
	var turn_steps = full_loop_steps/3;
				
	# The roll direction is a bit complicated because it is actually heading dir
	# and so it switches depending on whether pitch is positive or negative
	var rollDirEnterMult = dir;
	if (rolldirenter == "ccw") rollDirEnterMult = -dir;
	rollDirExitMult = -dir;
	if (rolldirexit == "ccw") rollDirExitMult = dir;
				
	if (loop_count >= round(full_loop_steps/4) - turn_steps/2 and loop_count < round(full_loop_steps/4) + turn_steps/2 )
	{
		var curr_heading_deg = getprop ("" ~ myNodeName ~ "/orientation/true-heading-deg");
		setprop ("" ~ myNodeName ~ "/orientation/true-heading-deg", curr_heading_deg + rollDirEnterMult * 180/turn_steps);
	}
				
	if (loop_count >= round(3 * full_loop_steps/4) - turn_steps/2 and loop_count < round(3 * full_loop_steps/4) + turn_steps/2 )
	{
		var curr_heading_deg = getprop ("" ~ myNodeName ~ "/orientation/true-heading-deg");
		setprop ("" ~ myNodeName ~ "/orientation/true-heading-deg", curr_heading_deg + rollDirExitMult * 180/turn_steps);
	}
}

##############################
# FUNCTION do_acrobatic_loop
#

var do_acrobatic_loop = func 
(
	myNodeName, loop_time = 20, full_loop_steps = 100, exit_steps = 100,  direction = "up", rolldirenter = "cc", 
	rolldirexit = "ccw", vert_speed_add_kt = nil 
)
{
	debprint 
	(
		sprintf
		(
			"Bombable: Starting acrobatic loop for %s loop_time %5.1f full_loop_steps %3.0f exit_steps %3.0f direction %s", 
			myNodeName,
			loop_time,
			full_loop_steps,
			exit_steps,
			direction,
			vert_speed_add_kt 
		)
	);
	attributes[myNodeName].controls.dodgeInProgress = 1;
	settimer
	( 
		func 
		{
			attributes[myNodeName].controls.dodgeInProgress = 0;
		}, 
		loop_time
	);

	# loopid same as other roll type maneuvers because only one can happen at a time
	var loopid = attributes[myNodeName].loopids.roll_loopid + 1;
	attributes[myNodeName].loopids.roll_loopid = loopid;
				
	if (vert_speed_add_kt == nil or vert_speed_add_kt <= 0)
	{
		# this basically means, convert all of the AC's current forward velocity
		# into vertical velocity.  100% of the airspeed seems too much so we're
		# trying 70%
		vert_speed_add_kt = .70 * getprop (""~myNodeName~"/velocities/true-airspeed-kt");
	}

	attributes[myNodeName].velocities.curr_acrobat_vertical_speed_fps = 0;

	#experimental - trying starting all acro maneuvers with 0 vert speed
	#setprop ("" ~ myNodeName ~ "/velocities/vertical-speed-fps", 0);

	#target-alt will affect the vert speed unless we keep it close to current alt
	var currAlt_ft = getprop(""~myNodeName~"/position/altitude-ft");
	setprop (""~myNodeName~"/controls/flight/target-alt", currAlt_ft );
	do_acrobatic_loop_loop(loopid, myNodeName, loop_time, full_loop_steps, exit_steps, direction, rolldirenter, rolldirexit, vert_speed_add_kt,  0 );
}


##################################################################
# Choose an acrobatic loop more or less randomly
#
var choose_random_acrobatic = func (myNodeName) {

	#get the object's initial altitude
	var lat = getprop(""~myNodeName~"/position/latitude-deg");
	var lon = getprop(""~myNodeName~"/position/longitude-deg");
	var elev_m = elev (lat, lon) * FT2M;
	var alt_m = getprop ("/position/altitude-ft") * FT2M;
	var altAGL_m = alt_m-elev_m;
	var alts = attributes[myNodeName].altitudes;
				
	var direction = "up";
	if (altAGL_m > alts.minimumAGL_m + 100 and rand() > .5) direction = "down";
				
	var rolldirenter = "cc";
	var rolldirexit = "cc";
	if (rand() > .5) rolldirenter = "ccw";
	if (rand() > .5) rolldirexit = "ccw";

	var skill = calcPilotSkill (myNodeName);
	var time = 12 + (7-skill) * 2.5 + rand() * 20;
	vels = attributes[myNodeName].velocities;
	var currSpeed_kt = getprop (""~myNodeName~"/velocities/true-airspeed-kt");
	var maxTime = (currSpeed_kt - vels.minSpeed_kt * 2.2) / vels.minSpeed_kt / 2.2 * 25 + 12;
				
	if (time > maxTime) time = maxTime;
				
	#loops of various sizes & between 1/4 & 100% complete
	do_acrobatic_loop ( myNodeName, time, 100, 25 + (1 - rand() * rand()) * 75, direction, rolldirenter, rolldirexit );
				
}

################################ choose_attack_acrobatic ##################################
# Function choose_attack_acrobatic
#
# Choose an acrobatic loop strategically at the beginning of an attack
# returns 1 if a loop was executed, 0 otherwise
#

var choose_attack_acrobatic = func 
(
	myNodeName, dist, myHeading_deg,
	targetHeading_deg, deltaHeading_deg, currSpeed_kt,
	skill, currAlt_m, targetAlt_m, elevTarget_m
)

{
	var ret = 1;
				
	var alts = attributes[myNodeName].altitudes;
	var skill = calcPilotSkill (myNodeName);
	var time = 12  + (7-skill) * 2.5 + 30 * math.abs(currAlt_m-targetAlt_m)/10000;
	if (time > 45) time = 45;
				
	# at 125 kts we can do a 20 second loop; at 250 a 60 second loop, maximum.
	# TODO: Should be airplane specific or dependent on the AC's characteristics
	# Formula below based on minSpeed_kt is the first try.
	var vels = attributes[myNodeName].velocities;
	var maxTime = (currSpeed_kt - vels.minSpeed_kt * 2.2) / vels.minSpeed_kt / 2.2 * 25 + 12;
				
	if (time > maxTime) time = maxTime;

	var currElev_m = elevGround (myNodeName);
	#rjw mod: if the arguments for elev are lat and long of current aircraft position why not read property tree directly?
	#geo.aircraft_position(); Returns the main aircraft's current position in the form of a geo.Coord object
	#currElev_m is the elevation of the ground beneath the AI aircraft - try
	#var currElev_m = elev (getprop(""~myNodeName~"/position/latitude-deg"),getprop(""~myNodeName~"/position/longitude-deg")) * FT2M;
	#or create a new geo object for the AI aircraft using coord = geo.Coord.new(geo.aircraft_position(myNodeName))
 			
	#loops only help if the target is behind us
	if ( math.abs(deltaHeading_deg) >= 100) 
	{
		# if we're going same direction as target and it is close behind
		# us we try a 3/4 loop to try to slip in right behind it
		if ( math.abs(normdeg180(myHeading_deg - targetHeading_deg)) < 90 and
		dist < currSpeed_kt * time / 3600 * nmiles2meters ) var steps = 85;
					
		#otherwise it is far behind us or headed in the opposite direction, we
		# just do an immelmann loop to get turned around in its direction
		else var steps = 48;
					
		# if target is above us or not enough room below for a loop,
		# or going too fast to do a downwards loop, we'll
		# loop upwards, otherwise downwards
		if ( currAlt_m-targetAlt_m < 0 or currAlt_m - currElev_m < alts.minimumAGL_m + 200 or
		currSpeed_kt > .75 * vels.cruiseSpeed_kt ) var direction = "up";
		else var direction = "down";
					
		# TODO: there is undoubtedly a best direction to choose for these,
		# which would leave the AI AC aimed more directly at the Main AC,
		# depending on the relative positions of Main & AI ACs

		var rolldirenter = "cc";
		var rolldirexit = "cc";
		if (rand() > .5) rolldirenter = "ccw";
		if (rand() > .5) rolldirexit = "ccw";
					
		debprint 
		(
			sprintf
			(
				"Bombable: Attack acrobatic loop %s for %s of %2.0f/100 steps, %s roll to enter, %s roll to exit",
				steps, 
				myNodeName, 
				direction,
				rolldirenter,
				rolldirexit
			)
		);
		do_acrobatic_loop (myNodeName, time, 100, steps, direction, rolldirenter , rolldirexit);

		attributes[myNodeName].controls.dodgeInProgress = 1;
		settimer 
		( 
			func
			{
				attributes[myNodeName].controls.dodgeInProgress = 0;
			}, 
			time
		);
					
		# the target is in front of us, so a loop really isn't going to help
		# we'll let the initial attack routine do its thing
	} 
	else
	{
		ret = 0;
	}
	return ret;
}

############################# rudder_roll_climb ############################
#rudder_roll_climb - sets the rudder position/roll degrees
#roll degrees controls aircraft & rudder position
#and for aircraft, sets an amount of climb
#controls ships, so we just change both, to be safe
#alt_ft is a change to the current altitude
#returns time which is determined by the type of turn 

var rudder_roll_climb = func (myNodeName, degrees = 15, alt_ft = -20, time = 10, roll_limit_deg = 85 ) {
	var type = attributes[myNodeName].type;
	var newTime = time;

	if (type == "aircraft")
	{
		var alts = attributes[myNodeName].altitudes;
		var currRoll = getprop(""~myNodeName~"/controls/flight/target-roll");
		if (currRoll == nil) currRoll = 0;
		if ( math.abs(degrees) > 0.1 ) aircraftRoll (myNodeName, degrees, time, roll_limit_deg);
		setprop(""~myNodeName~"/controls/flight/target-roll", currRoll + degrees);

		# altitude
		# This only works for aircraft but that's OK because it's not sensible
		# for a ground vehicle or ship to dive or climb above or below ground/sea
		# level anyway (submarines excepted . . . but under current the FG AI system
		# it would have to be operated as an "aircraft", not a "ship", if it
		# wants to be able to climb & dive).

		var currAlt_ft = getprop(""~myNodeName~"/position/altitude-ft"); #where the object is, in ft
		
		var newAlt_ft = currAlt_ft + alt_ft;			
		if (newAlt_ft < alts.minimumAGL_ft ) newAlt_ft = alts.minimumAGL_ft 
		elsif (newAlt_ft > alts.maximumAGL_ft ) newAlt_ft = alts.maximumAGL_ft;
		#
		# we set the target altitude, unless we are stalling and trying to move
		# higher, then we basically stop moving up
		# confusion about whether alt_ft is absolute or relative
		var stalling = attributes[myNodeName].controls.stalling;
		if (!stalling or newAlt_ft < currAlt_ft) 
		{
			setprop (""~myNodeName~"/controls/flight/target-alt", newAlt_ft);
			aircraftSetVertSpeed (myNodeName, newAlt_ft, "atts" );
		} 
		else 
		{
			#case: stalling
			newAlt_ft = currAlt_ft - rand() * 20 ;
			setprop (""~myNodeName~"/controls/flight/target-alt", newAlt_ft );
			aircraftSetVertSpeed (myNodeName, newAlt_ft, "atts" );
		}
	}
	else # ship or groundvehicle
	# spd < 5 uses fixed turn radius, see AIship parms
	# spd > 5 achieves max turn rate at 15 kts
	# unfortunate: the turn radius is a strong function of speed ( v - 15 )^2
	# ctrls.dodgeInProgress is a flag set by dodge()
	{
		if ( attributes[myNodeName].controls.dodgeInProgress )
		{
			setprop(""~myNodeName~"/surface-positions/rudder-pos-deg", degrees);
			var spd = getprop (""~myNodeName~"/velocities/speed-kts");

			var newSpd = (spd < 10) ? 4.8 : 15;
			var newTime *= ((spd < 10) ? 1 : 4);
			setprop(""~myNodeName~"/controls/tgt-speed-kts", newSpd);
			setprop(""~myNodeName~"/velocities/speed-kts", (spd + newSpd) / 2);
		}
		else # stop dodge, return to cruise speed
		{
			setprop(""~myNodeName~"/surface-positions/rudder-pos-deg", 0 );
			setprop(""~myNodeName~"/controls/tgt-speed-kts", attributes[myNodeName].velocities.cruiseSpeed_kt );
		}
	}
	debprint 
	(
		sprintf
			(
				"Bombable: rudder_roll_climb for %s deg:%6.1f time:%5.1f alt_ft:%6.1f",
				myNodeName,
				degrees,
				newTime,
				alt_ft
			)
	);
	return(newTime);
}
############################### dodge #################################
# function makes an object dodge
#
var dodge = func(myNodeName) 
{
	# dodgeDelay is the time to wait between dodges
	# dodgeDelay_remainder_sec is the amount of that time left
	# rollTime_sec is the time the AC rolls
	var ats = attributes[myNodeName];
	var ctrls = ats.controls;	
	if ( ! bombableMenu["ai-aircraft-attack-enabled"]
	or (ats.damage == 1)
	or ! bombableMenu["bombable-enabled"] )
	return;
	# rjw: unsure where to find attack-enabled on bombable menu. However it is set for B-17 scenario
				
	if ( ctrls.dodgeInProgress ) 
	{
		#debprint ("Bombable: Dodge temporarily locked for this object. ", myNodeName );
		return;
	}
	# Don't change rudder/roll again until the delay
	ctrls.dodgeInProgress = 1;
				
	var type = ats.type;
	var vels = ats.velocities;
	var dims = ats.dimensions;
	var evas = ats.evasions;

	debprint ("Bombable: Starting Dodge", myNodeName, " type = ", type);
				


	# skill ranges 0-6
	var skill = calcPilotSkill (myNodeName);
	var skillMult = (skill <= .2) ? 15 : 3/skill;
				
	# amount to dodge left-right, up to dodgeMax_deg in either direction
	# (1-rand() * rand()) favors rolls towards the high end of the allowed range
	var dodgeAmount_deg = (evas.dodgeMax_deg - evas.dodgeMin_deg) * (1 - rand() * rand()) + evas.dodgeMin_deg;
	# cut the amount of dodging down some for less skilled pilots
	dodgeAmount_deg  *=  (skill+6)/12;
				
	# If we're rolling hard one way then 'dodge' means roll the opposite way.
	# Otherwise we set the roll direction randomly according to the preferences
	# file
	var currRoll_deg = getprop(""~myNodeName~"/orientation/roll-deg");
	if (math.abs(currRoll_deg) > 30) dodgeAmount_deg = -math.sgn(currRoll_deg) * dodgeAmount_deg;
	else if (rand() > evas.dodgeROverLPreference_percent/100) dodgeAmount_deg = -dodgeAmount_deg;

	
	var dodgeDelay = (evas.dodgeDelayMax_sec - evas.dodgeDelayMin_sec) * rand() + evas.dodgeDelayMin_sec;
				
	var dodgeAltAmount_ft = 0;
				
	if (type == "aircraft") 
	{
		if (evas.dodgeAltMax_ft !=0) 
		{
			var dodgeAltDirection = (evas.dodgeAltMax_ft - evas.dodgeAltMin_ft) * rand() + evas.dodgeAltMin_ft;
			
			# we want to mostly dodge to upper/lower extremes of our altitude limits
			var dodgeAltFact = 1 - rand() * rand() * rand();
			# less skilled pilots don't dodge as far
			dodgeAltFact *=  (skill+3)/9;
			# the direction of the Alt dodge will favor the direction that has more
			# feet to dodge in the evasions definitions.  Some aircraft heavily favor
			# diving to escape, for instance.
						
			#target amount to climb or drop
			if (dodgeAltDirection >= 0)
			dodgeAltAmount_ft = dodgeAltFact * evas.dodgeAltMax_ft;
			else
			dodgeAltAmount_ft = dodgeAltFact * evas.dodgeAltMin_ft;
		} 
					
		var rollTime_sec = math.abs(dodgeAmount_deg / evas.rollRateMax_degpersec);
		var dodgeDelay_remainder_sec = dodgeDelay - rollTime_sec;
		if (dodgeDelay_remainder_sec < 0) dodgeDelay_remainder_sec = .1;

		var currSpeed_kt = getprop (""~myNodeName~"/velocities/true-airspeed-kt");
		if (currSpeed_kt == nil) currSpeed_kt = 0;
					
		# more skilled pilots do acrobatics more often
		# in the Zero 130 kt is about the minimum speed needed to
		# complete a loop without stalling.  TODO: This may vary by AC.
		# This could be linked to stall speed and maybe some other things.
		# As a first try we're going with 2X minSpeed_kt as the lowest
		# loop speed, and also 75% of cruise speed as a minimum.
		# We're putting a max width & length for acrobatics as large
		# bomber type a/c don't usually do acrobatics & loops.
		# TODO: This really all needs to be specified per a/c on the bombableinclude file.
		
		# rjw: check whether to start acrobatics
		if (currSpeed_kt > 2 * vels.minSpeed_kt and
		currSpeed_kt > .75 * vels.cruiseSpeed_kt
		and rand() < skill/7 and skill >= 3
		and dims.length_m < 22 and dims.width_m < 18 ) 
		{
			choose_random_acrobatic(myNodeName);
			return;
		}
					
		#set rudder or roll degrees to that amount
		rudder_roll_climb (myNodeName, dodgeAmount_deg, dodgeAltAmount_ft, rollTime_sec);

		#rjw next block not used
			
			dodgeVertSpeed_fps = 0;
						
			if ( dodgeAltAmount_ft > 0 )  dodgeVertSpeed_fps = math.abs ( evas.dodgeVertSpeedClimb_fps * dodgeAltAmount_ft / evas.dodgeAltMax_ft);
			if ( dodgeAltAmount_ft < 0 )  dodgeVertSpeed_fps = - math.abs ( evas.dodgeVertSpeedDive_fps * dodgeAltAmount_ft / evas.dodgeAltMin_ft );
						
			#velocities/vertical-speed-fps seems to be fps * 1000 for some reason?  At least, approximately, 300,000 seems to be about 300 fps climb, for instance.
			# and we reduce the amount of climb/dive possible depending on the current roll angle (can't climb/dive rapidly if rolled to 90 degrees . . . )
			#dodgeVertSpeed_fps *= 1000 * math.abs(math.cos(currRoll_deg* D2R));
			#dodgeVertSpeed_fps *=  math.abs(math.cos(currRoll_deg* D2R));
						
			#vert-speed prob
			#just putting a large number directly into vertical-speed-fps makes the aircraft
			#jump up or down far too abruptly for realism
			#if (dodgeVertSpeed_fps != 0) setprop ("" ~ myNodeName ~ "/velocities/vertical-speed-fps", dodgeVertSpeed_fps);
		
		#end unused


					
		# Roll/climb for rollTime_sec seconds, then wait dodgeDelay - rollTime seconds 
		# (to allow the aircraft's turn to develop from the roll).

		settimer ( func { aircraftRoll (myNodeName, dodgeAmount_deg, dodgeDelay_remainder_sec, evas.dodgeMax_deg); }, rollTime_sec);
		
		# After this delay FG's aircraft AI will automatically return it to near-level flight.
		# Return to near-level flight after a delay of 3-5x the duration of the roll. 

		settimer
			( func 
				{
				ctrls.dodgeInProgress = 0;
				setprop (""~myNodeName~"/controls/flight/target-roll", 0); 
				# This resets the aircraft to 0 deg roll (via FG's
				# AI system target roll; leaves target altitude
				# unchanged  )
				},
				dodgeDelay
			);

		stores.reduceFuel (myNodeName, dodgeDelay ); #deduct the amount of fuel from the tank, for this dodge

		debprint (sprintf("Dodging: %s dodgeAmount_deg = %6.1f dodgeAltAmount_ft = %6.1f dodgeVertSpeed_fps = %6.1f rollTime_sec = %5.1f dodgeDelay_remainder = %6.1f", 
		myNodeName, dodgeAmount_deg, dodgeAltAmount_ft, dodgeVertSpeed_fps ,rollTime_sec, dodgeDelay_remainder_sec));
	} 
	else 
	{  
		# for ships	and groundvehicles		
		# set rudder degrees for a change in direction
		# the dodge starts immediately and stops at turnTime, set according to type of turn

		var turnTime = rudder_roll_climb (myNodeName, dodgeAmount_deg, dodgeAltAmount_ft, dodgeDelay);
		stores.reduceFuel (myNodeName, dodgeDelay );
		settimer 
			( func 
				{
				ctrls.dodgeInProgress = 0;
				rudder_roll_climb (myNodeName, 0, 0, dodgeDelay );
				},
				turnTime 
			);	
	}
			
	# debprint ("Bombable: Dodge alt:", dodgeAltAmount_ft, " degrees:", dodgeAmount_deg, " delay:", dodgeDelay);
}

################################## stopDodgeAttack ################################

var stopDodgeAttack = func (myNodeName) {
	var ctrls = attributes[myNodeName].controls;
	ctrls.dodgeInProgress = 0;
	ctrls.attackInProgress = 0;
	inc_loopid(myNodeName, "roll");
	inc_loopid(myNodeName, "speed_adjust");
	inc_loopid(myNodeName, "attack");
}

##################### getCallSign ##########################
# FUNCTION getCallSign
# returns call sign for AI, MP, or Main AC
# If no callsign, uses one of several defaults
#

var getCallSign = func ( myNodeName ) {
	#Main AC
	if (myNodeName == "") 
	{
		callsign = getprop ("/sim/multiplay/callsign");
		if (callsign == nil) callsign = getprop ("/sim/aircraft");
		if (callsign == nil) callsign = "";
	}
	#AI or MP objects
	else
	{
		var callsign = getprop(""~myNodeName~"/callsign");
		if (callsign == nil or callsign == "") callsign = getprop(""~myNodeName~"/name");
		if (callsign == nil or callsign == "") 
			{
				var node = props.globals.getNode(myNodeName);
				callsign = node.getName() ~ "[" ~ node.getIndex() ~ "]";
			}
	}
	return callsign;
}

################################################################
# function updates damage to the ai or main aircraft when a msg
# is received over MP
# rjw assume no AI in MP mode.  What is the interaction between AI and MP aircraft?

#damageRise is the increase in damage sent by the remote MP aircraft
#damageTotal is the remote MP aircraft's current total of damage
# (This should always be <= our damage total, so it is a failsafe
# in case of some packet loss)
var mp_update_damage = func (myNodeName = "", damageRise = 0, damageTotal = 0, smokeStart = 0, fireStart = 0, callsign = "" ) {

	# rjw not sure whether this func updates damage for MP aircraft
	var damageValue = attributes[myNodeName].damage;
				
	if (damageValue < damageTotal) {
					
		damageValue = damageTotal;
		#note- in sprintf, %d just trims the decimal to make an integer
		# whereas %1.0f rounds to zero decimal places
		msg = sprintf( "Damage for "~string.trim(callsign)~" is %1.0f%%", damageValue * 100);
					
		if (myNodeName == "") mainStatusPopupTip (msg, 30);
		else targetStatusPopupTip (msg, 30);
		debprint ("Bombable: " ~ msg ~ " (" ~ myNodeName ~ ")" );
					
	}
				
	#make sure it's in range 0-1.0
	if(damageValue > 1.0)
	damageValue = 1.0;
	elsif(damageValue < 0.0)
	damageValue = 0.0;
				
	attributes[myNodeName].damage = damageValue;
				
	if (smokeStart) startSmoke ("damagedengine", myNodeName);
	else deleteSmoke("damagedengine", myNodeName);
				
	if (fireStart) startFire (myNodeName);
	else deleteFire(myNodeName);
				
				
				
	if (damageValue >= 1 and damageRise > 0 ) {
		#make the explosion
		var smokeStartsize = rand() * 10 + 5;
		settimer (func {setprop ("/bombable/fire-particles/smoke-startsize", smokeStartsize); }, 2.5);#turn the big explosion off sorta quickly
					
		var explosiveMass_kg = attributes[myNodeName].vulnerabilities.explosiveMass_kg;
					
		if (explosiveMass_kg == 0) explosiveMass_kg = 10000;
		smokeMultiplier = math.log10(explosiveMass_kg) * 10;
		setprop ("/bombable/fire-particles/smoke-startsize", smokeStartsize * smokeMultiplier + smokeMultiplier * rand());


	}
				
}

################################################################
# function sends the main aircraft's current damage, smoke, fire
# settings over MP.  This is is to update all other MP aircraft
# with this aircraft's current damage status.  Other aircraft
# track the damage internally, but if a 3rd aircraft damages this
# aircraft, or if damage is added due to fire, etc., then the
# only way other MP aircraft will know about the damage is via this
# update.
#

var mp_send_main_aircraft_damage_update = func (damageRise = 0 ) {

	if (!getprop(MP_share_pp)) return "";
	if (!getprop (MP_broadcast_exists_pp)) return "";
	if (!bombableMenu["bombable-enabled"] ) return;

				
	damageTotal = attributes[""].damage;
	if (damageTotal == nil) damageTotal = 0;
	smokeStart = getprop("/bombable/fire-particles/damagedengine-burning");
	if (smokeStart == nil) smokeStart = 0;
	fireStart = getprop("/bombable/fire-particles/fire-burning");
	if (fireStart == nil) fireStart = 0;
	#mp_send_damage("", damageRise, damageTotal, smokeStart, fireStart);
				
	callsign = getCallSign ("");

	var msg = damage_msg (callsign, damageRise, damageTotal, smokeStart, fireStart, 3);
	if (msg != nil and msg != "") {
		debprint ("Bombable MADU: MP sending: "~callsign~" "~damageRise~" "~damageTotal~" "~smokeStart~" "~fireStart~" "~msg);
		mpsend(msg);
	}

}

############################### mainAC_add_damage #################################
# function adds damage to the main aircraft when a msg
# is received over MP or for any other reason
#
# Also start smoke/fire if appropriate, and turn off engines/explode
# when damage reaches 100%.

#damageRise is the increase in damage sent by the remote MP aircraft
#damageTotal is the remote MP aircraft's current total of damage
# (This should always be <= our damage total, so it is a failsafe
# in case of some packet loss)
var mainAC_add_damage = func (damageRise = 0, damageTotal = 0, source = "", message = "") {
	if (!bombableMenu["bombable-enabled"] ) return 0;
				
	var damageValue = attributes[""].damage;
				
	prevDamageValue = damageValue;
				
	if(damageValue < 1.0)
	damageValue  +=  damageRise;
				
	if (damageValue < damageTotal) damageValue = damageTotal;
				
	#make sure it's in range 0-1.0
	if(damageValue > 1.0)
	damageValue = 1.0;
	elsif(damageValue < 0.0)
	damageValue = 0.0;
				
	attributes[""].damage = damageValue;
	setprop("/bombable/attributes/damage", damageValue); # mirror	
				
	damageIncrease = damageValue - prevDamageValue;
				
	if (damageIncrease > 0)  {
		addMsg1 = "You've been damaged!";
		addMsg2 = "You are out of commission! Engines/Magnetos off!";
		if (message != "")  {
			addMsg1 = message;
			addMsg2 = message;
		}
		if (damageValue < .01) msg = sprintf( addMsg1 ~ " Damage added %1.2f%% - Total damage %1.0f%%", damageIncrease * 100 , damageValue * 100 );
		elsif (damageValue < .1) msg = sprintf( addMsg1 ~ " Damage added %1.1f%% - Total damage %1.0f%%", damageIncrease * 100 , damageValue * 100);
		elsif (damageValue < 1) msg = sprintf( addMsg1 ~ " Damage added %1.0f%% - Total damage %1.0f%%", damageIncrease * 100, damageValue * 100);
		else msg = sprintf( " ==  ==  ==  == " ~ addMsg2 ~ " Damage added %1.0f%% - Total damage %1.0f%% ==  ==  ==  == ", damageIncrease * 100, damageValue * 100 );
		mainStatusPopupTip (msg, 15);
		debprint ("Bombable: " ~ msg );
					
		if (damageValue == 1) {
			#So that ppl know their engine/magneto has been switched off, so they'll
			#know they need to turn it back on.
			settimer ( func {
				if (getprop("/controls/engines/engine[0]/magnetos") == 0 ) {
					msg = " ==  ==  ==  == Damage 100% - your engines and magnetos have been switched off ==  ==  ==  == ";
					mainStatusPopupTip (msg, 10);
					debprint ("Bombable: " ~ msg );
				}
			} , 15);
		}
	}
				
				
	#Update--we don't allow remote control of main aircraft's
	# fire/smoke any more.  Causes problems.
	#if (smokeStart) startSmoke ("damagedengine", "");
	#else deleteSmoke("damagedengine", "");
				
	#if (fireStart) startFire ("");
	#else deleteFire("");
				
	#start smoke/fires if appropriate
	# really we need some way to customize this for every aircraft
	# just as we do for AI/MP aircraft.  But in the meanwhile this will work:
				
	myNodeName = ""; #main aircraft
				
	var vuls = attributes[myNodeName].vulnerabilities;

				
				

	var fireStarted = getprop("/bombable/fire-particles/fire-burning");
	if (fireStarted == nil ) fireStarted = 0;
	var damageEngineSmokeStarted = getprop("/bombable/fire-particles/damagedengine-burning");
	if (damageEngineSmokeStarted == nil ) damageEngineSmokeStarted = 0;
				
				
				
	if (!damageEngineSmokeStarted and !fireStarted and damageIncrease > 0 and rand() * 100 < vuls.engineDamageVulnerability_percent )
	startSmoke("damagedengine",myNodeName);
				

	# start fire if there is enough damage AND if the damage is caused by the right thing (weapons, crash, but not.
	# if a crash, always start a fire (sometimes we reach 100% damage, no fire,
	# then crash later--so when we crash, always start fire regardless)
	if( (  (
	damageValue >= 1 - vuls.fireVulnerability_percent/100
	and damageIncrease > 0 and !fireStarted
	) and
	(source == "weapons" or source == "crash" )
	) or
	(source == "crash" )
	) {
					
					
		debprint ("Bombable: Starting fire for main aircraft");
					
		#use small, medium, large smoke column depending on vuls.damageVulnerability
		#(high vuls.damageVulnerability means small/light/easily damaged while
		# low vuls.damageVulnerability means a difficult, hardened target that should burn
		# more vigorously once finally on fire)
		var fp = "";
		if (vuls.explosiveMass_kg < 1000 ) { fp = "AI/Aircraft/Fire-Particles/fire-particles-very-small.xml"; }
		elsif (vuls.explosiveMass_kg > 5000 ) { fp = "AI/Aircraft/Fire-Particles/fire-particles-small.xml"; }
		elsif (vuls.explosiveMass_kg > 50000 ) { fp = "AI/Aircraft/Fire-Particles/fire-particles-large.xml"; }
		else {fp = "AI/Aircraft/Fire-Particles/fire-particles.xml";}
					
		startFire(myNodeName, fp);
		#only one damage smoke at a time . . .
		deleteSmoke("damagedengine",myNodeName);
					
					
		#fire can be extinguished up to MaxTime_seconds in the future,
		#if it is extinguished we set up the damagedengine smoke so
		#the smoke doesn't entirely go away, but no more damage added
		if ( rand() * 100 < vuls.fireExtinguishSuccess_percentage ) {
						
			settimer (func {
				deleteFire (myNodeName);
				startSmoke("damagedengine",myNodeName);
			} ,
			rand() * vuls.fireExtinguishMaxTime_seconds + 15 ) ;
		};
	}
				
	#turn off engines if appropriate
	if (damageValue >= 1 and (prevDamageValue < 1 or getprop("/controls/engines/engine[0]/magnetos") > 0 or getprop("/controls/engines/engine[0]/throttle") > 0 )) {
					
		#turn off all engines
		setprop("/controls/engines/engine[0]/magnetos",0);
		setprop("/controls/engines/engine[0]/throttle",0);
		setprop("/controls/engines/engine[1]/magnetos",0);
		setprop("/controls/engines/engine[1]/throttle",0);
		setprop("/controls/engines/engine[2]/magnetos",0);
		setprop("/controls/engines/engine[2]/throttle",0);
		setprop("/controls/engines/engine[3]/magnetos",0);
		setprop("/controls/engines/engine[3]/throttle",0);
		debprint ("Main aircraft damage 100%, engines off, magnetos off");
					
		#if no smoke/fire yet, now is the time to start
		startSmoke ("damagedengine", "");
		if (source == "weapons" or source == "crash" ) startFire ("");
					
		var smokeStartsize = rand() * 10 + 5;
		settimer (func {setprop ("/bombable/fire-particles/smoke-startsize", smokeStartsize); }, 2.5);#turn the big explosion off sorta quickly
					
		smokeMultiplier = math.log10(vuls.explosiveMass_kg) * 10;
		setprop ("/bombable/fire-particles/smoke-startsize", smokeStartsize * smokeMultiplier + smokeMultiplier * rand());

	}
				
				
	mp_send_main_aircraft_damage_update (damageRise);
				
	return damageIncrease;
				
				
}


#send the damage message via multiplayer
var mp_send_damage = func (myNodeName = "", damageRise = 0 ) {
				
	if (!getprop(MP_share_pp)) return "";
	if (!getprop (MP_broadcast_exists_pp)) return "";
	if (!bombableMenu["bombable-enabled"] ) return;
				
				
	#messageType 1 is letting another MP aircraft know you have damaged it
	#messageType 3 is informing all other MP aircraft know about the main
	# aircraft's current damage, smoke, fire settings
	#
	messageType = 1;
	if (myNodeName == "") messageType = 3;
	var damageValue = attributes[myNodeName].damage;
				
	#This next statement appears to be dead/useless code because the callsign is picked up from the getCallSign function below?
	if (myNodeName == "")
	{
		callsign = getprop ("/sim/multiplay/callsign");
		}else {
		callsign = getprop (""~myNodeName~"/callsign");
	}
				
	var callsign = getCallSign (myNodeName);
				
	var fireStart = getprop(""~myNodeName~"/bombable/fire-particles/fire-burning");
	if (fireStart == nil ) fireStart = 0;
	var smokeStart = getprop(""~myNodeName~"/bombable/fire-particles/damagedengine-burning");
	if (smokeStart == nil ) smokeStart = 0;
	# debprint ("Bombable MSD: Preparing to send MP damage update to "~callsign);
	var msg = damage_msg (callsign, damageRise, damageValue, smokeStart, fireStart, messageType);
				
	if (msg != nil and msg != "") {
		debprint ("Bombable MSD: MP sending: "~callsign~" "~damageRise~" "~damageValue~" "~smokeStart~" "~fireStart~" "~messageType~" "~msg);
		mpsend(msg);
	}
				
}

###################### fireAIWeapon_stop ######################
# fireAIWeapon_stop: turns off one of the triggers in AI/Aircraft/Fire-Particles/projectile-tracer.xml
# rjw 
var fireAIWeapon_stop = func (id, myNodeName, index) {
	# index of the fire particle tied to the weapon that will stop firing
	id == attributes[myNodeName].loopids["fireAIWeapon" ~ index ~ "_loopid"] or return;
	setprop("bombable/fire-particles/projectile-tracer[" ~ index ~ "]/ai-weapon-firing", 0); 
}

###################### fireAIWeapon ######################
# fireAIWeapon: turns on/off one of the triggers in AI/Aircraft/Fire-Particles/projectile-tracer.xml
# Using the loopids ensures that it stays on for one full second after the last time it was
# turned on.
#
var fireAIWeapon = func (time_sec, myNodeName, elem, speed) {
	var index = elem.fireParticle;
	# index of the fire particle tied to the weapon
	# rjw speed is the calculated intercept speed in a stationary frame
	#if (myNodeName == "" or myNodeName == "environment") myNodeName = "/environment";
	var isFiring = getprop("bombable/fire-particles/projectile-tracer[" ~ index ~ "]/ai-weapon-firing");
	if (isFiring != nil) {
		if (isFiring == 1) return; #prevents double trigger
		}
	
	setprop("bombable/fire-particles/projectile-tracer[" ~ index ~ "]/speed", speed);
	setprop("bombable/fire-particles/projectile-tracer[" ~ index ~ "]/ai-weapon-firing", 1); 
	var loopid = inc_loopid(myNodeName, "fireAIWeapon" ~ index);
	# debprint (	"Bombable: myNodeName " ~ myNodeName ~
	# 			" index " ~ index,
	# 			" time " ~ time_sec);
	settimer ( func { fireAIWeapon_stop(loopid, myNodeName, index)}, time_sec);
}

###################### vertAngle_deg #########################
# calculates angle (vertical, degrees above or below directly
# horizontal) between two geocoords, in degrees
#
var vertAngle_deg = func (geocoord1, geocoord2) {
	var dist = geocoord2.direct_distance_to(geocoord1);
	if ( dist == 0 ) return 0;
	else return math.asin((geocoord2.alt() - geocoord1.alt())/dist ) * R2D;

}


####################### checkAim ########################
# Checks that the two objects are within the given distance
# and if so, checks whether myNodeName1 is aimed directly at myNodeName2
# within the heading & vertical angle given, OR that the two objects
# have crashed (ie, the position of one is within the damage radius of the other).
# If all that is true, returns number 0-1 telling how close the hit (1 being
# perfect hit), otherwise 0.
# In case of crash, this crashes both objects.
# works with AI/MP aircraft AND with the main aircraft, which can
# use myNodeNameX = ""
# Note: This whole approach won't work too well because the position &
# orientation properties are updated only 1X per second.
# But who knows--it might be 'good enough' . . .
#
# Notes on calculating the angular size of the target and being
# able to tell if the weapon is aimed at the target with sufficient accuracy.
# 3 degrees @ 200 meters, 2% damage add maximum.
# This equates to about a 6 meter damage radius
#
# 2.5, 7.5 equates to an object about 10 ft high and 30 feet wide
#     about the dimensions of a Sopwith Camel
#
# formula is tan (angle) = damage radius / distance
# or angle = atan (damage radius/ distance)
# however, for 0 < x < 1, atan(x)  ~=  x (x expressed in radians).
# Definitely this is close enough for our purposes.
#
# Since our angles are expressed in degrees the only difficulty is to convert radians
# to degrees. So the formula is:
# angle (degrees) = damage radius / distance * (180/pi)
#
# Below we'll put in the general height & width of the object, so the equations become:
#
# angle (degrees) = height/2 / distance * (180/pi)
# angle (degrees) = width/2 / distance * (180/pi)
# (approximate because our damage area is a rectangle projected
# onto the surface of a sphere here, not just a radius )
# Good estimate: dimension in feet divided by 4 equals angle to use here
#
# We make the 'hit' area tall & skinny rather than wide & flat
# because our fighters are vertically challenged as far as movement,
# but quite easily able to aim horizontally
#
# Notes:
# vertDeg & horzDeg define the angular size of the target (ie, our main aircraft) at
# the distance of maxDamageDistance_m.  3.43 degrees x 3.43 degrees at 500 meters is approximately right
# for (say) a sopwith camel.  Really this needs to be calculated more exactly based on the
# actual dimensions of the main aircraft. But for now this is at least close.  There doesn't
# seem to be a simple way to get the dimensions of the main aircraft from FG.
# Bombable calculates the hits as on a probabilistic basis if they hit within the given
# angular area.  The closer to the center of the target, the higher the probability of a hit
# and the more the damage.  Since the bombable/ai weapons system is simulated (ie, based on the probability
# of a hit determined by how close the aim is to the general area of the main aircraft, rather than
# actually launching projectiles and seeing if they hit) this is generally a 'good enough' approach.
# myNodeName1 is the AI aircraft and myNodeName2 is the main aircraft

# pRound is the probability of one round hitting the target calculated as the 
# overlap of the solid angle subtended by the target and a cone of 'accuracy' radians subtended by the shooter
# function updates thisWeapon.aim with weapon direction vectors in model and reference frames
# in 3D model co-ords the x-axis points 180 deg from direction of travel i.e. backwards
# the y-axis points at 90 deg to the direction of travel i.e. to the right
# weaponOffset is given in model co-ords; weaponOffsetRefFrame in the ground frame
# weaponAngle is a hash with components heading and elevation
# Function called only by weapons_loop
			

var checkAim = func ( thisWeapon, 
					tgtDisp,
					myNodeName1 = "", myNodeName2 = "",
					weapPowerSkill = 1, 
					damageValue = 0) 
{
	var targetSighted = 0; #flag true if target sighted which must happen before the weapon can be aimed
	var ats = attributes[myNodeName1];
	
	thisWeapon.aim.nHit = 0;
				
	# Weapons malfunction in proportion to the damageValue, to 100% of the time when damage = 100%
	# debprint ("Bombable: AI weapons, ", myNodeName1, ", ", myNodeName2);
	if (rand() < damageValue) return (targetSighted) ;

	# correct targetDispRefFrame for weapon offset
	# find interceptDir the direction required for a missile travelling at a constant speed to intercept the target 
	# given the relative velocities of shooter(1) and target(2)
	# calculate the angle between the direction in which the weapon is aimed and interceptDir
	# use this angle and the solid angle subtended by the target to determine pRound

	if (thisWeapon.parent == "")
	{
		# calculate the offset of the weapon in the ground reference frame
		thisWeapon.aim.weaponOffsetRefFrame = rotate_zxy
		(
			[
			thisWeapon.weaponOffset_m.y,
			-thisWeapon.weaponOffset_m.x,
			thisWeapon.weaponOffset_m.z
			],
			-ats.pitch_deg, -ats.roll_deg, ats.myHeading_deg
		);
	}
	else
	{
		# code could be optimized; no change in z
		# assume parent weapon located on axis of its turret
		var parentWeap = ats.weapons[thisWeapon.parent];
		var phi = getprop( "" ~ myNodeName1 ~ "/" ~ thisWeapon.parent ~ "/turret-pos-deg" );
		var myOffset = rotate_round_z_axis # in the model frame
		(
			[
			thisWeapon.weaponOffset_m.x - parentWeap.weaponOffset_m.x,
			thisWeapon.weaponOffset_m.y - parentWeap.weaponOffset_m.y,
			thisWeapon.weaponOffset_m.z - parentWeap.weaponOffset_m.z
			], # offset of weapon in model frame relative to parent weapon
			-phi
		);

		var index = thisWeapon.fireParticle;
		# index of the fire particle tied to the weapon

		# offset of weapon relative to AI model origin
		myOffset[0] += parentWeap.weaponOffset_m.x;
		myOffset[1] += parentWeap.weaponOffset_m.y;
		myOffset[2] += parentWeap.weaponOffset_m.z;

		setprop("bombable/fire-particles/projectile-tracer[" ~ index ~ "]/offset-x", myOffset[0]);
		setprop("bombable/fire-particles/projectile-tracer[" ~ index ~ "]/offset-y", myOffset[1]);
		setprop("bombable/fire-particles/projectile-tracer[" ~ index ~ "]/offset-z", myOffset[2]);

		thisWeapon.aim.weaponOffsetRefFrame = rotate_zxy
		(
			[
			myOffset[1],
			-myOffset[0],
			myOffset[2]
			],
			-ats.pitch_deg, -ats.roll_deg, ats.myHeading_deg
		);
	}

	# calculate the displacement of the target from the weapon using the displacement between the centres of the target and shooter
	# in effect correct the distance for the offset of the weapon relative to the shooter origin	 
	var targetDispRefFrame = vectorSubtract(tgtDisp, thisWeapon.aim.weaponOffsetRefFrame);
	
	# calculate the distance
	var distance_m = vectorModulus(targetDispRefFrame); 
	
	var intercept = findIntercept(
		myNodeName1, myNodeName2, 
		targetDispRefFrame,
		distance_m,
		thisWeapon.maxMissileSpeed_mps
	);

	if (intercept.time == 9999) return(targetSighted);
	#target has been sighted, now aim weapon towards it
	targetSighted = 1;
	thisWeapon.aim.interceptSpeed = vectorModulus(intercept.vector);

	var interceptDirRefFrame = vectorDivide(intercept.vector, thisWeapon.aim.interceptSpeed);
	
	# debprint (
		# sprintf(
			# "Bombable: intercept time =%8.1f Intercept vector =[%8.2f, %8.2f, %8.2f]",
			# intercept.time, interceptDirRefFrame[0], interceptDirRefFrame[1], interceptDirRefFrame[2] 
		# )
	# );

	#translate intercept direction to the frame of reference of the model
	var newDir = rotate_yxz(interceptDirRefFrame, ats.pitch_deg, ats.roll_deg, -ats.myHeading_deg);
	
		
	#form vector for the current direction of weapon, weapDir, in the reference frame of the model
	var cosWeapElev = math.cos(thisWeapon.weaponAngle_deg.elevation* D2R);
	var weapDir = 
	[
		cosWeapElev * math.sin(thisWeapon.weaponAngle_deg.heading* D2R),
		cosWeapElev * math.cos(thisWeapon.weaponAngle_deg.heading* D2R),
		math.sin(thisWeapon.weaponAngle_deg.elevation* D2R)
	];
	
	#calculate angular offset
	var cosOffset = dotProduct(newDir, weapDir);

	#calculate probability of hitting target, pRound
	if (cosOffset > 0.985)
	{
		# get targetSize. Can simplify only used here
		var targetSize_m = { vert : 4, horz : 8 }; 
		var dims = attributes[myNodeName2].dimensions;
		if (dims["height_m"] != nil) # check probably not needed since default dimensions are provided by setAttributes
		{
			targetSize_m = 
			{
				vert : dims.height_m ,
				horz : 0.5 * (dims.width_m + dims.length_m) ,
			};
		}
		# debprint ("Bombable: Target size ", targetSize_m.vert, " by ", targetSize_m.horz, " for ", myNodeName );

		# only calculate pRound if target direction within 10 degrees of weapon direction
		var targetOffset_rad = math.acos(cosOffset); # angular offset from weapon direction
		var targetSize_rad = math.atan2(math.sqrt(targetSize_m.horz * targetSize_m.vert) / 2 , distance_m);	
		# geometric mean of key dimensions and half angle

		# debprint (sprintf("Bombable: checkAim for %s targetOffset_rad =%8.2f targetSize_rad =%8.2f", 
			# myNodeName1,
			# targetOffset_rad,
			# targetSize_rad));
		# debprint (sprintf("Bombable: newDir[%8.2f,%8.2f,%8.2f] dist=%6.0f", newDir[0], newDir[1], newDir[2], distance_m));
		# debprint (sprintf("Bombable: weapDir[%8.2f,%8.2f,%8.2f]", weapDir[0], weapDir[1], weapDir[2]));
		

		# pRound ranges 0 to 1, 1 is a direct hit			
		# calculate pRound as a joint probability distribution: the angular range of fire of the weapon and the angular range subtended by the target
		# Assume:  pTargetHit = 1 within the angle range the target subtends at the weapon
		# Assume:  angular distribution of bullets from weapon is a normal distribution centred on weapon and of SD 5 degrees i.e. 1/12 radian
		# could approximate normal distribution using central limit https://en.wikipedia.org/wiki/Normal_distribution#Generating_values_from_normal_distribution and use MonteCarlo
		# instead use error function to calculate integral of normal distribution

		# probability x between p and q
		# p(x) = (a/pi)^0.5 * 0.5 * ( erf (q * a^0.5) - erf (p * a^0.5) )  where a = 1 / 2 / SD^2, if SD = 5 deg, sqrt_a  = 8.103

		# probability P of one hit or more over the period of fire is P = 1 - ( 1 - pRound) ^ (LOOP_TIME * rounds per sec)
		# pMiss for one round = 1 - pRound 
		# <nHit> is the average number of rounds that hit during LOOP_TIME

		var sqrt_a = 0.7071 / thisWeapon.accuracy;
		var pRound = erf(( targetOffset_rad + targetSize_rad ) * sqrt_a) -  erf(( targetOffset_rad - targetSize_rad ) * sqrt_a);
		pRound *= (1 - getprop (""~myNodeName1~"/velocities/true-airspeed-kt") / ats.velocities.maxSpeed_kt); # reduce probability if platform moving
		thisWeapon.aim.nHit = pRound * LOOP_TIME * thisWeapon.roundsPerSec; 

		# debprint 
		# (
		# 	sprintf(
		# 	"Bombable: Hit %s nHit = %6.3f offset deg = %6.2f weapPowerSkill = %4.1f",
		# 	myNodeName1 ~ ": " ~ thisWeapon.name,
		# 	thisWeapon.aim.nHit,
		# 	targetOffset_rad * R2D,
		# 	weapPowerSkill)
		# );
	}
	if (thisWeapon.aim.fixed == 1 or thisWeapon.weaponType == 1)
	# no change to weaponDirModelFrame (set in weapons_init_func)
	# but need to calculate direction of weapon in reference frame
	# usually this will be in direction of travel of AI object 
	# exceptions: rockets, ACs with vertically firing cannon
	{
		thisWeapon.aim.weaponDirRefFrame = rotate_zxy(weapDir, -ats.pitch_deg, -ats.roll_deg, ats.myHeading_deg);
	}
	else
	{
		# change orientation of weapon if not fixed
		# a skilled gunner changes the direction of their weapon more frequently 
		# weapons on slaved turrets ('children') must update more frequently since their aim is lost on movement of the parent turret
		if ( rand() < weapPowerSkill * ((thisWeapon.parent != "") ? 1 : .5) or thisWeapon.aim.weaponDirRefFrame[2] == -1)
		{ 
			# ensure that newDir is in range of movement of weapon
			var newElev = math.asin(newDir[2]) * R2D;
			var newHeading = math.atan2(newDir[0], newDir[1]) * R2D;

			if (newElev < thisWeapon.weaponAngle_deg.elevationMin)
				newElev = thisWeapon.weaponAngle_deg.elevationMin;
			elsif (newElev > thisWeapon.weaponAngle_deg.elevationMax)
				newElev = thisWeapon.weaponAngle_deg.elevationMax;

			var headingVal = keepInsideRange(thisWeapon.weaponAngle_deg.headingMin, thisWeapon.weaponAngle_deg.headingMax, newHeading);
			if (!headingVal.insideRange) newHeading = headingVal.newHdg;
			
			
			var cosNewElev = math.cos(newElev* D2R);
			newDir = 
			[
				cosNewElev * math.sin(newHeading* D2R),
				cosNewElev * math.cos(newHeading* D2R),
				math.sin(newElev* D2R)
			];

			thisWeapon.aim.weaponDirModelFrame = newDir;
			thisWeapon.aim.weaponDirRefFrame = rotate_zxy(newDir, -ats.pitch_deg, -ats.roll_deg, ats.myHeading_deg);
		}
	}
	return (targetSighted); 	
}

############################ weapons_loop #############################
# weapons_loop - main timer loop for check AI weapon aim & damage
# to main aircraft
#
# Todo: We could check how often this loop is being called (by all AI objects
# in total) and if it is being called too often, exit.  This can have a
# bad effect on the framerate if the main aircraft gets into a crowd
# of AI objects.
#
# We could implement an approach to finding distance/direction more like the one
# in test_impact, where we just use a local coordinate system of lat/lon/elev
# to calculate target distance. That seems far more frugal of CPU time than
# geoCoord and directdistanceto, which both seem quite expensive of CPU.
			
var weapons_loop = func (id, myNodeName1 = "") {
	var ats = attributes[myNodeName1];
	#we increment loopid if we want to kill this timer loop.  So check if we need to kill/exit:
	#myNodeName1 is the AI aircraft and myNodeName2 is its target
	id == ats.loopids.weapons_loopid or return;
				
	var loopTime = LOOP_TIME ;
	settimer (  func { weapons_loop (id, myNodeName1)}, loopTime);

	if (! size(ats.targetIndex)) return; # no target available

	if (! bombableMenu["ai-aircraft-weapons-enabled"] or ! bombableMenu["bombable-enabled"] ) return;
				
	#if no weapons set up for this Object then just return
	if (! getprop(""~myNodeName1~"/bombable/initializers/weapons-initialized")) return;
				
	#debprint ("aim-check damage");
	#If damage = 100% we're going to assume the weapons won't work.
	var damageValue = ats.damage;
	if (damageValue == 1) return;
	
	# weaponPower varies between 0 and 1
	var weaponPower = bombableMenu["ai-weapon-power"];
	if (weaponPower == nil) weaponPower = 0.2;
				
	# weapPowerSkill varies 0-1, average varies with power-skill combo
	var weapPowerSkill = ats.controls.weapons_pilot_ability;
				
	# use of AI power and skill
	# weaponPower determines the probability of damage if there is a hit
	# weapPowerSkill determines how frequently weapon aim is updated
	# currently both are attributes of the AC, ship or vehicle, not the individual weapon

	# info about shooter used by checkAim
	var alat_deg = getprop(""~myNodeName1~"/position/latitude-deg"); # shooter
	var alon_deg = getprop(""~myNodeName1~"/position/longitude-deg");
	var aAlt_m = getprop(""~myNodeName1~"/position/altitude-ft") * FT2M;
	
	# the heading, pitch and roll of the shooter are used to determine the orientation of its weapon in the ground frame of reference 
	# AI aircraft animate model using pitch and roll; for AI ships and ground_vehicles code animation here
	# roll increases clockwise in the direction of travel
	ats.myHeading_deg = getprop ("" ~ myNodeName1 ~ "/orientation/true-heading-deg");
	ats.roll_deg = getprop("" ~ myNodeName1 ~ "/orientation/roll-animation");
	ats.pitch_deg = getprop("" ~ myNodeName1 ~ "/orientation/pitch-animation");
	if (ats.roll_deg == nil)
	{
		ats.roll_deg = getprop("" ~ myNodeName1 ~ "/orientation/roll-deg");
		ats.pitch_deg = getprop("" ~ myNodeName1 ~ "/orientation/pitch-deg");
	}

	var myTargets = ats.targetIndex;
	var nTargets = size(myTargets);
	var targetData = [];
	var groundData = [];
	var noLoS = 1; # flag for no target in line of sight
	var rocketCarriers = []; # indices of targets carrying rockets
	var distClosest = 999;
	var threatLevel = 1; # 1 + number of targets within detection range
	var detectionRange = 7000;
	var indexClosest = nil;
	ats.attacks.allGround = 1;

	foreach (target; myTargets)
	{
		var myNodeName2 = nodes[target];
		# info about target used by checkAim
		var targetLat_deg = getprop(""~myNodeName2~"/position/latitude-deg"); # target
		var targetLon_deg = getprop(""~myNodeName2~"/position/longitude-deg");
		var targetAlt_m = getprop(""~myNodeName2~"/position/altitude-ft") * FT2M;

		# m_per_deg_lat/lon are bombable general variables
		var deltaX_m = (targetLon_deg - alon_deg) * m_per_deg_lon;
		var deltaY_m = (targetLat_deg - alat_deg) * m_per_deg_lat;
		var deltaAlt_m = targetAlt_m - aAlt_m;
					
		# calculate targetDispRefFrame, the displacement vector from node1 (shooter) to node2 (target) in a lon-lat-alt (x-y-z) frame of reference aka 'reference frame'
		# the shooter is at < 0,0,0 > 
		var targetDispRefFrame = [deltaX_m, deltaY_m, deltaAlt_m];
		var distance_m = math.sqrt(deltaX_m * deltaX_m + deltaY_m * deltaY_m + deltaAlt_m * deltaAlt_m);
		if (distClosest < distance_m) 
		{
			distClosest = distance_m;
			indexClosest = size (targetData);
		}
		if ( distance_m < detectionRange) 
		{
			threatLevel += 1;
			if (target) #ignore main AC
			{
				if (attributes[myNodeName2].type == "aircraft") ats.attacks.allGround = 0;
			}
		}

		append (targetData, [deltaX_m, deltaY_m, deltaAlt_m, distance_m]);

		# check for collision, 
		# ie target within damageRadius of shooter
		if (distance_m < ats.dimensions.crashRadius_m)
		{
			var msg = (attributes[myNodeName1].controls.kamikase == -1) ?
			"Kamikase strike " : "Collision ";
			msg = msg ~ " with " ~ getCallSign (myNodeName1) ~ " !";
			targetStatusPopupTip (msg, 5); # add_damage will immediately report damage stats
			var damageRatio = attributes[myNodeName2].vulnerabilities.explosiveMass_kg / 
			attributes[myNodeName1].vulnerabilities.explosiveMass_kg; 
			add_damage(10 * damageRatio, "collision", myNodeName1); # can withstand collision with object <10% of my mass
			if (myNodeName2 !="")
			{
				add_damage ( 10 / damageRatio, "collision", myNodeName2);
			}
			else
			{
				mainAC_add_damage ( 10 / damageRatio, 0, "collision", msg);
			}
			return;
		}

		var groundCheck = 1;
		if (ats.type == "groundvehicle")
		{
			# check line of sight by calculating the height above ground of the bullet trajectory at the mid point between shooter and target
			var mid_lat_deg = (alat_deg + targetLat_deg) / 2;
			var mid_lon_deg = (alon_deg + targetLon_deg) / 2;
			var mid_Alt_m = (aAlt_m + targetAlt_m) / 2;
			var GeoCoord = geo.Coord.new();
			GeoCoord.set_latlon(mid_lat_deg, mid_lon_deg);
			var ground_Alt_m = elev (GeoCoord.lat(), GeoCoord.lon()) * FT2M; 
			groundCheck = (ground_Alt_m < mid_Alt_m);
		}
		append ( groundData,  groundCheck );
		if (groundCheck) noLoS = 0;

		if (target ? (attributes[myNodeName2].nRockets > 0) : 0) append(rocketCarriers, target); # separate check for main AC

	}
	if (noLoS) return; #if no target in line of sight then no need to check aim.  Assumption! Does not hold for self-guided rockets or parabolic flight trajectory

			
	foreach (elem; keys (ats.weapons) ) 
	{	
		var thisWeapon = ats.weapons[elem];
		if (thisWeapon.destroyed == 1) continue; #skip this weapon if destroyed
		if ( stores.checkWeaponsReadiness ( myNodeName1, elem ) == 0) continue; # can only shoot if ammo left!
		var ind = thisWeapon.aim.target; # index of object to shoot at
		var pos = vecindex(myTargets, ind); # pos is the index in targetData and groundData; nil means target no longer exists

		if (thisWeapon.weaponType == 1 and pos == nil)
		# for a rocket with no target assigned, check first those targets still carrying functioning rockets
		# find the node of that rocket
		# ignore any rocket that is targeting me
		# if none available select from other targets
		# 

		{
			var rockets = [];
			var count = 0;
			debprint("Number of rocket carriers ", size(rocketCarriers));
			foreach (i; rocketCarriers)
			{
				var wps = attributes[nodes[i]].weapons; 
				foreach (targetWeap; keys(wps))
				{
					if (wps[targetWeap].destroyed) continue;
					if (wps[targetWeap].weaponType == 1 and wps[targetWeap].aim["rn"] != elem) # do not include rockets that are targeting me
					{
						append(rockets, i, targetWeap);
						count += 1;
					}
				}
			}
			debprint("Searching for rocket targets for " ~ myNodeName1 ~ " " ~ thisWeapon.name ~ ", Count= " ~ count);
			if (count)
			{
				var r = int(rand() * count) * 2;
				thisWeapon.aim.target = rockets[r];
				thisWeapon.aim.rn = rockets[r + 1];
				debprint(sprintf(
					"Selected rocket target for %s : %s >> %s : %s",
					getCallSign(myNodeName1),
					wps[rockets[r + 1]].name,
					getCallSign(nodes[rockets[r]]),
					elem
					)) ;
				continue;
			}
			else
			{
				thisWeapon.aim["rn"] = nil;
			}
		} 

		if (thisWeapon.aim["rn"] == nil) # the target is not a rocket
		{
			# if no target assigned - applies when weapons loop first called and when previous target destroyed
			if (pos == nil)
			{
				pos = int(rand() * nTargets);
				if (myTargets[pos] != 0 or nTargets == 1) thisWeapon.aim.target = myTargets[pos]; # only attack main AC when there are no AI targets
				continue;
			}

			# if no line of sight - only applies to groundvehicles - select one of the targets that passes ground check
			if (!groundData[pos])
			{
				pos = vecindex(groundData, 1); # find first target with line of sight
				thisWeapon.aim.target = myTargets[pos];
				continue;
			}

			# if target out of range
			if (targetData[pos][3] > thisWeapon.maxDamageDistance_m)
			{

				if (rand() < 0.2)
				{
					pos += 1;
					if (pos == nTargets) pos = indexClosest;
					if (myTargets[pos] != 0) thisWeapon.aim.target = myTargets[pos];
				}
				continue;
			}
		}
		var tgtDisp = [targetData[pos][0], targetData[pos][1], targetData[pos][2]];
		var myNodeName2 = nodes[ind];

		if (thisWeapon.weaponType == 0) 
		{
			var targetSighted = checkAim
				(
					thisWeapon, # pass pointer to weapon parameters
					tgtDisp,
					myNodeName1, myNodeName2, 
					weapPowerSkill,
					damageValue
				);
			if ( targetSighted ) weaponsOrientationPositionUpdate(myNodeName1, elem);
		}
	
		if (thisWeapon.weaponType == 1) 
		{
			if (thisWeapon.controls.launched == 1) continue;

			# for a rocket checkaim is only used to update co-ordinates of the launch platform

			var targetSighted = checkAim
				(
					thisWeapon,
					tgtDisp,
					myNodeName1, myNodeName2, 
					weapPowerSkill,
					damageValue
				);

			if ( targetSighted ) 
			{
				# launch rocket when high threat level and few rockets in the air (ship)
				# when escaping or attacking and target close by (aircraft)				
				weaponsOrientationPositionUpdate(myNodeName1, elem);
				r = ats.attacks.rocketsInAir;
				if (rand() < 0.02 * weapPowerSkill * threatLevel / (r * r * r + 1)) launchRocket (id, myNodeName1, elem);
			}
			continue;
		}
	
		# if (ats.index == 1) debprint("Bombable: Weapons_loop for ", nodes[ats.index], " target = ", ind, "pos = ", pos, sprintf(" distance = %5.0fm nHit = %5.3f", targetData[pos][3], thisWeapon.aim.nHit));
		if (thisWeapon.aim.nHit == 0) 
		{
			pos += 1;
			if (pos == nTargets) pos = 0;
			thisWeapon.aim.target = myTargets[pos];
			# there is no chance of hitting the target so look for another
			continue;
		}
		
		var ballisticMass_lb = thisWeapon.maxDamage_percent * thisWeapon.maxDamage_percent / 100;
		# approximate ballisticMass_lb:
		# 0.08 lb for a 303 Vickers - see getBallisticMass_lb func 
		# 0.13 lb for a WWII 20mm Oerlikon cannon
		# 25 lb for a M830 round from the M256 120mm gun used on the M1 Abram
		# corresponding maxDamage_percent figures: 3%, 4%, 50%

		if (thisWeapon.aim.nHit > 0.1)
		debprint (sprintf("Bombable: Weapons_loop %s  weapPowerSkill = %4.1f  total ballistic mass =  %5.2f", myNodeName1, weapPowerSkill, ballisticMass_lb * thisWeapon.aim.nHit));
		# debprint (
		# 	"Bombable: Weapons_loop " ~ myNodeName1 ~ " " ~ elem, 
		# 	" heading = ", thisWeapon.weaponAngle_deg.heading, 
		# 	" elevation = ", thisWeapon.weaponAngle_deg.elevation
		# );


		# fire weapon
		# expectation value of damage is no hits * ballistic mass per round
		# bad gunners waste more ammo by firing when low expectation value
		# expectation value = a * (power-skill level) + b
		# range of power-skill level 0.1 to 1.0 
		# a skilled gunner with an effective weapon will fire at a higher expected damage threshold, dHigh; a weak combination at dLow
		# then a = 10 * (dHigh - dLow ) / 9 ; b = (10 * dLow - dHigh ) / 9


		# if (thisWeapon.aim.nHit * ballisticMass_lb > ( 0.044444 * weapPowerSkill + 0.1555555 )) # 0.2;0.16
		# if (thisWeapon.aim.nHit * ballisticMass_lb > ( 0.55555 * weapPowerSkill + 0.3444444 )) # 0.9;0.4
		# if (thisWeapon.aim.nHit * ballisticMass_lb > (0.277777 * weapPowerSkill + 0.022222)) # 0.3;0.05

		# if (0) # omit for testing
		if (thisWeapon.aim.nHit * ballisticMass_lb > (0.0166666 * weapPowerSkill + 0.003333)) # 0.02;0.005
		{
			# debprint ("Bombable: AI aircraft aimed at main aircraft, ",
			# myNodeName1, " ", thisWeapon.name, " ", elem,
			# " accuracy ", round(thisWeapon.aim.nHit * 100 ),"%",
			# " interceptSpeed", round(thisWeapon.aim.interceptSpeed), " mps");
			
			# fire weapons for visual effect
			var time2Fire =  3;
			fireAIWeapon(time2Fire, myNodeName1, thisWeapon, thisWeapon.aim.interceptSpeed);

			#reduce ammo count
			if (stores.reduceWeaponsCount (myNodeName1, elem, time2Fire) == 1)
			{
				var msg = thisWeapon.name ~ " on " ~ 
				getCallSign ( myNodeName1 ) ~ 
				" out of ammo";

				targetStatusPopupTip (msg, 20);

				# reset turret and gun positions with some random variation
				setprop("" ~ myNodeName1 ~ "/" ~ elem ~ "/cannon-elev-deg" , thisWeapon.weaponAngle_deg.initialElevation + (4 * rand() - 2));
				setprop("" ~ myNodeName1 ~ "/" ~ elem ~ "/turret-pos-deg" , thisWeapon.weaponAngle_deg.initialHeading + (10 * rand() - 5));
			}
						
			# TODO: a smaller chance of doing a fairly high level of damage (up to 3X the regular max),
			# and the better/closer the hit, the greater chance of doing that significant damage.
			# Some chance of doing more damage (and a higher chance the closer the hit)
			# e.g. damage non-linear function of pRound;

			var ai_callsign = getCallSign (myNodeName1);

			# nHit (0-10); weaponPower (0-1); ballisticMass_lb (0-25); damageVulnerability (0-100)
			var damageAdd = thisWeapon.aim.nHit * weaponPower * ballisticMass_lb * attributes[myNodeName2].vulnerabilities.damageVulnerability / 100;
						
			weaponName = thisWeapon.name;
			if (weaponName == nil) weaponName = "Main Weapon";

			if (myNodeName2 == "")
			{
				mainAC_add_damage ( damageAdd, 0, "weapons",
				"Hit from " ~ ai_callsign ~ " - " ~ weaponName ~"!");								
			}
			else
			{
				add_damage
				(
					damageAdd, 
					"weapon", 
					myNodeName2, 
					myNodeName1,
					,
					ballisticMass_lb
				);
			}
		}
		# var t_weap = (ot.timestamp.elapsedUSec()/ot.resolution_uS);
		# debprint(sprintf("Bombable: "~elem~" t_weap = %6.3f msec", t_weap));
	} # next weapon
}

############################ launchRocket ##############################
# FUNCTION set flags, send message
# create flare on launch pad
# for aircraft test speed of platform

var launchRocket = func (id, myNodeName1, elem) {
	var ats = attributes[myNodeName1];
	var thisWeapon = ats.weapons[elem];
	if (thisWeapon.aim.weaponDirRefFrame[2] == -1) return; # cannot launch rocket before checkAim called
	var delta_t = LOOP_TIME;

	# get speed and orientation of launchpad or guide rail
	var launchPadSpeed = 
	getprop ("" ~ myNodeName1 ~ "/velocities/true-airspeed-kt");
	var launchPadPitch = 
	getprop ("" ~ myNodeName1 ~ "/orientation/pitch-deg");
	# AC rockets are smaller than ground based and use the speed of the platform to achieve range 
	if (ats.type == "aircraft") 
	{
		debprint
		(
			sprintf("launch pad speed = %3.1f pitch = %3.1f dodge = %i attack = %i",
			launchPadSpeed, launchPadPitch, ats.controls.dodgeInProgress, ats.controls.attackInProgress)
		);
		if 
		(
			launchPadSpeed < ats.velocities.cruiseSpeed_kt or math.abs(launchPadPitch > 30) or
			(!ats.controls.dodgeInProgress and !ats.controls.attackInProgress)
		) return;
	}

	var launchPadHeading = 
	getprop ("" ~ myNodeName1 ~ "/orientation/true-heading-deg");
	var alat_deg = 
	getprop("" ~ myNodeName1 ~ "/" ~ elem ~ "/position/latitude-deg"); 
	var alon_deg = 
	getprop("" ~ myNodeName1 ~ "/" ~ elem ~ "/position/longitude-deg");
	var alt_ft = 
	getprop("" ~ myNodeName1 ~ "/" ~ elem ~ "/position/altitude-ft");

	# the launchPad velocity is used to set the initial velocity of the missile
	# at launch the missile is not necessarily oriented with its direction of travel
	# from ships or groundvehicles rockets are launched from tubes creating an initial vertical velocity
	# release time = time the rocket is guided by the tube or rail

	
	thisWeapon.velocities.thrustDir = thisWeapon.aim.weaponDirRefFrame;
	# acceleration in launch tube (length s) is thrust - weight / mass
	# s = ut + 1/2 at^2
	# 
	var acc = math.abs(thisWeapon.thrust1 / thisWeapon.launchMass - thisWeapon.velocities.thrustDir[2] * grav_mpss);
	thisWeapon.timeRelease = (acc !=0) ? math.sqrt(2.0 * thisWeapon.lengthTube / acc) : 0.0 ; 
	thisWeapon.velocities.missileV_mps = 
	[
	launchPadSpeed * math.cos ( launchPadPitch * D2R ) * math.sin( launchPadHeading * D2R ),
	launchPadSpeed * math.cos ( launchPadPitch * D2R ) * math.cos( launchPadHeading * D2R ),
	launchPadSpeed * math.sin ( launchPadPitch * D2R )
	];

	thisWeapon.velocities.speed = launchPadSpeed ; # store both vector and its magnitude to reduce calculation
	thisWeapon.velocities.lastMissileSpeed = 0;



	# AI model of rocket initiated by moving it from {lat, lon} {0, 0} to location of AC / ship
	var rp = "ai/models/static[" ~ thisWeapon.rocketsIndex ~ "]";
	var pitch = math.asin(thisWeapon.aim.weaponDirRefFrame[2]) * R2D; # orientation of rocket
	var heading = math.atan2(thisWeapon.aim.weaponDirRefFrame[0], thisWeapon.aim.weaponDirRefFrame[1]) * R2D;

	thisWeapon.position.latitude_deg = alat_deg;
	thisWeapon.position.longitude_deg = alon_deg;
	thisWeapon.position.altitude_ft = alt_ft;
	thisWeapon.controls.flightTime = 0;

	setprop (rp ~ "/position/latitude-deg", alat_deg);
	setprop (rp ~ "/position/longitude-deg", alon_deg);
	setprop (rp ~ "/position/altitude-ft", alt_ft);
	setprop (rp ~ "/orientation/pitch-deg", pitch);
	setprop (rp ~ "/orientation/true-heading-deg", heading);

	thisWeapon.controls.engine = 0;
	var ignitionDelay = (thisWeapon.massFuel_1 == 0) ? thisWeapon.burn1 : 0;
	settimer( func {
		setprop (rp ~ "/controls/engine", 1);
		thisWeapon.controls.engine = 1;
		}, ignitionDelay);  # animate plume from model

	pidControllerInit (thisWeapon, "phi", delta_t);
	pidControllerInit (thisWeapon, "theta", delta_t);

	# initialise air density at rocket altitude
	updateAirDensity (thisWeapon, alt_ft);
		
	# put extra smoke / flash on launch pad
	startSmoke ("blaze", rp, "AI/Aircraft/Fire-Particles/blaze-particles.xml" ); 
	settimer (
		func {
			deleteSmoke("blaze", rp ); 
			}
		, 2.0
	);

	# add contrail to AI rocket
	settimer (
		func {
			startSmoke("skywriting", rp, model = "AI/Aircraft/Fire-Particles/skywriting-particles.xml");
			}
		, 2.0
	);
		
	thisWeapon.loopCount = 0;
	thisWeapon.controls.abortCount = 0;

	thisWeapon.mass = thisWeapon.launchMass; # reset weapon mass

	ats.attacks.rocketsInAir += 1;
	ats.nRockets -= 1;

	var msg = thisWeapon.name ~ " launched from " ~ getCallSign (myNodeName1);

	targetStatusPopupTip (msg, 20);

	debprint ("Bombable: " ~ msg ~ " " ~ myNodeName1 ~ ", " ~ thisWeapon.name ~ " " ~ elem);

	thisWeapon.controls.launched = 1;

	props.globals.getNode("" ~ myNodeName1 ~ "/" ~ elem ~ "/rearm", 1).setBoolValue(0);

	var id2 = setlistener
		(
			"" ~ myNodeName1 ~ "/" ~ elem ~ "/rearm", 
			func (n)
			{
				if ( n.getValue() == 1)
				{
					settimer
					(
						func {
							if ((thisWeapon.controls.launched == 1) and
							(thisWeapon.destroyed == 1))
							{
								thisWeapon.controls.launched = 0;
								thisWeapon.destroyed = 0;
								removelistener(id2);
							}
							n.setBoolValue(0);
						},
						1.0
					);
				}
			},
			0,
			1
		);

	settimer (  
		func
			{
			guideRocket 
			(
				id,
				myNodeName1,
				elem
			);
			}, 0.1);
}

############################ guideRocket ##############################
# FUNCTION check if run out of fuel, check collision with target, direction of target,
# change direction, calculate position after time delta_t, trigger messages and effects
# thisWeapon points to the attributes hash which holds results of the previous calculation of rocket velocity and speed 
# it is updated with the new velocity and the estimated velocity and position at the next calculation
# the new position is recorded in the property tree for the static AI model of the rocket
# node 1 is AI, node 2 is target
# the guideRocket loop is terminated when the main weapons loop is terminated. It does not have a separate id
# the target is passed by thisWeapon.aim.target, the index number of the bombable object
# aim.rn is the key of any rocket from the target which is targeted preferentially over the platform from which it is launched
# if the object is destroyed the rocket circles waiting for another target to be allocated
# the waiting pattern is created by chasing a high-speed dummy target on a circular trajectory of 10k radius and centred on the rocket 

var guideRocket = func 
(
	id,
	myNodeName1,
	elem
)
{
	id == attributes[myNodeName1].loopids.weapons_loopid or return; 

	# ot.reset();
	var thisWeapon = attributes[myNodeName1].weapons[elem];

	# check for hit by other rocket
	if (thisWeapon.destroyed)
	{
		var weapKey = elem; # otherwise killRocket uses value of elem at time of call !!!
		killRocket (myNodeName1, weapKey);
		return ();
	}

	var myNodeName2 = nodes[thisWeapon.aim.target];
	var dim = attributes[myNodeName1].dimensions;  
	var rp = "ai/models/static[" ~ thisWeapon.rocketsIndex ~ "]"; # static model for rocket
	var delta_t = LOOP_TIME;

	thisWeapon.aim.nHit = 0;

	var alat_deg = thisWeapon.position.latitude_deg; # AI
	var alon_deg = thisWeapon.position.longitude_deg;
	var aAlt_m = thisWeapon.position.altitude_ft * FT2M;
	var missileSpeed_mps = thisWeapon.velocities.speed; # magnitude of velocity vector
	var missileDir = 
	[
		thisWeapon.velocities.missileV_mps[0] / missileSpeed_mps,
		thisWeapon.velocities.missileV_mps[1] / missileSpeed_mps,
		thisWeapon.velocities.missileV_mps[2] / missileSpeed_mps
	];
	var a_delta_dist = missileSpeed_mps * delta_t; # distance missile travels over this stage which is divided into N_STEPS

	if (math.abs (aAlt_m - thisWeapon.lastAlt_m) > 200.0) updateAirDensity (thisWeapon, aAlt_m);

	var damageRadius = attributes[myNodeName2].dimensions.damageRadius_m;
	var targetRocket = thisWeapon.aim["rn"]; # name of rocket to be used as target
	var noTarget = (attributes[myNodeName2].damage == 1); # true if this rocket has no target
	if (targetRocket != nil)
	{
		rocketAts = attributes[myNodeName2].weapons[targetRocket];
		if (!rocketAts.destroyed)
		{
			noTarget = 0; # this rocket is targetting another rocket
			damageRadius = rocketAts.length;
		}
		else
		{
			debprint("Bombable: ", getCallSign(myNodeName1), " ", thisWeapon.name, " removing target ",rocketAts.name," launched from ",getCallSign(myNodeName2));
			thisWeapon.aim.rn = nil; # target platform instead
			targetRocket = nil;
		}
	}

	if (noTarget) # create a dummy target so the rocket dwells on station
	{
		var distance_m = 1e4; #metres
		var period = 45; # sec
		var targetSpeed = TWOPI * distance_m / period; #mps
		var targetPitch = 0;
		var targetHeading = math.mod(thisWeapon.controls.flightTime, period) * TWOPI / period;
		var deltaX_m = math.cos(targetHeading) * distance_m;
		var deltaY_m = math.sin(targetHeading) * distance_m;
		var deltaAlt_m = 0;
	}
	else
	{
		# determine distances of target rocket [0] and/or launch platform [1]
		var r = [[0, 0, 0, 0], [0, 0, 0, 0]];
		var checkRocket = (targetRocket != nil and rocketAts.controls.launched) ;
		var checkPlatform = (!checkRocket or ( rand() < delta_t / 3 )) ; # check location of rocket carrier _and_ target rocket once every 3 sec 

		forindex (var i; r)
		{
			if ((i and !checkPlatform) or !(i or checkRocket)) continue;
			if (i) # check the distance of the platform carrying the rocket
			{
				var targetLat_deg = getprop("" ~ myNodeName2 ~ "/position/latitude-deg"); # target
				var targetLon_deg = getprop("" ~ myNodeName2 ~ "/position/longitude-deg");
				var targetAlt_m = getprop("" ~ myNodeName2 ~ "/position/altitude-ft") * FT2M;
			}
			else # check the distance of the rocket
			{
				var targetLat_deg = rocketAts.position.latitude_deg; # check the rocket distance
				var targetLon_deg = rocketAts.position.longitude_deg;
				var targetAlt_m = rocketAts.position.altitude_ft * FT2M;
			}

			deltaLat_deg = targetLat_deg - alat_deg;
			deltaLon_deg = targetLon_deg - alon_deg ;

			# calculate targetDispRefFrame, the displacement vector from node1 (rocket) to node2 (target) in a lon-lat-alt (x-y-z) frame of reference aka 'reference frame'
			# this rocket is at < 0,0,0 > 
			# and the target is at < deltaX,deltaY,deltaAlt > in relation to it.

			r[i][0] = deltaLon_deg * m_per_deg_lon;
			r[i][1] = deltaLat_deg * m_per_deg_lat;
			r[i][2] = targetAlt_m - aAlt_m;
			var sum = 0;
			foreach (var val; r[i]) sum += val * val;
			r[i][3] = sum;
		}
		var choice = !checkRocket; # 0=choose rocket and 1=choose platform
		if (checkRocket and checkPlatform and ( r[1][3] < r[0][3] / 4)) 
		{
			# change choice from rocket to its platform - switch only happens once
			choice = 1; 
			thisWeapon.aim["rn"] = nil;
			debprint ("Bombable: "~getCallSign(myNodeName1)~" "~elem~" switching target to "~getCallSign(myNodeName2));
		}
		var deltaX_m = r[choice][0] ;
		var deltaY_m = r[choice][1] ;
		var deltaAlt_m = r[choice][2];
		var distance_m = math.sqrt(r[choice][3]) ;

		# get speed and heading of target
		if (choice)
		{
			var addTrue = (myNodeName2 == "") ? "" : "true-";
			var targetSpeed = getprop(""~myNodeName2~"/velocities/"~addTrue~"airspeed-kt") * KT2MPS;
			var targetPitch = getprop("" ~ myNodeName2 ~ "/orientation/pitch-deg") * D2R;
			var targetHeading = getprop(""~myNodeName2~"/orientation/"~addTrue~"heading-deg") * D2R;
		}
		else
		{
			var targetSpeed = rocketAts.velocities.speed;
			var v = rocketAts.velocities.missileV_mps;
			var targetPitch = math.asin(v[2]/targetSpeed);
			var targetHeading = math.atan2(v[1], v[0]);
		}
	}

	var targetDispRefFrame = [deltaX_m, deltaY_m, deltaAlt_m];
	var target_delta_dist = targetSpeed * delta_t;
	var targetVelocity =
		[
		math.cos ( targetPitch ) * math.sin( targetHeading ) * targetSpeed,
		math.cos ( targetPitch ) * math.cos( targetHeading ) * targetSpeed,
		math.sin ( targetPitch ) * targetSpeed
		];

	# variables used to calculate the new positions
	var step = a_delta_dist / N_STEPS;
	var time_inc = delta_t / N_STEPS;
	thisWeapon.controls.index = 0; # indexes flightpath used by moveRocket
	var deltaXYZ = [0, 0, 0];
	var deltaLat = 0;
	var deltaLon = 0;
	var deltaAlt = 0;
	var newMissileDir = missileDir;
	var t_intercept = 0;
	var abort = 0; # flag used to trigger kill rocket; set by checks on height, fuel, proximity to launch pad, collision
	
	# debprint (
	# 	sprintf(
	# 		"Bombable: distance vector to target =[%8.2f, %8.2f, %8.2f]",
	# 		deltaX_m, deltaY_m, deltaAlt_m 
	# 	)
	# );


	# abort if target cannot be reached
	if (thisWeapon.controls.engine == 0 and !abort)
	{
		if ( math.mod(thisWeapon.loopCount, 4) == 1)
		{	
			var glide_range = thisWeapon.liftDragRatio * ( missileSpeed_mps * missileSpeed_mps / 2 / grav_mpss - deltaAlt_m );
			# https://therestlesstechnophile.com/2018/05/26/useful-physics-equations-for-military-system-analysis/
			if (distance_m * distance_m > glide_range * glide_range + deltaAlt_m * deltaAlt_m ) 
			{
				thisWeapon.controls.abortCount += 1;
			}
			else
			{
				thisWeapon.controls.abortCount = 0; # reset
			}
			if (thisWeapon.controls.abortCount == 3) # require 3 successive
			{
				var msg = thisWeapon.name ~ " from " ~ 
				getCallSign(myNodeName1) ~ 
				" aborted - too far from target";
				targetStatusPopupTip (msg, 20);						
				abort = 1;
			}
		}
	
	}

	# abort if close to launch vehicle
	if ( math.mod(thisWeapon.loopCount, 4) == 2 and !abort)
	{
		if (thisWeapon.controls.flightTime > 10) # must exceed time for rocket to reach safeDist after launch
		{
			var dx = 
			(getprop("" ~ myNodeName1 ~ "/" ~ elem ~ "/position/latitude-deg") - thisWeapon.position.longitude_deg ) *
			m_per_deg_lat;
			var dy =
			(getprop("" ~ myNodeName1 ~ "/" ~ elem ~ "/position/longitude-deg") - thisWeapon.position.latitude_deg ) *
			m_per_deg_lon;
			var dz = 
			(getprop("" ~ myNodeName1 ~ "/" ~ elem ~ "/position/altitude-ft") - thisWeapon.position.altitude_ft ) *
			FT2M;
			if (dx * dx + dy * dy + dz * dz < dim.safeDistance_m * dim.safeDistance_m) 
			{
				var msg = thisWeapon.name ~ " from " ~ 
				getCallSign (myNodeName1) ~ 
				" aborted - too close to launch vehicle";
				targetStatusPopupTip (msg, 20);	
				abort = 1;					
			}
		}
	}

	if (distance_m < a_delta_dist and !abort)
	{	
		var dV = 
		[
			targetVelocity[0] - thisWeapon.velocities.missileV_mps[0],
			targetVelocity[1] - thisWeapon.velocities.missileV_mps[1],
			targetVelocity[2] - thisWeapon.velocities.missileV_mps[2]
		];
		var dV2 = dV[0] * dV[0] + dV[1] * dV[1] + dV[2] * dV[2];
		var dVdotR =  dV[0] * targetDispRefFrame[0] + dV[1] * targetDispRefFrame[1] + dV[2] * targetDispRefFrame[2];
		var t_intercept  = -dVdotR / dV2;
		if ((distance_m < damageRadius)
			and (t_intercept <0)) t_intercept = 0; # already reached closest approach
		# debprint (
		# 	sprintf(
		# 	"Bombable: time to intercept %5.2f", 
		# 	t_intercept
		# 	)
		# );
		if ((t_intercept >= 0) and (t_intercept < delta_t))
		{
			var closestApproach = math.pow(
				distance_m * distance_m + t_intercept * dVdotR,
				0.5
			);

			debprint (
				sprintf(
				"Bombable: closest approach for %s:%6.1fm intercept time:%6.1fs", 
				myNodeName1 ~ "/" ~ elem ,
				closestApproach,
				t_intercept
				)
			);

			var ratio = closestApproach / damageRadius;
			if (ratio < 2)
			{
				# run missile on to target and explode
				var steps_to_target = (t_intercept > 0) ? math.ceil ( t_intercept / time_inc  ) : 0;
				deltaXYZ = 
				[
					thisWeapon.velocities.missileV_mps[0] * time_inc,
					thisWeapon.velocities.missileV_mps[1] * time_inc,
					thisWeapon.velocities.missileV_mps[2] * time_inc
				];

				for (var i = 0; i < N_STEPS; i = i + 1) 
				{
					if (i < steps_to_target)
					{
						deltaLon = deltaLon + deltaXYZ[0] / m_per_deg_lon;
						deltaLat = deltaLat + deltaXYZ[1] / m_per_deg_lat;
						deltaAlt = deltaAlt + deltaXYZ[2];
					}
					thisWeapon.controls.flightPath[i].lon = alon_deg + deltaLon;
					thisWeapon.controls.flightPath[i].lat = alat_deg + deltaLat;
					thisWeapon.controls.flightPath[i].alt = ( aAlt_m + deltaAlt ) * M2FT;
				}
				settimer ( func{ moveRocket (thisWeapon, 0, time_inc )}, 0 ); # first step called immediately

				#rockets are either completely destroyed or left undamaged
				if (targetRocket != nil ? rocketAts.controls.launched == 1 : 0)
				{
					var msg = thisWeapon.name ~ " from " ~ 
					getCallSign (myNodeName1);
					if (ratio < 1 or ( rand() < ratio - 1))
					{
						rocketAts.destroyed = 1;
						msg ~= " destroyed ";
					}
					else
					{
						msg ~= " exploded near to ";

					}
					msg ~= rocketAts.name ~ " from " ~ 
					getCallSign (myNodeName2);
					targetStatusPopupTip (msg, 20);	
					debprint(msg);
				}
				else
				{
					# message rocket hit from add_damage
					var weaponPower = bombableMenu["ai-weapon-power"];
					if (weaponPower == nil) weaponPower = 0.2;
					thisWeapon.aim.nHit = rand() * damageRadius / closestApproach;
					var damagePercent = thisWeapon.aim.nHit * weaponPower * thisWeapon.mass * 2.2 * attributes[myNodeName2].vulnerabilities.damageVulnerability / 100; 
					debprint(sprintf("nHit= %5.1f damagePercent %5.1f callsign= %s", thisWeapon.aim.nHit, damagePercent, getCallSign(myNodeName2) ));
					add_damage
					(
						damagePercent, 
						"weapon", 
						myNodeName2, 
						myNodeName1, 
						, 
						thisWeapon.mass * 2.2, 
						thisWeapon.controls.flightPath[i-1].lat, 
						thisWeapon.controls.flightPath[i-1].lon, 
						thisWeapon.controls.flightPath[i-1].alt
					);
				}
				
				abort = 1;
			}
			elsif (closestApproach < (4.0 * damageRadius ))
			{
				var msg = "Near miss from " ~ 
				thisWeapon.name ~ " fired from " ~ 
				getCallSign (myNodeName1);

				targetStatusPopupTip (msg, 20);

				debprint (msg);
			}
		}
	}

	# check above ground level
	var GeoCoord = geo.Coord.new();
	GeoCoord.set_latlon(alat_deg, alon_deg);
	var ground_Alt_m = elev (GeoCoord.lat(), GeoCoord.lon()) * FT2M; 
	if (ground_Alt_m > aAlt_m) 
	{
		debprint (sprintf("Bombable: checkAGL for %s: rocket alt = %6.0fm ground alt = %6.0fm",  myNodeName1 ~ "/" ~ elem , aAlt_m, ground_Alt_m));

		var msg = thisWeapon.name ~ " from " ~ 
		getCallSign(myNodeName1) ~ 
		" hit ground and destroyed";
		targetStatusPopupTip (msg, 20);						
		abort = 1;
	}	

	# check for abort
	if (abort)
	{
		var weapKey = elem; # otherwise killRocket uses value of elem at time of call !!!
		settimer
		( func
			{
				killRocket (myNodeName1, weapKey);
			},
			t_intercept
		);
		return ();
	}

	# look ahead by assessing relative positions at next update 

	var nextMissileSpeed = 2.0 * missileSpeed_mps - thisWeapon.velocities.lastMissileSpeed;
	var nextTargetVelocity =
		[
			2.0 * targetVelocity[0] - thisWeapon.aim.lastTargetVelocity[0],
			2.0 * targetVelocity[1] - thisWeapon.aim.lastTargetVelocity[1],
			2.0 * targetVelocity[2] - thisWeapon.aim.lastTargetVelocity[2],
		];
	thisWeapon.velocities.lastMissileSpeed = missileSpeed_mps;
	thisWeapon.aim.lastTargetVelocity = targetVelocity;
	

	targetDispRefFrame = 
	[
		targetDispRefFrame[0] + (targetVelocity[0] - thisWeapon.velocities.missileV_mps[0]) * delta_t,
		targetDispRefFrame[1] + (targetVelocity[1] - thisWeapon.velocities.missileV_mps[1]) * delta_t,
		targetDispRefFrame[2] + (targetVelocity[2] - thisWeapon.velocities.missileV_mps[2]) * delta_t
	];

	distance_m = math.sqrt ( targetDispRefFrame[0] * targetDispRefFrame[0] + targetDispRefFrame[1] * targetDispRefFrame[1] + targetDispRefFrame[2] * targetDispRefFrame[2] );


	# find the velocity vector of a missile travelling at nextMissileSpeed (constant over journey)
	# to intercept the target

	var intercept = findIntercept2(
		targetDispRefFrame,
		distance_m,
		nextMissileSpeed,
		nextTargetVelocity
	);

	if (intercept.time != 9999) 
	{
		# if (!thisWeapon.rocketsIndex) 
		# {
		# 	debprint (
		# 		sprintf(
		# 			"Bombable: intercept.vector =[%0.1f, %0.1f, %0.1f] intercept.time= %3.1fs",
		# 			intercept.vector[0], intercept.vector[1], intercept.vector[2], intercept.time
		# 		)
		# 	);
		# }
		thisWeapon.aim.interceptSpeed = math.sqrt ( intercept.vector[0] * intercept.vector[0] + intercept.vector[1] * intercept.vector[1] + intercept.vector[2] * intercept.vector[2] );

		var interceptDirRefFrame = 
		[
			intercept.vector[0] / nextMissileSpeed,
			intercept.vector[1] / nextMissileSpeed,
			intercept.vector[2] / nextMissileSpeed
		];
	}
	else
	# no intercept - at start of flight it is not possible to calculate an intercept because the speed is too low 
	{
		# if (!thisWeapon.rocketsIndex) debprint("No intercept");
		var interceptDirRefFrame =
		[
			targetDispRefFrame[0] / distance_m,
			targetDispRefFrame[1] / distance_m,
			targetDispRefFrame[2] / distance_m
		];
	}

	# calculate angular offset between direction of missile and direction of target

	var turnRate = 0.0;
	# no turn if too soon in flight
	if ( thisWeapon.controls.flightTime > thisWeapon.timeNoTurn + rand()) 
	{	
		# only turn if above minimum speed 
		var speedFactor = missileSpeed_mps / thisWeapon.minTurnSpeed_mps ;
		if (speedFactor > 1.0) 
		{
			if (speedFactor < thisWeapon.speedX) 
			{
				turnRate = thisWeapon.maxTurnRate * (speedFactor - 1.0) / (thisWeapon.speedX - 1.0) * delta_t ; 
			}
			else
			{
				turnRate = thisWeapon.maxTurnRate;
			}
			
			if (turnRate * missileSpeed_mps > thisWeapon.maxG)
			{
				turnRate = thisWeapon.maxG / missileSpeed_mps;
			}

			# ground avoidance
			if (( aAlt_m - ground_Alt_m < -2.0 * a_delta_dist * missileDir[2] ) and
			( distance_m > 2.0 * a_delta_dist )) # look ahead distance - factors of 2 arbitrary
			{
				interceptDirRefFrame = [0, 0, 1]; # instead might follow gradient of ground in xy direction of travel
				# debprint (
				# 	sprintf(
				# 		"Bombable: ground avoidance: AGL =%8.1f Vertical speed =%8.1f mps",
				# 		aAlt_m - ground_Alt_m,
				# 		missileDir[2] * missileSpeed_mps * delta_t 
				# 	)
				# );	
			}
			else
			# only turn if space to do so, otherwise waste energy
			# TODO make this a smooth reduction in size of turn?
			{
			var cosOffset = dotProduct(missileDir, interceptDirRefFrame);
			var allowedTurn = distance_m * turnRate / missileSpeed_mps;
			if (cosOffset < -0.95) # 162 deg
				{
					if (allowedTurn < PI) turnRate = 0.0; 
				}
				elsif (cosOffset < 0)
				{
					if  (allowedTurn < PIBYTHREE) turnRate = 0.0;
				}
				elsif (cosOffset < 0.5)
				{
					if  (allowedTurn < PIBYSIX) turnRate = 0.0;
				}
			}
		}
	}

	# update pid controller limits on size of turn
	thisWeapon.pidData.phi.limMax = turnRate * delta_t;
	thisWeapon.pidData.phi.limMin = -thisWeapon.pidData.phi.limMax;
	thisWeapon.pidData.theta.limMax = thisWeapon.pidData.phi.limMax * 0.75; # vertical turn rate is smaller than the horizontal one
	thisWeapon.pidData.theta.limMin = -thisWeapon.pidData.theta.limMax; # and symmetric

	if (turnRate != 0) 
	{
		var newDir = changeDirection( thisWeapon, missileDir, interceptDirRefFrame, missileSpeed_mps, turnRate ) ; 
		var turnRad = thisWeapon.pidData.phi.out;
	}
	else
	{
		var newDir = missileDir;
		var turnRad = 0;
	}

	# creates set of intermediate positions in wayPoint hash
	# incremental change in position given by vector newMissileDir * time_inc
	# newMissileDir changes each time increment
	var newV = newVelocity( thisWeapon, missileSpeed_mps, newDir, turnRad, delta_t, aAlt_m );

	var newMissileSpeed_mps = math.sqrt ( newV[0] * newV[0] + newV[1] * newV[1] + newV[2] * newV[2] );
	var newMissileDir = 
	[
		newV[0] / newMissileSpeed_mps,
		newV[1] / newMissileSpeed_mps,
		newV[2] / newMissileSpeed_mps
	];
	var newPitch = math.asin( thisWeapon.velocities.thrustDir[2] ) * R2D; #change to the direction of the thrust vector
	var newHeading = math.atan2(newMissileDir[0], newMissileDir[1]) * R2D;
	
	
	var v = thisWeapon.velocities.missileV_mps; # current velocity
	var deltaV = 
	[
		( newV[0] - v[0] ) / N_STEPS,
		( newV[1] - v[1] ) / N_STEPS,
		( newV[2] - v[2] ) / N_STEPS
	]; # velocity increments

	# the rocket position and orientation are forced to follow the calculated rocket flightpath
	for (var i = 0; i < N_STEPS; i = i + 1) 
	{
		deltaLon = deltaLon + v[0] * time_inc / m_per_deg_lon;
		deltaLat = deltaLat + v[1] * time_inc / m_per_deg_lat;
		deltaAlt = deltaAlt + v[2] * time_inc;
		v = 
		[
			v[0] + deltaV[0],
			v[1] + deltaV[1],
			v[2] + deltaV[2]
		];
		thisWeapon.controls.flightPath[i].lon = alon_deg + deltaLon;
		thisWeapon.controls.flightPath[i].lat = alat_deg + deltaLat;
		thisWeapon.controls.flightPath[i].alt = ( aAlt_m + deltaAlt ) * M2FT;
		thisWeapon.controls.flightPath[i].pitch =  newPitch ; 
		thisWeapon.controls.flightPath[i].heading = math.atan2( v[0], v[1] ) * R2D;;
	}

	settimer ( func{ moveRocket (thisWeapon, 0, time_inc )}, 0 ); # first step called immediately

	thisWeapon.velocities.missileV_mps = newV;
	thisWeapon.velocities.speed = newMissileSpeed_mps;
	thisWeapon.position.longitude_deg = alon_deg + deltaLon;
	thisWeapon.position.latitude_deg = alat_deg + deltaLat;
	thisWeapon.position.altitude_ft = ( aAlt_m + deltaAlt ) * M2FT;

	var rp = "ai/models/static[" ~ thisWeapon.rocketsIndex ~ "]";
	setprop("" ~ rp ~ "/velocities/true-airspeed-kt", newMissileSpeed_mps); # used for debug only

	if ( math.mod(thisWeapon.loopCount, 6) == 0 ) # determines frequency of report out 
	{
		if ((intercept.time > 5) and (intercept.time < 20))
		{
			var msg = thisWeapon.name ~ " from " ~ 
			getCallSign ( myNodeName1 ) ~ 
			sprintf(
			" intercept in%5.1fs hdg%6.1f deg pitch%5.1f deg speed%7.1fmps mach %5.1f",
			intercept.time,
			newHeading,
			newPitch,
			newMissileSpeed_mps,
			newMissileSpeed_mps / thisWeapon.speedSound
			);

			targetStatusPopupTip (msg, 5);
		}
	}

	if ( thisWeapon.controls.flightTime + delta_t > thisWeapon.burn_1_2_3) 
		{
			if ( thisWeapon.controls.flightTime < thisWeapon.burn_1_2_3 )
			{
				var msg = thisWeapon.name ~ " fired from " ~ 
				getprop ("" ~ myNodeName1 ~ "/callsign") ~ 
				" out of fuel";

				targetStatusPopupTip (msg, 10);
				# reduce thrust to zero
				setprop (rp ~ "/controls/engine", 0);
				thisWeapon.controls.engine = 0;
			}
		}


	# section to print flight stats to fgfs.log in C:\Users\userName\AppData\Roaming\flightgear.org

	# debprint (
	# 	sprintf(
	# 		"Bombable: co-ords: lon = %8.4f lat = %8.4f alt = %8.4f",
	# 		alon_deg + deltaLon, alat_deg + deltaLat, aAlt_m + deltaAlt
	# 	)
	# );

	# debprint (
	# 	sprintf(
	# 		"Bombable: intercept vector  =[%8.3f, %8.3f, %8.3f] intercept time =%8.1f",
	# 		interceptDirRefFrame[0], interceptDirRefFrame[1], interceptDirRefFrame[2],
	# 		intercept.time
	# 	)
	# );

	# debprint (
	# 	sprintf(
	# 		"Bombable: thrust direction  =[%8.3f, %8.3f, %8.3f]",
	# 		thisWeapon.velocities.thrustDir[0], thisWeapon.velocities.thrustDir[1], thisWeapon.velocities.thrustDir[2] 
	# 	)
	# );

	# debprint (
	# 	sprintf(
	# 		"Bombable: missile direction =[%8.3f, %8.3f, %8.3f]",
	# 		newMissileDir[0], newMissileDir[1], newMissileDir[2] 
	# 	)
	# );

	# debprint(
	# 	sprintf(
	# 		"Bombable: t = %6.1f pitch = %6.1f hdg = %6.1f spd_mps = %8.2f mach = %6.1f turnRate = %6.2f",
	# 		thisWeapon.controls.flightTime,
	# 		newPitch,
	# 		newHeading,
	# 		newMissileSpeed_mps,
	# 		newMissileSpeed_mps / thisWeapon.speedSound,
	# 		turnRad / delta_t * R2D
	# 	)
	# );

	thisWeapon.controls.flightTime += delta_t ;
	thisWeapon.loopCount += 1;

	settimer (  
	func
		{
		guideRocket 
		(
			id,
			myNodeName1,
			elem 
		);
		}, delta_t);

	# var t_guideRocket = (ot.timestamp.elapsedUSec()/ot.resolution_uS);
	# debprint(sprintf("Bombable: " ~ elem ~ " t_guideRocket = %6.3f msec", t_guideRocket));

	return ();
}

############################ changeDirection ##############################
# changes the direction the rocket is pointing:
# uses theta and phi (pitch and heading)
# changes direction of thrust assumed to act along the axis of the rocket 
# returns new direction of velocity vector
# using only the thrust to change the velocity vector did not give sufficient control
# noTurn is a flag to allow the PID to init early in flight when no turn permitted

var changeDirection = func ( thisWeapon, missileDir, interceptDir, missileSpeed_mps, turnRate ) {
	var theta1 = math.asin(missileDir[2]);
	var theta2 = math.asin(interceptDir[2]);
	var thetaNew = pidController (thisWeapon, "theta", theta2, theta1) + theta1;

	var phi1 = math.atan2(missileDir[0], missileDir[1]);
	var phi2 = math.atan2(interceptDir[0], interceptDir[1]);
	var deltaPhi = pidControllerCircular (thisWeapon, "phi", phi2, phi1);
	var phiNew = deltaPhi + phi1;

	# debprint (
	# 	sprintf(
	# 		"Bombable: pid: theta =%8.3f int =%8.3f meas =%8.3f set =%8.3f err =%8.3f",
	# 		thetaNew,
	# 		thisWeapon.pidData.theta.integrator,
	# 		thisWeapon.pidData.theta.prevMeasurement,
	# 		theta2,
	# 		thisWeapon.pidData.theta.prevError
	# 	)
	# );
	# debprint (
	# 	sprintf(
	# 		"Bombable: pid: phi =%8.3f int =%8.3f meas =%8.3f set =%8.3f err =%8.3f",
	# 		phiNew,
	# 		thisWeapon.pidData.phi.integrator,
	# 		thisWeapon.pidData.phi.prevMeasurement,
	# 		phi2,
	# 		thisWeapon.pidData.phi.prevError			
	# 	)
	# );

	if ( turnRate == 0 ) return (missileDir);

	var newDir =
		[ 
			math.cos(thetaNew) * math.sin(phiNew),
			math.cos(thetaNew) * math.cos(phiNew),
			math.sin(thetaNew)		
		];

	# if need to fly up add an angle of attack to provide lift  
	if (theta2 > -thisWeapon.AoA)
	{
		var thrustTheta = thetaNew + thisWeapon.AoA;
		if (thrustTheta > PIBYTWO) 
		{
			thrustTheta = PIBYTWO;
		}
		thisWeapon.velocities.thrustDir[0] = math.cos(thrustTheta) * math.sin(phiNew);
		thisWeapon.velocities.thrustDir[1] = math.cos(thrustTheta) * math.cos(phiNew);
		thisWeapon.velocities.thrustDir[2] = math.sin(thrustTheta);
	}
	else
	{
		thisWeapon.velocities.thrustDir = newDir;
	}


	return ( newDir );
}

############################ newVelocity ##############################
# calculates velocity of rocket after delta_t accounting for:
# gravity, skin drag, turn drag
# missileDir is the direction of the velocity vector 
# it may not be aligned with the thrust vector
# func returns new velocity

var newVelocity = func (thisWeapon, missileSpeed_mps, missileDir, deltaPhi, delta_t, alt_m ) {

	var thrust = 0.0;
	if ( thisWeapon.controls.flightTime < thisWeapon.burn1 )
	{ 
		thrust = thisWeapon.thrust1;
		thisWeapon.mass -= delta_t * thisWeapon.fuelRate1;
	}
	elsif ( thisWeapon.controls.flightTime < thisWeapon.burn_1_2 )
	{
		thrust = thisWeapon.thrust2; # second stage of rocket
		thisWeapon.mass -= delta_t * thisWeapon.fuelRate2;
	}
	elsif ( thisWeapon.controls.flightTime < thisWeapon.burn_1_2_3 )
	{
		thrust = thisWeapon.thrust3; # third stage of rocket
		thisWeapon.mass -= delta_t * thisWeapon.fuelRate3;
	}

	var cD0 = zeroLiftDrag (thisWeapon, missileSpeed_mps); # note cD0, cN stored in attributes for debugging
	var cN = cN(thisWeapon);
	thisWeapon.axialForce = thisWeapon.rhoAby2 * cD0 * missileSpeed_mps * missileSpeed_mps; # component of drag acting along the missile axis
	var normalForce = thisWeapon.axialForce * cN / cD0; # magnitude of force acting at 90 degrees to missile axis

	# calculate net force
	var hdg = math.atan2 ( thisWeapon.velocities.thrustDir[0], thisWeapon.velocities.thrustDir[1] );
	var normalForceVector = 
	[
		- thisWeapon.velocities.thrustDir[2] * math.sin (hdg) * normalForce,
		- thisWeapon.velocities.thrustDir[2] * math.cos (hdg) * normalForce,
		math.cos ( math.asin(thisWeapon.velocities.thrustDir[2]) ) * normalForce
	]; 

	# debprint (
	# 	sprintf(
	# 		"Bombable: normal force vector =[%8.3f, %8.3f, %8.3f]",
	# 		normalForceVector[0], normalForceVector[1], normalForceVector[2] 
	# 	)
	# );

	# thrust minus drag plus lift
	if (thisWeapon.controls.flightTime < thisWeapon.timeRelease)
	{
		# weight only acts along the axis of thrust
		var ta = thrust - thisWeapon.axialForce - thisWeapon.velocities.thrustDir[2] * thisWeapon.mass * grav_mpss;
		var netForce = 
		[
			thisWeapon.velocities.thrustDir[0] * ta + normalForceVector[0],
			thisWeapon.velocities.thrustDir[1] * ta + normalForceVector[1],
			thisWeapon.velocities.thrustDir[2] * ta + normalForceVector[2]
		];
	}
	else
	{
		var ta = thrust - thisWeapon.axialForce;
		var netForce = 
		[
			thisWeapon.velocities.thrustDir[0] * ta + normalForceVector[0],
			thisWeapon.velocities.thrustDir[1] * ta + normalForceVector[1],
			thisWeapon.velocities.thrustDir[2] * ta + normalForceVector[2] - thisWeapon.mass * grav_mpss
		]; # adding weight along z-axis
	}

	thisWeapon.liftDragRatio = (cN - cD0 * thisWeapon.AoA) / (cN * thisWeapon.AoA + cD0); # small angle approximation for AoA
	# reduce speed according to rate of turn
	# approximation of (v - dv) / v = exp (-deltaPhi / liftDragRatio )
	# https://therestlesstechnophile.com/2020/05/04/modelling-missiles-in-the-atmosphere/

	# var dragFactor = 1.0 - math.abs(deltaPhi) / liftDragRatio + deltaPhi * deltaPhi / liftDragRatio / liftDragRatio / 2.0; # assume calc of exponent = expensive
	var dragFactor = math.exp(- math.abs(deltaPhi) / thisWeapon.liftDragRatio );
		
	# calculate resultant acceleration and change in velocity over delta_t and multiply by dragFactor
	var tbym = delta_t / thisWeapon.mass;
	var newV =
	[
		(missileDir[0] * missileSpeed_mps + netForce[0] * tbym) * dragFactor,
		(missileDir[1] * missileSpeed_mps + netForce[1] * tbym) * dragFactor,
		(missileDir[2] * missileSpeed_mps + netForce[2] * tbym) * dragFactor
	];
	
	# debprint (
	# 	sprintf(
	# 		"Bombable: new velocity vector =[%8.3f, %8.3f, %8.3f]",
	# 		newV[0], newV[1], newV[2] 
	# 	)
	# );

	return(newV);
}

############################ moveRocket ##############################
# moveRocket updates the position of the AI model
# note difference from guideRocket which calculates its overall flightpath
# updates position, speed and orientation of AI model of rocket
# the rocket position and orientation are forced to follow the calculated rocket flightpath
# the AI controls are overriden

var moveRocket = func (thisWeapon, index, timeInc) {

	#check allows the index to be incremented for early termination
	index == thisWeapon.controls.index or return;

	#get flighpath waypoint
	var fpath = thisWeapon.controls.flightPath;

	var rp = "ai/models/static[" ~ thisWeapon.rocketsIndex ~ "]";

	setprop (rp ~ "/position/longitude-deg", fpath[index].lon);
	setprop (rp ~ "/position/latitude-deg", fpath[index].lat);
	setprop (rp ~ "/position/altitude-ft", fpath[index].alt);
	setprop (rp ~ "/orientation/pitch-deg", fpath[index].pitch);
	setprop (rp ~ "/orientation/true-heading-deg", fpath[index].heading);

	var nextIndex = index + 1;
	thisWeapon.controls.index = nextIndex;
	if (nextIndex < N_STEPS) settimer ( func{ moveRocket (thisWeapon, nextIndex, timeInc)}, timeInc );

	return;
}



############################ killRocket ##############################
# call for effects	
# move rocket out of scene

var killRocket = func (myNodeName, elem) {
	# debprint("Bombable: " ~ elem ~ " killed: rocket index " ~ thisWeapon.rocketsIndex);
	var ats = attributes[myNodeName];
	var thisWeapon = ats.weapons[elem];
	thisWeapon.destroyed = 1;
	ats.attacks.rocketsInAir -= 1;
	thisWeapon.controls.index += 1;
	if (ats.maxTargets) ats.maxTargets -= 1;
	var rp = "ai/models/static[" ~ thisWeapon.rocketsIndex ~ "]";	
	setprop (rp ~ "/controls/engine", 0);
	deleteSmoke ("skywriting", rp);
	startSmoke ("flare", rp, "AI/Aircraft/Fire-Particles/large-explosion-particles.xml" ); 
	settimer (
		func {
			deleteSmoke("flare", rp ); 
			setprop (rp ~ "/position/latitude-deg", 0);
			setprop (rp ~ "/position/longitude-deg", 0);
			setprop (rp ~ "/position/altitude-ft", 0);
			}
		, 4.0
	);

}

############################ pidControllerInit ##############################
var pidControllerInit = func(thisWeapon, angle, delta_t) {

	# Clear controller variables 
	var pid = thisWeapon.pidData[angle];
	
	pid.integrator = 0.0;
	pid.prevError  = 0.0;

	pid.differentiator  = 0.0;
	pid.prevMeasurement = 0.0;

	pid.out = 0.0;

	pid.delta_t = delta_t;

	pid.tau = pid.delta_t * 3.0;

}

############################ pidController ##############################
# proportional - integral - differentiation controller on rocket direction	
# the angle of the velocity vector to the horizontal is the measurement
# code adapted from https://github.com/pms67/PID
# differentiator not used

var pidController = func (thisWeapon, angle, setpoint, measurement) {

	# pointer into hash containing PID data
	var pid = thisWeapon.pidData[angle];

	# Error signal
	var error = setpoint - measurement;

	# Proportional
    var proportional = pid.Kp * error;


	# Integral
    pid.integrator = pid.integrator + 0.5 * pid.Ki * pid.delta_t * (error + pid.prevError);

	# Anti-wind-up via integrator clamping 
    if (pid.integrator > pid.limMaxInt) 
	{
        pid.integrator = pid.limMaxInt;
		debprint(thisWeapon, " maxInt clamped");
    } 
	elsif (pid.integrator < pid.limMinInt) 
	{
        pid.integrator = pid.limMinInt;
		debprint(thisWeapon, " minInt clamped");
    }


	# Derivative (band-limited differentiator)
    # pid.differentiator = -(2.0 * pid.Kd * (measurement - pid.prevMeasurement)	# Note: derivative on measurement, therefore minus sign in front of equation! 
    #                     + (2.0 * pid.tau - pid.delta_t) * pid.differentiator)
    #                     / (2.0 * pid.tau + pid.delta_t);


	# Compute output and apply limits
    pid.out = proportional + pid.integrator + pid.differentiator;

    if (pid.out > pid.limMax) 
	{
        pid.out = pid.limMax;
    } 
	elsif (pid.out < pid.limMin) 
	{
        pid.out = pid.limMin;
    }

	# Store error and measurement for later use 
    pid.prevError       = error;
    pid.prevMeasurement = measurement;

	# Return controller output 
    return (pid.out);

}
############################ normaliseAngle ##############################
# helper func to manage circular variables
 var normaliseAngle = func(angle)
 {
	if (angle <= -PI) return (angle + TWOPI);
	if (angle > PI) return (angle - TWOPI);
	return (angle);
 }



############################ pidControllerCircular ##############################
# PID for circular variables

var pidControllerCircular = func (thisWeapon, angle, setpoint, measurement) {

	# pointer into hash containing PID data
	var pid = thisWeapon.pidData[angle];

	# Error signal
	var error = normaliseAngle ( setpoint - measurement );

	# Proportional
    var proportional = pid.Kp * error;


	# Integral
    pid.integrator = pid.integrator + 0.5 * pid.Ki * pid.delta_t * ( error + pid.prevError );
	pid.integrator = normaliseAngle (pid.integrator);

	# Anti-wind-up via integrator clamping 
    if (pid.integrator > pid.limMaxInt) 
	{
        pid.integrator = pid.limMaxInt;
    } 
	elsif (pid.integrator < pid.limMinInt) 
	{
        pid.integrator = pid.limMinInt;
    }


	# Derivative (band-limited differentiator)
    # pid.differentiator = -(2.0 * pid.Kd * (measurement - pid.prevMeasurement)	# Note: derivative on measurement, therefore minus sign in front of equation! 
    #                     + (2.0 * pid.tau - pid.delta_t) * pid.differentiator)
    #                     / (2.0 * pid.tau + pid.delta_t);


	# Compute output and apply limits
    pid.out = normaliseAngle ( proportional + pid.integrator + pid.differentiator );

	if (pid.out > pid.limMax) 
	{
        pid.out = pid.limMax;
    } 
	elsif (pid.out < pid.limMin) 
	{
        pid.out = pid.limMin;
    }

	# Store error and measurement for later use 
    pid.prevError       = error;
    pid.prevMeasurement = measurement;

	# Return controller output 
    return (pid.out);

}
############################ air density ##############################
# FUNCTION return air density in kg m^-3 at rocket altitude
# from https://eng.libretexts.org/Bookshelves/Aerospace_Engineering/Fundamentals_of_Aerospace_Engineering_(Arnedo)/02%3A_Generalities/2.03%3A_Standard_atmosphere/2.3.03%3A_ISA_equations
# altitude (h) is in metres above sea level
# two regimes 0 < h < 11000m the troposphere and
# 11000m <= h < 20000m the near stratosphere

var airDensity = func (h) {
	if (h < 11000)
	{
		return
		(
			1.225 * math.pow( 1 - 22.558e-6 * h, 4.2559)
		);
	}
	else
	{
		return
		(
			0.3639 * math.exp(-157.69e-6 * (h - 11000))

		);
	}
}

############################ update air density ##############################
# FUNCTION updates air density for a weapon given its height above sea level in ft
# the calculated air density is then used to update the drag/lift term 0.5 * rho * Aeff
# at zero lift the drag is equal to the axial force

var updateAirDensity = func (thisWeapon, alt_m) {
	thisWeapon.airDensity = airDensity ( alt_m );
	thisWeapon.lastAlt_m = alt_m;
	thisWeapon.rhoAby2 = 0.5 * thisWeapon.airDensity * thisWeapon.area;
	thisWeapon.speedSound = speedSound (alt_m);
	return();
}

############################ speed sound ##############################
# FUNCTION return speed of sound in mps at rocket altitude (m)
# see https://en.wikipedia.org/wiki/Speed_of_sound

var speedSound = func (h) {
	var speedSoundSeaLevel = 340; # mps
	var speedSound11000m = 295; # mps
	if (h < 11000)
	{
		return
		(
			speedSoundSeaLevel + h / 11000 * (speedSound11000m - speedSoundSeaLevel)
		);
	}
	else
	{
		return ( speedSound11000m );
	}
}

############################ zeroLiftDrag ##############################
# FUNCTION return zero lift drag coefficient at rocket Mach number
# accounts for reduction of drag caused by rocket plume 
# zero lift data from Khalil et al
# DOI: 10.1177/0954410018797882
# Mach number calculated from speed of sound at missile altitude

var zeroLiftDrag = func (thisWeapon, missileSpeed_mps) {
	var intervalData = 0.2;

	if (!thisWeapon.controls.engine)
	{
		var cD_data =
			[
			0.4009,
			0.3878,
			0.3691,
			0.3597,
			0.4,
			0.6803,
			0.6822,
			0.6316,
			0.5791,
			0.5416,
			0.5106,
			0.4853,
			0.4591,
			0.4403,
			0.4216,
			0.4084
			];
	}
	else
	{
		var cD_data =
			[
			0.3222,
			0.3091,
			0.2884,
			0.2809,
			0.3409,
			0.5997,
			0.6016,
			0.5547,
			0.5097,
			0.4759,
			0.4497,
			0.4272,
			0.4103,
			0.3953,
			0.3803,
			0.3653
			];
	}
	var sizeData = size(cD_data) - 1;
	var maxVal = sizeData * intervalData;
	var minVal = 0;
	var mach = missileSpeed_mps / thisWeapon.speedSound;
	if (mach >= maxVal) 
	{
		thisWeapon.cD0 = cD_data[sizeData];
		return (thisWeapon.cD0);
	}
	if (mach <= minVal) 
	{
		thisWeapon.cD0 = cD_data[0];
		return (thisWeapon.cD0);
	}
	var index = mach / intervalData;
	var intIndex = math.floor (index);
	var delta = index - intIndex;
	thisWeapon.cD0 = cD_data[intIndex] * (1 - delta) + cD_data[intIndex+1] * delta;
	return (thisWeapon.cD0);

}

############################ cN ##############################
# FUNCTION calculate coefficient of normal force
# given an angle of attack
# from Eugene Fleeman, Tactical Missile Design

var cN = func( thisWeapon )
{
	var length = 13.0; # multiple of diameter for missile used in Khalil ref
	var diameter = 1.0;
	var alpha = thisWeapon.AoA;
	thisWeapon.cN = math.sin(2.0*alpha) * math.cos(alpha/2.0) + 2.0 * length / diameter * math.sin(alpha) * math.sin(alpha);
	return(thisWeapon.cN);
}


##########################################################
# CLASS stores
# singleton class to hold methods for filling, depleting,
# checking AI aircraft stores, like fuel & weapon rounds
#
var stores = {};

############################ reduceWeaponsCount ##############################
# FUNCTION reduceWeaponsCount
# As the weapons are fired, reduce the count in the AC's stores
#
stores.reduceWeaponsCount = func (myNodeName, elem, time_sec) {

	var lastRound = 0;
	var stos = attributes[myNodeName].stores;
	var ammo_sec = attributes[myNodeName].weapons[elem].ammo_seconds;  #Number of seconds worth of ammo firing the weapon has
	#TODO: This should be set per aircraft per weapon
	if (stos["weapons"][elem] == nil) stos["weapons"][elem] = 0;
	if (stos.weapons[elem] > 0 ) stos.weapons[elem]  -=  time_sec / ammo_sec;
	if (stos.weapons[elem] < 0 ) 
	{
		stos.weapons[elem] = 0;
		lastRound = 1;
	}
	return (lastRound);
}


##########################################################
# FUNCTION reduceFuel
# As the AC attacks, reduce the amount of fuel in the stores
# For now we are just going for amount of time allowed for combat
# since typically fuel use in much higher in that situation.
# TODO: Also account for fuel use while patrolling etc.
#
stores.reduceFuel = func (myNodeName, time_sec) {

	var stos = attributes[myNodeName].stores;
	var fuel_seconds = 600;  #Number of seconds worth of combat time the AC has in
	#fuel reserves.
	#TODO: This should be set per aircraft
	if (stos["fuel"] == nil) stos["fuel"] = 0;
	stos.fuel  -=  time_sec / fuel_seconds;
	if (stos.fuel < 0 ) stos.fuel = 0;
}

###############################################
# FUNCTION fillFuel
#
# fuel is the amount of reserves remaining to carry
# out maneuvers & attacks, not the total fuel
#
#
stores.fillFuel = func (myNodeName,amount = 1){

	if ( ! contains ( attributes, myNodeName) or
	! contains ( attributes[myNodeName], "stores") ) return;
				
	var stos = attributes[myNodeName].stores;
	debprint ("Bombable: Filling fuel for", myNodeName);
	if (stos["fuel"] == nil) stos["fuel"] = 0;
	stos["fuel"] +=  amount;
	if (stos["fuel"] > 1 ) stos["fuel"] = 1;
}

###############################################
# FUNCTION fillWeapons
# each rocket is a separate weapon with a separate AI model - not tied to the parent AC/ship
#
#
stores.fillWeapons = func (myNodeName, amount = 1) {
	if ( ! contains ( attributes, myNodeName) or
	! contains ( attributes[myNodeName], "stores") or
	! contains ( attributes[myNodeName], "weapons") ) return;

	debprint ("Bombable: Filling weapons for", myNodeName);

	var ats = attributes[myNodeName];
	var weaps = ats.weapons;
	var stos = ats.stores;
	var nFixed = 0;
	var nRockets = 0;
	foreach (weap; keys(weaps))
	{
		if (stos["weapons"][weap] == nil) stos["weapons"][weap] = 0;
		stos["weapons"][weap] +=  amount;
		if (stos["weapons"][weap] > 1 ) stos["weapons"][weap] = 1;
		var thisWeapon = weaps[weap];
		if (thisWeapon.weaponType == 1) 
		{
			if (thisWeapon.controls.launched and !thisWeapon.destroyed) 
			{
				guideRocket(-1, myNodeName, weap);
				killRocket(myNodeName, weap);
			}
			thisWeapon.controls.launched = 0;
			thisWeapon.aim.rn = nil;
			nRockets += 1;
		}
		else
		{
			nFixed += thisWeapon.aim.fixed;
		}
		thisWeapon.destroyed = 0;
		thisWeapon.aim.target = -1;
		thisWeapon.aim.weaponDirRefFrame = [0,0,-1];
	}
	if (nRockets) ats.attacks.rocketsInAir = 0; # used to trigger rocket launch
	ats.nRockets = nRockets; # rockets available on platform accounting for those already launched 
	ats.nFixed = nFixed;
	ats.maxTargets = size(keys(weaps)) - (nFixed > 0 ? nFixed : 1) + 1; # the number of targets that the object can fire at simultaneously
}
###############################################
# FUNCTION repairDamage
#
# removes amount from damage
#
stores.repairDamage = func (myNodeName, amount = 0 ) {
	var ats = attributes[myNodeName];
	var damage = ats.damage - amount;
	if (damage > 1) damage = 1 elsif (damage < 0) damage = 0;
	ats.damage = damage;
}

###############################################
# FUNCTION checkWeaponsReadiness
#
# checks if a weapon or all weapons has ammo or not.  returns 1 if ammo, 0
# otherwise.  If elem is given, checks that single weapon, otherwise checks
# all weapons for that object.  Returns 1 if at least one weapon still has
# ammo
#
stores.checkWeaponsReadiness = func (myNodeName, elem = nil) {
	var stos = attributes[myNodeName].stores;
				
	if (elem != nil ) {
		if (stos.weapons[elem] != nil and stos.weapons[elem] == 0 ) return 0;
		else return 1;
		} else {
		foreach (elem;keys (stos.weapons) ) {
			if (stos.weapons[elem] != nil and stos.weapons[elem] > 0 ) return 1;
		}
		return 0;
	}
}

###############################################
# FUNCTION fuelLevel
#
# checks fuel level
# fuel is the amount of reserves remaining to carry
# out maneuvers & attacks, not the total fuel
#
stores.fuelLevel = func (myNodeName) {
	var stos = attributes[myNodeName].stores;
				
	if (stos.fuel != nil) return stos.fuel;
	else return 0;

}

###############################################
# FUNCTION checkAttackReadiness
#
# checks weapons, fuel, damage level, etc, to see if an AI
# should continue to attack or not
#
stores.checkAttackReadiness = func (myNodeName) {
	var ats = attributes[myNodeName];
	var ret = 1;
	var msg = "Bombable: CheckAttackReadiness for  " ~ myNodeName;
	var stos = ats.stores;
	var weaps = ats.weapons;
				
	if (stos["fuel"] != nil and stos.fuel < .2) ret = 0;
	msg ~=  " fuel:"~ stos.fuel;
	var damage = ats.damage;
	if (damage > .8) ret = 0;
	msg ~=  " damage:"~ damage;
				
	#for weapons, if at least 1 weapon has at least 20% ammo we
	# will continue to attack
	var weapret = 0;
	foreach (elem;keys (weaps) ) 
	{
		# if (stos.weapons[elem] != nil and stos.weapons[elem] > .2 ) weapret = 1;
		if (stos.weapons[elem] != nil) 
		{
			weapret += ( stos.weapons[elem] > .2 );
			msg ~=  " "~elem~" "~ stos.weapons[elem];
		}
	}
	if (! weapret) ret = 0;
	#debprint (msg, " Readiness: ", ret);
	if (ret == 0 and ! stos["messages"]["unreadymessageposted"] ) 
	{
		var callsign = getCallSign(myNodeName);
		var popmsg = callsign ~ " is low on weapons/fuel";
		targetStatusPopupTip (popmsg, 10);
		stos["messages"]["unreadymessageposted"] = 1;
		stos["messages"]["readymessageposted"] = 0;
	}
	return ret;
}

####################### revitalizeAttackReadiness ########################
# FUNCTION revitalizeAttackReadiness
#
# After the aircraft has left the attack zone it
# can start to refill weapons, fuel, repair damage etc.
# This function takes care of all that.
#
#
stores.revitalizeAttackReadiness = func (myNodeName,dist_m = 1000000){
	var ats = attributes[myNodeName];
	var atts = ats.attacks;
	var stos = ats.stores;
				
	#We'll say if the object is > .9X the minimum attack
	# distance it can refuel, refill weapons, start to repair
	# damage.
	if (dist_m > atts.maxDistance_m * .9 ) 
	{
		me.repairDamage (myNodeName, .01);
		deleteFire(myNodeName);
		if (ats.damage < .25) deleteSmoke("damagedengine", myNodeName);

		if (stos.fuel > 0.8) return(); # no need to restock
		me.fillFuel (myNodeName, 1);
		me.fillWeapons (myNodeName, 1);
					
		if (! stos["messages"]["readymessageposted"] ) {
			var callsign = getCallSign(myNodeName);
			var popmsg = callsign ~ " has reloaded weapons, fuel, and repaired damage";
			targetStatusPopupTip (popmsg, 10);
			stos["messages"]["unreadymessageposted"] = 0;
			stos["messages"]["readymessageposted"] = 1;
		}
		debprint ("Bombable: Revitalizing attack readiness for ", myNodeName);
	}
				
}

#END CLASS stores
###############################################
			

##################### elevGround ##########################
#returns ground height in m at myNodeName position
# works for any aircraft; for main aircraft myNodeName = ""
var elevGround = func (myNodeName) {
	var lat = getprop(""~myNodeName~"/position/latitude-deg");
	var lon = getprop(""~myNodeName~"/position/longitude-deg");
	var elev = geo.elevation(lat, lon);
	if (elev == nil) debprint("elevGround for ", myNodeName, " lat= ", lat, " lon= ", lon); # error debug 
	return geo.elevation(lat, lon);
}

##################### course1to2 ##########################
# returns a hash containing the distance between AI objects 1 and 2
# and the heading (absolute bearing) for 1 to travel to 2
# replaces calls to geo methods (4-5x faster)

var course1to2 = func (myNodeName1, myNodeName2) {
	# weapons_loop also reads the co-ords from the prop tree and stores them in the attributes hash
	var lat1 = getprop(""~myNodeName1~"/position/latitude-deg");
	var lon1 = getprop(""~myNodeName1~"/position/longitude-deg");
	var alt1 = getprop(""~myNodeName1~"/position/altitude-ft");
	var lat2 = getprop(""~myNodeName2~"/position/latitude-deg");
	var lon2 = getprop(""~myNodeName2~"/position/longitude-deg");
	var alt2 = getprop(""~myNodeName2~"/position/altitude-ft");
	var dx = (lon2 - lon1) * m_per_deg_lon;
	var dy = (lat2 - lat1) * m_per_deg_lat;
	var dz = (alt2 - alt1) * FT2M;
	var intercept = findIntercept3(myNodeName2, [dx, dy, dz], attributes[myNodeName1].velocities.attackSpeed_kt * KT2MPS);
	var hdg = (intercept.time < 0) ?
		math.atan2( dx, dy) * R2D :
		math.atan2 ( intercept.vector[0], intercept.vector[1]) * R2D ;
	# debprint (sprintf( "intercept hdg %6.1f intercept time %6.1f", hdg, intercept.time ));
	if (hdg < 0) hdg += 360;
	var dist_xy = math.sqrt (dx * dx + dy * dy);
	return
	{
		distance : [dist_xy, -dz], # minus for comparison with original code
		heading : hdg,
	}
}


######################### attack_loop ###########################
# Main loop for calculating attacks, changing direction, altitude, etc.
#

var attack_loop = func ( id, myNodeName ) {
	var ats = attributes[myNodeName];
	id == ats.loopids.attack_loopid or return;
	var ctrls = ats.controls;	
	var atts = ats.attacks;
				
	#debprint ("attack_loop starting");

	#skill ranges 0-6
	var skill = calcPilotSkill (myNodeName);
	var skillMult = (skill <= .2) ? 15 : 3/skill;
	var loopTimeActual = skillMult * atts.loopTime;

	#Higher skill makes the AI pilot react faster/more often:
	settimer ( func { attack_loop ( id, myNodeName ) }, loopTimeActual );
				
	if ( ! bombableMenu["ai-aircraft-attack-enabled"] or ! bombableMenu["bombable-enabled"] or 
		(ats.damage == 1) ) 
	{
		atts.loopTime = atts.attackCheckTime_sec;
		return;
	}
				
	# dodging takes priority over attacking
	if (ctrls.dodgeInProgress) return;

	# if (ats.targetIndex == -1) return; # no target assigned
	# var targetNode = nodes[ats.targetIndex];	

	# set the node to attack
	var targetNode = "";
	var addTrue = "";
	if (size(ats.targetIndex))
	{
		targetNode = nodes[ats.targetIndex[0]];
		addTrue = "true-";
	}

	var alts = ats.altitudes;

	var distHdg = course1to2 (myNodeName, targetNode); # returns a hash
	var dist = distHdg.distance;
	var courseToTarget_deg = distHdg.heading; # absolute bearing
	
	#debprint ("Bombable: Checking attack parameters: ", dist[0], " ", atts.maxDistance_m, " ",atts.minDistance_m, " ",dist[1], " ",-atts.altitudeLowerCutoff_m, " ",dist[1] < atts.altitudeHigherCutoff_m );
	if (ctrls.stayInFormation)
	{
		if ( dist[0] > atts.maxDistance_m or rand() < .1) return; # if check time 10s then will attack within 1s of entering maxDist perimeter
		var msg = getCallSign(myNodeName)~" breaking formation";
		targetStatusPopupTip (msg, 5);
		# debprint ("Bombable: "~msg); 
		ctrls.stayInFormation = 0; 			
	}
				
	var myHeading_deg = getprop ("" ~ myNodeName ~ "/orientation/true-heading-deg");
	var deltaHeading_deg = myHeading_deg - courseToTarget_deg;
	deltaHeading_deg = math.mod (deltaHeading_deg + 180, 360) - 180;
	var targetHeading_deg = getprop("" ~ targetNode ~ "/orientation/" ~ addTrue ~ "heading-deg");
				
	# whether or not to continue the attack when within minDistance:
	# If we are heading towards the target aircraft
	# (within continueAttackAngle_deg of straight on) we continue 
	# or if we are still way below or above the target,
	# we continue the attack
	# otherwise we break off the attack/evade
	var continueAttack = 0;
	if ( dist[0] < atts.minDistance_m ) 
	{
		var newAltLowerCutoff_m = atts.altitudeLowerCutoff_m / 4; # how much lower the shooter is than the target
		if (newAltLowerCutoff_m < 150) newAltLowerCutoff_m = 200;
		var newAltHigherCutoff_m = atts.altitudeHigherCutoff_m / 4;
		if (newAltHigherCutoff_m < 150) newAltHigherCutoff_m = 200;
					
		if (math.abs ( deltaHeading_deg ) < atts.continueAttackAngle_deg
		or dist[1] < -newAltLowerCutoff_m or dist[1] > newAltHigherCutoff_m ) continueAttack = 1; # dist[1] is alt shooter - alt target
	}
				
	# readiness = 0 means AC has little fuel or ammo left.  It will cease
	# attacking UNLESS the target comes very close by & attacks it.
	# However there is no point in attacking if no ammo at all, in that
	# case only dodging/evading will happen.
	var readinessAttack = 1;
	var attentionFactor = 1;
	var distanceFactor = 1;
	
	var gotAmmo = stores.checkWeaponsReadiness(myNodeName);
	if (!gotAmmo and (ctrls.kamikase == 1))
	{
		ctrls.kamikase = -1;
		atts.minDistance_m = 0;
	}

	if (ctrls.kamikase != -1) # if kamikase we will attack even if fuel is low; we are certainly paying attention
	{
		# readiness
		if ( ! stores.checkAttackReadiness(myNodeName) ) 
		{
			var newMaxDist_m = atts.maxDistance_m/8;
			if (newMaxDist_m < atts.minDistance_m) newMaxDist_m = atts.minDistance_m * 1.5;
						
			readinessAttack = ( dist[0] < newMaxDist_m ) and ( dist[0] > atts.minDistance_m or continueAttack ) and 
			(dist[1] > -atts.altitudeLowerCutoff_m/3) and (dist[1] < atts.altitudeHigherCutoff_m/3) and gotAmmo;
		}
		
		# attention
		# OK, we spend 13% of our time zoning out.  http://discovermagazine.com/2009/jul-aug/15-brain-stop-paying-attention-zoning-out-crucial-mental-state
		# Or maybe we are distracted by some other task or whatever.  At any rate,
		# this is a human factor for the possibility that they could have observed/
		# attacked in this situation, but didn't.  We'll assume more skilled
		# pilots are more observant and less distracted.  We'll assume 13% of the time
		# is the average.
		# Attention is presumably much higher during an attack but since this loop
		# runs much more often during an attack that should cancel out.  Plus there
		# might be other distractions during an attack, even if not so much
		# daydreaming.
		# This only applies to the start of an attack.  Once attacking, presumably
		# we are paying enough attention to continue.
		if (rand() < 0.2 - skill / 50) attentionFactor = 0;

		# target sighted?		
		# The further away we are, the less likely to notice the target and start
		# an attack.
		# This only applies to the start of an attack.  Once attacking, presumably
		# we are paying enough attention to continue.
		if (rand() < dist[0] / atts.maxDistance_m) distanceFactor = 0;
		if (dist[1] < 0) 
		{
			if  (rand() < -dist[1]/atts.altitudeLowerCutoff_m)  distanceFactor = 0;
		}
		else
		{
			if (rand() < dist[1]/atts.altitudeHigherCutoff_m)  distanceFactor = 0;
		}
	}
	#TODO: Other factors could be added here, like less likely to attack if
	#    behind a cloud, more likely if rest of squadron is, etc.
				
	var attack_inprogress = ctrls.attackInProgress;
				
	# criteria for not attacking
	# if we fail to meet any of these criteria we do a few things then exit without attacking. Logic: not (A and B) = not A or not B 
	# debprint ("Bombable: Attack criteria: ", (dist[0] < atts.maxDistance_m ), " ", ( dist[0] > atts.minDistance_m or continueAttack ) , " ",
	# (dist[1] > -atts.altitudeLowerCutoff_m), " ", (dist[1] < atts.altitudeHigherCutoff_m), " ",  
	# readinessAttack, " ", ( (attentionFactor and distanceFactor) or attack_inprogress ), " for ", myNodeName, " ");
	
	if ( ! (( dist[0] < atts.maxDistance_m ) and ( dist[0] > atts.minDistance_m or continueAttack ) and
	(dist[1] > -atts.altitudeLowerCutoff_m) and (dist[1] < atts.altitudeHigherCutoff_m)  and  
	readinessAttack and ( (attentionFactor and distanceFactor) or attack_inprogress ) ) )  
	{
		# debprint ("Bombable: Not attacking ", continueAttack, " ", readinessAttack, " ", attentionFactor, " ", distanceFactor, " ", attack_inprogress, " for ", myNodeName, " " );
		#OK, no attack, we're too far away or too close & passed it, too low, too high, etc etc etc
		#Instead we: 1. dodge if necessary 2. exit
		#always dodge when close to Target aircraft--unless we're aiming at it
		#ie, after passing it by, we make a dodge.  Less skilled pilots dodge less often.
					
		# are we ahead of or behind the target AC?  If behind, there is little point
		# in dodging.  aheadBehindTarget_deg will be 0 degrees if we're directly
		# behind, 90 deg if directly to the side.  
		# We dodge only if > 110 degrees, which puts us pretty much in frontish.
					
		if (dist[0] < atts.minDistance_m)
		{
			var aheadBehindTarget_deg = normdeg180 (targetHeading_deg - courseToTarget_deg);
		 	if (rand() < skill/5 and math.abs(aheadBehindTarget_deg) > 110) dodge(myNodeName);
		}
		# When the AC is done attacking & dodging it will continue to fly in
		# circles unless we do this
		elsif (attack_inprogress or rand() < 0.1) setprop (""~myNodeName~"/controls/flight/target-roll", rand() * 2 - 1); # rjw reduced frequency

		#If not attacking, every once in a while we turn the AI AC in the general
		#direction of the target
		#This is to keep the AI AC from getting too dispersed.
		#TODO: We could do lots of things here, like have the AC join up in squadrons,
		#return to a certain staging area, patrol a certain area, or whatever.

		if (rand() < ((ctrls.kamikase == -1) ? 0.5 : 0.2)) 
		# if attack check time 5 sec then < 0.2 gives an average delay of 25 sec til AC turns back into the fray
		# 10 sec for kamikase pilots 
		{
			ctrls.courseToTarget_deg = courseToTarget_deg;
			var whereNow = "target";
			if (!ats.side)
			{
				# check distance from main AC. 
				# (0), the defending side, stays close to main AC
				# keeps action close to main AC, which might be a neutral observer
				var distHdgMain = course1to2 (myNodeName, "");
				if ( distHdgMain.distance[0] > atts.maxDistance_m / 4 )
				{
					ctrls.courseToTarget_deg = distHdgMain.heading;
					whereNow = "main AC";
				}
			}
			aircraftTurnToHeading ( myNodeName, 60 );
			debprint ("Bombable: ", myNodeName, " Turning in direction of " ~ whereNow);
		}
		
		if ( dist[0] > atts.maxDistance_m ) stores.revitalizeAttackReadiness(myNodeName, dist[0]);

		atts.loopTime = atts.attackCheckTime_sec;
		ctrls.attackInProgress = 0;
		if (attack_inprogress) debprint ("Bombable: End of attack for ", myNodeName);
		return;
	}
				
				
	#ATTACK
	#
	# (1-rand() * rand()) makes it choose values at the higher end of the range more often
				
	stores.reduceFuel (myNodeName, loopTimeActual ); #deduct the amount of fuel from the tank
	
	var attackCheckTimeEngaged_sec = atts.attackCheckTimeEngaged_sec;
	var roll_deg = (1 - rand() * rand()) * (atts.rollMax_deg - atts.rollMin_deg) + atts.rollMin_deg;
				
	#debprint ("rolldeg:", roll_deg);
				
	#if we are aiming almost at our target we reduce the roll if we are
	#close to aiming at them
	if (roll_deg > 4 * math.abs(deltaHeading_deg))
	{
		roll_deg = 4 * math.abs(deltaHeading_deg);
	}
				
				
	#Easy mode makes the attack manuevers less aggressive
	#if (skill == 2) roll_deg *= 0.9;
	#if (skill == 1) roll_deg *= 0.8;
				
	#reduce the roll according to skill
	roll_deg *= (skill + 6) / 12;
				
	#debprint ("rolldeg:", roll_deg);
				
	ctrls.courseToTarget_deg = courseToTarget_deg + (rand() * 8 - 4) * skillMult; 
	#keeps the moves from being so robotic and makes the lower skilled AI pilots less able to aim for the Target aircraft
	#if skillMult 0.5 to 15 up to 120 degree error??? Reduced by 2

	#it turns out that the main AC's AGL is available in the prop tree, which is
	#far quicker to access then the elev function, which is very slow
	#elevTarget_m = elev (geo.aircraft_position().lat(),geo.aircraft_position().lon() ) * FT2M;
	#targetAGL_m = targetAlt_m-elevTarget_m;
				
	var targetAlt_m = getprop (targetNode ~ "/position/altitude-ft") * FT2M;
	if (targetNode == "")
	{
		var targetAGL_m = getprop ("/position/altitude-agl-ft") * FT2M;
		var elevTarget_m = targetAlt_m - targetAGL_m; # height of ground at main AC position
	}
	else
	{
		var elevTarget_m = elevGround ( targetNode ) ;
		var targetAGL_m = targetAlt_m - elevTarget_m;
	}
	var currAlt_m = getprop(""~myNodeName~"/position/altitude-ft") * FT2M;

	ctrls.attackInProgress = 1;

	# is this the start of our attack?  If so, or if we're heading away from the
	# target, we'll possibly do a loop or strong altitude move
	# to get turned around, and continue that until we are closer than 90 degrees
	# in heading delta

	# debprint( myNodeName, "continue attack: ", continueAttack, "readiness attack: ", readinessAttack );
	# debprint
	# (
	# 	sprintf
	# 	(
	# 		"deltaHeading_deg %5d attackClimbDiveInProgress %d attackClimbDiveTargetAGL_m %d", 
	# 		deltaHeading_deg, 
	# 		ctrls.attackClimbDiveInProgress, 
	# 		ctrls.attackClimbDiveTargetAGL_m
	# 	)
	# );			

	if ( !attack_inprogress or math.abs ( deltaHeading_deg ) >= 90 ) # rjw the second term allows the target to evade attack by a sharp 90 degree turn
	{
		# if we've already started an attack loop, keep doing it with the same
		# targetAGL, unless we have arrived within 500 meters of that elevation
		# already.  Also we randomly pick a new targetaltitude every so often
		if 
		(
			ctrls.attackClimbDiveInProgress and ctrls.attackClimbDiveTargetAGL_m > 0 and 
			math.abs(ctrls.attackClimbDiveTargetAGL_m + elevTarget_m - currAlt_m) > 500 and
			(rand() > 0.005 * skill)
		) 
		{
			targetAGL_m = ctrls.attackClimbDiveTargetAGL_m;
			# debprint ("Bombable: Continuing attack for ", myNodeName," targetAGL_m = ", targetAGL_m);
		} 
		else
		{
			# otherwise, we are starting a new attack so we need to figure out what to do

			# if we're skilled and we have enough speed we'll do a loop to get in better position
			# more skilled pilots do acrobatics more often
			# in the Zero 130 kt is about the minimum speed needed to
			# complete a loop without stalling.
			# TODO: This varies by AC.  As a first try we're going with 2X
			# minSpeed_kt to complete the loop.
			#
			# debprint ("Bombable: Starting attack for " ~ getCallSign (myNodeName) );
			vels = attributes[myNodeName].velocities;
			var currSpeed_kt = getprop (""~myNodeName~"/velocities/true-airspeed-kt");
			if (currSpeed_kt > 2.2 * vels.minSpeed_kt and rand() < (skill+8)/15 and (atts.allGround ? rand() < 0.2 : 1)) 
			{
				if ( choose_attack_acrobatic(myNodeName, dist[0], myHeading_deg,
				targetHeading_deg, deltaHeading_deg,
				currSpeed_kt, skill, currAlt_m, targetAlt_m, elevTarget_m))
				return;
			}
						
			# we want to mostly dodge to upper/lower extremes of our altitude limits
			var attackClimbDiveAddFact = 1 - rand() * rand() * rand();
			# worse pilots don't dodge as far
			attackClimbDiveAddFact *= (skill+3)/9;
			# the direction of the Alt dodge will favor the direction that has more
			# feet to dodge in the evasions definitions.  Some aircraft heavily favor
			# diving to escape, for instance.
			#
						
			# climb or dive more according to the aircraft's capabilities.
			# However note that by itself this will lead the AI AC to climb/dive
			# away from the Target AC unless climbPower & divePower are equal.  So we
			# mediate this by adjusting if it gets too far above/below the Target AC
			# altitude (see below))
			var attackClimbDiveAddDirection = rand() * (atts.climbPower + atts.divePower) - atts.divePower;
						
			# for this purpose we use 50/50 climbs & dives because using a different
			# proportion tends to put the aircraft way above or below the Target aircraft
			# over time, by a larger amount than they can correct in the reTargeting
			# part of their attack pattern.
			#var attackClimbDiveAddDirection = 2 * rand()-1;
						
						
			# if we're too high or too low compared with Target AC then we'll climb or
			# dive towards it always.  This prevents aircraft from accidentally
			# climbing/diving away from the Target AC too much.
			var deltaAlt_m = currAlt_m - targetAlt_m;
			if (deltaAlt_m > 0) 
			{
				if ( deltaAlt_m > atts.divePower/6 ) attackClimbDiveAddDirection = -1;
			}
			else
			{
				if ( -deltaAlt_m > atts.climbPower/6 ) attackClimbDiveAddDirection = 1;
			}
						
			# target amount to climb or drop
			# for FG's AI to make a good dive/climb the difference in altitude must be at least 5000 ft
			if ( ats.nRockets )
			{
				var attackClimbDiveAdd_m = 0; # do not want to match height of target if launching rockets, particularly gliders
			}
			else
			{
				var attackClimbDiveAdd_m = (attackClimbDiveAddDirection >= 0) ? attackClimbDiveAddFact * atts.climbPower : -attackClimbDiveAddFact * atts.divePower ;
			}
						
						
			targetAGL_m = currAlt_m + attackClimbDiveAdd_m - elevTarget_m;
						
			if (targetAGL_m < alts.minimumAGL_m) targetAGL_m = alts.minimumAGL_m;
			if (!atts.allGround)
			{
				if (targetAGL_m > alts.maximumAGL_m) targetAGL_m = alts.maximumAGL_m;
			}
			else
			{
				if (targetAGL_m > 1000) targetAGL_m = 1000; # if all targets are on the ground (or sea) then do not exceed 1000m AGL
			}						
			# debprint ("Bombable: Starting attack turn/loop for ", myNodeName," targetAGL_m = ", targetAGL_m);
			ctrls.attackClimbDiveInProgress = 1;
			ctrls.attackClimbDiveTargetAGL_m = targetAGL_m;
		}
	} 
	else
	{
		ctrls.attackClimbDiveInProgress = 0;
	}
				
	targetAlt_m = targetAGL_m + elevTarget_m;
				
	if (targetAGL_m < alts.minimumAGL_m ) targetAGL_m = alts.minimumAGL_m;
	if (targetAGL_m > alts.maximumAGL_m ) targetAGL_m = alts.maximumAGL_m;
				
	# rjw commented out since not used by aircraftTurnToHeading
	# 
	# sometimes when the deltaheading is near 180 degrees we turn the opposite way of normal
	#
	# var favor = getprop(""~myNodeName~"/bombable/favor-direction");
	# if ((favor != "normal" and favor != "opposite") or rand() < .003) {
	# 	var favor = "normal"; if (rand() > .5) favor = "opposite";
	# 	setprop(""~myNodeName~"/bombable/favor-direction", favor);
	# }
				
	aircraftSetVertSpeed (myNodeName, targetAlt_m - currAlt_m, "evas" );

	# turn to heading is called too frequently (t = 0.5 sec) for 0.1 s update time.  It aborts any existing turn.  However the AI AC is attacking so must track the target			
	if (rand() < 0.2) aircraftTurnToHeading ( myNodeName, roll_deg, targetAlt_m );
				
				
	#update more frequently when engaged with the main aircraft
	atts.loopTime = attackCheckTimeEngaged_sec;
				
				
}
			

################### aircraftSetVertSpeed ####################

var aircraftSetVertSpeed = func (myNodeName, dodgeAltAmount_ft, evasORatts = "evas") {

	var vels = attributes[myNodeName].velocities;
	var evas = attributes[myNodeName].evasions;
				
	var divAmt = 8;
	if (evasORatts == "atts") divAmt = 4;
				
	var dodgeVertSpeed_fps = 0;
	if ( dodgeAltAmount_ft > 150 )  dodgeVertSpeed_fps = math.abs ( evas.dodgeVertSpeedClimb_fps );
	elsif ( dodgeAltAmount_ft > 100 )  dodgeVertSpeed_fps = math.abs ( evas.dodgeVertSpeedClimb_fps/divAmt * 4 );
	elsif ( dodgeAltAmount_ft > 75 )  dodgeVertSpeed_fps = math.abs ( evas.dodgeVertSpeedClimb_fps/divAmt * 3 );
	elsif ( dodgeAltAmount_ft > 50 )  dodgeVertSpeed_fps = math.abs ( evas.dodgeVertSpeedClimb_fps/divAmt * 2 );
	elsif ( dodgeAltAmount_ft > 25 )  dodgeVertSpeed_fps = math.abs ( evas.dodgeVertSpeedClimb_fps/divAmt );
	elsif ( dodgeAltAmount_ft > 12.5 )  dodgeVertSpeed_fps = math.abs ( evas.dodgeVertSpeedClimb_fps/divAmt/2 );
	elsif ( dodgeAltAmount_ft > 6 )  dodgeVertSpeed_fps = math.abs ( evas.dodgeVertSpeedClimb_fps/divAmt/3 );
	elsif ( dodgeAltAmount_ft > 0 )  dodgeVertSpeed_fps = math.abs ( evas.dodgeVertSpeedClimb_fps/divAmt/5 );
	elsif  ( dodgeAltAmount_ft < -150 )  dodgeVertSpeed_fps = - math.abs ( evas.dodgeVertSpeedDive_fps);
	elsif  ( dodgeAltAmount_ft < -100 )  dodgeVertSpeed_fps = - math.abs ( evas.dodgeVertSpeedDive_fps/divAmt * 4);
	elsif  ( dodgeAltAmount_ft < -75 )  dodgeVertSpeed_fps = - math.abs ( evas.dodgeVertSpeedDive_fps/divAmt * 3);
	elsif  ( dodgeAltAmount_ft < -50 )  dodgeVertSpeed_fps = - math.abs ( evas.dodgeVertSpeedDive_fps/divAmt * 2);
	elsif  ( dodgeAltAmount_ft < -25 )  dodgeVertSpeed_fps = - math.abs ( evas.dodgeVertSpeedDive_fps/divAmt);
	elsif  ( dodgeAltAmount_ft < -12.5 )  dodgeVertSpeed_fps = - math.abs ( evas.dodgeVertSpeedDive_fps/divAmt/2);
	elsif  ( dodgeAltAmount_ft < -6 )  dodgeVertSpeed_fps = - math.abs ( evas.dodgeVertSpeedDive_fps/divAmt/3);
	elsif  ( dodgeAltAmount_ft < 0 )  dodgeVertSpeed_fps = - math.abs ( evas.dodgeVertSpeedDive_fps/divAmt/5);
				
	#for evasions, the size & speed of the vertical dive is proportional
	# to the amount of dodgeAlt selected.  For atts & climbs it makes more
	# sense to just do max climb/dive until close to the target alt
	if (evasORatts == "evas") 
	{
		if ( dodgeAltAmount_ft < 0 )  dodgeVertSpeed_fps = - math.abs ( evas.dodgeVertSpeedDive_fps * dodgeAltAmount_ft/evas.dodgeAltMin_ft );
	}
				
				
	# If we want a change in vertical speed then we are going to change /velocities/vertical-speed-fps
	# directly.  But by a max of 25 FPS at a time, otherwise it is too abrupt.
	if (dodgeVertSpeed_fps != 0)
	{
		#proportion the amount of vertical speed possible by our current speed
		# stops unreasonably large vertical speeds from happening
		dodgeVertSpeed_fps *= (getprop(""~myNodeName~"/velocities/true-airspeed-kt")-vels.minSpeed_kt)/(vels.maxSpeed_kt-vels.minSpeed_kt);
		var curr_vertical_speed_fps = getprop ("" ~ myNodeName ~ "/velocities/vertical-speed-fps");
		vertSpeedChange_fps = dodgeVertSpeed_fps - curr_vertical_speed_fps;
		if (vertSpeedChange_fps > 25) vertSpeedChange_fps = 25;
		if (vertSpeedChange_fps < -25) vertSpeedChange_fps = -25;
		var stalling = attributes[myNodeName].controls.stalling;
					
		#don't do this if we are stalling, except if it makes us fall faster
		if (!stalling or vertSpeedChange_fps < 0) setprop ("" ~ myNodeName ~ "/velocities/vertical-speed-fps", curr_vertical_speed_fps + vertSpeedChange_fps);
		#debprint ("VertSpdChange: ", myNodeName, dodgeAltAmount_ft, dodgeVertSpeed_fps, "vertspeedchange:", vertSpeedChange_fps);
	}

}

################### aircraftTurnToHeadingControl ####################
# for making AI aircraft turn to certain heading
# called by aircraftTurnToHeading
# rolldegrees is the maximum bank angle - always positive

var aircraftTurnToHeadingControl = func (myNodeName, id, rolldegrees = 45, targetAlt_m = "none" ,  roll_limit_deg = 85, correction = 0 ) {
	id == attributes[myNodeName].loopids.roll_loopid or return;
	if (!bombableMenu["bombable-enabled"] ) return;

	var updateinterval_sec = .1;
	var maxTurnTime = 60; #max time to stay in this loop/a failsafe
	var atts = attributes[myNodeName].attacks;
	var ctrls = attributes[myNodeName].controls;			
	var start_heading_deg = getprop (""~myNodeName~"/orientation/true-heading-deg"); # an absolute bearing
	var delta_heading_deg = ctrls.courseToTarget_deg - start_heading_deg;
	while ( delta_heading_deg < 0 ) delta_heading_deg  +=  360; #same as norm180
	if (delta_heading_deg > 180) delta_heading_deg  +=  -360;
	var delta_deg = math.sgn (delta_heading_deg) * atts.rollRateMax_degpersec * updateinterval_sec;
	var targetRoll_deg = ctrls.roll_deg_bombable + delta_deg;

	#Fg turns too quickly to be believable if the roll gets about 78 degrees or so.
	# rolldegrees limits the max roll allowed for this manoeuvre
	
	var dir = math.sgn (targetRoll_deg);
	if (targetRoll_deg * dir > rolldegrees) targetRoll_deg = rolldegrees * dir;
	setprop (""~myNodeName~ "/orientation/roll-deg", targetRoll_deg);
	ctrls.roll_deg_bombable = targetRoll_deg;
				
	# debprint ("Bombable: Setting roll-deg for ", myNodeName , " to ", targetRoll_deg);
				
	#set the target altitude as well.  flight/target-alt is in ft
	if (targetAlt_m != "none") 
	{
		var evas = attributes[myNodeName].evasions;
		var alts = attributes[myNodeName].altitudes;
					
		targetAlt_ft = targetAlt_m * M2FT;
		var currElev_m = elevGround (myNodeName);
		if (targetAlt_m - currElev_m < alts.minimumAGL_m ) targetAlt_ft = (alts.minimumAGL_m + currElev_m) * M2FT
		elsif (targetAlt_m - currElev_m > alts.maximumAGL_m ) targetAlt_ft = (alts.maximumAGL_m + currElev_m) * M2FT;
					
		# we set the target altitude, unless we are stalling and trying to move higher,
		# then we stop moving up
		if (!ctrls.stalling or targetAlt_m < currElev_m )
		{
			setprop ( "" ~ myNodeName ~ "/controls/flight/target-alt", targetAlt_ft );
		}
		else 
		{
			setprop ("" ~ myNodeName ~ "/controls/flight/target-alt", currElev_m * M2FT - 20 );
		}
					
		var currAlt_ft = getprop ("" ~ myNodeName ~ "/position/altitude-ft");
		var dodgeAltAmount_ft = targetAlt_ft - currAlt_ft;
		if (dodgeAltAmount_ft > evas.dodgeAltMax_ft) dodgeAltAmount_ft = evas.dodgeAltMax_ft
		elsif (dodgeAltAmount_ft < evas.dodgeAltMin_ft) dodgeAltAmount_ft = evas.dodgeAltMin_ft;
					
		aircraftSetVertSpeed (myNodeName, dodgeAltAmount_ft, "atts");

		# debprint ("Attacking: Change height for", myNodeName, " by ", dodgeAltAmount_ft);
	}
				
		# debprint(sprintf(
		# 	"Bombable: RollControl: delta = %3.1fdeg, target roll = %3.1fdeg, delta hdg = %4.1fdeg, %s",
		# 	delta_deg,
		# 	targetRoll_deg,
		# 	delta_heading_deg,
		# 	myNodeName
		# 	)
		# );

	var rollTimeElapsed = ctrls.rollTimeElapsed;
	var cutoff = rolldegrees / 5;
	if (cutoff < 1) cutoff = 1;
	# wait a while & then roll back.  correction makes sure we don't keep
	# doing this repeatedly
	if ( math.abs(delta_heading_deg) > cutoff and rollTimeElapsed < maxTurnTime ) 
	{
		ctrls.rollTimeElapsed = rollTimeElapsed + updateinterval_sec;
		settimer (func { aircraftTurnToHeadingControl(myNodeName, id, rolldegrees, targetAlt_m )}, updateinterval_sec );
	}
	else 
	{
		ctrls.rollTimeElapsed = 0;
		# debprint ("Bombable: Ending aircraft turn-to-heading routine for " ~ myNodeName);
		# aircraftRoll(myNodeName, 0, rolltime, roll_limit_deg);
		# rjw not needed since AI model flight lateral-mode control in "roll" - not "hdg" ?
		# setprop(""~myNodeName~"/controls/flight/target-hdg", targetdegrees);
	}
}

########################## aircraftTurnToHeading ############################
# make an aircraft turn to a certain heading
#
# called by attack_loop every t = attackCheckTimeEngaged_sec (too often)


var aircraftTurnToHeading = func (myNodeName, rolldegrees = 45, targetAlt_m = "none" ) {
	# same as roll-loopid ID because we can't turn to heading & roll @ the same time
	# rjw how to avoid function clash?
	var loopid = inc_loopid( myNodeName, "roll" );
	var ctrls = attributes[myNodeName].controls;
	ctrls.rollTimeElapsed = 0;
				
	var	currRoll_deg = getprop (""~myNodeName~ "/orientation/roll-deg");
	ctrls.roll_deg_bombable = currRoll_deg;
	
	# max roll angle = 75 - 85, more than this FG AI goes a bit wacky
	# depends on the aircraft/speed/etc so we let the individual aircraft set it individually
	var roll_limit_deg = attributes[myNodeName].attacks.rollMax_deg;
	var rolldegrees_ = (rolldegrees > roll_limit_deg) ? roll_limit_deg : rolldegrees;


	# rjw commented out by bhugh - if aircraftTurnToHeading called several times to set direction then will flip-flop
	# favor removed from function arguments

	# if close to 180 degrees off we sometimes/randomly choose to turn the
	# opposite direction.  Just for variety/reduce robotic-ness.
	# var start_heading_deg = getprop (""~myNodeName~"/orientation/true-heading-deg");
	# var delta_heading_deg = normdeg180(targetdegrees - start_heading_deg);
	# if (math.abs(delta_heading_deg) > 150 and favor == "opposite") {
		#   targetdegrees = start_heading_deg-delta_heading_deg;
	#   }

	aircraftTurnToHeadingControl ( myNodeName, loopid, rolldegrees_, targetAlt_m );
				
	# debprint (sprintf("Bombable: Starting turn-to-heading routine for %s, loopid= %d, rolldegrees= %3.1f, course_deg= %3.1f",
	# myNodeName, loopid, rolldegrees_, ctrls.courseToTarget_deg));
}


################### aircraftRollControl ###################
# rjw: function called by timer to progress the roll of the aircraft
# internal - for making AI aircraft roll/turn
# rolldegrees means the absolute roll degrees to move to, from whatever
# rolldegrees the AC currently is at.
var aircraftRollControl = func (myNodeName, id, rolldegrees, rolltime, roll_limit_deg, 
delta_deg, delta_t) 
{
	id == attributes[myNodeName].loopids.roll_loopid or return;
	if (!bombableMenu["bombable-enabled"] ) return;
				
	var atts = attributes[myNodeName].attacks;
	var ctrls = attributes[myNodeName].controls;
	#At a certain roll degrees aircraft behave very unrealistically--turning
	#far too fast etc. This is somewhat per aircraft and per velocity, but generally
	#anything more than 85 degrees just makes them turn on a dime rather
	#than realistically. 90 degrees is basically instant turn, so we're going
	# to disallow that, but allow anything up to that.
				

	var targetRoll_deg = ctrls.roll_deg_bombable + delta_deg;

	#Fg turns too quickly to be believable if the roll gets about 78 degrees or so.
	# rolldegrees limits the max roll allowed for this manoeuvre
	# bombable keeps the 'uncorrected' amount because normal behavior
	# is to go to a certain degree & then return.  If we capped at 85 deg
	# then we would end up returning too far
	
	var dir = math.sgn (targetRoll_deg);
	var rollMax_deg = atts.rollMax_deg;
	if (targetRoll_deg * dir > roll_limit_deg) targetRoll_deg = roll_limit_deg * dir;
	if (targetRoll_deg * dir > rollMax_deg) targetRoll_deg = rollMax_deg * dir;

	
	setprop (""~myNodeName~ "/orientation/roll-deg", targetRoll_deg);
	ctrls.roll_deg_bombable = targetRoll_deg;
				
	#debprint("Bombable: RollControl: delta = ",delta_deg, " ",targetRoll_deg," ", myNodeName);
	
	var rollTimeElapsed = ctrls.rollTimeElapsed;
				
	if ( rollTimeElapsed < rolltime )
	{
		ctrls.rollTimeElapsed = rollTimeElapsed + delta_t;
		settimer (func 
		{ 
			aircraftRollControl(myNodeName, id, rolldegrees, rolltime, roll_limit_deg, delta_deg, delta_t)
		}, 
		delta_t );
	}
	else 
	{
		ctrls.rollTimeElapsed = 0;
		debprint ("Bombable: Ending aircraft roll routine");
	}
}

################################## aircraftRoll ################################
# Will roll the AC from whatever roll deg it is at, to rolldegrees in rolltime
# Initialises aircraftRollControl
var aircraftRoll = func (myNodeName, rolldegrees = -60, rolltime = 5, roll_limit_deg = 85) {
	var loopid = inc_loopid( myNodeName, "roll" );
	var updateinterval_sec = .1;
	var ctrls = attributes[myNodeName].controls;
	ctrls.rollTimeElapsed = 0;
	var currRoll_deg = getprop (""~myNodeName~ "/orientation/roll-deg");
	ctrls.roll_deg_bombable = currRoll_deg;
	if (math.abs(rolldegrees) >= 90 ) rolldegrees = 88 * math.sgn(rolldegrees);
	if (rolltime < updateinterval_sec) rolltime = updateinterval_sec;
	var delta_deg = ( rolldegrees - currRoll_deg ) * updateinterval_sec / rolltime;

	aircraftRollControl(myNodeName, loopid, rolldegrees, rolltime, roll_limit_deg, delta_deg, updateinterval_sec);
				
	debprint (sprintf("Bombable: Starting roll routine, loopid = %d rolldegrees = %6.1f rolltime = %5.1f for %s",loopid, rolldegrees, rolltime, myNodeName));
}

################################# aircraftCrashControl #################################
# rjw: the aircraft crashes progressively, i.e. falls/glides to earth
# rjw: the aircraft descent is either a powered dive or unpowered glide with the AC air speed potentially increasing or decreasing
# See http://www.dept.aoe.vt.edu/~lutze/AOE3104/glidingflight.pdf
# Could measure dive speeds for different engine rpm and pitch angle for the aircraft model and include as attribute?
#
# Using initial airspeed + x ft/sec as the terminal velocity (x <= maxVertSpeed) & running this loop ~2X per second
# elapsed measures the time elapsed since the aircraft first 'crashed'
# called the first time when damage reaches 100%
# delta_ft is the vertical drop in time interval loopTime
#
# t/(t+5) is a crude approximation of tanh(t), which is the real equation
# to use for terminal velocity under gravity with drag proportional to v squared.  However tanh is very expensive
# and since we have to approximate the coefficient of drag and other variables
# related to the damaged aircraft anyway, based on very incomplete information,
# this approximation is about good enough and definitely much faster than tanh
			

var aircraftCrashControl = func (myNodeName) {

	if (!bombableMenu["bombable-enabled"] ) return;
	var ats = attributes[myNodeName];
	var ctrls = ats.controls;	
				
	#If we reset the damage levels, stop crashing:
	if (ats.damage < 1 ) return;
	
	#If we have hit the ground, stop crashing:
	if (ctrls.onGround) 
	{
		debprint ("Bombable: Ending aircraft crash control for " ~ myNodeName);
		return();
	}

	# var loopTime = 0.095 + rand() * .01; #rjw add noise
	var loopTime = 0.475 + rand() * .05; #rjw reduced frequency - looks OK - else could keep high frequency for picth and speed change

	var crash = ctrls.crash;
	crash.elapsedTime += loopTime;
	# crash.crashCounter += 1;	# removed - debug only
	
	var newTrueAirspeed_fps = crash.initialSpeed + crash.speedChange / ( 1 + 5 / crash.elapsedTime );

	if (crash.speedChange < 0) 
	{
		var newVertSpeed = crash.vertSpeed;
		if (newVertSpeed > -crash.maxVertSpeed)  # limit at terminal velocity
		{
			newVertSpeed -= grav_fpss * (1 - newTrueAirspeed_fps * newTrueAirspeed_fps / crash.initialSpeed / crash.initialSpeed) * loopTime; 
		}
		var newPitchAngle = math.asin(newVertSpeed / newTrueAirspeed_fps) * R2D;
		if (rand() < .002) reduceRPM(myNodeName);
	}
	else
	{
		var newPitchAngle = crash.initialPitch + crash.pitchChange / ( 1 + 5 / crash.elapsedTime );
		var newVertSpeed = math.sin(newPitchAngle * D2R) * newTrueAirspeed_fps;
	}

	
	# Change speeds
	setprop (""~myNodeName~ "/velocities/vertical-speed-fps", newVertSpeed);
	setprop (""~myNodeName~ "/velocities/true-airspeed-kt", newTrueAirspeed_fps * FPS2KT);
	crash.vertSpeed = newVertSpeed;	

	# Change pitch
	setprop (""~myNodeName~ "/orientation/pitch-deg", newPitchAngle );
	# setprop (""~myNodeName~ "/controls/flight/target-pitch", newPitchAngle ); # not sure useful since vertical-mode is 'alt'
	# rjw:  maximum pitch is 70 degrees
	# if (pitchAngle > -70) pitchAngle +=  pitchPerLoop;
	# setprop (""~myNodeName~ "/orientation/pitch-deg", pitchAngle); 
	
	# Change target speed and alt
	setprop (""~myNodeName~ "/controls/flight/target-spd", newTrueAirspeed_fps * FPS2KT);
	var currAlt_ft = getprop(""~myNodeName~ "/position/altitude-ft");
	var delta_ft = newVertSpeed * loopTime;	
	setprop (""~myNodeName~ "/controls/flight/target-alt", currAlt_ft + 4 * delta_ft); # force the AC down

	# Make it roll
	var rollAngle = getprop (""~myNodeName~ "/orientation/roll-deg");
	if (rand() < (loopTime / 5) or math.abs(rollAngle) > 70) setprop (""~myNodeName~ "/controls/flight/target-roll", (rand() - .5) * 140); 
	
	# if (math.fmod(crash.crashCounter , 10) == 0) 
	# debprint
	# (
	# 	sprintf(
	# 	"Bombable: CrashControl for %s: newTrueAirspeed_fps = %6.1f newVertSpeed = %6.1f newPitchAngle = %6.1f target-alt = %5.0f",
	# 	getCallSign(myNodeName),
	# 	newTrueAirspeed_fps,
	# 	newVertSpeed,
	# 	newPitchAngle,	
	# 	currAlt_ft + 4 * delta_ft
	# ));

	# elevation of -1371 ft is a failsafe (lowest elevation on earth); so is
	# elapsed, so that we don't get stuck in this routine forever
	if ( attributes[myNodeName].controls.onGround != 1 and currAlt_ft > -1371 and crash.elapsedTime < 600 ) 
	{
		settimer (func { aircraftCrashControl(myNodeName)}, loopTime );
	}
	else 
	{
		# we should be crashed at this point but just in case:
		if ( currAlt_ft <= -1371 ) add_damage(1, "crash", myNodeName);
	}
	
}

################################## aircraftCrash ################################
# rjw initializes the aircraft crash loop

var aircraftCrash = func (myNodeName) {
	if (!bombableMenu["bombable-enabled"] ) return;

	stopDodgeAttack(myNodeName);

	var pitch = getprop(""~myNodeName~ "/orientation/pitch-deg");
	var speed = getprop(""~myNodeName~ "/velocities/true-airspeed-kt") * KT2FPS;
	var initialVertSpeed = getprop(""~myNodeName~ "/velocities/vertical-speed-fps");

	var r = rand();
	if (rand() > .7 ) 
	{
		var pitchChange = -20 - r * 40; 
		var speedChange = (.5 + r) * 120;
		#how much to change pitch (deg) and speed (fps) of aircraft over course of crash
	}
		else
	{
		var pitchChange = 0; 
		var speedChange = -speed * (.25 + .25 * r);
		reduceRPM(myNodeName);
	}
		
	attributes[myNodeName].controls["crash"] =
	{
		initialPitch : pitch ,
		pitchChange : pitchChange , 
		initialSpeed : speed ,
		vertSpeed : initialVertSpeed ,
		maxVertSpeed : -speedChange * 0.94, # i.e. sin(70 deg), assuming max pitch 70 deg & max 50% reduction of airspeed to reach terminal velocity 
		speedChange : speedChange ,
		elapsedTime : 0 ,
	};

	debprint (sprintf("Bombable: Starting crash control for %s, pitchChange = %5.1f, speedChange = %5.1f",
		getCallSign(myNodeName),
		pitchChange,
		speedChange	
	));
	
	aircraftCrashControl(myNodeName);
}

################################ variable_safe ##################################
# return string converted into nasal variable/proptree safe form
#
var variable_safe = func(str) {
	var s = "";
	if (str == nil) return s;
	if (size(str) > 0 and !string.isalpha(str[1])) s = "_"; #make sure we always start with alpha char OR _
	for (var i = 0; i < size(str); i  +=  1) {
		if (string.isalnum(str[i]) or str[i] == `_` )
		s  ~=  chr(str[i]);
		if (str[i] == ` ` or str[i] == `-` or str[i] == `.`) s ~=  "_";
	}
	return s;
}

################################ un_variable_safe ##################################
# return string converted from nasal variable/proptree safe form
# back into human readable form
#
var un_variable_safe = func(str) {
	var s = "";
	for (var i = 0; i < size(str); i  +=  1) {
		if (str[i] == `_` and i != 1 )	s ~=  " "; #ignore initial _, that is only to make a numeric value start with _ so it is a legal variable name
		else s  ~=  chr(str[i]);
	}
	return s;
}

#######################################################################
# insertionSort
# x = a vector of keys into hash, h
# k is a key of h.x
# a and b are the values of k
# function f returns > 0 if a > b and direction > 0
# f returns > 0 if a < b and direction < 0
# 
# The default sort is by string using the values of h.x, but not using k; see below
#
var insertionSort = func (x = nil, f = nil, h = nil, k = nil, direction = 1 ) {
	#the default is to sort by string for all values, including numbers
	#but if some of the values are numbers we have to convert them to string
	if (f == nil) f = func(a, b, h, k, direction) 
	{
		if (num(a) == nil) var acomp = a else var acomp = sprintf("%f", a);
		if (num(b) == nil) var bcomp = b else var bcomp = sprintf("%f", b);
		return (cmp (acomp,bcomp) * direction);
	};

	for (var i = 1; i < size(x); i += 1) 
	{
		var key1 = x[i];
		var j = i-1;
		var done = 0;
		while (!done) 
		{
			if (f(x[j], key1, h, k, -1) > 0 ) 
			{
				x[j+1] = x[j];
				j -= 1;
				if (j < 0) done = 1;
			}
			else 
			{
				done = 1;
			}
		}
		x[j+1] = key1;
	}
	return(x);
}

#######################################################################
# newSort
# x = a vector of indices
# y is a vector containing the values of x
# direction == 1 is an ascending sort
var newSort = func (x = nil, y = nil, direction = 1 ) {
	for (var i = 1; i < size(x); i += 1) 
	{
		var index = x[i];
		var j = i-1;
		var done = 0;
		var swap = 0;
		while (!done) 
		{
			swap = ((direction == 1) ? (y[x[j]] > y[index]) : (y[x[j]] < y[index]));
			if (swap)
			{
				x[j+1] = x[j];
				j -= 1;
				if (j < 0) done = 1;
			}
			else 
			{
				done = 1;
			}
		}
		x[j+1] = index;
	}
	return(x);
}
############################ records ##############################
# CLASS records
#
# Class for keeping stats on hits & misses, printing &
# displaying stats, etc.
#
var records = {};

records.init = func () {
	me.impactTotals = {};
	me.impactTotals.Overall = {};
	me.impactTotals.Overall.Total_Impacts = 0;
	me.impactTotals.Overall.Damaging_Impacts = 0;
	me.impactTotals.Overall.Total_Damage_Added = 0;
	me.impactTotals.Objects = {};
	me.impactTotals.Ammo_Categories = {};
	me.impactTotals.Ammo_Type = {};
	me.impactTotals.Sorted = {};
	me.impactTotals.Sorted.Ammo_Categories = [];
	me.impactTotals.Sorted.Objects = [];
	me.impactTotals.Sorted.Ammo_Type = [];
}
				
records.record_impact = func ( myNodeName = "", damageRise = 0, damageIncrease = 0, damageValue = 0, 
impactNodeName = nil, ballisticMass_lb = nil, lat_deg = nil, lon_deg = nil, alt_m = nil ) 
{
	# we will get damaging impacts twice--once from add_damage
	# and once from put_splash.  So we count total impacts from
	# put_splash (damageRise == 0) and damaging impacts from
	# add_damage (damageRise >= 0).
	if (damageRise > 0 ) me.impactTotals.Overall.Damaging_Impacts += 1;
	else me.impactTotals.Overall.Total_Impacts += 1;
					
	me.impactTotals.Overall.Total_Damage_Added  +=  100 * damageRise;
					
	var weaponType = nil;
	if (impactNodeName != nil) weaponType = getprop (""~impactNodeName~"/name");
	var ballCategory = nil;
	if ( ballisticMass_lb < 1) ballCategory = "Small arms";
	elsif ( ballisticMass_lb <= 10) ballCategory = "1 to 10 lb ordinance";
	elsif ( ballisticMass_lb <= 100) ballCategory = "11 to 100 lb ordinance";
	elsif ( ballisticMass_lb <= 500) ballCategory = "101 to 500 lb ordinance";
	elsif ( ballisticMass_lb <= 1000) ballCategory = "501 to 1000 lb ordinance";
	elsif ( ballisticMass_lb > 1000) ballCategory = "Over 1000 lb ordinance";

	var callsign = getCallSign (myNodeName);
	if (myNodeName == "") callsign = nil;
					
	var items = [callsign, weaponType, ballCategory];
	for (var count = 0; count < size (items); count += 1  ) {
		var item = items[count];
		if (item == nil or item == "") continue;
						
		var i = variable_safe (item);
		var category = "Objects";
		if (item == weaponType)  category = "Ammo_Type";
		if (item == ballCategory) category = "Ammo_Categories";
						
						
		if (! contains ( me.impactTotals, category)) {
			me.impactTotals[category] = {};
		}
						
		var currHash = me.impactTotals[category];
						
		if (! contains (currHash, i) ){
			currHash[i] = {};
							
			# We don't save impacts per callsign, because misses & terrain
			# impacts can be picked up by any AI object or the main AC in a
			# fairly random/meaningless fashion
			if (count != 0) currHash[i].Total_Impacts = 0;
			currHash[i].Damaging_Impacts = 0;
			currHash[i].Total_Damage_Added = 0;
							
		}
						
		if ( damageRise > 0 ) currHash[i]["Damaging_Impacts"] += 1;
		elsif (currHash[i]["Total_Impacts"] != nil) currHash[i]["Total_Impacts"] += 1;

		currHash[i].Total_Damage_Added  +=  100 * damageRise;
						
	}

}

records.sort_keys = func 
{
	# rjw the records class defines 4 categories for reporting
	# all except "Overall" are sorted by Total_Damage_Added (largest first)
	# the sorted keys are stored under key 'Stored'
	# var k below contains the 4 categories

	var impactDB = me.impactTotals;

	foreach (var k; keys(impactDB))
	{
		if ((k == "Overall") or (k == "Sorted")) continue;
		var childKeys = keys(impactDB[k]); 
		impactDB.Sorted["" ~ k] = insertionSort
		(
			childKeys,
			func(a, b, impactDB, k, direction)
			{
				var acomp = impactDB[k][a].Total_Damage_Added;
				var bcomp = impactDB[k][b].Total_Damage_Added;
				if ( acomp == bcomp ) return 0;
				return ( acomp > bcomp) ? direction : -direction;
			},
			impactDB,
			k
		);
	}			
}
			
records.display_results = func 
{
	debug.dump (me.impactTotals);
	me.show_totals_dialog();
}
			
records.add_property_tree = func (location, hash ) {
	# not working, we have spaces in our names
	props.globals.getNode(location,1).removeChildren();
	props.globals.getNode(location,1).setValues( hash );
	return props.globals.getNode(location);
}

records.create_printable_summary = func (obj, sortkey = nil, prefix = "") {
# for some of the objects the keys have been sorted using func sort_keys
# if not use unsorted keys 
	var msg = "";
	if (typeof(obj) != "hash") return;
	if (sortkey == nil) sortkey = keys(obj);
					
	foreach (var i; sortkey) 
	{
		if (i == "Sorted") continue;
		if (typeof(obj[i]) == "hash" ) 
		{
			msg  ~=  prefix ~ un_variable_safe (i) ~ ": \n";
			msg  ~=  me.create_printable_summary (obj[i], me.impactTotals.Sorted["" ~ i], prefix~"  ");
		}
		else 
		{
			var num = sprintf("%1.0f", obj[i]);
			msg  ~=  prefix ~ "  " ~ un_variable_safe(i) ~ ": " ~ num ~ "\n";
		}
	}
	return msg;
}
			
records.show_totals_dialog = func 
{
	var totals = 
	{
		title: "Bombable Impact Statistics Summary",
		line: "Note: Rounds which do not impact terrain or an AI object are not recorded"
	};
	me.sort_keys();
	totals.text = me.create_printable_summary (me.impactTotals);
	node = me.add_property_tree ("/bombable/records", me.impactTotals);
	node = me.add_property_tree ("/bombable/dialogs/records", totals);
	gui.showHelpDialog ("/bombable/dialogs/records");
}
			

################################ add_damage ################################
# function adds damage to an AI aircraft, ship or groundvehicle
# (called by the fire loop and ballistic impact
# listener function, typically)
# returns the amount of damage added (which may be smaller than the damageRise requested, for various reasons)
# damagetype is "weapon" or "nonweapon".  nonweapon damage (fire, crash into
# ground, etc) is not passed on via multiplayer (fire, crash, etc damage is
# handled on their end and if all connected players add fire & crash damage via
# multiplayer, too, then it creates a nasty cascade)
# Also slows down the vehicle whenever damage increases.
# vuls.damageVulnerability multiplies the damage, with an M1 tank = 1.  vuls.damageVulnerability = 2
# means 2X the damage.
# maxSpeedReduce is a percentage, the maximum percentage to reduce speed
# in one step.  An airplane might keep moving close to the same speed
# even if the engine dies completely.  A tank might stop forward motion almost
# instantly.

var add_damage = func
(
	damageRise, 
	damagetype = "weapon", 
	myNodeName = nil, # target 
	myNodeName2 = nil, # shooter
	impactNodeName = nil, # projectile
	ballisticMass_lb = nil, 
	lat_deg = nil, 
	lon_deg = nil, 
	alt_m = nil 
)

{
	if (!bombableMenu["bombable-enabled"] ) return 0;

	var callsign = getCallSign (myNodeName);
	var callsign2 = "";
	var msg2 = "";
	if (myNodeName2 != nil) 
	{
		callsign2 = getCallSign (myNodeName2);
		msg2 = " Shooter: " ~ callsign2 ;
	}
					
	if (myNodeName == "") 
	{
		var damAdd = mainAC_add_damage(damageRise, 0, "weapons", "Damaged by" ~ msg2);
		return damAdd;
	}

	var ats = attributes[myNodeName];
	# check for destroyed AC on ground; if so, no further action needed

	# change to stop crashed destroyed ACs continuing at minSpeed_kts at 0m AGL 
	if (ats.controls.onGround) return 0; # 
	# if (ats.exploded) return 0;

	
	var vuls = ats.vulnerabilities;
	var spds = ats.velocities;
	var livs = ats.damageLiveries;
	var ctrls = ats.controls;
	var liveriesCount = livs.count;
	var type = ats.type;

	var damageValue = ats.damage;

	# make sure it's in range 0-1.0
	if(damageRise > 1.0)
		damageRise = 1.0;
	elsif(damageRise < 0.0)
		damageRise = 0.0;
	# rjw damageRise < 0!!! Is this function called for repair?
					
	# update attributes.damage: 0.0 means no damage, 1.0 means full damage
	var prevDamageValue = damageValue;
	damageValue  +=  damageRise;
					
	#make sure it's in range 0-1.0
	if (damageValue > 1.0)
	{
		damageValue = 1.0;
	}
	elsif (damageValue < 0.0)
	{
		damageValue = 0.0;
	}
	ats.damage = damageValue;
	setprop(""~myNodeName~"/bombable/attributes/damage", damageValue); # prop tree hook for animation of AI models via xml
	var damageIncrease = damageValue - prevDamageValue;

	if (liveriesCount > 0 and liveriesCount != nil ) 
	{							
		var livery = livs.damageLivery [ int ( damageValue * ( liveriesCount - 1 ) ) ];
		setprop(""~myNodeName~"/bombable/texture-corps-path", livery );
	}

	if (damageIncrease > 0.05 and type == "aircraft") reduceRPM(myNodeName);
	#rjw: big hit so spin down an engine				

	if (damagetype == "weapon") 
	{
		ctrls.stayInFormation = 0;
		records.record_impact ( myNodeName, damageRise, damageIncrease, damageValue, impactNodeName, ballisticMass_lb, lat_deg, lon_deg, alt_m );
	}
						
	var weapPowerSkill = ctrls.weapons_pilot_ability;				
	if ( damageIncrease > 0 ) 
	{
		# Always display the message if a weapon hit or large damageRise. Otherwise
		# only display about 1 in 20 of the messages.
		# If we don't do this the small damageRises from fires overwhelm the message area
		# and we don't know what's going on.
		if (damagetype == "weapon" or damageRise > 0.1 or rand() < .05)
		{
			damageRiseDisplay = round( damageRise * 100 );
			if (damageRise < .01) damageRiseDisplay = sprintf ("%1.3f",damageRise * 100);
			elsif (damageRise < .1) damageRiseDisplay = sprintf ("%1.1f",damageRise * 100);
							
							
			var msg = "Damage added: " ~ damageRiseDisplay ~ "% for " ~  callsign ~ " Total: " ~ round ( damageValue * 100 ) ~ "%, Skill: " ~ math.ceil(10 * weapPowerSkill) ~ msg2;
			debprint ( "Bombable: " ~ msg ~ " " ~ myNodeName );
							
			targetStatusPopupTip (msg, 20);
		}

		if (damagetype == "weapon") 
		{
			if (damageRise > 0.025 and rand() < 0.5 and myNodeName2 != nil and !ats.controls.attackInProgress)
			{
				# change target to shooter, if not already
				var ats2 = attributes[myNodeName2];
				if (vecindex(ats.targetIndex, ats2.index) == nil)
				{
					if (size(ats.targetIndex) == ats.maxTargets) removeTarget(ats.index);
					append (ats.targetIndex, ats2.index); # add shooter causing the damage to my targets
					append (ats2.shooterIndex, ats.index); # add me to the shooter's list of shooters
					debprint (callsign, " set ", callsign2, " as new target");
				}
			}

			if ((rand() < calcPilotSkill (myNodeName) / 6)) dodge (myNodeName);
		}

		if (damageRise > 0.1 and rand() > .5 and contains(ats,"weapons"))
		# a large hit can knock-out a weapon
		# if lose a weapon, lose a target
		{
			var weaps = keys(ats.weapons);
			var nWeapons = size(weaps) ;
			var index = int (rand() * nWeapons) ;
			var thisWeapon = ats.weapons[weaps[index]];
			var destroyNow = (thisWeapon.weaponType == 1) ? !(thisWeapon.controls["launched"] == 1 or thisWeapon.destroyed) : !thisWeapon.destroyed;
			# if rocket it cannot be destroyed if already launched
			if (destroyNow) 
			{
				thisWeapon.destroyed = 1;
				ats.nRockets -= 1;
				debprint(callsign," "~weaps[index]~" destroyed");
				if (ats.maxTargets) 
				{
					if (!thisWeapon.aim.fixed or ats.nFixed < 2)
					{
						ats.maxTargets -= 1;
						removeTarget(ats.index);
					}
					ats.nFixed -= thisWeapon.aim.fixed;
				}
			}
		}
	}

	# for moving objects (ships & aircraft), reduce velocity each time damage added
	# eventually  stopping when damage = 1.
	# But don't reduce speed below minSpeed.
	# we put it here outside the "if" statement so that burning
	# objects continue to slow/stop even if their damage is already at 1
	# this happens when file/reset is chosen in FG					
	# rjw: tgt-speed-kts is used for ships and flight_tgt_spd for aircraft and groundvehicles

	# max speed reduction due to damage, in %
	var maxSpeedReduceFactor = 1 - spds.maxSpeedReduce_percent / 100; 

	if ( damageValue == 1 and damageIncrease > 0) 
	{
		resetTargetShooter(ats.index);  
		if (type == "aircraft") 
		{
			# rjw: aircraft will now crash
			reduceRPM(myNodeName);
			aircraftCrash (myNodeName);
		}
		else
		{
			# for ships and ground vehicles decelerate at the maxSpeedReduce
			var loopid = ats.loopids.ground_loopid;
			settimer( func{reduceSpeed(loopid, myNodeName, maxSpeedReduceFactor, type)},1);
		}
		# exit here or skip next block but need to start ship listing?
	}

	var speedReduce = 1 - damageValue;
	if (speedReduce < maxSpeedReduceFactor) speedReduce = maxSpeedReduceFactor;
					
	minSpeed = spds.minSpeed_kt;
	# debprint("spds.speedOnFlat = ",spds.speedOnFlat);				
	if (type == "ship") 
	# ships are controlled in a similar way to ground vehicles
	{
		var tgt_spd_kts = getprop (""~myNodeName~"/controls/tgt-speed-kts");
		if (tgt_spd_kts == nil ) tgt_spd_kts = 0;

		if (tgt_spd_kts > minSpeed) 
		{
			setprop(""~myNodeName~"/controls/tgt-speed-kts", tgt_spd_kts * speedReduce);
			spds.speedOnFlat *= speedReduce;
		}
		else
		{
			setprop(""~myNodeName~"/controls/tgt-speed-kts", minSpeed);
			spds.speedOnFlat = minSpeed;
		}
			
		# believe that the ship AI will handle this
		# if (true_spd > minSpeed)
		# setprop(""~myNodeName~"/velocities/true-airspeed-kt",
		# true_spd * speedReduce);
		
		# start listing of ship
		# progressive motion managed by ground loop
		if ((rand() < 0.5) and ( damageValue >= 0.75)) 
		{
			if (ctrls["target_roll"] == nil) 
			{
				ctrls.target_roll = rand() * 60 - 30; #up to 30 degrees
				
				ctrls.target_pitch = rand() * 10 - 5; #up to 5 degrees
			}
		}
	}
	else  # aircraft or GV
	{
		if (type == "aircraft") minSpeed = trueAirspeed2indicatedAirspeed (myNodeName, spds.minSpeed_kt); # correct for altitude
		var flight_tgt_spd = getprop (""~myNodeName~"/controls/flight/target-spd");
		if (flight_tgt_spd == nil ) flight_tgt_spd = 0;
					
		var true_spd = getprop (""~myNodeName~"/velocities/true-airspeed-kt");
		if (true_spd == nil ) true_spd = 0;


		# only reduce speeds at high damage values
		if ( damageValue >= 0.75) 
		{
			if (flight_tgt_spd > minSpeed) 
			{
				setprop(""~myNodeName~"/controls/flight/target-spd", flight_tgt_spd * speedReduce);
				if (type == "groundvehicle") spds.speedOnFlat *= speedReduce;
			}
			else 
			{
				setprop(""~myNodeName~"/controls/flight/target-spd", minSpeed);
				if (type == "groundvehicle") spds.speedOnFlat = minSpeed;
			}
		}
	}  

	var fireStarted = getprop(""~myNodeName~"/bombable/fire-particles/fire-burning");
	if (fireStarted == nil ) fireStarted = 0;

	var damageEngineSmokeStarted = getprop(""~myNodeName~"/bombable/fire-particles/damagedengine-burning");
	if (damageEngineSmokeStarted == nil ) damageEngineSmokeStarted = 0;
					
	#don't print this for every fire damage rise, but otherwise . . .
	#if (!fireStarted or damageRise > vuls.fireDamageRate_percentpersecond * 2.5 ) debprint ("Damage added: ", damageRise, ", Total damage: ", damageValue);
	#Start damaged engine smoke but only sometimes; greater chance when hitting an aircraft

	if (!damageEngineSmokeStarted and !fireStarted and rand() < damageRise * vuls.engineDamageVulnerability_percent / 2 ) 
	{
		startSmoke("damagedengine",myNodeName);
		#rjw: can reduce engine rpm at startSmoke or at this point
		if (type == "aircraft") reduceRPM(myNodeName);
	}
	# start fire if there is enough damage.
	# if(damageValue >= 1 - vuls.fireVulnerability_percent/100 and !fireStarted ) {
						
	# a percentage change of starting a fire with each hit
	if( rand() < .035 * damageRise * (vuls.fireVulnerability_percent) and !fireStarted ) 
	{
		debprint ("Bombable: Starting fire for" ~ myNodeName);
						
		#use small, medium, large smoke column depending on vuls.damageVulnerability
		#(high vuls.damageVulnerability means small/light/easily damaged while
		# low vuls.damageVulnerability means a difficult, hardened target that should burn
		# more vigorously once finally on fire)
		#var fp = "";
		#if (vuls.explosiveMass_kg < 1000 ) { fp = "AI/Aircraft/Fire-Particles/fire-particles-small.xml"; }
		#elsif (vuls.explosiveMass_kg > 50000 ) { fp = "AI/Aircraft/Fire-Particles/fire-particles-large.xml"; }
		#else {fp = "AI/Aircraft/Fire-Particles/fire-particles.xml";}

		#small, med, large fire depending on size of hit that caused it
		var fp = "";
		if (damageRise < 0.2 ) { fp = "AI/Aircraft/Fire-Particles/fire-particles-very-small.xml"; }
		elsif (damageRise > 0.5 ) { fp = "AI/Aircraft/Fire-Particles/fire-particles.xml"; }
		else {fp = "AI/Aircraft/Fire-Particles/fire-particles-small.xml";}
						
		startFire(myNodeName, fp);
		#only one damage smoke at a time . . .
		deleteSmoke("damagedengine",myNodeName);
						
						
		#fire can be extinguished up to MaxTime_seconds in the future,
		#if it is extinguished we set up the damagedengine smoke so
		#the smoke doesn't entirely go away, but no more damage added
		if ( rand() * 100 < vuls.fireExtinguishSuccess_percentage ) 
		{
			settimer (func {
				deleteFire (myNodeName);
				startSmoke("damagedengine",myNodeName);
			} ,
			rand() * vuls.fireExtinguishMaxTime_seconds + 15 ) ;
		}
		#      debprint ("started fire");
						
		#Set livery to the one corresponding to this amount of damage
		if (liveriesCount > 0 and liveriesCount != nil ) 
		{
			livery = livs.damageLivery [ int ( damageValue * ( liveriesCount - 1 ) ) ];
			setprop(""~myNodeName~"/bombable/texture-corps-path",
			livery );
		}
	} # end of starting fire block
	#only send damage via multiplayer if it is weapon damage from our weapons
	if (type == "multiplayer" and damagetype == "weapon") 
	{
		mp_send_damage(myNodeName, damageRise);
	}
	return  damageIncrease;
}


######################### inc_loopid ###########################
#functions to increment loopids
#these are called on init and destruct (which should be called
#when the object loads/unloads)
#When the loopid increments it will kill any timer functions
#using that loopid for that object.  (Otherwise they will just
#continue to run indefinitely even though the object itself is unloaded)
var inc_loopid = func (nodeName = "", loopName = "") {
	var s = loopName ~ "_loopid";
	var loopid = attributes[nodeName].loopids[s];
	if ( loopid == nil ) loopid = 0;
	loopid  +=  1;
	attributes[nodeName].loopids[s] = loopid;
	return loopid;
}

######################## set_livery #############################
# Set livery color (including normal through
# slightly and then completely damaged)
#
# Example:
#
# liveries = [
#          "Models/livery_nodamage.png",
#          "Models/livery_slightdamage.png",
#          "Models/livery_highdamage.png"
#  ];
#  bombable.set_livery (cmdarg().getPath(), liveries);

var set_livery = func (myNodeName, liveries) {
	if (!bombableMenu["bombable-enabled"] ) return;
						
						
	if (! contains (bombable.attributes, myNodeName)) {
		bombable.attributes[myNodeName] = {};
		debprint("Bombable: set_livery:" ~ myNodeName ~ " node not initialised yet");
	}

	bombable.attributes[myNodeName].damageLiveries = {};
	var livs = bombable.attributes[myNodeName].damageLiveries;

	#set new liveries, also set the count to the number
	#of liveries installed
	if (liveries == nil or size ( liveries) == 0 ) return();
	livs.damageLivery = liveries;
	livs.count = size (liveries);
	bombable.attributes[myNodeName].damageLiveries = livs;
	debprint("Bombable: Set_livery: livs = ",livs);					

	
	#current color (we'll set it to the undamaged color;
	#if the object is on fire/damage damaged this will soon be updated)
	#by the timer function
	#To actually work, the aircraft's xml file must be set up with an
	#animation to change the texture, keyed to the bombable/texture-corps-path
	#property

	setprop(""~myNodeName~"/bombable/texture-corps-path", liveries[0]);
}

######################## checkRange #############################
var checkRange = func (v = nil, low = nil, high = nil, default = 1) {

	if ( v == nil ) v = default;
	if ( low != nil and v < low  ) v = low;
	if ( high != nil and v > high ) v = high;
						
	return v;
}

var checkRangeHash = func (b = nil, v = nil, low = nil, high = nil, default = 1) {
	if (contains (b, v)) return checkRange (b[v],low, high, default)
	else return default;
}

######################################################################
################################ initialize ######################################
######################################################################
#delaying all the _init functions until FG's initialization sequence
#has settled down seems to solve a lot of FG crashes on startup when
#bombable is running with scenarios.
#It takes about 60 seconds to get them all initialized.
#
var initialize = func (b) {
	debprint ("Bombable: Delaying initialize . . . ", b.objectNodeName);
	settimer (func {initialize_func(b);}, 30, 1);

}


######################### initialize_func ############################
# initialize: Do sanity checking, then
# slurp the pertinent properties into
# the object's node tree under sub-node "bombable"
# so that they can be accessed by all the different
# subroutines
#
# The new way: All these variables are stored in attributes[myNodeName]
# (myNodeName = "" for the main aircraft).
#
# This saves a lot of a reading/writing from the property tree,
# which turns out to be quite slow.
#
# The old way:
#
# If you just need a certain property or two you can simply read it
# with getprops.
#
# But for those routines that use many/all we can just grab them all with
# var b = props.globals.getNode (""~myNodeName~"/bombable/attributes");
# bomb = b.getValues();  #all under the "bombable/attributes" branch
# Then use values like bomb.dimensions.width_m etc.
# Normally don't do this as it slurps in MANY values
#
# But (better if you only need one sub-branch)
# dims = b.getNode("dimensions").getValues();
# Gets values from subbranch 'dimensions'.
# Then your values are dims.width_m etc.
#
#
var initialize_func = func ( b ){

	# only allow initialization for ai & multiplayer objects
	# in FG 2.4.0 we're having trouble with strange(!?) init requests from
	# joysticks & the like
	var init_allowed = 0;
	if (find ("/ai/models/", b.objectNodeName ) != -1 ) init_allowed = 1;
	if (find ("/multiplayer/", b.objectNodeName ) != -1 ) init_allowed = 1;

	if (init_allowed != 1) {
		debprint ("Bombable: Attempt to initialize a Bombable subroutine on an object that is not AI or Multiplayer; aborting initialization. ", b.objectNodeName);
		return;
	}
						


	#do sanity checking on input
	#also calculate a few values that will be useful later on &
	#add them to the object

	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	# if it 1 and we're trying to initialize, something has gone wrong and we abort with a message.
	var inited = getprop(""~b.objectNodeName~"/bombable/initializers/attributes-initialized");
						
	if (inited == 1) {
		debprint ("Bombable: Attempt to re-initialize attributes when it has not been de-initialized; aborting re-initialization. ", b.objectNodeName);
		return;
	}
	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	setprop(""~b.objectNodeName~"/bombable/initializers/attributes-initialized", 1);
	debprint( "Bombable: Initializing bombable attributes for ", b.objectNodeName);


	b.updateTime_s = checkRange ( b.updateTime_s, 0, 10, 1);

	# Set an individual pilot ability, -1 to 1, with 0 being average
	var ability = math.pow (rand(), 1.5); # U-shaped distribution???
	if (rand() > .5) ability = -ability;

	# add controls key, used to control animation of damaged ships and aircraft
	b.controls = 
	{ 
		groundLoopCounter : 0, 
		onGround : 0, 
		damageAltAddCurrent_ft : 0, 
		damageAltAddCumulative_ft : 0, 
		dodgeInProgress : 0, 
		avoidCliffInProgress : 0,
		attackInProgress : 0, 
		attackClimbDiveInProgress : 0,
		attackClimbDiveTargetAGL_m : 0,
		stalling: 0,
		rollTimeElapsed: 0,
		roll_deg_bombable: 0,
		courseToTarget_deg: 0,
		pilotAbility: ability,
		weapons_pilot_ability : 0.2,
		stayInFormation : 1,
		kamikase : 0, # true for kamikase behaviour when ammo used up
	};

	b.loopids = 
	{
		roll_loopid : 0,
	};
	# hash used by inc_loopid for loop counters

	b.damage = 0;
	b.exploded = 0;
	b.team = nil;
	b.side = -1;
	b.index = -1;
	b.targetIndex = [];
	b.shooterIndex = [];
	b.nFixed = 0; 
	b.nRockets = 0; 
	b.maxTargets = 0;

	if (! contains (b, "type")) b["type"] = props.globals.getNode(""~b.objectNodeName).getName(); 
	# key allows AI ship models to be used as ground vehicles by adding type:"groundvehicle" to Bombable attributes hash
						
	# altitudes sanity checking
	if (contains (b, "altitudes") and typeof (b.altitudes) == "hash") {
		b.altitudes.wheelsOnGroundAGL_m = checkRange ( b.altitudes.wheelsOnGroundAGL_m, -1000000, 1000000, 0 );
		b.altitudes.minimumAGL_m = checkRange ( b.altitudes.minimumAGL_m, -1000000, 1000000, 0 );
		b.altitudes.maximumAGL_m = checkRange ( b.altitudes.maximumAGL_m, -1000000, 1000000, 0 );
		#keep this one negative or zero:
		b.altitudes.crashedAGL_m = checkRange ( b.altitudes.crashedAGL_m, -1000000, 0, -0.001 );
		if (b.altitudes.crashedAGL_m == 0 )b.altitudes.crashedAGL_m = -0.001;
							
		b.altitudes.initialized = 0; #this is how ground_loop knows to initialize the alititude on its first call
		b.altitudes.wheelsOnGroundAGL_ft = b.altitudes.wheelsOnGroundAGL_m * M2FT;
		b.altitudes.minimumAGL_ft = b.altitudes.minimumAGL_m * M2FT;
		b.altitudes.maximumAGL_ft = b.altitudes.maximumAGL_m * M2FT;
		b.altitudes.crashedAGL_ft = b.altitudes.crashedAGL_m * M2FT;
							
		#crashedAGL must be at least a bit lower than minimumAGL
		if (b.altitudes.crashedAGL_m > b.altitudes.minimumAGL_m )
		b.altitudes.crashedAGL_m = b.altitudes.minimumAGL_m - 0.001;
	}
						
	# evasions sanity checking
	if (contains (b, "evasions") and typeof (b.evasions) == "hash") {
							
		b.evasions.dodgeDelayMax_sec = checkRangeHash ( b.evasions, "dodgeDelayMax_sec", 0, 600, 30 );
		b.evasions.dodgeDelayMin_sec = checkRangeHash ( b.evasions, "dodgeDelayMin_sec", 0, 600, 5 );
		if (b.evasions.dodgeDelayMax_sec < b.evasions.dodgeDelayMin_sec)
		b.evasions.dodgeDelayMax_sec = b.evasions.dodgeDelayMin_sec;
							
		b.evasions.dodgeMax_deg = checkRangeHash ( b.evasions, "dodgeMax_deg", 0, 180, 90 );
		b.evasions.dodgeMin_deg = checkRangeHash ( b.evasions, "dodgeMin_deg", 0, 180, 30 );
		if (b.evasions.dodgeMax_deg < b.evasions.dodgeMin_deg)
		b.evasions.dodgeMax_deg = b.evasions.dodgeMax_deg;

		b.evasions.rollRateMax_degpersec = checkRangeHash ( b.evasions, "rollRateMax_degpersec", 1, 720, 45 );
							
		if (b.evasions.dodgeROverLPreference_percent == nil) b.evasions.dodgeROverLPreference_percent = 50;
		b.evasions.dodgeROverLPreference_percent = checkRangeHash ( b.evasions,"dodgeROverLPreference_percent", 0, 100, 50 );
							
		b.evasions.dodgeAltMax_m = checkRangeHash ( b.evasions, "dodgeAltMax_m", -100000, 100000, 20 );
		b.evasions.dodgeAltMin_m = checkRangeHash ( b.evasions, "dodgeAltMin_m", -100000, 100000, -20 );
		if (b.evasions.dodgeAltMax_m < b.evasions.dodgeAltMin_m)
		b.evasions.dodgeAltMax_m = b.evasions.dodgeAltMin_m;
		b.evasions.dodgeAltMin_ft = b.evasions.dodgeAltMin_m * M2FT;
		b.evasions.dodgeAltMax_ft = b.evasions.dodgeAltMax_m * M2FT;
							
							
		b.evasions.dodgeVertSpeedClimb_mps = checkRangeHash (b.evasions, "dodgeVertSpeedClimb_mps", 0, 3000, 0 );
		b.evasions.dodgeVertSpeedDive_mps = checkRangeHash ( b.evasions, "dodgeVertSpeedDive_mps", 0, 5000, 0 );
		b.evasions.dodgeVertSpeedClimb_fps = b.evasions.dodgeVertSpeedClimb_mps * M2FT;
		b.evasions.dodgeVertSpeedDive_fps = b.evasions.dodgeVertSpeedDive_mps * M2FT;

	}
						
						
						
	# dimensions sanity checking
	# Need to re-write checkRange so it integrates the check of whether b.dimensions.XXXX
	# even exists and takes appropriate action if not
	if (contains (b, "dimensions") and typeof (b.dimensions) == "hash") {
		b.dimensions.width_m = checkRange ( b.dimensions.width_m, 0, nil , 30 );
		b.dimensions.length_m = checkRange ( b.dimensions.length_m, 0, nil, 30 );
		b.dimensions.height_m = checkRange ( b.dimensions.height_m, 0, nil, 30 );
		if (!contains(b.dimensions, "damageRadius_m")) b.dimensions.damageRadius_m = nil;
		b.dimensions.damageRadius_m = checkRange ( b.dimensions.damageRadius_m, 0, nil, 6 );
		if (!contains(b.dimensions, "vitalDamageRadius_m")) b.dimensions.vitalDamageRadius_m = nil;
		b.dimensions.vitalDamageRadius_m = checkRange ( b.dimensions.vitalDamageRadius_m, 0, nil, 2.5 );
		if (!contains(b.dimensions, "crashRadius_m")) b.dimensions.crashRadius_m = nil;
		b.dimensions.crashRadius_m = checkRange ( b.dimensions.crashRadius_m, 0, nil, b.dimensions.vitalDamageRadius_m );
							


		# add some helpful new:
		#
		b.dimensions.width_ft = b.dimensions.width_m * M2FT;
		b.dimensions.length_ft = b.dimensions.length_m * M2FT;
		b.dimensions.height_ft = b.dimensions.height_m * M2FT;
		b.dimensions.damageRadius_ft = b.dimensions.damageRadius_m * M2FT;
	}
						
	# velocities sanity checking
	if (contains (b, "velocities") and typeof (b.velocities) == "hash") 
	{
		b.velocities.maxSpeedReduce_percent = checkRangeHash ( b.velocities, "maxSpeedReduce_percent", 0, 100, 1 );
		b.velocities.minSpeed_kt = checkRangeHash (b.velocities, "minSpeed_kt", 0, nil, 0 );
		b.velocities.cruiseSpeed_kt = checkRangeHash (b.velocities, "cruiseSpeed_kt", 0, nil, 100 );
		b.velocities.attackSpeed_kt = checkRangeHash (b.velocities, "attackSpeed_kt", 0, nil, 150 );
		b.velocities.maxSpeed_kt = checkRangeHash (b.velocities, "maxSpeed_kt", 0, nil, 250 );
		
		b.velocities["curr_acrobat_vertical_speed_fps"] = 0; # used to control AI aircraft when flying loops					
		b.velocities.damagedAltitudeChangeMaxRate_meterspersecond = checkRangeHash (b.velocities, "damagedAltitudeChangeMaxRate_meterspersecond", 0, nil, 0.5 );
							
		if (contains (b.velocities, "diveTerminalVelocities") and typeof (b.velocities.diveTerminalVelocities) == "hash") {
			var ave = 0;
			var count = 0;
			var sum = 0;
			var dTV = b.velocities.diveTerminalVelocities;
			var sinPitch = 0; var deltaV_kt = 0; var factor = 0;
			foreach (k; keys (dTV) ) {
				dTV[k].airspeed_kt = checkRangeHash (dTV[k], "airspeed_kt", 0, nil, nil );
									
				dTV[k].vertical_speed_fps = checkRangeHash (dTV[k], "vertical_speed_fps", -100000, 0, nil );
									
				if ( dTV[k].airspeed_kt != nil and dTV[k].vertical_speed_fps != nil ){
					dTV[k].airspeed_fps = dTV[k].airspeed_kt * KT2FPS;
					sinPitch = math.abs(dTV[k].vertical_speed_fps/dTV[k].airspeed_fps);
					deltaV_kt = dTV[k].airspeed_kt  - b.velocities.attackSpeed_kt;
					factor = deltaV_kt/sinPitch;
					sum += factor;
					count += 1;
					} else {
					dTV[k].airspeed_fps = nil;
				}
			}
			if (count > 0) {
				ave = sum/count;
				b.velocities.diveTerminalVelocityFactor = ave;
				} else {
				b.velocities.diveTerminalVelocityFactor = 700; #average of Camel & Zero values, so a good typical value
			}
		}
							
		if (contains (b.velocities, "climbTerminalVelocities") and typeof (b.velocities.climbTerminalVelocities) == "hash") {
			var ave = 0;
			var count = 0;
			var sum = 0;
			var cTV = b.velocities.climbTerminalVelocities;
			var sinPitch = 0; var deltaV_kt = 0; var factor = 0;
			foreach (k; keys (cTV) ) {
				cTV[k].airspeed_kt = checkRangeHash (cTV[k], "airspeed_kt", 0, nil, nil );
									
				cTV[k].vertical_speed_fps = checkRangeHash (cTV[k], "vertical_speed_fps", 0, nil, nil );
									
				if ( cTV[k].airspeed_kt != nil and cTV[k].vertical_speed_fps != nil ){
					cTV[k].airspeed_fps = cTV[k].airspeed_kt * KT2FPS;
					sinPitch = math.abs(cTV[k].vertical_speed_fps/cTV[k].airspeed_fps);
					deltaV_kt = b.velocities.attackSpeed_kt - cTV[k].airspeed_kt;
					factor = deltaV_kt/sinPitch;
					sum += factor;
					count += 1;
					} else {
					cTV[k].airspeed_fps = nil;
				}
			}
			if (count > 0) {
				ave = sum/count;
				b.velocities.climbTerminalVelocityFactor = ave;
				} else {
				b.velocities.climbTerminalVelocityFactor = 750; #average of Camel & Zero values, so a good typical value
			}
		}
	}

	# damage sanity checking
	if (contains (b, "vulnerabilities") and typeof (b.vulnerabilities) == "hash") {
		if (b.vulnerabilities.damageVulnerability <= 0) b.vulnerabilities.damageVulnerability = 1;
		b.vulnerabilities.engineDamageVulnerability_percent = checkRange (b.vulnerabilities.engineDamageVulnerability_percent, 0, 100, 1 );
		b.vulnerabilities.fireVulnerability_percent = checkRange (b.vulnerabilities.fireVulnerability_percent, -1, 100, 20 );
		b.vulnerabilities.fireDamageRate_percentpersecond = checkRange (b.vulnerabilities.fireDamageRate_percentpersecond, 0, 100, 1 );
		b.vulnerabilities.fireExtinguishMaxTime_seconds = checkRange (b.vulnerabilities.fireExtinguishMaxTime_seconds, 0, nil, 3600 );
		b.vulnerabilities.fireExtinguishSuccess_percentage = checkRange ( b.vulnerabilities.fireExtinguishSuccess_percentage, 0, 100, 10 );
		b.vulnerabilities.explosiveMass_kg = checkRange ( b.vulnerabilities.explosiveMass_kg, 0, 10000000, 1000 );
	}
						
	if (contains (b, "attacks") and typeof (b.attacks) == "hash") {
		# attacks sanity checking
		if (!b.attacks.minDistance_m) # a value of zero is the flag for kamikase behaviour
		{
			b.controls.kamikase = 1; 
			b.attacks.minDistance_m = 300;
		}
		if (b.attacks.minDistance_m < 0) b.attacks.maxDistance_m = 100;
		if (b.attacks.maxDistance_m < b.attacks.minDistance_m ) b.attacks.maxDistance_m = 2 * b.attacks.minDistance_m;
		if (b.attacks.rollMin_deg == nil ) b.attacks.rollMin_deg = 30;
		if (b.attacks.rollMin_deg < 0) b.attacks.rollMin_deg = 100;
		if (b.attacks.rollMax_deg == nil ) b.attacks.rollMax_deg = 80;
		if (b.attacks.rollMax_deg < b.attacks.rollMax_deg) b.attacks.rollMax_deg = b.attacks.rollMin_deg + 30;
		b.attacks.rollRateMax_degpersec = checkRangeHash ( b.attacks, "rollRateMax_degpersec", 1, 720, 45 );

		if (b.attacks.climbPower == nil ) b.attacks.climbPower = 2000;
		if (b.attacks.climbPower < 0) b.attacks.climbPower = 2000;
		if (b.attacks.divePower == nil ) b.attacks.divePower = 4000;
		if (b.attacks.divePower < 0) b.attacks.divePower = 4000;
		if (b.attacks.attackCheckTime_sec == nil ) b.attacks.attackCheckTime_sec = 15;
		if (b.attacks.attackCheckTime_sec < 0.1) b.attacks.attackCheckTime_sec = 0.1;
		if (b.attacks.attackCheckTimeEngaged_sec == nil ) b.attacks.attackCheckTimeEngaged_sec = 1.25;
		if (b.attacks.attackCheckTimeEngaged_sec < 0.1) b.attacks.attackCheckTimeEngaged_sec = 0.1;
		b.attacks.allGround = 0; #flag used to switch to ground based targets
	}
						
	b["stores"] = {};
	b.stores["fuel"] = 1;
	b.stores["weapons"] = {};
	b.stores["messages"] = {};
	b.stores["messages"]["unreadymessageposted"] = 0;
	b.stores["messages"]["readymessageposted"] = 1;

	# weapons sanity checking
	if (contains(b, "weapons") and typeof (b.weapons) == "hash") {
		var n = 0;
		foreach (elem ; keys(b.weapons)) {
			n += 1;
			if (b.weapons[elem]["name"] == nil ) b.weapons[elem].name = "Weapon " ~ n;
			if (b.weapons[elem]["maxDamage_percent"] == nil ) b.weapons[elem].maxDamage_percent = 5;
			if (b.weapons[elem].maxDamage_percent < 0) b.weapons[elem].maxDamage_percent = 0;
			if (b.weapons[elem].maxDamage_percent > 100) b.weapons[elem].maxDamage_percent = 100;
			if (b.weapons[elem]["maxDamageDistance_m"] == nil ) b.weapons[elem].maxDamageDistance_m = 500;
			if (b.weapons[elem].maxDamage_percent <= 0) b.weapons[elem].maxDamageDistance_m = 1;
			if (b.weapons[elem]["weaponAngle_deg"] == nil) b.weapons[elem].weaponAngle_deg= {};
			if (b.weapons[elem].weaponAngle_deg["heading"] == nil ) b.weapons[elem].weaponAngle_deg.heading = 0;
			if (b.weapons[elem].weaponAngle_deg["elevation"] == nil ) b.weapons[elem].weaponAngle_deg.elevation = 0;
			if (b.weapons[elem].weaponAngle_deg["headingMin"] == nil ) b.weapons[elem].weaponAngle_deg.headingMin = -60;
			if (b.weapons[elem].weaponAngle_deg["headingMax"] == nil ) b.weapons[elem].weaponAngle_deg.headingMax = 60;
			if (b.weapons[elem].weaponAngle_deg["elevationMin"] == nil ) b.weapons[elem].weaponAngle_deg.elevationMin = -20;
			if (b.weapons[elem].weaponAngle_deg["elevationMax"] == nil ) b.weapons[elem].weaponAngle_deg.elevationMax = 20;
			if (b.weapons[elem]["weaponOffset_m"] == nil ) b.weapons[elem].weaponOffset_m = {};
			if (b.weapons[elem].weaponOffset_m["x"] == nil ) b.weapons[elem].weaponOffset_m.x = 0;
			if (b.weapons[elem].weaponOffset_m["y"] == nil ) b.weapons[elem].weaponOffset_m.y = 0;
			if (b.weapons[elem].weaponOffset_m["z"] == nil ) b.weapons[elem].weaponOffset_m.z = 0;

			if (!contains(b.weapons[elem], "weaponSize_m"))
			b.weapons[elem].weaponSize_m = {start:nil, end:nil};
								
			if (b.weapons[elem].weaponSize_m.start == nil
			or b.weapons[elem].weaponSize_m.start <= 0 ) b.weapons[elem].weaponSize_m.start = 0.07;
			if (b.weapons[elem].weaponSize_m.end == nil
			or b.weapons[elem].weaponSize_m.end <= 0 ) b.weapons[elem].weaponSize_m.end = 0.05;
		}
	}
						
	if (contains (b, "damageLiveries") and typeof (b.damageLiveries) == "hash") {
		b.damageLiveries.count = size (b.damageLiveries.damageLivery) ;
	}
						
						
						
	# the object has stored the node telling where to store itself on the
	# property tree as b.objectNodeName.  This creates "/bombable/attributes"
	# under the nodename & saves these values there.

						
	# for now we are saving the attributes hash to the property tree and
	# then also under attributes[myNodeName].  Many of the functions above
	# still get certain
	# attributes values from the property tree.  However it is far better for
	# performance to simply store the values in a local variable.
	# In future for performance reasons we might just save it under local
	# variable attributes[myNodeName] and not in the property tree at all, unless
	# something needs to be made globally available to change at runtime (outside bombable namespace).

	# rjw implemented - deleted property tree attributes
	# b.objectNode.getNode("bombable/attributes",1).setValues( b );

	
	attributes[b.objectNodeName] = b;
	var myNodeName = b.objectNodeName;
	stores.fillFuel(myNodeName, 1);

	# determines how AI aircraft are controlled - Bombable sets altitudes and roll
	if (attributes[myNodeName].type == "aircraft")
	{
		setprop (""~myNodeName~"/controls/flight/vertical-mode", "alt");
		setprop (""~myNodeName~"/controls/flight/lateral-mode", "roll");
	}

	addToTargets(myNodeName); # add AI model to list of targets and ID its team.  Targets are assigned after scenario initialization

	
}

######################## update_m_per_deg_latlon #########################
# update_m_per_deg_latlon_loop
# loop to periodically update the m_per_deg_lat & lon
#
var update_m_per_deg_latlon = func  {

	alat_deg = getprop ("/position/latitude-deg");
	var aLat_rad = alat_deg* D2R;
	m_per_deg_lat = 111699.7 - 1132.978 * math.cos (aLat_rad);
	m_per_deg_lon = 111321.5 * math.cos (aLat_rad);
	#Note these are bombable general variables
}

####################### update_m_per_deg_latlon_loop ##########################
# update_m_per_deg_latlon_loop
# loop to periodically update the m_per_deg_lat & lon
#
var update_m_per_deg_latlon_loop = func (id) {
	id == attributes[""].loopids.update_m_per_deg_latlon_loopid or return;
	#debprint ("update_m_per_deg_latlon_loop starting");
	settimer (func {update_m_per_deg_latlon_loop(id)}, 63.2345);

	update_m_per_deg_latlon();
						
}

##################### setMaxLatLon #####################
# setMaxLatLon
#

var setMaxLatLon = func (myNodeName, damageDetectDistance_m){

	if ( m_per_deg_lat == nil or m_per_deg_lon == nil ) 
	{
		update_m_per_deg_latlon();
	}

	var maxLat_deg =  damageDetectDistance_m / m_per_deg_lat;
	var maxLon_deg =  damageDetectDistance_m / m_per_deg_lon;
						
	debprint ("Bombable: maxLat = ", maxLat_deg, " maxLon = ", maxLon_deg);
						
	#put these in global hash so they are accessible
						
	attributes[myNodeName].dimensions['maxLat'] = maxLat_deg;
	attributes[myNodeName].dimensions['maxLon'] = maxLon_deg;
						
	# debprint ("Bombable: maxLat = ", attributes[myNodeName].dimensions.maxLat, " maxLon = ", attributes[myNodeName].dimensions.maxLon, " for ", myNodeName );
}


######################### bombable_init ############################
var bombable_init = func (myNodeName = "") {
	debprint ("Bombable: Delaying bombable_init . . . ", myNodeName);
	settimer (func {bombable_init_func(myNodeName);}, 35 + rand(),1);
}

######################### bombable_init_func ############################
# call to make an object bombable
#
# features/parameters are set by a bombableObject and
# a previous call to initialize (above)
var bombable_init_func = func(myNodeName)
{
	#only allow initialization for ai & multiplayer objects
	# in FG 2.4.0 we're having trouble with strange(!?) init requests from
	# joysticks & the like
	var init_allowed = 0;
	if (find ("/ai/models/", myNodeName ) != -1 ) init_allowed = 1;
	if (find ("/multiplayer/", myNodeName ) != -1 ) init_allowed = 1;

	if (init_allowed != 1) {
		debprint ("Bombable: Attempt to initialize a Bombable subroutine on an object that is not AI or Multiplayer; aborting initialization. ", myNodeName);
		return;
	}

						
	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	# if it 1 and we're trying to initialize, something has gone wrong and we abort with a message.
	var inited = getprop(""~myNodeName~"/bombable/initializers/bombable-initialized");
	if (inited == 1) {
		debprint ("Bombable: Attempt to re-initialize bombable_init when it has not been de-initialized; aborting re-initialization. ", myNodeName);
		return;
	}
	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	setprop(""~myNodeName~"/bombable/initializers/bombable-initialized", 1);



						
	debprint ("Bombable: Starting to initialize for "~myNodeName);
	if (myNodeName == "" or myNodeName == nil) {
		myNodeName = cmdarg().getPath();
		debprint ("Bombable: myNodeName blank, re-reading: "~myNodeName);
	}
						
	var node = props.globals.getNode (""~myNodeName);
	var ats = attributes[myNodeName];
	var alts = ats.altitudes;
	var dims = ats.dimensions;
	var vels = ats.velocities;

	var type = node.getName();
						
	setMaxLatLon(myNodeName, dims.damageRadius_m+200);

	var listenerids = [];
						
	#impactReporters is the list of (theoretically) all places in the property
	#tree where impacts/collisions will be reported.  It is set in the main
	#bombableInit function
	foreach (var i; bombable.impactReporters) 
	{
		#debprint ("i: " , i);
		listenerid = setlistener(i, func ( changedImpactReporterNode ) {
			if (!bombableMenu["bombable-enabled"] ) return 0;
		test_impact( changedImpactReporterNode, myNodeName ); });
		append(listenerids, listenerid);
	}

	#we increment this each time we are inited or de-inited
	#when the loopid is changed it kills the timer loops that have that id
	loopid = inc_loopid(myNodeName, "fire");
	#start the loop to check for fire damage
	settimer(func{fire_loop(loopid,myNodeName);},5.2 + rand());
						
	debprint ("Bombable: Effect * bombable * loaded for "~myNodeName~" loopid = "~ loopid);

	#what to do when re-set is selected
	setlistener("/sim/signals/reinit", func {
		if (!bombableMenu["bombable-enabled"] ) return 0;
		resetBombableDamageFuelWeapons (myNodeName);
		if (type == "multiplayer") mp_send_damage(myNodeName, 0);
		debprint ("Bombable: Damage level and smoke reset for "~ myNodeName);
	});
						
	if (type == "multiplayer") 
	{
							
		#set up the mpreceive listener.  The final 0, 0) makes it
		# trigger only when the location has * changed * .  This is necessary
		# because the location is written to each frame, but only changed
		# occasionally.
		listenerid = setlistener(myNodeName~MP_message_pp,mpreceive, 0, 0);
		append(listenerids, listenerid);
							
		#We're using a listener rather than the settimer now, so the line below is removed
		#settimer (func {mpreceive(myNodeName,loopid)}, mpTimeDelayReceive);
		debprint ("Bombable: Setup mpreceive for ", myNodeName);
	}
						
	props.globals.getNode(""~myNodeName~"/bombable/listenerids",1).setValues({"listenerids":listenerids });
						
	return;
}


######################### ground_init ############################

var ground_init = func (myNodeName = "") {

	if (!getprop("/sim/ai/scenario-initialized"))
	{
		settimer (func {ground_init(myNodeName);}, 5, 1);
		return;
	}	
	debprint ("Bombable: Delaying ground_init . . . ", myNodeName);
	settimer (func {bombable.ground_init_func(myNodeName);}, 45 + rand(),1);

}

######################### ground_init_func ############################
# Call to make your object stay on the ground, or at a constant
# distance above ground level--like a jeep or tank that drives along
# the ground, or an aircraft that moves along at, say, 500 ft AGL.
# The altitude will be continually readjusted
# as the object (set up as, say, and AI ship or aircraft moves.
# In addition, for "ships" the pitch will change to (roughly) match
# when going up or downhill.
#
var ground_init_func = func( myNodeName ) {
	#return;
	#only allow initialization for ai & multiplayer objects
	# in FG 2.4.0 we're having trouble with strange(!?) init requests from
	# joysticks & the like
	var init_allowed = 0;
	if (find ("/ai/models/", myNodeName ) != -1 ) init_allowed = 1;
	if (find ("/multiplayer/", myNodeName ) != -1 ) init_allowed = 1;

	if (init_allowed != 1) {
		debprint ("Bombable: Attempt to initialize a Bombable subroutine on an object that is not AI or Multiplayer; aborting initialization. ", myNodeName);
		return;
	}

	var node = props.globals.getNode(myNodeName);
	type = node.getName();
						
	#don't even try to do this to multiplayer aircraft
	if (type == "multiplayer") return;


	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	# if it 1 and we're trying to initialize, something has gone wrong and we abort with a message.
	var inited = getprop(""~myNodeName~"/bombable/initializers/ground-initialized");
	if (inited == 1) 
	{
		debprint ("Bombable: Attempt to re-initialize ground_init when it has not been de-initialized; aborting re-initialization. ", myNodeName);
		return;
	}
						

	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	setprop(""~myNodeName~"/bombable/initializers/ground-initialized", 1);



	alts = attributes[myNodeName].altitudes;
						
						
	#we increment this each time we are inited or de-inited
	#when the loopid is changed it kills the timer loops that have that id
	var loopid = inc_loopid(myNodeName, "ground");
						
	# Add some useful nodes
						
						
						
	#get the object's initial altitude
	var lat = getprop(""~myNodeName~"/position/latitude-deg");
	var lon = getprop(""~myNodeName~"/position/longitude-deg");
	var alt = elev (lat, lon);
						
	#Do some checking for the ground_loop function so we don't always have
	#to check this in that function
	#damageAltAdd is the (maximum) amount the object will descend
	#when it is damaged.
						
	settimer(func {ground_loop(loopid, myNodeName); }, 4.1 + rand());
						
	debprint ("Bombable: Effect * maintain altitude above ground level * loaded for "~ myNodeName);
	# altitude adjustment = ", alts.wheelsOnGroundAGL_ft, " max drop/fall when damaged = ",
	# damageAltAdd, " loopid = ", loopid);
	
	# this loop allows ships to adjust their height according to the vertical speed.  Experiment abandoned 050318
	# if (type == "ship") {							
		# var haloopid = inc_loopid (myNodeName, "height_adjust");
		# settimer (func {height_adjust_loop ( haloopid, myNodeName, .1 + rand()/100); }, 12 + rand());
		# debprint ("Bombable: Effect * adjust height * loaded for "~ myNodeName);

	# }


}
######################## location_init #############################

var location_init = func (myNodeName = "") {
	# function disabled:  incorrect logic
	# debprint ("Bombable: Delaying location_init . . . ", myNodeName);
	debprint ("Bombable: Disabled location_init . . . ", myNodeName);
	# settimer (func {bombable.location_init_func(myNodeName);}, 50 + rand(),1);

}

######################## location_init_func #############################
# Call to make your object keep its location even after a re-init
# (file/reset).  For instance a fleet of tanks, cars, or ships
# will keep its position after the reset rather than returning
# to their initial position.'
#
# Put this nasal code in your object's load:
#      bombable.location_init (cmdarg().getPath())

var location_init_func = func(myNodeName) 
{
	#return;
						
	#only allow initialization for ai & multiplayer objects
	# in FG 2.4.0 we're having trouble with strange(!?) init requests from
	# joysticks & the like
	var init_allowed = 0;
	if (find ("/ai/models/", myNodeName ) != -1 ) init_allowed = 1;
	if (find ("/multiplayer/", myNodeName ) != -1 ) init_allowed = 1;

	if (init_allowed != 1) {
		debprint ("Bombable: Attempt to initialize a Bombable subroutine on an object that is not AI or Multiplayer; aborting initialization. ", myNodeName);
		return;
	}

	var node = props.globals.getNode(myNodeName);
	type = node.getName();
	#don't even try to do this to multiplayer aircraft
	if (type == "multiplayer") return;

						
	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	# if it 1 and we're trying to initialize, something has gone wrong and we abort with a message.
	var inited = getprop(""~myNodeName~"/bombable/initializers/location-initialized");
	if (inited == 1) {
		debprint ("Bombable: Attempt to re-initialize location_init when it has not been de-initialized; aborting re-initialization. ", myNodeName);
		return;
	}
	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	setprop(""~myNodeName~"/bombable/initializers/location-initialized", 1);

						
	#we increment this each time we are inited or de-inited
	#when the loopid is changed it kills the timer loops that have that id
	var loopid = inc_loopid(myNodeName, "location");
						
						
	settimer(func { location_loop(loopid, myNodeName); }, 15.15 + rand());

	debprint ("Bombable: Effect * relocate after reset * loaded for "~ myNodeName~ " loopid = "~ loopid);

}

var attack_init = func (myNodeName = "") {
	if (!getprop("/sim/ai/scenario-initialized"))
	{
		settimer (func {attack_init(myNodeName);}, 5, 1);
		return;
	}
	debprint ("Bombable: Delaying attack_init . . . ", myNodeName);
	settimer (func {bombable.attack_init_func(myNodeName);}, 55 + rand(),1 );
}

############################ attack_init_func ##############################
# Call to make your object turn & attack the main aircraft
#
# Put this nasal code in your object's load:
#      bombable.attack_init (cmdarg().getPath())

var attack_init_func = func(myNodeName) 
{
	#return;
	#only allow initialization for ai & multiplayer objects
	# in FG 2.4.0 we're having trouble with strange(!?) init requests from
	# joysticks & the like
	var init_allowed = 0;
	if (find ("/ai/models/", myNodeName ) != -1 ) init_allowed = 1;
	if (find ("/multiplayer/", myNodeName ) != -1 ) init_allowed = 1;
						
	if (init_allowed != 1) 
	{
		debprint ("Bombable: Attempt to initialize a Bombable subroutine on an object that is not AI or Multiplayer; aborting initialization. ", myNodeName);
		return;
	}

	var node = props.globals.getNode(myNodeName);
	var type = node.getName();
	#don't even try to do this to multiplayer aircraft
	if (type == "multiplayer") {
		debprint ("Bombable: Not initializing attack for multiplayer aircraft; exiting . . . ");
		return;
	}

	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	# if 1 and we're trying to initialize, something has gone wrong and we abort with a message.
	var inited = getprop(""~myNodeName~"/bombable/initializers/attack-initialized");
	if (inited == 1)
	{
		debprint ("Bombable: Attempt to re-initialize attack_init when it has not been de-initialized; aborting re-initialization. ", myNodeName);
		return;
	}
	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	setprop(""~myNodeName~"/bombable/initializers/attack-initialized", 1);
						
	# we increment this each time we are inited or de-inited
	# when the loopid is changed it kills the timer loops that have that id
	var loopid = inc_loopid (myNodeName, "attack");
						
	var atts = attributes[myNodeName].attacks;

	var attackCheckTime = atts.attackCheckTime_sec;
	if (attackCheckTime == nil or attackCheckTime < 0.5) attackCheckTime = 0.5;
						

	atts["loopTime"] = attackCheckTime + rand();						
	settimer( func {attack_loop(loopid, myNodeName); }, atts.loopTime );
						
	#start the speed adjust loop.  Adjust speed up/down depending on climbing/
	# diving, or level flight; only for AI aircraft.
	
	if (type == "aircraft") 
	{							
		var speedAdjust_loopid = inc_loopid (myNodeName, "speed_adjust");
		settimer (func {speed_adjust_loop ( speedAdjust_loopid, myNodeName, .3 + rand() / 30); }, 12 + rand() );
	}
	
	debprint ("Bombable: Effect * attack * loaded for "~ myNodeName~ " loopid = "~ loopid, " attackCheckTime = ", attackCheckTime);

}



######################## weaponsOrientationPositionUpdate #########################
# weaponsOrientationPositionUpdate
# to update the position/angle of weapons attached
# to AI aircraft.  Use for visual weapons effects

var weaponsOrientationPositionUpdate = func (myNodeName, elem) {
	# make weapon and projectile point in the direction of the target
	# the direction is calculated in the checkAim loop
	# first point weapon in the direction of the target
	# and then the projectile in the direction of the weapon

	# no need to do this if any of these are turned off in the bombable menu
	# though we may update weapons_loop to rely on these numbers as well
	if (! getprop ( trigger1_pp~"ai-weapon-fire-visual"~trigger2_pp)
	or ! bombableMenu["bombable-enabled"]
	) return;
	
	var thisWeapon = attributes[myNodeName].weapons[elem];
	var aim = thisWeapon.aim;

	if (!aim.fixed)
	{	
		# first, point the weapon.  The first frame is relative to the model, 
		# the second is lon-lat-alt (x-y-z), aka 'reference frame'
		var newElev = math.asin(aim.weaponDirModelFrame[2]) * R2D;
		var newHeading = math.atan2(aim.weaponDirModelFrame[0], aim.weaponDirModelFrame[1]) * R2D;
		thisWeapon.weaponAngle_deg.heading = newHeading;
		thisWeapon.weaponAngle_deg.elevation = newElev;

		setprop("" ~ myNodeName ~ "/" ~ elem ~ "/cannon-elev-deg" , newElev);
		setprop("" ~ myNodeName ~ "/" ~ elem ~ "/turret-pos-deg" , -newHeading);
		# position in model frame of reference
	}
	
	# next, point the projectile
	# the projectile models follow the aircraft using these orientation and position data from the property tree

	var newElev_ref = math.asin(aim.weaponDirRefFrame[2]) * R2D;
	var newHeading_ref = math.atan2(aim.weaponDirRefFrame[0], aim.weaponDirRefFrame[1]) * R2D;
	setprop("" ~ myNodeName ~ "/" ~ elem ~ "/orientation/pitch-deg", newElev_ref);
	setprop("" ~ myNodeName ~ "/" ~ elem ~ "/orientation/true-heading-deg", newHeading_ref);

	# note weapon offset in m; altitude is in feet
	setprop("" ~ myNodeName ~ "/" ~ elem ~ "/position/altitude-ft",
	getprop("" ~ myNodeName ~ "/position/altitude-ft") + aim.weaponOffsetRefFrame[2] * M2FT);

	setprop("" ~ myNodeName ~ "/" ~ elem ~ "/position/latitude-deg",
	getprop("" ~ myNodeName ~ "/position/latitude-deg") + aim.weaponOffsetRefFrame[1] / m_per_deg_lat); 

	setprop("" ~ myNodeName ~ "/" ~ elem ~ "/position/longitude-deg",
	getprop("" ~ myNodeName ~ "/position/longitude-deg") + aim.weaponOffsetRefFrame[0] / m_per_deg_lon);

		
	# debprint("weaponsOrientationPositionUpdate_loop " ~ elem ~ 
		# sprintf(" newElev =%8.1f pitch-deg =%8.1f newHeading =%8.1f true-heading-deg =%8.1f", 
			# newElev, 
			# newElev_ref,
			# newHeading, 
			# newHeading_ref
		# )			
	# );
}



################# weaponsTrigger_listener ####################
# weaponsTrigger_listener
# Listen when the remote MP aircraft triggers weapons and un-triggers them,
# and show our local visual weapons effect whenever they are triggered
# Todo: Make the visual weapons effect stop triggering when the remote MP
# aircraft is out of ammo
#
var weaponsTrigger_listener = func (changedNode,listenedNode){

	#for now there is only one trigger for ALL AI visual weapons
	# so we just turn it on/off depending on the trigger value
	# TODO: Since there are possibly multiple triggers there is the possibility
	# of the MP aircraft holding both trigger1 and trigger2 and then
	# releasing only trigger2, which will turn off the visual effect
	# for all weapons here.  It would take some logic to fix that little flaw.
	# TODO: there is only one visual effect & one trigger for EVERYTHING for now, so setting the
	# trigger = 1 turns on all weapons for all AI/Multiplayer aircraft.
	# Making it turn on/off individually per weapon per aircraft is going to be a
	# fair-sized job.
	
	# rjw TODO include MP ACs in the stack of projectile tracer models 
	if (!bombableMenu["bombable-enabled"] ) return 0;
	# debprint ("Bombable: WeaponsTrigger_listener: ",changedNode.getValue(), " ", changedNode.getPath());
	if ( changedNode.getValue()) {
		setprop("/bombable/fire-particles/ai-weapon-firing",1);
		} else {
		setprop("/bombable/fire-particles/ai-weapon-firing",0);
	}

}

############################## weapons_init ##############################
# called by model nasal
# Put this nasal code in your object's load:
#      bombable.weapons_init (cmdarg().getPath())

var weapons_init = func (myNodeName = "") {

	if (!getprop("/sim/ai/scenario-initialized"))
	{
		settimer (func {weapons_init(myNodeName);}, 5, 1);
		return;
	}
	debprint ("Bombable: Delaying weapons_init . . . ", myNodeName);

	settimer (func {weapons_init_func(myNodeName);}, 60 + rand(), 1);

}

############################## weapons_init_func ##############################
# Call to make your object fire weapons at the main aircraft
# If the main aircraft gets in the 'fire zone' directly ahead
# of the weapons you set up, the main aircraft will be damaged
#
# Put this nasal code in your object's load:
#      bombable.weapons_init (cmdarg().getPath())
# weapFixed indicates that the weapon can only fire in a fixed direction relative to its platform

var weapons_init_func = func(myNodeName) 
{
	#return;
	myNode = props.globals.getNode(myNodeName);
	type = myNode.getName();
						
	# only allow initialization for ai & multiplayer objects
	# in FG 2.4.0 we're having trouble with strange(!?) init requests from
	# joysticks & the like
	var init_allowed = 0;
	if (find ("/ai/models/", myNodeName ) != -1 ) init_allowed = 1;
	if (find ("/multiplayer/", myNodeName ) != -1 ) init_allowed = 1;
						
	if (init_allowed != 1) {
		debprint ("Bombable: Attempt to initialize a Bombable subroutine on an object that is not AI or Multiplayer; aborting initialization. ", myNodeName);
		return;
	}
						
						
						
	# don't do this for multiplayer . . .
	# if (type == "multiplayer") return;
	# oops . . . now we ARE doing part of this for MP, so they can have the weapons visual effect

	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	# if it 1 and we're trying to initialize, something has gone wrong and we abort with a message.
	var inited = getprop(""~myNodeName~"/bombable/initializers/weapons-initialized");
	if (inited == 1) {
		debprint ("Bombable: Attempt to re-initialize weapons_init when it has not been de-initialized; aborting re-initialization. ", myNodeName);
		return;
	}
						
	#don't do this if the 'weapons' attributes are not included
	var weapsSuccess = 1;
	var ats = attributes[myNodeName];
	if (!contains (ats, "weapons")) { debprint ("no attributes.weapons, exiting"); weapsSuccess = 0;}
	else {
		var weaps = ats.weapons;
		if (weaps == nil or typeof(weaps) != "hash") {
			debprint ("attributes.weapons not a hash"); 
			weapsSuccess = 0; 
		}
	}
						
	if (weapsSuccess == 0) return;   #alternatively we could implement a fake/basic armament here
	#for any MP aircraft that don't have a bombable section.
						
						
	# set to 1 if initialized and 0 when de-inited. Nil if never before inited.
	setprop(""~myNodeName~"/bombable/initializers/weapons-initialized", 1);
						
	var count = getprop ("/bombable/fire-particles/index") ;
	if (count == nil) {
		count = 0; #index of first fire particle for AI aircraft
		}
	var rocketCount = getprop ("/bombable/rockets/index") ;
	if (rocketCount == nil) {
		rocketCount = 0; #index of first rocket for AI aircraft
		}

	foreach (elem;keys (weaps) ) 
	{
		var thisWeapon = weaps[elem]; # a pointer into the attributes hash
		if (thisWeapon["weaponType"] == nil) thisWeapon["weaponType"] = 0;
		# key to allow inclusion of new types of weapons such as rockets

		if (thisWeapon["weaponType"] == 1) 
		{
			if (rocket_init_func (thisWeapon, rocketCount))
			{
				rocketCount += 1;
			}
			else
			{
				debprint ("Weaps: ", myNodeName, " failed to init ", thisWeapon.name);
				delete(weaps, elem);
				continue;
			}
		}
		
		# the weapon particle system is offset from the AI model origin by vector weaponOffset_m, 
		# which is defined in the AI model include file and in the co-ordinates of the model
		# the x-axis points 180 degrees from the direction of travel; the y-axis points right; the z-axis up

		put_tied_weapon
			(
				myNodeName, elem,
				"AI/Aircraft/Fire-Particles/projectile-tracer/projectile-tracer-" ~ count ~ ".xml"
			);
		setprop ("/bombable/fire-particles/projectile-tracer[" ~ count ~ "]/projectile-startsize", thisWeapon.weaponSize_m.start);
		setprop ("/bombable/fire-particles/projectile-tracer[" ~ count ~ "]/projectile-endsize", thisWeapon.weaponSize_m.end);
		setprop ("/bombable/fire-particles/projectile-tracer[" ~ count ~ "]/ai-weapon-firing", 0); 
		setprop ("/bombable/fire-particles/projectile-tracer[" ~ count ~ "]/ai-weapon-firing", 0); 
		setprop ("/bombable/fire-particles/projectile-tracer[" ~ count ~ "]/offset-x", thisWeapon.weaponOffset_m.x); 
		setprop ("/bombable/fire-particles/projectile-tracer[" ~ count ~ "]/offset-y", thisWeapon.weaponOffset_m.y); 
		setprop ("/bombable/fire-particles/projectile-tracer[" ~ count ~ "]/offset-z", thisWeapon.weaponOffset_m.z); 
		
		# form vector for weapon direction, weapDir
		var weapAngles = thisWeapon.weaponAngle_deg;
		var cosWeapElev = math.cos(weapAngles.elevation* D2R);
		var weapDir = [
			cosWeapElev * math.sin(weapAngles.heading* D2R),
			cosWeapElev * math.cos(weapAngles.heading* D2R),
			math.sin(weapAngles.elevation* D2R)
		];		
		# set turret and gun to their default positions
		setprop ("" ~ myNodeName ~ "/" ~ elem ~ "/turret-pos-deg", weapAngles.heading);
		setprop ("" ~ myNodeName ~ "/" ~ elem ~ "/cannon-elev-deg", weapAngles.elevation);

		# store initial values
		weapAngles["initialHeading"] = weapAngles.heading;
		weapAngles["initialElevation"] = weapAngles.elevation;

		var weapFixed = (weapAngles.elevationMin == weapAngles.elevationMax) and (weapAngles.headingMin == weapAngles.headingMax) and (thisWeapon["weaponType"] != 1);
		# do not include rockets since a different target can be assigned to each 'fixed' rocket 
			
		thisWeapon["aim"] = {
			nHit:0, 
			weaponDirModelFrame:weapDir, 
			weaponOffsetRefFrame:[0,0,0], 
			weaponDirRefFrame:[0,0,-1], #-1 flag to show not initialized 
			lastTargetVelocity:[0,0,0],
			interceptSpeed:0,
			fixed:weapFixed, 
			target:-1, #index of object to shoot at; -1 flag to show not initialized
			}; 
		# aim records direction weapon is pointing
		# in the frame of reference of model and the frame of reference of the scene
		
		thisWeapon["fireParticle"] = count;
		# new key to link the weapon to a fire particle

		if (thisWeapon["maxMissileSpeed_mps"] == nil) thisWeapon["maxMissileSpeed_mps"] = 300;
			
		if (thisWeapon["roundsPerSec"] == nil) thisWeapon["roundsPerSec"] = 3; # firing rate 
		if (thisWeapon["nRounds"] == nil) thisWeapon["nRounds"] = 180; # number of rounds 
		thisWeapon["ammo_seconds"] = thisWeapon.nRounds / thisWeapon.roundsPerSec; # time weapon can fire til out of ammo 
		if (thisWeapon["accuracy"] == nil) thisWeapon["accuracy"] = 3; # angular variation of fire in degrees 
		thisWeapon.accuracy *= D2R; # covert to radians	

		# key to indicate the position of the weapon is determined by the position of a parent weapon, e.g. turret and subturret
		if (thisWeapon["parent"] == nil) 
		{
			thisWeapon["parent"] = "";
		}
		elsif (!contains ( weaps, thisWeapon["parent"] ))
		{
			debprint ("Weaps: ", myNodeName ~ "/" ~ elem ~ "parent not found - weapon failed to init");
			delete(weaps, elem);
			continue;
		}
		elsif (thisWeapon["parent"] == "" ~ elem)
		{
			debprint ("Weaps: ", myNodeName ~ "/" ~ elem ~ "parent declared as self - weapon failed to init");
			delete(weaps, elem);
			continue;
		}	
		thisWeapon.destroyed = 0;
		debprint ("Weaps: ", myNodeName, " initialized ", thisWeapon.name);
		count += 1;
	}
	
	setprop ("/bombable/fire-particles/index" , count) ; #next unassigned fire particle
	setprop ("/bombable/rockets/index" , rocketCount) ; #next unassigned rocket

	if (rocketCount) 
	{
		if (ats.dimensions["safeDistance_m"] == nil) ats.dimensions["safeDistance_m"] = 200;
	}

	props.globals.getNode(""~myNodeName~"/bombable/weapons/listenerids",1);
	#do the visual weapons effect setup for multiplayer . . .
						
	if (type == "multiplayer") {

		debprint ("Bombable: Setting up MP weapons for ", myNodeName, " type ", type);
							
		#setup alias for remote weapon trigger(s) and a listener to trigger
		# our local weapons visual effect whenever it a trigger is set to 1
		# sets /ai/models/multiplayer[X]/controls/armament/triggerN (for n = 0..10)
		# as alias of the multiplayer generic int0..10 properties & then sets
		# up a listener for each of them to turn the visual weapons effect on
		# whenever a trigger is pulled.
		listenerids = [];
		for (n = 0;n < 10;n += 1) {

			var genericintNum = n+10;
								
			# OK, the idea of an alias sounded great but apparently listeners don't work on aliases (???)
			# if (n == 0) var appendnum = ""; else var appendnum = n;
			# myNode.getNode("controls/armament/trigger"~appendnum, 1).
			# listenNodeName = ""~myNodeName~"/controls/armament/trigger";
			# alias(myNode.getNode("sim/multiplay/generic/int["~genericintNum~"]"));
			# debprint ("Bombable: Setting up listener for ", listenNodeName ~ appendnum);
			# listenerid = setlistener ( listenNodeName ~ appendnum, weaponsTrigger_listener, 1, 0 );  #final 0 makes it listen only when the value is changed
								
			#So we're doing it the basic way: just listen directly to the generic/int node, 10-19:
			listenerid = setlistener (""~myNodeName~"/sim/multiplay/generic/int["~genericintNum~"]", weaponsTrigger_listener, 1, 0 );  
			#final 0 makes it listen only when the listened value is changed; for MP it is written every frame but only changed occasionally
			append(listenerids, listenerid);
		}
		props.globals.getNode(""~myNodeName~"/bombable/weapons/listenerids",1).setValues({listenerids: listenerids});
	}
	#don't do this bit (AI logic for automatic firing of weapons) for multiplayer, only for AI aircraft . . .

	
	
	if (type != "multiplayer") {
		#overall height & width of main aircraft in meters
		# TODO: Obviously, this needs to be set per aircraft in an XML file, along with aircraft
		# specific damage vulnerability etc.
		# the target size depends on its orientation relative to the shooter
		# assume the target is flying horizontally but on any heading relative to the shooter 
		# pRound is calculated on the average solid angle subtended by the target  
		# recalculate the size if new target assigned

		setWeaponPowerSkill (myNodeName);
		stores.fillWeapons (myNodeName, 1);						
							
		#we increment this each time we are inited or de-inited
		#when the loopid is changed it kills the timer loops that have that id
		var loopid = inc_loopid (myNodeName, "weapons");
		settimer (  func { weapons_loop (loopid, myNodeName)}, 5 + rand());
	}

	debprint ("Bombable: Effect * weapons * loaded for ", myNodeName);

						
}

############################## clamp ##############################
# clamps value of a between minA and maxA
# returns clamped value

var clamp = func(a, minA, maxA)
{
	if (a < minA) return (minA) ;
	if (a > maxA) return (maxA) ;
	return (a) ;
}

############################## rocketParmCheck ##############################
# checks rocket parameters supplied by user in include file for AI object carrying rockets
# low speed turns are limited by control surfaces; high speed turns by max allowed G force

var rocketParmCheck = func( thisWeapon )
{
	var weaponVars =
	[
		{name: "burn1",								default: 20.0,		lowerBound: 0.1,		upperBound: 20.0		}, # burn time of first rocket stage in sec
		{name: "burn2",								default: 0.0,		lowerBound: 0.0,		upperBound: 3600.0		}, # burn time of second rocket stage in sec
		{name: "burn3",								default: 0.0,		lowerBound: 0.0,		upperBound: 3600.0		}, # burn time of second rocket stage in sec
		{name: "massFuel_1",						default: 100.0,		lowerBound: 0.0,		upperBound: 1000.0		}, # kg fuel stage 1. Zero to release rocket pre-ignition so as to make Stage 1 freefall
		{name: "massFuel_2",						default: 100.0,		lowerBound: 0.0,		upperBound: 1000.0		}, # kg fuel stage 2
		{name: "massFuel_3",						default: 100.0,		lowerBound: 0.0,		upperBound: 1000.0		}, # kg fuel stage 3
		{name: "specificImpulse1",					default: 250.0,		lowerBound: 200.0,		upperBound: 1000.0		}, # sec, measured per unit weight in N
		{name: "specificImpulse2",					default: 250.0,		lowerBound: 200.0,		upperBound: 1000.0		}, # sec, measured per unit weight in N
		{name: "specificImpulse3",					default: 250.0,		lowerBound: 200.0,		upperBound: 1000.0		}, # sec, measured per unit weight in N
		{name: "launchMass",						default: 500.0,		lowerBound: 200.0,		upperBound: 1000.0		}, # weapon mass during flight accounting for fuel depletion
		{name: "maxMissileSpeed_mps",				default: 500.0,		lowerBound: 300.0,		upperBound: 1200.0		}, # used to estimate initial intercept time
		{name: "minTurnSpeed_mps",					default: 30.0,		lowerBound: 25.0,		upperBound: 50.0		}, # turn disabled below this speed - regime I
		{name: "speedX",							default: 4.0,		lowerBound: 2.0,		upperBound: 10.0		}, # multiple of minTurnSpeed when maxTurnRate achieved - turn rate increases linearly with speed in regime II
		{name: "maxTurnRate",						default: 30.0*D2R,	lowerBound: 20.0*D2R,	upperBound: 45.0*D2R	}, # turn rate is constant in regime III
		{name: "maxG",								default: 300.0,		lowerBound: 200.0,		upperBound: 400.0		}, # maximum G force the missile can withstand in mps limiting turn rate at high speeds - turn rate varies inversely with speed in regime IV
		{name: "AoA",								default: 13.0*D2R,	lowerBound: 8.0*D2R,	upperBound: 15.0*D2R	}, # angle of attack to maximise lift, typically between 10 and 15 degrees turn disabled below this speed - dependent on whether missile has vectored thrust
		{name: "timeNoTurn",						default: 1.5,		lowerBound: 0.5,		upperBound: 3.0			}, # seconds after launch when not possible to turn
		{name: "length",							default: 2.0,		lowerBound: 1.0,		upperBound: 5.0		    }, # length of rocket
		{name: "lengthTube",						default: 3.0,		lowerBound: 0.0,		upperBound: 10.0		}, # length of launch tube or guide rail
		{name: "area",								default: 0.04,		lowerBound: 0.0225,		upperBound: 0.09		}, # effective area of rocket in metres squared; note length to diameter is fixed
	];

	var nBurn = 0;
	var nSpecificImpulse = 0;
	var nMassFuel = 0;
	# check nThrust, nBurn, nMassFuel and nSpecificImpulse
	forindex(i; weaponVars) 
	{
		var gotValue = (thisWeapon[weaponVars[i].name] != nil);
		for (var j=0; j < 3; j = j+1) {
			if ((weaponVars[i].name == "burn" ~ j ) and gotValue) nBurn += 1;
			if ((weaponVars[i].name == "massFuel_" ~ j ) and gotValue) nMassFuel += 1;
			if ((weaponVars[i].name == "specificImpulse" ~ j ) and gotValue) nSpecificImpulse += 1;
		}
	}


	# check rocket parameters have been specified in include AI model include file
	# provide default values if not specified
	# check within bounds
	forindex(i; weaponVars) 
	{
		if (thisWeapon[weaponVars[i].name] == nil) 
		{
			thisWeapon[weaponVars[i].name] = weaponVars[i].default;
			debprint ("Bombable: " ~ thisWeapon.name ~ " " ~ weaponVars[i].name ~ " set to " ~ weaponVars[i].default);
		}
		thisWeapon[weaponVars[i].name] = clamp
		(
			thisWeapon[weaponVars[i].name],
			weaponVars[i].lowerBound,
			weaponVars[i].upperBound			
		);
	}

	if (nBurn == 0) 
	{
		debprint ("Bombable: warning: no stages specified, using default values");
	}
	elsif ((nBurn != nMassFuel) or (nMassFuel != nSpecificImpulse)) 
	{
		debprint ("Bombable: error: specify burn time, mass of fuel and specific impulse of fuel for each stage (up to 3)");
		return(0);
	}

	# check fuel mass / total mass limit not exceeded
	var massFraction = 0;
	for (var j=1; j < 4; j = j + 1) 
	{
		if (thisWeapon["burn"~j] != 0) 
		{
			thisWeapon["fuelRate"~j] = 0;
			massFraction += thisWeapon["massFuel_"~j];
			thisWeapon["fuelRate"~j] = thisWeapon["massFuel_"~j] / thisWeapon["burn"~j];
		}
	}
	massFraction /= thisWeapon.launchMass;
	var maxMassFraction = 0.5;
	if ( massFraction > maxMassFraction )
	{ 
		debprint ("Bombable: warning: total launch mass must be at least 2x fuel mass, mass of fuel scaled down");
		for (var j=1; j < 4; j = j + 1) thisWeapon["massFuel_"~j] *= ( maxMassFraction / massFraction);
	}

	# calculate thrust from burn time and fuel mass and SPI
	for (var j=1; j < 4; j = j + 1) 
	{
		thisWeapon["thrust"~j] = (thisWeapon["burn"~j] != 0) ? thisWeapon["massFuel_"~j] * grav_mpss * thisWeapon["specificImpulse"~j] / thisWeapon["burn"~j] : 0.0;
		debprint (
		sprintf
			(
				"Bombable: %s stage %i thrust %6.0fN calculated from burn time %6.0fs, fuel mass %6.0fkg and specific impulse %6.0fs",
				thisWeapon.name,
				j,
				thisWeapon["thrust"~j],
				thisWeapon["burn"~j],
				thisWeapon["massFuel_"~j],
				thisWeapon["specificImpulse"~j]
			)
		);
	}

	# calculate cumulative times
	thisWeapon["burn_1_2"] = thisWeapon.burn1 + thisWeapon.burn2;
	thisWeapon["burn_1_2_3"] = thisWeapon.burn1 + thisWeapon.burn2 + thisWeapon.burn3;
	thisWeapon["mass"] = thisWeapon.launchMass; # init here but not strictly needed

	return (1) ;
}


############################## rocket_init_func ##############################
# rockets are a special type of weapon (type = 1) requiring extra initialisation
# thisWeapon is a pointer to the attributes hash
# two sets of parameters:  user supplied and internal

var rocket_init_func = func (thisWeapon, rocketCount) {			
	# check parameters supplied by user
	if (!rocketParmCheck( thisWeapon )) return (0);

	# set-up internal parameters

	thisWeapon["axialForce"] = 0; # drag term
	thisWeapon["liftDragRatio"] = 1; # lift drag ratio used to determine glide performance
	
	thisWeapon["timeRelease"] = 0; # determined by length of launch tube or rail

	thisWeapon["velocities"] = 
	{
		missileV_mps: [0,0,0],
		lastMissileSpeed: 0,
		thrustDir:[0,0,0],
	};

	thisWeapon["position"] = 
	{
		latitude_deg: 0,
		longitude_deg: 0,
		altitude_ft: 0,
	};

	# set-up buffer to store intermediate positions of rocket
	var wayPoint = {lon:0, lat:0, alt:0, pitch:0, heading:0}; # template
	var new_wayPoint = func {
		return {parents:[wayPoint] };
		}

	var fp = [];
	setsize(fp, N_STEPS);
	forindex(var i; fp)
		fp[i] = new_wayPoint(); # flight pathvector used to store intermediate rocket locations

	thisWeapon["controls"] = 
	{
		flightTime: 0,
		flightPath: fp,
		launched: 0,
		engine: 0,
		abortCount: 0,
	};

	var maxLimit = LOOP_TIME * 40.0 * D2R; # initial value for maxRate turn 

	# set-up parameters for PID controller - PID values are reset on rocket launch
	var pidVals = 
	{
		Kp: 0.7,
		Ki: 0.002,
		Kd: 0.0,
		limMaxInt: maxLimit * 0.75 , # Integrator limits to be set below
		limMinInt: -maxLimit * 0.75 ,
		tau: 0.25 , # Derivative low-pass filter time constant secs
		limMax: maxLimit ,
		limMin: -maxLimit ,
		differentiator: 0.0 ,
		integrator: 0.0 ,
		prevError: 0.0 ,
		prevMeasurement: 0.0 ,
		out: 0.0 ,	
	};
	var new_pidVals = func 
	{
		return {parents:[pidVals] };
	}

	thisWeapon["pidData"] = 
	{
		phi: [], 
		theta: [],
	};
	thisWeapon.pidData.theta = new_pidVals();
	thisWeapon.pidData.phi = new_pidVals();

	thisWeapon["rocketsIndex"] = rocketCount;

	thisWeapon["loopCount"] = 0; # counts calls to guideRocket

	thisWeapon["cN"] = 0;
	thisWeapon["cD0"] = 0;
	thisWeapon["airDensity"] = 0.0;
	thisWeapon["lastAlt_m"] = 0.0;
	thisWeapon["rhoAby2"] = 0.0;
	thisWeapon["speedSound"] =0.0;

	return(1);

}


#####################################################
#unload function (delete/destructor) for initialize
#
#typical usage:
# < PropertyList > 
#...
# < nasal > 
#...
#  < unload > 
#      bombable.initialize_del (cmdarg().getPath(), id);
#  < /unload
# < /nasal > 
# < /PropertyList > 
# Note: As of Bombable 3.0m, id is not used for anything
# (listenerids are stored as nodes, which works much better)

var initialize_del = func(myNodeName, id = "") {
						
	#set this to 0/false when de-inited
	setprop(""~myNodeName~"/bombable/initializers/attributes-initialized", 0);
	debprint ("Bombable: Effect initialize unloaded for "~ myNodeName );
						
}



#####################################################
#unload function (delete/destructor) for bombable_init
#
#typical usage:
# < PropertyList > 
#...
# < nasal > 
#...
#  < unload > 
#      bombable.bombable_del (cmdarg().getPath(), id);
#  < /unload
# < /nasal > 
# < /PropertyList > 
# Note: As of Bombable 3.0m, id is not used for anything
# (listenerids are stored as nodes, which works much better)

var bombable_del = func(myNodeName, id = "") {
						
	#we increment this each time we are inited or de-inited
	#when the loopid is changed it kills the timer loops that have that id
	inc_loopid(myNodeName, "fire");
	inc_loopid(myNodeName, "attributes");
						
						
	listids = props.globals.getNode(""~myNodeName~"/bombable/listenerids",1).getValues();
						
	#remove the listener to check for impact damage
	if (listids != nil and contains (listids, "listenerids")) {
		foreach (k;listids.listenerids) { removelistener(k); }
		props.globals.getNode(""~myNodeName~"/bombable/listenerids",1).removeChildren();
	}
						
						
	#this loop will be killed when we increment loopid as well
	#settimer(func { fire_loop(loopid, myNodeName); }, 5.0+rand());
						
	#set this to 0/false when de-inited
	setprop(""~myNodeName~"/bombable/initializers/bombable-initialized", 0);
	debprint ("Bombable: Effect * bombable * unloaded for "~ myNodeName~ " loopid2 = ", loopid2);
						

}

########################## ground_del ###########################
# del/destructor function for ground_init
# Put this nasal code in your object's unload:
# bombable.bombable_del (cmdarg().getPath());
var ground_del = func(myNodeName) {
						
	# we increment this each time we are inited or de-inited
	# when the loopid is changed it kills the timer loops that have that id
	var loopid = inc_loopid(myNodeName, "ground");
	var haloopid = inc_loopid(myNodeName, "height_adjust");

	#set this to 0/false when de-inited
	setprop(""~myNodeName~"/bombable/initializers/ground-initialized", 0);
						
	debprint ("Bombable: Effect * drive on ground * unloaded for "~ myNodeName~ " loopid = "~ loopid);

						
}

######################### location_del ############################
# del/destructor for location_init
# Put this nasal code in your object's unload:
#      bombable.location_del (cmdarg().getPath());

var location_del = func(myNodeName) {
						
						
	#we increment this each time we are inited or de-inited
	#when the loopid is changed it kills the timer loops that have that id
	var loopid = inc_loopid(myNodeName, "location");
						
	#set this to 0/false when de-inited
	setprop(""~myNodeName~"/bombable/initializers/location-initialized", 0);

	debprint ("Bombable: Effect * relocate after reset * unloaded for "~ myNodeName~ " loopid = "~ loopid);

}

########################## attack_del ###########################
# del/destructor for attack_init
# Put this nasal code in your object's unload:
#      bombable.location_del (cmdarg().getPath());

var attack_del = func(myNodeName) 
{
	#we increment this each time we are inited or de-inited
	#when the loopid is changed it kills the timer loops that have that id
	var loopid = inc_loopid(myNodeName, "attack");
	var speedAdjust_loopid = inc_loopid(myNodeName, "speed_adjust");
						
	#set this to 0/false when de-inited
	setprop(""~myNodeName~"/bombable/initializers/attack-initialized", 0);

	debprint ("Bombable: Effect * attack * unloaded for "~ myNodeName~ " loopid = "~ loopid);

}

########################## weapons_del ###########################
# del/destructor for weapons_init
# Put this nasal code in your object's unload:
#      bombable.location_del (cmdarg().getPath());

var weapons_del = func(myNodeName) 
{
	#set this to 0/false when de-inited
	setprop(""~myNodeName~"/bombable/initializers/weapons-initialized", 0);
						
	#we increment this each time we are inited or de-inited
	#when the loopid is changed it kills the timer loops that have that id
	var loopid = inc_loopid(myNodeName, "weapons");
	var loopid2 = inc_loopid(myNodeName, "weaponsOrientation");

	listids = props.globals.getNode(""~myNodeName~"/bombable/weapons/listenerids",1).getValues();

	#remove the listener to check for impact damage
	if (listids != nil and contains (listids, "listenerids")) {
		foreach (k;listids.listenerids) { removelistener(k); }
	}
	props.globals.getNode(""~myNodeName~"/bombable/weapons/listenerids",1).removeChildren();

	debprint ("Bombable: Effect * weapons * unloaded for "~ myNodeName~ " weapons loopid = "~ loopid ~
	" and weaponsOrientation loopid = "~loopid2);
}


###########################################################
# initializers
#
#
# Turn fire/smoke on globally for the fire-particles system.
# As soon as a fire-particle model is placed it will
# start burning.  To stop it from burning, simply remove the model.
# You can turn off all smoke/fires globally by setting the trigger to false


var countmsg = 0;
var broadcast = nil;
var Binary = nil;
var seq = 0;
var rad2degrees = 180/math.pi;
var feet2meters = .3048;
var meters2feet = 1/feet2meters;
var nmiles2meters = 1852;
var meters2nmiles = 1/nmiles2meters;
var knots2fps = 1.68780986;
var knots2mps = knots2fps * feet2meters;
var fps2knots = 1/knots2fps;
var grav_fpss = 32.174;
var grav_mpss = grav_fpss * feet2meters;
var PIBYTWO = math.pi / 2.0;
var PIBYTHREE = math.pi / 3.0;
var PIBYFOUR = math.pi / 4.0;
var PIBYSIX = math.pi / 6.0;
var PI = math.pi;
var TWOPI = math.pi * 2.0;
var bomb_menu_pp = "/bombable/menusettings/";
var bombable_settings_file = getprop("/sim/fg-home") ~ "/state/bombable-startup-settings.xml";

var bomb_menuNum = -1; #we set this to -1 initially and then the FG menu number when it is assigned

var trigger1_pp = "" ~ bomb_menu_pp ~ "fire-particles/";
var trigger2_pp = "-trigger";
var burning_pp = "-burning";
var life1_pp = "/bombable/fire-particles/";
var life2_pp = "-life-sec";
var burntime1_pp = "/bombable/fire-particles/";
var burntime2_pp = "-burn-time";
var attributes_pp = "/bombable/attributes";
var vulnerabilities_pp = attributes_pp ~ "/vulnerabilities/";
var GF_damage_pp = vulnerabilities_pp ~ "gforce_damage/";
var GF_damage_menu_pp = bomb_menu_pp ~ "gforce_damage/";
					
var MP_share_pp = bomb_menu_pp ~ "/MP-share-events/";
var MP_broadcast_exists_pp = "/bombable/mp_broadcast_exists/";
var screenHProp = nil;

records.init();

var tipArgTarget = nil;
var tipArgSelf = nil;
var currTimerTarget = 0;
var currTimerSelf = 0;
var tipMessageAI = "\n\n\n\n";
var tipMessageMain = "\n\n\n\n";

var lockNum = 0;
var lockWaitTime = 1;
var masterLockWaitTime = .3;
var crashListener = 0;

#set initial m_per_deg_lon & lat
var alat_deg = 45;
var aLat_rad = alat_deg* D2R;
var m_per_deg_lat = 111699.7 - 1132.978 * math.cos (aLat_rad);
var m_per_deg_lon = 111321.5 * math.cos (aLat_rad);


#where we'll save the attributes for each AI object & the main aircraft, too
var attributes = {};
#global variable used for sighting weapons
var LOOP_TIME = 0.25; # timing of weapons loop and guide rocket
var N_STEPS = 8; # resolution of flight path calculation
var ot = emexec.OperationTimer.new("VSD");
var handicap = 0; #percentage handicap for side (1)

# List of nodes that listeners will use when checking for impact damage.
# FG aircraft use a wide variety of nodes to report impact of armament
# So we try to check them all.  There is no real overhead to this as
# only the one(s) active with a particular aircraft will ever get any activity.
# This should make all aircraft in the CVS version of FG (as of Aug 2009),
# which have armament that reports an impact, work with bombable.nas AI
# objects.
#
var impactReporters = [
"ai/models/model-impact",  #this is the FG default reporter
"sim/armament/weapons/impact",
"sim/ai/aircraft/impact/bullet",
"sim/ai/aircraft/impact/gun",
"sim/ai/aircraft/impact/cannon",
"sim/model/bo105/weapons/impact/MG",
"sim/model/bo105/weapons/impact/HOT",
"sim/ai/aircraft/impact/droptank",
"sim/ai/aircraft/impact/bomb"
];

####################################
# Set up initial variables for the mpsend/receive/queue system
#
# The location we use for exchanging the messages
# send at MP_message_pp and receive at myNodeName~MP_message_pp
# ie, "/ai/models/multiplayer[3]"~MP_message_pp
var MP_message_pp = "/sim/multiplay/generic/string[9]";
var msgTable = {};
# we'll make delaySend 2X delayReceive--should make message receipt more reliable
var mpTimeDelayReceive = .12409348; #delay between checking for mp messages in seconds
var mpTimeDelaySend = .25100234; #delay between sending messages.
var mpsendqueue = [];
settimer (func {mpprocesssendqueue()}, 5.2534241); #wait ~5 seconds before initial send

# Add damage when aircraft is accelerated beyond reasonable bounds
var damageCheckTime = 1 + rand()/10;
settimer (func {damageCheck () }, 60.11); #wait 30 sec before first damage check because sometimes there is a very high transient g-force on initial startup

####################################
# global variables for managing targets for AI objects
var teams = 
{
	A:{indices: [0], target: nil, count: 1},
}; 
# the main AC is assigned team A, side 0 and index 0
var allPlayers = 
[
	[0],
	[]
];
var nodes = [""]; #1st element is main AC

settimer (func 
{
	mainStatusPopupTip ("Pan around you. The scenario does not load until you have seen the AI objects  . . .", 15 );
	debprint ("Bombable: Delaying start scenario . . . ", getprop("/sim/ai/scenario"));
}, 5);

bombableMenu = {}; # used for menu items accessed frequently

setprop("/sim/ai/scenario-initialized", 0);

settimer (func { waitForAI() }, 5); #wait till objects loaded



#################################### bombableInit ####################################
var bombableInit = func {
	debprint("Bombable: Initializing variables.");
	screenHProp = props.globals.getNode("/sim/startup/ysize");
	tipArgTarget = props.Node.new({ "dialog-name" : "PopTipTarget" });
	tipArgSelf = props.Node.new({ "dialog-name" : "PopTipSelf" });
						
	if ( ! getprop("/sim/ai/enabled") ) {
		var msg = "Bombable: WARNING! The Bombable module is active, but you have disabled the
		entire FlightGear AI system using --disable-ai-models.  You will not be able to see
		any AI or Multiplayer objects or use Bombable.  To fix this problem, remove
		--disable-ai-models from your command line (or check/un-check the appropriate item in
		your FlightGear startup
		manager) and restart.";
		print (msg);
		#mainStatusPopupTip (msg, 10 );
	}

						
	# read any existing bombable-startup-settings.xml  file if it exists
	# getprop("/sim/fg-home") = fg-home directory
						
	# for some reason this isn't working; trying a 5 sec delay to
	# see if that fixes it.  Something is maybe coming along
	# afterward and overwriting the values?
	# settimer (setupBombableMenu, 5.12);
	setupBombableMenu();
						
	# Add some useful nodes
	# these are for the "mothership" not the AI or MP objects
						
	setprop ("/bombable/fire-particles/smoke-startsize", 11.0);
	setprop ("/bombable/fire-particles/smoke-endsize", 50.0);
	setprop ("/bombable/fire-particles/smoke-startsize-small", 6.5);
	setprop ("/bombable/fire-particles/smoke-endsize-small", 40);
						
	setprop ("/bombable/fire-particles/smoke-startsize-very-small", .316);
	setprop ("/bombable/fire-particles/smoke-endsize-very-small", 21);
						
	setprop ("/bombable/fire-particles/smoke-startsize-large", 26.0);
	setprop ("/bombable/fire-particles/smoke-endsize-large", 150.0);
	setprop ("/bombable/fire-particles/flack-startsize", 0.25);
	setprop ("/bombable/fire-particles/flack-endsize", 1.0);
						
	props.globals.getNode(bomb_menu_pp ~ "fire-particles/fire-trigger", 1).setBoolValue(1);
	# props.globals.getNode(bomb_menu_pp ~ "fire-particles/flack-trigger", 1).setBoolValue(0);

	#set attributes for main aircraft
	attributesSet = getprop (""~attributes_pp~"/attributes_set");
	if (attributesSet == nil or ! attributesSet ) setAttributes ();
	setprop("/bombable/attributes/damage", 0); # mirrors the value in attributes hash

	# turn on the loop to occasionally re-calc the m_per_deg lat & lon
	# must be done before setMaxLatLon
	var loopid = inc_loopid("", "update_m_per_deg_latlon");
	settimer (func { update_m_per_deg_latlon_loop(loopid);}, 5.5435);

	# sets max lat & lon for test_impact for main aircraft
	settimer (func { setMaxLatLon("", 500);}, 6.2398471);

						
	# this is zero if no AI or MP models have impact detection loaded, and > 0 otherwise
	var numModelImpactListeners = 0;
						
	#adds the main aircraft to the impact report detection list
	foreach (var i; bombable.impactReporters) 
	{
		#debprint ("i: " , i);
		listenerid = setlistener(i, func ( changedImpactReporterNode ) {
		if (!bombableMenu["bombable-enabled"] ) return 0;
		test_impact( changedImpactReporterNode, "" ); });
		#append(listenerids, listenerid);
	}
						

						
	#if (getprop (""~bomb_menu_pp~"debug") == nil ) {
		#  setprop (bomb_menu_save_lock, 1); #save_lock prevents this change from being written to the menu save file
		#	  props.globals.getNode(bomb_menu_pp~"debug", 1).setBoolValue(0);
		#	setprop (bomb_menu_save_lock, 0);
	#}
						
	#turn on debug flag (for testing)
	#  setprop (bomb_menu_save_lock, 1); #save_lock prevents this change from being written to the menu save file
	#props.globals.getNode(bomb_menu_pp~, 1).setBoolValue(1);
	#  setprop (bomb_menu_save_lock, 0); #save_lock prevents this change from being written to the menu save file
						
	#we increment this each time we are inited or de-inited
	#when the loopid is changed it kills the timer loops that have that id
	var loopid = inc_loopid("", "fire");
	settimer(func{fire_loop(loopid,"");},5.04 + rand());
						
	#what to do when re-set is selected
	setlistener("/sim/signals/reinit", func 
	{
		reset_damage_fires ();
		#for some reason this isn't work; trying a 5 sec delay to
		# see if that fixes it
		#settimer (setupBombableMenu, 5.32);
		setupBombableMenu();
	});
						
	# action to take when main aircraft crashes (or un-crashes)
	setlistener("/sim/crashed", func {
		if (getprop("/sim/crashed")) 
		{
							
			if (!bombableMenu["bombable-enabled"] ) return 0;
			mainAC_add_damage(1, 1, "crash", "You crashed!");   #adds the damage to the main aircraft
								
			debprint ("Bombable: You crashed - on fire and damage set to 100%");
								
			#experimental/doesn't quite work right yet
			#aircraftCrash(""); #Experimental!
		} 
		else
		{
			debprint ("Bombable: Un-crashed--resetting damage & fires.");
			reset_damage_fires ();
		}
	});
						
	#whenever the main aircraft's damage level, fire or smoke levels are updated,
	# broadcast the updated damage level via MP, but with a delay
	# (delay is because the mp_broadcast system seems to get overwhelmed)
	# when a lot of firing is going on)
	#
	setlistener("/bombable/attributes/damage", func {
		if (!bombableMenu["bombable-enabled"] ) return 0;
		settimer (func {mp_send_main_aircraft_damage_update (0)}, 4.36);
							
	});
	setlistener("/bombable/fire-particles/fire-burning", func {
		if (!bombableMenu["bombable-enabled"] ) return 0;
		settimer (func {mp_send_main_aircraft_damage_update (0)}, 3.53);
							
	});
	setlistener("/bombable/fire-particles/damagedengine-burning", func {
		if (!bombableMenu["bombable-enabled"] ) return 0;
		settimer (func {mp_send_main_aircraft_damage_update (0)}, 4.554);
							
	});

					
						
	print ("Bombable (ver. "~ bombableVersion ~") loaded - bombable, weapons, damage, fire, and explosion effects");

	#we save this for last because mp_broadcast doesn't exist for some people,
	# so runtime error & exit at this point for them.
						
	props.globals.getNode(MP_broadcast_exists_pp, 1).setBoolValue(0);
						
	# is multiplayer enabled (overall for FG)?
	if ( getprop("/sim/multiplay/txhost") ) 
	{
		Binary = mp_broadcast.Binary;
		print("Bombable: Bombable successfully set up and enabled for multiplayer dogfighting (you can disable Multiplayer Bombable in the Bombable menu)");
		props.globals.getNode(MP_broadcast_exists_pp, 1).setBoolValue(1);
	}
						
	#broadcast = mp_broadcast.BroadcastChannel.new(msg_channel_mpp, parse_msg, 0);
	#if (broadcast == nil) print ("Bombable: Error, mp_broadcast was not set up correctly");
	#else {
							
	#};
	#test_msg();


}


#we do the setlistener to wait until various things in FG are initialized
# which the functions etc in bombableInit depend on.  Then we wait an additional 15 seconds

var fdm_init_listener = _setlistener("/sim/signals/fdm-initialized", func {
	removelistener(fdm_init_listener);
	bombableInit();
	print("Bombable initalized");

});


########################## reduceRPM ###########################

var reduceRPM = func(myNodeName) {
	# Spin down engine.  Preferentially spin down engines already damaged 
	var engineRevs = [0, 0, 0, 0, 0, 0];
	var revs = 0;
	for (var noEngine = 0; noEngine < 6; noEngine  +=  1) {
		engineRevs[noEngine] = getprop(""~myNodeName~"/engines/engine["~noEngine~"]/rpm");
		if (engineRevs[noEngine] == nil) break;
		#debprint("Bombable: revs = ",revs);
		}
	# debprint("Bombable: noEngines for " ~ myNodeName ~ " = ",noEngine);
	if (noEngine == 0) return;
	var offset = int( rand() * noEngine );
	var chooseEngine = offset; # the engine for which we reduce rpm
	for (var i = offset; i < (noEngine + offset); i  +=  1) {
		if (i < noEngine) {
			var j = i;
			}
		else {
			var j = i - noEngine; 
			}
		# debprint("Bombable: j = ",j,"revs = ",engineRevs[j]);
		# 90% of calls will spin down engines that are already damaged
		if (rand() > .1) {
			if (engineRevs[j] == 400) chooseEngine = j;
			if (engineRevs[j] == 1000) chooseEngine = j;
			}
		}
	if (engineRevs[chooseEngine] == 3000) {
		setprop(""~myNodeName~"/engines/engine["~chooseEngine~"]/rpm" , 1000);
		}
	elsif (engineRevs[chooseEngine] == 1000) {
		setprop(""~myNodeName~"/engines/engine["~chooseEngine~"]/rpm" , 400);
		}
	else {
		setprop(""~myNodeName~"/engines/engine["~chooseEngine~"]/rpm" , 0);
		}
	}


########################## startEngines ###########################

var startEngines = func(myNodeName) {
	# Call after game reset 
	# Clunky. Iterate through children of node?
	var revs = 0;
	for (var noEngine = 0; noEngine < 6; noEngine  +=  1) {
		revs = getprop(""~myNodeName~"/engines/engine["~noEngine~"]/rpm");
		if (revs == nil) break;
	}
	if (noEngine == 0) return;
	for (var i = 0; i < noEngine ; i  +=  1) {
		setprop(""~myNodeName~"/engines/engine["~i~"]/rpm" , 3000);
	}
}
		


########################## killEngines ###########################

var killEngines = func(myNodeName) 
{
	for (var noEngine = 0; noEngine < 6; noEngine  +=  1) 
	{
		if ( getprop(""~myNodeName~"/engines/engine["~noEngine~"]/rpm") == nil) continue;
		setprop
		(
			""~myNodeName~"/engines/engine["~noEngine~"]/rpm" , 
			(rand() > .1) ? 0 : 400
		);
	}
}
########################## reduceSpeed ###########################
# reduceSpeed is called when a groundvehicle or ship is destroyed
# Uses same id as groundloop and so continues until the groundloop is terminated using inc_loopid
# Bombable ships and groundvehicles both use the AIship model
# The object AI code is used to provide a smooth deceleration

var reduceSpeed = func(id, myNodeName, factorSlowDown, type) 
{
	id == attributes[myNodeName].loopids.ground_loopid or return;

	var tgt_spd_kts = getprop (""~myNodeName~"/controls/tgt-speed-kts");
	if (tgt_spd_kts == nil ) tgt_spd_kts = 0;

	setprop(""~myNodeName~"/controls/tgt-speed-kts", tgt_spd_kts * factorSlowDown);
		
	settimer( func{reduceSpeed(id, myNodeName, factorSlowDown,type)},1);
}

########################## rotate_round ###########################
var rotate_round_x_axis = func (vector, alpha) {
 
    var c_alpha = math.cos(alpha * D2R);
    var s_alpha = math.sin(alpha * D2R);

    var matrix = [
        [
           1,
            0,
            0
        ],

        [
           0,
            c_alpha,
            -s_alpha
        ],

        [
            0,
            s_alpha,
            c_alpha
        ]
    ];

    var x2 = vector[0] * matrix[0][0] + vector[1] * matrix[1][0] + vector[2] * matrix[2][0];
    var y2 = vector[0] * matrix[0][1] + vector[1] * matrix[1][1] + vector[2] * matrix[2][1];
    var z2 = vector[0] * matrix[0][2] + vector[1] * matrix[1][2] + vector[2] * matrix[2][2];

    # debug.dump(vector, alpha, x2, y2, z2);
    return [x2, y2, z2];
}
var rotate_round_y_axis = func (vector, beta) {
 
    var c_beta = math.cos(beta * D2R);
    var s_beta = math.sin(beta * D2R);

    var matrix = [
        [
           c_beta,
            0,
            s_beta
        ],

        [
           0,
            1,
            0
        ],

        [
            -s_beta,
            0,
            c_beta
        ]
    ];

    var x2 = vector[0] * matrix[0][0] + vector[1] * matrix[1][0] + vector[2] * matrix[2][0];
    var y2 = vector[0] * matrix[0][1] + vector[1] * matrix[1][1] + vector[2] * matrix[2][1];
    var z2 = vector[0] * matrix[0][2] + vector[1] * matrix[1][2] + vector[2] * matrix[2][2];

    # debug.dump(vector, beta, x2, y2, z2);
    return [x2, y2, z2];
}
var rotate_round_z_axis = func (vector, gamma) {
	#rotate gamma degrees clockwise viewed in direction of -z
 
    var c_gamma = math.cos(gamma * D2R);
    var s_gamma = math.sin(gamma * D2R);

    var matrix = [
        [
           c_gamma,
            -s_gamma,
            0
        ],

        [
           s_gamma,
            c_gamma,
            0
        ],

        [
            0,
            0,
            1
        ]
    ];

    var x2 = vector[0] * matrix[0][0] + vector[1] * matrix[1][0] + vector[2] * matrix[2][0]; # [row_no] [col_no]
    var y2 = vector[0] * matrix[0][1] + vector[1] * matrix[1][1] + vector[2] * matrix[2][1];
    var z2 = vector[0] * matrix[0][2] + vector[1] * matrix[1][2] + vector[2] * matrix[2][2];

    # debug.dump(vector, gamma, x2, y2, z2);
    return [x2, y2, z2];
}
# # tank starts heading due N
# gunDir = [0,1,0];
# # raise gun
# raiseGun = (rotate_round_x_axis(gunDir,20));
# turnTurret = (rotate_round_z_axis(raiseGun,45));
# rollTank =(rotate_round_y_axis(turnTurret,10));
# pitchTank =(rotate_round_x_axis(rollTank,-15));
# # tank turns to heading of 30 deg
# setDir = (rotate_round_z_axis(pitchTank,30));

# debug.dump(gunDir, raiseGun, turnTurret, rollTank, pitchTank, setDir);

########################## rotate_zxy ###########################
# from http://www.songho.ca/opengl/gl_anglestoaxes.html
# rotations of the x-, y-, and z-axes in a counterclockwise direction when looking towards the origin

var rotate_zxy = func (vector, alpha, beta, gamma) {
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

    var x2 = vector[0] * matrix[0][0] + vector[1] * matrix[1][0] + vector[2] * matrix[2][0]; # [row_no] [col_no]
    var y2 = vector[0] * matrix[0][1] + vector[1] * matrix[1][1] + vector[2] * matrix[2][1];
    var z2 = vector[0] * matrix[0][2] + vector[1] * matrix[1][2] + vector[2] * matrix[2][2];

    # debug.dump(vector, gamma, x2, y2, z2);
    return [x2, y2, z2];
}

########################## rotate_yxz ###########################
# from http://www.songho.ca/opengl/gl_anglestoaxes.html
# rotations of the x-, y-, and z-axes in a counterclockwise direction when looking towards the origin

var rotate_yxz = func (vector, alpha, beta, gamma) {
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
           c_beta * c_gamma - s_beta * s_alpha * s_gamma,
           -c_alpha * s_gamma,
           s_beta * c_gamma + c_beta * s_alpha * s_gamma
        ],

        [
           c_beta * s_gamma + s_beta * s_alpha * c_gamma,
           c_alpha * c_gamma,
           s_beta * s_gamma - c_beta * s_alpha * c_gamma
        ],

        [
            -s_beta * c_alpha,
            s_alpha,
            c_beta * c_alpha
        ]
    ];

    var x2 = vector[0] * matrix[0][0] + vector[1] * matrix[1][0] + vector[2] * matrix[2][0]; # [row_no] [col_no]
    var y2 = vector[0] * matrix[0][1] + vector[1] * matrix[1][1] + vector[2] * matrix[2][1];
    var z2 = vector[0] * matrix[0][2] + vector[1] * matrix[1][2] + vector[2] * matrix[2][2];

    # debug.dump(vector, gamma, x2, y2, z2);
    return [x2, y2, z2];
}
########################## erf ###########################
var erf = func (xVal) {
	# calculates for halfspace, 0 to xVal
	# odd function so negate result for xVal < 0
	# from https://en.wikipedia.org/wiki/Error_function
	
	var expVal = math.exp(-xVal * xVal);
	var result = math.sqrt(1 - expVal);
	result *= (.5 + 0.08744939 * expVal - 0.02404858 * expVal * expVal);
	return( (xVal < 0) ? -result : result );
}
########################## setWeaponPowerSkill ###########################
# called by resetBombableDamageFuelWeapons and weapons_init_func
# note weapon power measures effectiveness - not simply explosive force - which includes how accurately it can be targetted

var setWeaponPowerSkill = func(myNodeName)
{
	var power = bombableMenu["ai-weapon-power"];
	if (power == nil) power = 0.2;
	var skill = bombableMenu["ai-aircraft-skill-level"];
	if (skill == nil) skill = 1;

	# power ranges 0.2 to 1; skill ranges 1 to 5
	# separate pilot skill from gunner skill, e.g. relevant for Flying Fortress?
	# var skill = 0;
	# var n = 8;
	# for (var i = 0; i < n; i += 1) {skill += rand();} # ~norm distribution central limit theorem 
	# SD = 1 / sqrt(12n)
	# skill /= n;
	# var skill = rand();


	# Set weapPowerSkill, 0 to 1, an equal combination of weapon effectiveness and skill of pilot or gunner
	# probability of a hit depends on effectiveness, skill and the number of attempts
	# attempt frequency is set by LOOP_TIME the update time for the weapons loop (not modelled)
	# var weapPowerSkill = math.pow(( power + skill ) / 2.0, LOOP_TIME);


	var weapPowerSkill = ( power + skill / 5.0 ) / 2.0 + rand() * 0.4 - 0.2 ;
	if (weapPowerSkill > 1) weapPowerSkill = 1 elsif (weapPowerSkill < 0) weapPowerSkill = 0;

	if (attributes[myNodeName].side == 1) weapPowerSkill *= ( 1 - handicap / 100 ); # apply handicap for side (1)
	
	attributes[myNodeName].controls.weapons_pilot_ability = weapPowerSkill;

	debprint
	(
		sprintf
		(
			"weapPowerSkill set to %4.1f for "~myNodeName~"",
			weapPowerSkill
		)
	);
}


########################## keepInsideRange ###########################
# function to check whether heading, hd, is within 2 range limits, hd1 and hd2
# hd2 is clockwise of hd1
# values range from -180 to + 180 where 0 is in the direction of travel
# if hd is outside the range it is set to the limit that is closest in angle
# function returns a hash with the new value of hd and a flag that is set to 1 if in range

var keepInsideRange = func(hd1, hd2, hd)
{
	var inRange = 0;
	if (hd1 > hd2) 
	{
		if ((hd2 >= hd) or (hd >= hd1)) inRange = 1;
	}
	else
	{
		if ((hd2 >= hd) and (hd >= hd1)) inRange = 1;
	}
	if (!inRange)
	{
		var deltaHd1 = math.fmod ( hd1 - hd + 360, 360);
		var deltaHd2 = math.fmod ( hd - hd2 + 360, 360);
		var newHd = hd1;
		if (deltaHd1 > deltaHd2) newHd = hd2;
	}
	else
	{
		var newHd = hd;
	}
	return ({newHdg:newHd, insideRange:inRange});
}

########################## findRoots ###########################
# simple quadratic solver
# returns hash of roots and flag if real
# a, b, c are the coefficients of form ax2 + bx + c = 0
var findRoots = func(a, b, c)
{
	# debprint(sprintf("a= %5.3f b= %5.3f, c= %5.3f", a, b, c));
	var d = b * b - 4 * a * c;
	if (d < 0) return ({x1:0, x2:0, isReal:0});
	# if (a == 0) return ({x1: -c / b x2: -c / b, isReal:0});
	var term1 = -b / 2 / a;
	var term2 = math.sqrt(d) / 2 / a;
	return ({x1:term1 - term2, x2:term1 + term2, isReal:1});
}
########################## findIntercept ###########################
# calculate intercept vector given:
# speed of interceptor, displacement vector between aircraft1 and aircraft2
# dist_m is the magnitude of the displacement vector
# returns hash of time to intercept and velocity vector of interceptor
var findIntercept = func (myNodeName1, myNodeName2, displacement, dist_m, interceptSpeed) {
	var speed1 = getprop(""~myNodeName1~"/velocities/true-airspeed-kt") * KT2MPS; # AI
	var pitch1 = getprop(""~myNodeName1~"/orientation/pitch-deg") * D2R;
	var heading1 = getprop(""~myNodeName1~"/orientation/true-heading-deg") * D2R;
	var addTrue = (myNodeName2 == "") ? "" : "true-";
	var speed2 = getprop(""~myNodeName2~"/velocities/"~addTrue~"airspeed-kt") * KT2MPS;
	var pitch2 = getprop(""~myNodeName2~"/orientation/pitch-deg") * D2R;
	var heading2 = getprop(""~myNodeName2~"/orientation/"~addTrue~"heading-deg") * D2R;
	var vxy1 = math.cos(pitch1) * speed1;
	var vxy2 = math.cos(pitch2) * speed2;
	var velocity1 = [
					vxy1 * math.sin(heading1),
					vxy1 * math.cos(heading1),
					math.sin(pitch1) * speed1
					];
	var velocity2 = [
					vxy2 * math.sin(heading2),
					vxy2 * math.cos(heading2),
					math.sin(pitch2) * speed2
					];
	var velocity21 = [
					velocity2[0] - velocity1[0],
					velocity2[1] - velocity1[1],
					velocity2[2] - velocity1[2]
					];
	var deltaV = dotProduct(velocity21, velocity21) - interceptSpeed * interceptSpeed;
	if (deltaV * deltaV < 1e-10) return ({time:9999, vector:[0, 0, 0]});
	var time_sec = findRoots(
		deltaV,
		2 * dotProduct(displacement, velocity21),
		dist_m * dist_m); # in reference frame of AC1
	# if (time_sec.isReal != 1) debprint ("not real");
	if (time_sec.isReal != 1) return ({time:9999, vector:[0, 0, 0]});
	# debprint(sprintf("Roots are %5.3f and %5.3f", time_sec.x1, time_sec.x2));
	var chooseRoot = time_sec.x2;
	if (time_sec.x1 < 0) 
	{
		if (time_sec.x2 < 0) return ({time:9999, vector:[0, 0, 0]});
	}
	else 
	{
		if ((time_sec.x2 < 0) or (time_sec.x2 > time_sec.x1)) chooseRoot = time_sec.x1;
	}
	return (
	{
	time:chooseRoot, 
	vector:[
				displacement[0] / chooseRoot + velocity2[0], 
				displacement[1] / chooseRoot + velocity2[1],
				displacement[2] / chooseRoot + velocity2[2]
				] 
	}); # in earth reference frame
}

########################## findIntercept2 ###########################
# findIntercept without velocity of platform
# returns velocity vector of node1 for an intercept
# displacement is r2 - r1

var findIntercept2 = func (r21, modr21, speed1, velocity2) {
	var deltaV = velocity2[0] * velocity2[0] + velocity2[1] * velocity2[1] + velocity2[2] * velocity2[2] - speed1 * speed1;

	var time_sec = findRoots(
		deltaV,
		2 * (r21[0] * velocity2[0] + r21[1] * velocity2[1] + r21[2] * velocity2[2]), # dot product
		modr21 * modr21);
	
	if (time_sec.isReal != 1) return ({time:9999, vector:[0, 0, 0]});
	# debprint(sprintf("Roots are %5.3f and %5.3f", time_sec.x1, time_sec.x2));
	var chooseRoot = time_sec.x2;
	
	if (time_sec.x1 < 0) 
	{
		if (time_sec.x2 < 0) return ({time:9999, vector:[0, 0, 0]});
	}
	else 
	{
		if ((time_sec.x2 < 0) or (time_sec.x2 > time_sec.x1)) chooseRoot = time_sec.x1;
	}
	
	return (
	{
	time:chooseRoot, 
	vector:[
				r21[0] / chooseRoot + velocity2[0], 
				r21[1] / chooseRoot + velocity2[1],
				r21[2] / chooseRoot + velocity2[2]
				] 
	}); # in earth reference frame
}	

########################## findIntercept3 ###########################
# calculate velocity vector for aircraft1 to intercept aircraft2
# given: displacement vector between aircraft1 and aircraft2, and speed of aircraft1  
# returns: hash of time to intercept, and velocity vector
# time < 0 indicates no intercept is possible

var findIntercept3 = func (myNodeName2, displacement, speed1) {
	var dist_m = math.sqrt
	(
		displacement[0] * displacement[0] +
		displacement[1] * displacement[1] +
		displacement[2] * displacement[2]
	);
	var addTrue = (myNodeName2 == "") ? "" : "true-";
	var speed2 = getprop(""~myNodeName2~"/velocities/"~addTrue~"airspeed-kt") * KT2MPS;
	var pitch2 = getprop(""~myNodeName2~"/orientation/pitch-deg") * D2R;
	var heading2 = getprop(""~myNodeName2~"/orientation/"~addTrue~"heading-deg") * D2R;
	var vxy2 = math.cos(pitch2) * speed2;
	var velocity2 = [
					vxy2 * math.sin(heading2),
					vxy2 * math.cos(heading2),
					math.sin(pitch2) * speed2
					];
	var deltaV = speed2 * speed2 - speed1 * speed1;
	if (deltaV * deltaV < 1e-10) return ({time:-1, vector:[0, 0, 0]});
	var time_sec = findRoots(
		deltaV,
		2 * dotProduct(displacement, velocity2),
		dist_m * dist_m); # in reference frame of AC1
	# if (time_sec.isReal != 1) debprint ("not real");
	if (time_sec.isReal != 1) return ({time:-2, vector:[0, 0, 0]});
	# debprint(sprintf("Roots are %5.3f and %5.3f", time_sec.x1, time_sec.x2));
	var chooseRoot = time_sec.x2;
	if (time_sec.x1 < 0) 
	{
		if (time_sec.x2 < 0) return ({time:-3, vector:[0, 0, 0]});
	}
	else 
	{
		if ((time_sec.x2 < 0) or (time_sec.x2 > time_sec.x1)) chooseRoot = time_sec.x1;
	}
	return (
	{
	time:chooseRoot, 
	vector:[
				displacement[0] / chooseRoot + velocity2[0], 
				displacement[1] / chooseRoot + velocity2[1],
				displacement[2] / chooseRoot + velocity2[2]
				] 
	}); # in earth reference frame
}

########################## vectorModulus ###########################

var vectorModulus = func(vector) {
	if (size(vector) == 0) return 0;
	var mod = 0;
	for (var i = 0; i < size(vector); i += 1)
	{
		mod += vector[i] * vector [i];
	}
	# if (mod < 0) {
	# 	debprint (
	# 		sprintf(
	# 		"Bombable: modulus_vector =[%8.3f, %8.3f, %8.3f]",
	# 		vector[0], vector[1], vector[2] 
	# 		)
	# 	);
	# }
	return math.sqrt(mod);
}

########################## dotProduct ###########################
#calculate dot product of two 3D vectors, v1 and v2
var dotProduct = func(v1, v2)
{
	return(v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]);
}


########################## vectorSum ###########################
#calculate vector sum of two 3D vectors, v1 and v2
var vectorSum = func(v1, v2)
{
	return([v1[0] + v2[0] , v1[1] + v2[1] , v1[2] + v2[2]]);
}

########################## vectorSubtract ###########################
#subtract 3D vectors v2 from v1
var vectorSubtract = func(v1, v2)
{
	return([v1[0] - v2[0] , v1[1] - v2[1] , v1[2] - v2[2]]);
}

########################## vectorMultiply ###########################
#multiply 3D vector v1 by scalar s
var vectorMultiply = func(v1, s)
{
	return([v1[0] * s , v1[1] * s , v1[2] * s]);
}
########################## vectorDivide ###########################
#divide 3D vector v1 by scalar s
var vectorDivide = func(v1, s)
{
	return([v1[0] / s , v1[1] / s , v1[2] / s]);
}

########################## normalize ###########################
#normalize 3D vector v1
var normalize = func(v1)
{
	var magnitude = vectorModulus (v1);
	return(vectorDivide(v1, magnitude));
}

########################## crossProduct ###########################
# cross product v1 ^ v2
var crossProduct = func(v1, v2)
{

	return(
		[
		v1[1] * v2[2] - v1[2] * v2[1],
		v1[2] * v2[0] - v1[0] * v2[2],
		v1[0] * v2[1] - v1[1] * v2[0]
		]
	);
}
########################## vectorRotate ###########################
# rotate unit vector v1 by alpha, in the direction of unit vector v2 
# using rotation axis defined by cross product k = v1 ^ v2
# algorithm from https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
# returns rotated unit vector

var vectorRotate = func(v1, v2, alpha)
{
	var v1v2 = dotProduct (v1, v2);
	var magnitude_k = math.abs (math.sin (math.acos (v1v2)));
	# abs needed since angles between 90 and 180 or -180 and -90 will otherwise give magnitude < 0
	var sinAbymagK = math.sin(alpha) / magnitude_k;
	var v3 = vectorMultiply (v1, ( math.cos(alpha) - v1v2 * sinAbymagK ));
	var v4 = vectorMultiply (v2, sinAbymagK );
	return (vectorSum (v3, v4));
}

########################## approxTanh ###########################
# coarse approximation of tanh function for values of x between 0 and 3

var approxTanh = func(x)
{
	var MAX = 3.0;
	var NUM_ELEMENTS = 10 ;
	var lookup =
	[
		0,
		0.291312612,
		0.537049567,
		0.71629787,
		0.833654607,
		0.905148254,
		0.946806013,
		0.970451937,
		0.983674858,
		0.991007454,
		0.995054754
	];
	var sign = (x < 0.0) ? -1.0 : 1.0 ;
	var absScaleX = x * NUM_ELEMENTS / MAX * sign ;
	if (x >= NUM_ELEMENTS) return ( sign ) ;
	var smallest = math.floor ( absScaleX ) ;
	var mod = absScaleX - smallest ;
	return( 
		sign * 
		( lookup[ smallest ] * ( 1.0 - mod ) + lookup[ smallest + 1 ] * mod )
	);
}

########################## shuffle ###########################
# shuffles elements of vector x 
# a random number is assigned to each element of x
# the random numbers are sorted to give the new order of the elements

var shuffle = func(x)
{
	var y = [];
	var z = [];
	var randNos = []; 
	var n = size(x);
	setsize (y, n);
	setsize (z, n);
	setsize (randNos, n);
	forindex (var i; y)
	{
		y[i] = i;
		randNos[i] = rand();
	}
	y = (newSort(y, randNos));
	forindex (var i; z)
	{
		z[i] = x[y[i]];
	}
	return (z);
}

########################## addToTargets ###########################
# create a list of AI objects, shooters and targets
# each object is assigned a unique index, stored in hash 'teams'
# objects are grouped into teams, which are named A - Z
# each team is a key in the hash, which contains a vector of the indices for that team
# teams are grouped into two opposing sides:  0: A-M and 1: N-Z
# the main AC is in a team by itself - team A
# allPlayers contains all objects, with indices '0' and '1', indicating the side
# the team name (B-Z) should be the last character of the object's callsign
# nodes records the nodeName for each index; reverse lookup (nodename -> index) is by attributes

var addToTargets = func(myNodeName)
{
	var myIndex = getprop("/bombable/targets/index");
	if (myIndex == nil) myIndex = 1; # 0 is for main AC
	var ats = attributes[myNodeName];
	ats.index = myIndex;
	append(nodes, myNodeName);
	setprop("/bombable/targets/index", myIndex + 1);
	var callsign = getCallSign(myNodeName); 
	var teamName = right(callsign, 1);
	#check valid team
	if (find(teamName, "BCDEFGHIJKLMNOPQRSTUVWXYZ") == -1)
	{
		debprint(callsign, " not a valid team - require (B-Z) - A is the main aircraft");
		return;
	}
	if (teams[teamName] == nil) teams[teamName] = {indices: [], target: nil, count: 0};
	append(teams[teamName].indices, myIndex);
	var side = (find(teamName, "ABCDEFGHIJKLM") == -1);
	append(allPlayers[side], myIndex);
	ats.team = teamName;
	ats.side = side;
}
########################## initTargets ###########################
# assigns a target for each object in each team, except for the main AC, team A
# attributes.shooterIndex is a vector containing the indices of the nodes shooting at the object
# targetTeam is a team on the opposing side set by the scenario
# the list of targets is shuffled before being assigned
# the number of targets per object is determined by the number of its weapons - maxTargets
# can assign several targets to one shooter
# objects with no AI target may be assigned in later action
#

var initTargets = func () {
	# wait til all weapons initialized
	var ready = 1;
	foreach (var myNodeName; nodes)
	{
		if (myNodeName == "") continue;
		ready = getprop(""~myNodeName~"/bombable/initializers/weapons-initialized");
		if (!ready) break;
	}
	if (!ready) 
	{
		settimer(func{initTargets()}, 5); 
		return;
	}
	debprint("Bombable: initializing targets");

	var foundTarget = -1;
	foreach (var side; [0, 1]) allPlayers[side] = shuffle(allPlayers[side]);
	foreach (teamName; keys(teams))
	{
		if (teamName == "A") continue; #main AC has no assigned targets
		teams[teamName].indices = shuffle(teams[teamName].indices);
		var targetTeam = teams[teamName].target;
		var side = (find(targetTeam, "ABCDEFGHIJKLM") == -1);
		var count = 0;
		forindex (var i; teams[teamName].indices)
		{
			var myIndex = teams[teamName].indices[i];
			for (var j = 0; j < attributes[nodes[myIndex]].maxTargets; j = j + 1 )
			{		
				if (targetTeam != nil)
				{
					foundTarget = assignOneTarget (myIndex, teams[targetTeam].indices); 
				}
				if (foundTarget == -1) 
				{
					foundTarget = assignOneTarget (myIndex, allPlayers[side]);
				}
				if (foundTarget == -1) break;
			}
			debprint("Bombable: initTargets: ", j, " targets assigned for ", getCallSign(nodes[myIndex]), " team ", teamName);
			count += j;
		}
		debprint( "Bombable: initTargets: Total of ", count, " targets in", (targetTeam != nil ) ? " team " ~ targetTeam : "", " side (", side, ") assigned for team ", teamName );
	}

	# apply handicap to side (1) by reducing pilot skills by a fixed percentage
	forindex (var i; allPlayers[1])
	{
		attributes[nodes[allPlayers[1][i]]].controls.pilotAbility *= ( 1 - handicap / 100 );
	}
	debprint("Bombable: Handicap of ", handicap, "% applied to side (1)");
}


########################## assignOneTarget ###########################
# I am the shooter
# func assigns a target given a list of indices of targets
# it selects the target with the smallest number of shooters
# and returns the index of the new target
# reverse loop avoids pairing
# a target _will_ be assigned
# if there are more shooters than targets then some targets will have more than one shooter 
# max no of shooters for one target
# only called by AI objects - not by main AC

var assignOneTarget = func (myIndex, targets) {
	if (size(targets) == 0) return(-1);
	var myNodeName = nodes[myIndex];
	var j = 999;
	var k = -1;
	var s = 0;
	var maxNo = 3;
	foreach (var i; targets) 
	{
		if (attributes[nodes[i]].damage == 1) continue; # if object destroyed cannot be a target
		if (vecindex(attributes[myNodeName].targetIndex, i) != nil) continue; # already a target
		s = size(attributes[nodes[i]].shooterIndex);
		if (s < j) 
		{
			j = s;
			k = i;
		}
	}
	if ((k == -1) or (j > maxNo)) return (-1); # no targets available
	append(attributes[nodes[k]].shooterIndex, myIndex);
	append(attributes[myNodeName].targetIndex, k);
	return (k);
}

########################## assignOneShooter ###########################
# I am the target
# func assigns a shooter from the list of indices of shooters
# targetIndex == [] indicates object has no target assigned
# returns the index of my new shooter
# main AC cannot be assigned since its target index is nil
# reverse order ?

var assignOneShooter = func (myIndex, shooters) {
	if (size(shooters) == 0) return(-1);
	var myNodeName = nodes[myIndex];
	var k = -1;
	var maxNo = 3;
	foreach (var i; shooters)
	{
		if (attributes[nodes[i]].damage == 1 or !i) continue; # if object destroyed, or main aircraft, cannot be a shooter
		if (size(attributes[nodes[i]].targetIndex) <= attributes[nodes[i]].maxTargets) k = i; # no of targets limited by no of weapons
	}
	if (k == -1) return (-1); # no shooters available
	append(attributes[nodes[k]].targetIndex, myIndex);
	append(attributes[myNodeName].shooterIndex, k);
	return (k);
}

########################## findNewTarget ###########################
# I am the shooter
# return index of new target or -1
var findNewTarget = func (myIndex) {
	var myTeam = attributes[nodes[myIndex]].team;
	var targetTeam = teams[myTeam].target;
	var otherSide = !attributes[nodes[myIndex]].side;
	var foundTarget = -1;
	if (targetTeam != nil)
	{
		foundTarget = assignOneTarget (myIndex, teams[targetTeam].indices);
	}
	if (foundTarget == -1)
	{
		foundTarget = assignOneTarget (myIndex, allPlayers[otherSide]);
	}
	debprint("Bombable: foundTarget ", (foundTarget != -1) ? nodes[foundTarget] : "fail", " for ", nodes[myIndex]);
	return(foundTarget);
}

########################## findNewShooter ###########################
var findNewShooter = func (myIndex) {
# I am the target
	var myTeam = attributes[nodes[myIndex]].team;
	var shooterTeam = teams[myTeam].target;
	var otherSide = !attributes[nodes[myIndex]].side;
	var foundShooter = -1;
	if (shooterTeam != nil)
	{
		foundShooter = assignOneShooter (myIndex, teams[shooterTeam].indices);
	}
	if (foundShooter == -1)
	{
		foundShooter = assignOneShooter (myIndex, allPlayers[otherSide]);
	}
	debprint("Bombable: foundShooter ", (foundShooter !=-1) ? nodes[foundShooter] : "fail", " for ", nodes[myIndex]);
	return(foundShooter);
}

########################## removeTarget ###########################
# called when I am attacked and need to swap out a target
var removeTarget = func (myIndex) {
	var ats = attributes[nodes[myIndex]];
	var nTargets = size(ats.targetIndex);
	if (!nTargets) return;
	var oldTarget = ats.targetIndex[int(rand() * nTargets)];
	var ats2 = attributes[nodes[oldTarget]];
	ats.targetIndex = removeElem(ats.targetIndex, oldTarget);
	ats2.shooterIndex = removeElem(ats2.shooterIndex, myIndex);
	if (size(ats2.shooterIndex) == 0) findNewShooter(oldTarget);
}

########################## resetTargetShooter ###########################
# called when I am destroyed - not necessarily by my shooter
# find new target for my shooter
# find new shooter for each of my targets
# index 0 is main AC

var resetTargetShooter = func (myIndex) {
	var ats = attributes[nodes[myIndex]];

	# remove me from my targets' list of shooters
	# if any of my targets no longer has a shooter, find one

	foreach (myTarget; ats.targetIndex)
	{
		var ats2 = attributes[nodes[myTarget]];
		ats2.shooterIndex = removeElem(ats2.shooterIndex, myIndex);
		if (size(ats2.shooterIndex) == 0) findNewShooter(myTarget);
	}
	
	var foundTarget = -1;
		# find new target for my shooters
		foreach (var myShooter; ats.shooterIndex) 
		{
			var ats2 = attributes[nodes[myShooter]];
			ats2.targetIndex = removeElem(ats2.targetIndex, myIndex);
			if (size(ats2.targetIndex) == 0) findNewTarget(myShooter);
		}
		allPlayers[ats.side] = removeElem(allPlayers[ats.side], myIndex);
		ats.targetIndex = [];
		ats.shooterIndex = []; # remove shooters from dead object
		debprint("Bombable: ", nodes[myIndex], " no longer a target");
}

########################## waitForAI ###########################
# delay to allow AI objects to load

var waitForAI = func()
{
	# if (getprop("/bombable/targets/index") != getprop("/ai/models/count"))
	if (getprop("/bombable/targets/index") != 
	size(props.globals.getNode ("/ai/models").getChildren("aircraft")) +
	size(props.globals.getNode ("/ai/models").getChildren("ship")) + 1)  # bombable uses only two types of AI object; 1 for main AC
	{
		settimer (func {waitForAI();}, 5, 1); # wait til all 3D models have been loaded
		return;
	}	

	foreach (var myNodeName; nodes)
	{
		if (myNodeName != "") # omit main AC
		{
			setprop(""~myNodeName~"/position/latitude-deg", 0);
			setprop(""~myNodeName~"/position/longitude-deg", 0);
		}
	}

	# ensure not paused
	props.globals.getNode("sim/freeze/master", 1).setBoolValue(0);
	props.globals.getNode("sim/freeze/clock", 1).setBoolValue(0);

	# wait to clear the smoke from the airport
	var timeNow = getprop("/sim/time/elapsed-sec");
	var startTime = timeNow + 120;
	setprop("/sim/speed-up", 16);
	debprint("Bombable: delaying start");
	settimer(func{startScenario(startTime)}, 1);
}

########################## startScenario ###########################
# a scenario consists of:
# a set of groups of objects 
# each group is assigned to a team (can be the same team)
# and is provided with co-ordinates relative to an airport, by assuming that
# the group is on course to the airport at a distance set by its arrival time and speed
# each object in the group is given an offset in metres relative to the lead object
# the number of offsets defines the number of objects in the group
# the scenario xml file positions all AI objects on the airport runway close to the main AC to ensure they are loaded quickly
# the call to startScenario is delayed until FG has loaded all aircraft and ship objects into the airport scene
# a scenario-initialized flag is set which will then enable initialization of weapons, ground and attack loops 

var startScenario = func(startTime)
{
	var timeNow = getprop("/sim/time/elapsed-sec");
	if (timeNow < startTime)
	{
		settimer(func{startScenario(startTime)}, 1);
		return;
	}
	setprop("/sim/speed-up", 1);
	var scenarioName = getprop("/sim/ai/scenario");
	debprint("Bombable: starting scenario "~scenarioName);
	if (scenarioName == "BOMB-MarinCountySixZerosSixF6Fs")
	{
		var scenario = 
		{
			group1:
			{
			team :			"D",
			target :		"Y",
			arrivalTime :	90, # sec
			airSpeed : 		250 * KT2MPS,
			airportName :	"CA35",
			heading :		220,# 0 - 360 degrees
			alt :			6000, # in feet
			offsets :
						[
							[0, 0, 0], # offset behind, offset to right, in metres, i.e. model co-ord system
							[75, 75, 2],
							[75, -75, 1],
							[150, 150, -5],
							[150, -150, 4],
							[225, 225, 2]
						],
			},
			group2:
			{
			team :			"Y",
			target :		"D",
			arrivalTime :	90, # sec
			airSpeed : 		280 * KT2MPS,
			airportName :	"CA35",
			heading :		70,
			alt :			7000,
			offsets :
						[
							[0, 0, 0],
							[50, -50, -3],
							[75, 50, 2]
						],
			},
			group3:
			{
			team :			"Z",
			target :		"D",
			arrivalTime :	110, # sec
			airSpeed : 		280 * KT2MPS,
			airportName :	"CA35",
			heading :		350,
			alt :			8000,
			offsets :
						[
							[0, 0, 0],
							[50, 100, 1],
							[100, -50, -2]
						],
			},
		};

	}
	elsif (scenarioName == "BOMB-MarinCountyFiveB17FiveA6M5TwoF6F")
	{
		var scenario = 
		{
			group1: #Zeros
			{
			team :			"D",
			target :		"Y",
			arrivalTime :	90, # sec
			airSpeed : 		250 * KT2MPS,
			airportName :	"CA35",
			heading :		90,# 0 - 360 degrees
			alt :			6000, # in feet
			offsets :
						[
							[0, 0, 0], # offset behind, offset to right, in metres, i.e. model co-ord system
							[75, 75, 2],
							[75, -75, 1],
							[150, 150, -5],
							[150, -150, 4]
						],
			},
			group2: #B17s
			{
			team :			"Y",
			target :		"D",
			arrivalTime :	90, # sec
			airSpeed : 		182 * KT2MPS,
			airportName :	"CA35",
			heading :		70,
			alt :			5000,
			offsets :
						[
							[0, 0, 0],
							[80, -80, -3],
							[80, 80, 2],
							[160, -160, 2],
							[160, 160, 2]
						],
			},
			group3: #F6Fs
			{
			team :			"Z",
			target :		"D",
			arrivalTime :	120, # sec
			airSpeed : 		280 * KT2MPS,
			airportName :	"CA35",
			heading :		350,
			alt :			10000,
			offsets :
						[
							[0, 0, 0],
							[50, 100, 1]
						],
			},
		};
	}
	elsif (scenarioName == "BOMB-MarinCountyThreeB17NineA6M5")
	{
		var scenario = 
		{
			group1: #B17s
			{
			team :			"Y",
			target :		"D",
			arrivalTime :	90, # sec
			airSpeed : 		182,
			airportName :	"CA35",
			heading :		70,
			alt :			5000,
			offsets :
						[
							[0, 0, 0],
							[80, -80, -3],
							[80, 80, 2]
						],
			},
			group2: #Zeros
			{
			team :			"D",
			target :		"Y",
			arrivalTime :	90, # sec
			airSpeed : 		250,
			airportName :	"CA35",
			heading :		90,# 0 - 360 degrees
			alt :			6000, # in feet
			offsets :
						[
							[0, 0, 0], # offset behind, offset to right, in metres, i.e. model co-ord system
							[40, 40, 2],
							[40, -40, 1]
						],
			},
			group3: #Zeros
			{
			team :			"B",
			target :		"Y",
			arrivalTime :	110, # sec
			airSpeed : 		250,
			airportName :	"CA35",
			heading :		90,
			alt :			6100,
			offsets :
						[
							[0, 0, 0],
							[30, 50, 1],
							[45, -30, -2]
						],
			},
			group4: #Zeros
			{
			team :			"C",
			target :		"Y",
			arrivalTime :	130, # sec
			airSpeed : 		280,
			airportName :	"CA35",
			heading :		350,
			alt :			8000,
			offsets :
						[
							[0, 0, 2],
							[30, 40, 1],
							[30, -45, 1]
						],
			},
		};
	}
	elsif (scenarioName == "BOMB-MarinCountyNineA6M5ThreeB17")
	{
		var scenario = 
		{
			group1: #B17s
			{
			team :			"B",
			target :		"X",
			arrivalTime :	90, # sec
			airSpeed : 		182,
			airportName :	"CA35",
			heading :		70,
			alt :			5000,
			offsets :
						[
							[0, 0, 0],
							[80, -80, -3],
							[80, 80, 2]
						],
			},
			group2: #Zeros
			{
			team :			"X",
			target :		"B",
			arrivalTime :	90, # sec
			airSpeed : 		250,
			airportName :	"CA35",
			heading :		90,# 0 - 360 degrees
			alt :			6000, # in feet
			offsets :
						[
							[0, 0, 0], # offset behind, offset to right, in metres, i.e. model co-ord system
							[40, 40, 2],
							[40, -40, 1]
						],
			},
			group3: #Zeros
			{
			team :			"Y",
			target :		"B",
			arrivalTime :	110, # sec
			airSpeed : 		250,
			airportName :	"CA35",
			heading :		90,
			alt :			6100,
			offsets :
						[
							[0, 0, 0],
							[30, 50, 1],
							[45, -30, -2]
						],
			},
			group4: #Zeros
			{
			team :			"Z",
			target :		"B",
			arrivalTime :	130, # sec
			airSpeed : 		280,
			airportName :	"CA35",
			heading :		350,
			alt :			8000,
			offsets :
						[
							[0, 0, 2],
							[30, 40, 1],
							[30, -45, 1]
						],
			},
		};
	}
	elsif (scenarioName == "BOMB-Llandbehr_Type45")
	{
		var scenario = 
		{
			group1: #Type45
			{
			team :			"B",
			target :		"X",
			arrivalTime :	-360, # sec
			airSpeed : 		25,
			airportName :	"EGOD",
			heading :		225,
			alt :			0,
			offsets :
						[
							[0, 0, 0]
						],
			},
			group2: #F15
			{
			team :			"X",
			target :		"B",
			arrivalTime :	200, # sec
			airSpeed : 		500,
			airportName :	"EGOD",
			heading :		30,
			alt :			5000,
			offsets :
						[
							[0, 1000, 0],
							# [-20, 21, 0],
							[-1000, -1000, 0]
						],
			},
		};
	}
	elsif (scenarioName == "BOMB-Llandbehr_Type45_F15_rocket")
	{
		var scenario = 
		{
			group1: #Type45
			{
			team :			"Z",
			target :		"B",
			arrivalTime :	-30, # sec
			airSpeed : 		25,
			airportName :	"EGOD",
			heading :		225,
			alt :			0,
			offsets :
						[
							[0, 0, 0]
						],
			},
			group2: #F15
			{
			team :			"B",
			target :		"Z",
			arrivalTime :	120, # sec
			airSpeed : 		512,
			airportName :	"EGOD",
			heading :		45,
			alt :			8000,
			offsets :
						[
							[0, 0, 0],
							[-20, 21, 0]
							# [-50, -50, 0]
						],
			},
		};
	}
	else
	{
		debprint("Bombable: startScenario: Error "~scenarioName~" not in database");
	}
	var myNodeName = "";
    var GeoCoord = geo.Coord.new();
    var GeoCoord2 = geo.Coord.new();
	foreach (var group ; keys(scenario))
	{
		var from = airportinfo(scenario[group].airportName);
		var teamName = scenario[group].team;
		if (teamName == "A")
		{
			debprint("Bombable: startScenario: Error in scenario definition for "~group~" - \"A\" reserved for main AC");
			break;
		}
		if (find(teamName, "BCDEFGHIJKLMNOPQRSTUVWXYZ") == -1)
		{
			debprint("Bombable: startScenario: Error in scenario definition for "~group~" - no team");
			break;
		}
		if(!contains(teams, teamName))
		{
			debprint("Bombable: startScenario: Error scenario team "~teamName~" not found in objects");
			break;
		}
		var targetTeam = scenario[group].target;
		if (find(targetTeam, "ABCDEFGHIJKLMNOPQRSTUVWXYZ") != -1)
		{
			if(!contains(teams, targetTeam))
			{
				debprint("Bombable: startScenario: Error scenario team "~targetTeam~" not found in objects");
				break;
			}
			teams[teamName].target = targetTeam;
			var msg = (targetTeam == "A") ? "main AC" : targetTeam;
			debprint("Bombable: startScenario: Target team for "~group~" is " ~ msg);
		}
		# location lead aircraft
		GeoCoord.set_latlon(from.lat, from.lon);
		var dist = scenario[group].airSpeed * KT2MPS * scenario[group].arrivalTime;
		var heading = scenario[group].heading;
		GeoCoord.apply_course_distance(heading + 180, dist);
		foreach (var o ; scenario[group].offsets)  
		{
			#calculate lon, lat
			GeoCoord2.set_latlon ( GeoCoord.lat(), GeoCoord.lon());
			var myHeading = math.atan2(o[1], -o[0]) * R2D;
			var deltaHeading = heading + myHeading ;
			dist2me = math.sqrt(o[0]*o[0] + o[1]*o[1]); 
			GeoCoord2.apply_course_distance(deltaHeading, dist2me);    #frontreardist in meters
			#get node
			var count = teams[teamName].count;
			if (count < size(teams[teamName].indices)) # check to ensure scenario definition and xml file are consistent
			{
				myNodeName = nodes[teams[teamName].indices[count]];
				var type = attributes[myNodeName].type;
				count += 1;
				teams[teamName].count = count;
				setprop(""~myNodeName~"/orientation/true-heading-deg", scenario[group].heading);
				setprop(""~myNodeName~"/orientation/roll-deg", 0);
				setprop(""~myNodeName~"/orientation/pitch-deg", 0);
				setprop(""~myNodeName~"/position/latitude-deg", GeoCoord2.lat());
				setprop(""~myNodeName~"/position/longitude-deg", GeoCoord2.lon());
				if (type == "aircraft")
				{
					setprop(""~myNodeName~"/velocities/true-airspeed-kt", scenario[group].airSpeed);
					setprop(""~myNodeName~"/controls/flight/target-spd", scenario[group].airSpeed);
					setprop(""~myNodeName~"/controls/flight/target-alt", scenario[group].alt + o[2] * M2FT);
					setprop(""~myNodeName~"/position/altitude-ft", scenario[group].alt + o[2] * M2FT);
				}
				elsif (type == "ship")
				{
					setprop(""~myNodeName~"/controls/tgt-heading-degs", scenario[group].heading);
					setprop(""~myNodeName~"/velocities/speed-kts", scenario[group].airSpeed);
					setprop(""~myNodeName~"/controls/tgt-speed-kts", scenario[group].airSpeed);
					setprop (""~myNodeName~"/surface-positions/rudder-pos-deg", 0);					
				}
			}
		}
	}
	foreach (var t; keys(teams))
	{
		if (teams[t].count != size(teams[t].indices)) debprint("Bombable: startScenario: Count for "~teams[t]~" in scenario: "~count~" is not equal to objects loaded: "~teams[t].indices);
	}

	mainStatusPopupTip ("Scenario "~scenarioName~" loaded . . .", 15 );

	initTargets();

	setprop("/sim/ai/scenario-initialized", 1);
	
}
########################## removeAll ###########################
# removes all occurrences of element from vector
# returns vector
var removeAll = func(vector, element)
{
var result = [];
foreach (var elem; vector)
{
	if (elem != element) append(result, elem);
}
return(result);
}


########################## removeElem ###########################
# remove first occurrence of element from vector
# returns vector
var removeElem = func(vector, element)
{
	var index = vecindex(vector , element);
	if (index == nil) return (vector);
	if (index == 0) return ( size(vector) != 1 ? vector[1 : ] : []);
	if (index == size(vector) - 1) return vector[:index-1];
	return(vector[:index-1, index+1:]);
}

########################## resetScenario ###########################
# stop attack, weapons and ground loops
# rebuild teams and assign new targets
# repair and refuel all AI ships, planes
# reload weapons


var resetScenario = func()
{
	# clear pop-up message log
	tipMessageAI = "\n\n\n\n";
	tipMessageMain = "\n\n\n\n";

	# end all loops for all targets

	var loops =
		[
		"weapons",
		"ground",
		"attack",
		"roll",
		"speed_adjust"
		];
	foreach (var myNodeName; nodes)
	{
		if (myNodeName != "") 
		{
			foreach (var loopName; loops) inc_loopid(myNodeName, loopName);
		}
	}

	# rebuild teams and players
	teams = 
	{
		A:{indices: [0], target: nil, count: 1},
	}; 
	allPlayers = 
	[
		[0],
		[]
	];

	foreach (var myNodeName; nodes)
	{
		var ats = attributes[myNodeName];
		if (myNodeName != "") # omit main AC
		{
			var myIndex = ats.index ;
			var teamName = ats.team ;
			var side = ats.side ;
			if (teams[teamName] == nil) teams[teamName] = {indices: [], target: nil, count: 0};
			append(teams[teamName].indices, myIndex);
			append(allPlayers[side], myIndex);
			ats.targetIndex = []; # could initialise in initTargets
			ats.shooterIndex = [];
		} 
	}

	# move all AI objects out of scene and repair them
	foreach (var myNodeName; nodes)	resetBombableDamageFuelWeapons (myNodeName);
	
	foreach (var myNodeName; nodes)
	{
		if (myNodeName != "") # omit main AC
		{
			setprop(""~myNodeName~"/position/latitude-deg", 0);
			setprop(""~myNodeName~"/position/longitude-deg", 0);
		}
	}

	records.init();

	setprop("/sim/ai/scenario-initialized", 0); # flag used to delay start of loops until after start of scenario
	restartAllLoops(loops);

	# ensure not paused
	props.globals.getNode("sim/freeze/master", 1).setBoolValue(0);
	props.globals.getNode("sim/freeze/clock", 1).setBoolValue(0);

	# wait a while to clear the smoke and contrails
	var timeNow = getprop("/sim/time/elapsed-sec");
	var startTime = timeNow + 120;
	setprop("/sim/speed-up", 16);
	debprint("Bombable: delaying restart");
	settimer(func{startScenario(startTime)}, 1);
}



########################## restartAllLoops ###########################
# restart all loops
# the foreach loop must call a helper function - calling settimer directly from within the loop 
# causes it to use only the last element of nodes 

var restartAllLoops = func(loops)
{
	if (!getprop("/sim/ai/scenario-initialized"))
	{
		settimer (func {restartAllLoops(loops);}, 5);
		return;
	}
	foreach (var myNodeName; nodes)
	{
		if (myNodeName != "") # omit main AC
		{
			foreach (var loopName; loops) restartLoop(myNodeName, loopName);
		}
	}
}

########################## restartLoop ###########################
var restartLoop = func(myNodeName, loopName)
{
	var loopid = inc_loopid (myNodeName, loopName);
	var type= attributes[myNodeName].type;
	var r = rand() - 0.5;
	if (loopName == "weapons") 
	{
		settimer ( func {weapons_loop (loopid, myNodeName); }, r + 8);
	}
	elsif (loopName == "ground") 
	{
		settimer( func {ground_loop (loopid, myNodeName); }, r + 3);
	}
	elsif (type == "aircraft") 
	{
		if (loopName == "attack") 
		{
			settimer( func {attack_loop (loopid, myNodeName); }, r + 6);
		}
		elsif (loopName == "speed_adjust") 
		{
			settimer ( func {speed_adjust_loop ( loopid, myNodeName, .3 + rand() / 30); }, r + 4);
		}
	}
}

########################## END ###########################