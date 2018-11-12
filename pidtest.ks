declare function zeroSpeedAltitude {
	declare local vi is ship:VERTICALSPEED.
	declare local alt is ALT:RADAR.
	local accel is ship:MAXTHRUST/ship:MASS - 9.81.
	local x is (( (vi * vi)/accel )/2) - ((vi * vi)/accel) + alt.
	return x.
}

declare function zeroAccelerationThrottle {
	declare local tVal is (9.81 * ship:mass / ship:maxthrust).
	return tVal.
}

declare function twrToThrottle {
	declare local parameter twr.
	declare local tVal is (twr * zeroAccelerationThrottle()).
	return tVal.
}
declare function takeoffandsetup {
	sas off.
	rcs on.
	ag1 on.
	ag1 off.
	print("taking off").
	brakes off.
	lock steering to up.
	stage.
	until verticalspeed > 100 { lock throttle to twrToThrottle(2). }
	gear off.
	SET state to "ascending to 100m".
	until alt:radar > 100 { lock throttle to twrToThrottle(1). }
}

declare function updatescreen {
	print("Throttle: " + tval) at(0,1).
	print("velocity PID Output: " + AltitudeToVelocityPID:OUTPUT + "                   ") at(0,2).
	print("velocity PID Setpoint: " + AltitudeToVelocityPID:SETPOINT + "    ") AT(0,3).
	print("Radar Altitude: " + alt:radar) at(0,4).
	print("Position Error: " + error) at(0,5).
	PRINT("Speed: " + verticalspeed) at(0,6).
	print ("speed setpoint: " + VelocityToThrottlePID:SETPOINT) AT(0,7).
	print("Velocity error: " + velocityError) at(0,8).
	// print("Latitude: " + GeoCoordinates:LAT) at(0,9).
	// print("Longitude: " + GeoCoordinates:LNG) at(0,10).

}


clearscreen.
// takeoffandsetup().

SET targetZeroSpeedAltitude to 25. //SET target "zero speed altitude" (alt:radar is 19-20 when landed)
SET landingModeTriggerspeed to -5. //SET speed at which landing mode will be trigggered
SET landingModeVelocity to -3.5. //SET slow descent speed
SET landingModeTWRdelta to 0.3. //SET "Dumb P" value for landing



SET KpAV TO 1.25. //altitude -> velocity PID
SET KiAV TO 0.5.
SET KdAV TO 0.
SET minimumAV TO -10.
SET maximumAV TO 10.
SET KpVT TO 1.5. //velocity -> throttle PID
SET KiVT TO 4.
SET KdVT TO 0.
SET minimumVT TO 0.1.
SET maximumVT TO 1.
SET KpLatV to 1. //latitude -> velocity PID
SET KiLatV to 0. 
SET KdLatV to 0. 
SET minimumLatV to -0.5. //Minimum and maximum Lat velocity
SET maximumLatV to 0.5.
SET KpVP to 1. //velocity -> pitch PID
SET KiVP to 0. 
SET KdVP to 0. 
SET minimumVP to -0.5. //Minimum and maximum pitch values
SET maximumVP to 0.5.
SET KpLngV to 1. //longitude -> velocity PID
SET KiLngV to 0. 
SET KdLngV to 0. 
SET minimumLngV to -0.5. //Minimum and maximum Lng velocity
SET maximumLngV to 0.5.
SET KpVY to 1. //velocity -> yaw PID
SET KiVY to 0. 
SET KdVY to 0. 
SET minimumVY to -0.5.//Minimum and maximum yaw values
SET maximumVY to 0.5.
SET KpROLL to 1. //roll pid
SET KiROLL to 0. 
SET KdROLL to 0. 
SET minimumROLLctrl to -0.5.//Minimum and maximum roll input values
SET maximumROLLctrl to 0.5.

global AltitudeToVelocityPID is PIDLOOP(KpAV, KiAV, KdAV, minimumAV, maximumAV).
global VelocityToThrottlePID is PIDLOOP(KpVT, KiVT, KdVT, minimumVT, maximumVT). //PID stuff
global LatitudeToVelocityPID is PIDLOOP(KpLatV, KiLatV, KdLatV, minimumLatV, maximumLatV).
global LatVelocityToPitchPID is PIDLOOP(KpVP, KiVP, KdVP, minimumVP, maximumVP).
global LongitudeToVelocityPID is PIDLOOP(KpLngV, KiLngV, KdLngV, minimumLngV, maximumLngV).
global LngVelocityToYawPID is PIDLOOP(KpVY, KiVY, KdVY,minimumVY, maximumVY).
global rollPID is PIDLOOP(KpROLL, KiROLL, KdROLL, minimumROLLctrl, maximumROLLctrl).

SET throttle to 0.
//wait until ship:verticalspeed < -1.
brakes on.
rcs on.
SET runmode to 1.
clearscreen.
SET height to 100. //SET desired height.
// SET targetLatitude to GeoCoordinates:LAT.
// SET targetLongitude to GeoCoordinates:LNG.
SET targetRoll to 0. //set target roll
SET previousVelLat to 0.
SET previousVelLng to 0.
SET previousTime to time:seconds.

SET error to 0.
lock steering to up. //TODO comment this out
log "Time,Error" to PID.csv.
until runmode = 0 {
	//Update perameters
	SET dt to (time:seconds - previousTime).
	SET previousTime to time:seconds.
	//SET velLat to (GeoCoordinates:LAT - previousLat)/dt.
	//SET velLng to (GeoCoordinates:LNG - previousLng)/dt.
	// SET previousLat to GeoCoordinates:Lng.
	// SET previousLng to GeoCoordinates:Lng.

	//Update the throttle PID loop
	SET AltitudeToVelocityPID:SETPOINT TO height. //setpoint for velocity PID. Changes target altitude. Is a constant-ish
	SET VelocityToThrottlePID:SETPOINT TO -2 .//AltitudeToVelocityPID:UPDATE(TIME:SECONDS, alt:radar). //PID function for position -> velocity. and SETPOINT for throttle PID, calculated by velocity PID loop.
	SET tval to VelocityToThrottlePID:UPDATE(TIME:SECONDS, verticalspeed). //PID function for calcualted target velocity -> throttle value
	lock throttle to tval.
	
	// //Update Pitch PID loop
	// SET LatitudeToVelocityPID:SETPOINT to targetLatitude. //Sets setpoint for latural velocity PID loop
	// SET LatVelocityToPitchPID:SETPOINT to LatitudeToVelocityPID:UPDATE(Time:Seconds, GeoCoordinates:LAT).
	// SET controlStick:PITCH to LatVelocityToPitchPID:UPDATE(Time:seconds, velLat).
	LOCK STEERING TO UP.
	// //Update Yaw PID loop
	// SET LongitudeToVelocityPID:SETPOINT to targetLongitude.
	// SET LngVelocityToYawPID:SETPOINT to LongitudeToVelocityPID:UPDATE(Time:seconds, GeoCoordinates:LNG).
	// SET controlStick:YAW to LngVelocityToYawPID:UPDATE(Time:Seconds, velLng).
	
	//Update Roll PID loop
	//SET rollPID:SETPOINT to targetRoll.
	
	//Fancy terminal pictures
	SET error to (alt:radar - AltitudeToVelocityPID:SETPOINT).
	SET velocityError to (verticalspeed - VelocityToThrottlePID:SETPOINT).
	updatescreen().
}


