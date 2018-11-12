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

	// print("Throttle: " + tval) at(0,1).
	print("velocity PID Output: " + AltitudeToVelocityPID:OUTPUT + "                   ") at(0,2).
	print("velocity PID Setpoint: " + AltitudeToVelocityPID:SETPOINT + "    ") AT(0,3).
	print("Radar Altitude: " + alt:radar) at(0,4).
	print("Position Error: " + error) at(0,5).
	PRINT("Speed: " + verticalspeed) at(0,6).
	print ("speed setpoint: " + VelocityToThrottlePID:SETPOINT) AT(0,7).
	print("Velocity error: " + velocityError) at(0,8).
	print("Roll PID setpoint: " + rollPID:SETPOINT) at (0,9).
	print("Roll PID output: " + rollPID:OUTPUT) at (0,10).
	print("roll PID error: " + rollError) at (0,11).

	// print("Latitude: " + GeoCoordinates:LAT) at(0,9).
	// print("Longitude: " + GeoCoordinates:LNG) at(0,10).

}


clearscreen.
// takeoffandsetup().

SET targetZeroSpeedAltitude to 25. //SET target "zero speed altitude" (alt:radar is 19-20 when landed)
SET landingModeTriggerspeed to -5. //SET speed at which landing mode will be trigggered
SET landingModeVelocity to -3.5. //SET slow descent speed
SET landingModeTWRdelta to 0.3. //SET "Dumb P" value for landing
SET targetVelocityNorth to 0.
SET targetVelocityEast to 0.
SET targetRoll to 0.


//altitude -> velocity PID
SET KpAV TO 0.4. SET KiAV TO 0.0. SET KdAV TO 0. SET minimumAV TO -20. SET maximumAV TO 80.
//velocity -> throttle PID
SET KpVT TO 0.2. SET KiVT TO 0.4. SET KdVT TO 0.0. SET minimumVT TO 0.1. SET maximumVT TO 1.


//forward velocity -> pitch angle PID
SET KpForeSpeed TO 2. SET KiForeSpeed TO 0. SET KdForeSpeed TO 0.0. SET minimumForeSpeed TO -5. SET maximumForeSpeed TO 5.
//angle -> steering (pitch) ctrl
SET KpPitch TO 0.1. SET KiPitch TO 0.1. SET KdPitch TO 0.17. SET minimumYAWctrl TO -0.5. SET maximumYAWctrl TO 0.5.


//angle -> steering (yaw) PID
SET KpYaw TO 0.1. SET KiYaw TO 0.0. SET KdYaw TO 0.2. SET minimumYAWctrl TO -1. SET maximumYAWctrl TO 1.


//lateral velocity -> roll angle PID
SET KpLatSpeed TO 0.2. SET KiLatSpeed TO 0. SET KdLatSpeed TO 0.0. SET minimumLatSpeed TO -5. SET maximumLatSpeed TO 5.
//angle -> steering (roll) PID
SET KpROLL TO 0.02. SET KiROLL TO 0.18. SET KdROLL TO 0.06. SET minimumROLLctrl TO -1. SET maximumROLLctrl TO 1.



global AltitudeToVelocityPID is PIDLOOP(KpAV, KiAV, KdAV, minimumAV, maximumAV).
global VelocityToThrottlePID is PIDLOOP(KpVT, KiVT, KdVT, minimumVT, maximumVT). //PID stuff
global pitchPID is PIDLOOP(KpPitch, KiPitch, KdPitch, minimumYAWctrl, maximumYAWctrl).
global yawPID is PIDLOOP(KpYaw, KiYaw, KdYaw, minimumYAWctrl, maximumYAWctrl).
global rollPID is PIDLOOP(KpROLL, KiROLL, KdROLL, minimumROLLctrl, maximumROLLctrl).
global foreSpeedPID is PIDLOOP(KpForeSpeed, KiForeSpeed, KdPitch, minimumForeSpeed, maximumForeSpeed).
global latSpeedPID is PIDLOOP(KpLatSpeed, KiLatSpeed, KdLatSpeed, minimumLatSpeed, maximumLatSpeed).



SET throttle to 0.
brakes on.
rcs on.
SET runmode to 1.
clearscreen.
SET targetheight to 300. //SET desired height.
SET targetRoll to 0. //set target roll
SET targetPitch to 0.
SET targetYaw to 90. //target heading
SET targetForeSpeed to 0.
SET targetLatSpeed to 0.

SET northPole TO latlng(90,0).
LOCK headin TO mod(360 - northPole:bearing,360).

// switch to 0. run hoverkraft.ks.

SET error to 0.
sas off.
rcs off.
// lock throttle to zeroAccelerationThrottle().
// lock steering to up. //TODO comment this out
// stage.
// log "Time,Error" to PID.csv.
until runmode = 0 {

	//Update the throttle PID loop
	SET AltitudeToVelocityPID:SETPOINT TO targetheight. //setpoint for velocity PID. Changes target altitude. Is a constant-ish
	SET VelocityToThrottlePID:SETPOINT TO AltitudeToVelocityPID:UPDATE(TIME:SECONDS, alt:radar). //PID function for position -> velocity. and SETPOINT for throttle PID, calculated by velocity PID loop.
	SET tval to VelocityToThrottlePID:UPDATE(TIME:SECONDS, verticalspeed). //PID function for calcualted target velocity -> throttle value
	lock throttle to tval.
	
	//Update Pitch PID
	Set foreSpeedPID:SETPOINT to targetForeSpeed. // set setpoint of forward velocity
	//set pitch setpoint to output of velocity and input the current forward speed
	SET pitchPID:SETPOINT to -foreSpeedPID:UPDATE(TIME:SECONDS, (ship:velocity:surface * ship:facing:forevector)). 
	SET SHIP:CONTROL:PITCH to pitchPID:UPDATE(TIME:SECONDS, 90 - VECTORANGLE(UP:VECTOR, SHIP:FACING:FOREVECTOR)).

	//Update Yaw PID
	set latSpeedPID:SETPOINT to targetLatSpeed.
	SET yawPID:SETPOINT to targetYaw.
	SET SHIP:CONTROL:YAW to yawPID:UPDATE(TIME:SECONDS, headin).

	//Update Roll PID loop and input lateral velocity
	SET latSpeedPID:SETPOINT to targetLatSpeed. //set target lateral speed (left rigtht)
	//update the roll PID with target sideways speed
	SET rollPID:SETPOINT to -latSpeedPID:UPDATE(TIME:SECONDS, (ship:velocity:surface * ship:facing:starvector)).
	SET SHIP:CONTROL:ROLL to rollPID:UPDATE(TIME:SECONDS, VECTORANGLE(UP:VECTOR, SHIP:FACING:STARVECTOR) - 90).
	
	//Fancy terminal pictures
	SET error to (alt:radar - AltitudeToVelocityPID:SETPOINT).
	SET velocityError to (verticalspeed - VelocityToThrottlePID:SETPOINT).
	set rollError to ((VECTORANGLE(UP:VECTOR, SHIP:FACING:STARVECTOR) - 90) - rollPID:SETPOINT ).
	updatescreen().

	// wait 0.01.

	on ag2 {
		set runmode to 0.
	}
}



