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

declare function initscreen{
	print("Forward velocity data: ") at (0,0).
	print("Velocity:         | Target:        | PID output: ") at (0,1).

	print("Pitch input data: ") at (0,3).
	print("Current:          | Target:        | PID output: ") at (0,4).

	print("Altitude PID data: ") at (0,6).
	print("Altitude:         | Target:        | PID output: ") at (0,7).

	print("Throttle input data: ") at (0,9).
	print("Velocity:         | Target:        | PID output: ") at (0,10).

	print("Lateral velocity data: ") at (0,12).
	print("Velocity:         | Target:        | PID output: ") at (0,13).
	
	print("Roll input data: ") at (0,15).
	print("Current:          | Target:        | PID output: ") at (0,16).

	print("Yaw input data: ") at (0,18).
	print("Heading:          | Target:        | PID output: ") at (0,19).

	print("At a glance:") at (0,21).
	print("Speed: ") at (0,22).
	print("Angle: ") at (0,23).


}

declare function updatescreen {
	print(round((ship:velocity:surface * ship:facing:forevector), 2)) at (11,1).
	print(round(targetForeSpeed, 2)) at (28,1).
	print(round(foreSpeedPID:OUTPUT, 4)) at (49,1).

	print(round(pitchPID:SETPOINT, 2)) at (28,4).
	print(round((ship:velocity:surface * ship:facing:forevector),2)) at (11,4).
	print(round(pitchPID:OUTPUT,4)) at (49,4).
	
	print(round(ALT:RADAR,2)) at (11,7).
	print(round(targetheight,2)) at (28,7).
	print(round(AltitudeToVelocityPID:OUTPUT,4)) at (49,7).

	print(round(ship:verticalspeed,2)) at (11,10).
	print(round(AltitudeToVelocityPID:OUTPUT,2)) at (28,10).
	print(round(VelocityToThrottlePID:OUTPUT,2)) at (49,10).

	print(round(latSpeedPID:SETPOINT,2)) at (28,13).
	print(round((ship:velocity:surface * ship:facing:starvector),2)) at (11,13).
	print(round(latSpeedPID:OUTPUT,4)) at (49,13).

	print(round(rollPID:SETPOINT,2)) at (28,16).
	print(round((VECTORANGLE(UP:VECTOR, SHIP:FACING:STARVECTOR) - 90),2)) at (11,16).
	print(round(rollPID:output,4)) at (49,16).

	print(round(yawPID:SETPOINT,2)) at (28,19).
	print(round(headin,2)) at (11,19).
	print(round(yawPID:output,4)) at (49,19).

	print( round((ship:velocity:surface * ship:facing:forevector),2) + " | " + round((ship:velocity:surface * ship:facing:starvector),2) + " | " + round(ship:verticalspeed,2)) at (7,22). //  at (7,22).


}


clearscreen.
// takeoffandsetup().
initscreen().

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
SET KpForeSpeed TO 1. SET KiForeSpeed TO 0. SET KdForeSpeed TO 0.0. SET minimumForeSpeed TO -20. SET maximumForeSpeed TO 20.
//angle -> steering (pitch) ctrl
SET KpPitch TO 0.15. SET KiPitch TO 0.1. SET KdPitch TO 0.17. SET minimumYAWctrl TO -0.5. SET maximumYAWctrl TO 0.5.

//angle -> steering (yaw) PID
SET KpYaw TO 0.1. SET KiYaw TO 0.0. SET KdYaw TO 0.2. SET minimumYAWctrl TO -1. SET maximumYAWctrl TO 1.

//lateral velocity -> roll angle PID
SET KpLatSpeed TO 2. SET KiLatSpeed TO 0. SET KdLatSpeed TO 0.0. SET minimumLatSpeed TO -20. SET maximumLatSpeed TO 20.
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
// clearscreen.
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
	SET rollPID:SETPOINT to latSpeedPID:UPDATE(TIME:SECONDS, (ship:velocity:surface * ship:facing:starvector)).
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



