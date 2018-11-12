switch to 0.
run pid_lib.ks.

clearscreen.
// takeoffandsetup().

SET targetZeroSpeedAltitude to 25. //SET target "zero speed altitude" (alt:radar is 19-20 when landed)
SET landingModeTriggerspeed to -5. //SET speed at which landing mode will be trigggered
SET landingModeVelocity to -3.5. //SET slow descent speed
SET landingModeTWRdelta to 0.3. //SET "Dumb P" value for landing
SET targetVelocityNorth to 0.
SET targetVelocityEast to 0.
SET targetRoll to 0.



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
SET KpPitch TO 1.5. //angle -> steering (pitch) ctrl
SET KiPitch TO 4.
SET KdPitch TO 0.
SET minimumYAWctrl TO 0.1.
SET maximumYAWctrl TO 1.
SET KpYaw TO 1.5. //angle -> steering (yaw) PID
SET KiYaw TO 4.
SET KdYaw TO 0.
SET minimumYAWctrl TO 0.1.
SET maximumYAWctrl TO 1.
SET KpROLL TO 1.5. //angle -> steering (roll) PID
SET KiROLL TO 4.
SET KdROLL TO 0.
SET minimumROLLctrl TO 0.1.
SET maximumROLLctrl TO 1.


global AltitudeToVelocityPID is PIDLOOP(KpAV, KiAV, KdAV, minimumAV, maximumAV).
global VelocityToThrottlePID is PIDLOOP(KpVT, KiVT, KdVT, minimumVT, maximumVT). //PID stuff
global rollPID is PIDLOOP(KpROLL, KiROLL, KdROLL, minimumROLLctrl, maximumROLLctrl).
global pitchPID is PIDLOOP(KpPitch, KiPitch, KdPitch, minimumYAWctrl, maximumYAWctrl).
global yawPID is PIDLOOP(KpYaw, KiYaw, KdYaw, minimumYAWctrl, maximumYAWctrl).

SET throttle to 0.
brakes on.
rcs on.
SET runmode to 1.
clearscreen.
SET targetheight to 50. //SET desired height.
SET targetRoll to 0. //set target roll
SET targetPitch to 0.
SET targetYaw to 0.
SET previousVelLat to 0.
SET previousVelLng to 0.
SET previousTime to time:seconds.

SET error to 0.
lock steering to up. //TODO comment this out
log "Time,Error" to PID.csv.
until runmode = 0 {

	//Update the throttle PID loop
	SET AltitudeToVelocityPID:SETPOINT TO targetheight. //setpoint for velocity PID. Changes target altitude. Is a constant-ish
	SET VelocityToThrottlePID:SETPOINT TO -2 .//AltitudeToVelocityPID:UPDATE(TIME:SECONDS, alt:radar). //PID function for position -> velocity. and SETPOINT for throttle PID, calculated by velocity PID loop.
	SET tval to VelocityToThrottlePID:UPDATE(TIME:SECONDS, verticalspeed). //PID function for calcualted target velocity -> throttle value
	lock throttle to tval.
	
	//Update Pitch and Yaw PID loops
	
	//Update Roll PID loop
	SET rollPID:SETPOINT to targetRoll.
	SET SHIP:CONTROL:ROLL to rollPID:UPDATE(TIME:SECONDS, VECTORANGLE(UP:VECTOR, SHIP:FACING:STARVECTOR)).
	
	//Fancy terminal pictures
	SET error to (alt:radar - AltitudeToVelocityPID:SETPOINT).
	SET velocityError to (verticalspeed - VelocityToThrottlePID:SETPOINT).
	updatescreen().
}


