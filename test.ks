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
declare function updateTerminal {
	print("zeroSpeedAltitude: " + zeroSpeedAltitude()) AT(0,1).
	print("targetZeroSpeedAltitude: " + targetZeroSpeedAltitude) AT(0,2).
	print("delta: " + delta) AT(0,3).
	print("altitude: " + (alt:radar-20)) AT(0,4).
	print("vertical speed: " + verticalspeed) AT(0,5).
	print("ground speed: " + groundspeed) AT(0,6).
	print("airspeed: " + airspeed) AT(0,7).
	print("runmode: " + runmode) AT(0,8).
	print("State: " + state + "           ") AT(0,9).
	print("throttle: " + throttle + "     ") AT(0,10).
	print("Current TWR: " + round((throttle * ship:maxthrust / (ship:mass * 9.81)),2) + "    ") AT(0,11).
	print("Gear: " + gear + "        ") AT(0,11).
	print("PID output: " + hoverslamPID:OUTPUT + "            ") at(0,12).
	print("PID 2 output: " + touchdownPID:OUTPUT + "       ") at(0,13).
	return 1.
}
declare function takeoffandsetup {
	sas off.
	rcs on.
	ag1 on.
	ag1 off.
	print("taking off").
	brakes off.
	lock steering to up.//heading(90,85).
	lock throttle to 1.
	gear off.
	set state to "accelerating to 150 m/s".
	wait until verticalspeed > 200.
	set state to "ascending to 1000m".
	until alt:radar > 2200 { lock throttle to twrToThrottle(1.05). }
}

clearscreen.
//takeoffandsetup().

set targetZeroSpeedAltitude to 100. //Set target "zero speed altitude" (alt:radar is 19-20 when landed)
set landingModeTriggerspeed to -10. //Set speed at which landing mode will be trigggered
set landingModeVelocity to -3.5. //Set slow descent speed
set landingModeTWRdelta to 0.3. //Set "Dumb P" value for landing

SET KpH TO 0.1.
SET KiH TO 0.
SET KdH TO 0.
SET MINOUTPUT TO 0.5.
SET MAXOUTPUT TO 1.
SET KpVT TO 0.01. //velocity -> throttle PID
SET KiVT TO 4.
SET KdVT TO 0.
SET minimumVT TO 0.15.
SET maximumVT TO 1.
global hoverslamPID is PIDLOOP(KpH, KiH, KdH, MINOUTPUT, MAXOUTPUT). //PID stuff
global touchdownPID is PIDLOOP(KpVT, KiVT, KdVT, minimumVT, maximumVT).


set throttle to 0.
wait until ship:verticalspeed < -10.
brakes on.
rcs on.
set counter to 0.
set runmode to 3.
clearscreen.
set state to " ".
lock steering to -SHIP:VELOCITY:SURFACE.
until runmode = 0 {
	wait 0.01.
	set delta to (zeroSpeedAltitude() - targetZeroSpeedAltitude ).
	updateTerminal().	
	log verticalspeed to PID.csv.
	
	IF runmode = 1 {
		set state to "waiting for hoverslam trigger".
		sas off.
		if zeroSpeedAltitude() < targetZeroSpeedAltitude { set runmode to 2. }
	}
	
//	if runmode = 2 { //"HOVERSLAM" - ISH
//		set state to "hoverslam-ish                  ".
//		if zeroSpeedAltitude() < targetZeroSpeedAltitude { lock throttle to 1. }
//		else if zeroSpeedAltitude() > targetZeroSpeedAltitude and (zeroSpeedAltitude() - targetZeroSpeedAltitude < 10) { lock throttle to 0.65. }
//		else { lock throttle to 0. }
//		if verticalspeed > landingModeTriggerspeed { set runmode to 3. }
//	}
	
	if runmode = 2 { //PID hoverslam
		set state to "pid hoverslam  ".
		set hoverslamPID:SETPOINT to targetZeroSpeedAltitude.
		set throttle to hoverslamPID:UPDATE(TIME:SECONDS, delta).
		if verticalspeed > landingModeTriggerspeed { set runmode to 3. }
	}
	
	if runmode = 3 {
		set state to "Landing mode".
		
		set touchdownPID:SETPOINT to landingModeVelocity.
		set throttle to touchdownPID:UPDATE(Time:seconds, verticalspeed).
		
		if alt:radar < 21 {
			lock throttle to twrToThrottle(0.5).
			set state to "Stableizing".
			set counter to (counter+1).
			wait 0.1.
		}
		if counter > 10 {
			unlock throttle.
			unlock steering.
			brakes off.
			rcs off.
			set throttle to 0.
			set runmode to 0.
		} 
	}
	if alt:radar < 150 { gear on. } //Deploy gear at trigger altitude
	if ship:verticalspeed > -80 and runmode > 1 { lock steering to up. }
	
}