declare function zeroSpeedAltitude {
	declare local vi is Vessel:VERTICALSPEED.
	declare local alt is SHIP:ALTITUDE.
	local accel is Vessel:MAXTHRUST/Vessel:MASS - 9.81.
	local x is (( (vi * vi)/accel )/2) - ((vi * vi)/accel) + alt.
	return x.
}

declare function zeroAccelerationThrottle {
	declare local tVal is (9.81 * vessel:mass / vessel:maxthrust).
	return tVal.
}

declare function targetSpeed {


set runmode to -1
UNTIL runmode = 0 {
	if runmode = -1 {
	lock steering to Up.
	}
	If runmode = 1 {
		sas off.
		lock steering to -velocity:surface.
		brakes on.
		rcs on.
		ag1 on. //remember to bind this to 3 engines only
		if ship:altitude < 8000 { set runmode to 2.	}
	}
	if runmode = 2 {
		until ship:altitude < 50 {
			print(zeroSpeedAltitude).
			if zeroSpeedAltitude() < 50 {
				lock throttle to 1. 
			} else { lock throttle to zeroAccelerationThrottle(). } //If we are below 50m according to zeroSpeedAltitude, burn. Else, don't accelerate.
		}
		set runmode to 0.
		lock throttle to 0.
		print("done").
	}



}