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
