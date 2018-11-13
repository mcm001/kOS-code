run once hoverkraftlib.ks.

stage.
switch to 0. run AutoPathPlanner.ks.

declare function changeAltitude {
    local parameter targetAltitude.
    local parameter targetHeading is 90.
    
    set targetLat to SHIP:LATITUDE.
    set targetLng to SHIP:LONGITUDE.

    set xy_tolerence to 2. // n meter tolerence in x and y

    until geoposDistance(ship:LATITUDE, ship:LONGITUDE, targetLat, targetLng) < xy_tolerencev and abs(alt:radar - targetAltitude) < 1 and ship:groundspeed < 0.1 {
        updatePIDloops(targetLat, targetLng, targetAltitude, targetHeading).
        print(geoposDistance(ship:LATITUDE, ship:LONGITUDE, targetLat, targetLng)) at (0,25).
        print(abs(alt:radar - targetAltitude)) at (0,26).
    }
    print("done!").
}

declare function linearTravel {

}

changeAltitude(300,90).
