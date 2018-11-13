run once hoverkraftlib.ks.
initscreen().

stage.
switch to 0. run AutoPathPlanner.ks.

declare function changeAltitude {
    local parameter targetAltitude.
    local parameter targetHeading.
    local parameter speed.

    set initialLat to SHIP:LATITUDE.
    set initialLng to SHIP:LONGITUDE.
    set targetLat to SHIP:LATITUDE.
    set targetLng to SHIP:LONGITUDE.
    

    set xy_tolerence to 2. // n meter tolerence in x and y

    until geoposDistance(ship:LATITUDE, ship:LONGITUDE, targetLat, targetLng) < xy_tolerence and abs(alt:radar - targetAltitude) < 1 and ship:groundspeed < 0.1 {
        updatePIDloops(initialLat, initialLng, targetLat, targetLng, targetAltitude, targetHeading, 10, 1, 10).
        print(geoposDistance(ship:LATITUDE, ship:LONGITUDE, targetLat, targetLng)) at (0,25).
        print(abs(alt:radar - targetAltitude)) at (0,26).
    }
    print("done!").
}

declare function linearTravel {

}

changeAltitude(300,135,10).
