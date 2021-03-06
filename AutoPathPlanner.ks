switch to 0.
run once hoverkraftlib.ks.
clearscreen.
initscreen().

// stage.
// switch to 0. run AutoPathPlanner.ks.

declare function changeAltitude {
    local parameter targetAltitude.
    local parameter targetHeading.
    local parameter speed.

    set initialLat to SHIP:LATITUDE.
    set initialLng to SHIP:LONGITUDE.
    set targetLat to SHIP:LATITUDE + 0.01.
    set targetLng to SHIP:LONGITUDE.
    

    set xy_tolerence to 2. // n meter tolerence in x and y

    until geoposDistance(ship:LATITUDE, ship:LONGITUDE, targetLat, targetLng) < xy_tolerence and abs(alt:radar - targetAltitude) < 1 and ship:groundspeed < 0.1 {
        updatePIDloops(initialLat, initialLng, targetLat, targetLng, targetAltitude, targetHeading, speed, 1, 5).
        print("distance: " + geoposDistance(ship:LATITUDE, ship:LONGITUDE, targetLat, targetLng)) at (0,25).
        print(abs(alt:radar - targetAltitude)) at (0,26).
        

        // wait 0.1.

    }
    print("done!").
}

declare function linearTravel {

}

changeAltitude(300,90,5).
