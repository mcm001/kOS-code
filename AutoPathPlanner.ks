run hoverkraftlib.ks.

declare function changeAltitude {
    local parameter targetAltitude.
    local parameter targetHeading.
    
    set targetLat to SHIP:LATITUDE.
    set targetLng to ship:SHIP:LONGITUDE.

    set xy_tolerence to 10. // n meter tolerence in x and y

    until latdistance(ship:lat, ship:lng, targetLat, targetLng) < 10 and abs(alt:radar - targetAltitude) < 2 {
        updatePIDloops(targetLat, targetLng, targetAltitude, targetHeading).
    }

}