run hoverkraftlib.ks.

declare function changeAltitude {
    local parameter targetAltitude.
    local parameter targetHeading.
    
    set targetLat to ship:lat.//TODO current altitude
    set targetLng to ship:lng.// TODO set curent longiude

    set xy_tolerence to 10. //10 feet tolerence

    until distance(ship:lat, ship:lng, targetLat, targetLng) < 10 and abs(alt:radar - targetAltitude) < 2 {
        updatePIDloops(targetLat, targetLng, targetAltitude, targetHeading).
    }

}