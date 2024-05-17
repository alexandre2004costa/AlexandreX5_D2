//
// Created by Alexandre on 17/05/2024.
//

#include <cmath>
#include "Harvesine.h"

double pi = M_PI;
double earthradius=6371000; //meters
/**
 * @brief Converts degrees to radians.
 * @param coord Value of the coordinate in degrees.
 * @return Value of the coordinate in radians.
 */
double toRadians(double coord){
    return coord*pi/180.0;
}

/**
 * @brief Gives the Haversine distance between two coordinates.
 * @param lat1 Latitude of the first point in degrees.
 * @param lon1 Longitude of the first point in degrees.
 * @param lat2 Latitude of the second point in degrees.
 * @param lon2 Longitude of the second point in degrees.
 * @return The Haversine distance between the two points in meters.
 */
double Harvesine::haversineDistance(double lat1, double lon1, double lat2, double lon2){
    double radLat1=toRadians(lat1);
    double radLon1=toRadians(lon1);
    double radLat2=toRadians(lat2);
    double radLon2=toRadians(lon2);

    double deltaLat=radLat2-radLat1;
    double deltaLon=radLon2-radLon1;

    double a=sin(deltaLat/2.0)*sin(deltaLat/2.0)+cos(radLat1)*cos(radLat2)*sin(deltaLon/2.0)*sin(deltaLon/2.0);
    double c=2.0*atan2(sqrt(a), sqrt(1.0-a));

    return earthradius*c;
}