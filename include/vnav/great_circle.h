#pragma once

#include <cmath>
#include <tuple>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace vnav {

struct GeoPoint {
    double latitude_deg;
    double longitude_deg;

    GeoPoint() : latitude_deg(0.0), longitude_deg(0.0) {}
    GeoPoint(double lat, double lon) : latitude_deg(lat), longitude_deg(lon) {}
};

class GreatCircle {
public:
    static constexpr double EARTH_RADIUS_M = 6371008.8;
    static constexpr double NM_TO_M = 1852.0;
    static constexpr double DEG_TO_RAD = M_PI / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / M_PI;

    static double toRad(double deg) { return deg * DEG_TO_RAD; }
    static double toDeg(double rad) { return rad * RAD_TO_DEG; }

    static double distanceMeters(const GeoPoint& p1, const GeoPoint& p2) {
        double lat1 = toRad(p1.latitude_deg);
        double lat2 = toRad(p2.latitude_deg);
        double dlat = toRad(p2.latitude_deg - p1.latitude_deg);
        double dlon = toRad(p2.longitude_deg - p1.longitude_deg);
        double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
                   std::cos(lat1) * std::cos(lat2) *
                   std::sin(dlon / 2.0) * std::sin(dlon / 2.0);
        double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
        return EARTH_RADIUS_M * c;
    }

    static double distanceNm(const GeoPoint& p1, const GeoPoint& p2) {
        return distanceMeters(p1, p2) / NM_TO_M;
    }

    static double initialBearingDeg(const GeoPoint& p1, const GeoPoint& p2) {
        double lat1 = toRad(p1.latitude_deg);
        double lat2 = toRad(p2.latitude_deg);
        double dlon = toRad(p2.longitude_deg - p1.longitude_deg);
        double y = std::sin(dlon) * std::cos(lat2);
        double x = std::cos(lat1) * std::sin(lat2) -
                   std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
        double bearing = toDeg(std::atan2(y, x));
        return std::fmod(bearing + 360.0, 360.0);
    }

    static double finalBearingDeg(const GeoPoint& p1, const GeoPoint& p2) {
        double bearing = initialBearingDeg(p2, p1) + 180.0;
        return std::fmod(bearing + 360.0, 360.0);
    }

    static GeoPoint intermediatePoint(const GeoPoint& p1, const GeoPoint& p2, double fraction) {
        double lat1 = toRad(p1.latitude_deg);
        double lon1 = toRad(p1.longitude_deg);
        double lat2 = toRad(p2.latitude_deg);
        double lon2 = toRad(p2.longitude_deg);

        double d = 2.0 * std::asin(std::sqrt(
            std::sin((lat1 - lat2) / 2.0) * std::sin((lat1 - lat2) / 2.0) +
            std::cos(lat1) * std::cos(lat2) *
            std::sin((lon1 - lon2) / 2.0) * std::sin((lon1 - lon2) / 2.0)));

        double a = std::sin((1.0 - fraction) * d) / std::sin(d);
        double b = std::sin(fraction * d) / std::sin(d);

        double x = a * std::cos(lat1) * std::cos(lon1) +
                   b * std::cos(lat2) * std::cos(lon2);
        double y = a * std::cos(lat1) * std::sin(lon1) +
                   b * std::cos(lat2) * std::sin(lon2);
        double z = a * std::sin(lat1) + b * std::sin(lat2);

        GeoPoint result;
        result.latitude_deg = toDeg(std::atan2(z, std::sqrt(x * x + y * y)));
        result.longitude_deg = toDeg(std::atan2(y, x));
        return result;
    }

    static GeoPoint pointAtDistanceBearing(const GeoPoint& start, double distance_m, double bearing_deg) {
        double lat1 = toRad(start.latitude_deg);
        double lon1 = toRad(start.longitude_deg);
        double brng = toRad(bearing_deg);
        double ang_dist = distance_m / EARTH_RADIUS_M;

        GeoPoint result;
        result.latitude_deg = toDeg(std::asin(
            std::sin(lat1) * std::cos(ang_dist) +
            std::cos(lat1) * std::sin(ang_dist) * std::cos(brng)));
        result.longitude_deg = toDeg(lon1 + std::atan2(
            std::sin(brng) * std::sin(ang_dist) * std::cos(lat1),
            std::cos(ang_dist) - std::sin(lat1) * std::sin(toRad(result.latitude_deg))));
        return result;
    }

    static double crossTrackDistanceM(const GeoPoint& a, const GeoPoint& b, const GeoPoint& p) {
        double d13 = distanceMeters(a, p) / EARTH_RADIUS_M;
        double th13 = toRad(initialBearingDeg(a, p));
        double th12 = toRad(initialBearingDeg(a, b));
        return std::asin(std::sin(d13) * std::sin(th13 - th12)) * EARTH_RADIUS_M;
    }

    static double alongTrackDistanceM(const GeoPoint& a, const GeoPoint& b, const GeoPoint& p) {
        double d13 = distanceMeters(a, p) / EARTH_RADIUS_M;
        double dxt = crossTrackDistanceM(a, b, p) / EARTH_RADIUS_M;
        return std::acos(std::cos(d13) / std::cos(dxt)) * EARTH_RADIUS_M;
    }
};

}
