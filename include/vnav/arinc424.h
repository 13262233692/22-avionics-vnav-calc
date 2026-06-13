#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include "great_circle.h"

namespace vnav {

enum class AltitudeConstraintType {
    ALT_NONE = 0,
    ALT_AT = 1,
    ALT_AT_OR_ABOVE = 2,
    ALT_AT_OR_BELOW = 3,
    ALT_BETWEEN = 4,
    ALT_GLIDE_SLOPE = 5
};

struct AltitudeConstraint {
    AltitudeConstraintType type;
    double altitude1_ft;
    double altitude2_ft;
    bool is_flight_level;
};

enum class WaypointType {
    WP_AIRPORT = 1,
    WP_NDB = 2,
    WP_VOR = 3,
    WP_DME = 4,
    WP_FIX = 5,
    WP_RUNWAY = 6,
    WP_USER = 7
};

enum class SpeedLimit {
    NONE = 0,
    AT_OR_BELOW = 1,
    AT_OR_ABOVE = 2,
    AT = 3
};

struct Waypoint {
    std::string identifier;
    std::string region_code;
    std::string icao_code;
    WaypointType type;
    GeoPoint position;
    double magnetic_variation;
    AltitudeConstraint altitude_constraint;
    double speed_limit_kt;
    SpeedLimit speed_limit_type;
    double turn_radius_nm;
    bool is_fly_by;
    bool is_overfly;
    std::string description;

    Waypoint()
        : type(WaypointType::WP_FIX),
        magnetic_variation(0.0),
        speed_limit_kt(0.0),
        speed_limit_type(SpeedLimit::NONE),
        turn_radius_nm(0.0),
        is_fly_by(true),
        is_overfly(false) {
        altitude_constraint.type = AltitudeConstraintType::ALT_NONE;
        altitude_constraint.altitude1_ft = 0.0;
        altitude_constraint.altitude2_ft = 0.0;
        altitude_constraint.is_flight_level = false;
    }
};

enum class ProcedureType {
    PROC_SID = 1,
    PROC_STAR = 2,
    PROC_APPROACH = 3,
    PROC_ENROUTE = 4
};

struct ProcedureLeg {
    std::string waypoint_id;
    std::string recommended_navaid;
    double arc_radius_nm;
    double magnetic_course_deg;
    double distance_nm;
    AltitudeConstraint alt_constraint;
    double speed_limit_kt;
    SpeedLimit speed_limit_type;
    bool is_mandatory;
    int leg_sequence_number;

    ProcedureLeg()
        : arc_radius_nm(0.0),
          magnetic_course_deg(0.0),
          distance_nm(0.0),
          speed_limit_kt(0.0),
          speed_limit_type(SpeedLimit::NONE),
          is_mandatory(false),
          leg_sequence_number(0) {
        alt_constraint.type = AltitudeConstraintType::ALT_NONE;
        alt_constraint.altitude1_ft = 0.0;
        alt_constraint.altitude2_ft = 0.0;
        alt_constraint.is_flight_level = false;
    }
};

struct Procedure {
    std::string name;
    ProcedureType type;
    std::string airport_icao;
    std::string runway;
    std::string transition;
    std::vector<ProcedureLeg> legs;
};

struct Airport {
    std::string icao_code;
    std::string region_code;
    std::string name;
    GeoPoint position;
    double elevation_ft;
    double transition_altitude_ft;
    std::vector<std::string> runways;
    std::vector<Procedure> sids;
    std::vector<Procedure> stars;
    std::vector<Procedure> approaches;
};

struct NavigationDatabase {
    std::map<std::string, Airport> airports;
    std::map<std::string, Waypoint> waypoints;
    std::map<std::string, Waypoint> ndbs;
    std::map<std::string, Waypoint> vors;
};

class Arinc424Parser {
public:
    NavigationDatabase parseFile(const std::string& filepath);
    NavigationDatabase parseString(const std::string& content);
    std::vector<std::string> getParseErrors() const { return errors_; }

private:
    std::vector<std::string> errors_;

    void parseRecord(const std::string& record, NavigationDatabase& db);
    void parseAirportRecord(const std::string& record, NavigationDatabase& db);
    void parseWaypointRecord(const std::string& record, NavigationDatabase& db);
    void parseNdbRecord(const std::string& record, NavigationDatabase& db);
    void parseVorRecord(const std::string& record, NavigationDatabase& db);
    void parseSidRecord(const std::string& record, NavigationDatabase& db);
    void parseStarRecord(const std::string& record, NavigationDatabase& db);
    void parseApproachRecord(const std::string& record, NavigationDatabase& db);

    std::string getField(const std::string& record, size_t start, size_t length);
    double parseLatitude(const std::string& field);
    double parseLongitude(const std::string& field);
    AltitudeConstraint parseAltitudeConstraint(const std::string& field1, const std::string& field2);
    double parseMagneticVariation(const std::string& field);
    WaypointType parseWaypointType(char code);
    AltitudeConstraintType parseAltConstraintType(char code);
};

}
