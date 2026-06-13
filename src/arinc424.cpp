#include "vnav/arinc424.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>

namespace vnav {

std::string Arinc424Parser::getField(const std::string& record, size_t start, size_t length) {
    if (start >= record.size()) return "";
    size_t actual_length = std::min(length, record.size() - start);
    std::string field = record.substr(start, actual_length);
    field.erase(0, field.find_first_not_of(" \t\n\r"));
    field.erase(field.find_last_not_of(" \t\n\r") + 1);
    return field;
}

double Arinc424Parser::parseLatitude(const std::string& field) {
    if (field.size() < 9) return 0.0;
    char sign = field[0];
    int deg = std::stoi(field.substr(1, 2));
    int min = std::stoi(field.substr(3, 2));
    double sec = std::stod(field.substr(5, 4)) / 100.0;
    double lat = deg + min / 60.0 + sec / 3600.0;
    return (sign == 'S' || sign == '-') ? -lat : lat;
}

double Arinc424Parser::parseLongitude(const std::string& field) {
    if (field.size() < 10) return 0.0;
    char sign = field[0];
    int deg = std::stoi(field.substr(1, 3));
    int min = std::stoi(field.substr(4, 2));
    double sec = std::stod(field.substr(6, 4)) / 100.0;
    double lon = deg + min / 60.0 + sec / 3600.0;
    return (sign == 'W' || sign == '-') ? -lon : lon;
}

AltitudeConstraintType Arinc424Parser::parseAltConstraintType(char code) {
    switch (code) {
        case '+': return AltitudeConstraintType::ALT_AT_OR_ABOVE;
        case '-': return AltitudeConstraintType::ALT_AT_OR_BELOW;
        case 'B': return AltitudeConstraintType::ALT_BETWEEN;
        case 'G': return AltitudeConstraintType::ALT_GLIDE_SLOPE;
        default: return AltitudeConstraintType::ALT_AT;
    }
}

AltitudeConstraint Arinc424Parser::parseAltitudeConstraint(const std::string& field1, const std::string& field2) {
    AltitudeConstraint constraint;
    constraint.type = AltitudeConstraintType::ALT_NONE;
    constraint.altitude1_ft = 0.0;
    constraint.altitude2_ft = 0.0;
    constraint.is_flight_level = false;

    if (field1.empty()) return constraint;

    char type_char = field1[0];
    std::string alt_str = field1.substr(1);
    bool is_fl = false;
    if (alt_str.size() >= 2 && alt_str.substr(0, 2) == "FL") {
        is_fl = true;
        alt_str = alt_str.substr(2);
    }

    if (type_char != ' ' && !alt_str.empty()) {
        constraint.type = parseAltConstraintType(type_char);
        try {
            double alt_val = std::stod(alt_str);
            constraint.altitude1_ft = is_fl ? alt_val * 100.0 : alt_val * 100.0;
            constraint.is_flight_level = true;
        } catch (...) {}
    } else if (!alt_str.empty()) {
        constraint.type = AltitudeConstraintType::ALT_AT;
        try {
            double alt_val = std::stod(alt_str);
            constraint.altitude1_ft = is_fl ? alt_val * 100.0 : alt_val * 100.0;
            constraint.is_flight_level = true;
        } catch (...) {}
    }

    if (!field2.empty() && field2.size() > 1) {
        try {
            double alt2_val = std::stod(field2.substr(1));
            constraint.altitude2_ft = alt2_val * 100.0;
            if (constraint.type == AltitudeConstraintType::ALT_NONE) {
                constraint.type = AltitudeConstraintType::ALT_BETWEEN;
            }
        } catch (...) {}
    }

    return constraint;
}

double Arinc424Parser::parseMagneticVariation(const std::string& field) {
    if (field.empty()) return 0.0;
    try {
        char sign = field.back();
        double val = std::stod(field.substr(0, field.size() - 1));
        return (sign == 'W' || sign == '-') ? -val : val;
    } catch (...) {
        return 0.0;
    }
}

WaypointType Arinc424Parser::parseWaypointType(char code) {
    switch (code) {
        case 'A': return WaypointType::WP_AIRPORT;
        case 'N': return WaypointType::WP_NDB;
        case 'V': return WaypointType::WP_VOR;
        case 'D': return WaypointType::WP_DME;
        case 'R': return WaypointType::WP_RUNWAY;
        default: return WaypointType::WP_FIX;
    }
}

void Arinc424Parser::parseAirportRecord(const std::string& record, NavigationDatabase& db) {
    std::string icao = getField(record, 6, 4);
    if (icao.empty()) return;

    Airport apt;
    apt.icao_code = icao;
    apt.region_code = getField(record, 1, 4);
    apt.name = getField(record, 93, 30);
    apt.position.latitude_deg = parseLatitude(getField(record, 32, 11));
    apt.position.longitude_deg = parseLongitude(getField(record, 43, 12));
    apt.elevation_ft = 0.0;
    std::string elev_str = getField(record, 55, 5);
    if (!elev_str.empty()) {
        try { apt.elevation_ft = std::stod(elev_str); } catch (...) {}
    }
    apt.transition_altitude_ft = 18000.0;
    std::string trans_alt = getField(record, 27, 5);
    if (!trans_alt.empty()) {
        try { apt.transition_altitude_ft = std::stod(trans_alt); } catch (...) {}
    }

    auto it = db.airports.find(icao);
    if (it == db.airports.end()) {
        db.airports[icao] = apt;
    } else {
        if (apt.position.latitude_deg != 0.0 || apt.position.longitude_deg != 0.0) {
            it->second.position = apt.position;
        }
        if (!apt.name.empty()) it->second.name = apt.name;
    }
}

void Arinc424Parser::parseWaypointRecord(const std::string& record, NavigationDatabase& db) {
    Waypoint wp;
    wp.identifier = getField(record, 13, 5);
    if (wp.identifier.empty()) return;

    wp.region_code = getField(record, 1, 4);
    wp.icao_code = getField(record, 6, 4);
    wp.type = WaypointType::WP_FIX;
    wp.position.latitude_deg = parseLatitude(getField(record, 32, 11));
    wp.position.longitude_deg = parseLongitude(getField(record, 43, 12));
    wp.magnetic_variation = parseMagneticVariation(getField(record, 55, 5));
    wp.description = getField(record, 93, 30);

    std::string key = wp.icao_code + "/" + wp.identifier;
    db.waypoints[key] = wp;
}

void Arinc424Parser::parseNdbRecord(const std::string& record, NavigationDatabase& db) {
    Waypoint wp;
    wp.identifier = getField(record, 13, 4);
    if (wp.identifier.empty()) return;

    wp.region_code = getField(record, 1, 4);
    wp.icao_code = getField(record, 6, 4);
    wp.type = WaypointType::WP_NDB;
    wp.position.latitude_deg = parseLatitude(getField(record, 32, 11));
    wp.position.longitude_deg = parseLongitude(getField(record, 43, 12));
    wp.magnetic_variation = parseMagneticVariation(getField(record, 55, 5));
    wp.description = getField(record, 94, 30);

    std::string key = wp.icao_code + "/" + wp.identifier;
    db.ndbs[key] = wp;
    db.waypoints[key] = wp;
}

void Arinc424Parser::parseVorRecord(const std::string& record, NavigationDatabase& db) {
    Waypoint wp;
    wp.identifier = getField(record, 13, 3);
    if (wp.identifier.empty()) return;

    wp.region_code = getField(record, 1, 4);
    wp.icao_code = getField(record, 6, 4);
    wp.type = WaypointType::WP_VOR;
    wp.position.latitude_deg = parseLatitude(getField(record, 32, 11));
    wp.position.longitude_deg = parseLongitude(getField(record, 43, 12));
    wp.magnetic_variation = parseMagneticVariation(getField(record, 55, 5));
    wp.description = getField(record, 94, 30);

    std::string key = wp.icao_code + "/" + wp.identifier;
    db.vors[key] = wp;
    db.waypoints[key] = wp;
}

void Arinc424Parser::parseSidRecord(const std::string& record, NavigationDatabase& db) {
    std::string apt_icao = getField(record, 6, 4);
    std::string sid_name = getField(record, 13, 6);
    if (apt_icao.empty() || sid_name.empty()) return;

    ProcedureLeg leg;
    leg.waypoint_id = getField(record, 29, 5);
    leg.recommended_navaid = getField(record, 20, 4);
    leg.magnetic_course_deg = 0.0;
    std::string course_str = getField(record, 38, 5);
    if (!course_str.empty()) {
        try { leg.magnetic_course_deg = std::stod(course_str); } catch (...) {}
    }
    leg.distance_nm = 0.0;
    std::string dist_str = getField(record, 46, 4);
    if (!dist_str.empty()) {
        try { leg.distance_nm = std::stod(dist_str) / 10.0; } catch (...) {}
    }
    leg.alt_constraint = parseAltitudeConstraint(getField(record, 71, 5), getField(record, 76, 5));
    leg.speed_limit_kt = 0.0;
    std::string speed_str = getField(record, 65, 4);
    if (!speed_str.empty()) {
        try { leg.speed_limit_kt = std::stod(speed_str); } catch (...) {}
    }
    leg.is_mandatory = getField(record, 19, 1) == "M";
    std::string seq_str = getField(record, 27, 2);
    try { leg.leg_sequence_number = std::stoi(seq_str); } catch (...) { leg.leg_sequence_number = 0; }

    auto& airport = db.airports[apt_icao];
    airport.icao_code = apt_icao;

    Procedure* target_proc = nullptr;
    for (auto& proc : airport.sids) {
        if (proc.name == sid_name) {
            target_proc = &proc;
            break;
        }
    }
    if (!target_proc) {
        Procedure proc;
        proc.name = sid_name;
        proc.type = ProcedureType::PROC_SID;
        proc.airport_icao = apt_icao;
        proc.runway = getField(record, 10, 3);
        proc.transition = getField(record, 19, 5);
        airport.sids.push_back(proc);
        target_proc = &airport.sids.back();
    }

    if (!leg.waypoint_id.empty()) {
        target_proc->legs.push_back(leg);
    }
}

void Arinc424Parser::parseStarRecord(const std::string& record, NavigationDatabase& db) {
    std::string apt_icao = getField(record, 6, 4);
    std::string star_name = getField(record, 13, 6);
    if (apt_icao.empty() || star_name.empty()) return;

    ProcedureLeg leg;
    leg.waypoint_id = getField(record, 29, 5);
    leg.recommended_navaid = getField(record, 20, 4);
    leg.magnetic_course_deg = 0.0;
    std::string course_str = getField(record, 38, 5);
    if (!course_str.empty()) {
        try { leg.magnetic_course_deg = std::stod(course_str); } catch (...) {}
    }
    leg.distance_nm = 0.0;
    std::string dist_str = getField(record, 46, 4);
    if (!dist_str.empty()) {
        try { leg.distance_nm = std::stod(dist_str) / 10.0; } catch (...) {}
    }
    leg.alt_constraint = parseAltitudeConstraint(getField(record, 71, 5), getField(record, 76, 5));
    leg.speed_limit_kt = 0.0;
    std::string speed_str = getField(record, 65, 4);
    if (!speed_str.empty()) {
        try { leg.speed_limit_kt = std::stod(speed_str); } catch (...) {}
    }
    leg.is_mandatory = getField(record, 19, 1) == "M";
    std::string seq_str = getField(record, 27, 2);
    try { leg.leg_sequence_number = std::stoi(seq_str); } catch (...) { leg.leg_sequence_number = 0; }

    auto& airport = db.airports[apt_icao];
    airport.icao_code = apt_icao;

    Procedure* target_proc = nullptr;
    for (auto& proc : airport.stars) {
        if (proc.name == star_name) {
            target_proc = &proc;
            break;
        }
    }
    if (!target_proc) {
        Procedure proc;
        proc.name = star_name;
        proc.type = ProcedureType::PROC_STAR;
        proc.airport_icao = apt_icao;
        proc.runway = getField(record, 10, 3);
        proc.transition = getField(record, 19, 5);
        airport.stars.push_back(proc);
        target_proc = &airport.stars.back();
    }

    if (!leg.waypoint_id.empty()) {
        target_proc->legs.push_back(leg);
    }
}

void Arinc424Parser::parseApproachRecord(const std::string& record, NavigationDatabase& db) {
    std::string apt_icao = getField(record, 6, 4);
    std::string app_name = getField(record, 13, 6);
    if (apt_icao.empty() || app_name.empty()) return;

    ProcedureLeg leg;
    leg.waypoint_id = getField(record, 29, 5);
    leg.recommended_navaid = getField(record, 20, 4);
    leg.magnetic_course_deg = 0.0;
    std::string course_str = getField(record, 38, 5);
    if (!course_str.empty()) {
        try { leg.magnetic_course_deg = std::stod(course_str); } catch (...) {}
    }
    leg.distance_nm = 0.0;
    std::string dist_str = getField(record, 46, 4);
    if (!dist_str.empty()) {
        try { leg.distance_nm = std::stod(dist_str) / 10.0; } catch (...) {}
    }
    leg.alt_constraint = parseAltitudeConstraint(getField(record, 71, 5), getField(record, 76, 5));
    leg.speed_limit_kt = 0.0;
    std::string speed_str = getField(record, 65, 4);
    if (!speed_str.empty()) {
        try { leg.speed_limit_kt = std::stod(speed_str); } catch (...) {}
    }

    auto& airport = db.airports[apt_icao];
    airport.icao_code = apt_icao;

    Procedure* target_proc = nullptr;
    for (auto& proc : airport.approaches) {
        if (proc.name == app_name) {
            target_proc = &proc;
            break;
        }
    }
    if (!target_proc) {
        Procedure proc;
        proc.name = app_name;
        proc.type = ProcedureType::PROC_APPROACH;
        proc.airport_icao = apt_icao;
        proc.runway = getField(record, 10, 3);
        airport.approaches.push_back(proc);
        target_proc = &airport.approaches.back();
    }

    if (!leg.waypoint_id.empty()) {
        target_proc->legs.push_back(leg);
    }
}

void Arinc424Parser::parseRecord(const std::string& record, NavigationDatabase& db) {
    if (record.size() < 4) return;

    std::string section_code = getField(record, 4, 1);
    std::string sub_code = getField(record, 5, 1);

    if (section_code == "P") {
        if (sub_code == "A" || sub_code == "B" || sub_code == "C") {
            parseAirportRecord(record, db);
        }
    } else if (section_code == "D") {
        if (sub_code == "B" || sub_code == "C" || sub_code == "N") {
            parseWaypointRecord(record, db);
        } else if (sub_code == "G" || sub_code == "H") {
            parseNdbRecord(record, db);
        } else if (sub_code == " " || sub_code == "V") {
            parseVorRecord(record, db);
        }
    } else if (section_code == "S") {
        if (sub_code == "D") {
            parseSidRecord(record, db);
        } else if (sub_code == "E") {
            parseStarRecord(record, db);
        } else if (sub_code == "F" || sub_code == "G") {
            parseApproachRecord(record, db);
        }
    }
}

NavigationDatabase Arinc424Parser::parseString(const std::string& content) {
    NavigationDatabase db;
    errors_.clear();

    std::istringstream iss(content);
    std::string line;
    int line_num = 0;
    while (std::getline(iss, line)) {
        line_num++;
        if (!line.empty()) {
            while (line.size() < 132) {
                line.push_back(' ');
            }
            try {
                parseRecord(line, db);
            } catch (const std::exception& e) {
                errors_.push_back("Line " + std::to_string(line_num) + ": " + e.what());
            }
        }
    }

    return db;
}

NavigationDatabase Arinc424Parser::parseFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        errors_.push_back("Cannot open file: " + filepath);
        return NavigationDatabase();
    }
    std::stringstream ss;
    ss << file.rdbuf();
    return parseString(ss.str());
}

}
