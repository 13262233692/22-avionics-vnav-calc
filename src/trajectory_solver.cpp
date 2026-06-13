#include "vnav/trajectory_solver.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace vnav {

TrajectorySolver::TrajectorySolver()
    : current_time_sec_(0.0),
      current_fuel_kg_(15000.0),
      distance_along_route_nm_(0.0),
      cumulative_fuel_burn_(0.0),
      initialized_(false),
      complete_(false) {
}

TrajectorySolver::TrajectorySolver(const TrajectoryConfig& config)
    : config_(config),
      current_time_sec_(0.0),
      current_fuel_kg_(15000.0),
      distance_along_route_nm_(0.0),
      cumulative_fuel_burn_(0.0),
      initialized_(false),
      complete_(false) {
}

void TrajectorySolver::setConfig(const TrajectoryConfig& config) {
    config_ = config;
}

void TrajectorySolver::setAircraft(const AircraftPerformance& ac) {
    aircraft_ = ac;
}

void TrajectorySolver::setLnavConfig(const LnavConfig& config) {
    lnav_.config() = config;
}

void TrajectorySolver::setVnavConfig(const VnavConfig& config) {
    vnav_.configure(config);
}

void TrajectorySolver::loadFlightPlan(const std::vector<Waypoint>& waypoints) {
    lnav_.setFlightPlan(waypoints);
    vnav_.autoGenerateConstraints(lnav_);
    vnav_.computeVerticalProfile(lnav_);
}

void TrajectorySolver::loadFlightPlanFromArinc(const NavigationDatabase& db,
                                                 const std::string& origin,
                                                 const std::string& sid_name,
                                                 const std::string& runway_dep,
                                                 const std::vector<std::string>& enroute_wps,
                                                 const std::string& destination,
                                                 const std::string& star_name,
                                                 const std::string& runway_arr) {
    std::vector<Waypoint> waypoints;

    auto origin_it = db.airports.find(origin);
    if (origin_it != db.airports.end()) {
        Waypoint dep_wp;
        dep_wp.identifier = origin;
        dep_wp.icao_code = origin;
        dep_wp.type = WaypointType::WP_AIRPORT;
        dep_wp.position = origin_it->second.position;
        dep_wp.altitude_constraint.type = AltitudeConstraintType::ALT_AT;
        dep_wp.altitude_constraint.altitude1_ft = origin_it->second.elevation_ft;
        waypoints.push_back(dep_wp);

        for (const auto& sid : origin_it->second.sids) {
            if (sid.name == sid_name || (sid_name.empty() && !sid.legs.empty())) {
                for (const auto& leg : sid.legs) {
                    std::string key = origin + "/" + leg.waypoint_id;
                    auto wp_it = db.waypoints.find(key);
                    if (wp_it != db.waypoints.end()) {
                        Waypoint wp = wp_it->second;
                        wp.altitude_constraint = leg.alt_constraint;
                        wp.speed_limit_kt = leg.speed_limit_kt;
                        wp.speed_limit_type = leg.speed_limit_type;
                        waypoints.push_back(wp);
                    }
                }
                break;
            }
        }
    }

    for (const auto& wp_id : enroute_wps) {
        for (const auto& [key, wp] : db.waypoints) {
            if (wp.identifier == wp_id) {
                waypoints.push_back(wp);
                break;
            }
        }
    }

    auto dest_it = db.airports.find(destination);
    if (dest_it != db.airports.end()) {
        for (const auto& star : dest_it->second.stars) {
            if (star.name == star_name || (star_name.empty() && !star.legs.empty())) {
                for (const auto& leg : star.legs) {
                    std::string key = destination + "/" + leg.waypoint_id;
                    auto wp_it = db.waypoints.find(key);
                    if (wp_it != db.waypoints.end()) {
                        Waypoint wp = wp_it->second;
                        wp.altitude_constraint = leg.alt_constraint;
                        wp.speed_limit_kt = leg.speed_limit_kt;
                        wp.speed_limit_type = leg.speed_limit_type;
                        waypoints.push_back(wp);
                    }
                }
                break;
            }
        }

        Waypoint arr_wp;
        arr_wp.identifier = destination;
        arr_wp.icao_code = destination;
        arr_wp.type = WaypointType::WP_AIRPORT;
        arr_wp.position = dest_it->second.position;
        arr_wp.altitude_constraint.type = AltitudeConstraintType::ALT_AT;
        arr_wp.altitude_constraint.altitude1_ft = dest_it->second.elevation_ft;
        waypoints.push_back(arr_wp);
    }

    loadFlightPlan(waypoints);
}

void TrajectorySolver::setInitialConditions(const GeoPoint& start_pos,
                                             double initial_alt_ft,
                                             double initial_cas_kt,
                                             double initial_track_deg,
                                             double initial_fuel_kg) {
    double initial_tas_ms = InternationalStandardAtmosphere::casToTas(
        VnavStateMachine::knotsToMetersPerSec(initial_cas_kt),
        VnavStateMachine::feetToMeters(initial_alt_ft));
    lnav_.initialize(start_pos, initial_track_deg, initial_tas_ms);
    vnav_.initialize(initial_alt_ft, initial_cas_kt);
    current_fuel_kg_ = initial_fuel_kg;
    current_time_sec_ = 0.0;
    distance_along_route_nm_ = 0.0;
    cumulative_fuel_burn_ = 0.0;
    initialized_ = true;
    complete_ = false;
    trajectory_.clear();
}

TrajectoryPoint TrajectorySolver::createPoint(const LnavState& lnav_state,
                                               const VnavState& vnav_state,
                                               double time_sec) const {
    TrajectoryPoint pt;
    pt.time_sec = time_sec;
    pt.position = lnav_state.position;
    pt.altitude_ft = vnav_state.altitude_ft;
    pt.altitude_m = VnavStateMachine::feetToMeters(vnav_state.altitude_ft);
    pt.ground_track_deg = lnav_state.ground_track_deg;
    pt.heading_deg = lnav_state.ground_track_deg;
    pt.ground_speed_ms = lnav_state.ground_speed_ms;
    pt.ground_speed_kt = VnavStateMachine::metersPerSecToKnots(lnav_state.ground_speed_ms);
    pt.tas_ms = vnav_state.tas_ms;
    pt.tas_kt = VnavStateMachine::metersPerSecToKnots(vnav_state.tas_ms);
    pt.cas_kt = vnav_state.cas_kt;
    pt.mach_number = vnav_state.mach_number;
    pt.vertical_speed_fpm = vnav_state.vertical_speed_fpm;
    pt.vertical_speed_mps = VnavStateMachine::fpmToMps(vnav_state.vertical_speed_fpm);
    pt.pitch_deg = vnav_state.pitch_deg;
    pt.roll_deg = (lnav_state.phase == LnavPhase::TURNING) ?
        (lnav_state.turn_rate_deg_per_sec > 0 ? 15.0 : -15.0) : 0.0;

    ISAData isa = InternationalStandardAtmosphere::compute(pt.altitude_m);
    pt.true_air_temp_k = isa.temperature + config_.isa_deviation_c;
    pt.total_temp_k = pt.true_air_temp_k * (1.0 + 0.2 * pt.mach_number * pt.mach_number);
    pt.static_pressure_pa = isa.pressure;
    pt.density_kgm3 = isa.density;
    pt.speed_of_sound_ms = isa.speed_of_sound;
    pt.flight_path_angle_deg = vnav_state.flight_path_angle_deg;
    pt.distance_along_route_nm = distance_along_route_nm_;
    pt.distance_to_next_wp_nm = lnav_state.distance_to_next_nm;
    pt.cross_track_error_nm = lnav_state.cross_track_error_nm;
    pt.current_wp_index = lnav_state.current_waypoint_index;
    pt.lnav_phase = lnav_state.phase;
    pt.vnav_phase = vnav_state.phase;
    pt.fuel_remaining_kg = current_fuel_kg_;
    pt.fuel_burn_kg = cumulative_fuel_burn_;

    return pt;
}

double TrajectorySolver::computeWindCorrectedTrack(double track_deg, double tas_ms) const {
    if (!config_.include_wind || config_.wind_speed_ms <= 0.0) return track_deg;

    double wind_dir_rad = GreatCircle::toRad(config_.wind_direction_deg);
    double track_rad = GreatCircle::toRad(track_deg);

    double wind_ns = -config_.wind_speed_ms * std::cos(wind_dir_rad);
    double wind_ew = -config_.wind_speed_ms * std::sin(wind_dir_rad);

    double desired_ns = tas_ms * std::cos(track_rad);
    double desired_ew = tas_ms * std::sin(track_rad);

    double required_ns = desired_ns - wind_ns;
    double required_ew = desired_ew - wind_ew;

    double heading = GreatCircle::toDeg(std::atan2(required_ew, required_ns));
    return std::fmod(heading + 360.0, 360.0);
}

double TrajectorySolver::computeGroundSpeed(double track_deg, double tas_ms, double heading_deg) const {
    double heading_rad = GreatCircle::toRad(heading_deg);
    double tas_ns = tas_ms * std::cos(heading_rad);
    double tas_ew = tas_ms * std::sin(heading_rad);

    double wind_ns = 0.0, wind_ew = 0.0;
    if (config_.include_wind) {
        double wind_dir_rad = GreatCircle::toRad(config_.wind_direction_deg);
        wind_ns = -config_.wind_speed_ms * std::cos(wind_dir_rad);
        wind_ew = -config_.wind_speed_ms * std::sin(wind_dir_rad);
    }

    double gs_ns = tas_ns + wind_ns;
    double gs_ew = tas_ew + wind_ew;
    return std::sqrt(gs_ns * gs_ns + gs_ew * gs_ew);
}

double TrajectorySolver::computeFuelFlow(VnavPhase phase, double altitude_ft, double mach) const {
    double base_flow = 0.0;
    switch (phase) {
        case VnavPhase::CLIMB:
            base_flow = aircraft_.climb_fuel_flow_kg_per_hr;
            break;
        case VnavPhase::CRUISE:
            base_flow = aircraft_.cruise_fuel_flow_kg_per_hr;
            break;
        case VnavPhase::DESCENT:
        case VnavPhase::APPROACH:
            base_flow = aircraft_.descent_fuel_flow_kg_per_hr;
            break;
        default:
            base_flow = 500.0;
    }

    double alt_factor = 1.0;
    if (altitude_ft > 0.0) {
        alt_factor = std::max(0.5, 1.0 - altitude_ft / 80000.0);
    }

    return base_flow * alt_factor;
}

void TrajectorySolver::advanceTime() {
    current_time_sec_ += config_.integration_dt_sec;
}

void TrajectorySolver::integrateStep() {
    if (!initialized_ || complete_) return;

    double dt = config_.integration_dt_sec;

    const LnavState& lnav_state = lnav_.getState();
    const VnavState& vnav_state = vnav_.getState();

    double heading_deg = computeWindCorrectedTrack(lnav_state.ground_track_deg, vnav_state.tas_ms);
    double ground_speed_ms = computeGroundSpeed(
        lnav_state.ground_track_deg, vnav_state.tas_ms, heading_deg);

    double prev_dist = 0.0;
    if (!trajectory_.empty()) {
        prev_dist = trajectory_.back().distance_along_route_nm;
    }

    LnavState new_lnav = lnav_.update(dt, ground_speed_ms);
    distance_along_route_nm_ = prev_dist +
        VnavStateMachine::metersToNm(ground_speed_ms * dt);

    VnavState new_vnav = vnav_.update(dt, ground_speed_ms, distance_along_route_nm_);

    if (config_.enable_fuel_calc) {
        double fuel_flow_kgph = computeFuelFlow(new_vnav.phase, new_vnav.altitude_ft, new_vnav.mach_number);
        double fuel_burn = fuel_flow_kgph * dt / 3600.0;
        cumulative_fuel_burn_ += fuel_burn;
        current_fuel_kg_ = std::max(0.0, current_fuel_kg_ - fuel_burn);
    }

    TrajectoryPoint pt = createPoint(new_lnav, new_vnav, current_time_sec_);

    double last_output_time = 0.0;
    if (!trajectory_.empty()) {
        last_output_time = trajectory_.back().time_sec;
    }
    if (current_time_sec_ - last_output_time >= config_.output_interval_sec - 0.01 ||
        trajectory_.empty()) {
        trajectory_.push_back(pt);
    }

    if (new_lnav.phase == LnavPhase::COMPLETE || new_vnav.phase == VnavPhase::COMPLETE) {
        complete_ = true;
        if (trajectory_.empty() || trajectory_.back().time_sec != current_time_sec_) {
            trajectory_.push_back(pt);
        }
    }

    advanceTime();
}

bool TrajectorySolver::stepIntegration() {
    if (!initialized_) return false;
    integrateStep();
    return !complete_;
}

bool TrajectorySolver::computeFullTrajectory() {
    if (!initialized_) return false;

    trajectory_.clear();
    current_time_sec_ = 0.0;
    complete_ = false;

    TrajectoryPoint initial_pt = createPoint(lnav_.getState(), vnav_.getState(), 0.0);
    trajectory_.push_back(initial_pt);
    advanceTime();

    int max_iterations = 100000;
    int iterations = 0;
    while (!complete_ && iterations < max_iterations) {
        integrateStep();
        iterations++;
    }

    return complete_ && iterations < max_iterations;
}

TrajectoryPoint TrajectorySolver::getCurrentState() const {
    if (trajectory_.empty()) {
        return TrajectoryPoint();
    }
    return trajectory_.back();
}

double TrajectorySolver::getTotalDistanceNm() const {
    if (trajectory_.empty()) return 0.0;
    return trajectory_.back().distance_along_route_nm;
}

double TrajectorySolver::getTotalTimeSec() const {
    if (trajectory_.empty()) return 0.0;
    return trajectory_.back().time_sec;
}

double TrajectorySolver::getTotalFuelBurnKg() const {
    if (trajectory_.empty()) return 0.0;
    return trajectory_.back().fuel_burn_kg;
}

double TrajectorySolver::getMaxAltitudeFt() const {
    double max_alt = 0.0;
    for (const auto& pt : trajectory_) {
        max_alt = std::max(max_alt, pt.altitude_ft);
    }
    return max_alt;
}

double TrajectorySolver::getAverageGroundSpeedKt() const {
    if (trajectory_.size() < 2) return 0.0;
    double total_gs = 0.0;
    for (const auto& pt : trajectory_) {
        total_gs += pt.ground_speed_kt;
    }
    return total_gs / trajectory_.size();
}

std::vector<TrajectoryPoint> TrajectorySolver::getWaypointProfiles() const {
    std::vector<TrajectoryPoint> profiles;
    int last_wp_idx = -1;
    for (const auto& pt : trajectory_) {
        if (pt.current_wp_index != last_wp_idx) {
            profiles.push_back(pt);
            last_wp_idx = pt.current_wp_index;
        }
    }
    return profiles;
}

TrajectoryPoint TrajectorySolver::getPointAtDistance(double distance_nm) const {
    if (trajectory_.empty()) return TrajectoryPoint();
    for (size_t i = 1; i < trajectory_.size(); i++) {
        if (trajectory_[i].distance_along_route_nm >= distance_nm) {
            double fraction = 0.0;
            double range = trajectory_[i].distance_along_route_nm -
                          trajectory_[i-1].distance_along_route_nm;
            if (range > 0.0) {
                fraction = (distance_nm - trajectory_[i-1].distance_along_route_nm) / range;
            }
            fraction = std::max(0.0, std::min(1.0, fraction));
            return trajectory_[i-1];
        }
    }
    return trajectory_.back();
}

TrajectoryPoint TrajectorySolver::getPointAtTime(double time_sec) const {
    if (trajectory_.empty()) return TrajectoryPoint();
    for (size_t i = 1; i < trajectory_.size(); i++) {
        if (trajectory_[i].time_sec >= time_sec) {
            return trajectory_[i-1];
        }
    }
    return trajectory_.back();
}

}
