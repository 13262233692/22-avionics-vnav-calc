#include "vnav/lnav.h"
#include <cmath>
#include <algorithm>

namespace vnav {

LnavStateMachine::LnavStateMachine() {
    state_.current_waypoint_index = 0;
    state_.next_waypoint_index = 1;
    state_.phase = LnavPhase::LEG_TRACKING;
    state_.waypoint_captured = false;
    state_.cross_track_error_nm = 0.0;
    state_.distance_to_next_nm = 0.0;
    state_.distance_along_leg_nm = 0.0;
    state_.course_deviation_deg = 0.0;
    state_.ground_speed_ms = 0.0;
    state_.ground_track_deg = 0.0;
    state_.turn_angle_deg = 0.0;
    state_.turn_rate_deg_per_sec = 0.0;
    state_.time_to_turn_sec = 0.0;
}

LnavStateMachine::LnavStateMachine(const LnavConfig& config) : config_(config) {
    LnavStateMachine();
}

void LnavStateMachine::setFlightPlan(const std::vector<Waypoint>& waypoints) {
    waypoints_ = waypoints;
    state_.current_waypoint_index = 0;
    state_.next_waypoint_index = waypoints_.size() > 1 ? 1 : 0;
    state_.phase = LnavPhase::LEG_TRACKING;
    state_.waypoint_captured = false;
}

void LnavStateMachine::initialize(const GeoPoint& start_pos, double start_track_deg, double ground_speed_ms) {
    state_.position = start_pos;
    state_.ground_track_deg = start_track_deg;
    state_.ground_speed_ms = ground_speed_ms;
    state_.phase = LnavPhase::LEG_TRACKING;
    state_.current_waypoint_index = 0;
    state_.next_waypoint_index = waypoints_.size() > 1 ? 1 : 0;

    if (!waypoints_.empty() && state_.next_waypoint_index < (int)waypoints_.size()) {
        const auto& wp = waypoints_[state_.next_waypoint_index];
        state_.distance_to_next_nm = GreatCircle::distanceNm(start_pos, wp.position);
        state_.ground_track_deg = GreatCircle::initialBearingDeg(start_pos, wp.position);
    }
}

double LnavStateMachine::getTurnRadiusM(double ground_speed_ms) const {
    double rate_rad = config_.standard_turn_rate_deg_per_sec * M_PI / 180.0;
    return ground_speed_ms / rate_rad;
}

double LnavStateMachine::getTurnRadiusNm(double ground_speed_ms) const {
    return getTurnRadiusM(ground_speed_ms) / GreatCircle::NM_TO_M;
}

double LnavStateMachine::computeTurnAnticipationDistance(double ground_speed_ms, double turn_angle_deg) const {
    double radius_nm = getTurnRadiusNm(ground_speed_ms);
    double half_angle = turn_angle_deg / 2.0 * M_PI / 180.0;
    return radius_nm * std::tan(half_angle) + config_.turn_anticipation_nm;
}

double LnavStateMachine::computeRequiredTurnRate() const {
    if (state_.current_waypoint_index >= (int)waypoints_.size() ||
        state_.next_waypoint_index >= (int)waypoints_.size()) {
        return 0.0;
    }
    if (state_.current_waypoint_index + 1 >= (int)waypoints_.size()) return 0.0;

    const auto& curr_wp = waypoints_[state_.current_waypoint_index];
    const auto& next_wp = waypoints_[state_.next_waypoint_index];
    int next_next_idx = state_.next_waypoint_index + 1;
    if (next_next_idx >= (int)waypoints_.size()) return 0.0;

    const auto& next_next_wp = waypoints_[next_next_idx];
    double bear_in = GreatCircle::initialBearingDeg(curr_wp.position, next_wp.position);
    double bear_out = GreatCircle::initialBearingDeg(next_wp.position, next_next_wp.position);
    double turn_angle = std::fmod(bear_out - bear_in + 540.0, 360.0) - 180.0;
    return turn_angle;
}

Waypoint LnavStateMachine::getCurrentWaypoint() const {
    if (state_.current_waypoint_index < (int)waypoints_.size()) {
        return waypoints_[state_.current_waypoint_index];
    }
    return Waypoint();
}

Waypoint LnavStateMachine::getNextWaypoint() const {
    if (state_.next_waypoint_index < (int)waypoints_.size()) {
        return waypoints_[state_.next_waypoint_index];
    }
    return Waypoint();
}

double LnavStateMachine::getTotalDistanceNm() const {
    double total = 0.0;
    for (size_t i = 1; i < waypoints_.size(); i++) {
        total += GreatCircle::distanceNm(waypoints_[i-1].position, waypoints_[i].position);
    }
    return total;
}

double LnavStateMachine::getRemainingDistanceNm() const {
    if (state_.next_waypoint_index >= (int)waypoints_.size()) return 0.0;
    double remaining = state_.distance_to_next_nm;
    for (size_t i = state_.next_waypoint_index + 1; i < waypoints_.size(); i++) {
        remaining += GreatCircle::distanceNm(waypoints_[i-1].position, waypoints_[i].position);
    }
    return remaining;
}

double LnavStateMachine::getDistanceToWaypoint(int index) const {
    if (index < 0 || index >= (int)waypoints_.size()) return -1.0;
    return GreatCircle::distanceNm(state_.position, waypoints_[index].position);
}

void LnavStateMachine::advanceToNextWaypoint() {
    state_.current_waypoint_index = state_.next_waypoint_index;
    state_.next_waypoint_index++;
    state_.waypoint_captured = true;

    if (state_.next_waypoint_index >= (int)waypoints_.size()) {
        state_.phase = LnavPhase::COMPLETE;
        state_.distance_to_next_nm = 0.0;
    } else {
        state_.phase = LnavPhase::LEG_TRACKING;
    }
}

GeoPoint LnavStateMachine::predictPosition(double dt_sec, double ground_speed_ms) const {
    double distance_m = ground_speed_ms * dt_sec;
    return GreatCircle::pointAtDistanceBearing(state_.position, distance_m, state_.ground_track_deg);
}

void LnavStateMachine::updateLegTracking(double dt_sec, double ground_speed_ms) {
    if (state_.next_waypoint_index >= (int)waypoints_.size()) {
        state_.phase = LnavPhase::COMPLETE;
        return;
    }

    const auto& curr_wp = waypoints_[state_.current_waypoint_index];
    const auto& next_wp = waypoints_[state_.next_waypoint_index];

    double desired_track = GreatCircle::initialBearingDeg(state_.position, next_wp.position);
    state_.ground_track_deg = desired_track;
    state_.ground_speed_ms = ground_speed_ms;

    double distance_m = ground_speed_ms * dt_sec;
    state_.position = GreatCircle::pointAtDistanceBearing(state_.position, distance_m, desired_track);

    state_.distance_to_next_nm = GreatCircle::distanceNm(state_.position, next_wp.position);
    state_.distance_along_leg_nm = GreatCircle::distanceNm(curr_wp.position, state_.position);
    state_.cross_track_error_nm = GreatCircle::crossTrackDistanceM(
        curr_wp.position, next_wp.position, state_.position) / GreatCircle::NM_TO_M;

    double leg_distance = GreatCircle::distanceNm(curr_wp.position, next_wp.position);
    state_.course_deviation_deg = state_.cross_track_error_nm > 0 ? 90.0 : -90.0;

    double turn_angle = computeRequiredTurnRate();
    state_.turn_angle_deg = turn_angle;
    double turn_anticipation = computeTurnAnticipationDistance(ground_speed_ms, std::abs(turn_angle));

    if (state_.distance_to_next_nm <= config_.capture_distance_nm) {
        advanceToNextWaypoint();
    } else if (state_.distance_to_next_nm <= turn_anticipation && std::abs(turn_angle) > 5.0) {
        state_.phase = LnavPhase::TURNING;
        state_.turn_rate_deg_per_sec = turn_angle > 0 ? config_.standard_turn_rate_deg_per_sec
                                                      : -config_.standard_turn_rate_deg_per_sec;
        state_.time_to_turn_sec = std::abs(turn_angle) / config_.standard_turn_rate_deg_per_sec;
    }
}

void LnavStateMachine::updateTurning(double dt_sec, double ground_speed_ms) {
    if (state_.next_waypoint_index >= (int)waypoints_.size()) {
        state_.phase = LnavPhase::COMPLETE;
        return;
    }

    const auto& curr_wp = waypoints_[state_.current_waypoint_index];
    const auto& next_wp = waypoints_[state_.next_waypoint_index];

    state_.ground_track_deg += state_.turn_rate_deg_per_sec * dt_sec;
    state_.ground_track_deg = std::fmod(state_.ground_track_deg + 360.0, 360.0);
    state_.ground_speed_ms = ground_speed_ms;

    double distance_m = ground_speed_ms * dt_sec;
    state_.position = GreatCircle::pointAtDistanceBearing(state_.position, distance_m, state_.ground_track_deg);

    state_.distance_to_next_nm = GreatCircle::distanceNm(state_.position, next_wp.position);
    state_.cross_track_error_nm = GreatCircle::crossTrackDistanceM(
        curr_wp.position, next_wp.position, state_.position) / GreatCircle::NM_TO_M;

    state_.time_to_turn_sec -= dt_sec;

    if (state_.time_to_turn_sec <= 0.0 || state_.distance_to_next_nm <= config_.capture_distance_nm) {
        advanceToNextWaypoint();
    }
}

void LnavStateMachine::updateApproachingWaypoint(double dt_sec, double ground_speed_ms) {
    updateLegTracking(dt_sec, ground_speed_ms);
}

LnavState LnavStateMachine::update(double dt_sec, double ground_speed_ms) {
    switch (state_.phase) {
        case LnavPhase::LEG_TRACKING:
            updateLegTracking(dt_sec, ground_speed_ms);
            break;
        case LnavPhase::TURNING:
            updateTurning(dt_sec, ground_speed_ms);
            break;
        case LnavPhase::APPROACHING_WAYPOINT:
            updateApproachingWaypoint(dt_sec, ground_speed_ms);
            break;
        case LnavPhase::COMPLETE:
        case LnavPhase::HOLDING:
        default:
            break;
    }
    return state_;
}

LnavState LnavStateMachine::updatePosition(const GeoPoint& position, double ground_track_deg, double ground_speed_ms) {
    state_.position = position;
    state_.ground_track_deg = ground_track_deg;
    state_.ground_speed_ms = ground_speed_ms;

    if (state_.next_waypoint_index < (int)waypoints_.size()) {
        const auto& curr_wp = waypoints_[state_.current_waypoint_index];
        const auto& next_wp = waypoints_[state_.next_waypoint_index];
        state_.distance_to_next_nm = GreatCircle::distanceNm(position, next_wp.position);
        state_.distance_along_leg_nm = GreatCircle::distanceNm(curr_wp.position, position);
        state_.cross_track_error_nm = GreatCircle::crossTrackDistanceM(
            curr_wp.position, next_wp.position, position) / GreatCircle::NM_TO_M;
    }

    if (state_.next_waypoint_index < (int)waypoints_.size() &&
        state_.distance_to_next_nm <= config_.capture_distance_nm) {
        advanceToNextWaypoint();
    }

    return state_;
}

}
