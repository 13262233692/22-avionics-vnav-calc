#pragma once

#include <vector>
#include <string>
#include "great_circle.h"
#include "arinc424.h"

namespace vnav {

enum class LnavPhase {
    APPROACHING_WAYPOINT = 0,
    TURNING = 1,
    LEG_TRACKING = 2,
    HOLDING = 3,
    COMPLETE = 4
};

struct LnavState {
    GeoPoint position;
    double ground_track_deg;
    double ground_speed_ms;
    double distance_to_next_nm;
    double distance_along_leg_nm;
    double cross_track_error_nm;
    double course_deviation_deg;
    int current_waypoint_index;
    int next_waypoint_index;
    LnavPhase phase;
    double turn_angle_deg;
    double turn_rate_deg_per_sec;
    double time_to_turn_sec;
    bool waypoint_captured;
};

struct LnavConfig {
    double standard_turn_rate_deg_per_sec;
    double turn_bank_angle_deg;
    double capture_distance_nm;
    double turn_anticipation_nm;
    double max_cross_track_error_nm;

    LnavConfig()
        : standard_turn_rate_deg_per_sec(3.0),
          turn_bank_angle_deg(25.0),
          capture_distance_nm(1.0),
          turn_anticipation_nm(2.0),
          max_cross_track_error_nm(5.0) {}
};

class LnavStateMachine {
public:
    LnavStateMachine();
    explicit LnavStateMachine(const LnavConfig& config);

    void setFlightPlan(const std::vector<Waypoint>& waypoints);
    void initialize(const GeoPoint& start_pos, double start_track_deg, double ground_speed_ms);

    LnavState update(double dt_sec, double current_ground_speed_ms);
    LnavState updatePosition(const GeoPoint& position, double ground_track_deg, double ground_speed_ms);

    const std::vector<Waypoint>& getWaypoints() const { return waypoints_; }
    const LnavState& getState() const { return state_; }
    LnavConfig& config() { return config_; }

    double getTotalDistanceNm() const;
    double getRemainingDistanceNm() const;
    double getDistanceToWaypoint(int index) const;
    double getTurnRadiusM(double ground_speed_ms) const;
    double getTurnRadiusNm(double ground_speed_ms) const;

    Waypoint getCurrentWaypoint() const;
    Waypoint getNextWaypoint() const;

    bool isComplete() const { return state_.phase == LnavPhase::COMPLETE; }
    int getActiveWaypointIndex() const { return state_.current_waypoint_index; }
    int getNextWaypointIndex() const { return state_.next_waypoint_index; }

    GeoPoint predictPosition(double dt_sec, double ground_speed_ms) const;

private:
    std::vector<Waypoint> waypoints_;
    LnavState state_;
    LnavConfig config_;

    void updateLegTracking(double dt_sec, double ground_speed_ms);
    void updateTurning(double dt_sec, double ground_speed_ms);
    void updateApproachingWaypoint(double dt_sec, double ground_speed_ms);
    void advanceToNextWaypoint();
    double computeTurnAnticipationDistance(double ground_speed_ms, double turn_angle_deg) const;
    double computeRequiredTurnRate() const;
};

}
