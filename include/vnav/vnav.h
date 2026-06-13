#pragma once

#include <vector>
#include <string>
#include "isa.h"
#include "arinc424.h"
#include "lnav.h"

namespace vnav {

enum class VnavPhase {
    GROUND = 0,
    CLIMB = 1,
    CRUISE = 2,
    DESCENT = 3,
    APPROACH = 4,
    GO_AROUND = 5,
    COMPLETE = 6
};

enum class ClimbMode {
    CAS_CLIMB = 0,
    MACH_CLIMB = 1,
    ALTITUDE_CAPTURE = 2
};

enum class DescentMode {
    MACH_DESCENT = 0,
    CAS_DESCENT = 1,
    ALTITUDE_CAPTURE = 2,
    PATH_CAPTURE = 3
};

struct VnavConstraint {
    double altitude_ft;
    double speed_kt;
    AltitudeConstraintType alt_type;
    int waypoint_index;
    double distance_from_start_nm;
};

struct VnavWaypointProfile {
    double altitude_ft;
    double vertical_speed_fpm;
    double mach_number;
    double cas_kt;
    double tas_ms;
    double pitch_deg;
    double distance_from_start_nm;
    int waypoint_index;
    VnavPhase phase;
};

struct VnavState {
    double altitude_ft;
    double vertical_speed_fpm;
    double mach_number;
    double cas_kt;
    double tas_ms;
    double pitch_deg;
    VnavPhase phase;
    ClimbMode climb_mode;
    DescentMode descent_mode;
    double target_altitude_ft;
    double target_mach;
    double target_cas_kt;
    int active_constraint_index;
    double distance_to_top_of_climb_nm;
    double distance_to_top_of_descent_nm;
    double flight_path_angle_deg;
    double distance_along_route_nm;
    bool altitude_captured;
    bool constraint_met;
};

struct VnavConfig {
    double climb_cas_kt;
    double climb_mach;
    double cruise_mach;
    double descent_cas_kt;
    double descent_mach;
    double cruise_altitude_ft;
    double initial_climb_rate_fpm;
    double standard_descent_rate_fpm;
    double approach_speed_kt;
    double transition_altitude_ft;
    double transition_level_ft;
    double crossover_altitude_ft;

    VnavConfig()
        : climb_cas_kt(280.0),
          climb_mach(0.78),
          cruise_mach(0.78),
          descent_cas_kt(300.0),
          descent_mach(0.78),
          cruise_altitude_ft(35000.0),
          initial_climb_rate_fpm(3000.0),
          standard_descent_rate_fpm(2500.0),
          approach_speed_kt(140.0),
          transition_altitude_ft(18000.0),
          transition_level_ft(18000.0),
          crossover_altitude_ft(31000.0) {}
};

class VnavStateMachine {
public:
    VnavStateMachine();
    explicit VnavStateMachine(const VnavConfig& config);

    void configure(const VnavConfig& config);
    void setConstraints(const std::vector<VnavConstraint>& constraints);
    void autoGenerateConstraints(const LnavStateMachine& lnav);
    void computeVerticalProfile(const LnavStateMachine& lnav);
    void initialize(double initial_alt_ft, double initial_cas_kt);

    VnavState update(double dt_sec, double ground_speed_ms, double distance_along_nm);

    const VnavState& getState() const { return state_; }
    const VnavConfig& getConfig() const { return config_; }
    const std::vector<VnavConstraint>& getConstraints() const { return constraints_; }
    const std::vector<VnavWaypointProfile>& getProfile() const { return profile_; }

    VnavPhase getPhase() const { return state_.phase; }
    double getAltitudeFt() const { return state_.altitude_ft; }
    double getVerticalSpeedFpm() const { return state_.vertical_speed_fpm; }
    double getMach() const { return state_.mach_number; }
    double getCasKt() const { return state_.cas_kt; }

    void setCruiseAltitude(double alt_ft);
    void setPhase(VnavPhase phase);

    double computeRequiredVsForPath(double current_alt_ft, double target_alt_ft,
                                     double distance_nm, double ground_speed_kt) const;

    double computeTopOfDescentDistanceNm(double current_alt_ft, double target_alt_ft,
                                          double descent_rate_fpm, double ground_speed_kt) const;

    double computeFlightPathAngle(double vs_fpm, double gs_kt) const;

    static double feetToMeters(double ft);
    static double metersToFeet(double m);
    static double knotsToMetersPerSec(double kt);
    static double metersPerSecToKnots(double ms);
    static double fpmToMps(double fpm);
    static double mpsToFpm(double mps);
    static double nmToMeters(double nm);
    static double metersToNm(double m);

private:
    VnavState state_;
    VnavConfig config_;
    std::vector<VnavConstraint> constraints_;
    std::vector<VnavWaypointProfile> profile_;

    void updateClimb(double dt_sec, double ground_speed_ms, double distance_along_nm);
    void updateCruise(double dt_sec, double ground_speed_ms, double distance_along_nm);
    void updateDescent(double dt_sec, double ground_speed_ms, double distance_along_nm);
    void updateApproach(double dt_sec, double ground_speed_ms, double distance_along_nm);

    void updateSpeeds(double altitude_ft, double& mach, double& cas_kt, double& tas_ms);
    VnavConstraint findNextConstraint(double distance_along_nm);
    void checkConstraintCapture(double distance_along_nm, double altitude_ft);
};

}
