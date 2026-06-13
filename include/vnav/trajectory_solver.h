#pragma once

#include <vector>
#include <string>
#include "isa.h"
#include "great_circle.h"
#include "arinc424.h"
#include "lnav.h"
#include "vnav.h"

namespace vnav {

struct TrajectoryPoint {
    double time_sec;
    GeoPoint position;
    double altitude_ft;
    double altitude_m;
    double ground_track_deg;
    double ground_speed_ms;
    double ground_speed_kt;
    double tas_ms;
    double tas_kt;
    double cas_kt;
    double mach_number;
    double vertical_speed_fpm;
    double vertical_speed_mps;
    double pitch_deg;
    double roll_deg;
    double heading_deg;
    double true_air_temp_k;
    double total_temp_k;
    double static_pressure_pa;
    double density_kgm3;
    double speed_of_sound_ms;
    double flight_path_angle_deg;
    double distance_along_route_nm;
    double distance_to_next_wp_nm;
    double cross_track_error_nm;
    int current_wp_index;
    LnavPhase lnav_phase;
    VnavPhase vnav_phase;
    double fuel_remaining_kg;
    double fuel_burn_kg;
};

struct AircraftPerformance {
    double gross_weight_kg;
    double zero_fuel_weight_kg;
    double fuel_capacity_kg;
    double cruise_fuel_flow_kg_per_hr;
    double climb_fuel_flow_kg_per_hr;
    double descent_fuel_flow_kg_per_hr;
    double max_climb_rate_fpm;
    double service_ceiling_ft;
    double wing_area_m2;

    AircraftPerformance()
        : gross_weight_kg(75000.0),
          zero_fuel_weight_kg(60000.0),
          fuel_capacity_kg(20000.0),
          cruise_fuel_flow_kg_per_hr(2500.0),
          climb_fuel_flow_kg_per_hr(4000.0),
          descent_fuel_flow_kg_per_hr(1200.0),
          max_climb_rate_fpm(3500.0),
          service_ceiling_ft(41000.0),
          wing_area_m2(125.0) {}
};

struct TrajectoryConfig {
    double integration_dt_sec;
    double output_interval_sec;
    double wind_direction_deg;
    double wind_speed_ms;
    double isa_deviation_c;
    bool include_wind;
    bool enable_fuel_calc;

    TrajectoryConfig()
        : integration_dt_sec(0.5),
          output_interval_sec(1.0),
          wind_direction_deg(0.0),
          wind_speed_ms(0.0),
          isa_deviation_c(0.0),
          include_wind(false),
          enable_fuel_calc(true) {}
};

class TrajectorySolver {
public:
    TrajectorySolver();
    explicit TrajectorySolver(const TrajectoryConfig& config);

    void setConfig(const TrajectoryConfig& config);
    void setAircraft(const AircraftPerformance& ac);
    void setLnavConfig(const LnavConfig& config);
    void setVnavConfig(const VnavConfig& config);

    void loadFlightPlan(const std::vector<Waypoint>& waypoints);
    void loadFlightPlanFromArinc(const NavigationDatabase& db,
                                  const std::string& origin,
                                  const std::string& sid_name,
                                  const std::string& runway_dep,
                                  const std::vector<std::string>& enroute_wps,
                                  const std::string& destination,
                                  const std::string& star_name,
                                  const std::string& runway_arr);

    void setInitialConditions(const GeoPoint& start_pos,
                               double initial_alt_ft,
                               double initial_cas_kt,
                               double initial_track_deg,
                               double initial_fuel_kg);

    bool computeFullTrajectory();
    bool stepIntegration();

    const std::vector<TrajectoryPoint>& getTrajectory() const { return trajectory_; }
    TrajectoryPoint getCurrentState() const;
    bool isComplete() const { return complete_; }

    const LnavStateMachine& getLnav() const { return lnav_; }
    const VnavStateMachine& getVnav() const { return vnav_; }
    const TrajectoryConfig& getConfig() const { return config_; }
    const AircraftPerformance& getAircraft() const { return aircraft_; }

    double getTotalDistanceNm() const;
    double getTotalTimeSec() const;
    double getTotalFuelBurnKg() const;
    double getMaxAltitudeFt() const;
    double getAverageGroundSpeedKt() const;

    std::vector<TrajectoryPoint> getWaypointProfiles() const;
    TrajectoryPoint getPointAtDistance(double distance_nm) const;
    TrajectoryPoint getPointAtTime(double time_sec) const;

private:
    TrajectoryConfig config_;
    AircraftPerformance aircraft_;
    LnavStateMachine lnav_;
    VnavStateMachine vnav_;
    std::vector<TrajectoryPoint> trajectory_;

    double current_time_sec_;
    double current_fuel_kg_;
    double distance_along_route_nm_;
    double cumulative_fuel_burn_;
    bool initialized_;
    bool complete_;

    TrajectoryPoint createPoint(const LnavState& lnav_state,
                                 const VnavState& vnav_state,
                                 double time_sec) const;

    double computeWindCorrectedTrack(double track_deg, double tas_ms) const;
    double computeGroundSpeed(double track_deg, double tas_ms, double heading_deg) const;
    double computeFuelFlow(VnavPhase phase, double altitude_ft, double mach) const;
    GeoPoint applyWindToPosition(const GeoPoint& pos, double dt_sec,
                                  double heading_deg, double tas_ms) const;

    void advanceTime();
    void integrateStep();
};

}
