#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "vnav/isa.h"
#include "vnav/great_circle.h"
#include "vnav/arinc424.h"
#include "vnav/lnav.h"
#include "vnav/vnav.h"
#include "vnav/trajectory_solver.h"

namespace py = pybind11;
using namespace vnav;

PYBIND11_MODULE(vnav_core, m) {
    m.doc() = "VNAV Vertical Navigation Calculator - ARINC 424 compliant 3D trajectory solver";

    py::class_<ISAData>(m, "ISAData")
        .def(py::init<>())
        .def_readwrite("temperature", &ISAData::temperature)
        .def_readwrite("pressure", &ISAData::pressure)
        .def_readwrite("density", &ISAData::density)
        .def_readwrite("speed_of_sound", &ISAData::speed_of_sound);

    py::class_<InternationalStandardAtmosphere>(m, "InternationalStandardAtmosphere")
        .def(py::init<>())
        .def_static("compute", &InternationalStandardAtmosphere::compute)
        .def_static("pressure_altitude", &InternationalStandardAtmosphere::pressureAltitude)
        .def_static("mach_to_tas", &InternationalStandardAtmosphere::machToTas)
        .def_static("tas_to_mach", &InternationalStandardAtmosphere::tasToMach)
        .def_static("cas_to_tas", &InternationalStandardAtmosphere::casToTas)
        .def_readonly_static("SEA_LEVEL_TEMP", &InternationalStandardAtmosphere::SEA_LEVEL_TEMP)
        .def_readonly_static("SEA_LEVEL_PRESSURE", &InternationalStandardAtmosphere::SEA_LEVEL_PRESSURE)
        .def_readonly_static("TROPOPAUSE_ALT", &InternationalStandardAtmosphere::TROPOPAUSE_ALT);

    py::class_<GeoPoint>(m, "GeoPoint")
        .def(py::init<>())
        .def(py::init<double, double>())
        .def_readwrite("latitude_deg", &GeoPoint::latitude_deg)
        .def_readwrite("longitude_deg", &GeoPoint::longitude_deg)
        .def("__repr__", [](const GeoPoint& p) {
            return "GeoPoint(lat=" + std::to_string(p.latitude_deg) + ", lon=" + std::to_string(p.longitude_deg) + ")";
        });

    py::class_<GreatCircle>(m, "GreatCircle")
        .def(py::init<>())
        .def_static("distance_meters", &GreatCircle::distanceMeters)
        .def_static("distance_nm", &GreatCircle::distanceNm)
        .def_static("initial_bearing_deg", &GreatCircle::initialBearingDeg)
        .def_static("final_bearing_deg", &GreatCircle::finalBearingDeg)
        .def_static("intermediate_point", &GreatCircle::intermediatePoint)
        .def_static("point_at_distance_bearing", &GreatCircle::pointAtDistanceBearing)
        .def_static("cross_track_distance_m", &GreatCircle::crossTrackDistanceM)
        .def_static("along_track_distance_m", &GreatCircle::alongTrackDistanceM)
        .def_readonly_static("EARTH_RADIUS_M", &GreatCircle::EARTH_RADIUS_M)
        .def_readonly_static("NM_TO_M", &GreatCircle::NM_TO_M);

    py::enum_<AltitudeConstraintType>(m, "AltitudeConstraintType")
        .value("ALT_NONE", AltitudeConstraintType::ALT_NONE)
        .value("ALT_AT", AltitudeConstraintType::ALT_AT)
        .value("ALT_AT_OR_ABOVE", AltitudeConstraintType::ALT_AT_OR_ABOVE)
        .value("ALT_AT_OR_BELOW", AltitudeConstraintType::ALT_AT_OR_BELOW)
        .value("ALT_BETWEEN", AltitudeConstraintType::ALT_BETWEEN)
        .value("ALT_GLIDE_SLOPE", AltitudeConstraintType::ALT_GLIDE_SLOPE);

    py::class_<AltitudeConstraint>(m, "AltitudeConstraint")
        .def(py::init<>())
        .def_readwrite("type", &AltitudeConstraint::type)
        .def_readwrite("altitude1_ft", &AltitudeConstraint::altitude1_ft)
        .def_readwrite("altitude2_ft", &AltitudeConstraint::altitude2_ft)
        .def_readwrite("is_flight_level", &AltitudeConstraint::is_flight_level);

    py::enum_<WaypointType>(m, "WaypointType")
        .value("AIRPORT", WaypointType::WP_AIRPORT)
        .value("NDB", WaypointType::WP_NDB)
        .value("VOR", WaypointType::WP_VOR)
        .value("DME", WaypointType::WP_DME)
        .value("FIX", WaypointType::WP_FIX)
        .value("RUNWAY", WaypointType::WP_RUNWAY)
        .value("USER", WaypointType::WP_USER);

    py::enum_<SpeedLimit>(m, "SpeedLimit")
        .value("NONE", SpeedLimit::NONE)
        .value("AT_OR_BELOW", SpeedLimit::AT_OR_BELOW)
        .value("AT_OR_ABOVE", SpeedLimit::AT_OR_ABOVE)
        .value("AT", SpeedLimit::AT);

    py::class_<Waypoint>(m, "Waypoint")
        .def(py::init<>())
        .def_readwrite("identifier", &Waypoint::identifier)
        .def_readwrite("region_code", &Waypoint::region_code)
        .def_readwrite("icao_code", &Waypoint::icao_code)
        .def_readwrite("type", &Waypoint::type)
        .def_readwrite("position", &Waypoint::position)
        .def_readwrite("magnetic_variation", &Waypoint::magnetic_variation)
        .def_readwrite("altitude_constraint", &Waypoint::altitude_constraint)
        .def_readwrite("speed_limit_kt", &Waypoint::speed_limit_kt)
        .def_readwrite("speed_limit_type", &Waypoint::speed_limit_type)
        .def_readwrite("turn_radius_nm", &Waypoint::turn_radius_nm)
        .def_readwrite("is_fly_by", &Waypoint::is_fly_by)
        .def_readwrite("is_overfly", &Waypoint::is_overfly)
        .def_readwrite("description", &Waypoint::description);

    py::enum_<ProcedureType>(m, "ProcedureType")
        .value("SID", ProcedureType::PROC_SID)
        .value("STAR", ProcedureType::PROC_STAR)
        .value("APPROACH", ProcedureType::PROC_APPROACH)
        .value("ENROUTE", ProcedureType::PROC_ENROUTE);

    py::class_<ProcedureLeg>(m, "ProcedureLeg")
        .def(py::init<>())
        .def_readwrite("waypoint_id", &ProcedureLeg::waypoint_id)
        .def_readwrite("recommended_navaid", &ProcedureLeg::recommended_navaid)
        .def_readwrite("arc_radius_nm", &ProcedureLeg::arc_radius_nm)
        .def_readwrite("magnetic_course_deg", &ProcedureLeg::magnetic_course_deg)
        .def_readwrite("distance_nm", &ProcedureLeg::distance_nm)
        .def_readwrite("alt_constraint", &ProcedureLeg::alt_constraint)
        .def_readwrite("speed_limit_kt", &ProcedureLeg::speed_limit_kt)
        .def_readwrite("speed_limit_type", &ProcedureLeg::speed_limit_type)
        .def_readwrite("is_mandatory", &ProcedureLeg::is_mandatory)
        .def_readwrite("leg_sequence_number", &ProcedureLeg::leg_sequence_number);

    py::class_<Procedure>(m, "Procedure")
        .def(py::init<>())
        .def_readwrite("name", &Procedure::name)
        .def_readwrite("type", &Procedure::type)
        .def_readwrite("airport_icao", &Procedure::airport_icao)
        .def_readwrite("runway", &Procedure::runway)
        .def_readwrite("transition", &Procedure::transition)
        .def_readwrite("legs", &Procedure::legs);

    py::class_<Airport>(m, "Airport")
        .def(py::init<>())
        .def_readwrite("icao_code", &Airport::icao_code)
        .def_readwrite("name", &Airport::name)
        .def_readwrite("position", &Airport::position)
        .def_readwrite("elevation_ft", &Airport::elevation_ft)
        .def_readwrite("transition_altitude_ft", &Airport::transition_altitude_ft)
        .def_readwrite("runways", &Airport::runways)
        .def_readwrite("sids", &Airport::sids)
        .def_readwrite("stars", &Airport::stars)
        .def_readwrite("approaches", &Airport::approaches);

    py::class_<NavigationDatabase>(m, "NavigationDatabase")
        .def(py::init<>())
        .def_readwrite("airports", &NavigationDatabase::airports)
        .def_readwrite("waypoints", &NavigationDatabase::waypoints)
        .def_readwrite("ndbs", &NavigationDatabase::ndbs)
        .def_readwrite("vors", &NavigationDatabase::vors);

    py::class_<Arinc424Parser>(m, "Arinc424Parser")
        .def(py::init<>())
        .def("parse_file", &Arinc424Parser::parseFile)
        .def("parse_string", &Arinc424Parser::parseString)
        .def("get_parse_errors", &Arinc424Parser::getParseErrors);

    py::enum_<LnavPhase>(m, "LnavPhase")
        .value("APPROACHING_WAYPOINT", LnavPhase::APPROACHING_WAYPOINT)
        .value("TURNING", LnavPhase::TURNING)
        .value("LEG_TRACKING", LnavPhase::LEG_TRACKING)
        .value("HOLDING", LnavPhase::HOLDING)
        .value("COMPLETE", LnavPhase::COMPLETE);

    py::class_<LnavState>(m, "LnavState")
        .def(py::init<>())
        .def_readwrite("position", &LnavState::position)
        .def_readwrite("ground_track_deg", &LnavState::ground_track_deg)
        .def_readwrite("ground_speed_ms", &LnavState::ground_speed_ms)
        .def_readwrite("distance_to_next_nm", &LnavState::distance_to_next_nm)
        .def_readwrite("distance_along_leg_nm", &LnavState::distance_along_leg_nm)
        .def_readwrite("cross_track_error_nm", &LnavState::cross_track_error_nm)
        .def_readwrite("course_deviation_deg", &LnavState::course_deviation_deg)
        .def_readwrite("current_waypoint_index", &LnavState::current_waypoint_index)
        .def_readwrite("next_waypoint_index", &LnavState::next_waypoint_index)
        .def_readwrite("phase", &LnavState::phase)
        .def_readwrite("turn_angle_deg", &LnavState::turn_angle_deg)
        .def_readwrite("turn_rate_deg_per_sec", &LnavState::turn_rate_deg_per_sec)
        .def_readwrite("time_to_turn_sec", &LnavState::time_to_turn_sec)
        .def_readwrite("waypoint_captured", &LnavState::waypoint_captured);

    py::class_<LnavConfig>(m, "LnavConfig")
        .def(py::init<>())
        .def_readwrite("standard_turn_rate_deg_per_sec", &LnavConfig::standard_turn_rate_deg_per_sec)
        .def_readwrite("turn_bank_angle_deg", &LnavConfig::turn_bank_angle_deg)
        .def_readwrite("capture_distance_nm", &LnavConfig::capture_distance_nm)
        .def_readwrite("turn_anticipation_nm", &LnavConfig::turn_anticipation_nm)
        .def_readwrite("max_cross_track_error_nm", &LnavConfig::max_cross_track_error_nm);

    py::class_<LnavStateMachine>(m, "LnavStateMachine")
        .def(py::init<>())
        .def(py::init<const LnavConfig&>())
        .def("set_flight_plan", &LnavStateMachine::setFlightPlan)
        .def("initialize", &LnavStateMachine::initialize)
        .def("update", &LnavStateMachine::update)
        .def("update_position", &LnavStateMachine::updatePosition)
        .def("get_waypoints", &LnavStateMachine::getWaypoints, py::return_value_policy::reference_internal)
        .def("get_state", &LnavStateMachine::getState, py::return_value_policy::reference_internal)
        .def("config", &LnavStateMachine::config, py::return_value_policy::reference_internal)
        .def("get_total_distance_nm", &LnavStateMachine::getTotalDistanceNm)
        .def("get_remaining_distance_nm", &LnavStateMachine::getRemainingDistanceNm)
        .def("get_distance_to_waypoint", &LnavStateMachine::getDistanceToWaypoint)
        .def("get_turn_radius_m", &LnavStateMachine::getTurnRadiusM)
        .def("get_turn_radius_nm", &LnavStateMachine::getTurnRadiusNm)
        .def("get_current_waypoint", &LnavStateMachine::getCurrentWaypoint)
        .def("get_next_waypoint", &LnavStateMachine::getNextWaypoint)
        .def("is_complete", &LnavStateMachine::isComplete)
        .def("predict_position", &LnavStateMachine::predictPosition);

    py::enum_<VnavPhase>(m, "VnavPhase")
        .value("GROUND", VnavPhase::GROUND)
        .value("CLIMB", VnavPhase::CLIMB)
        .value("CRUISE", VnavPhase::CRUISE)
        .value("DESCENT", VnavPhase::DESCENT)
        .value("APPROACH", VnavPhase::APPROACH)
        .value("GO_AROUND", VnavPhase::GO_AROUND)
        .value("COMPLETE", VnavPhase::COMPLETE);

    py::enum_<ClimbMode>(m, "ClimbMode")
        .value("CAS_CLIMB", ClimbMode::CAS_CLIMB)
        .value("MACH_CLIMB", ClimbMode::MACH_CLIMB)
        .value("ALTITUDE_CAPTURE", ClimbMode::ALTITUDE_CAPTURE);

    py::enum_<DescentMode>(m, "DescentMode")
        .value("MACH_DESCENT", DescentMode::MACH_DESCENT)
        .value("CAS_DESCENT", DescentMode::CAS_DESCENT)
        .value("ALTITUDE_CAPTURE", DescentMode::ALTITUDE_CAPTURE)
        .value("PATH_CAPTURE", DescentMode::PATH_CAPTURE);

    py::class_<VnavConstraint>(m, "VnavConstraint")
        .def(py::init<>())
        .def_readwrite("altitude_ft", &VnavConstraint::altitude_ft)
        .def_readwrite("speed_kt", &VnavConstraint::speed_kt)
        .def_readwrite("alt_type", &VnavConstraint::alt_type)
        .def_readwrite("waypoint_index", &VnavConstraint::waypoint_index)
        .def_readwrite("distance_from_start_nm", &VnavConstraint::distance_from_start_nm);

    py::class_<VnavWaypointProfile>(m, "VnavWaypointProfile")
        .def(py::init<>())
        .def_readwrite("altitude_ft", &VnavWaypointProfile::altitude_ft)
        .def_readwrite("vertical_speed_fpm", &VnavWaypointProfile::vertical_speed_fpm)
        .def_readwrite("mach_number", &VnavWaypointProfile::mach_number)
        .def_readwrite("cas_kt", &VnavWaypointProfile::cas_kt)
        .def_readwrite("tas_ms", &VnavWaypointProfile::tas_ms)
        .def_readwrite("pitch_deg", &VnavWaypointProfile::pitch_deg)
        .def_readwrite("distance_from_start_nm", &VnavWaypointProfile::distance_from_start_nm)
        .def_readwrite("waypoint_index", &VnavWaypointProfile::waypoint_index)
        .def_readwrite("phase", &VnavWaypointProfile::phase);

    py::class_<VnavState>(m, "VnavState")
        .def(py::init<>())
        .def_readwrite("altitude_ft", &VnavState::altitude_ft)
        .def_readwrite("vertical_speed_fpm", &VnavState::vertical_speed_fpm)
        .def_readwrite("mach_number", &VnavState::mach_number)
        .def_readwrite("cas_kt", &VnavState::cas_kt)
        .def_readwrite("tas_ms", &VnavState::tas_ms)
        .def_readwrite("pitch_deg", &VnavState::pitch_deg)
        .def_readwrite("phase", &VnavState::phase)
        .def_readwrite("climb_mode", &VnavState::climb_mode)
        .def_readwrite("descent_mode", &VnavState::descent_mode)
        .def_readwrite("target_altitude_ft", &VnavState::target_altitude_ft)
        .def_readwrite("target_mach", &VnavState::target_mach)
        .def_readwrite("target_cas_kt", &VnavState::target_cas_kt)
        .def_readwrite("active_constraint_index", &VnavState::active_constraint_index)
        .def_readwrite("distance_to_top_of_climb_nm", &VnavState::distance_to_top_of_climb_nm)
        .def_readwrite("distance_to_top_of_descent_nm", &VnavState::distance_to_top_of_descent_nm)
        .def_readwrite("flight_path_angle_deg", &VnavState::flight_path_angle_deg)
        .def_readwrite("distance_along_route_nm", &VnavState::distance_along_route_nm)
        .def_readwrite("altitude_captured", &VnavState::altitude_captured)
        .def_readwrite("constraint_met", &VnavState::constraint_met);

    py::class_<VnavConfig>(m, "VnavConfig")
        .def(py::init<>())
        .def_readwrite("climb_cas_kt", &VnavConfig::climb_cas_kt)
        .def_readwrite("climb_mach", &VnavConfig::climb_mach)
        .def_readwrite("cruise_mach", &VnavConfig::cruise_mach)
        .def_readwrite("descent_cas_kt", &VnavConfig::descent_cas_kt)
        .def_readwrite("descent_mach", &VnavConfig::descent_mach)
        .def_readwrite("cruise_altitude_ft", &VnavConfig::cruise_altitude_ft)
        .def_readwrite("initial_climb_rate_fpm", &VnavConfig::initial_climb_rate_fpm)
        .def_readwrite("standard_descent_rate_fpm", &VnavConfig::standard_descent_rate_fpm)
        .def_readwrite("approach_speed_kt", &VnavConfig::approach_speed_kt)
        .def_readwrite("transition_altitude_ft", &VnavConfig::transition_altitude_ft)
        .def_readwrite("transition_level_ft", &VnavConfig::transition_level_ft)
        .def_readwrite("crossover_altitude_ft", &VnavConfig::crossover_altitude_ft)
        .def_readwrite("max_descent_rate_fpm", &VnavConfig::max_descent_rate_fpm)
        .def_readwrite("max_flight_path_angle_deg", &VnavConfig::max_flight_path_angle_deg)
        .def_readwrite("max_backward_iterations", &VnavConfig::max_backward_iterations)
        .def_readwrite("descent_gradient_tolerance", &VnavConfig::descent_gradient_tolerance)
        .def_readwrite("enable_constraint_relaxation", &VnavConfig::enable_constraint_relaxation);

    py::class_<VnavStateMachine>(m, "VnavStateMachine")
        .def(py::init<>())
        .def(py::init<const VnavConfig&>())
        .def("configure", &VnavStateMachine::configure)
        .def("set_constraints", &VnavStateMachine::setConstraints)
        .def("auto_generate_constraints", &VnavStateMachine::autoGenerateConstraints)
        .def("compute_vertical_profile", &VnavStateMachine::computeVerticalProfile)
        .def("initialize", &VnavStateMachine::initialize)
        .def("update", &VnavStateMachine::update)
        .def("get_state", &VnavStateMachine::getState, py::return_value_policy::reference_internal)
        .def("get_config", &VnavStateMachine::getConfig, py::return_value_policy::reference_internal)
        .def("get_constraints", &VnavStateMachine::getConstraints, py::return_value_policy::reference_internal)
        .def("get_profile", &VnavStateMachine::getProfile, py::return_value_policy::reference_internal)
        .def("get_phase", &VnavStateMachine::getPhase)
        .def("get_altitude_ft", &VnavStateMachine::getAltitudeFt)
        .def("get_vertical_speed_fpm", &VnavStateMachine::getVerticalSpeedFpm)
        .def("get_mach", &VnavStateMachine::getMach)
        .def("get_cas_kt", &VnavStateMachine::getCasKt)
        .def("set_cruise_altitude", &VnavStateMachine::setCruiseAltitude)
        .def("set_phase", &VnavStateMachine::setPhase)
        .def("compute_required_vs_for_path", &VnavStateMachine::computeRequiredVsForPath)
        .def("compute_top_of_descent_distance_nm", &VnavStateMachine::computeTopOfDescentDistanceNm)
        .def("compute_flight_path_angle", &VnavStateMachine::computeFlightPathAngle)
        .def("get_max_descent_rate_fpm", &VnavStateMachine::getMaxDescentRateFpm)
        .def("check_descent_feasibility", [](VnavStateMachine& self, double alt_diff, double dist, double gs) {
            double req_vs = 0.0;
            bool ok = self.checkDescentFeasibility(alt_diff, dist, gs, req_vs);
            return py::make_tuple(ok, req_vs);
        })
        .def("compute_descent_profile_backward", [](VnavStateMachine& self, LnavStateMachine& lnav,
                                                     double cruise_alt, double final_alt) {
            std::vector<VnavWaypointProfile> profile;
            bool ok = self.computeDescentProfileBackward(lnav, profile, cruise_alt, final_alt);
            return py::make_tuple(ok, profile);
        })
        .def_static("feet_to_meters", &VnavStateMachine::feetToMeters)
        .def_static("meters_to_feet", &VnavStateMachine::metersToFeet)
        .def_static("knots_to_mps", &VnavStateMachine::knotsToMetersPerSec)
        .def_static("mps_to_knots", &VnavStateMachine::metersPerSecToKnots)
        .def_static("fpm_to_mps", &VnavStateMachine::fpmToMps)
        .def_static("mps_to_fpm", &VnavStateMachine::mpsToFpm)
        .def_static("nm_to_meters", &VnavStateMachine::nmToMeters)
        .def_static("meters_to_nm", &VnavStateMachine::metersToNm);

    py::class_<TrajectoryPoint>(m, "TrajectoryPoint")
        .def(py::init<>())
        .def_readwrite("time_sec", &TrajectoryPoint::time_sec)
        .def_readwrite("position", &TrajectoryPoint::position)
        .def_readwrite("altitude_ft", &TrajectoryPoint::altitude_ft)
        .def_readwrite("altitude_m", &TrajectoryPoint::altitude_m)
        .def_readwrite("ground_track_deg", &TrajectoryPoint::ground_track_deg)
        .def_readwrite("ground_speed_ms", &TrajectoryPoint::ground_speed_ms)
        .def_readwrite("ground_speed_kt", &TrajectoryPoint::ground_speed_kt)
        .def_readwrite("tas_ms", &TrajectoryPoint::tas_ms)
        .def_readwrite("tas_kt", &TrajectoryPoint::tas_kt)
        .def_readwrite("cas_kt", &TrajectoryPoint::cas_kt)
        .def_readwrite("mach_number", &TrajectoryPoint::mach_number)
        .def_readwrite("vertical_speed_fpm", &TrajectoryPoint::vertical_speed_fpm)
        .def_readwrite("vertical_speed_mps", &TrajectoryPoint::vertical_speed_mps)
        .def_readwrite("pitch_deg", &TrajectoryPoint::pitch_deg)
        .def_readwrite("roll_deg", &TrajectoryPoint::roll_deg)
        .def_readwrite("heading_deg", &TrajectoryPoint::heading_deg)
        .def_readwrite("true_air_temp_k", &TrajectoryPoint::true_air_temp_k)
        .def_readwrite("total_temp_k", &TrajectoryPoint::total_temp_k)
        .def_readwrite("static_pressure_pa", &TrajectoryPoint::static_pressure_pa)
        .def_readwrite("density_kgm3", &TrajectoryPoint::density_kgm3)
        .def_readwrite("speed_of_sound_ms", &TrajectoryPoint::speed_of_sound_ms)
        .def_readwrite("flight_path_angle_deg", &TrajectoryPoint::flight_path_angle_deg)
        .def_readwrite("distance_along_route_nm", &TrajectoryPoint::distance_along_route_nm)
        .def_readwrite("distance_to_next_wp_nm", &TrajectoryPoint::distance_to_next_wp_nm)
        .def_readwrite("cross_track_error_nm", &TrajectoryPoint::cross_track_error_nm)
        .def_readwrite("current_wp_index", &TrajectoryPoint::current_wp_index)
        .def_readwrite("lnav_phase", &TrajectoryPoint::lnav_phase)
        .def_readwrite("vnav_phase", &TrajectoryPoint::vnav_phase)
        .def_readwrite("fuel_remaining_kg", &TrajectoryPoint::fuel_remaining_kg)
        .def_readwrite("fuel_burn_kg", &TrajectoryPoint::fuel_burn_kg);

    py::class_<AircraftPerformance>(m, "AircraftPerformance")
        .def(py::init<>())
        .def_readwrite("gross_weight_kg", &AircraftPerformance::gross_weight_kg)
        .def_readwrite("zero_fuel_weight_kg", &AircraftPerformance::zero_fuel_weight_kg)
        .def_readwrite("fuel_capacity_kg", &AircraftPerformance::fuel_capacity_kg)
        .def_readwrite("cruise_fuel_flow_kg_per_hr", &AircraftPerformance::cruise_fuel_flow_kg_per_hr)
        .def_readwrite("climb_fuel_flow_kg_per_hr", &AircraftPerformance::climb_fuel_flow_kg_per_hr)
        .def_readwrite("descent_fuel_flow_kg_per_hr", &AircraftPerformance::descent_fuel_flow_kg_per_hr)
        .def_readwrite("max_climb_rate_fpm", &AircraftPerformance::max_climb_rate_fpm)
        .def_readwrite("service_ceiling_ft", &AircraftPerformance::service_ceiling_ft)
        .def_readwrite("wing_area_m2", &AircraftPerformance::wing_area_m2);

    py::class_<TrajectoryConfig>(m, "TrajectoryConfig")
        .def(py::init<>())
        .def_readwrite("integration_dt_sec", &TrajectoryConfig::integration_dt_sec)
        .def_readwrite("output_interval_sec", &TrajectoryConfig::output_interval_sec)
        .def_readwrite("wind_direction_deg", &TrajectoryConfig::wind_direction_deg)
        .def_readwrite("wind_speed_ms", &TrajectoryConfig::wind_speed_ms)
        .def_readwrite("isa_deviation_c", &TrajectoryConfig::isa_deviation_c)
        .def_readwrite("include_wind", &TrajectoryConfig::include_wind)
        .def_readwrite("enable_fuel_calc", &TrajectoryConfig::enable_fuel_calc);

    py::class_<TrajectorySolver>(m, "TrajectorySolver")
        .def(py::init<>())
        .def(py::init<const TrajectoryConfig&>())
        .def("set_config", &TrajectorySolver::setConfig)
        .def("set_aircraft", &TrajectorySolver::setAircraft)
        .def("set_lnav_config", &TrajectorySolver::setLnavConfig)
        .def("set_vnav_config", &TrajectorySolver::setVnavConfig)
        .def("load_flight_plan", &TrajectorySolver::loadFlightPlan)
        .def("load_flight_plan_from_arinc", &TrajectorySolver::loadFlightPlanFromArinc)
        .def("set_initial_conditions", &TrajectorySolver::setInitialConditions)
        .def("compute_full_trajectory", &TrajectorySolver::computeFullTrajectory)
        .def("step_integration", &TrajectorySolver::stepIntegration)
        .def("get_trajectory", &TrajectorySolver::getTrajectory, py::return_value_policy::reference_internal)
        .def("get_current_state", &TrajectorySolver::getCurrentState)
        .def("is_complete", &TrajectorySolver::isComplete)
        .def("get_lnav", &TrajectorySolver::getLnav, py::return_value_policy::reference_internal)
        .def("get_vnav", &TrajectorySolver::getVnav, py::return_value_policy::reference_internal)
        .def("get_config", &TrajectorySolver::getConfig, py::return_value_policy::reference_internal)
        .def("get_aircraft", &TrajectorySolver::getAircraft, py::return_value_policy::reference_internal)
        .def("get_total_distance_nm", &TrajectorySolver::getTotalDistanceNm)
        .def("get_total_time_sec", &TrajectorySolver::getTotalTimeSec)
        .def("get_total_fuel_burn_kg", &TrajectorySolver::getTotalFuelBurnKg)
        .def("get_max_altitude_ft", &TrajectorySolver::getMaxAltitudeFt)
        .def("get_average_ground_speed_kt", &TrajectorySolver::getAverageGroundSpeedKt)
        .def("get_waypoint_profiles", &TrajectorySolver::getWaypointProfiles)
        .def("get_point_at_distance", &TrajectorySolver::getPointAtDistance)
        .def("get_point_at_time", &TrajectorySolver::getPointAtTime);
}
