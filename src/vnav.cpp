#include "vnav/vnav.h"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace vnav {

VnavStateMachine::VnavStateMachine() {
    state_.altitude_ft = 0.0;
    state_.vertical_speed_fpm = 0.0;
    state_.mach_number = 0.0;
    state_.cas_kt = 0.0;
    state_.tas_ms = 0.0;
    state_.pitch_deg = 0.0;
    state_.phase = VnavPhase::GROUND;
    state_.climb_mode = ClimbMode::CAS_CLIMB;
    state_.descent_mode = DescentMode::CAS_DESCENT;
    state_.target_altitude_ft = config_.cruise_altitude_ft;
    state_.target_mach = config_.climb_mach;
    state_.target_cas_kt = config_.climb_cas_kt;
    state_.active_constraint_index = 0;
    state_.distance_to_top_of_climb_nm = 0.0;
    state_.distance_to_top_of_descent_nm = 0.0;
    state_.flight_path_angle_deg = 0.0;
    state_.distance_along_route_nm = 0.0;
    state_.altitude_captured = false;
    state_.constraint_met = false;
}

VnavStateMachine::VnavStateMachine(const VnavConfig& config) : config_(config) {
    VnavStateMachine();
}

void VnavStateMachine::configure(const VnavConfig& config) {
    config_ = config;
}

void VnavStateMachine::setConstraints(const std::vector<VnavConstraint>& constraints) {
    constraints_ = constraints;
    std::sort(constraints_.begin(), constraints_.end(),
              [](const VnavConstraint& a, const VnavConstraint& b) {
                  return a.distance_from_start_nm < b.distance_from_start_nm;
              });
}

double VnavStateMachine::feetToMeters(double ft) { return ft * 0.3048; }
double VnavStateMachine::metersToFeet(double m) { return m / 0.3048; }
double VnavStateMachine::knotsToMetersPerSec(double kt) { return kt * 0.514444; }
double VnavStateMachine::metersPerSecToKnots(double ms) { return ms / 0.514444; }
double VnavStateMachine::fpmToMps(double fpm) { return fpm * 0.00508; }
double VnavStateMachine::mpsToFpm(double mps) { return mps / 0.00508; }
double VnavStateMachine::nmToMeters(double nm) { return nm * GreatCircle::NM_TO_M; }
double VnavStateMachine::metersToNm(double m) { return m / GreatCircle::NM_TO_M; }

void VnavStateMachine::autoGenerateConstraints(const LnavStateMachine& lnav) {
    constraints_.clear();
    const auto& waypoints = lnav.getWaypoints();
    double cumulative_dist = 0.0;

    for (size_t i = 0; i < waypoints.size(); i++) {
        const auto& wp = waypoints[i];
        if (i > 0) {
            cumulative_dist += GreatCircle::distanceNm(waypoints[i-1].position, wp.position);
        }

        if (wp.altitude_constraint.type != AltitudeConstraintType::ALT_NONE) {
            VnavConstraint c;
            c.waypoint_index = (int)i;
            c.distance_from_start_nm = cumulative_dist;
            c.alt_type = wp.altitude_constraint.type;
            c.altitude_ft = wp.altitude_constraint.altitude1_ft;
            c.speed_kt = wp.speed_limit_kt;
            constraints_.push_back(c);
        }
    }
}

void VnavStateMachine::computeVerticalProfile(const LnavStateMachine& lnav) {
    profile_.clear();
    const auto& waypoints = lnav.getWaypoints();
    if (waypoints.empty()) return;

    double total_dist = lnav.getTotalDistanceNm();
    double cruise_alt = config_.cruise_altitude_ft;
    double climb_rate = config_.initial_climb_rate_fpm;
    double descent_rate = config_.standard_descent_rate_fpm;
    double avg_gs_kt = 450.0;

    double initial_alt = 0.0;
    if (waypoints[0].altitude_constraint.type != AltitudeConstraintType::ALT_NONE) {
        initial_alt = waypoints[0].altitude_constraint.altitude1_ft;
    }
    double climb_alt_needed = cruise_alt - initial_alt;
    double climb_time_min = climb_alt_needed / climb_rate;
    double climb_dist_nm = avg_gs_kt * climb_time_min / 60.0;

    double final_alt = 0.0;
    if (!waypoints.empty()) {
        const auto& last_wp = waypoints.back();
        if (last_wp.altitude_constraint.type != AltitudeConstraintType::ALT_NONE) {
            final_alt = last_wp.altitude_constraint.altitude1_ft;
        }
    }
    double descent_alt_needed = cruise_alt - final_alt;
    double descent_time_min = descent_alt_needed / descent_rate;
    double descent_dist_nm = avg_gs_kt * descent_time_min / 60.0;

    double tod_nm = total_dist - descent_dist_nm;
    double toc_nm = climb_dist_nm;

    std::vector<VnavWaypointProfile> backward_descent;
    bool backward_success = computeDescentProfileBackward(lnav, backward_descent, cruise_alt, final_alt);

    if (backward_success && !backward_descent.empty()) {
        double actual_tod_nm = backward_descent.front().distance_from_start_nm;
        for (size_t i = 0; i < backward_descent.size(); i++) {
            if (backward_descent[i].altitude_ft < cruise_alt - 500.0) {
                actual_tod_nm = backward_descent[i].distance_from_start_nm;
                break;
            }
        }
        tod_nm = std::min(tod_nm, actual_tod_nm);
    }

    state_.distance_to_top_of_climb_nm = toc_nm;
    state_.distance_to_top_of_descent_nm = tod_nm;

    double cumulative_dist = 0.0;
    for (size_t i = 0; i < waypoints.size(); i++) {
        if (i > 0) {
            cumulative_dist += GreatCircle::distanceNm(waypoints[i-1].position, waypoints[i].position);
        }

        VnavWaypointProfile p;
        p.waypoint_index = (int)i;
        p.distance_from_start_nm = cumulative_dist;
        p.mach_number = 0.0;
        p.vertical_speed_fpm = 0.0;
        p.pitch_deg = 0.0;

        if (cumulative_dist <= toc_nm) {
            p.phase = VnavPhase::CLIMB;
            double climb_fraction = cumulative_dist / std::max(toc_nm, 0.1);
            p.altitude_ft = initial_alt + climb_alt_needed * climb_fraction;
            p.vertical_speed_fpm = climb_rate;
            p.cas_kt = config_.climb_cas_kt;
        } else if (cumulative_dist >= tod_nm && backward_success && i < backward_descent.size()) {
            p.phase = VnavPhase::DESCENT;
            p.altitude_ft = backward_descent[i].altitude_ft;
            p.vertical_speed_fpm = backward_descent[i].vertical_speed_fpm;
            p.cas_kt = config_.descent_cas_kt;
        } else if (cumulative_dist >= tod_nm) {
            p.phase = VnavPhase::DESCENT;
            double descent_fraction = (cumulative_dist - tod_nm) / std::max(descent_dist_nm, 0.1);
            descent_fraction = std::min(descent_fraction, 1.0);
            p.altitude_ft = cruise_alt - descent_alt_needed * descent_fraction;
            p.vertical_speed_fpm = -descent_rate;
            p.cas_kt = config_.descent_cas_kt;
        } else {
            p.phase = VnavPhase::CRUISE;
            p.altitude_ft = cruise_alt;
            p.vertical_speed_fpm = 0.0;
            p.cas_kt = metersPerSecToKnots(InternationalStandardAtmosphere::machToTas(
                config_.cruise_mach, feetToMeters(cruise_alt)));
            p.mach_number = config_.cruise_mach;
        }

        if (waypoints[i].altitude_constraint.type != AltitudeConstraintType::ALT_NONE) {
            switch (waypoints[i].altitude_constraint.type) {
                case AltitudeConstraintType::ALT_AT:
                    p.altitude_ft = waypoints[i].altitude_constraint.altitude1_ft;
                    break;
                case AltitudeConstraintType::ALT_AT_OR_ABOVE:
                    p.altitude_ft = std::max(p.altitude_ft, waypoints[i].altitude_constraint.altitude1_ft);
                    break;
                case AltitudeConstraintType::ALT_AT_OR_BELOW:
                    p.altitude_ft = std::min(p.altitude_ft, waypoints[i].altitude_constraint.altitude1_ft);
                    break;
                case AltitudeConstraintType::ALT_BETWEEN:
                    p.altitude_ft = std::max(p.altitude_ft, waypoints[i].altitude_constraint.altitude1_ft);
                    p.altitude_ft = std::min(p.altitude_ft, waypoints[i].altitude_constraint.altitude2_ft);
                    break;
                default:
                    break;
            }
        }

        double alt_m = feetToMeters(p.altitude_ft);
        if (p.mach_number <= 0.001) {
            double tas_ms = knotsToMetersPerSec(p.cas_kt);
            p.tas_ms = InternationalStandardAtmosphere::casToTas(knotsToMetersPerSec(p.cas_kt), alt_m);
            p.mach_number = InternationalStandardAtmosphere::tasToMach(p.tas_ms, alt_m);
        } else {
            p.tas_ms = InternationalStandardAtmosphere::machToTas(p.mach_number, alt_m);
            p.cas_kt = metersPerSecToKnots(p.tas_ms);
        }

        profile_.push_back(p);
    }
}

void VnavStateMachine::initialize(double initial_alt_ft, double initial_cas_kt) {
    state_.altitude_ft = initial_alt_ft;
    state_.cas_kt = initial_cas_kt;
    state_.phase = VnavPhase::CLIMB;
    state_.climb_mode = ClimbMode::CAS_CLIMB;
    state_.target_altitude_ft = config_.cruise_altitude_ft;
    state_.target_cas_kt = config_.climb_cas_kt;
    state_.target_mach = config_.climb_mach;
    state_.active_constraint_index = 0;
    state_.altitude_captured = false;
    state_.constraint_met = false;

    double alt_m = feetToMeters(initial_alt_ft);
    state_.tas_ms = InternationalStandardAtmosphere::casToTas(knotsToMetersPerSec(initial_cas_kt), alt_m);
    state_.mach_number = InternationalStandardAtmosphere::tasToMach(state_.tas_ms, alt_m);
    state_.vertical_speed_fpm = config_.initial_climb_rate_fpm;
}

double VnavStateMachine::computeRequiredVsForPath(double current_alt_ft, double target_alt_ft,
                                                    double distance_nm, double ground_speed_kt) const {
    if (distance_nm <= 0.0) return 0.0;
    double delta_alt = target_alt_ft - current_alt_ft;
    double time_min = distance_nm / std::max(ground_speed_kt, 0.1) * 60.0;
    return delta_alt / std::max(time_min, 0.1);
}

double VnavStateMachine::computeTopOfDescentDistanceNm(double current_alt_ft, double target_alt_ft,
                                                         double descent_rate_fpm, double ground_speed_kt) const {
    double delta_alt = current_alt_ft - target_alt_ft;
    double time_min = delta_alt / std::max(std::abs(descent_rate_fpm), 1.0);
    return ground_speed_kt * time_min / 60.0;
}

double VnavStateMachine::computeFlightPathAngle(double vs_fpm, double gs_kt) const {
    if (gs_kt <= 0.0) return 0.0;
    double vs_nm_per_min = vs_fpm / 6076.12;
    double gs_nm_per_min = gs_kt / 60.0;
    return std::atan2(vs_nm_per_min, gs_nm_per_min) * 180.0 / M_PI;
}

double VnavStateMachine::getMaxDescentRateFpm(double altitude_ft, double gs_kt) const {
    double max_vs_by_angle = 0.0;
    if (gs_kt > 0.0) {
        double max_angle_rad = config_.max_flight_path_angle_deg * M_PI / 180.0;
        double vs_nm_per_min = std::tan(max_angle_rad) * (gs_kt / 60.0);
        max_vs_by_angle = vs_nm_per_min * 6076.12;
    }
    return std::min(config_.max_descent_rate_fpm, max_vs_by_angle);
}

bool VnavStateMachine::checkDescentFeasibility(double alt_diff_ft, double distance_nm,
                                                double gs_kt, double& required_vs_fpm) const {
    if (distance_nm <= 0.001 || gs_kt <= 0.1) {
        required_vs_fpm = 0.0;
        return false;
    }
    double time_min = distance_nm / gs_kt * 60.0;
    required_vs_fpm = alt_diff_ft / time_min;
    double max_descent = getMaxDescentRateFpm(0.0, gs_kt);
    if (std::abs(required_vs_fpm) > max_descent + config_.descent_gradient_tolerance * 100.0) {
        return false;
    }
    return true;
}

bool VnavStateMachine::computeDescentProfileBackward(const LnavStateMachine& lnav,
                                                      std::vector<VnavWaypointProfile>& descent_profile,
                                                      double cruise_alt_ft,
                                                      double final_alt_ft) {
    const auto& waypoints = lnav.getWaypoints();
    if (waypoints.size() < 2) return false;

    int n = (int)waypoints.size();
    std::vector<double> distances_nm(n, 0.0);
    std::vector<double> altitudes_ft(n, 0.0);
    std::vector<bool> constraint_is_hard(n, false);
    std::vector<double> original_constraint_alt(n, 0.0);

    double cumulative_dist = 0.0;
    for (int i = 0; i < n; i++) {
        if (i > 0) {
            cumulative_dist += GreatCircle::distanceNm(
                waypoints[i-1].position, waypoints[i].position);
        }
        distances_nm[i] = cumulative_dist;

        const auto& ac = waypoints[i].altitude_constraint;
        if (ac.type == AltitudeConstraintType::ALT_AT) {
            altitudes_ft[i] = ac.altitude1_ft;
            original_constraint_alt[i] = ac.altitude1_ft;
            constraint_is_hard[i] = true;
        } else if (ac.type == AltitudeConstraintType::ALT_AT_OR_BELOW) {
            altitudes_ft[i] = ac.altitude1_ft;
            original_constraint_alt[i] = ac.altitude1_ft;
            constraint_is_hard[i] = false;
        } else if (ac.type == AltitudeConstraintType::ALT_AT_OR_ABOVE) {
            altitudes_ft[i] = ac.altitude1_ft;
            original_constraint_alt[i] = ac.altitude1_ft;
            constraint_is_hard[i] = true;
        } else if (ac.type == AltitudeConstraintType::ALT_BETWEEN) {
            altitudes_ft[i] = ac.altitude2_ft;
            original_constraint_alt[i] = ac.altitude2_ft;
            constraint_is_hard[i] = false;
        } else {
            altitudes_ft[i] = -1.0;
            original_constraint_alt[i] = -1.0;
            constraint_is_hard[i] = false;
        }
    }

    altitudes_ft[n-1] = final_alt_ft;
    original_constraint_alt[n-1] = final_alt_ft;
    constraint_is_hard[n-1] = true;

    double avg_gs_kt = 400.0;
    int iterations = 0;
    bool converged = false;
    const double convergence_tolerance_ft = 1.0;

    while (!converged && iterations < config_.max_backward_iterations) {
        converged = true;
        iterations++;
        double max_adjustment_ft = 0.0;

        for (int i = n - 2; i >= 0; i--) {
            double seg_dist = distances_nm[i+1] - distances_nm[i];
            if (seg_dist <= 0.001) continue;

            double next_alt = altitudes_ft[i+1];
            double current_alt = altitudes_ft[i];
            double prev_alt = current_alt;

            if (current_alt < 0) {
                double max_alt_drop = getMaxDescentRateFpm(cruise_alt_ft, avg_gs_kt)
                                    * (seg_dist / avg_gs_kt * 60.0);
                double min_alt_for_segment = next_alt + std::abs(max_alt_drop);

                if (min_alt_for_segment > cruise_alt_ft) {
                    altitudes_ft[i] = cruise_alt_ft;
                } else {
                    altitudes_ft[i] = min_alt_for_segment;
                }
                max_adjustment_ft = std::max(max_adjustment_ft,
                    std::abs(altitudes_ft[i] - prev_alt));
                converged = false;
                continue;
            }

            if (current_alt < next_alt - 1.0) {
                altitudes_ft[i] = next_alt;
                max_adjustment_ft = std::max(max_adjustment_ft,
                    std::abs(altitudes_ft[i] - prev_alt));
                converged = false;
                continue;
            }

            double alt_diff = current_alt - next_alt;
            double time_min = seg_dist / avg_gs_kt * 60.0;
            double required_vs = alt_diff / std::max(time_min, 0.1);
            double max_allowed_vs = getMaxDescentRateFpm(current_alt, avg_gs_kt);

            if (required_vs > max_allowed_vs + config_.descent_gradient_tolerance * 100.0) {
                if (!constraint_is_hard[i] && config_.enable_constraint_relaxation) {
                    double max_alt_drop = max_allowed_vs * (seg_dist / avg_gs_kt * 60.0);
                    double new_alt = next_alt + std::abs(max_alt_drop);
                    if (new_alt > current_alt + convergence_tolerance_ft) {
                        altitudes_ft[i] = std::min(new_alt, cruise_alt_ft);
                        max_adjustment_ft = std::max(max_adjustment_ft,
                            std::abs(altitudes_ft[i] - prev_alt));
                        converged = false;
                    }
                } else if (constraint_is_hard[i]) {
                    double max_alt_drop = max_allowed_vs * (seg_dist / avg_gs_kt * 60.0);
                    double max_possible_next_alt = current_alt - std::abs(max_alt_drop);

                    if (max_possible_next_alt > next_alt + convergence_tolerance_ft) {
                        if (!constraint_is_hard[i+1] && config_.enable_constraint_relaxation) {
                            altitudes_ft[i+1] = max_possible_next_alt;
                            max_adjustment_ft = std::max(max_adjustment_ft,
                                std::abs(altitudes_ft[i+1] - next_alt));
                            converged = false;
                        }
                    }
                }
            }
        }

        if (max_adjustment_ft < convergence_tolerance_ft) {
            converged = true;
        }
    }

    if (iterations >= config_.max_backward_iterations) {
        return false;
    }

    descent_profile.clear();
    for (int i = 0; i < n; i++) {
        VnavWaypointProfile p;
        p.waypoint_index = i;
        p.distance_from_start_nm = distances_nm[i];
        p.altitude_ft = std::max(altitudes_ft[i], final_alt_ft);
        p.phase = VnavPhase::DESCENT;
        p.mach_number = 0.0;
        p.cas_kt = config_.descent_cas_kt;
        p.vertical_speed_fpm = 0.0;

        if (i < n - 1) {
            double seg_dist = distances_nm[i+1] - distances_nm[i];
            if (seg_dist > 0.001) {
                double alt_diff = altitudes_ft[i+1] - altitudes_ft[i];
                double time_min = seg_dist / avg_gs_kt * 60.0;
                p.vertical_speed_fpm = alt_diff / std::max(time_min, 0.1);
            }
        }

        descent_profile.push_back(p);
    }

    if (iterations >= config_.max_backward_iterations) {
        return false;
    }
    return true;
}

void VnavStateMachine::setCruiseAltitude(double alt_ft) {
    config_.cruise_altitude_ft = alt_ft;
    state_.target_altitude_ft = alt_ft;
}

void VnavStateMachine::setPhase(VnavPhase phase) {
    state_.phase = phase;
}

VnavConstraint VnavStateMachine::findNextConstraint(double distance_along_nm) {
    for (const auto& c : constraints_) {
        if (c.distance_from_start_nm > distance_along_nm - 0.5) {
            return c;
        }
    }
    VnavConstraint empty;
    empty.altitude_ft = 0.0;
    empty.speed_kt = 0.0;
    empty.alt_type = AltitudeConstraintType::ALT_NONE;
    empty.waypoint_index = -1;
    empty.distance_from_start_nm = -1.0;
    return empty;
}

void VnavStateMachine::checkConstraintCapture(double distance_along_nm, double altitude_ft) {
    while (state_.active_constraint_index < (int)constraints_.size()) {
        const auto& c = constraints_[state_.active_constraint_index];
        if (distance_along_nm >= c.distance_from_start_nm) {
            state_.constraint_met = true;
            state_.active_constraint_index++;
        } else {
            break;
        }
    }
}

void VnavStateMachine::updateSpeeds(double altitude_ft, double& mach, double& cas_kt, double& tas_ms) {
    double alt_m = feetToMeters(altitude_ft);
    if (state_.phase == VnavPhase::CLIMB) {
        if (altitude_ft < config_.crossover_altitude_ft) {
            state_.climb_mode = ClimbMode::CAS_CLIMB;
            cas_kt = config_.climb_cas_kt;
            tas_ms = InternationalStandardAtmosphere::casToTas(knotsToMetersPerSec(cas_kt), alt_m);
            mach = InternationalStandardAtmosphere::tasToMach(tas_ms, alt_m);
        } else {
            state_.climb_mode = ClimbMode::MACH_CLIMB;
            mach = config_.climb_mach;
            tas_ms = InternationalStandardAtmosphere::machToTas(mach, alt_m);
            cas_kt = metersPerSecToKnots(tas_ms);
        }
    } else if (state_.phase == VnavPhase::CRUISE) {
        mach = config_.cruise_mach;
        tas_ms = InternationalStandardAtmosphere::machToTas(mach, alt_m);
        cas_kt = metersPerSecToKnots(tas_ms);
    } else if (state_.phase == VnavPhase::DESCENT) {
        if (altitude_ft > config_.crossover_altitude_ft) {
            state_.descent_mode = DescentMode::MACH_DESCENT;
            mach = config_.descent_mach;
            tas_ms = InternationalStandardAtmosphere::machToTas(mach, alt_m);
            cas_kt = metersPerSecToKnots(tas_ms);
        } else {
            state_.descent_mode = DescentMode::CAS_DESCENT;
            cas_kt = config_.descent_cas_kt;
            tas_ms = InternationalStandardAtmosphere::casToTas(knotsToMetersPerSec(cas_kt), alt_m);
            mach = InternationalStandardAtmosphere::tasToMach(tas_ms, alt_m);
        }
    } else if (state_.phase == VnavPhase::APPROACH) {
        cas_kt = config_.approach_speed_kt;
        tas_ms = InternationalStandardAtmosphere::casToTas(knotsToMetersPerSec(cas_kt), alt_m);
        mach = InternationalStandardAtmosphere::tasToMach(tas_ms, alt_m);
    }
}

void VnavStateMachine::updateClimb(double dt_sec, double ground_speed_ms, double distance_along_nm) {
    double gs_kt = metersPerSecToKnots(ground_speed_ms);
    double dt_min = dt_sec / 60.0;

    auto next_constraint = findNextConstraint(distance_along_nm);
    double target_alt = config_.cruise_altitude_ft;
    if (next_constraint.alt_type != AltitudeConstraintType::ALT_NONE) {
        switch (next_constraint.alt_type) {
            case AltitudeConstraintType::ALT_AT:
            case AltitudeConstraintType::ALT_AT_OR_ABOVE:
                target_alt = std::min(target_alt, next_constraint.altitude_ft);
                break;
            case AltitudeConstraintType::ALT_AT_OR_BELOW:
                target_alt = std::min(target_alt, next_constraint.altitude_ft);
                break;
            default:
                break;
        }
    }
    state_.target_altitude_ft = target_alt;

    double alt_diff = target_alt - state_.altitude_ft;
    if (alt_diff <= 100.0) {
        state_.vertical_speed_fpm = std::min(state_.vertical_speed_fpm, alt_diff / dt_min);
        if (alt_diff <= 10.0) {
            state_.altitude_ft = target_alt;
            state_.vertical_speed_fpm = 0.0;
            state_.altitude_captured = true;
            state_.phase = VnavPhase::CRUISE;
        }
    } else {
        state_.vertical_speed_fpm = config_.initial_climb_rate_fpm;
    }

    state_.altitude_ft += state_.vertical_speed_fpm * dt_min;
    state_.altitude_ft = std::min(state_.altitude_ft, target_alt);

    updateSpeeds(state_.altitude_ft, state_.mach_number, state_.cas_kt, state_.tas_ms);
    state_.flight_path_angle_deg = computeFlightPathAngle(state_.vertical_speed_fpm, gs_kt);
    state_.pitch_deg = state_.flight_path_angle_deg + 2.0;
    state_.distance_along_route_nm = distance_along_nm;

    checkConstraintCapture(distance_along_nm, state_.altitude_ft);
}

void VnavStateMachine::updateCruise(double dt_sec, double ground_speed_ms, double distance_along_nm) {
    double gs_kt = metersPerSecToKnots(ground_speed_ms);
    state_.altitude_ft = config_.cruise_altitude_ft;
    state_.vertical_speed_fpm = 0.0;
    state_.altitude_captured = true;

    updateSpeeds(state_.altitude_ft, state_.mach_number, state_.cas_kt, state_.tas_ms);
    state_.flight_path_angle_deg = 0.0;
    state_.pitch_deg = 2.0;
    state_.distance_along_route_nm = distance_along_nm;

    if (distance_along_nm >= state_.distance_to_top_of_descent_nm &&
        state_.distance_to_top_of_descent_nm > 0.0) {
        state_.phase = VnavPhase::DESCENT;
    }

    auto next_constraint = findNextConstraint(distance_along_nm);
    if (next_constraint.alt_type != AltitudeConstraintType::ALT_NONE &&
        next_constraint.altitude_ft < state_.altitude_ft - 1000.0 &&
        next_constraint.distance_from_start_nm > 0.0) {
        double dist_to_constraint = next_constraint.distance_from_start_nm - distance_along_nm;
        double tod_dist = computeTopOfDescentDistanceNm(
            state_.altitude_ft, next_constraint.altitude_ft,
            config_.standard_descent_rate_fpm, gs_kt);
        if (dist_to_constraint <= tod_dist + 5.0) {
            state_.phase = VnavPhase::DESCENT;
            state_.target_altitude_ft = next_constraint.altitude_ft;
        }
    }

    if (!constraints_.empty()) {
        const auto& last_c = constraints_.back();
        if (last_c.altitude_ft < state_.altitude_ft - 1000.0) {
            double dist_to_last = last_c.distance_from_start_nm - distance_along_nm;
            double tod_dist = computeTopOfDescentDistanceNm(
                state_.altitude_ft, last_c.altitude_ft,
                config_.standard_descent_rate_fpm, gs_kt);
            if (dist_to_last <= tod_dist + 5.0) {
                state_.phase = VnavPhase::DESCENT;
                state_.target_altitude_ft = last_c.altitude_ft;
            }
        }
    }

    checkConstraintCapture(distance_along_nm, state_.altitude_ft);
}

void VnavStateMachine::updateDescent(double dt_sec, double ground_speed_ms, double distance_along_nm) {
    double gs_kt = metersPerSecToKnots(ground_speed_ms);
    double dt_min = dt_sec / 60.0;

    auto next_constraint = findNextConstraint(distance_along_nm);
    double target_alt = 0.0;
    if (next_constraint.alt_type != AltitudeConstraintType::ALT_NONE) {
        switch (next_constraint.alt_type) {
            case AltitudeConstraintType::ALT_AT:
            case AltitudeConstraintType::ALT_AT_OR_BELOW:
                target_alt = next_constraint.altitude_ft;
                break;
            case AltitudeConstraintType::ALT_AT_OR_ABOVE:
                target_alt = std::max(target_alt, next_constraint.altitude_ft);
                break;
            default:
                break;
        }
    } else {
        target_alt = 0.0;
    }
    state_.target_altitude_ft = target_alt;

    double dist_to_constraint = next_constraint.distance_from_start_nm - distance_along_nm;
    if (dist_to_constraint > 0.0 && next_constraint.alt_type != AltitudeConstraintType::ALT_NONE) {
        double required_vs = computeRequiredVsForPath(
            state_.altitude_ft, target_alt, dist_to_constraint, gs_kt);
        double max_descent = -getMaxDescentRateFpm(state_.altitude_ft, gs_kt);
        state_.vertical_speed_fpm = std::max(required_vs, max_descent);
    } else {
        state_.vertical_speed_fpm = -config_.standard_descent_rate_fpm;
    }

    double alt_diff = state_.altitude_ft - target_alt;
    if (alt_diff <= 50.0 && target_alt > 0.0) {
        state_.altitude_ft = target_alt;
        state_.vertical_speed_fpm = 0.0;
        state_.altitude_captured = true;
        if (target_alt < 10000.0) {
            state_.phase = VnavPhase::APPROACH;
        }
    }

    state_.altitude_ft += state_.vertical_speed_fpm * dt_min;
    state_.altitude_ft = std::max(state_.altitude_ft, target_alt);

    if (state_.altitude_ft <= 1500.0) {
        state_.phase = VnavPhase::APPROACH;
    }

    updateSpeeds(state_.altitude_ft, state_.mach_number, state_.cas_kt, state_.tas_ms);
    state_.flight_path_angle_deg = computeFlightPathAngle(state_.vertical_speed_fpm, gs_kt);
    state_.pitch_deg = state_.flight_path_angle_deg + 2.0;
    state_.distance_along_route_nm = distance_along_nm;

    checkConstraintCapture(distance_along_nm, state_.altitude_ft);
}

void VnavStateMachine::updateApproach(double dt_sec, double ground_speed_ms, double distance_along_nm) {
    double gs_kt = metersPerSecToKnots(ground_speed_ms);
    double dt_min = dt_sec / 60.0;

    state_.vertical_speed_fpm = -800.0;
    state_.altitude_ft += state_.vertical_speed_fpm * dt_min;

    if (state_.altitude_ft <= 0.0) {
        state_.altitude_ft = 0.0;
        state_.vertical_speed_fpm = 0.0;
        state_.phase = VnavPhase::COMPLETE;
    }

    double alt_m = feetToMeters(std::max(state_.altitude_ft, 0.0));
    state_.cas_kt = std::max(config_.approach_speed_kt, state_.cas_kt - 5.0 * dt_min);
    state_.tas_ms = InternationalStandardAtmosphere::casToTas(knotsToMetersPerSec(state_.cas_kt), alt_m);
    state_.mach_number = InternationalStandardAtmosphere::tasToMach(state_.tas_ms, alt_m);
    state_.flight_path_angle_deg = computeFlightPathAngle(state_.vertical_speed_fpm, gs_kt);
    state_.pitch_deg = state_.flight_path_angle_deg + 3.0;
    state_.distance_along_route_nm = distance_along_nm;
}

VnavState VnavStateMachine::update(double dt_sec, double ground_speed_ms, double distance_along_nm) {
    switch (state_.phase) {
        case VnavPhase::GROUND:
            if (state_.altitude_ft > 50.0) {
                state_.phase = VnavPhase::CLIMB;
            }
            break;
        case VnavPhase::CLIMB:
            updateClimb(dt_sec, ground_speed_ms, distance_along_nm);
            break;
        case VnavPhase::CRUISE:
            updateCruise(dt_sec, ground_speed_ms, distance_along_nm);
            break;
        case VnavPhase::DESCENT:
            updateDescent(dt_sec, ground_speed_ms, distance_along_nm);
            break;
        case VnavPhase::APPROACH:
            updateApproach(dt_sec, ground_speed_ms, distance_along_nm);
            break;
        case VnavPhase::COMPLETE:
        case VnavPhase::GO_AROUND:
        default:
            break;
    }
    return state_;
}

}
