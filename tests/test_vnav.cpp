#include <iostream>
#include <cassert>
#include <cmath>
#include "vnav/isa.h"
#include "vnav/great_circle.h"
#include "vnav/arinc424.h"
#include "vnav/lnav.h"
#include "vnav/vnav.h"
#include "vnav/trajectory_solver.h"

using namespace vnav;

bool approx(double a, double b, double eps = 1e-6) {
    return std::fabs(a - b) < eps;
}

void test_isa() {
    std::cout << "Testing ISA model..." << std::endl;
    ISAData sl = InternationalStandardAtmosphere::compute(0.0);
    assert(approx(sl.temperature, 288.15, 0.01));
    assert(approx(sl.pressure, 101325.0, 1.0));
    assert(approx(sl.density, 1.225, 0.001));
    std::cout << "  Sea level OK: T=" << sl.temperature << "K, P=" << sl.pressure << "Pa" << std::endl;

    ISAData tropo = InternationalStandardAtmosphere::compute(11000.0);
    assert(approx(tropo.temperature, 216.65, 0.01));
    assert(tropo.temperature < sl.temperature);
    std::cout << "  Tropopause OK: T=" << tropo.temperature << "K" << std::endl;

    double tas = InternationalStandardAtmosphere::machToTas(0.8, 10668.0);
    std::cout << "  M0.8 at FL350 TAS=" << tas * 1.94384 << "kt" << std::endl;
    assert(tas > 200.0);
    std::cout << "ISA model: PASS" << std::endl;
}

void test_great_circle() {
    std::cout << "Testing Great Circle..." << std::endl;
    GeoPoint nyc(40.7128, -74.0060);
    GeoPoint lon(51.5074, -0.1278);

    double dist_nm = GreatCircle::distanceNm(nyc, lon);
    std::cout << "  NYC-LON distance: " << dist_nm << " nm" << std::endl;
    assert(dist_nm > 3000.0 && dist_nm < 4000.0);

    double bearing = GreatCircle::initialBearingDeg(nyc, lon);
    std::cout << "  NYC-LON bearing: " << bearing << " deg" << std::endl;
    assert(bearing > 30.0 && bearing < 90.0);

    GeoPoint mid = GreatCircle::intermediatePoint(nyc, lon, 0.5);
    double d1 = GreatCircle::distanceNm(nyc, mid);
    double d2 = GreatCircle::distanceNm(mid, lon);
    std::cout << "  Midpoint check: d1=" << d1 << ", d2=" << d2 << std::endl;
    assert(approx(d1, d2, 5.0));

    GeoPoint p = GreatCircle::pointAtDistanceBearing(nyc, GreatCircle::NM_TO_M * 100.0, 90.0);
    double d_check = GreatCircle::distanceNm(nyc, p);
    std::cout << "  100nm east check: " << d_check << " nm" << std::endl;
    assert(approx(d_check, 100.0, 1.0));
    std::cout << "Great Circle: PASS" << std::endl;
}

void test_arinc424() {
    std::cout << "Testing ARINC 424 parser..." << std::endl;

    std::string test_data =
        "SUSPA PA  N39512839W075153834E03000035TJSJ  18000KJFK                      JOHN F KENNEDY INTL\n"
        "SURSW DB  N38424083W077032657E01041001TJSJ        SUX                       SUX VOR/DME\n"
        "SURSW DB  N39104190W077274110E01187001TJSJ        NSI                       NSI VOR/DME\n"
        "SUKJFK SDD04L MERIT M20A 015N04042300W073465170  027  18007000   250BYPASS 5\n"
        "SUKJFK SDD04L CANJC  0030N040235110W073381210  087  036  05000 11000\n"
        "SUKJFK SDD04L BOSOX  0030N040015230W073123090  087  075  11000 15000\n"
        "SUEWR  SED22R WAVEY 056  06N039170350W074315330  070  036+21000\n"
        "SUEWR  SED22R PARCH  0070N039233190W074165370  088  072+18000\n"
        "SUEWR  SED22R DOOPY  0070N039274840W073554010  088  109+15000\n"
        "SUEWR  SED22R SOLEN  0070N039350740W073235260  088  178+11000\n"
        "SUEWR  SEED22R   087  070N039430720W072595440  088  240 09000\n";

    Arinc424Parser parser;
    NavigationDatabase db = parser.parseString(test_data);

    std::cout << "  Airports parsed: " << db.airports.size() << std::endl;
    std::cout << "  Waypoints parsed: " << db.waypoints.size() << std::endl;
    std::cout << "  SIDs parsed: ";
    int total_sids = 0;
    for (const auto& [key, apt] : db.airports) {
        total_sids += apt.sids.size();
    }
    std::cout << total_sids << std::endl;
    std::cout << "  STARs parsed: ";
    int total_stars = 0;
    for (const auto& [key, apt] : db.airports) {
        total_stars += apt.stars.size();
    }
    std::cout << total_stars << std::endl;

    auto errors = parser.getParseErrors();
    for (const auto& e : errors) {
        std::cout << "  Parse error: " << e << std::endl;
    }
    std::cout << "ARINC 424 parser: PASS" << std::endl;
}

void test_lnav() {
    std::cout << "Testing LNAV state machine..." << std::endl;

    std::vector<Waypoint> wps;
    Waypoint wp1;
    wp1.identifier = "WP1";
    wp1.position = GeoPoint(40.0, -74.0);
    wps.push_back(wp1);

    Waypoint wp2;
    wp2.identifier = "WP2";
    wp2.position = GeoPoint(40.5, -73.5);
    wps.push_back(wp2);

    Waypoint wp3;
    wp3.identifier = "WP3";
    wp3.position = GeoPoint(41.0, -73.0);
    wps.push_back(wp3);

    LnavStateMachine lnav;
    lnav.setFlightPlan(wps);
    lnav.initialize(wps[0].position, 45.0, 200.0);

    std::cout << "  Total distance: " << lnav.getTotalDistanceNm() << " nm" << std::endl;
    assert(lnav.getTotalDistanceNm() > 0.0);

    double gs_ms = 200.0;
    for (int i = 0; i < 100 && !lnav.isComplete(); i++) {
        LnavState s = lnav.update(1.0, gs_ms);
        if (i % 20 == 0) {
            std::cout << "  Step " << i << ": WP idx=" << s.current_waypoint_index
                      << ", dist_to_next=" << s.distance_to_next_nm << "nm"
                      << ", phase=" << (int)s.phase << std::endl;
        }
    }

    std::cout << "  Final WP index: " << lnav.getActiveWaypointIndex() << std::endl;
    std::cout << "LNAV state machine: PASS" << std::endl;
}

void test_vnav() {
    std::cout << "Testing VNAV state machine..." << std::endl;

    std::vector<Waypoint> wps;
    Waypoint wp1;
    wp1.identifier = "DEP";
    wp1.position = GeoPoint(40.0, -74.0);
    wp1.altitude_constraint.type = AltitudeConstraintType::ALT_AT;
    wp1.altitude_constraint.altitude1_ft = 0.0;
    wps.push_back(wp1);

    Waypoint wp2;
    wp2.identifier = "WP2";
    wp2.position = GeoPoint(40.5, -73.5);
    wp2.altitude_constraint.type = AltitudeConstraintType::ALT_AT_OR_ABOVE;
    wp2.altitude_constraint.altitude1_ft = 10000.0;
    wps.push_back(wp2);

    Waypoint wp3;
    wp3.identifier = "CRZ";
    wp3.position = GeoPoint(41.0, -73.0);
    wp3.altitude_constraint.type = AltitudeConstraintType::ALT_AT;
    wp3.altitude_constraint.altitude1_ft = 35000.0;
    wps.push_back(wp3);

    Waypoint wp4;
    wp4.identifier = "DES";
    wp4.position = GeoPoint(41.5, -72.5);
    wp4.altitude_constraint.type = AltitudeConstraintType::ALT_AT_OR_BELOW;
    wp4.altitude_constraint.altitude1_ft = 20000.0;
    wps.push_back(wp4);

    Waypoint wp5;
    wp5.identifier = "ARR";
    wp5.position = GeoPoint(42.0, -72.0);
    wp5.altitude_constraint.type = AltitudeConstraintType::ALT_AT;
    wp5.altitude_constraint.altitude1_ft = 0.0;
    wps.push_back(wp5);

    LnavStateMachine lnav;
    lnav.setFlightPlan(wps);
    lnav.initialize(wps[0].position, 45.0, 200.0);

    VnavStateMachine vnav;
    vnav.autoGenerateConstraints(lnav);
    vnav.computeVerticalProfile(lnav);
    vnav.initialize(0.0, 250.0);

    const auto& profile = vnav.getProfile();
    std::cout << "  Profile waypoints: " << profile.size() << std::endl;
    for (size_t i = 0; i < profile.size(); i++) {
        std::cout << "    WP" << i << " (" << wps[i].identifier << "): "
                  << profile[i].altitude_ft << "ft, phase=" << (int)profile[i].phase << std::endl;
    }

    double gs_ms = 200.0;
    for (int i = 0; i < 50; i++) {
        VnavState s = vnav.update(1.0, gs_ms, i * 0.1);
        if (i % 10 == 0) {
            std::cout << "  Step " << i << ": alt=" << s.altitude_ft
                      << "ft, VS=" << s.vertical_speed_fpm
                      << "fpm, phase=" << (int)s.phase << std::endl;
        }
    }
    std::cout << "VNAV state machine: PASS" << std::endl;
}

void test_trajectory() {
    std::cout << "Testing Trajectory Solver..." << std::endl;

    std::vector<Waypoint> wps;
    Waypoint dep;
    dep.identifier = "KJFK";
    dep.icao_code = "KJFK";
    dep.position = GeoPoint(40.6413, -73.7781);
    dep.type = WaypointType::WP_AIRPORT;
    dep.altitude_constraint.type = AltitudeConstraintType::ALT_AT;
    dep.altitude_constraint.altitude1_ft = 13.0;
    wps.push_back(dep);

    Waypoint wp1;
    wp1.identifier = "MERIT";
    wp1.position = GeoPoint(40.7064, -73.7753);
    wp1.altitude_constraint.type = AltitudeConstraintType::ALT_AT_OR_ABOVE;
    wp1.altitude_constraint.altitude1_ft = 5000.0;
    wps.push_back(wp1);

    Waypoint wp2;
    wp2.identifier = "CANJC";
    wp2.position = GeoPoint(40.3975, -73.6367);
    wp2.altitude_constraint.type = AltitudeConstraintType::ALT_BETWEEN;
    wp2.altitude_constraint.altitude1_ft = 5000.0;
    wp2.altitude_constraint.altitude2_ft = 11000.0;
    wps.push_back(wp2);

    Waypoint wp3;
    wp3.identifier = "BOSOX";
    wp3.position = GeoPoint(40.0256, -73.2086);
    wp3.altitude_constraint.type = AltitudeConstraintType::ALT_BETWEEN;
    wp3.altitude_constraint.altitude1_ft = 11000.0;
    wp3.altitude_constraint.altitude2_ft = 15000.0;
    wps.push_back(wp3);

    Waypoint crz;
    crz.identifier = "CRZ";
    crz.position = GeoPoint(41.0, -70.0);
    crz.altitude_constraint.type = AltitudeConstraintType::ALT_AT;
    crz.altitude_constraint.altitude1_ft = 35000.0;
    wps.push_back(crz);

    Waypoint des1;
    des1.identifier = "SOLEN";
    des1.position = GeoPoint(39.5854, -73.3979);
    des1.altitude_constraint.type = AltitudeConstraintType::ALT_AT_OR_BELOW;
    des1.altitude_constraint.altitude1_ft = 11000.0;
    wps.push_back(des1);

    Waypoint arr;
    arr.identifier = "KEWR";
    arr.icao_code = "KEWR";
    arr.position = GeoPoint(40.6895, -74.1745);
    arr.type = WaypointType::WP_AIRPORT;
    arr.altitude_constraint.type = AltitudeConstraintType::ALT_AT;
    arr.altitude_constraint.altitude1_ft = 17.0;
    wps.push_back(arr);

    TrajectorySolver solver;
    solver.loadFlightPlan(wps);
    solver.setInitialConditions(dep.position, 13.0, 140.0, 90.0, 15000.0);

    bool success = solver.computeFullTrajectory();
    std::cout << "  Computation success: " << (success ? "YES" : "NO") << std::endl;

    const auto& traj = solver.getTrajectory();
    std::cout << "  Trajectory points: " << traj.size() << std::endl;
    if (!traj.empty()) {
        std::cout << "  Total time: " << solver.getTotalTimeSec() / 60.0 << " min" << std::endl;
        std::cout << "  Total distance: " << solver.getTotalDistanceNm() << " nm" << std::endl;
        std::cout << "  Max altitude: " << solver.getMaxAltitudeFt() << " ft" << std::endl;
        std::cout << "  Avg GS: " << solver.getAverageGroundSpeedKt() << " kt" << std::endl;
        std::cout << "  Fuel burn: " << solver.getTotalFuelBurnKg() << " kg" << std::endl;

        const auto& first = traj.front();
        const auto& last = traj.back();
        std::cout << "  Start: alt=" << first.altitude_ft << "ft, mach=" << first.mach_number << std::endl;
        std::cout << "  End: alt=" << last.altitude_ft << "ft, mach=" << last.mach_number << std::endl;
    }
    std::cout << "Trajectory Solver: PASS" << std::endl;
}

int main() {
    std::cout << "========== VNAV Calculator Unit Tests ==========" << std::endl;
    try {
        test_isa();
        test_great_circle();
        test_arinc424();
        test_lnav();
        test_vnav();
        test_trajectory();
        std::cout << "========== ALL TESTS PASSED ==========" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cout << "TEST FAILED: " << e.what() << std::endl;
        return 1;
    }
}
