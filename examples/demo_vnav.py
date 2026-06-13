import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build', 'Release'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build'))

import vnav_core as vnav

def print_separator(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}\n")

def test_isa():
    print_separator("国际标准大气模型 (ISA) 测试")

    sl = vnav.InternationalStandardAtmosphere.compute(0.0)
    print(f"海平面 (0m):")
    print(f"  温度: {sl.temperature:.2f} K ({sl.temperature - 273.15:.2f}°C)")
    print(f"  气压: {sl.pressure:.2f} Pa")
    print(f"  密度: {sl.density:.4f} kg/m^3")
    print(f"  音速: {sl.speed_of_sound:.2f} m/s")

    tropo = vnav.InternationalStandardAtmosphere.compute(11000.0)
    print(f"\n对流层顶 (11000m):")
    print(f"  温度: {tropo.temperature:.2f} K ({tropo.temperature - 273.15:.2f}°C)")
    print(f"  气压: {tropo.pressure:.2f} Pa")

    fl350_m = vnav.VnavStateMachine.feet_to_meters(35000.0)
    tas_ms = vnav.InternationalStandardAtmosphere.mach_to_tas(0.78, fl350_m)
    tas_kt = vnav.VnavStateMachine.mps_to_knots(tas_ms)
    print(f"\nFL350, M0.78:")
    print(f"  TAS: {tas_ms:.2f} m/s = {tas_kt:.2f} kt")

def test_great_circle():
    print_separator("大圆航线 (Great Circle) 计算测试")

    nyc = vnav.GeoPoint(40.6413, -73.7781)
    lon = vnav.GeoPoint(51.4700, -0.4543)

    dist_nm = vnav.GreatCircle.distance_nm(nyc, lon)
    dist_km = dist_nm * 1.852
    print(f"纽约肯尼迪 (KJFK) -> 伦敦希思罗 (EGLL):")
    print(f"  距离: {dist_nm:.2f} NM = {dist_km:.2f} km")

    bearing = vnav.GreatCircle.initial_bearing_deg(nyc, lon)
    print(f"  初始方位角: {bearing:.2f}°")

    final_bearing = vnav.GreatCircle.final_bearing_deg(nyc, lon)
    print(f"  最终方位角: {final_bearing:.2f}°")

    mid = vnav.GreatCircle.intermediate_point(nyc, lon, 0.5)
    print(f"  中点: ({mid.latitude_deg:.4f}°, {mid.longitude_deg:.4f}°)")

    point_nm = vnav.GreatCircle.point_at_distance_bearing(
        nyc, vnav.GreatCircle.NM_TO_M * 100.0, 90.0)
    print(f"\n从KJFK向正东100NM处:")
    print(f"  坐标: ({point_nm.latitude_deg:.4f}°, {point_nm.longitude_deg:.4f}°)")

def test_arinc424():
    print_separator("ARINC 424 导航数据库解析测试")

    data_path = os.path.join(os.path.dirname(__file__), '..', 'data', 'sample_airports.dat')
    parser = vnav.Arinc424Parser()

    if os.path.exists(data_path):
        print(f"解析文件: {data_path}")
        db = parser.parse_file(data_path)
    else:
        print("使用内置测试数据解析")
        test_data = (
            "SUSPA PA  N40382300W073464430E01300035TJNY  18000KJFK                      JOHN F KENNEDY INTL\n"
            "SUSWP PA  N40411560W074095930E01600035TJNY  18000KEWR                      NEWARK LIBERTY INTL\n"
            "SUKJFK DB  N40421970W073464230E00016001TJSJ        MERIT                     MERIT INTERSECTION\n"
        )
        db = parser.parse_string(test_data)

    errors = parser.get_parse_errors()
    if errors:
        print(f"解析错误 ({len(errors)}个):")
        for e in errors:
            print(f"  - {e}")

    print(f"\n解析结果统计:")
    print(f"  机场数量: {len(db.airports)}")
    for icao, apt in db.airports.items():
        print(f"    {icao}: {apt.name}")
        print(f"      位置: ({apt.position.latitude_deg:.4f}°, {apt.position.longitude_deg:.4f}°)")
        print(f"      标高: {apt.elevation_ft:.0f} ft")
        print(f"      SID数量: {len(apt.sids)}")
        print(f"      STAR数量: {len(apt.stars)}")

    print(f"\n  航路点数量: {len(db.waypoints)}")
    for key, wp in list(db.waypoints.items())[:5]:
        print(f"    {wp.identifier}: ({wp.position.latitude_deg:.4f}°, {wp.position.longitude_deg:.4f}°)")

def create_test_flight_plan():
    wps = []

    dep = vnav.Waypoint()
    dep.identifier = "KJFK"
    dep.icao_code = "KJFK"
    dep.type = vnav.WaypointType.AIRPORT
    dep.position = vnav.GeoPoint(40.6413, -73.7781)
    dep.altitude_constraint.type = vnav.AltitudeConstraintType.ALT_AT
    dep.altitude_constraint.altitude1_ft = 13.0
    wps.append(dep)

    wp1 = vnav.Waypoint()
    wp1.identifier = "MERIT"
    wp1.position = vnav.GeoPoint(40.7064, -73.7753)
    wp1.altitude_constraint.type = vnav.AltitudeConstraintType.ALT_AT_OR_ABOVE
    wp1.altitude_constraint.altitude1_ft = 5000.0
    wp1.speed_limit_kt = 250.0
    wps.append(wp1)

    wp2 = vnav.Waypoint()
    wp2.identifier = "CANJC"
    wp2.position = vnav.GeoPoint(40.3975, -73.6367)
    wp2.altitude_constraint.type = vnav.AltitudeConstraintType.ALT_BETWEEN
    wp2.altitude_constraint.altitude1_ft = 5000.0
    wp2.altitude_constraint.altitude2_ft = 11000.0
    wps.append(wp2)

    wp3 = vnav.Waypoint()
    wp3.identifier = "BOSOX"
    wp3.position = vnav.GeoPoint(40.0256, -73.2086)
    wp3.altitude_constraint.type = vnav.AltitudeConstraintType.ALT_BETWEEN
    wp3.altitude_constraint.altitude1_ft = 11000.0
    wp3.altitude_constraint.altitude2_ft = 15000.0
    wps.append(wp3)

    crz = vnav.Waypoint()
    crz.identifier = "CRZ"
    crz.position = vnav.GeoPoint(41.0, -70.0)
    crz.altitude_constraint.type = vnav.AltitudeConstraintType.ALT_AT
    crz.altitude_constraint.altitude1_ft = 35000.0
    wps.append(crz)

    des1 = vnav.Waypoint()
    des1.identifier = "WAVEY"
    des1.position = vnav.GeoPoint(39.2782, -74.5315)
    des1.altitude_constraint.type = vnav.AltitudeConstraintType.ALT_AT_OR_BELOW
    des1.altitude_constraint.altitude1_ft = 21000.0
    wps.append(des1)

    des2 = vnav.Waypoint()
    des2.identifier = "PARCH"
    des2.position = vnav.GeoPoint(39.3922, -74.2760)
    des2.altitude_constraint.type = vnav.AltitudeConstraintType.ALT_AT_OR_BELOW
    des2.altitude_constraint.altitude1_ft = 18000.0
    wps.append(des2)

    des3 = vnav.Waypoint()
    des3.identifier = "SOLEN"
    des3.position = vnav.GeoPoint(39.5854, -73.3979)
    des3.altitude_constraint.type = vnav.AltitudeConstraintType.ALT_AT_OR_BELOW
    des3.altitude_constraint.altitude1_ft = 11000.0
    wps.append(des3)

    arr = vnav.Waypoint()
    arr.identifier = "KEWR"
    arr.icao_code = "KEWR"
    arr.type = vnav.WaypointType.AIRPORT
    arr.position = vnav.GeoPoint(40.6895, -74.1745)
    arr.altitude_constraint.type = vnav.AltitudeConstraintType.ALT_AT
    arr.altitude_constraint.altitude1_ft = 17.0
    wps.append(arr)

    return wps

def test_trajectory_solver():
    print_separator("三维轨迹数值积分求解器测试")

    print("创建飞行计划: KJFK -> (航路点) -> KEWR")
    wps = create_test_flight_plan()
    print(f"  航路点数量: {len(wps)}")
    for i, wp in enumerate(wps):
        print(f"    {i+1}. {wp.identifier:8s} ({wp.position.latitude_deg:8.4f}°, {wp.position.longitude_deg:9.4f}°) "
              f"高度约束: {wp.altitude_constraint.altitude1_ft:.0f}ft")

    solver = vnav.TrajectorySolver()
    solver.load_flight_plan(wps)

    traj_config = vnav.TrajectoryConfig()
    traj_config.integration_dt_sec = 1.0
    traj_config.output_interval_sec = 10.0
    traj_config.isa_deviation_c = 0.0
    solver.set_config(traj_config)

    vnav_config = vnav.VnavConfig()
    vnav_config.cruise_altitude_ft = 35000.0
    vnav_config.climb_cas_kt = 280.0
    vnav_config.climb_mach = 0.78
    vnav_config.cruise_mach = 0.78
    vnav_config.descent_cas_kt = 300.0
    vnav_config.descent_mach = 0.78
    solver.set_vnav_config(vnav_config)

    solver.set_initial_conditions(
        wps[0].position,
        13.0,
        160.0,
        90.0,
        15000.0
    )

    print("\n开始轨迹积分计算...")
    success = solver.compute_full_trajectory()
    print(f"计算完成: {'成功' if success else '失败'}")

    traj = solver.get_trajectory()
    print(f"\n轨迹统计:")
    print(f"  轨迹点数: {len(traj)}")
    print(f"  总时间: {solver.get_total_time_sec() / 60.0:.2f} 分钟")
    print(f"  总距离: {solver.get_total_distance_nm():.2f} NM")
    print(f"  最大高度: {solver.get_max_altitude_ft():.0f} ft")
    print(f"  平均地速: {solver.get_average_ground_speed_kt():.1f} kt")
    print(f"  燃油消耗: {solver.get_total_fuel_burn_kg():.1f} kg")

    print("\n各阶段剖面采样:")
    print(f"  {'时间(s)':>8} {'纬度':>10} {'经度':>11} {'高度(ft)':>10} {'M数':>6} "
          f"{'CAS(kt)':>8} {'GS(kt)':>8} {'VS(fpm)':>10} {'航迹角':>7}")
    print(f"  {'-'*8} {'-'*10} {'-'*11} {'-'*10} {'-'*6} {'-'*8} {'-'*8} {'-'*10} {'-'*7}")

    sample_indices = [0, len(traj)//6, len(traj)//3, len(traj)//2,
                      2*len(traj)//3, 5*len(traj)//6, len(traj)-1]
    for idx in sample_indices:
        idx = max(0, min(idx, len(traj)-1))
        pt = traj[idx]
        phase_names = {0: '地面', 1: '爬升', 2: '巡航', 3: '下降', 4: '进近', 6: '完成'}
        vphase = phase_names.get(int(pt.vnav_phase), '?')
        print(f"  {pt.time_sec:8.1f} {pt.position.latitude_deg:10.4f} {pt.position.longitude_deg:11.4f} "
              f"{pt.altitude_ft:10.0f} {pt.mach_number:6.3f} {pt.cas_kt:8.1f} {pt.ground_speed_kt:8.1f} "
              f"{pt.vertical_speed_fpm:10.0f} {vphase:>4}")

    print("\n航路点通过情况:")
    wp_profiles = solver.get_waypoint_profiles()
    for i, pt in enumerate(wp_profiles):
        wp_id = wps[min(pt.current_wp_index, len(wps)-1)].identifier
        print(f"  {i+1}. {wp_id:8s} - T+{pt.time_sec/60:.1f}min, "
              f"{pt.altitude_ft:.0f}ft, M{pt.mach_number:.3f}, GS={pt.ground_speed_kt:.0f}kt")

def main():
    print("""
╔══════════════════════════════════════════════════════════════╗
║     VNAV 垂直导航计算器 - 符合ARINC 424规范的               ║
║            三维剖面数值积分求解器 Python示例                 ║
╚══════════════════════════════════════════════════════════════╝
    """)

    test_isa()
    test_great_circle()
    test_arinc424()
    test_trajectory_solver()

    print_separator("所有测试完成")

if __name__ == "__main__":
    main()
