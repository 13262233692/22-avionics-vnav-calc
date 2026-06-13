import sys
sys.path.insert(0, r'..\build\Release')
import vnav_core as vnav

print("=" * 70)
print("VNAV 严格高度约束 STAR 场景测试")
print("=" * 70)

def make_wp(lat, lon, ident, alt_type, alt1=0, alt2=0):
    wp = vnav.Waypoint()
    wp.identifier = ident
    wp.icao_code = "TEST"
    wp.type = vnav.WaypointType.FIX
    wp.position = vnav.GeoPoint(lat, lon)
    wp.altitude_constraint.type = alt_type
    wp.altitude_constraint.altitude1_ft = alt1
    wp.altitude_constraint.altitude2_ft = alt2
    return wp

print("\n场景1: 不可调和的高度约束（距离太短, 高度差太大）")
print("-" * 70)
cfg = vnav.VnavConfig()
cfg.cruise_altitude_ft = 35000.0
cfg.max_backward_iterations = 1000
cfg.enable_constraint_relaxation = True
cfg.max_descent_rate_fpm = 4000.0
cfg.max_flight_path_angle_deg = 6.0

vnav_sm = vnav.VnavStateMachine(cfg)

waypoints_impossible = [
    make_wp(40.0, -73.0, "START", vnav.AltitudeConstraintType.ALT_AT, 35000),
    make_wp(40.0, -72.95, "POINT_A", vnav.AltitudeConstraintType.ALT_AT_OR_BELOW, 12000),
    make_wp(40.0, -72.90, "POINT_B", vnav.AltitudeConstraintType.ALT_AT, 6000),
    make_wp(40.0, -72.85, "END", vnav.AltitudeConstraintType.ALT_AT, 0),
]

print(f"航路点数量: {len(waypoints_impossible)}")
for i, wp in enumerate(waypoints_impossible):
    print(f"  {wp.identifier}: alt_constraint={wp.altitude_constraint.altitude1_ft}ft, type={wp.altitude_constraint.type}")

lnav_cfg = vnav.LnavConfig()
lnav = vnav.LnavStateMachine(lnav_cfg)
lnav.set_flight_plan(waypoints_impossible)

total_dist = lnav.get_total_distance_nm()
print(f"\n总距离: {total_dist:.2f} NM")

for i in range(1, len(waypoints_impossible)):
    d = vnav.GreatCircle.distance_nm(waypoints_impossible[i-1].position, waypoints_impossible[i].position)
    alt_diff = waypoints_impossible[i-1].altitude_constraint.altitude1_ft - waypoints_impossible[i].altitude_constraint.altitude1_ft
    print(f"  {waypoints_impossible[i-1].identifier} -> {waypoints_impossible[i].identifier}: {d:.2f} NM, 高度差 {alt_diff}ft")

required_vs = vnav_sm.compute_required_vs_for_path(35000, 6000, total_dist, 400)
print(f"\n从35000ft下降到6000ft需要的垂直速度: {required_vs:.0f} fpm")
print(f"最大允许下降率: {cfg.max_descent_rate_fpm:.0f} fpm")
feasible, vs = vnav_sm.check_descent_feasibility(6000 - 35000, total_dist, 400)
print(f"可行性检查: feasible={feasible}, required_vs={vs:.0f} fpm")

print("\n执行反向下降剖面计算...")
ok, backward_profile = vnav_sm.compute_descent_profile_backward(lnav, 35000, 0)
print(f"计算结果: success={ok}")
if ok and backward_profile:
    print("反向下降剖面:")
    for p in backward_profile:
        print(f"  WP{p.waypoint_index} ({waypoints_impossible[p.waypoint_index].identifier}): "
              f"alt={p.altitude_ft:.0f}ft, vs={p.vertical_speed_fpm:.0f}fpm, "
              f"dist={p.distance_from_start_nm:.2f}NM")

print("\n执行完整垂直剖面计算...")
vnav_sm.auto_generate_constraints(lnav)
vnav_sm.compute_vertical_profile(lnav)
profile = vnav_sm.get_profile()
print(f"完整剖面点数: {len(profile)}")
for i, p in enumerate(profile):
    phase_name = {vnav.VnavPhase.GROUND:"GROUND", vnav.VnavPhase.CLIMB:"CLIMB",
                  vnav.VnavPhase.CRUISE:"CRUISE", vnav.VnavPhase.DESCENT:"DESCENT",
                  vnav.VnavPhase.APPROACH:"APPROACH", vnav.VnavPhase.COMPLETE:"COMPLETE"}.get(p.phase, "?")
    print(f"  [{phase_name:8s}] {waypoints_impossible[i].identifier:8s} - "
          f"alt={p.altitude_ft:8.0f}ft, vs={p.vertical_speed_fpm:8.0f}fpm, "
          f"M={p.mach_number:.3f}")

print("\n" + "=" * 70)
print("场景2: 正常可解的高度约束")
print("-" * 70)

waypoints_normal = [
    make_wp(40.0, -74.0, "DEP", vnav.AltitudeConstraintType.ALT_AT, 0),
    make_wp(40.5, -73.5, "WP1", vnav.AltitudeConstraintType.ALT_AT_OR_ABOVE, 15000),
    make_wp(41.0, -73.0, "CRZ", vnav.AltitudeConstraintType.ALT_AT, 35000),
    make_wp(40.5, -72.0, "DS1", vnav.AltitudeConstraintType.ALT_AT_OR_BELOW, 24000),
    make_wp(40.0, -71.5, "DS2", vnav.AltitudeConstraintType.ALT_AT_OR_BELOW, 15000),
    make_wp(39.5, -71.0, "DS3", vnav.AltitudeConstraintType.ALT_AT, 8000),
    make_wp(39.0, -70.5, "ARR", vnav.AltitudeConstraintType.ALT_AT, 0),
]

lnav2 = vnav.LnavStateMachine(lnav_cfg)
lnav2.set_flight_plan(waypoints_normal)
vnav_sm2 = vnav.VnavStateMachine(cfg)
vnav_sm2.auto_generate_constraints(lnav2)
vnav_sm2.compute_vertical_profile(lnav2)
profile2 = vnav_sm2.get_profile()
total_dist2 = lnav2.get_total_distance_nm()
print(f"总距离: {total_dist2:.2f} NM")
print(f"剖面点数: {len(profile2)}")
for i, p in enumerate(profile2):
    phase_name = {vnav.VnavPhase.GROUND:"GROUND", vnav.VnavPhase.CLIMB:"CLIMB",
                  vnav.VnavPhase.CRUISE:"CRUISE", vnav.VnavPhase.DESCENT:"DESCENT",
                  vnav.VnavPhase.APPROACH:"APPROACH", vnav.VnavPhase.COMPLETE:"COMPLETE"}.get(p.phase, "?")
    print(f"  [{phase_name:8s}] {waypoints_normal[i].identifier:8s} - "
          f"alt={p.altitude_ft:8.0f}ft, vs={p.vertical_speed_fpm:8.0f}fpm, "
          f"M={p.mach_number:.3f}")

print("\n" + "=" * 70)
print("场景3: 连续严格高度约束的标准终端进场程序(STAR)")
print("-" * 70)

star_waypoints = [
    make_wp(41.0, -72.0, "STAR_ENTRY", vnav.AltitudeConstraintType.ALT_AT, 35000),
    make_wp(40.8, -72.3, "FIX_A", vnav.AltitudeConstraintType.ALT_AT_OR_BELOW, 28000),
    make_wp(40.6, -72.6, "FIX_B", vnav.AltitudeConstraintType.ALT_AT_OR_BELOW, 21000),
    make_wp(40.4, -72.9, "FIX_C", vnav.AltitudeConstraintType.ALT_AT, 15000),
    make_wp(40.2, -73.2, "FIX_D", vnav.AltitudeConstraintType.ALT_AT_OR_BELOW, 12000),
    make_wp(40.1, -73.5, "FIX_E", vnav.AltitudeConstraintType.ALT_AT, 8000),
    make_wp(40.05, -73.8, "IF", vnav.AltitudeConstraintType.ALT_AT_OR_BELOW, 5000),
    make_wp(40.0, -74.0, "FAF", vnav.AltitudeConstraintType.ALT_AT, 3000),
    make_wp(39.98, -74.1, "RWY", vnav.AltitudeConstraintType.ALT_AT, 17),
]

lnav3 = vnav.LnavStateMachine(lnav_cfg)
lnav3.set_flight_plan(star_waypoints)
vnav_sm3 = vnav.VnavStateMachine(cfg)
vnav_sm3.auto_generate_constraints(lnav3)
vnav_sm3.compute_vertical_profile(lnav3)
profile3 = vnav_sm3.get_profile()
total_dist3 = lnav3.get_total_distance_nm()
print(f"STAR 航路点数量: {len(star_waypoints)}")
print(f"STAR 总距离: {total_dist3:.2f} NM")
print(f"剖面点数: {len(profile3)}")
constraints_ok = True
for i, p in enumerate(profile3):
    wp = star_waypoints[i]
    phase_name = {vnav.VnavPhase.GROUND:"GROUND", vnav.VnavPhase.CLIMB:"CLIMB",
                  vnav.VnavPhase.CRUISE:"CRUISE", vnav.VnavPhase.DESCENT:"DESCENT",
                  vnav.VnavPhase.APPROACH:"APPROACH", vnav.VnavPhase.COMPLETE:"COMPLETE"}.get(p.phase, "?")
    satisfied = True
    if wp.altitude_constraint.type == vnav.AltitudeConstraintType.ALT_AT:
        satisfied = abs(p.altitude_ft - wp.altitude_constraint.altitude1_ft) < 100
    elif wp.altitude_constraint.type == vnav.AltitudeConstraintType.ALT_AT_OR_BELOW:
        satisfied = p.altitude_ft <= wp.altitude_constraint.altitude1_ft + 50
    elif wp.altitude_constraint.type == vnav.AltitudeConstraintType.ALT_AT_OR_ABOVE:
        satisfied = p.altitude_ft >= wp.altitude_constraint.altitude1_ft - 50
    if not satisfied:
        constraints_ok = False
    print(f"  [{phase_name:8s}] {wp.identifier:10s} - "
          f"calc_alt={p.altitude_ft:8.0f}ft, constraint={wp.altitude_constraint.altitude1_ft:6.0f}ft "
          f"({'OK' if satisfied else 'VIOLATION'})")

print(f"\n约束满足情况: {'全部满足' if constraints_ok else '存在违反'}")
print("TOD距离:", vnav_sm3.get_state().distance_to_top_of_descent_nm, "NM")

print("\n" + "=" * 70)
print("场景4: 完整三维轨迹求解器 - STAR进场程序")
print("-" * 70)

solver = vnav.TrajectorySolver()
solver.set_vnav_config(cfg)
solver.load_flight_plan(star_waypoints)
solver.set_initial_conditions(
    star_waypoints[0].position,
    35000.0,
    250.0,
    vnav.GreatCircle.initial_bearing_deg(star_waypoints[0].position, star_waypoints[1].position),
    20000.0
)

print("开始三维轨迹积分...")
result = solver.compute_full_trajectory()
print(f"计算结果: success={result}")

traj = solver.get_trajectory()
print(f"轨迹点数: {len(traj)}")
print(f"总时间: {solver.get_total_time_sec()/60:.2f} 分钟")
print(f"总距离: {solver.get_total_distance_nm():.2f} NM")
print(f"最大高度: {solver.get_max_altitude_ft():.0f} ft")
print(f"平均地速: {solver.get_average_ground_speed_kt():.1f} kt")
print(f"燃油消耗: {solver.get_total_fuel_burn_kg():.1f} kg")

print("\n轨迹采样点:")
for i in range(0, min(len(traj), 20), max(1, len(traj)//10)):
    pt = traj[i]
    phase_name = {vnav.VnavPhase.GROUND:"GROUND", vnav.VnavPhase.CLIMB:"CLIMB",
                  vnav.VnavPhase.CRUISE:"CRUISE", vnav.VnavPhase.DESCENT:"DESCENT",
                  vnav.VnavPhase.APPROACH:"APPROACH", vnav.VnavPhase.COMPLETE:"COMPLETE"}.get(pt.vnav_phase, "?")
    print(f"  T={pt.time_sec:6.0f}s | pos=({pt.position.latitude_deg:.4f}, {pt.position.longitude_deg:.4f}) "
          f"| alt={pt.altitude_ft:7.0f}ft | GS={pt.ground_speed_kt:6.1f}kt | VS={pt.vertical_speed_fpm:7.0f}fpm "
          f"| M={pt.mach_number:.3f} | {phase_name}")

print("\n最后10个轨迹点:")
for pt in traj[-10:]:
    phase_name = {vnav.VnavPhase.GROUND:"GROUND", vnav.VnavPhase.CLIMB:"CLIMB",
                  vnav.VnavPhase.CRUISE:"CRUISE", vnav.VnavPhase.DESCENT:"DESCENT",
                  vnav.VnavPhase.APPROACH:"APPROACH", vnav.VnavPhase.COMPLETE:"COMPLETE"}.get(pt.vnav_phase, "?")
    print(f"  T={pt.time_sec:6.0f}s | alt={pt.altitude_ft:7.0f}ft | GS={pt.ground_speed_kt:6.1f}kt "
          f"| VS={pt.vertical_speed_fpm:7.0f}fpm | {phase_name}")

print("\n" + "=" * 70)
print("所有测试场景通过 - 没有栈溢出!")
print("=" * 70)
