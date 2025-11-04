#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import CtrlCmd

# ---------------- Params & pubs ----------------
cmd_pub = None
debug_wp_pub = None

wheelbase_L = 0.0
lookahead_ = 0.0
free_thresh_ = 0.0
roi_front_ = 0.0
roi_half_width_ = 0.0
bias_to_heading_ = 0.0

cruise_kmh_ = 0.0
avoid_kmh_ = 0.0
obstacle_dist_thresh_ = 0.0

steer_left_limit_rad_ = 0.0
steer_right_limit_rad_ = 0.0
steer_invert_ = False

min_gap_width_m_ = 0.0  # 최소 통과 폭(m)

scan_topic_ = "/scan"
publish_debug_wp_ = True

# 15Hz throttle timestamp
_last_pub = None

# --------------- Small helpers ----------------
def clampd(v, lo, hi):
    return max(lo, min(v, hi))

def isFree(r):
    return (r > 0.0) and math.isfinite(r) and (r > free_thresh_)

# 라디안 조향각 → [-1..+1] 정규화 (0=직진, +1=좌, -1=우)
def rad_to_norm(delta_rad):
    L = steer_left_limit_rad_
    R = steer_right_limit_rad_
    lo = min(L, R)
    hi = max(L, R)
    d  = clampd(delta_rad, lo, hi)

    denom = (L - R)
    A = 0.0 if abs(denom) < 1e-6 else (2.0 / denom)
    B = 1.0 - A * L
    n = A * d + B
    n = clampd(n, -1.0, 1.0)
    if steer_invert_:
        n = -n
    return n

def estimateGapRange(s, a, b):
    mid = (a + b) // 2
    Lidx = max(a, mid - 2)
    Ridx = min(b, mid + 2)
    acc = 0.0
    cnt = 0
    for i in range(Lidx, Ridx + 1):
        if s[i] > 0.0:
            acc += s[i]
            cnt += 1
    return (acc / cnt) if cnt > 0 else free_thresh_

def publishDebugWP(x, y):
    if not publish_debug_wp_:
        return
    p = PoseStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = "base_link"
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.w = 1.0
    debug_wp_pub.publish(p)

# ------------------- Core ---------------------
def scanCb(msg):
    global _last_pub
    # 15 Hz throttle
    now = rospy.Time.now()
    if _last_pub is not None and (now - _last_pub).to_sec() < (1.0/15.0):
        return
    _last_pub = now

    N = len(msg.ranges)
    if N == 0:
        return

    # 1) copy + clean + light smoothing
    r = list(msg.ranges)
    for i in range(N):
        if not math.isfinite(r[i]):
            r[i] = msg.range_max
        r[i] = float(clampd(r[i], msg.range_min, msg.range_max))

    s = list(r)
    for i in range(1, N-1):
        s[i] = (r[i-1] + r[i] + r[i+1]) / 3.0
    s[0] = r[0]
    s[-1] = r[-1]

    # 2) rectangular ROI in base_link: 0<x<=roi_front, |y|<=roi_half_width
    roi_valid_cnt = 0
    for i in range(N):
        th = msg.angle_min + i * msg.angle_increment
        x = s[i] * math.cos(th)
        y = s[i] * math.sin(th)
        if (x > 0.0) and (x <= roi_front_) and (abs(y) <= roi_half_width_):
            roi_valid_cnt += 1
        else:
            s[i] = 0.0
    rospy.logdebug_throttle(0.5, "[gap_nav] ROI valid beams: %d", roi_valid_cnt)

    # quick occupancy check in ROI (for speed mode)
    any_obstacle_near = False
    near_cap = min(msg.range_max*0.98, obstacle_dist_thresh_)
    for i in range(N):
        if s[i] > 0.0 and s[i] < near_cap:
            any_obstacle_near = True
            break

    # 장애물 감지 상태 전이 로그
    if not hasattr(scanCb, "prev_obstacle"):
        scanCb.prev_obstacle = False
    if any_obstacle_near and not scanCb.prev_obstacle:
        rospy.logwarn("[gap_nav] obj detected ! (%.2fm near) -> AVOID mode", obstacle_dist_thresh_)
    elif (not any_obstacle_near) and scanCb.prev_obstacle:
        rospy.loginfo("[gap_nav] obj clear. return to cruise.")
    scanCb.prev_obstacle = any_obstacle_near

    # 장애물 없으면: 크루즈 모드(직진 0.0)로 즉시 복귀하고 종료
    if not any_obstacle_near:
        cmd = CtrlCmd()
        cmd.longlCmdType = 2
        cmd.steering = 0.0          # 0=직진, +1=좌, -1=우
        cmd.velocity = cruise_kmh_  # (필요시 m/s면 /3.6)
        cmd.accel = 0.0
        cmd.brake = 0.0
        cmd_pub.publish(cmd)

        rospy.loginfo_throttle(0.5, "[gap_nav] CRUISE steer=0.00 v=%.1f km/h (no obstacle in ROI)", cmd.velocity)
        return

    # 3) bubble around closest obstacle (inflate)
    min_idx = -1
    min_r = float("inf")
    for i in range(N):
        if s[i] > 0.0 and s[i] < min_r:
            min_r = s[i]
            min_idx = i
    if min_idx >= 0 and math.isfinite(min_r):
        # cap 12deg, constant 0.5
        ang_pad = min(math.radians(12.0),
                      2.0 * math.asin(min(1.0, 0.5 / (2.0 * max(0.1, float(min_r))))))
        pad = int(math.ceil(ang_pad / msg.angle_increment))
        for k in range(-pad, pad+1):
            j = min_idx + k
            if 0 <= j < N:
                s[j] = 0.0

    # 4) find gaps (continuous free beams)
    class Gap:
        __slots__ = ("a","b","score")
        def __init__(self,a,b,score): self.a=a; self.b=b; self.score=score

    gaps = []
    i = 0
    while i < N:
        while i < N and not isFree(s[i]):
            i += 1
        if i >= N:
            break
        a = i
        while i < N and isFree(s[i]):
            i += 1
        b = i - 1
        if b >= a:
            mid = (a + b) // 2
            d = s[mid] if s[mid] > 0.0 else estimateGapRange(s, a, b)

            # 개선된 폭 계산: 양 끝 각도의 tan 차이 이용
            th_a = msg.angle_min + a * msg.angle_increment
            th_b = msg.angle_min + b * msg.angle_increment
            gap_width = d * abs(math.tan(th_b) - math.tan(th_a))

            ang_span = (b - a + 1) * msg.angle_increment
            ang_center = msg.angle_min + ((a + b) / 2.0) * msg.angle_increment
            heading_bias = (1.0 - bias_to_heading_ * min(1.0, abs(ang_center) / (math.pi / 3.0)))
            score = ang_span * heading_bias * (0.5 + 0.5 * (d / max(0.1, lookahead_)))

            rospy.logdebug_throttle(
                0.3, "[gap_nav] gap a=%d b=%d span=%.1fdeg d=%.2f w=%.2f",
                a, b, math.degrees(ang_span), d, gap_width
            )

            if gap_width >= min_gap_width_m_:
                gaps.append(Gap(a,b,score))

    # 선택/폴백
    if not gaps:
        # cmd = CtrlCmd()
        # cmd.longlCmdType = 2
        # cmd.steering = 0.0
        # cmd.velocity = 0.0
        # cmd.accel = 0.0
        # cmd.brake = 0.0
        # cmd_pub.publish(cmd)
        # rospy.logwarn_throttle(0.5, "[gap_nav] no usable gap -> STOP")
        cmd = CtrlCmd()
        cmd.longlCmdType = 2
        cmd.steering = 0.0
        cmd.velocity = 2.0
        cmd.accel = 0.0
        cmd.brake = 0.0
        cmd_pub.publish(cmd)
        rospy.logwarn_throttle(0.5, "[gap_nav] no usable gap -> STOP / SLOW TUNED V=2.0 km/h")
        return
    else:
        best_gap = max(gaps, key=lambda g: g.score)
        mid = (best_gap.a + best_gap.b) // 2
        th = msg.angle_min + mid * msg.angle_increment
        dm = s[mid] if (0 <= mid < N and s[mid] > 0.0) else lookahead_
        d  = min(dm, lookahead_)

    # 5) convert target (d, th) to steering
    x = d * math.cos(th)
    y = d * math.sin(th)
    delta = math.atan2(2.0 * wheelbase_L * math.sin(th), lookahead_)  # rad

    steer_norm = rad_to_norm(delta)

    # (optional) 디버그 웨이포인트
    publishDebugWP(x, y)

    # 6) publish MORAI CtrlCmd
    cmd = CtrlCmd()
    cmd.longlCmdType = 2
    cmd.steering = steer_norm      # -1..+1
    cmd.velocity = avoid_kmh_      # 회피 중 속도(환경이 m/s면 /3.6)
    cmd.accel = 0.0
    cmd.brake = 0.0
    cmd_pub.publish(cmd)

    rospy.loginfo_throttle(
        0.5,
        "[gap_nav] AVOID th=%.1fdeg delta=%.3frad steer=%.2f v=%.1f km/h (x=%.2f,y=%.2f)",
        math.degrees(th), delta, steer_norm, cmd.velocity, x, y
    )

def main():
    global cmd_pub, debug_wp_pub
    global wheelbase_L, lookahead_, free_thresh_, roi_front_, roi_half_width_, bias_to_heading_
    global cruise_kmh_, avoid_kmh_, obstacle_dist_thresh_
    global steer_left_limit_rad_, steer_right_limit_rad_, steer_invert_, min_gap_width_m_
    global scan_topic_, publish_debug_wp_

    rospy.init_node("gap_navigator_morai_ctrl")

    # ---- params ----
    scan_topic_        = rospy.get_param("~scan_topic",        "/scan")
    wheelbase_L        = rospy.get_param("~wheelbase",         3.0)
    lookahead_         = rospy.get_param("~lookahead",         4.5)
    free_thresh_       = rospy.get_param("~free_thresh",       0.80)
    roi_front_         = rospy.get_param("~roi_front",         6.0)
    roi_half_width_    = rospy.get_param("~roi_half_width",    2.0)
    bias_to_heading_   = rospy.get_param("~bias_to_heading",   0.3)
    cruise_kmh_        = rospy.get_param("~cruise_kmh",        5.0)
    avoid_kmh_         = rospy.get_param("~avoid_kmh",         3.0)
    obstacle_dist_thresh_ = rospy.get_param("~obstacle_dist_thresh", 2.5)

    steer_left_limit_rad_  = rospy.get_param("~steer_left_limit_rad",  -0.523599)  # -30deg
    steer_right_limit_rad_ = rospy.get_param("~steer_right_limit_rad",  0.523599)  # +30deg
    steer_invert_          = rospy.get_param("~steer_invert",           False)

    min_gap_width_m_       = rospy.get_param("~min_gap_width_m",        0.4)

    publish_debug_wp_      = rospy.get_param("~publish_debug_wp",       True)

    cmd_pub      = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=2)
    if publish_debug_wp_:
        debug_wp_pub = rospy.Publisher("/avoid_waypoint", PoseStamped, queue_size=1)
    rospy.Subscriber(scan_topic_, LaserScan, scanCb, queue_size=1)

    # spin (callback에서 15Hz로 스로틀링)
    rospy.spin()

if __name__ == "__main__":
    main()
