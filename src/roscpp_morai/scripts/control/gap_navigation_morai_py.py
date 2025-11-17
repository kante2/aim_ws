#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import numpy as np
import cv2

from sensor_msgs.msg import LaserScan
from morai_msgs.msg import CtrlCmd
from geometry_msgs.msg import PoseStamped

def clampd(v, lo, hi): return max(lo, min(v, hi))

class FanGapAvoid:
    """
    팬(부채꼴) ROI 기반 회피 + 갭 주행
      - ROI: |theta| <= roi_half_fov, r <= roi_front
      - 장애물: r < obstacle_range_thresh 인 빔들
      - 안전 확장: 차폭/여유를 각도로 팽창: pad = atan(safety / max(r,eps))
      - Free 각도 구간(ROI에서 blocked 각도 빼기) 계산
      - Free 구간이 하나: 그쪽 중앙 각도로 조향(한쪽 장애물 반대 방향)
      - Free 구간이 둘 이상: 가운데 갭이 충분히 넓으면(폭 >= min_gap_width_m) 중앙 통과
      - 갭 폭: lookahead 거리에서 d*(tan(th_b)-tan(th_a))
    """

    def __init__(self):
        rospy.init_node("fan_gap_avoid", anonymous=False)

        # --- Topics ---
        self.scan_topic   = rospy.get_param("~scan_topic", "/scan")
        self.ctrl_topic   = rospy.get_param("~ctrl_topic", "/ctrl_cmd")
        self.wp_topic     = rospy.get_param("~wp_topic", "/avoid_waypoint")

        # --- Vehicle / steering ---
        self.wheelbase    = rospy.get_param("~wheelbase", 3.0)
        self.lookahead    = rospy.get_param("~lookahead", 4.0)
        self.steer_left   = rospy.get_param("~steer_left_limit_rad",  -0.523599)  # -30°
        self.steer_right  = rospy.get_param("~steer_right_limit_rad",  0.523599)  # +30°
        self.steer_invert = rospy.get_param("~steer_invert", False)
        self.steer_gain   = rospy.get_param("~steer_gain", 1.0)

        # --- ROI (fan) ---
        self.roi_front    = rospy.get_param("~roi_front", 8.0)                  # m
        self.roi_half_fov = math.radians(rospy.get_param("~roi_half_fov_deg", 40.0))  # rad

        # --- Obstacle / gap ---
        self.obst_r_th    = rospy.get_param("~obstacle_range_thresh", 3.0)      # m (이 이내는 장애물로 본다)
        self.vehicle_half = rospy.get_param("~vehicle_half_width", 0.9)         # m
        self.lat_margin   = rospy.get_param("~lateral_margin", 0.2)             # m
        self.min_gap_m    = rospy.get_param("~min_gap_width_m", 0.8)            # m (lookahead에서 통과폭)

        # --- Speed ---
        self.cruise_kmh   = rospy.get_param("~cruise_kmh", 8.0)
        self.avoid_kmh    = rospy.get_param("~avoid_kmh", 3.0)

        # --- Viz ---
        self.show_window  = rospy.get_param("~show_window", True)
        self.img_scale    = rospy.get_param("~img_scale", 70.0)
        self.win_name     = rospy.get_param("~win_name", "FanGapAvoid")
        self.arc_depth_m  = rospy.get_param("~viz_arc_depth_m", 0.9*min(self.lookahead, self.roi_front))

        # pubs/subs
        self.cmd_pub = rospy.Publisher(self.ctrl_topic, CtrlCmd, queue_size=2)
        self.wp_pub  = rospy.Publisher(self.wp_topic, PoseStamped, queue_size=1)
        self.sub     = rospy.Subscriber(self.scan_topic, LaserScan, self.cb, queue_size=1)

        # state
        self.last_pub = rospy.Time(0)
        self.max_hz   = 15.0

        if self.show_window:
            try: cv2.startWindowThread()
            except: pass
            cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.win_name, 1000, 560)

        rospy.loginfo("[fan_gap_avoid] ROI fan: +-%.1f deg, front=%.1f m",
                      math.degrees(self.roi_half_fov), self.roi_front)

    # ---------- math / utils ----------
    def _rad_to_norm(self, delta):
        L, R = self.steer_left, self.steer_right
        d = clampd(delta, min(L, R), max(L, R))
        A = 0.0 if abs(L-R) < 1e-6 else (2.0/(L-R))
        B = 1.0 - A*L
        n = clampd(A*d + B, -1.0, 1.0)
        if self.steer_invert: n = -n
        n = clampd(self.steer_gain * n, -1.0, 1.0)
        return n

    def _pp_delta_from_angle(self, th):
        # lookahead 방향을 th로 두면, 표준 PP는 delta ≈ atan2(2L*sin(th), lookahead)
        return math.atan2(2.0*self.wheelbase*math.sin(th), self.lookahead)

    def _publish_cmd(self, steer_norm, kmh):
        cmd = CtrlCmd()
        cmd.longlCmdType = 2
        cmd.steering = float(steer_norm)
        cmd.velocity = float(kmh)
        cmd.accel = 0.0; cmd.brake = 0.0
        self.cmd_pub.publish(cmd)

    def _publish_wp(self, x, y):
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "base_link"
        p.pose.position.x = x; p.pose.position.y = y
        p.pose.orientation.w = 1.0
        self.wp_pub.publish(p)

    # ---------- fan ROI / intervals ----------
    def _fan_mask_indices(self, msg):
        """ROI fan에 들어오는 인덱스들 반환"""
        idx = []
        th = msg.angle_min
        for i, rr in enumerate(msg.ranges):
            th = msg.angle_min + i*msg.angle_increment
            if abs(th) <= self.roi_half_fov:
                r = rr if math.isfinite(rr) else msg.range_max
                if r <= self.roi_front:
                    idx.append(i)
        return idx

    def _blocked_angle_intervals(self, msg, idx_list):
        """장애물 각도 구간(확장 포함) 리스트 [[th_a, th_b], ...] (정렬/병합 포함)"""
        safety = self.vehicle_half + self.lat_margin
        intervals = []
        for i in idx_list:
            r = msg.ranges[i]
            if not math.isfinite(r): r = msg.range_max
            r = clampd(r, msg.range_min, msg.range_max)

            th = msg.angle_min + i*msg.angle_increment
            if r < self.obst_r_th:  # 장애물로 판정
                # 각도 패딩으로 차폭 보정: pad = atan(safety / r)
                pad = math.atan2(safety, max(0.1, r))
                a = max(-self.roi_half_fov, th - pad)
                b = min(+self.roi_half_fov, th + pad)
                if a <= b:
                    intervals.append([a, b])

        if not intervals:
            return []

        intervals.sort(key=lambda ab: ab[0])
        merged = [intervals[0]]
        for a, b in intervals[1:]:
            if a <= merged[-1][1] + 1e-6:
                merged[-1][1] = max(merged[-1][1], b)
            else:
                merged.append([a, b])
        return merged

    def _free_angle_intervals(self, blocked):
        """ROI 각도[-h, +h]에서 blocked를 뺀 free 구간"""
        h = self.roi_half_fov
        if not blocked:
            return [[-h, h]]
        frees = []
        cur = -h
        for a, b in blocked:
            a = clampd(a, -h, h); b = clampd(b, -h, h)
            if a > cur:
                frees.append([cur, a])
            cur = max(cur, b)
        if cur < h: frees.append([cur, h])
        return frees

    def _gap_width_at_lookahead(self, th_a, th_b):
        """lookahead 에서의 갭 폭: d*(tan(b)-tan(a))"""
        d = self.lookahead
        return d * abs(math.tan(th_b) - math.tan(th_a))

    # ---------- visualization ----------
    def _to_px(self, x, y, Wpx, Hpx, s):
        u = int(x*s); v = int((self.roi_front*0 + self.roi_front) * 0)  # dummy
        # 화면: x 전방→오른쪽, y 좌측→위쪽
        u = int(clampd(u, 0, Wpx-1))
        v = int((-y + self.roi_front*0) * 0)  # not used (우린 각도뷰 위주)
        return u, v

    def _draw_fan_view(self, msg, blocked, free, target_th):
        if not self.show_window: return
        # 극좌표식 팬 디스플레이 (각도 중심)
        Wpx = 1000; Hpx = 560
        img = np.zeros((Hpx, Wpx, 3), dtype=np.uint8)

        cx, cy = 120, Hpx//2
        R = int(self.img_scale * min(self.roi_front, 8.0))

        # ROI fan 테두리
        a0 = int(-math.degrees(self.roi_half_fov)); a1 = int(math.degrees(self.roi_half_fov))
        cv2.ellipse(img, (cx, cy), (R, R), 0, a0, a1, (120,120,120), 1)

        # blocked(빨강), free(초록) 부채꼴 칠하기
        def fill_sector(a_deg, b_deg, color, alpha=0.25):
            overlay = img.copy()
            # 扇形 polygon
            pts = [(cx,cy)]
            for ang in np.linspace(a_deg, b_deg, num=50):
                rad = math.radians(ang)
                x = cx + int(R*math.cos(rad))
                y = cy - int(R*math.sin(rad))
                pts.append((x,y))
            pts = np.array(pts, dtype=np.int32)
            cv2.fillPoly(overlay, [pts], color)
            cv2.addWeighted(overlay, alpha, img, 1-alpha, 0, img)

        for a,b in blocked:
            fill_sector(math.degrees(a), math.degrees(b), (0,0,200), 0.35)
        for a,b in free:
            fill_sector(math.degrees(a), math.degrees(b), (60,200,60), 0.18)

        # 레이 포인트 조금 찍기(간격샘플)
        step = max(1, int(math.radians(2.0)/msg.angle_increment))
        for i in range(0, len(msg.ranges), step):
            th = msg.angle_min + i*msg.angle_increment
            if abs(th) > self.roi_half_fov: continue
            r = msg.ranges[i]
            if not math.isfinite(r): continue
            r = min(r, self.roi_front)
            x = cx + int((R*r/self.roi_front) * math.cos(th))
            y = cy - int((R*r/self.roi_front) * math.sin(th))
            color = (120,240,120) if r >= self.obst_r_th else (0,220,220)
            cv2.circle(img, (x,y), 2, color, -1)

        # 타겟 각도 표시
        if target_th is not None:
            x = cx + int(R * math.cos(target_th))
            y = cy - int(R * math.sin(target_th))
            cv2.circle(img, (x,y), 6, (0,255,255), -1)
            cv2.line(img, (cx,cy), (x,y), (255,255,0), 1)
            cv2.putText(img, f"target {math.degrees(target_th):.1f} deg",
                        (cx+R+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230,230,230), 1, cv2.LINE_AA)

        # 범례
        cv2.putText(img, "ROI fan", (20,28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1, cv2.LINE_AA)
        cv2.rectangle(img, (20,45), (40,60), (0,0,200), -1);  cv2.putText(img,"blocked",(50,58),cv2.FONT_HERSHEY_SIMPLEX,0.5,(220,220,220),1)
        cv2.rectangle(img, (20,70), (40,85), (60,200,60), -1);cv2.putText(img,"free",(50,83),cv2.FONT_HERSHEY_SIMPLEX,0.5,(220,220,220),1)
        cv2.circle(img, (30,110), 6, (0,255,255), -1);       cv2.putText(img,"target",(50,115),cv2.FONT_HERSHEY_SIMPLEX,0.5,(220,220,220),1)

        cv2.imshow(self.win_name, img); cv2.waitKey(1)

    # ---------- main ----------
    def cb(self, msg: LaserScan):
        # rate limit
        now = rospy.Time.now()
        if self.last_pub != rospy.Time(0) and (now - self.last_pub).to_sec() < (1.0/self.max_hz):
            return
        self.last_pub = now

        # 1) 팬 ROI 내 인덱스
        roi_idx = self._fan_mask_indices(msg)

        # 2) 장애물 각도구간(차폭 보정으로 확장) → 병합
        blocked = self._blocked_angle_intervals(msg, roi_idx)

        # 3) Free 각도 구간
        free = self._free_angle_intervals(blocked)

        target_th = None

        if not free:
            # 완전 막힘 → 감속
            self._publish_cmd(steer_norm=0.0, kmh=1.0)
            self._draw_fan_view(msg, blocked, free, target_th)
            rospy.logwarn_throttle(0.5, "[fan_gap_avoid] no free sector -> slow")
            return

        # 4) 가운데(0 rad)와 가장 가까운 free 구간을 우선
        #    + 갭 폭 평가(lookahead 기준)
        def gap_score(iv):
            a,b = iv
            width = self._gap_width_at_lookahead(a,b)   # m
            center_bias = -abs((a+b)/2.0)               # 중앙 선호
            span = b-a
            return (width >= self.min_gap_m, width, center_bias, span)

        # (1) 임계폭 이상인 free 구간 후보만 추림 → 없으면 전체에서 best
        ok = [iv for iv in free if self._gap_width_at_lookahead(iv[0], iv[1]) >= self.min_gap_m]
        if ok:
            best = max(ok, key=gap_score)
        else:
            best = max(free, key=gap_score)

        th_a, th_b = best
        target_th = (th_a + th_b)/2.0

        # 5) 조향 및 속도
        delta = self._pp_delta_from_angle(target_th)
        steer = self._rad_to_norm(delta)
        self._publish_cmd(steer_norm=steer, kmh=self.avoid_kmh)

        # 6) 목표점 퍼블리시 (lookahead 지점)
        x_goal = self.lookahead*math.cos(target_th)
        y_goal = self.lookahead*math.sin(target_th)
        self._publish_wp(x_goal, y_goal)

        # 7) 시각화 (팬 ROI + blocked/free + target)
        self._draw_fan_view(msg, blocked, free, target_th)

    def spin(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    node = FanGapAvoid()
    node.spin()
