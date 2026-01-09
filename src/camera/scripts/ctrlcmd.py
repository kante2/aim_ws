#!/usr/bin/env python3
import rospy
from camera.msg import traffic_light_detection
from morai_msgs.msg import CtrlCmd

class TLCtrlCmdController:
    def __init__(self):
        self.pub = rospy.Publisher("/ctrl_cmd_0", CtrlCmd, queue_size=10)
        rospy.Subscriber("/TrafficLight", traffic_light_detection, self.cb)

        # 파라미터(원하면 rosparam으로 바꿔도 됨)
        self.go_speed_kmh   = rospy.get_param("~go_speed_kmh", 8.0)   # 초록일 때 유지 속도
        self.slow_speed_kmh = rospy.get_param("~slow_speed_kmh", 3.0) # 감속 상태 속도(옵션)
        self.confirm_sec    = rospy.get_param("~confirm_sec", 0.2)    # elapsed가 이 이상일 때만 반영
        self.timeout_sec    = rospy.get_param("~timeout_sec", 0.5)    # 신호등 토픽 끊기면 정지

        self.last_msg_time = rospy.Time(0)

        # 기본 steering은 0으로(조향은 다른 노드가 한다면 여기서 건드리면 충돌 가능)
        self.steering_rad = 0.0

        # velocity control 모드로 고정 (문서: longlCmdType==2면 velocity[km/h] 사용) :contentReference[oaicite:2]{index=2}
        self.longlCmdType = 2

        self.rate = rospy.Rate(20)

    def publish_ctrl(self, target_kmh: float):
        cmd = CtrlCmd()
        cmd.longlCmdType = self.longlCmdType
        cmd.steering = self.steering_rad

        # velocity control 모드에서는 velocity만 주면 됨 (accel/brake는 0으로 두는 편이 안전)
        cmd.velocity = float(max(0.0, target_kmh))
        cmd.accel = 0.0
        cmd.brake = 0.0
        cmd.acceleration = 0.0

        self.pub.publish(cmd)

    def cb(self, msg: traffic_light_detection):
        self.last_msg_time = rospy.Time.now()

        # 안정화: 같은 상태가 일정 시간 지속될 때만 반영(원하면 제거 가능)
        if msg.state != "none" and msg.elapsed.to_sec() < self.confirm_sec:
            return

        # if문 제어 로직
        if msg.state in ["red", "yellow"]:
            self.publish_ctrl(0.0)  # 정지
        elif msg.state == "green":
            self.publish_ctrl(self.go_speed_kmh)  # 저속 주행
        else:
            # none이면 안전 정지(또는 유지로 바꿀 수 있음)
            self.publish_ctrl(0.0)

    def spin(self):
        while not rospy.is_shutdown():
            # 신호등 토픽 끊기면 안전 정지
            if (rospy.Time.now() - self.last_msg_time).to_sec() > self.timeout_sec:
                self.publish_ctrl(0.0)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("tl_ctrlcmd_controller")
    TLCtrlCmdController().spin()
