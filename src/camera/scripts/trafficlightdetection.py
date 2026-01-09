#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from camera.msg import traffic_light_detection
from rospy import Duration, Time


# 전역변수 선언
topic_data = None

# 경과 시간 측정 관련
prev_state = None
start_time = None

# callback 함수 
def callback(msg):
    global topic_data
    topic_data = msg 
    # 받아온 토픽 메시지 데이터를 전역 변수 topic_data에 저장해야. 
    # 콜백함수의 매개변수인 msg가 수신한 토픽 데이터.


# 신호등 색상 검출 함수
def detection (topic_data):
    bridge = CvBridge()
    global prev_state, start_time

    bgr = bridge.compressed_imgmsg_to_cv2(topic_data, desired_encoding="bgr8")

    # roi 설정
    x = 300; 
    # y = 100; 
    w = 380; 
    h = 280
    roi = bgr[:h, x:x+w]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)


# HSV 범위 지정
    red1_lower = np.array([0, 50,50])
    red1_upper= np.array([10, 255,255])
    red2_lower = np.array([165, 50,50])
    red2_upper = np.array([180, 255,255])
    yellow_lower = np.array([15, 50, 50])
    # yellow_upper = np.array([40, 100, 100])
    yellow_upper = np.array([40, 255, 255])
    green_lower = np.array([45, 50, 50])
    green_upper = np.array([85, 255, 255])

# mask 생성
    mask_red1 = cv2.inRange(hsv, red1_lower, red1_upper)
    mask_red2 = cv2.inRange(hsv, red2_lower, red2_upper)
    mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
    mask_green = cv2.inRange(hsv, green_lower, green_upper)

# 마스크로 각 색상만 추출
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
   
    pixel_red = cv2.countNonZero(mask_red)
    pixel_yellow = cv2.countNonZero(mask_yellow)
    pixel_green = cv2.countNonZero(mask_green)

    threshold = 200

    detected = []
    if pixel_red >= threshold:
        detected.append("red")
    if pixel_yellow >= threshold:
        detected.append("yellow")
    if pixel_green >= threshold:
        detected.append("green")

    result = traffic_light_detection()

    # 1) 아무것도 감지 안 되면 "none" (또는 이전 상태 유지로 바꿀 수 있음)
    if len(detected) == 0:
        result.state = prev_state if prev_state is not None else "none"

# 2) 여러 개 감지되면 우선순위 (보수적으로 red > yellow > green)
    elif len(detected) >= 2:
        if "red" in detected:
            result.state = "red"
        elif "yellow" in detected:
            result.state = "yellow"
        else:
            result.state = "green"

# 3) 하나만 감지되면 그 색
    else:
        result.state = detected[0]

# --- elapsed (기존 전역변수 구조 유지) ---
    now = rospy.Time.now()


    if prev_state is None:
        prev_state = result.state
        start_time = now
        result.elapsed = rospy.Duration(0.0)

    elif prev_state == result.state:
        result.elapsed = now - start_time   # rospy.Duration

    else:
        prev_state = result.state
        start_time = now
        result.elapsed = rospy.Duration(0.0)

    cv2.imshow("bgr", roi)
    cv2.waitKey(1)

    return result, pixel_red, pixel_yellow, pixel_green

def mainfunc():
    rospy.init_node('trafficlight_detection', anonymous=True)
    rospy.Subscriber ('/TLCam', CompressedImage, callback)
    pub = rospy.Publisher ('/TrafficLight',  traffic_light_detection, queue_size=10) 

    rate = rospy.Rate(10)



    while not rospy.is_shutdown():
        if topic_data is not None:
            # rs= detection (topic_data)
            rs, pr, py, pg = detection(topic_data)
            pub.publish(rs)
            rospy.loginfo(f"pixels r:{pr} y:{py} g:{pg}")
        
        rate.sleep()

if __name__ == '__main__':
    mainfunc()
