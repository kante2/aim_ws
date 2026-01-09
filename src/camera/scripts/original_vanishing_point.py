
#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
#from camera.msg import LaneInfo #float64 VanishingPoint_angle
import math

class LaneDetection():
    def __init__(self):
        rospy.init_node('lanedetector', anonymous=True)
        self.last_Rlines = []  # 이전 프레임에서 검출된 오른쪽 차선
        self.last_Llines = []  # 이전 프레임에서 검출된 왼쪽 차선
        self.last_R = {}  # 이전 프레임에서 검출된 오른쪽 차선
        self.last_L = {}  # 이전 프레임에서 검출된 왼쪽 차선
        self.prev_vanishPoint_x=[]
        self.prev_vanishPoint_y=[]

        self.bridge = CvBridge()
        #self.Publisher= rospy.Publisher("/LaneInfo", LaneInfo, queue_size=1)
        self.Subscriber = rospy.Subscriber("/LaneCam", CompressedImage, self.ImageCallback)
        
        self.rate = rospy.Rate(30)
        rospy.spin()

    def ImageCallback(self, msg):
        NpArr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(NpArr, cv2.IMREAD_COLOR)

        edges = img_filter(image)

        l1 = hough_tf(image, edges)
        l2 = grouping(image, l1)
        l3 = avglines(image, l2)
        R, L = filter_noise(image, l3,self)

        line_img = create_line(image, R, L, self)
        # cv2.imshow('Result', line_img)
        
        if cv2.waitKey(1) & 0xFF == ord('x'):
            rospy.signal_shutdown("Exit")
            cv2.destroyAllWindows()

        self.rate.sleep() #frame 속도 제한

def img_filter(image):
    # 1. HSV Filtering
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_blue = np.array([82, 112, 112])
    upper_blue = np.array([100, 255, 255])
    lower_white1 = np.array([13, 20, 220])
    upper_white1 = np.array([25, 60, 255])
    lower_white2 = np.array([83, 33, 47])
    upper_white2 = np.array([106, 83, 91])
    lower_yellow = np.array([10, 60, 0])
    upper_yellow = np.array([35, 255, 255])

    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    white_mask1 = cv2.inRange(hsv, lower_white1, upper_white1)
    white_mask2 = cv2.inRange(hsv, lower_white2, upper_white2)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    mask1 = cv2.bitwise_or(blue_mask, white_mask1)
    mask2 = cv2.bitwise_or(white_mask2, yellow_mask)
    mask = cv2.bitwise_or(mask1, mask2)
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    # 필터링 결과 Grayscale로 변환
    result = cv2.bitwise_and(image, mask_bgr)
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

    # 2. Gaussian Blur 적용
    blurred = cv2.GaussianBlur(gray, (0, 0), 1)
    
    # 3. Canny Edge Detection
    edges = cv2.Canny(blurred, 50, 100)
    return edges

def create_line(image, R, L, lane_detector):
    img_width = image.shape[1]
    line_img = image.copy()

    if R or L is not None:
        # 이전 프레임의 오른쪽 및 왼쪽 차선 정보
        last_Rlines = lane_detector.last_Rlines
        last_Llines = lane_detector.last_Llines

        # 현재 프레임의 오른쪽 및 왼쪽 차선 중 가장 유사한 선 두 개 선택
        Rs = find_similar_lines(image, R, last_Rlines)
        Ls = find_similar_lines(image, L, last_Llines)

        final_r = one_line(image, Rs)
        final_l = one_line(image, Ls)

        # 선택된 오른쪽 차선 그리기
        if not final_r=={}:
            x1, y1, x2, y2 = final_r['pos']
            cv2.line(line_img, (x1, y1), (x2, y2), (255, 255, 255), 2)  
        
        # 선택된 왼쪽 차선 그리기
        if not final_l=={}:
            x1, y1, x2, y2 = final_l['pos']
            cv2.line(line_img, (x1, y1), (x2, y2), (255, 255, 255), 2) 

        if (not final_r=={}) and (not final_l=={}) :
            vanishing_point(line_img, final_r, final_l, lane_detector)

        # 현재 프레임의 차선 정보를 저장하여 다음 프레임에서 활용
        lane_detector.last_Rlines = Rs
        lane_detector.last_Llines = Ls

    return line_img

def hough_tf(image, edges):
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=80, minLineLength=20, maxLineGap=50)
    
    # if lines is not None:
    #     hough_lines_image = image.copy()
    #     for line in lines:
    #         x1, y1, x2, y2 = line[0]
    #         cv2.line(hough_lines_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    #         cv2.imshow('Hough Lines', hough_lines_image)

    if lines is None:
        return []
    return lines
        
def grouping(image, lines, a_th=0.2, i_th=10):
    group_list = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1 + 1e-6)
        angle = np.degrees(np.arctan(slope))
        y_intercept = y1 - slope * x1
        x_intercept = -y_intercept / (slope + 1e-6)
        similar = False

        if not (20 < abs(angle) < 75):
            continue

        for group in group_list:
            if abs(group[0]['angle'] - angle) < a_th and abs(group[0]['x_intercept'] - x_intercept) < i_th:
                group.append({'slope': slope, 'angle': angle, 'x_intercept': x_intercept,'y_intercept': y_intercept, 'pos': (x1, y1, x2, y2)})
                similar = True
                break
        
        if not similar:
            group_list.append([{'slope': slope, 'angle': angle, 'x_intercept': x_intercept, 'y_intercept': y_intercept, 'pos': (x1, y1, x2, y2)}])
    
    grouped_lines_image = image.copy()
    # for group in group_list:
    #     for line in group:
    #         x1, y1, x2, y2 = line['pos']
    #         cv2.line(grouped_lines_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    #         cv2.imshow('Group Lines', grouped_lines_image)
    #         rospy.loginfo("Angle={0:.2f}, x_intercept={1:.2f}".format(line['angle'],line['x_intercept']))

    return group_list

def avglines(image, group_list):  
    avg_lines = []
    for group in group_list:
        if len(group) == 0:
            continue  # 그룹이 비어있으면 건너뜀.
        avg_slope = sum([line['slope'] for line in group]) / len(group)
        avg_x_intercept = sum([line['x_intercept'] for line in group]) / len(group)
        avg_y_intercept = sum([line['y_intercept'] for line in group]) / len(group)
        avg_angle = np.degrees(np.arctan(avg_slope))

        x1, x2 = 0, image.shape[1]
        y1 = int(avg_slope*x1 + avg_y_intercept)
        y2 = int(avg_slope*x2 + avg_y_intercept)
        
        avg_lines.append({'slope': avg_slope, 'angle': avg_angle, 'x_intercept': avg_x_intercept,'y_intercept': avg_y_intercept, 'pos': (x1, y1, x2, y2)})

    # avg_lines_image = image.copy()
    # for line in avg_lines:
    #     x1, y1, x2, y2 = line['pos']
    #     cv2.line(avg_lines_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    #     cv2.imshow('AVG Lines', avg_lines_image)
    #     rospy.loginfo("Angle={0:.2f}, x_intercept={1:.2f}".format(line['angle'],line['x_intercept']))

    return avg_lines

def filter_noise(image, avg_lines, lane_detection, a_th=12, i_th=80):
    filtered_Rlines = []
    filtered_Llines = []
    last_Rlines = lane_detection.last_Rlines
    last_Llines = lane_detection.last_Llines

    for line in avg_lines:
        angle, slope, x_intercept, y_intercept = line['angle'], line['slope'], line['x_intercept'], line['y_intercept']
        valid=True

        if line['angle'] > 0 : #오른쪽 차선
            if last_Rlines:  
                for last_line in last_Rlines: #직전 차선과 비교
                    x = -last_line['y_intercept'] / last_line['slope']
                    if abs(last_line['angle'] - angle) > a_th or abs(x - x_intercept) > i_th:
                        valid = False

            if valid:
                #rospy.loginfo(f"Right: angle={angle}, x_intercept={x_intercept}")
                filtered_Rlines.append(line)

        else: #왼쪽 차선
            if last_Llines:  
                for last_line in last_Llines:
                    x = -last_line['y_intercept'] / last_line['slope']
                    if abs(last_line['angle'] - angle) > a_th or abs(x - x_intercept) > i_th:
                        valid = False

            if valid:
                #rospy.loginfo(f"Left: angle={angle}, x_intercept={x_intercept}")
                filtered_Llines.append(line)

    # filtered_image = image.copy()
    # for line in filtered_Rlines:
    #     x1, y1, x2, y2 = line['pos']
    #     cv2.line(filtered_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    #     cv2.imshow('Filtered Lines', filtered_image)
    #     rospy.loginfo("Right Angle={0:.2f}, x_intercept={1:.2f}".format(line['angle'],line['x_intercept']))

    # for line in filtered_Llines:
    #     x1, y1, x2, y2 = line['pos']
    #     cv2.line(filtered_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    #     cv2.imshow('Filtered Lines', filtered_image)
    #     # rospy.loginfo("Left Angle={0:.2f}, x_intercept={1:.2f}".format(line['angle'],line['x_intercept']))

    return filtered_Rlines, filtered_Llines

def one_line(image, lines):

    N=len(lines)
    sum_slope = 0
    sum_y_intercept = 0

    if N>0:
        for l in lines:
            sum_slope += l['slope']
            sum_y_intercept += l['y_intercept']

        avg_slope = sum_slope / N
        avg_y_intercept = sum_y_intercept / N
        
        y1, y2 = 0, image.shape[1]
        x1 = int((y1 - avg_y_intercept) / avg_slope)
        x2 = int((y2 - avg_y_intercept) / avg_slope)

        return {'slope': avg_slope, 'y_intercept': avg_y_intercept, 'pos': (int(x1), int(y1), int(x2), int(y2))}

    else:
        return {}

def find_similar_lines(image, current_lines, previous_lines):
    if not previous_lines:
        return current_lines[:2]

    similarities = []
    for curr_line in current_lines:
        for prev_line in previous_lines:
            angle_diff = abs(curr_line['angle'] - prev_line['angle'])
            x_intercept_diff = abs(curr_line['x_intercept'] - prev_line['x_intercept'])
            similarities.append((angle_diff + x_intercept_diff, curr_line))

    # 유사도 기준으로 정렬하여 상위 top_n개의 선을 선택
    similarities.sort(key=lambda x: x[0])
    most_similar_lines = [line for _, line in similarities[:2]]

    two_lines_image = image.copy()

    return most_similar_lines

def vanishing_point(image, line1, line2, lane_detector):
    prev_vanish_size = 30 ##과거 소실점 담을 크기
    vanish_img = image.copy()
    #pub_msg = LaneInfo()

    m1, b1 = line1['slope'], line1['y_intercept']
    m2, b2 = line2['slope'], line2['y_intercept']

    if m1 == m2: #두선이 평행한 경우
        return None

    vanish_x = (b2 - b1) / (m1 - m2)
    vanish_y = m1 * vanish_x + b1
    point = (int(vanish_x),int(vanish_y))


    if point is not None:
        #중앙선
        center_x = image.shape[1]//2  # 도로 중앙 X 좌표
        center_y = image.shape[0]
        cv2.line(image, (center_x, 0), (center_x, center_y), (255, 255, 255), 2)  # 흰색 실선
        cv2.line(image, (int(vanish_x), int(vanish_y)), (center_x, center_y), (255, 0, 255), 2)
        cv2.circle(image, point, 5, (255,0,255), -1)

        # 벡터 1: 중앙선 벡터 (기준선, 아래 방향)
        v_center_x = 0  # x 변화 없음
        v_center_y = -center_y   # y 방향으로만 이동
        
        # 벡터 2: 소실점에서 중앙선 하단까지 연결하는 벡터
        v_vanish_x = int(vanish_x) - center_x
        v_vanish_y = int(vanish_y) - center_y

        #두 벡터 사이 각도 계산
        angle = math.degrees(math.atan2(v_vanish_y, v_vanish_x) - math.atan2(v_center_y, v_center_x))
        #pub_msg.VanishingPoint_angle = angle

        #결과 범위를 [-180 ~ 180]로
        if angle > 180:
            angle -= 360
        elif angle < -180:
            angle += 360

        # lane_detector.prev_vanishPoint_x.append(int(vanish_x))
        # lane_detector.prev_vanishPoint_y.append(int(vanish_y))

        # if len(lane_detector.prev_vanishPoint_x)>prev_vanish_size:
        #         lane_detector.prev_vanishPoint_x.pop(0) 
        #         lane_detector.prev_vanishPoint_y.pop(0)
        
        # avg_vanish_x = int(sum(lane_detector.prev_vanishPoint_x) / len(lane_detector.prev_vanishPoint_x))
        # avg_vanish_y = int(sum(lane_detector.prev_vanishPoint_y) / len(lane_detector.prev_vanishPoint_y))

        # x,y,w,h = 400,300,200,200

        # roi= vanish_img[y:y+h,x:x+w]

        # scale_factor = 4

        # zoomed_vanish_img = cv2.resize(roi, (w * scale_factor, h * scale_factor), interpolation=cv2.INTER_LINEAR)


        # cv2.circle(zoomed_vanish_img, (int(vanish_x), int(vanish_y)), 2, (0, 255, 0), 20)
        # cv2.circle(zoomed_vanish_img, (avg_vanish_x, avg_vanish_y), 2, (0, 255, 255), 20)

        # cv2.imshow('MAF',zoomed_vanish_img)

    # 메시지 퍼블리시
        #lane_detector.Publisher.publish(pub_msg)
    

if __name__ == '__main__':
    LaneDetection()
LaneDetection.py