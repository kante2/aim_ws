#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
# from camera.msg import LaneInfo #float64 VanishingPoint_angle
import math

class LaneInformation:
    def __init__(self, slope, angle, y_intercept, line=0.0):
        self.slope = slope
        self.angle = angle
        self.y_intercept = y_intercept
        self.x_intercept = (720 - y_intercept)/slope #y=0 이 아니라 y=max 일 때 x값
        self.line = line

class LaneDetection:
    def __init__(self):
        rospy.init_node('lanedetector', anonymous=True)
        
        self.bridge = CvBridge()
        self.Subscriber = rospy.Subscriber("/LaneCam", CompressedImage, self.callback)
        #self.Publisher = rospy.Publisher("/LaneInfo", LaneInfo, queue_size=1)
        
        self.last_Rlines = []
        self.last_Llines = []
        self.last_R = {}
        self.last_L = {}
        self.prev_vanishPoint_x = []
        self.prev_vanishPoint_y = []

        self.rate = rospy.Rate(30)
        rospy.spin()
        
        
    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            print("Image decode failed:", e)
            return
        
        h, w = img_bgr.shape[:2]
        
        roi = img_bgr[int(h*1.3/3):int(h*0.8), :]
        # [높이, 너비, 채널 수]인데 왜 지금 높이값만 사용하고 있노??
        
        edges = hsv(roi)
        
        hough_lines = hough(roi, edges)
        grouping_lines = grouping(hough_lines)
        avg_lines = avglines(roi, grouping_lines)
        R, L = filter_noise(roi, avg_lines, self)
        
        line_img = create_line(roi, R, L, self)
        
        debug_img = roi.copy()
        
        for line in hough_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow("hough", debug_img)
        
        cv2.imshow("Lane Detection", line_img)

        
        if cv2.waitKey(1) & 0xFF == ord('x'):
            rospy.signal_shutdown("Exit")
            cv2.destroyAllWindows()
            
        self.rate.sleep()
        
    
def hsv(roi):
    #HSV 변환 
    img_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
    lower_blue = np.array([80, 50, 100])
    upper_blue = np.array([100, 255, 255])

    #흰색 차선 범
    # lower_white1 = np.array([0, 0, 180])
    # upper_white1 = np.array([25, 80, 255])
    # lower_white2 = np.array([83, 33, 47])
    # upper_white2 = np.array([180, 60, 91])
    # mask_wlane1 = cv2.inRange(img_hsv, lower_white1, upper_white1)
    # mask_wlane2 = cv2.inRange(img_hsv, lower_white2, upper_white2)
    
    # lower_white = np.array([0, 30, 220])
    # upper_white = np.array([180, 80, 255])
    lower_white = np.array([14, 14, 200])
    upper_white = np.array([45, 50, 255])

    #노란색 차선 범위
    lower_yellow = np.array([10, 60, 0])
    upper_yellow = np.array([35, 255, 255])

    mask_blue = cv2.inRange(img_hsv, lower_blue, upper_blue)
    mask_white = cv2.inRange(img_hsv, lower_white, upper_white)
    mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

    #마스크 결합
    mask1 = cv2.bitwise_or(mask_blue, mask_white)
    mask2 = cv2.bitwise_or(mask_white, mask_yellow)
    mask = cv2.bitwise_or(mask1, mask2)

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
    result = cv2.bitwise_and(roi, mask_bgr)

    # grayscale로 변환
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(gray, (5, 5), 1)
    edges = cv2.Canny(blurred, 100, 200)# 40, 255
    
    return edges


def hough(roi, edges):
    
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100, minLineLength=20, maxLineGap=50)

    # 허프라인 잘 검출되는지 그려보기
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(roi, (x1, y1), (x2, y2), (0,0, 255), 1)

    if lines is None:
        print (" no lines ")

    return lines 

    # 일반 허프라인

    # lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=130)
    # for line in lines:
    #     r, theta - line[0]
    #     tx, ty = np.cos(theta), np.sin(theta)
    #     x0, y0 = tx*r, ty*r
    #     cv2.circle(img2, (abs(x0), abs(y0)), 3, (0,0,255), -1)
    #     x1, y1 = int(x0 + w*(-ty)), int(y0 + h * tx)
    #     x2, y2 = int(x0 - w*(-ty)), int(y0 - h * tx)
    #     cv2.line(img2, (x1, y1), (x2, y2), (0,255,0), 1)



# 피드백 : 계속 안쪽 선이 클러스터링 되도록 해야함.
# def grouping(roi, lines, angle_l=1.0, x_l=10): #angle_l = 1.0 / x_l = 10 정도로 완화 권장

# 각도, 선길이 필터링
def grouping(lines):
    group_list = []
    
    # 각 선분마다 기울기와 절편을 이용해 직선의 방정식 y=mx+c를 구한다. 

    # 기존 코드
    # for line in lines:
    #     x1, y1, x2, y2 = line[0]
    #     # 검출된 직선의 두 끝점 좌표(실제 차선 좌표)
    #     slope = (y2 - y1)/(x2 - x1 + 1e-6)
    #     angle = np.degrees(np.arctan(slope)) # angle 기울기를 degree로 변환 (각도)
    #     y_intercept = y1 - slope * x1 # y절편 (선이 y축을 지나는 지점)
    #     x_intercept = -y_intercept / (slope + 1e-6) # x절편 (y=0일 때 x값, 화면 아래에서 만나는 위치)
    #     similar = False
        
    #     if (25 < abs(angle) < 75):
    #         for group in group_list:
    #             if abs(group[0]['angle']-angle) < angle_l and abs(group[0]['x_intercept']-x_intercept) < x_l:
    #                 group.append({'slope': slope, 'angle': angle, 'x_intercept': x_intercept,'y_intercept': y_intercept, 'pos': (x1, y1, x2, y2)})
    #                 similar = True
    #                 break
    #     else :
    #         continue
            
    #     if not similar:
    #         group_list.append([{'slope': slope, 'angle': angle, 'x_intercept': x_intercept, 'y_intercept': y_intercept, 'pos': (x1, y1, x2, y2)}])
            
    # return group_list

# ----------------------------------------------------------------------------------

    # 다른 방법
    for line in lines: # lines 는 허프라인
        x1, y1, x2, y2 = line[0]

        # Calculating equation of the line: y = mx + c
        if x1 != x2:
            slope = (y2 - y1) / (x2 - x1)
        else:
            slope = 100000000
        y_intercept = y1 - slope*x1 


        angle = np.degrees(np.arctan(slope))
        # 세타는 수평선과 이루는 각도. 

        # Threshold by which lines will be rejected wrt the horizontal
        angle_threshold = 10
        # 거의 수평 (0), 수직 (90)에 가까운 선들은 배제
        # --> 허용하는 각도 범위 [-86, -4] or [+4, +86]

        # 같은 선끼리 그룹핑을 해야 함. 
        # 왜 해야하냐, 같은 그룹 안에 있는 선들이 완전히 똑같은 선은 아니기 때문에 그룹 안에서 
        # 평균을 내고, 오른쪽, 왼쪽 차선 클러스터링(?)을 하게 된다. 

        if angle_threshold <= abs(angle) <= (90 - angle_threshold):
            # l = math.sqrt( (y2 - y1)**2 + (x2 - x1)**2 )    # length of the line

            LI = LaneInformation (slope, angle, y_intercept, line)

            similar = False
            max_angle = 4
            max_yintercept = 10
 
            for group in group_list:

                if (group[0].angle * LI.angle) >= 0 and abs(group[0].angle - LI.angle) < max_angle and abs(group[0].y_intercept - LI.y_intercept) < max_yintercept:
                    # group.append({'slope': slope, 'angle': angle, 'x_intercept': x_intercept,'y_intercept': y_intercept, 'pos': (x1, y1, x2, y2)})
                    group.append(LI)
                    similar = True
                    break
            
            if not similar:
                group_list.append([LI])

    return group_list
        
    # # 선 길이 필터링
    # if len(group_list) > 15:
    #     # 각도 필터링 이후 남은 선들 중, 가장 긴 15개 선만 선택
    #     group_list = sorted(group_list, key=lambda x: x[-1], reverse=True)
    #     # 가장 긴 선을 선택하기 위해, 선들을 길이에 따라 정렬 (sort())
    #     group_list = group_list[:15]
    #     # 그 중 앞에서부터 15개 선 선택

    # return group_list
    # 점선 같은 경우에는 상위 긴 선에 포함되지 않을 수도 있으니깐 길이 필터링은 안해도 되지 않을까



# grouping() : 실제 차선의 양 끝점을 이용해서 기울기와 절편을 구함
# avglines() : 직선들의 평균 기울기와 절편을 구한 뒤 직선을 그리기 위해 x값 2개 임의 선택 후 y값 계산

def avglines(roi, group_list):
    avg_lines = []
    for group in group_list:
        # if len(group) == 0:
        #     continue
        
        avg_slope = sum([info.slope for info in group]) / len(group)
        # 각 group 안의 LaneInformation() 객체가 info
        avg_y_intercept = sum([info.y_intercept for info in group]) / len(group)
        avg_angle = sum([info.angle for info in group]) / len(group)
        # avg_angle = np.degrees(np.arctan(avg_slope))
        # 이걸 보니깐 차선 정보를 다루기에는 클래스의 객체를 이용하는 게 확실히 좋아보이긴 한다. 
        # 차선 정보 저장하는 클래스 만들자. 

        
        # group 내 실제 pos들에서 최소/최대 x 또는 y 범위를 잡기 
        x1 = min(info.line[0] for info in group)
        x2 = max(info.line[2] for info in group)

        # 그 구간에서 평균 직선 식을 기반으로 y 계산
        y1 = int(avg_slope * x1 + avg_y_intercept)
        y2 = int(avg_slope * x2 + avg_y_intercept)

        avg_line = (x1, y1, x2, y2)

        ALI = LaneInformation(avg_slope, avg_angle, avg_y_intercept, avg_line)

        avg_lines.append(ALI)

        # avg_lines.append({'slope': avg_slope, 'angle': avg_angle, 'x_intercept': avg_x_intercept,'y_intercept': avg_y_intercept, 'pos': (x1, y1, x2, y2)})
        # pos의 좌표는 평균 slope, 평균 intercept으로 만들어낸 대표 직선의 두 끝점이라고 봐야 한다. 정확한 실제 차선경계가 아님.
    return avg_lines


# 오른쪽, 왼쪽 차선 클러스터링
# 실제 차선이 아닌 노이즈 제거 함수
# 그렇다면 뭐를 노이즈라고 할 건데? 

def filter_noise(roi, avg_lines, lane_detection, angle_l = 12, x_l = 80): #기준값 범위를 좁힐 필요성 있음
        # 클래스의 초기화 함수에서 설정한 변수들

        # self.last_Rlines = []
        # self.last_Llines = []
        # self.last_R = {}
        # self.last_L = {}
        # self.prev_vanishPoint_x = []
        # self.prev_vanishPoint_y = []

        # self.N = 4

        # self.L_lane_queue = deque(maxlen=self.N)
        # self.R_lane_queue = deque(maxlen=self.N) 

    filtered_Rlines = []
    filtered_Llines = []
    last_Rlines = lane_detection.last_Rlines
    last_Llines = lane_detection.last_Llines
    
    R_Lines = []
    L_Lines = []

    # 일단 오른쪽, 왼쪽으로 차선 정보 분류
    # 1. x절편 / 2. 기울기 가 분류 기준. 
    
    for line in avg_lines:
    # avg_lines 구성 상태
    # avg_lines = [ LaneInformation(avg_slope, avg_angle, avg_y_intercept, avg_line) 객체1,
    #               LaneInformation(avg_slope, avg_angle, avg_y_intercept, avg_line) 객체2,
    #               LaneInformation(avg_slope, avg_angle, avg_y_intercept, avg_line) .....
    #             ]

    # # 세번째
    #     angle = line['angle']
    #     x_intercept = line['x_intercept']

        # if line.angle > 0:
        #     R_Lines.append(line)
        # elif line.angle < 0:
        #     L_Lines.append(line)


    # 일단 오/왼 으로 나누기
        # 오른쪽 차선
        if line.angle > 0:
            R_Lines.append(line)
            # R_Lines가 아니라 lane_detection.R_lane_queue.append(line) 해야하는 거 아닌가?

            # 관찰한 이전 프레임보다 검출된 오른쪽 차선의 개수가 적을 때

            if len(lane_detection.R_lane_queue) < lane_detection.N:
                # 난 R_lane_queue에 차선정보를 저장한 적이 없는데 ?????????????????????/

                # 클래스의 init함수의 멤버 변수에 외부 함수(filter_noise)가 접근하고 있기 때문에. 
                # 오른쪽 차선 정보를 저장하는 디큐 객체와 디큐의 최대 길이 

                Sorted_R_Lines = sorted(R_Lines, key=lambda x: x.angle)
                # angle 값을 기준으로 오름차순 정렬(다른 값들도 정렬)하여 새로운 리스트를 반환.

                if len(Sorted_R_Lines) == 1:
                    avg_R_slope = Sorted_R_Lines.slope
                    avg_R_intercept = Sorted_R_Lines.x_intercept
                else:
                    avg_R_slope = round((Sorted_R_Lines[0].slope + Sorted_R_Lines[1].slope) / 2, 4)
                    avg_R_intercept = round((Sorted_R_Lines[0].x_intercept + Sorted_R_Lines[1].x_intercept) / 2, 4)


            # 관찰한 이전 프레임보다 검출된 오른쪽 차선의 개수가 같을 때 (큐 특성 상 N보다 클 수는 없다)
            elif len(lane_detection.R_lane_queue) == lane_detection.N:
                Sorted_R_Lines = sorted(R_Lines, key=lambda x: abs(x.x_intercept - lane_detection.R_lane_queue[-1].x_intercept))
                # R_Lines는 최근 라인들, 큐에 저장한 거는 전 프레임의 대표(평균)선들. [-1]의 차선이 과거 프레임 중 가장 최근 차선.

                Sorted_R_Lines_P = sorted(
                    filter(lambda x: x.x_intercept - lane_detection.R_lane_queue[-1].x_intercept < 0, R_Lines),
                    key=lambda x: abs(x.x_intercept - lane_detection.R_lane_queue[-1].x_intercept)
                )
                #현재 x절편에서 과거 x절편 뺀 게 음수 -> 오른쪽 차선에서도 차선이 여러개가 있을 건데, 그 중 현재 차선이 더 안쪽에 있다는 거다. 
                Sorted_R_Lines_N = sorted(
                    filter(lambda x: x.x_intercept - lane_detection.R_lane_queue[-1].x_intercept > 0, R_Lines),
                    key=lambda x: abs(x.x_intercept - lane_detection.R_lane_queue[-1].x_intercept)
                )
                # 현재 오른쪽 차선이 더 바깥쪽에 있다는 거. 

                # Soted_R_Lines에는 현재 오른쪽 차선의 x절편 - 과거 오른쪽 차선의 x절편 = 값을 '기준'으로 
                # line들이 오름차순으로 정렬되어있다. (작은 거부터)
                if len(Sorted_R_Lines) == 1:
                    # 애초에 N=4인데 현재 차선의 x절편 - 과거 프레임의 x절편 = 절댓값을 기준으로 현재 차선 R_lines를 정렬했을 때
                    # Sorted_R_Lines의 길이가 1일 수가 있지 암암
                    # 왜냐면 지금 과거 프레임이 max 4를 채웠을 때인 거니깐, 비교할 현재 프레임이 1개만 들어오면 x절편 차 비교하고 
                    # Sorted_R_Lines 리스트에 현재 line 정보 1개만 저장하게 되는 거지. 

                    avg_R_slope = round((Sorted_R_Lines[0].slope + lane_detection.R_lane_queue[-1].slope) / 2, 4)
                    # 근데 왜 현재 차선의 기울기와 과거 프레임의 기울기를 평균내는 거지????????????????????????

                    avg_R_intercept = round((Sorted_R_Lines[0].x_intercept + lane_detection.R_lane_queue[-1].x_intercept) / 2, 4)
                else:
                    if len(Sorted_R_Lines_P) > 0 and len(Sorted_R_Lines_N) > 0:
                        # 과거 차선보다 바깥쪽에도 있고 안쪽에도 있으면 가장 과거차선과 x절편의 차가 작은=가까운 
                        # 두 안쪽 바깥쪽 line의 기울기와 x절편의 평균을 구한다. 
                        avg_R_slope = round((Sorted_R_Lines_P[0].slope + Sorted_R_Lines_N[0].slope) / 2, 4)
                        avg_R_intercept = round((Sorted_R_Lines_P[0].x_intercept + Sorted_R_Lines_N[0].x_intercept) / 2, 4)
                    else:
                        avg_R_slope = round((Sorted_R_Lines[0].slope + Sorted_R_Lines[1].slope) / 2, 4)
                        avg_R_intercept = round((Sorted_R_Lines[0].x_intercept + Sorted_R_Lines[1].x_intercept) / 2, 4)

    # avg 값을 바탕으로 대표 차선 객체 생성
        avg_R_angle = np.degrees(np.arctan(avg_R_slope))
        avg_y_intercept = -avg_R_slope * avg_R_intercept
        
        x1 = 0
        x2 = roi.shape[1]
        
        y1 = int(avg_R_slope * x1 + avg_y_intercept)
        y2 = int(avg_R_slope * x2 + avg_y_intercept)
        
        filtered_Rlines.append({
            'slope': avg_R_slope,
            'x_intercept': avg_R_intercept,
            'y_intercept': avg_y_intercept,
            'angle': avg_R_angle,  # angle은 필요 시 재계산
            'pos': (x1, y1, x2, y2)    # pos 필요시 추가
        })

    else:  # 오른쪽 차선 없음
        if len(lane_detection.R_lane_queue) == 0:
            filtered_Rlines = []
        else:
            filtered_Rlines = [lane_detection.R_lane_queue[0]]


    # 왼쪽 차선
    if len(L_Lines) > 0:  # 왼쪽 차선이 존재할 때
        L_Lines.append(line)

        if len(lane_detection.L_lane_queue) < lane_detection.N:
            Sorted_L_Lines = sorted(L_Lines, key=lambda x: x['angle'])

            if len(Sorted_L_Lines) == 1:
                avg_L_slope = Sorted_L_Lines[0]['slope']
                avg_L_intercept = Sorted_L_Lines[0]['x_intercept']
            else:
                avg_L_slope = round((Sorted_L_Lines[0]['slope'] + Sorted_L_Lines[1]['slope']) / 2, 4)
                avg_L_intercept = round((Sorted_L_Lines[0]['x_intercept'] + Sorted_L_Lines[1]['x_intercept']) / 2, 4)

        else:
            Sorted_L_Lines = sorted(L_Lines, key=lambda x: abs(x['x_intercept'] - lane_detection.L_lane_queue[0]['x_intercept']))
            Sorted_L_Lines_P = sorted(
                filter(lambda x: x['x_intercept'] - lane_detection.L_lane_queue[0]['x_intercept'] > 0, L_Lines),
                key=lambda x: abs(x['x_intercept'] - lane_detection.L_lane_queue[0]['x_intercept'])
            )
            Sorted_L_Lines_N = sorted(
                filter(lambda x: x['x_intercept'] -lane_detection.L_lane_queue[0]['x_intercept'] < 0, L_Lines),
                key=lambda x: abs(x['x_intercept'] - lane_detection.L_lane_queue[0]['x_intercept'])
            )

            if len(Sorted_L_Lines) == 1:
                avg_L_slope = round((Sorted_L_Lines[0]['slope'] + lane_detection.L_lane_queue[0]['slope']) / 2, 4)
                avg_L_intercept = round((Sorted_L_Lines[0]['x_intercept'] + lane_detection.L_lane_queue[0]['x_intercept']) / 2, 4)
            else:
                if len(Sorted_L_Lines_P) > 0 and len(Sorted_L_Lines_N) > 0:
                    avg_L_slope = round((Sorted_L_Lines_P[0]['slope'] + Sorted_L_Lines_N[0]['slope']) / 2, 4)
                    avg_L_intercept = round((Sorted_L_Lines_P[0]['x_intercept'] + Sorted_L_Lines_N[0]['x_intercept']) / 2, 4)
                else:
                    avg_L_slope = round((Sorted_L_Lines[0]['slope'] + Sorted_L_Lines[1]['slope']) / 2, 4)
                    avg_L_intercept = round((Sorted_L_Lines[0]['x_intercept'] + Sorted_L_Lines[1]['x_intercept']) / 2, 4)

        # avg 값을 바탕으로 대표 차선 객체 생성
        avg_L_angle = np.degrees(np.arctan(avg_L_slope))
        avg_y_intercept = -avg_L_slope * avg_L_intercept 
        
        x1 = 0
        x2 = roi.shape[1]
        
        y1 = int(avg_L_slope * 0 + avg_y_intercept)
        y2 = int(avg_L_slope * roi.shape[1] + avg_y_intercept)

        filtered_Llines.append({
            'slope': avg_L_slope,
            'x_intercept': avg_L_intercept,
            'y_intercept': avg_y_intercept,
            'angle': avg_L_angle,  # angle은 평균 기반으로 다시 구해도 됨 (atan)
            'pos': (x1, y1, x2, y2)    # pos가 필요한 경우 추가 구현 필요
        })

    else:  # 왼쪽 차선 없음
        if len(lane_detection.L_lane_queue) == 0:
            filtered_Llines = []
        else:
            filtered_Llines = [lane_detection.L_lane_queue[0]]  # 과거 차선

    
    print("R : ", filtered_Llines)
    print("L : ", filtered_Rlines)
    
    return filtered_Llines, filtered_Rlines

# 좌,우 한 쪽씩 find_similar_lines() 적용됨
def find_similar_lines(roi, current_lines, previous_lines):
    if not previous_lines:
        return current_lines[:2]
    
    similarities = []
    for curr_line in current_lines:
        for prev_line in previous_lines:
            angle_diff = abs(curr_line['angle'] - prev_line['angle'])
            x_intercept_diff = abs(curr_line['x_intercept'] - prev_line['x_intercept'])
            similarities.append((angle_diff + x_intercept_diff, curr_line))
            
    similarities.sort(key=lambda x: x[0])
    most_similar_lines = [line for _, line in similarities[:2]]
    
    return most_similar_lines

# 반환값 most_similar_lines 을 파라미터로 사용
def one_line(roi, lines):
    N = len(lines)
    
    if N >= 2:
        avg_slope = (lines[0]['slope'] + lines[1]['slope']) / 2
        avg_y_intercept = (lines[0]['y_intercept'] + lines[1]['y_intercept']) / 2

        y_val = []
        for line in lines:
            y_val.extend([line['pos'][1], line['pos'][3]])

        y1 = min(y_val)
        y2 = max(y_val)

        x1 = int((y1 - avg_y_intercept) / (avg_slope + 1e-6))
        x2 = int((y2 - avg_y_intercept) / (avg_slope + 1e-6))
        
        return {
            'slope': avg_slope,
            'y_intercept': avg_y_intercept,
            'pos': (x1, y1, x2, y2)
        }

    elif N == 1:
        avg_slope = lines[0]['slope']
        avg_y_intercept = lines[0]['y_intercept']
        x1, y1, x2, y2 = lines[0]['pos']
        
        return {
            'slope': avg_slope,
            'y_intercept': avg_y_intercept,
            'pos': (x1, y1, x2, y2)
        }
        
    else:
        return {}
    
    
def vanishing_point(roi, line1, line2, lane_detector):
    # prev_vanish_size = 30
    #vanish_img = img_bgr.copy()
    # pub_msg = LaneInfo()
    
    a1, y1 = line1['slope'], line1['y_intercept']
    a2, y2 = line2['slope'], line2['y_intercept']
    
    #print(f"a1: {a1:.4f}, y1: {y1:.2f}, a2: {a2:.4f}, y2: {y2:.2f}")

    
    #if a1 == a2:
        #return None
    if abs(a1-a2) < 1e-3:
        return None
    
    vanish_x = (y2 - y1)/(a1 - a2)
    vanish_y = a1 * vanish_x + y1
    point = (int(vanish_x), int(vanish_y))
    
    #print(f"Vanishing Point: ({vanish_x:.1f}, {vanish_y:.1f})")
    
    if point is not None:
        #cv2.line(roi, (center_x, 0), (center_x, center_y), (255, 255, 255), 2)
        #cv2.line(roi, (int(vanish_x), int(vanish_y)), (center_x, center_y), (255, 0, 255), 2)
        x1, y1, x2, y2 = line1['pos']
        x3, y3, x4, y4 = line2['pos']
        #print ("right", x1, y1, x2, y2)
        #print ("left", x3, y3, x4, y4)
        cv2.line(roi, (x2, y2), point, (0, 0, 255), 2) #right
        cv2.line(roi, (x4, y4), point, (0, 0, 255), 2) #left
        
        cv2.circle(roi, point, 5, (0,255,0), -1)
        
        center_x = roi.shape[1]//2
        center_y = roi.shape[0]
    
        v_center_x = 0
        v_center_y = -center_y
        v_vanish_x = int(vanish_x) - center_x
        v_vanish_y = int(vanish_y) - center_y
        
        angle = math.degrees(math.atan2(v_vanish_y, v_vanish_x) - math.atan2(v_center_y, v_center_x))

        # pub_msg.VanishingPoint_angle = angle

        if angle > 180:
            angle -= 360
        elif angle < -180:
            angle += 360

        # lane_detector.Publisher.publish(pub_msg)
        
        print("Vanishing Point: ", point)
    
def create_line(roi, R, L, lane_detector):
    img_width = roi.shape[1]
    line_img = roi.copy()
    
    if R or L is not None:
        last_Rlines = lane_detector.last_Rlines
        last_Llines = lane_detector.last_Llines
        
        Rs = find_similar_lines(roi, R, last_Rlines)
        Ls = find_similar_lines(roi, L, last_Llines)
        
        final_r = one_line(roi, Rs)
        final_l = one_line(roi, Ls)
        
        print("final_r:", final_r)
        print("final_l:", final_l)
        
        if final_r != {} and final_l != {}:
            a1, y1 = final_r['slope'], final_r['y_intercept']
            a2, y2 = final_l['slope'], final_l['y_intercept']

            if abs(a1 - a2) > 1e-3:
                vanish_x = (y2 - y1) / (a1 - a2)
                vanish_y= a1 * vanish_x + y1
                vanish = (int(vanish_x), int(vanish_y))

                # 오른쪽 선: 끝점을 소실점으로 수정
                x1_r, y1_r, _, _ = final_r['pos']
                x2_r = int(vanish_x)
                y2_r = int(vanish_y)
                cv2.line(line_img, (x1_r, y1_r), (x2_r, y2_r), (0, 0, 255), 2)
                
                # 왼쪽 선: 끝점을 소실점으로 수정
                x1_l, y1_l, _, _ = final_l['pos']
                x2_l = int(vanish_x)
                y2_l = int(vanish_y)
                cv2.line(line_img, (x1_l, y1_l), (x2_l, y2_l), (0, 0, 255), 2)
                cv2.circle(line_img, vanish, 5, (0,255,0), -1)
                vanishing_point(line_img, final_r, final_l, lane_detector)
            elif final_r != {}:
                x1, y1, x2, y2 = final_r['pos']
                cv2.line(line_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            elif final_l != {}:
                x3, y3, x4, y4 = final_l['pos']
                cv2.line(line_img, (x3, y3), (x4, y4), (0, 0, 255), 2)

                lane_detector.last_Rlines = Rs
                lane_detector.last_Llines = Ls

    return line_img

if __name__ == '__main__':
    LaneDetection()
 