#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
# from drive.msg import LaneInfo #float64 VanishingPoint_angle
from collections import deque
import math

class LaneDetection():
    def __init__(self):
        rospy.init_node('lanedetector', anonymous=True)
        
        self.bridge = CvBridge()
        self.Subscriber = rospy.Subscriber("/LaneCam", CompressedImage, self.callback)
        # self.Publisher = rospy.Publisher("/LaneInfo", LaneInfo, queue_size=1)
        
        self.width = 1020
        self.height = 720

        self.last_Rlines = []
        self.last_Llines = []
        self.last_R = {}
        self.last_L = {}
        self.prev_vanishPoint_x = []
        self.prev_vanishPoint_y = []

        # self.N = 4

        # self.L_lane_queue = deque(maxlen=self.N)
        # self.R_lane_queue = deque(maxlen=self.N) 

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
        
        # roi = img_bgr[int(h*1.4/3):int(h*0.8), :] # [높이, 너비, 채널]
        roi = img_bgr[:int(h*0.8), :]
        # 피드백 : 경사의 경우, 도로가 화면 위쪽에 잡히기 때문에 roi로 하늘을 자르지 않는 것이 좋다. 
        #         (지금은 평면이니 ㄱㅊ)


        edges = hsv(roi)
        hough_line = hough(roi, edges)
        grouping_line = grouping(roi, hough_line)
        avg_line = avglines(roi, grouping_line)
        L_filtered, R_filtered = filter_noise(roi, avg_line, self)

        R_similar = find_similar_lines(roi, R_filtered, self.last_Rlines)
        L_similar = find_similar_lines(roi, L_filtered, self.last_Llines)

        R_one_line = one_line(roi, R_similar)
        L_one_line = one_line(roi, L_similar)

        # line_img = create_line(roi, R_filtered, L_filtered, self)
        # create_line 문제였다

        
        hough_img = roi.copy()

        color = [(255, 0, 0), (0,255,0), (0,0,255), (0,255,255), (255,255,0), (255,0,255),(0,0,0),(255,255,255)]
        
        # -------------------------- edges ---------------------------
        # for line in hough_line:
        #     x1, y1, x2, y2 = line[0]
        #     cv2.line(hough_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # cv2.imshow("hough", hough_img)

        # -------------------------- hough ---------------------------
        # for line in hough_line:
        #     x1, y1, x2, y2 = line[0]
        #     cv2.line(hough_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # cv2.imshow("hough", hough_img)

        # -------------------------- 개별 그룹 ----------------------------------- 
        # group = grouping_line[1]

        # for lines in group:
        #     x1, y1, x2, y2 = lines['pos']
        #     cv2.line(hough_img, (x1, y1), (x2, y2), (0,0,255), 2)

        # # print ("group_list", group)

        # cv2.imshow("grouping", hough_img)

        # --------------------------- grouping --------------------------
        # for idx, group in enumerate(grouping_line):
        #     print("group_line_size:", len(grouping_line))
        #     color_idx = idx % len(color)
        #     group_color = color[color_idx]

        #     for lines in group:
        #         x1, y1, x2, y2 = lines['pos']
        #         cv2.line(hough_img, (x1, y1), (x2, y2), group_color, 2)

        # cv2.imshow("grouping", hough_img)

        # ----------------------------avg_lines ------------------------------
        # for idx, group in enumerate(avg_line):
        #     # print("group_line_size:", len(grouping_line))
        #     color_idx = idx % len(color) 
        #     group_color = color[color_idx]

        #     x1, y1, x2, y2 = group['pos']
        #     cv2.line(hough_img, (x1, y1), (x2, y2), group_color, 2)

        # cv2.imshow("avg_lines", hough_img)


        # -------------------------filtered_liens ---------------------
        # print ("R : ", len(R_filtered))
        # print ("L : ", len(L_filtered))

        # for idx, group in enumerate(R_filtered):
        #     color_idx = idx % len(color)  # 색 개수보다 그룹이 많을 경우 반복 사용
        #     group_color = color[color_idx]

        #     # 이 pos는 ㄹㅇ 차선이 아니라 차선을 그리기 위한 x1 = 0, x2 = 1080 일 때 -> y1, y2
        #     x1, y1, x2, y2 = group['pos']
        #     # print ("R - x1: ", x1, " x2: ", x2)
        #     cv2.line(hough_img, (x1, y1), (x2, y2), group_color, 2)
        

        # for idx, group in enumerate(L_filtered):
        #     color_idx = idx % len(color)  # 색 개수보다 그룹이 많을 경우 반복 사용
        #     group_color = color[color_idx]

        #     x1, y1, x2, y2 = group['pos']
        #     # print ("L - x1: ", x1, " x2: ", x2)
        #     cv2.line(hough_img, (x1, y1), (x2, y2), group_color, 2)

        # cv2.imshow("filtered", hough_img)

        # ----------------------------similar_lines 표시------------------------------
        # for idx, group in enumerate(R_similar):
        #     color_idx = idx % len(color)  # 색 개수보다 그룹이 많을 경우 반복 사용
        #     group_color = color[color_idx]

        #     x1, y1, x2, y2 = group['pos']
        #     cv2.line(hough_img, (x1, y1), (x2, y2), group_color, 2)
                

        # for idx, group in enumerate(L_similar):
        #     color_idx = idx % len(color)  # 색 개수보다 그룹이 많을 경우 반복 사용
        #     group_color = color[color_idx]

        #     x1, y1, x2, y2 = group['pos']
        #     cv2.line(hough_img, (x1, y1), (x2, y2), group_color, 2)

        # cv2.imshow ("similar", hough_img)


        # --------------------------- one lines 표시 ------------------------------------
        # if R_one_line:        

        #     x1, y1, x2, y2 = R_one_line['pos']
        #     cv2.line(hough_img, (x1, y1), (x2, y2), (0,0,255), 2)
                

        # if L_one_line:
        #     x1, y1, x2, y2 = L_one_line['pos']
        #     cv2.line(hough_img, (x1, y1), (x2, y2), (0,255,255), 2)  # green 또는 원하는 색


        # cv2.imshow ("one_line", hough_img)

        # ------------------------------ vanishing point ------------------------
        # cv2.imshow("Lane Detection", line_img)

        
        if cv2.waitKey(1) & 0xFF == ord('x'):
            rospy.signal_shutdown("Exit")
            cv2.destroyAllWindows()
            
        self.rate.sleep()
        
    
def hsv(roi):
    img_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([90, 78, 116])  #[85, 23, 100]
    upper_blue = np.array([105, 255, 159]) # [115, 163, 161]

    lower_white = np.array([0, 12, 208]) # 0, 12, 208
    upper_white = np.array([44, 63, 255]) # 44, 63, 255

    lower_yellow = np.array([17, 63, 185])
    upper_yellow = np.array([28, 151, 255])

    mask_blue = cv2.inRange(img_hsv, lower_blue, upper_blue)
    mask_white = cv2.inRange(img_hsv, lower_white, upper_white)
    mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

    mask1 = cv2.bitwise_or(mask_blue, mask_white)
    mask2 = cv2.bitwise_or(mask_yellow, mask_white)
    mask = cv2.bitwise_or(mask1, mask2)

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
    result = cv2.bitwise_and(roi, mask_bgr)
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(gray, (5, 5), 1)
    edges = cv2.Canny(blur,100, 200)
       
    return edges


def hough(roi, edges):
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=60, minLineLength=20, maxLineGap=10)

    if lines is None:
        return "no lines"
    return lines


def grouping(roi, lines): 
    group_list = []
    
    for line in lines:
        x1, y1, x2, y2 = line[0]

        slope = (y2 - y1)/(x2 - x1 + 1e-6) 
        angle = np.degrees(np.arctan(slope)) 
        y_intercept = y1 - slope * x1 
        # x_intercept = y_intercept / (slope + 1e-6) # 화면 아래 x값으로 설정해야. 
        x_under = (720 - y_intercept) / slope  # 화면 아래 x값

        if slope != 0:
        
            if (32 < abs(angle) < 70):  
                #print("x1: ",x1, ", y1: ",y1, ", x2: ", x2, ", y2: ", y2)
                similar = False
                for group in group_list:
               
                    if (group[0]['slope'] * slope) > 0 and abs(group[0]['slope']-slope) < 20.0 and abs(group[0]['x_under']-x_under) < 40:
                        group.append({'slope': slope, 'angle': angle, 'x_under': x_under,'y_intercept': y_intercept, 'pos': (x1, y1, x2, y2)})
                        similar = True
                        break 
                
                if not similar:
                    group_list.append([{'slope': slope, 'angle': angle, 'x_under': x_under, 'y_intercept': y_intercept, 'pos': (x1, y1, x2, y2)}])

    return group_list


def avglines(roi, group_list):
    avg_lines = []
    for group in group_list:
        if len(group) == 0:
            continue
        
        avg_slope = sum([line['slope'] for line in group]) / len(group)
        avg_x_under = sum([line['x_under'] for line in group]) / len(group)
        avg_y_intercept = sum([line['y_intercept'] for line in group]) / len(group)
        avg_angle = np.degrees(np.arctan(avg_slope))
        #x_under = (720 - avg_y_intercept) / avg_slope
        # print ("avg_x_under : ", avg_x_under)
        
        # roi 꽉 차게 선을 그리기 위한 설정
        x1, x2 = 0,roi.shape[1]
        y1 = int(avg_slope*x1 + avg_y_intercept)
        y2 = int(avg_slope*x2 + avg_y_intercept)

        # 실제 avg_line pos


        avg_lines.append({'slope': avg_slope, 'angle': avg_angle, 'x_under': avg_x_under,'y_intercept': avg_y_intercept, 'pos': (x1, y1, x2, y2)})
        # 여기서의 pos의 좌표는 
        # 1. 평균 slope, 평균 intercept으로 만들어낸 대표 직선의 두 끝점이라고 봐야 한다. 정확한 실제 차선 좌표가 아님.
        # 2. 실제 연산에 사용될 좌표가 아니라 러프하게 시각화하기 위한 좌표
    return avg_lines

# 여기서 과거 프레임과 비교하니깐 버스차선을 없애야 할 거 같은데, x_th 값을 아무리 높여도 안된다. 
# 뭘 놓치고 있지?

def filter_noise(roi, avg_lines, lane_detection, angle_th = 20, x_th = 200): #기준값 범위를 좁힐 필요성 있음
    filtered_Rlines = []
    filtered_Llines = []
    last_Rlines = lane_detection.last_Rlines 
    last_Llines = lane_detection.last_Llines
    # 하나의 프레임 안에 있는 여러 선들을 저장하고 있다. 
    # lane_detection 의 파라미터는 self 이므로, LaneDetection의 객체. 
    # LaneDetection 의 객체가 기억하고 있는 직전 프레임의 오른쪽, 왼쪽 선 리스트를 꺼내와라. 
    # 질문)
    # 직전 프레임의 선 리스트들을 언제 저장했지? 저장 안했음. 그럼 어디에 저장해야?
    # 질문)
    # 일단 여기서 비교하는 last 프레임의 선은 filtering 된 avg_lines. 
    # (ㄹㅇ 차선이라고 인정된 애들을 다음 프레임의 기준으로 쓰는 것)
    # (근데 이제 그 기준인 last_Rlines를 filtered 전체로 할지, 가장 실제 차선과 유사한 top1,2로 할지는 얼마나 빡세게 필터링할지에 따라 달라짐)

    R_Lines = []
    L_Lines = []

    for line in avg_lines: 
        # 한 그룹 당 한 평균 line 이 생성되어 avg_lines 리스트에 담겨있으니 
        # avg_lines를 순회하면 바로 line이 나오는 게 맞다
        angle, slope, x_under, y_intercept = line['angle'], line['slope'], line['x_under'], line['y_intercept']
        similar=False

        if slope > 0 : # 오른쪽 차선
            if last_Rlines: # 리스트가 비어있지 않으면 True
                for last_line in last_Rlines: #직전 차선과 비교
                    # x = (roi.shape[0] -last_line['y_intercept']) / last_line['slope'] # y=height 에서 x 좌표

                    if abs(last_line['angle'] - angle) <= angle_th and abs(last_line['x_under'] - x_under) <= x_th:

                        similar = True
                        # (상당히 빡센 필터링 - 한 프레임 안에서 검출되는 여러 평균 선들 중 하나의 선이라도 noise 범위에 해당하면 바로 노이즈 처리)
                        # 한 번 false 되면 다시 true로 복구 하지 않기 때문에, 과거 차선들 중 딱 한 개라도 기준에서 많이 벗어나면
                        # 그 현재 차선은 noise가 된다. 
                        # 점선은 필터링 되지 않고, 버스차선은 필터링되게끔하려면 파라미터값을 어떻게 조정해야할까?
            else:
                similar = True

            if similar: # 직전 프레임의 차선들이 현재 차선과 유사하다면, 
                #rospy.loginfo(f"Right: angle={angle}, x_under={x_under}")
                filtered_Rlines.append(line)

        else: #왼쪽 차선
            if last_Llines:  
                for last_line in last_Llines:
                    if abs(last_line['angle'] - angle) <= angle_th and abs(last_line['x_under'] - x_under) <= x_th:
                        similar = True
            else:
                similar = True

            if similar:
                #rospy.loginfo(f"Left: angle={angle}, x_under={x_under}")
                filtered_Llines.append(line)


    return filtered_Llines, filtered_Rlines 
    # 실제 차선과 x_under 값이 차이나는 선분들은 날라가고 ㄹㅇ 차선들만 담겨있는 리스트


# 좌,우 한 쪽씩 find_similar_lines() 적용됨
# R, L -> filter_Rlines, _Lines
# Rs = find_similar_lines(roi, R, last_Rlines)
def find_similar_lines(roi, current_lines, previous_lines):
    if not previous_lines: # last_Rlines
        return current_lines[:2] # 유사한 라인들(filtered_Rlines) 중에서 가장 previous와 가까운 top -2만 고르자
    
    similarities = []
    for curr_line in current_lines:
        for prev_line in previous_lines:
            angle_diff = abs(curr_line['angle'] - prev_line['angle'])
            x_under_diff = abs(curr_line['x_under'] - prev_line['x_under'])
            similarities.append((angle_diff + x_under_diff, curr_line))
            
    similarities.sort(key=lambda x: x[0])
    most_similar_lines = [line for _, line in similarities[:2]] 
    # similarities 값이 작은 순으로 정렬 (angle, x_under의 차이를 더한 것이기 때문에 차이가 작을 수록 비슷하다)
    # 가장 유사한 선 2개 선 추출
    
    return most_similar_lines


# 반환값 most_similar_lines 을 파라미터로 사용
def one_line(roi, lines):
    N = len(lines)
    
    if N == 2:
        # avg_slope = (lines[0]['slope'] + lines[1]['slope']) / 2
        # avg_y_intercept = (lines[0]['y_intercept'] + lines[1]['y_intercept']) / 2

        if lines[0]['slope'] > 0:
        # 오른쪽 차선 → x_under 작은 것이 안쪽
            inner = min(lines, key=lambda l: l['x_under'])
        else:
        # 왼쪽 차선 → x_under 큰 것이 안쪽
            inner = max(lines, key=lambda l: l['x_under'])

        avg_slope = inner['slope']
        avg_y_intercept = inner['y_intercept']

        y_val = []
        for line in lines:
            y_val.extend([line['pos'][1], line['pos'][3]]) 
            # pos는 roi상에 러프하게 그리는 용이라서 직접 계산에 쓰면 안됨.

        y1 = min(y_val) # 전체 직선 범위를 제대로 표현하기 위해 가장 위
        y2 = max(y_val) # 가장 아래

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
    
    slope1, y1 = line1['slope'], line1['y_intercept']
    slope2, y2 = line2['slope'], line2['y_intercept']
    
    #print(f"slope1: {slope1:.4f}, y1: {y1:.2f}, slope2: {slope2:.4f}, y2: {y2:.2f}")

    
    #if slope1 == slope2:
        #return None
    if abs(slope1-slope2) < 1e-3:
        return None
    
    vanish_x = (y2 - y1)/(slope1 - slope2)
    vanish_y = slope1 * vanish_x + y1
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
        
        ##print("Vanishing Point: ", point)


# R, L -> filter_Rlines, _Lines
# create_line(roi, R_filtered, L_filtered, self)
def create_line(roi, R, L, lane_detector):
    img_width = roi.shape[1]
    line_img = roi.copy()
    
    if R or L is not None:
        # filter_Lines=[]에 하나의 line 이라도 존재하면, 
        last_Rlines = lane_detector.last_Rlines
        last_Llines = lane_detector.last_Llines
        #last_ -> 직전 과거 프레임에서 저장해 둔 오른쪽/왼쪽 차선 후보들 = 노이즈를 제거한 베스트 라인들

        
        
        Rs = find_similar_lines(roi, R, last_Rlines)
        Ls = find_similar_lines(roi, L, last_Llines)
        
        R_one_line = one_line(roi, Rs)
        L_one_line = one_line(roi, Ls)
        
        ##print("R_one_line:", R_one_line)
        ##print("L_one_line:", L_one_line)
        
        if R_one_line != {} and L_one_line != {}:
            slope1, y1 = R_one_line['slope'], R_one_line['y_intercept']
            slope2, y2 = L_one_line['slope'], L_one_line['y_intercept']

            if abs(slope1 - slope2) > 1e-3:
                vanish_x = (y2 - y1) / (slope1 - slope2)
                vanish_y= slope1 * vanish_x + y1
                vanish = (int(vanish_x), int(vanish_y))

                # 오른쪽 선: 끝점을 소실점으로 수정
                x1_r, y1_r, _, _ = R_one_line['pos']
                x2_r = int(vanish_x)
                y2_r = int(vanish_y)
                # cv2.line(line_img, (x1_r, y1_r), (x2_r, y2_r), (0, 0, 255), 2)
                
                # 왼쪽 선: 끝점을 소실점으로 수정
                x1_l, y1_l, _, _ = L_one_line['pos']
                x2_l = int(vanish_x)
                y2_l = int(vanish_y)

                # cv2.line(line_img, (x1_l, y1_l), (x2_l, y2_l), (0, 0, 255), 2)
                # cv2.circle(line_img, vanish, 5, (0,255,0), -1)
                vanishing_point(line_img, R_one_line, L_one_line, lane_detector)

            elif R_one_line != {}:
                x1, y1, x2, y2 = R_one_line['pos']
                # cv2.line(line_img, (x1, y1), (x2, y2), (0, 0, 255), 2)

            elif L_one_line != {}:
                x3, y3, x4, y4 = L_one_line['pos']
        
                # cv2.line(line_img, (x3, y3), (x4, y4), (0, 0, 255), 2)
            
        lane_detector.last_Rlines = Rs
        lane_detector.last_Llines = Ls
 
    return line_img

if __name__ == '__main__':
    LaneDetection()
