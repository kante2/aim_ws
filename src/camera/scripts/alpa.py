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

        # self.last_Rlines = []
        # self.last_Llines = []
        # self.last_R = {}
        # self.last_L = {}
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
        
        # roi = img_bgr[int(h*1.4/3):int(h*0.8), :] # [높이, 너비, 채널]
        roi = img_bgr[:int(h*0.8), :]

        edges = hsv(roi)
        hough_line = hough(roi, edges)
        grouping_line = grouping(roi, hough_line)
        avg_line = avglines(roi, grouping_line)
        R_one_line, L_one_line = one_line(roi, avg_line)
        # one_line() 은 라인이 없을 때 return {} 을 하기도 한다. -> {} 에 대한 처리도 해줘야

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


        # --------------------------- one lines 표시 ----------------------------------
        
        x1, y1, x2, y2 = R_one_line[0]['pos']
        cv2.line(hough_img, (x1, y1), (x2, y2), (0,0,255), 2)
                
        x1, y1, x2, y2 = L_one_line[0]['pos']
        cv2.line(hough_img, (x1, y1), (x2, y2), (0,255,255), 2) 


        cv2.imshow ("one_line", hough_img)


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
        
            if (34 < abs(angle) < 70):  
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



# avg_lines 를 바로 받아온다. 
# one_line()의 반환값이 오른쪽, 왼쪽 차선인 걸로 하자. 
def one_line(roi, avg_lines):
    R_one_line = []
    L_one_line = []
    N = len(avg_lines)
    
    if N > 0:
        # avg_slope = (avg_lines[0]['slope'] + avg_lines[1]['slope']) / 2
        # avg_y_intercept = (avg_lines[0]['y_intercept'] + avg_lines[1]['y_intercept']) / 2
        for line in avg_lines:
            # line : dictionary

            angle, slope, x_intercept, y_intercept = line['angle'], line['slope'], line['x_under'], line['y_intercept']

            if slope > 0:
                out_line = max(avg_lines, key=lambda l: l['x_under'])
                R_one_line.append(out_line)
            else:
                out_line = min(avg_lines, key=lambda l: l['x_under'])
                L_one_line.append(out_line)

        return R_one_line, L_one_line
    
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
def create_line(roi, lane_detector):
    img_width = roi.shape[1]
    line_img = roi.copy()
    
    # if R or L is not None:
    last_Rlines = lane_detector.last_Rlines
    last_Llines = lane_detector.last_Llines
    
    Rs = avglines(roi, last_Rlines)
    Ls = avglines(roi, last_Llines)
    # avglines는 R,L 구분X
    
    R_one_line = one_line(roi, Rs)
    L_one_line = one_line(roi, Ls)
    
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
        
    return line_img

if __name__ == '__main__':
    LaneDetection()
