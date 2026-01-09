import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

img_data = None

def callback(msg):
    global img_data
    img_data = msg  # 최신 영상만 갱신

def on_trackbar_change(val):
    pass

def main():
    global img_data

    rospy.init_node("find_hsv", anonymous=True)
    rospy.Subscriber("/LaneCam", CompressedImage, callback)

    cv2.namedWindow("Image")

    # --- 트랙바는 단 한 번만 생성해야 함 ---
    cv2.createTrackbar("H_low", "Image", 0,   179, on_trackbar_change)
    cv2.createTrackbar("H_high", "Image", 179, 179, on_trackbar_change)
    cv2.createTrackbar("S_low", "Image", 0,   255, on_trackbar_change)
    cv2.createTrackbar("S_high", "Image", 255, 255, on_trackbar_change)
    cv2.createTrackbar("V_low", "Image", 0,   255, on_trackbar_change)
    cv2.createTrackbar("V_high", "Image", 255, 255, on_trackbar_change)

    rate = rospy.Rate(30)  # FPS 조절

    while not rospy.is_shutdown():
        if img_data is not None:
            # ROS 이미지 → OpenCV 이미지 변환
            np_arr = np.frombuffer(img_data.data, np.uint8)
            bgr_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            h, w = bgr_img.shape[:2]
            roi_img = bgr_img[int(h*1.3/3):int(h*0.8), :]

            # 트랙바 값 읽기
            h_low = cv2.getTrackbarPos("H_low", "Image")
            h_high = cv2.getTrackbarPos("H_high", "Image")
            s_low = cv2.getTrackbarPos("S_low", "Image")
            s_high = cv2.getTrackbarPos("S_high", "Image")
            v_low = cv2.getTrackbarPos("V_low", "Image")
            v_high = cv2.getTrackbarPos("V_high", "Image")

            # HSV 변환 및 마스크 적용
            hsv_image = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
            lower = np.array([h_low, s_low, v_low])
            upper = np.array([h_high, s_high, v_high])

            mask = cv2.inRange(hsv_image, lower, upper)
            result = cv2.bitwise_and(roi_img, roi_img, mask=mask)

            cv2.imshow("Image", result)

        if cv2.waitKey(1) == 27:
            break

        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

# import cv2
# import numpy as np
# import rospy
# from sensor_msgs.msg import CompressedImage

# img_data = None

# def callback(img):
#     global img_data
#     img_data = img

# def on_trackbar_change(val):
#     pass # 이 함수는 트랙바 값이 바뀔 때마다 호출됩니다. 여기서는 아무 작업도 수행하지 않습니다.

# def callback(img_data):
#     np_arr = np.frombuffer(img_data.data, np.uint8)
#     bgr_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#     h, w = bgr_img.shape[:2]
#     roi_img = bgr_img[int(h*1.3/3):int(h*0.8), :]
#     # image = cv2.imread('/home/autonav/Desktop/BEV_screenshot_28.07.2025.png')

#     # 윈도우 생성
#     cv2.namedWindow("Image")

#     # 트랙바 생성
#     cv2.createTrackbar("H_low", "Image", 0, 179, on_trackbar_change)
#     cv2.createTrackbar("H_high", "Image", 179, 179, on_trackbar_change)
#     cv2.createTrackbar("S_low", "Image", 0, 255, on_trackbar_change)
#     cv2.createTrackbar("S_high", "Image", 255, 255, on_trackbar_change)
#     cv2.createTrackbar("V_low", "Image", 0, 255, on_trackbar_change)
#     cv2.createTrackbar("V_high", "Image", 255, 255, on_trackbar_change)

#     while True:
#     # 트랙바 값을 읽어서 HSV 범위 설정
#         h_low = cv2.getTrackbarPos("H_low", "Image")
#         h_high = cv2.getTrackbarPos("H_high", "Image")
#         s_low = cv2.getTrackbarPos("S_low", "Image")
#         s_high = cv2.getTrackbarPos("S_high", "Image")
#         v_low = cv2.getTrackbarPos("V_low", "Image")
#         v_high = cv2.getTrackbarPos("V_high", "Image")

#         # HSV 범위로 마스크 생성
#         hsv_image = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
#         lower_bound = np.array([h_low, s_low, v_low])
#         upper_bound = np.array([h_high, s_high, v_high])
#         mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

#         # 마스크를 이용하여 원본 이미지에 색상 표시
#         result = cv2.bitwise_and(roi_img, roi_img, mask=mask)

#         # 결과 이미지 출력
#         cv2.imshow("Image", result)

#         # 키 입력 대기
#         key = cv2.waitKey(1)

#         if key == 27: # ESC 키를 누르면 종료
#             break

#     cv2.destroyAllWindows()

# def main():
#     rospy.init_node("find_hsv", anonymous=True)
#     rospy.Subscriber("/LaneCam", CompressedImage, callback)

#     rospy.spin()

# if __name__ == "__main__":
#     main()
