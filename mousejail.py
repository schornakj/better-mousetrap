import numpy as np
import cv2
import serial
import time
import subprocess

def main():
    print("Hello")
    # arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=.5)
    cv2.waitKey(1000)  # give the connection a second to settle

    command = 'v4l2-ctl -d /dev/video1 -c focus_auto=0'
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    cv2.waitKey(100)
    command = 'v4l2-ctl -d /dev/video1 -c focus_absolute=40'
    process1 = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    cv2.waitKey(100)
    command = 'v4l2-ctl -d /dev/video1 -c exposure_auto=1'
    process2 = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    cv2.waitKey(100)
    command = 'v4l2-ctl -d /dev/video1 -c exposure_absolute=255'
    process2 = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    cv2.waitKey(100)



    cap = cv2.VideoCapture(1)
    cv2.waitKey(1000)
    ret, frame = cap.read()
    while cap.isOpened():
        if cv2.waitKey(10) == ord('q') or frame is None:
            break
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hsv_range_min = np.array([90, 50, 40])
        hsv_range_max = np.array([180, 120, 200])

        # mask = cv2.inRange(gray, 0, 75)

        mask = cv2.inRange(hsv, hsv_range_min, hsv_range_max)

        kernel = np.ones((15, 15), np.uint8)
        mask_opened = cv2.erode(cv2.dilate(mask, kernel, iterations=1), kernel, iterations=1)

        img, contours, hierarchy = cv2.findContours(mask_opened, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


        frame_annotated = frame

        if len(contours) > 0:
            areas = []
            for i, c in enumerate(contours):
                area = cv2.contourArea(c)
                areas.append(area)
            contours_sorted = sorted(zip(areas, contours), key=lambda x: x[0], reverse=True)
            contour_largest = contours_sorted[0][1]

            M = cv2.moments(contour_largest)
            # print(M)
            if M['m00'] > 0.01:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                area = cv2.contourArea(contour_largest)
                frame_annotated = draw_target_marker(frame, (cx, cy))
                cv2.drawContours(frame_annotated, [contour_largest], 0, (0, 255, 0), 3)
                if area > 200:
                    print("Found a mouselike object with area " + str(area))
        # cv2.waitKey(100)
        # time.sleep(1/30)
        cv2.imshow('camera', frame_annotated)
        cv2.imshow('mask', mask_opened)
        # arduino.write("go")
        #
        # data = arduino.readline()
        # if data:
        #     data.rstrip('\n')  # strip out the new lines for now
        #     # (better to do .read() in the long run for this reason
        #     print("Received: " + str(data))
        # time.sleep(3)

def draw_target_marker(image, target_coords):
    output = image.copy()
    cv2.circle(output, target_coords, 10, (0, 255, 0))
    return output

# def get_dense_flow(image_past, image_current):
#     flow = cv2.calcOpticalFlowFarneback(cv2.cvtColor(image_past, cv2.COLOR_BGR2GRAY),
#                                         cv2.cvtColor(image_current, cv2.COLOR_BGR2GRAY),
#                                         None,
#                                         self.flow_params[0], self.flow_params[1], self.flow_params[2],
#                                         self.flow_params[3], self.flow_params[4], self.flow_params[5],
#                                         self.flow_params[6])
#     flow_magnitude, flow_angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])
#
#     hsv = np.zeros_like(image_current)
#     hsv[..., 1] = 255
#
#     hsv[..., 0] = (flow_angle * (180 / np.pi) - 90) * 0.5
#     hsv[..., 2] = cv2.normalize(flow_magnitude, None, 0, 255, cv2.NORM_MINMAX)
#     bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
#     return hsv, bgr



if __name__ == '__main__':
    main()