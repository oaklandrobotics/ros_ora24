import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

UPPER_BOUND = 255
LOWER_BOUND = 120
KERNEL = np.ones((5, 5), np.uint8)

class LinePerception(Node):
    def __init__(self):
        super().__init__('line_perception')
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(Image, '/depth_camera/image_raw', self.img_callback, 10)

    def img_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
        #Masking
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        cv2.rectangle(mask, (0, 240), (1280, 720), 255, -1)
        masked_image = cv2.bitwise_and(img, img, mask=mask)
        #cv2.imshow('hi', masked_image)

        #Black and white
        bw = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

        #Erosion
        erosion = cv2.erode(bw, KERNEL, iterations=1)

        #Dilation
        dilate = cv2.dilate(erosion, KERNEL, iterations=1)

        #Thresholding
        _, threshold = cv2.threshold(dilate, LOWER_BOUND, UPPER_BOUND, cv2.THRESH_BINARY)

        #Gaussian Blur
        blur = cv2.GaussianBlur(threshold, (5, 5), 0)
        #cv2.imshow('fuck you', blur)

        #Canny edge detection
        edges = cv2.Canny(blur, 50, 150)

        #Find the lines and draw them
        """ lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=30, maxLineGap=10)
        print(len(lines))
        for line in lines:
            #print(line)
            x1, y1, x2, y2 = line[0]
            #print(f'({x1}, {y1}), ({x2}, {y2})')
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2) """

        #Display the things
        SCALE = 2
        height, width = img.shape[0:2]
        frame_resized = cv2.resize(img, (int(width / SCALE), int(height / SCALE)))
        edges_resized = cv2.resize(edges, (int(width / SCALE), int(height / SCALE)))
        edges_resized = cv2.cvtColor(edges_resized, cv2.COLOR_GRAY2BGR)  # Repeat the single channel along the third axis
        stack = np.hstack((frame_resized, edges_resized))
        cv2.imshow('stack', stack)
        #cv2.imshow('frame', frame)
        cv2.imshow('window', threshold)
    
        #TODO process threshold: white pixels -> query depth -> add to point cloud if within certain range of ground plane -> publish point cloud

        cv2.waitKey(1)

def main():
    rclpy.init()
    
    line_perception = LinePerception()

    rclpy.spin(line_perception)

    line_perception.destroy_node()  
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()