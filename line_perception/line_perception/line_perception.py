import rclpy
import message_filters
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
        self.img_raw_sub = message_filters.Subscriber(self, Image, '/depth_camera/image_raw')
        self.img_depth_sub = message_filters.Subscriber(self, Image, '/depth_camera/depth/image_raw')
        self.img_filtered_pub = self.create_publisher(Image, '/depth_camera/depth/image_raw_filtered', 10)
        self.ts = message_filters.ApproximateTimeSynchronizer((self.img_raw_sub, self.img_depth_sub), 10, slop=0.1)
        self.ts.registerCallback(self.img_callback)
        self.get_logger().info('Hello from line_perception')

    def img_callback(self, img_raw: Image, img_depth: Image):
        cv_raw = self.bridge.imgmsg_to_cv2(img_raw, 'bgr8')
        cv_depth = self.bridge.imgmsg_to_cv2(img_depth, '32FC1')

        #Masking
        mask = np.zeros(cv_raw.shape[:2], dtype=np.uint8)
        cv2.rectangle(mask, (0, 240), (1280, 720), 255, -1)
        masked_image = cv2.bitwise_and(cv_raw, cv_raw, mask=mask)
        #cv2.imshow('hi', masked_image)

        #Black and white
        bw = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

        #TODO clean up the processing pipeline
        #Erosion
        erosion = cv2.erode(bw, KERNEL, iterations=1)

        #Dilation
        dilate = cv2.dilate(erosion, KERNEL, iterations=1)

        #Thresholding
        _, threshold = cv2.threshold(dilate, LOWER_BOUND, UPPER_BOUND, cv2.THRESH_BINARY)

        #Gaussian Blur
        blur = cv2.GaussianBlur(threshold, (5, 5), 0)

        #Canny edge detection
        edges = cv2.Canny(blur, 50, 150)

        #use threshold as a mask for the depth image
        masked = cv2.bitwise_and(cv_depth, cv_depth, mask=threshold)

        #Display the things
        SCALE = 2
        height, width = cv_raw.shape[0:2]
        raw_resized = cv2.resize(cv_raw, (int(width / SCALE), int(height / SCALE)))
        masked_resized = cv2.resize(masked, (int(width / SCALE), int(height / SCALE)))
        masked_resized = cv2.cvtColor(masked_resized, cv2.COLOR_GRAY2BGR)
        #cv2.imshow('raw', raw_resized)
        #cv2.imshow('masked', masked_resized)
        #TODO display the images side by side in one window, seems to be a problem with the hconcat function
        #stack = np.hstack((raw_resized, masked_resized))
        #stack = cv2.hconcat([raw_resized, masked_resized])
        #cv2.imshow('stack', stack)
        cv2.waitKey(1)

        #Publish the filtered image
        img_filtered = Image()
        img_filtered = self.bridge.cv2_to_imgmsg(masked, '32FC1')
        img_filtered.header = img_depth.header
        self.img_filtered_pub.publish(img_filtered)

def main():
    rclpy.init()
    
    line_perception = LinePerception()

    rclpy.spin(line_perception)

    line_perception.destroy_node()  
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()