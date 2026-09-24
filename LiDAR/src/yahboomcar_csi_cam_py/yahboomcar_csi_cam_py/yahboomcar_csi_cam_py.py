import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import base64
import cv2
from cv_bridge import CvBridge
import math
import io
import numpy as np
import json
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import os
import time
from hobot_vio import libsrcampy as srcampy


try:
    import simplejpeg
except ImportError:
    simplejpeg = None
    try:
        import cv2
        print("simplejpeg not found. Falling back to cv2 for JPEG encoding.")
    except ImportError:
        cv2 = None
        try:
            import PIL
            from PIL import Image
            print("simplejpeg not found. Falling back to PIL for JPEG encoding.")
        except ImportError:
            PIL = None



class Mipi_Camera(object):
    
    def __init__(self, width=320, height=240, debug=False):
        self.__debug = debug
        self.__state = False
        self.__width = width
        self.__height = height

        self.__camera = srcampy.Camera()
        self.__sensor_reset_shell()
        # open_cam(pipe_id, video_index, fps=30, width=1920, height=1080)
        error = self.__camera.open_cam(1, 1, 30, self.__width, self.__height)
        if error == 0:
            self.__state = True
            if self.__debug:
                print("Open CSI Camera OK")
        else:
            self.__state = False
            if self.__debug:
                print("Fail to Open CSI Camera")

    def __del__(self):
        self.__clear()
        if self.__debug:
                print("---Mipi_Camera Del---")

    # 配置CSI摄像头
    def __sensor_reset_shell(self):
        os.system("echo 19 > /sys/class/gpio/export")
        os.system("echo out > /sys/class/gpio/gpio19/direction")
        os.system("echo 0 > /sys/class/gpio/gpio19/value")
        time.sleep(0.2)
        os.system("echo 1 > /sys/class/gpio/gpio19/value")
        os.system("echo 19 > /sys/class/gpio/unexport")
        os.system("echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart")

    # nv12图像转化成bgr图像
    def __nv12_to_bgr_opencv(self, image, width, height):
        frame = np.frombuffer(image , dtype=np.uint8)
        img_bgr = cv2.cvtColor(frame.reshape((height * 3 // 2, width)), cv2.COLOR_YUV2BGR_NV12)
        return img_bgr
    
    # 摄像头是否打开成功
    # Check whether the camera is enabled successfully
    def isOpened(self):
        return self.__state

    # 释放摄像头 Release the camera
    def __clear(self):
        if self.__state:
            self.__camera.close_cam()
            self.__state = False

    # 获取摄像头的一帧图片 
    # Gets a frame of the camera
    def get_frame(self):
        image = self.__camera.get_img(2)
        if image is None:
            return False, bytes({1})
        image = self.__nv12_to_bgr_opencv(image, self.__width, self.__height)
        success = True
        return success, image

    # 获取摄像头的jpg图片 
    # Gets the JPG image of the camera
    def get_frame_jpg(self, text="", color=(0, 255, 0)):
        image = self.__camera.get_img(2)
        if image is None:
            return False, bytes({1})
        image = self.__nv12_to_bgr_opencv(image, self.__width, self.__height)
        success = True
        if text != "":
            # 各参数依次是：图片，添加的文字，左上角坐标，字体，字体大小，颜色，字体粗细
            # The parameters are: image, added text, top left coordinate, font, font size, color, font size  
            cv2.putText(image, str(text), (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        success, jpeg = cv2.imencode('.jpg', image)
        return success, jpeg.tobytes()


    # 获取摄像头的一帧图片 
    # Gets a frame of the camera
    def read(self):
        image = self.__camera.get_img(2)
        if image is None:
            return False, bytes({1})
        image = self.__nv12_to_bgr_opencv(image, self.__width, self.__height)
        success = True
        return success, image
    
    # 释放摄像头设备总线 
    # Release the camera
    def release(self):
        self.__clear()






class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.frame_mask_pub = self.create_publisher(CompressedImage, '/csi/image_raw/compressed', 100)
        self.cv_bridge = CvBridge()

    def run(self, img):
        
        compressed_image_msg = self.cv_bridge.cv2_to_compressed_imgmsg(img, dst_format='jpeg')
        self.frame_mask_pub.publish(compressed_image_msg)
        return img
        

    def process_image(self, msg):
        cv_img = self.run(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    img_width = 640
    img_height = 480
    g_debug = True
    camera = Mipi_Camera(width=img_width, height=img_height, debug=g_debug)
    while rclpy.ok():
        #ret, frame = cap.read()
        ret, img = camera.get_frame()
        if ret:
            node.process_image(img)
    cap.release()
    node.destroy_node() 
    rclpy.shutdown()
