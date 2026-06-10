import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image as SensorImage, NavSatFix
from tf2_geometry_msgs import TransformStamped
from tf2_ros import TransformBroadcaster
from PIL import Image as PILImage
import numpy as np
from rclpy.executors import ExternalShutdownException
import math

EARTH_RADIUS = 6378137.0 # in meters
PATH = "/home/triffid/hua_ws/gesture_module_v2"
FPS = 1.0

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def abs_xy_to_gps(x, y) -> tuple[float]:
    '''
    abs_xy -> GPS
    Parameters:
        x,y:        With origin the initial robot position and "orientation" the same with the "flatten" meridians/parallels (in mm)
    Returns:
        - longitude:  GPS (degrees)
        - latitude:   GPS (degrees)
    '''
    lat = ((y/1000) / EARTH_RADIUS) * (180.0 / math.pi)
    lon = ((x/1000) / (EARTH_RADIUS * math.cos(math.radians(0)))) * (180.0 / math.pi)
    return float(lon), float(lat)

class Producer(Node):

    def __init__(self):
        super().__init__("producer_node")
        
        self.__color_publisher=self.create_publisher(
            msg_type = SensorImage,
            topic = "/b2/camera_front_435i/realsense_front_435i/color/image_raw_test",
            qos_profile = 10
        )
        self.__depth_publisher=self.create_publisher(
            msg_type = SensorImage,
            topic = "/b2/camera_front_435i/realsense_front_435i/depth/image_rect_raw_test",
            qos_profile = 10
        )
        self.__info_publisher=self.create_publisher(
            msg_type=CameraInfo,
            topic="/b2/camera_front_435i/realsense_front_435i/color/camera_info_test",
            qos_profile = 10
        )
        self.__gps_publisher=self.create_publisher(
            msg_type=NavSatFix,
            topic="/fix_test",
            qos_profile = 10
        )
        self.__broadcaster = TransformBroadcaster(self)

        # self.__rgb_frames = [
        #     "frames/high_Come-to-me_2_color.png",
        #     "frames/high_Come-to-me_98_color.png",
        #     "frames/high_Come-to-me_1214_color.png",
        #     "frames/high_Emergency-situation_101_color.png",
        #     "frames/high_Evacuate-the-area_175_color.png",
        #     "frames/high_Fetch-a-gas-mask_49_color.png",
        #     "frames/high_Fetch-a-gas-mask_181_color.png",
        #     "frames/high_Fetch-a-shovel_33_color.png",
        #     "frames/high_Freeze_16_color.png",
        #     "frames/high_Freeze_40_color.png",
        #     "frames/high_Freeze_184_color.png",
        #     "frames/high_Ok-to-go_203_color.png",
        #     "frames/high_Ok-to-go_263_color.png",
        #     "frames/STOP_high_16_color.png",
        #     "frames/STOP_high_90_color.png",   
        #     "frames/multi_person.png",
        #     "frames/no_person.png",
        # ]

        # self.__depth_frames = [
        #     "frames/high_Come-to-me_2_depth.png",
        #     "frames/high_Come-to-me_98_depth.png",
        #     "frames/high_Come-to-me_1214_depth.png",
        #     "frames/high_Emergency-situation_101_depth.png",
        #     "frames/high_Evacuate-the-area_175_depth.png",
        #     "frames/high_Fetch-a-gas-mask_49_depth.png",
        #     "frames/high_Fetch-a-gas-mask_181_depth.png",
        #     "frames/high_Fetch-a-shovel_33_depth.png",
        #     "frames/high_Freeze_16_depth.png",
        #     "frames/high_Freeze_40_depth.png",
        #     "frames/high_Freeze_184_depth.png",
        #     "frames/high_Ok-to-go_203_depth.png",
        #     "frames/high_Ok-to-go_263_depth.png",
        #     "frames/STOP_high_16_depth.png",
        #     "frames/STOP_high_90_depth.png",
        #     "frames/high_Ok-to-go_263_depth.png",   # dummy
        #     "frames/high_Ok-to-go_263_depth.png",   # dummy
        # ]

        self.__rgb_frames = [
            
            "frames/multi_person.png", # dummy
            "frames/multi_person.png", # dummy
            "frames/multi_person.png", # dummy
            "frames/multi_person.png", # dummy
            
            "frames/high_Come-to-me_338_color.png",
            "frames/high_Come-to-me_338_color.png",

            "frames/multi_person.png", # dummy

            "frames/high_Come-to-me_338_color.png", # 4+1 successive
            "frames/high_Come-to-me_338_color.png",
            "frames/high_Come-to-me_338_color.png",
            "frames/high_Come-to-me_338_color.png",
            "frames/high_Come-to-me_338_color.png",

            "frames/multi_person.png", # dummy, low confidence

            "frames/high_Come-to-me_338_color.png", # 2 successive
            "frames/high_Come-to-me_338_color.png",

            "frames/multi_person.png", # dummy, low confidence

            "frames/high_Come-to-me_338_color.png", # 1 single

            "frames/multi_person.png", # dummy, low confidence

            "frames/high_Come-to-me_338_color.png", # 4+1 successive
            "frames/high_Come-to-me_338_color.png",
            "frames/high_Come-to-me_338_color.png",
            "frames/high_Come-to-me_338_color.png",
            "frames/high_Come-to-me_338_color.png",

            "frames/multi_person.png", # dummy, low confidence

            "frames/high_Come-to-me_338_color.png", # 4+1 successive
            "frames/high_Come-to-me_338_color.png",
            "frames/high_Come-to-me_338_color.png",
            "frames/high_Come-to-me_338_color.png",
            "frames/high_Come-to-me_338_color.png",

            "frames/high_Emergency-situation_341_color.png", # 4 + 1
            "frames/high_Emergency-situation_341_color.png",
            "frames/high_Emergency-situation_341_color.png",
            "frames/high_Emergency-situation_341_color.png",
            "frames/high_Emergency-situation_341_color.png",

            "frames/multi_person.png", # dummy, low confidence

            "frames/high_Emergency-situation_341_color.png",

            "frames/high_Fetch-a-gas-mask_337_color.png",
            "frames/high_Fetch-a-gas-mask_337_color.png",
            "frames/high_Fetch-a-gas-mask_337_color.png",
            "frames/high_Fetch-a-gas-mask_337_color.png",

            "frames/high_Fetch-a-shovel_357_color.png",
            "frames/high_Fetch-a-shovel_357_color.png",
            "frames/high_Fetch-a-shovel_357_color.png",
            "frames/high_Fetch-a-shovel_357_color.png",

            "frames/high_Fetch-an-axe_346_color.png",
            "frames/high_Fetch-an-axe_346_color.png",
            "frames/high_Fetch-an-axe_346_color.png",
            "frames/high_Fetch-an-axe_346_color.png",

            "frames/high_Freeze_340_color.png",
            "frames/high_Freeze_340_color.png",
            "frames/high_Freeze_340_color.png",
            "frames/high_Freeze_340_color.png",
        
            "frames/high_I-lost-connection_344_color.png",
            "frames/high_I-lost-connection_344_color.png",
            "frames/high_I-lost-connection_344_color.png",
            "frames/high_I-lost-connection_344_color.png",

            "frames/high_I-need-help_342_color.png",
            "frames/high_I-need-help_342_color.png",
            "frames/high_I-need-help_342_color.png",
            "frames/high_I-need-help_342_color.png",

            "frames/high_Move-away-from-here_348_color.png",
            "frames/high_Move-away-from-here_348_color.png",
            "frames/high_Move-away-from-here_348_color.png",
            "frames/high_Move-away-from-here_348_color.png",

            "frames/high_Ok-to-go_347_color.png",
            "frames/high_Ok-to-go_347_color.png",
            "frames/high_Ok-to-go_347_color.png",
            "frames/high_Ok-to-go_347_color.png",

            "frames/high_Operation-finished_339_color.png",
            "frames/high_Operation-finished_339_color.png",
            "frames/high_Operation-finished_339_color.png",
            "frames/high_Operation-finished_339_color.png",

            "frames/STOP_high_16_color.png",
            "frames/STOP_high_16_color.png",
            "frames/STOP_high_16_color.png",
            "frames/STOP_high_16_color.png",
            "frames/STOP_high_16_color.png",
            "frames/STOP_high_16_color.png",
            "frames/STOP_high_16_color.png",
            "frames/STOP_high_16_color.png",
            "frames/STOP_high_16_color.png",
            "frames/STOP_high_16_color.png",
            "frames/STOP_high_16_color.png",
            "frames/STOP_high_16_color.png",
            "frames/STOP_high_16_color.png",
            "frames/STOP_high_16_color.png",

        ]

        self.__depth_frames = ["frames/high_Come-to-me_338_depth.png"] * len(self.__rgb_frames)

        self.__total = len(self.__rgb_frames)
        assert len(self.__depth_frames) == self.__total

        self.__x_mm = 0.0
        self.__idx = 0

        self.__timer = self.create_timer(1/FPS, self.publish)

    def publish(self, path=PATH):
        if self.__idx >= len(self.__rgb_frames):
            return
        depth_path = f"{path}/{self.__depth_frames[self.__idx]}"
        color_path = f"{path}/{self.__rgb_frames[self.__idx]}"
        self.get_logger().info(f"Publishing {color_path} and {depth_path}...")
        # self.__idx = (self.__idx + 1) % self.__total
        self.__idx += 1

        depth = np.asarray(PILImage.open(depth_path),dtype=np.uint16)
        color = np.asarray(PILImage.open(color_path).convert("RGB"))
        
        stamp = self.get_clock().now().to_msg()

        q = euler_to_quaternion(roll=0, pitch=0, yaw=np.pi/2)
        base_to_map = TransformStamped()
        base_to_map.header.stamp = stamp
        base_to_map.header.frame_id = 'map'
        base_to_map.child_frame_id = 'base_link'
        base_to_map.transform.translation.x = float(self.__x_mm / 1000.0)
        base_to_map.transform.translation.y = 0.0
        base_to_map.transform.translation.z = 0.0
        base_to_map.transform.rotation.x = float(q[0].item())
        base_to_map.transform.rotation.y = float(q[1].item())
        base_to_map.transform.rotation.z = float(q[2].item())
        base_to_map.transform.rotation.w = float(q[3].item())
        self.__broadcaster.sendTransform(base_to_map)

        camera_to_base = TransformStamped()
        camera_to_base.header.stamp = stamp
        camera_to_base.header.frame_id = "base_link"
        camera_to_base.child_frame_id = "camera_depth_frame"
        camera_to_base.transform.translation.x = 0.0
        camera_to_base.transform.translation.y = 0.0
        camera_to_base.transform.translation.z = 0.0
        q_camera = euler_to_quaternion(roll=0, pitch=np.pi/2, yaw=0)
        camera_to_base.transform.rotation.x = float(q_camera[0].item())
        camera_to_base.transform.rotation.y = float(q_camera[1].item())
        camera_to_base.transform.rotation.z = float(q_camera[2].item())
        camera_to_base.transform.rotation.w = float(q_camera[3].item())
        self.__broadcaster.sendTransform(camera_to_base)

        msg = SensorImage()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_depth_frame"
        msg.height = depth.shape[0]
        msg.width = depth.shape[1]
        msg.encoding = "16UC1"
        msg.is_bigendian = False
        msg.step = 2 * depth.shape[1]
        msg.data = depth.tobytes()
        self.__depth_publisher.publish(msg)

        msg = SensorImage()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_depth_frame"
        msg.height = color.shape[0]
        msg.width = color.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * color.shape[1]
        msg.data = color.tobytes()
        self.__color_publisher.publish(msg)

        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_depth_frame"
        msg.height = color.shape[0]
        msg.width = color.shape[1]
        msg.k = [500.0, 0.0, 640.0, 0.0, 500.0, 360.0, 0.0, 0.0, 1.0]
        self.__info_publisher.publish(msg)

        msg = NavSatFix()
        msg.header.stamp = stamp
        (msg.longitude, msg.latitude) = abs_xy_to_gps(x=self.__x_mm,y=0)
        self.__gps_publisher.publish(msg)

        self.__x_mm += 1000


def main():
    try:
        rclpy.init()
        rclpy.spin(node=Producer())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    main()
