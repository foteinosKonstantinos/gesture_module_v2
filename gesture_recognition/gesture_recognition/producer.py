import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import CameraInfo, Image as SensorImage, NavSatFix
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import TransformStamped
from tf2_ros import TransformBroadcaster
from PIL import Image as PILImage
import numpy as np
from rclpy.executors import ExternalShutdownException
import math
import abc

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


class Test(abc.ABC):
    @abc.abstractmethod
    def get_rgb_frame(self, timestep:int) -> list[str]:pass     # timestep >= 0, relative path
    @abc.abstractmethod
    def get_depth_frame(self, timestep:int) -> list[str]:pass   # timestep >= 0, relative path
    @abc.abstractmethod
    def get_xy(self, timestep:int) -> tuple[int]:pass           # in mm, must be floats (ROS requirement)
    @abc.abstractmethod
    def get_orientation(self, timestep:int) -> float:pass       # degrees, must be float (ROS requirement), 0<=omega<360
    @abc.abstractmethod
    def finished(self, timestep:int) -> bool:pass

    def generate_full_trajectory(self, max_timestep=math.inf) -> dict[list]:
        timestep = 0
        trajectory = {
            "rgb_frames": [],
            "depth_frames": [],
            "xy (mm)": [],
            "omega (deg)": []
        }
        while not self.finished(timestep) and timestep<=max_timestep:
            trajectory["rgb_frames"].append(self.get_rgb_frame(timestep))
            trajectory["depth_frames"].append(self.get_depth_frame(timestep))
            trajectory["xy (mm)"].append(self.get_xy(timestep))
            trajectory["omega (deg)"].append(self.get_orientation(timestep))
            timestep += 1
        return trajectory


class Filter_Test(Test):
    def __init__(self):
        self.__rgb_frames = [
                    
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
                    "frames/multi_person.png", # dummy
                    "frames/multi_person.png", # dummy
                    "frames/multi_person.png", # dummy
                    "frames/multi_person.png", # dummy
        
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
    def get_rgb_frame(self, timestep:int):
        return self.__rgb_frames[timestep]
    def get_depth_frame(self, timestep:int):
        return self.__depth_frames[timestep]
    def get_xy(self, timestep):
        return (0.0, 0.0)
    def get_orientation(self, timestep):
        return 0.0
    def finished(self, timestep):
        return timestep >= len(self.__rgb_frames)

class Classification_Test(Test):
    def __init__(self):
        self.__rgb_frames = ['frames/high_Move-away-from-here_348_color.png', 'frames/high_Evacuate-the-area_343_color.png', 
                             'frames/high_Ok-to-go_347_color.png', 'frames/high_Emergency-situation_341_color.png', 
                             'frames/high_Freeze_340_color.png', 'frames/high_Operation-finished_339_color.png', 
                             'frames/high_I-lost-connection_344_color.png', 'frames/high_I-need-help_342_color.png', 
                             'frames/high_Come-to-me_338_color.png', 'frames/high_Fetch-an-axe_346_color.png', 
                             'frames/high_Fetch-a-gas-mask_337_color.png', 'frames/STOP_high_16_color.png', 
                             'frames/high_Fetch-a-shovel_357_color.png', 'frames/multi_person.png'] # the last image is negative
        self.__depth_frames = ['frames/high_Move-away-from-here_348_depth.png', 'frames/high_Evacuate-the-area_343_depth.png', 
                             'frames/high_Ok-to-go_347_depth.png', 'frames/high_Emergency-situation_341_depth.png', 
                             'frames/high_Freeze_340_depth.png', 'frames/high_Operation-finished_339_depth.png', 
                             'frames/high_I-lost-connection_344_depth.png', 'frames/high_I-need-help_342_depth.png', 
                             'frames/high_Come-to-me_338_depth.png', 'frames/high_Fetch-an-axe_346_depth.png', 
                             'frames/high_Fetch-a-gas-mask_337_depth.png', 'frames/STOP_high_16_depth.png', 
                             'frames/high_Fetch-a-shovel_357_depth.png', 'frames/high_Fetch-a-shovel_357_depth.png'] # the last depth map is dummy
    def get_rgb_frame(self, timestep):
        return self.__rgb_frames[timestep]
    def get_depth_frame(self, timestep):
        return self.__depth_frames[timestep]
    def get_xy(self, timestep):
        return (0.0, 0.0)
    def get_orientation(self, timestep):
        return 0.0
    def finished(self, timestep):
        return timestep >= len(self.__rgb_frames)

class Action_Test(Test):
    def __init__(self):
        self.__pairs = [
            ('frames/high_Freeze_340_color.png', 'frames/high_Freeze_340_depth.png'),                           # FREEZE
            ('frames/high_Ok-to-go_347_color.png', 'frames/high_Ok-to-go_347_depth.png'),                       # UNFREEZE (OK-TO-GO)
            ('frames/high_Move-away-from-here_348_color.png', 'frames/high_Move-away-from-here_348_depth.png'), # MOVE-AWAY-FROM-HERE
            ('frames/high_Evacuate-the-area_343_color.png', 'frames/high_Evacuate-the-area_343_depth.png'),     # EVACUATE-THE-AREA
            ('frames/high_Operation-finished_339_color.png', 'frames/high_Operation-finished_339_depth.png'),   # OPERATION-FINISHED
            ('frames/high_I-lost-connection_344_color.png', 'frames/high_I-lost-connection_344_depth.png'),     # I-LOST-CONNECTION
            ('frames/high_I-need-help_342_color.png', 'frames/high_I-need-help_342_depth.png'),                 # I-NEED-HELP
            ('frames/high_Come-to-me_338_color.png', 'frames/high_Come-to-me_338_depth.png'),                   # COME-TO-ME
            ('frames/high_Fetch-an-axe_346_color.png', 'frames/high_Fetch-an-axe_346_depth.png'),               # FETCH-AN-AXE
            ('frames/high_Fetch-a-gas-mask_337_color.png', 'frames/high_Fetch-a-gas-mask_337_depth.png'),       # FETCH-A-GAS-MASK
            ('frames/STOP_high_16_color.png', 'frames/STOP_high_16_depth.png'),                                 # STOP                  "cancels all running actions except emergency, then auto-clears after ~1 s."
            ('frames/high_Fetch-a-shovel_357_color.png', 'frames/high_Fetch-a-shovel_357_depth.png'),           # FETCH-A-SHOVEL
            ('frames/high_Emergency-situation_341_color.png', 'frames/high_Emergency-situation_341_depth.png'), # EMERGENCY-SITUATION   Cancels everything
            ('frames/high_Fetch-a-shovel_357_color.png', 'frames/high_Fetch-a-shovel_357_depth.png'),           # FETCH-A-SHOVEL        Should be rejected
        ]
    def get_rgb_frame(self, timestep):
        return self.__pairs[timestep][0]
    def get_depth_frame(self, timestep):
        return self.__pairs[timestep][1]
    def get_xy(self, timestep):
        return (0.0, 0.0)
    def get_orientation(self, timestep):
        return 0.0
    def finished(self, timestep):
        return timestep >= len(self.__pairs)

class Localization_Parabola_Test(Test):
    def __init__(self):
        self.__rgb_frame = 'frames/high_Freeze_340_color.png'
        self.__depth_frame = 'frames/high_Freeze_340_depth.png'
    def get_rgb_frame(self, timestep):
        return self.__rgb_frame
    def get_depth_frame(self, timestep):
        return self.__depth_frame
    def get_xy(self, timestep):
        return (timestep * 1000.0, timestep**2 * 1000.0) # mm
    def get_orientation(self, timestep):
        return 0.0
    def finished(self, timestep):
        return False

class Localization_Line_Test(Test):
    def __init__(self):
        self.__rgb_frame = 'frames/high_Freeze_340_color.png'
        self.__depth_frame = 'frames/high_Freeze_340_depth.png'
    def get_rgb_frame(self, timestep):
        return self.__rgb_frame
    def get_depth_frame(self, timestep):
        return self.__depth_frame
    def get_xy(self, timestep):
        return (timestep * 1000.0, timestep * 1000.0) # mm
    def get_orientation(self, timestep):
        return 360.0-45.0 # degress, perpedincular to the line
    def finished(self, timestep):
        return False


class Producer(Node):

    def __init__(self, test_scenario:Test, fps:float=FPS):
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
        # self.__odo_publisher=self.create_publisher(
        #     msg_type=Odometry,
        #     topic="/dog_odom_test",
        #     qos_profile = 10
        # )
        self.__heading_publisher = self.create_publisher(
            msg_type=Float32,
            topic = "/b2/nicla/magnetometer/heading_test",
            qos_profile=10
        )
        # self.__broadcaster = TransformBroadcaster(self)

        self.__scenario = test_scenario

        self.__timestep = 0
        self.__timer = self.create_timer(1/fps, self.publish)

    def publish(self, path=PATH):
        if self.__scenario.finished(self.__timestep):
            self.get_logger().info("\033[1;102mTEST FINISHED\033[0;0m")
            return

        depth_path = f"{path}/{self.__scenario.get_depth_frame(self.__timestep)}"
        depth = np.asarray(PILImage.open(depth_path),dtype=np.uint16)
        color_path = f"{path}/{self.__scenario.get_rgb_frame(self.__timestep)}"
        color = np.asarray(PILImage.open(color_path).convert("RGB"))

        x_mm, y_mm = self.__scenario.get_xy(self.__timestep)
        omega = self.__scenario.get_orientation(self.__timestep)
        
        stamp = self.get_clock().now().to_msg()

        # q = euler_to_quaternion(roll=0, pitch=0, yaw=np.pi/2)
        # base_to_map = TransformStamped()
        # base_to_map.header.stamp = stamp
        # base_to_map.header.frame_id = 'map'
        # base_to_map.child_frame_id = 'base_link'
        # base_to_map.transform.translation.x = float(x_mm / 1000.0)
        # base_to_map.transform.translation.y = float(y_mm / 1000.0)
        # base_to_map.transform.translation.z = 0.0
        # base_to_map.transform.rotation.x = float(q[0].item())
        # base_to_map.transform.rotation.y = float(q[1].item())
        # base_to_map.transform.rotation.z = float(q[2].item())
        # base_to_map.transform.rotation.w = float(q[3].item())
        # self.__broadcaster.sendTransform(base_to_map)

        # camera_to_base = TransformStamped()
        # camera_to_base.header.stamp = stamp
        # camera_to_base.header.frame_id = "base_link"
        # camera_to_base.child_frame_id = "camera_depth_frame"
        # camera_to_base.transform.translation.x = 0.0
        # camera_to_base.transform.translation.y = 0.0
        # camera_to_base.transform.translation.z = 0.0
        # q_camera = euler_to_quaternion(roll=0, pitch=np.pi/2, yaw=0)
        # camera_to_base.transform.rotation.x = float(q_camera[0].item())
        # camera_to_base.transform.rotation.y = float(q_camera[1].item())
        # camera_to_base.transform.rotation.z = float(q_camera[2].item())
        # camera_to_base.transform.rotation.w = float(q_camera[3].item())
        # self.__broadcaster.sendTransform(camera_to_base)

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
        msg.k = [606.0, 0.0, 423.0, 0.0, 605.0, 231.0, 0.0, 0.0, 1.0] # FR-GESTURE camera intrinsics
        self.__info_publisher.publish(msg)

        msg = NavSatFix()
        msg.header.stamp = stamp
        (msg.longitude, msg.latitude) = abs_xy_to_gps(x=x_mm,y=y_mm)
        self.__gps_publisher.publish(msg)

        # msg = Odometry()
        # q = euler_to_quaternion(roll=0,pitch=0,yaw=np.pi/2)
        # msg.header.stamp = stamp
        # msg.pose.pose.orientation.x = q[0].item()
        # msg.pose.pose.orientation.y = q[1].item()
        # msg.pose.pose.orientation.z = q[2].item()
        # msg.pose.pose.orientation.w = q[3].item()
        # self.__odo_publisher.publish(msg)

        msg = Float32()
        msg.data = omega # degrees, magnetic north, clockwise (!)
        self.__heading_publisher.publish(msg)

        self.get_logger().info(f"Publishing {color_path} at x={x_mm}, y={y_mm}, omega={omega}")

        self.__timestep += 1


def main():
    try:
        # test = Localization_Line_Test()
        # print(test.generate_full_trajectory(max_timestep=100))
        test = Classification_Test()
        rclpy.init()
        rclpy.spin(node=Producer(test_scenario=test,fps=FPS))
    except (ExternalShutdownException, KeyboardInterrupt) as e:
        print(e)

if __name__ == '__main__':
    main()
