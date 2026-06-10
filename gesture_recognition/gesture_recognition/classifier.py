#   Foteinos Konstantinos
#   kfoteinos@hua.gr
#   
#   Harokopio University of Athens,
#   Department of Informatics and Telematics
#   HUA Computer Vision Group

import rclpy
import rclpy.duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
import tf2_ros
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Pose, Transform
from message_filters import Subscriber, ApproximateTimeSynchronizer
import cv2
import json
import math
import numpy as np
import torch
import torchvision
import torchvision.transforms as transforms
from ultralytics import YOLO
from torch.nn.functional import softmax
from robal_interfaces.action import NavigateTo, Trigger, ReturnToBaseFetch, HelpRequest, ReturnToBase
import time
import abc
import os

# Configuration-- -------------------------------------------------------------------------------------------------------------------------------------------------------------

class Configuration:
    def __init__(self,
            debugging:bool,
            transforms_available:bool,
            fix_available:bool,
            nav_fix_topic:str,
            depth_topic:str,
            rgb_topic:str,
            camera_info:str,
            output_topic:str,
            target_timeout_seconds:float,
            earth_radius:float,
            pose_estimation_threshold:float,
            device:str,
            classification_threshold:float,
            min_occurences:str,
            no_servers:bool,
            server_timeout:float,
            slop:float,
            max_classification_rate:float,
            depth_max_threshold:float,
            depth_min_threshold:float,
            trigger_stop:str,
            trigger_help_request:str,
            trigger_return_to_base_fetch:str,
            trigger_freeze:str,
            trigger_retreat:str,
            trigger_emergency:str,
            trigger_return_to_base:str,
            trigger_navigation:str,
            min_sec_between_commands:float
            ):
        self.debugging = debugging
        self.transforms_available = transforms_available
        self.fix_available = fix_available
        self.nav_fix_topic = nav_fix_topic
        self.depth_topic = depth_topic
        self.rgb_topic = rgb_topic
        self.camera_info = camera_info
        self.output_topic = output_topic
        self.target_timeout_seconds = target_timeout_seconds
        self.earth_radius = earth_radius
        self.pose_estimation_threshold = pose_estimation_threshold
        self.device = device
        self.classification_threshold = classification_threshold
        self.min_occurences = min_occurences
        self.no_servers = no_servers
        self.server_timeout = server_timeout
        self.slop = slop
        self.max_classification_rate = max_classification_rate
        self.depth_max_threshold = depth_max_threshold
        self.depth_min_threshold = depth_min_threshold
        self.trigger_stop = trigger_stop
        self.trigger_help_request = trigger_help_request
        self.trigger_return_to_base_fetch = trigger_return_to_base_fetch
        self.trigger_freeze = trigger_freeze
        self.trigger_retreat = trigger_retreat
        self.trigger_emergency = trigger_emergency
        self.trigger_return_to_base = trigger_return_to_base
        self.trigger_navigation = trigger_navigation
        self.min_sec_between_commands = min_sec_between_commands

# Transformations -------------------------------------------------------------------------------------------------------------------------------------------------------------

class Transformations:
    def __init__(self, node:Node, config:Configuration):
        '''"node" should implement a .info(str) method'''
        self.__tf_buffer = tf2_ros.Buffer()
        self.__tf_listener = tf2_ros.TransformListener(self.__tf_buffer, node)
        self.__init_latitude = None
        self.__init_longitude = None
        self.__node = node
        self.config = config
    def register_initial_gps(self, position:NavSatFix):
        if self.__init_latitude is None or self.__init_longitude is None:
            self.__init_latitude = position.latitude
            self.__init_longitude = position.longitude
            self.__node.info(f"Initial position in (latitude, longitude) = ({self.__init_latitude}, {self.__init_longitude})")
    # @staticmethod
    def uvd_to_rel_xyz(self, u, v, depth, intrinsics) -> np.ndarray:
        '''
        uvd -> rel_xyz (Backprojects a point to 3D space)
        Parameters:
            u:          x' (horizontal) (in pixels)
            v:          y' (vertical) (in pixels)
            depth:      disantce from the optical center (in mm)
            intrinsics: camera intrinsics (in pixels)
        Returns:
            the relative position in 3D space (in mm) w.r.t. to camera frame
        '''
        p_2D_h = np.asarray([u, v, 1]) # homogeneous coordinates
        p_3D = depth * (np.linalg.inv(intrinsics) @ p_2D_h)
        return p_3D
    # @staticmethod
    def rel_xyz_to_base_xyz(self, xyz:np.ndarray, stamp) -> tuple[float]:
        '''xyz in mm'''
        msg = PointStamped()
        msg.header.frame_id = "camera_depth_frame"
        msg.header.stamp = stamp
        msg.point.x = xyz[0].item() / 1000
        msg.point.y = xyz[1].item() / 1000
        msg.point.z = xyz[2].item() / 1000
        transform = self.__tf_buffer.transform(msg,"base_link",timeout=rclpy.duration.Duration(seconds=self.config.target_timeout_seconds))
        return float(transform.point.x) * 1000,float(transform.point.y) * 1000,float(transform.point.z) * 1000
    # @staticmethod
    def base_xyz_to_abs_xyz(self, xyz:tuple[float], stamp) -> tuple[float]:
        '''xyz in mm'''
        msg = PointStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = stamp
        msg.point.x = xyz[0] / 1000
        msg.point.y = xyz[1] / 1000
        msg.point.z = xyz[2] / 1000
        transform = self.__tf_buffer.transform(msg,"map",timeout=rclpy.duration.Duration(seconds=self.config.target_timeout_seconds)) # map or odom
        return float(transform.point.x) * 1000,float(transform.point.y) * 1000,float(transform.point.z) * 1000
    # @staticmethod
    def abs_xy_to_gps(self, x, y) -> tuple[float]:
        '''
        abs_xy -> GPS
        Parameters:
            x,y:        With origin the initial robot position and "orientation" the same with the "flatten" meridians/parallels (in mm)
        Returns:
            - longitude:  GPS (degrees)
            - latitude:   GPS (degrees)
        '''
        lat = self.__init_latitude + ((y/1000) / self.config.earth_radius) * (180.0 / math.pi)
        lon = self.__init_longitude + ((x/1000) / (self.config.earth_radius * math.cos(math.radians(self.__init_latitude)))) * (180.0 / math.pi)
        return float(lon), float(lat)
    # @staticmethod
    def gps_to_abs_xy(self, lat, lon) -> tuple[float]:
        '''
        GPS -> abs_xy (the inverse of the previous)
        '''
        y = (lat - self.__init_latitude) * (math.pi / 180.0) * self.config.earth_radius # in meters
        x = (lon - self.__init_longitude) * (math.pi / 180.0) * (self.config.earth_radius * math.cos(math.radians(self.__init_latitude))) # in meters
        return float(x * 1000), float(y * 1000) # (in mm)

# Pose estimation --------------------------------------------------------------------------------------------------------------------------------------------------------------

class Pose_Estimator(abc.ABC):
    @abc.abstractmethod
    def detect_keypoints(self, color, depth) -> list[dict]:pass
    @staticmethod
    @abc.abstractmethod
    def aggregate(keypoints):pass
    @staticmethod
    @abc.abstractmethod
    def get_single_person(all_keypoints:list[dict]):pass
    @staticmethod
    def accept(confidence, config:Configuration):
        return confidence >= config.pose_estimation_threshold

class YOLO_Pose_Wrapper(Pose_Estimator):
    def __init__(self, model:str, config:Configuration):
        self.__device = config.device
        self.__pose_estimator = YOLO(model).to(self.__device)
        self.config = config
        self.__names = [
            "Nose",
            "Left Eye",
            "Right Eye",
            "Left Ear",
            "Right Ear",
            "Left Shoulder",
            "Right Shoulder",
            "Left Elbow",
            "Right Elbow",
            "Left Wrist",
            "Right Wrist",
            "Left Hip",
            "Right Hip",
            "Left Knee",
            "Right Knee",
            "Left Ankle",
            "Right Ankle"
        ]
    def detect_keypoints(self, color, depth=None) -> list[dict]:
        '''
        Parameters:
            color:      numpy array with dimensions height x width x 3 (H x W x 3)
            depth:      numpy array with dimensions height x width x 1 (H x W x 1)
        Returns:
            [
                {
                    Keys:       Keypoint name
                    Values:     [u coordinate (pixels), v coordinate (pixels), confidence ([0,1]), depth (units similar to the depth)] (primitives, not np arrays)
                },
                for every detected person
                ...
            ]
            Depth is zero if not available
        '''
        all_result = self.__pose_estimator(color, verbose=False)[0]
        result = all_result.keypoints.data
        if self.config.debugging and len(result)>0:
            all_result.save("vis.png")
        keypoints = []
        for person in range(len(result)):
            keypoints.append(dict({}))
            for i in range(len(self.__names)):
                uvcd:list = result[person][i][[0,1]].cpu().numpy().astype(int).tolist()
                uvcd.append(result[person][i][2].item())
                if depth is not None:
                    try:
                        uvcd.append(depth[uvcd[1],uvcd[0]].item())
                    except IndexError:
                        uvcd.append(0)
                else:
                    uvcd.append(0)
                keypoints[-1][self.__names[i]] = uvcd
        return keypoints
    @staticmethod
    def aggregate(keypoints:dict) -> float:
        '''
        Parameters:
            keypoints:  Keypoints of a single person in the format of detect_keypoints output
            Only the left & right shoulder are considered
        Returns:
            d:      average depth of the keypoints (in the given depth measurement units)
            u:      average u (in pixels)
            v:      average v (in pixels)
            c:      average confidence ([0,1])
        '''
        # d = u = v = c = total = 0
        # for keypoint_name in keypoints.keys():
        #     if keypoints[keypoint_name][3] == 0:
        #         continue
        #     total += 1
        #     u += keypoints[keypoint_name][0]
        #     v += keypoints[keypoint_name][1]
        #     c += keypoints[keypoint_name][2]
        #     d += keypoints[keypoint_name][3]
        # d /= total
        # u /= total
        # v /= total
        # c /= total
        # return d, u, v, c
        ls = keypoints["Left Shoulder"]
        rs = keypoints["Right Shoulder"]
        if ls[3] !=0 and rs[3] !=0:
            u = (ls[0]+rs[0])/2
            v = (ls[1]+rs[1])/2
            c = (ls[2]+rs[2])/2
            d = (ls[3]+rs[3])/2
            return d, u, v, c
        return None
    @staticmethod
    def get_single_person(all_keypoints:list[dict]): # choose the one in shortest distance
        '''Returns None if cannot infer human distance or no humans are present'''
        argmin_u = None
        argmin_v = None
        argmin_c = None
        argmin_idx = None
        min_depth = math.inf
        for idx, single_person_keypoints in enumerate(all_keypoints):
            agg = YOLO_Pose_Wrapper.aggregate(single_person_keypoints)
            if agg is None:
                continue
            depth, u, v, c = agg[0], agg[1], agg[2], agg[3]
            if depth < min_depth:
                argmin_u = u
                argmin_v = v
                argmin_c = c
                min_depth = depth
                argmin_idx = idx
        return argmin_u, argmin_v, argmin_c, min_depth, argmin_idx

# Classification wrappers ------------------------------------------------------------------------------------------------------------------------------------------------------

class Classifier(abc.ABC):
    # @abc.abstractmethod
    # def load_weights(self, path:str):pass
    @abc.abstractmethod
    def classify(self, image):pass # 8-bit RGB image (H x W x 3)
    @staticmethod
    def accept(confidence, config:Configuration):
        return confidence >= config.classification_threshold

class EfficientNetB0_Wrapper(Classifier):
    def __init__(self,  config:Configuration, path:str|None=None):
        self.__device = config.device
        self.__classes = [
            "fetch-a-gas-mask", # G0
            "come-to-me", # G1
            "unfreeze", # G10 (previously named "ok-to-go")
            "move-away-from-here", # G11
            "stop", # G12
            "operation-finished", # G2
            "freeze", # G3
            "emergency-situation", # G4
            "i-need-help", # G5
            "evacuate-the-area", # G6
            "i-lost-connection", # G7
            "fetch-a-shovel", # G8
            "fetch-an-axe" # G9
        ]
        self.__model = torchvision.models.efficientnet_b0(num_classes=len(self.__classes))
        self.__model = self.__model.to(self.__device)
        assert path is not None, "path should be a valid path (str)"
        # if path is not None:
        #     self.load_weights(path)
        self.load_weights(path)
        self.__to_tensor = transforms.ToTensor()
        self.__resize = transforms.Resize((224,224))
    def load_weights(self, path):
        assert os.path.isfile(path), f"{path} is not a valid path"
        self.__model.load_state_dict(torch.load(path,map_location=torch.device(self.__device)))
    def classify(self, image): # array
        self.__model.eval()
        # probabilities = softmax(self.__model(self.__resize((self.__to_tensor(image)/255).unsqueeze(dim=0)).to(self.__device))[0], dim=0).detach().cpu().numpy()
        probabilities = softmax(self.__model(self.__resize((self.__to_tensor(image)).unsqueeze(dim=0)).to(self.__device))[0], dim=0).detach().cpu().numpy()
        argmax = probabilities.argmax()
        pred_class = self.__classes[argmax]
        confidence = probabilities[argmax].item()
        return {
            "class": pred_class,
            "confidence": confidence
        }

class YOLO_Classification_Wrapper(Classifier):
    def __init__(self, path:str, config:Configuration):
        self.__device = config.device
        self.__classes = {
            "0": "fetch-a-gas-mask",
            "1": "come-to-me",
            "2": "operation-finished",
            "3": "freeze",
            "4": "emergency-situation",
            "5": "i-need-help",
            "6": "evacuate-the-area",
            "7": "i-lost-connection",
            "8": "fetch-a-shovel",
            "9": "fetch-an-axe",
            "10": "unfreeze",
            "11": "move-away-from-here",
            "12": "stop"
        }
        assert os.path.isfile(path), f"{path} is not a valid path"
        self.__model = YOLO(path).to(self.__device)
    # def load_weights(self, path):
    #     raise NotImplementedError()
    def classify(self, image): # array or path
        result = self.__model(image, verbose=False)[0]
        confidence = result.probs.top1conf.item()
        pred_class = self.__classes[result.names[result.probs.top1]]
        return {
            "class": pred_class,
            "confidence": confidence
        }

# Filter ----------------------------------------------------------------------------------------------------------------------------------------------------------------------

class Command_Filter:
    def __init__(self, node:Node, config:Configuration):
        '''"node" should implement a .info(str) method'''
        self.restart()
        self.__node = node
        self.__min_occurs = config.min_occurences
        self.config = config
    def register_command(self, gesture_command:str, confidence:float):
        if self.__current is None or self.__current != gesture_command:
            self.__current = gesture_command
            self.__counter = 1
        # current command = given
        else:
            self.__counter += 1
    def restart(self):
        self.__current = None
        self.__counter = 0
        self.__last_time_triggered = -math.inf
        self.__not_triggered_due_to_time = False # the action was not accepted, due to the "inter-command" time interval constraint
    def accept(self):
        if self.__counter < self.__min_occurs:
            if self.config.debugging: self.__node.info(f"{self.__counter} < {self.__min_occurs} for {self.__current}")
            return False
        # it occured many times succesively
        elif self.__counter == int(self.__min_occurs):
            if self.config.debugging: self.__node.info(f"{self.__counter} = {self.__min_occurs} for {self.__current}")
            now = time.time()
            if now - self.__last_time_triggered >= self.config.min_sec_between_commands:
                self.__last_time_triggered = now
                return True
            else:
                if self.config.debugging: self.__node.info(f"Action rejected, as the last one was accepted before {now - self.__last_time_triggered} < {self.config.min_sec_between_commands} (sec)")
                self.__not_triggered_due_to_time = True
                return False
        # the action has already been called
        else:
            if self.config.debugging: self.__node.info(f"{self.__counter} > {self.__min_occurs} for {self.__current}")
            if self.__not_triggered_due_to_time:
                self.__not_triggered_due_to_time = False
                self.__last_time_triggered = time.time()
                if self.config.debugging: self.__node.info("Action accepted (was not accepted previously due to constraint on time between triggers)")
                return True
            else:
                return False

# Action caller ---------------------------------------------------------------------------------------------------------------------------------------------------------------

class Action_Caller:
    def __init__(self, node:Node, config:Configuration):
        '''"node" should impement .info(str), .error(str)'''
        self.__stop = ActionClient(node, Trigger, config.trigger_stop)
        self.__help = ActionClient(node, HelpRequest, config.trigger_help_request)
        self.__fetch = ActionClient(node, ReturnToBaseFetch, config.trigger_return_to_base_fetch)
        self.__freeze = ActionClient(node, Trigger, config.trigger_freeze)
        self.__retreat = ActionClient(node, Trigger, config.trigger_retreat)
        self.__emergency = ActionClient(node, Trigger, config.trigger_emergency)
        self.__return_bos = ActionClient(node, ReturnToBase, config.trigger_return_to_base)
        self.__navigation = ActionClient(node, NavigateTo, config.trigger_navigation)
        self.__node = node
        self.config = config
    def __call_server(self, server, msg):
        if not self.config.no_servers:
            if not server.wait_for_server(timeout_sec=self.config.server_timeout):
                self.__node.error(f"SERVER UNAVAILABLE (timeout = {self.config.server_timeout:0.2f})")
            else:
                server.send_goal_async(msg)
                self.__node.info("\033[1;102mMESSAGE WAS SENT\033[0;0m")
    def trigger_action(self, gesture_command:str, **args): # args in mm        
        # https://asantamarianavarro.gitlab.io/code/projects/triffid/aurops/sections/triffid/ugv_planning.html#gesture-commander
        if gesture_command == "come-to-me":
            msg = NavigateTo.Goal()
            msg.goal_pose = Pose() # map frame
            msg.goal_pose.position.x = float(args["x"]) / 1000 # convert to meters
            msg.goal_pose.position.y = float(args["y"]) / 1000
            msg.goal_pose.position.z = float(args["z"]) / 1000
            msg.goal_pose.orientation.x = float(args["q0"])
            msg.goal_pose.orientation.y = float(args["q1"])
            msg.goal_pose.orientation.z = float(args["q2"])
            msg.goal_pose.orientation.w = float(args["q3"])
            msg.timeout = -1.0
            self.__call_server(self.__navigation, msg)
        
        elif gesture_command == "unfreeze": # Previously named "ok-to-go", unfreeze
            msg = Trigger.Goal()
            msg.activate = False
            self.__call_server(self.__freeze, msg)
        
        elif gesture_command == "move-away-from-here":
            msg = Trigger.Goal()
            msg.activate = True
            msg.timeout = -1.0
            self.__call_server(self.__retreat, msg)
        
        elif gesture_command == "operation-finished":
            msg = ReturnToBase.Goal()
            msg.activate = True
            msg.timeout = -1.0
            self.__call_server(self.__return_bos, msg)
        
        elif gesture_command == "freeze":
            msg = Trigger.Goal()
            msg.activate = True
            self.__call_server(self.__freeze, msg)
        
        elif gesture_command == "stop":
            msg = Trigger.Goal()
            msg.activate = True
            self.__call_server(self.__stop, msg)

        elif gesture_command == "emergency-situation":
            msg = Trigger.Goal()
            msg.activate = True
            self.__call_server(self.__emergency, msg)

        elif gesture_command == "i-need-help":
            msg = HelpRequest.Goal()
            msg.target_transform = Transform()
            msg.target_transform.translation.x = float(args["x"]) / 1000
            msg.target_transform.translation.y = float(args["y"]) / 1000
            msg.target_transform.translation.z = float(args["z"]) / 1000
            msg.target_transform.rotation.x = float(args["q0"])
            msg.target_transform.rotation.y = float(args["q1"])
            msg.target_transform.rotation.z = float(args["q2"])
            msg.target_transform.rotation.w = float(args["q3"])
            msg.help_type = "aids"
            msg.timeout = -1.0
            self.__call_server(self.__help, msg)
        
        elif gesture_command == "evacuate-the-area": # TODO: map this command to an action
            msg = ReturnToBase.Goal()
            msg.activate = True
            msg.timeout = -1.0
            self.__call_server(self.__return_bos, msg)
        
        elif gesture_command == "i-lost-connection":
            msg = HelpRequest.Goal()
            msg.target_transform = Transform()
            msg.target_transform.translation.x = float(args["x"]) / 1000
            msg.target_transform.translation.y = float(args["y"]) / 1000
            msg.target_transform.translation.z = float(args["z"]) / 1000
            msg.target_transform.rotation.x = float(args["q0"])
            msg.target_transform.rotation.y = float(args["q1"])
            msg.target_transform.rotation.z = float(args["q2"])
            msg.target_transform.rotation.w = float(args["q3"])
            msg.help_type = "technical"
            msg.timeout = -1.0
            self.__call_server(self.__help, msg)
        
        elif gesture_command == "fetch-a-gas-mask":
            msg = ReturnToBaseFetch.Goal()
            msg.activate = True
            msg.object = "gas_mask"
            msg.timeout = -1.0
            self.__call_server(self.__fetch, msg)
        
        elif gesture_command == "fetch-a-shovel":
            msg = ReturnToBaseFetch.Goal()
            msg.activate = True
            msg.object = "shovel"
            msg.timeout = -1.0
            self.__call_server(self.__fetch, msg)
        
        elif gesture_command == "fetch-an-axe":
            msg = ReturnToBaseFetch.Goal()
            msg.activate = True
            msg.object = "axe"
            msg.timeout = -1.0
            self.__call_server(self.__fetch, msg)

        else:
            self.__node.error(f"Unknown command: {gesture_command}")

# Perception ------------------------------------------------------------------------------------------------------------------------------------------------------------------

class Perceptron(abc.ABC):
    @abc.abstractmethod
    def get_arrays(self, color_image, depth_image):pass

class DEMO_Perceptron(Perceptron):
    def get_arrays(self, color_image, depth_image):
        color_array = np.asarray(color_image.data, dtype=np.uint8).reshape((color_image.height, color_image.width, 3)) # H x W x 3
        depth_array = cv2.resize(np.asarray(np.frombuffer(depth_image.data,dtype=np.uint16), dtype=np.float32).reshape((depth_image.height, depth_image.width)),\
                                 dsize=(color_image.width, color_image.height)).reshape((color_image.height, color_image.width, 1)) # H x W x 1
        return color_array, depth_array
    
class RealSense_Perceptron(Perceptron):
    def get_arrays(self, color_image, depth_image):
        yuyv = np.frombuffer(color_image.data, dtype=np.uint8)
        yuyv = yuyv.reshape((color_image.height, color_image.width, 2))
        bgr = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
        color_array = bgr.reshape((color_image.height, color_image.width, 3))
        depth_array = np.frombuffer(depth_image.data, dtype=np.uint16).reshape((depth_image.height, depth_image.width))
        return color_array, depth_array

# Cancelled
class Camera360_Perceptron(Perceptron):
    def get_arrays(self, color_image, depth_image):
        return super().get_arrays(color_image, depth_image)

# Gesture commander -----------------------------------------------------------------------------------------------------------------------------------------------------------

class Gesture_Commander_Coordinator(Node):
    def __init__(self, 
            classifier:Classifier,
            pose_estimator:Pose_Estimator,
            perceptron:Perceptron,
            config:Configuration
        ):
        super().__init__("gesture_commander")
        self.__time_synchronizer = ApproximateTimeSynchronizer(
            fs=[
                Subscriber(node=self, msg_type=Image, topic=config.rgb_topic), 
                Subscriber(node=self, msg_type=Image, topic=config.depth_topic), 
                Subscriber(node=self, msg_type=CameraInfo, topic=config.camera_info),
            ] + ([Subscriber(node=self, msg_type=NavSatFix, topic=config.nav_fix_topic)] if config.fix_available else []),
            queue_size=10,
            slop=config.slop
        )
        self.__time_synchronizer.registerCallback(self.__main_callback)
        self.__publisher=self.create_publisher(
            msg_type = String,
            topic = config.output_topic,
            qos_profile = 10
        )
        self.__classifier = classifier
        self.__pose_estimator = pose_estimator
        self.__command_filter = Command_Filter(node=self, config=config)
        self.__command_filter.restart()
        self.__perceptron = perceptron
        self.__action_caller = Action_Caller(node=self, config=config)
        if config.transforms_available:
            self.__transformations = Transformations(node=self, config=config)
        self.__log_counter = 0
        self.__last = None
        self.__detection_id = 0
        self.config = config
        self.info(f"Running on {self.config.device}")

    def info(self, text:str) -> None:
        self.get_logger().info(f"[{self.__log_counter}] {text}")

    def error(self, text:str) -> None:
        self.get_logger().error(f"[{self.__log_counter}] {text}")

    def warn(self, text:str) -> None:
        self.get_logger().warning(f"[{self.__log_counter}] {text}")

    def __main_callback(self, color_image:Image, depth_image:Image, intrinsics:CameraInfo, global_position:NavSatFix=None):
        '''
        Parameters:
            color_image:    8-bit RGB image (H x W x 3)
            depth_image:    16UC1 in mm depth map (H x W x 2) of the same dimensions and aligned to the color_image
            intrinsics:     Camera intrinsics (the code uses only K matrix)
            global_position:Longitude/latitude (degrees)
            odomometry:     The code needs the "absolute" orientation (w.r.t. to the "standard" xy plane), as described in (*)
        Publishes:
            See README
        '''

        try:
        
            if self.config.transforms_available:
                self.__transformations.register_initial_gps(global_position)

            if self.__last is None: # first frame
                self.__last = time.time()
            else:
                current = time.time()
                approx_rate = 1 / max(current - self.__last, 1e-6) # 1/sec
                if approx_rate > self.config.max_classification_rate:
                    self.info(f"Omit: {approx_rate:.2f} > {self.config.max_classification_rate:.2f} FPS")
                    return
                self.__last = current # do not place it before the if statement, as it needs to estimate the classification rate (not the camera FPS)
                self.info(f"Current approximated classification rate: {approx_rate:.2f} FPS")
            self.__log_counter += 1

            color_array, depth_array = self.__perceptron.get_arrays(color_image=color_image, depth_image=depth_image)

            # Human(s) pose estimation
            all_keypoints = self.__pose_estimator.detect_keypoints(color=color_array, depth=depth_array)
            self.info(f"{len(all_keypoints)} detected humans")
            # Filter: Are there any humans?
            if len(all_keypoints) == 0:
                self.__command_filter.restart()
                return
            argmin_u, argmin_v, argmin_c, min_depth, argmin_idx = self.__pose_estimator.get_single_person(all_keypoints)
            # # Filter: Is pose estimation confident enough?
            # if not self.__pose_estimator.accept(argmin_c, config=self.config):
            #     self.__command_filter.restart()
            #     self.warn(f"Low confidence in pose estimation")
            #     return
            # Filter: Is human depth available?
            if argmin_u is None:
                self.__command_filter.restart()
                self.warn(f"Cannot infer human distance")
                return
            # # Filter: Is human sufficiently near?
            # if min_depth > self.config.depth_max_threshold or min_depth < self.config.depth_min_threshold:
            #     self.__command_filter.restart()
            #     self.warn(f"Distance from camera ({min_depth}) exceeds threshold (> {self.config.depth_max_threshold}) or < {self.config.depth_min_threshold} (mm)")
            #     return

            prediction = self.__classifier.classify(color_array)
            # Filter: Is the classification confident enough?
            if not self.__classifier.accept(confidence=prediction['confidence'], config=self.config):
                self.__command_filter.restart()
                self.warn(f"Low confidence.")
                return

            self.__command_filter.register_command(gesture_command=prediction['class'], confidence=prediction['confidence'])
            # Filter: Successive occurences (given filter 4, of high confidence!)
            if not self.__command_filter.accept():
                # Do not restart!
                self.warn(f"Ignoring {prediction['class']}")
                return

            self.info(f"\033[1;102mACTION ACCEPTED FOR {prediction['class']}\033[0;0m")

            if self.config.transforms_available:
                rel_xyz = self.__transformations.uvd_to_rel_xyz(u=argmin_u,v=argmin_v,depth=min_depth,intrinsics=np.asarray(intrinsics.k).reshape((3,3)))
                base_xyz = self.__transformations.rel_xyz_to_base_xyz(xyz=rel_xyz,stamp=color_image.header.stamp)
                abs_xyz = self.__transformations.base_xyz_to_abs_xyz(xyz=base_xyz,stamp=color_image.header.stamp)
                gps = self.__transformations.abs_xy_to_gps(x=abs_xyz[0],y=abs_xyz[1]) # lon, lat
                self.info(f"Detection position: {gps} (GPS) [or ({self.__transformations.gps_to_abs_xy(lat=gps[1],lon=gps[0])}) (xy in mm)]")
            else:
                abs_xyz = (0, 0, 0)
                self.warn("No available frame transformations")

            self.__action_caller.trigger_action(gesture_command=prediction['class'], x=abs_xyz[0], y=abs_xyz[1], z=abs_xyz[2], q0=0, q1=0, q2=0, q3=1)
            self.__publisher.publish(String(data=json.dumps({
                "type": "FeatureCollection",
                "features":[
                    {
                        "type": "Feature",
                        "geometry": {
                            "type": "Point",
                            "coordinates": list(gps) if self.config.transforms_available and self.config.fix_available else None
                        },
                        "properties": {
                            "class":prediction["class"],
                            "confidence":prediction["confidence"],
                            "depth":min_depth,
                            "id":self.__detection_id,
                            "timestamp":self.get_clock().now().nanoseconds,
                            "keypoints_and_depths": all_keypoints[argmin_idx],
                            "camera_frame_position": {
                                "rel_x":rel_xyz[0],
                                "rel_y":rel_xyz[1],
                                "rel_z":rel_xyz[2]
                            } if self.config.transforms_available and self.config.fix_available else None
                        }
                    }
                ]
            })))
            self.__detection_id += 1

        except BaseException as e:
            self.error(f"FATAL ERROR : {e}")

# -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

def main():
    try:
        config = Configuration(
            debugging = True,
            transforms_available = False,
            fix_available = False,
            nav_fix_topic = "/fix",
            depth_topic = "/b2/camera_front_435i/realsense_front_435i/depth/image_rect_raw",
            rgb_topic = "/b2/camera_front_435i/realsense_front_435i/color/image_raw",
            camera_info = "/b2/camera_front_435i/realsense_front_435i/color/camera_info",
            output_topic = "/gesture_command",
            target_timeout_seconds = 1e-1,
            earth_radius = 6_378_137.0, # in meters
            pose_estimation_threshold = 0.80,
            device = "cuda" if torch.cuda.is_available() else "cpu",
            classification_threshold = 0.80,
            min_occurences = 4,
            no_servers = False, # previous NO_UNDERLYING_IMPL, change this to False during integration with UPC
            server_timeout = 1.0,
            slop = 1e-1,
            max_classification_rate = 2,
            depth_max_threshold = 100_000, # mm
            depth_min_threshold = 1_000, # mm
            trigger_stop = "/b2/local/trigger_stop",
            trigger_help_request = "/b2/local/trigger_help_request",
            trigger_return_to_base_fetch = "/b2/local/trigger_return_to_base_fetch",
            trigger_freeze = "/b2/local/trigger_freeze",
            trigger_retreat = "/b2/local/trigger_retreat",
            trigger_emergency = "/b2/global/trigger_emergency",
            trigger_return_to_base = "/b2/local/trigger_return_to_base",
            trigger_navigation = "/b2/local/trigger_navigation",
            min_sec_between_commands = 10 # seconds
        )
        rclpy.init()
        rclpy.spin(node=Gesture_Commander_Coordinator(
            classifier = EfficientNetB0_Wrapper(config=config,path="/home/triffid/hua_ws/gesture_module_v2/gesture_recognition/gesture_recognition/efficientnetb0_color_pretrained_ext.pt"),
            # classifier = YOLO_Classification_Wrapper(config=config,path="/home/triffid/hua_ws/gesture_module_v2/gesture_recognition/gesture_recognition/yolo26m-cls-FR-GESTURE.pt"),
            pose_estimator = YOLO_Pose_Wrapper(model="yolo26n-pose.pt", config=config),
            # perceptron = DEMO_Perceptron(),
            perceptron = RealSense_Perceptron(),
            config = config
        ))
    except (ExternalShutdownException, KeyboardInterrupt) as e:
        print(e)


if __name__ == '__main__':
    main()