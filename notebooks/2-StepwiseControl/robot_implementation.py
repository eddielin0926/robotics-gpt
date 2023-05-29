# Python 3
# Interface for a robot arm

import enum
import roslibpy
import time
import math
import os
from typing import List

class Color(enum.Enum):
    Blue =   "blue"   #blue
    Green =  "green"  #green
    Yellow = "yellow" #yellow
    Red =    "red"    #red
    Orange = "orange" #orange
    Black =  "black"  #black
    
class SceneObjectType(enum.Enum):
    Block =  "block"  #block
    Bin =    "bin"    #bin
    Ball =   "ball"   #ball
    
class Pose:
    def __init__(self, serialized):
        self.position = serialized["position"]
        self.orientation = serialized["orientation"]
        self.serialized = serialized
    
    def as_dict(self):
        return self.serialized;

class SceneObject:
    def __init__(self, serialized):
        self.color = Color(serialized["color"])
        self.object_type = SceneObjectType(serialized["object_type"])
        self.pose = Pose(serialized["pose"])
        
    def get_color(self) -> Color:
        return self.color
    
    def get_object_type(self) -> SceneObjectType:
        return self.object_type
    
    def get_pose(self) -> Pose:
        return self.pose

class RobotController:
    def __init__(self):
        self.ros_client = roslibpy.Ros(host='localhost', port=8080)
        self.ros_client.run()
        print('Is ROS connected?', self.ros_client.is_connected)
        self.service = roslibpy.Service(self.ros_client, '/unity/object_pose_svc', 'niryo_moveit/ObjectPoseService')
        self.get_scene_objects_svc = roslibpy.Service(self.ros_client,
                                                      '/unity/get_scene_objects_svc',
                                                      'niryo_moveit/GetSceneObjectsService')
        
        # Create a publisher on the 'move_commands' topic
        self.move_publisher = roslibpy.Topic(self.ros_client, '/unity/move_commands', 'niryo_moveit/MoveCommand')
        
        # Create a publisher on the 'move_to_pose' topic
        self.move_to_pose_publisher = roslibpy.Topic(self.ros_client, '/unity/move_to_pose', 'geometry_msgs/Pose')
    
    def get_scene_objects(self) -> List[SceneObject]:
        """Gets all objects in the visual scene."""
        request = roslibpy.ServiceRequest()
        
        print('Calling service...')
        result = self.get_scene_objects_svc.call(request)
        found_objects = []
        for scene_obj_json in result["scene_objects"]:
            found_objects.append(SceneObject(scene_obj_json))
        
        return found_objects
    
        
    def get_object_pose(self, name):
        """
        Finds the pose of an object matching the specified name.
        """
        request = roslibpy.ServiceRequest()
        request["object_name"] = name

        print('Calling service...')
        result = self.service.call(request)
        
        return result["object_pose"]
        
    def rotate_left(self, radians):
        """Rotate the arm left by the amount of radians specified"""
        self.move_publisher.publish(roslibpy.Message({'move_command': "rotate_left", 'radians': radians}))
        print('Sending rotate_left message...')
        time.sleep(3)
        
    def rotate_right(self, radians):
        """Rotate the arm right by the amount of radians specified"""
        self.move_publisher.publish(roslibpy.Message({'move_command': "rotate_right", 'radians': radians}))
        print('Sending rotate_right message...')
        time.sleep(3)
        
    def tilt_up(self, radians):
        """Tilt the arm up by the amount of radians specified"""
        self.move_publisher.publish(roslibpy.Message({'move_command': "tilt_up", 'radians': radians}))
        print('Sending tilt_up message...')
        time.sleep(3)
        
    def tilt_down(self, radians):
        """Tilt the arm down by the amount of radians specified"""
        self.move_publisher.publish(roslibpy.Message({'move_command': "tilt_down", 'radians': radians}))
        print('Sending tilt_down message...')
        time.sleep(3)
        
    def move_forward(self, radians):
        """Extend the arm forward, away from center, by the distance specified in radians"""
        self.move_publisher.publish(roslibpy.Message({'move_command': "move_forward", 'radians': radians}))
        print('Sending move_forward message...')
        time.sleep(5)
        
    def move_backward(self, radians):
        """Retract the arm backward, away from center, by the distance specified in radians"""
        self.move_publisher.publish(roslibpy.Message({'move_command': "move_backward", 'radians': radians}))
        print('Sending move_backward message...')
        time.sleep(5)
        
    def grasp(self):
        """Close the gripper to grasp an object"""
        self.move_publisher.publish(roslibpy.Message({'move_command': "grasp"}))
        print('Sending grasp message...')
        time.sleep(1)
        
    def release(self):
        """Open the gripper to release an object"""
        self.move_publisher.publish(roslibpy.Message({'move_command': "release"}))
        print('Sending release message...')
        time.sleep(2)
        
    def go_to_pose(self, target_pose:Pose):
        """Move the arm to the specified Pose"""
        self.move_to_pose_publisher.publish(roslibpy.Message(target_pose.as_dict()))
        print('Sending move_to_pose message...')
        time.sleep(10)
        
    def get_scene_objects(self) -> List[SceneObject]:
        """Gets all objects in the visual scene."""
        request = roslibpy.ServiceRequest()
        
        print('Calling service...')
        result = self.get_scene_objects_svc.call(request)
        found_objects = []
        for scene_obj_json in result["scene_objects"]:
            found_objects.append(SceneObject(scene_obj_json))
        
        return found_objects

    def __del__(self):
        print('Destructor called. Shutting down ROS connection.')
        #self.talker.unadvertise()
        self.ros_client.terminate()
    
###