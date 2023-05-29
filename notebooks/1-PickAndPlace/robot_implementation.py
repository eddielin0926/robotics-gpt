# Python 3
# Interface for a robot arm

import enum
import roslibpy
import time
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
        self.talker = roslibpy.Topic(self.ros_client, '/unity/pick_and_place', 'niryo_moveit/PickAndPlace')
        self.service = roslibpy.Service(self.ros_client, '/unity/object_pose_svc', 'niryo_moveit/ObjectPoseService')
        self.get_scene_objects_svc = roslibpy.Service(self.ros_client,
                                                      '/unity/get_scene_objects_svc',
                                                      'niryo_moveit/GetSceneObjectsService')
    
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
    
    def pick_and_place(self, pick_pose:Pose, place_pose:Pose):
        """The robot will pick up the object at target_pose and move it to the placement_pose"""
        self.talker.publish(roslibpy.Message({'pick_pose': pick_pose.as_dict(), 'place_pose': place_pose.as_dict()}))
        print('Sending message...')
        time.sleep(12)
        
    def __del__(self):
        print('Destructor called. Shutting down ROS connection.')
        #self.talker.unadvertise()
        self.ros_client.terminate()
    
###