# Python 3
# Interface for a robot arm

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

class SceneObject:
    def get_color(self) -> Color:
        """Gets the color of the scene object"""
    
    def get_object_type(self) -> SceneObjectType:
        """Gets the object type of the scene object"""
    
    def get_pose(self) -> Pose:
        """Gets the pose of the scene object"""
        
class RobotController:
    def __init__(self):
        """RobotController initializer"""
        
    def rotate_left(self, radians):
        """Rotate the arm left by the amount of radians specified"""
        
    def rotate_right(self, radians):
        """Rotate the arm right by the amount of radians specified"""
        
    def tilt_up(self, radians):
        """Tilt the arm up by the amount of radians specified"""
        
    def tilt_down(self, radians):
        """Tilt the arm down by the amount of radians specified"""
        
    def move_forward(self, radians):
        """Extend the arm forward, away from center, by the distance specified in radians"""
        
    def move_backward(self, radians):
        """Retract the arm backward, away from center, by the distance specified in radians"""
        
    def grasp(self):
        """Close the gripper to grasp an object"""
        
    def release(self):
        """Open the gripper to release an object"""
        
    def go_to_pose(self, pose):
        """Move the arm to the specified Pose"""
        
    def get_scene_objects(self) -> List<SceneObject>:
        """Gets all objects in the visual scene."""

###