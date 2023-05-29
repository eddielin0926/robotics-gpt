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
        
    def get_scene_objects(self) -> List<SceneObject>:
        """Gets all objects in the visual scene."""

    def pick_and_place(self, pick_pose:Pose, place_pose:Pose):
        """The robot will pick up the object at target_pose and move it to the placement_pose"""
    
###