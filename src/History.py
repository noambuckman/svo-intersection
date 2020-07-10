### TODO:  nbuckman Is this actually being used?

from sys import getsizeof, stderr
from itertools import chain
from collections import deque
try:
    from reprlib import repr
except ImportError:
    pass
    
    
class PoseHistory:
    # This class is used to store the history of poses and times for each agent
    def __init__(self):
        self.poses = []
        self.speeds = []
        self.times = []
    
    def add_to_history(self,current_time,current_pose,current_speed):
        self.times.append(current_time)
        self.poses.append(current_pose)
        self.speeds.append(current_speed)

    def get_history_lists(self):
        return self.times, self.poses, self.speeds
    

