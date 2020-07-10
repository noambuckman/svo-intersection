import numpy as np
import copy as copy
import time #These are all due to pi implementation
import kinematics as kn
import time
import enum as enum

import History as history
import shapely as sh

class TurnDirection(enum.Enum):
    STRAIGHT = 0
    RIGHT = 1
    LEFT = 2
    
class AutonomousVehicle:
    def __init__(self, agent_id):
        self.agent_id = agent_id

        self.pose = kn.Pose(None,None)

        self.time = 0
        #Info Passed from Sim
        self.notcommunicating = False
        self.notcollaborating = False

        ## Added Noam
        self._intended_lane = 7
        self.current_lane = "1"
        self._maxSpeed = 1.0
        self._minSpeed = 0.001
        self.speed = 1.0
        self.active = True

        # Car Outline

        # self.car_outline = ((self.pose.))


        ## Map Information
        self.map_width = 10
        self.map_height = 10
        self.lane_width = 1.0
        self.entrance_regions = {}
        self.exit_regions = {}
        self.control_regions = {}        

        self.exit_lane = "1"
        self.turn_direction = TurnDirection.STRAIGHT  #Possible values:  "S", "L", "R"
        self.current_lane = None
        self.initial_lane = None
        self.lane_to_pose = {1: np.pi/2.0, 2: np.pi, 3:3*np.pi/2, 4: 2*np.pi}
        self.in_intersection = False

        self.intersection_window = (-1,-1) #start_window, end_window

        self.traffic_coordinator = None
        self.time_increment = 0.01
        self.previous_state = "Straight"

        # self.car_width = 0.5*self.lane_width
        # self.car_length = 1*self.lane_width
        self.car_width = 0.4*self.lane_width
        self.car_length = .6*self.lane_width        

        self.pose_history = history.PoseHistory()

        self.time_entered_control = -1
        self.time_entered_intersection = -1
        self.time_exited_intersection = -1
        self.time_entered_intersection_entrance = -1

        self.batch = False
        self.svo_theta = np.pi/2.0
        self.location_zone = "Location Unassigned"

        self.prob_swap_back_comm = 1.001
        self.prob_swap_back_notcomm = 1.001  #        

        #TODO:  Don't specify shape twice, only once in AV
        
    def get_location_zone(self, pose):
        location = "Location Unassigned"
        if self.check_in_intersection_entrance(pose):
            location = "Intersection Entrance"
        elif self.check_in_intersection_exit(pose):
            location =  "Intersection Exit"
        elif self.check_in_control_region(pose):
            location = "Control Region"            
        elif self.check_intersection(pose): #TODO: Check Notation
            location = "Intersection"
        else:
            location = "Map Entrance/Exit"
        return location



    def loop(self,time_increment,traffic_light=-1,occupancy_map=None):
        self.speed = self._maxSpeed #The order of this section isn't great, split up would be better
        # self.in_intersection = self.check_intersection_entrance()
        self.time_increment = time_increment
        self.location_zone = self.get_location_zone(self.pose)
        # print(self.agent_id, self.location_zone, self.current_lane)
        # State Transition:  Intersection Entrance -> Driving in Intersection
        if self.location_zone == "Intersection" and self.previous_state == "Straight": 
            self.previous_state = "Intersection"
            if self.time_entered_intersection == -1:  self.time_entered_intersection = self.time
            
        if self.location_zone == "Intersection Entrance": # Intersection entrance:  Agent moves to the intersection lane
            reservation_start_time = self.intersection_window[0]
            if self.time_entered_intersection_entrance == -1:
                self.current_lane = self.turn_direction_to_intersection_lane(self.current_lane,self.turn_direction)
                self.time_entered_intersection_entrance = self.time
            # print("Agent %d entered intersection entrance. Reservation: %.02f-> %.02f"%(self.agent_id,self.intersection_window[0],self.intersection_window[1]))
            # print("Current Lane:",self.current_lane)

            if reservation_start_time == -1: 
                print("How did agent",self.agent_id,"Get into the intersection entrance without a valid reservation?")
                pass
            # elif (reservation_start_time + 0.05) < self.time:
            #     print("WARNING:  Agent %d missed start by 0.05 second...resetting reservation"%self.agent_id) 
            #     self.intersection_window = (-1,-1) #It needs a new reservation since its opening has already passed
            else:
                self.current_lane = self.turn_direction_to_intersection_lane(self.current_lane,self.turn_direction)
                self.in_intersection = True   
        elif self.location_zone == "Intersection Exit":
            # State Transition:  Intersection Exit -> Exit Lane
            if self.time_exited_intersection == -1: 
                self.time_exited_intersection = self.time
                self.current_lane = self.intersection_lane_to_exit_lane(self.current_lane)            
            self.in_intersection = False
            self.previous_state = "Straight"

                

        ###### Request reservation ######
        if self.intersection_window[0] == -1 and (self.location_zone == "Control Region" or self.location_zone == "Intersection Entrance"):
            if self.time_entered_control == -1: self.time_entered_control = self.time        
            self.request_reservation_window()                   

            # self.intersection_window = return_reservation

        ##### Move the vehicle #####

        new_pose = self.get_next_step_pose_lane(self.current_lane,time_increment*self._maxSpeed)

        if self.get_location_zone(new_pose) == "Intersection Entrance" and (self.intersection_window[0] == -1 or self.time < self.intersection_window[0] - 0.25/self._maxSpeed):
            self.speed = 0
        elif self.get_location_zone(new_pose) == "Intersection" and (self.intersection_window[0] == -1 or self.time < self.intersection_window[0]):
            self.speed = 0
        elif not self.collision_free(new_pose,occupancy_map,self.pose):
            self.speed = 0
        else:
            self.speed = self._maxSpeed
            self.pose = new_pose

        self.pose_history.add_to_history(self.time,self.pose,self.speed)
        self.active = self.check_in_map(self.pose)

    def request_reservation_window(self):
        future_lane = self.turn_direction_to_intersection_lane(self.current_lane,self.turn_direction,self.notcommunicating)
        time_in_intersection = self.time_required_intersection(self.turn_direction,self._maxSpeed)
        time_to_intersection = self.get_time_to_intersection(self.pose,self.current_lane,self._maxSpeed)
        # return_reservation = self.request_reservation_time_window(self.time,self.time+time_required)
        # print("t:",'%.02f'%self.time,"AG",self.agent_id,"t_to_intersection",'%.02f'%time_to_intersection,"t_in_int",'%.02f'%time_in_intersection)
        start_time = self.time + time_to_intersection
        end_time = start_time + time_in_intersection
        # print("Request:","S->E",'%.02f'%start_time,"->",'%.02f'%end_time)
        # return_reservation = self.traffic_coordinator.request_intersection_lane(start_time,end_time,self.current_lane,self.agent_id)

        bid = self.time - self.time_entered_control
        return_reservation = self.traffic_coordinator.request_intersection_lane(start_time,end_time,future_lane,self.agent_id,self,bid)                   



    def receive_reservation(self,reservation):
        self.intersection_window = reservation


    def collision_free(self,desired_pose,occupancy_map, current_pose):
        # Take a global occupancy_map and check that no other vehicles are within a minimum distance.  Otherwise, return zero velocity
        # If there's a vehicle within a radius of 2*V*deltaT:  Do not move HARD STOP
        # If your future location is within the 2*radius of another vehicle (and they have right of way): DO NOT MOVE
        # TODO:  This needs to be cleaned up ALOT
        if occupancy_map == None:
            return True
        for vehicle in occupancy_map:
            if vehicle == self:
                pass
            else:
                other_pose = occupancy_map[vehicle]
                radius = np.sqrt(self.car_width**2+self.car_length**2)
                max_dimension = np.max([self.car_width,self.car_length])
                pose_to_pose_distance = np.sqrt((other_pose.x-desired_pose.x)**2 + (other_pose.y-desired_pose.y)**2)
                other_pose_to_current_pose_distance = np.sqrt((other_pose.x-desired_pose.x)**2 + (other_pose.y-desired_pose.y)**2)

                # minimum_distance = self._maxSpeed*self.time_increment*(2*radius)

                angle_btwn_other_me = np.arctan2(other_pose.y-current_pose.y,other_pose.x-current_pose.x)
                
                angle_btwn_heading_other = np.pi - (abs(abs(current_pose.phi - angle_btwn_other_me)-np.pi))
                other_has_right_of_way = -np.pi/2 <  np.arctan2(other_pose.y-current_pose.y,other_pose.x-current_pose.x) < np.pi/2
                
                other_car_behind = abs(angle_btwn_heading_other) > np.pi/2
        

                angle_phi_to_phi = np.pi - (abs(abs(other_pose.phi - current_pose.phi)-np.pi))
                simple_prediction = True
                if simple_prediction:
                    if other_car_behind:
                        pass 
                    elif pose_to_pose_distance < (1*self._maxSpeed*self.time_increment):
                        return False
                    elif abs(angle_phi_to_phi)<np.pi/4 and pose_to_pose_distance < (2*self._maxSpeed*self.time_increment+radius):
                        return False                 
                else:
                    if other_car_behind:
                        pass
                    elif abs(angle_phi_to_phi)>=np.pi/2:
                        # HEAD-ON TRAFFIC
                        if (pose_to_pose_distance <= 1*self._maxSpeed*self.time_increment):
                            raise Exception("Collision A: %d %.02f %.02f Other: %d %.02f %.02f"%(self.agent_id,self.pose.x,self.pose.y,vehicle.agent_id,vehicle.x,vehicle.y))
                        elif other_has_right_of_way and other_pose_to_current_pose_distance <= (1*self._maxSpeed*self.time_increment):
                            raise Exception("Collision B: %d %.02f %.02f Other: %d %.02f %.02f"%(self.agent_id,self.pose.x,self.pose.y,vehicle.agent_id,vehicle.x,vehicle.y))
                    elif abs(angle_phi_to_phi)<np.pi/2 and (pose_to_pose_distance <= 1*self._maxSpeed*self.time_increment+ max_dimension):
                        # print("Rear-End",self.agent_id,"->",vehicle.agent_id)
                        return False
        return True

    def request_reservation_time_window(self,start_time,end_time):
        # Request a reservation from self.traffic_coordinator for a specific time window
        return_reservation = self.traffic_coordinator.request(start_time,end_time)
        print("Car",self.agent_id," Return",return_reservation)
        return return_reservation       

    def set_traffic_coordinator(self,traffic_coordinator):
        # Initialize a traffic coordinator
        self.traffic_coordinator = traffic_coordinator

    def check_in_rectangle(self,pose,x_interval_tuple,y_interval_tuple):
        # Provide it with two tuples of an x and y interval, check if pose is in that interval
        if x_interval_tuple[0] >= x_interval_tuple[1]:
            print("X Interval must be in increasing order")
        elif y_interval_tuple[0] >= y_interval_tuple[1]:
            print("Y interval must be in increasing order",y_interval_tuple)
        if (pose.x > x_interval_tuple[0]) and (pose.x < x_interval_tuple[1]) and (pose.y > y_interval_tuple[0]) and (pose.y < y_interval_tuple[1]):
            return True
        else:
            return False

    def check_in_control_region(self, pose):
        for (x_interval,y_interval) in self.control_regions.values():
            in_entrance = self.check_in_rectangle(pose,x_interval,y_interval)
            if in_entrance:
                return True
        return False   

    def check_in_map(self, pose):
        eps = 0.001*self.map_width
        return self.check_in_rectangle(pose,(-self.map_width/2.0-eps,self.map_width/2.0+eps),(-self.map_height/2.0-eps,self.map_height/2.0+eps))               

    def check_in_intersection_entrance(self,pose):
        # Check if vehicle approached intersection entrance
        for (x_interval,y_interval) in self.entrance_regions.values():
            in_entrance = self.check_in_rectangle(pose,x_interval,y_interval)
            if in_entrance:
                return True
        return False
    
    def check_in_intersection_exit(self,pose):
        # Check if vehicle approached intersection exit
        for (x_interval,y_interval) in self.exit_regions.values():
            in_entrance = self.check_in_rectangle(pose,x_interval,y_interval)
            if in_entrance:
                return True
        return False        

    def check_intersection(self, pose):
        # Check if vehicle approached intersection entrance
        intersection_center = (0,0)
        distance_to_intersection_center = (pose.x-intersection_center[0])**2+(pose.y-intersection_center[1])**2
        return distance_to_intersection_center <= self.lane_width*2

    def time_required_intersection(self,turn_direction,speed):
        # Computes time required to travel through the intersection.
        sample_entrance = self.entrance_regions["1"]
        sample_exit = self.exit_regions["1"]
        entrance_distance = abs(sample_entrance[0][0]-sample_entrance[0][1])
        exit_distance = abs(sample_exit[0][0]-sample_exit[0][1])
        if turn_direction == TurnDirection.LEFT:
            turn_distance = (np.pi/2)*1.5*self.lane_width
        elif turn_direction == TurnDirection.RIGHT:
            turn_distance = (np.pi/2)*0.5*self.lane_width
        elif turn_direction == TurnDirection.STRAIGHT:
            turn_distance = 2*self.lane_width 
        else:
            turn_distance = (np.pi/2)*1.5*self.lane_width
            raise Exception("Incorrect Turn",turn_direction)
        turn_distance += entrance_distance + exit_distance
        time = turn_distance/self._maxSpeed
        kTime_buffer_multiplier = 1.0
        time_required = kTime_buffer_multiplier * time 
        return time_required

    def get_time_to_intersection(self,current_pose=None, current_lane=None, current_speed=None):
        if current_pose == None:
            current_pose = self.pose
            current_lane = self.current_lane
            current_speed = self._maxSpeed            
        if current_lane == "1":
            distance_to_intersection = abs(-self.lane_width - current_pose.x)
        elif current_lane == "-1":
            distance_to_intersection = abs(self.lane_width - current_pose.x)
        elif current_lane == "2":
            distance_to_intersection = abs(-self.lane_width - current_pose.y)
        elif current_lane == "-2":
            distance_to_intersection = abs(self.lane_width - current_pose.y)
        else:
            distance_to_intersection = 0.2 #Agent is in entrance
        return distance_to_intersection/current_speed


    # def update_robot_pose(self,deltaT):
    #     if self.nextTask!=None:
    #         # nextTaskID = self.cbbaManager.p_path[0]
    #         # nextTask = self.cbbaManager.tasksList[nextTaskID]
    #         dist = self.speed*deltaT
    #         heading = np.arctan2(self.nextTask.pose.y-self.pose.y,self.nextTask.pose.x-self.pose.x)
    #         self.pose.x += dist*np.cos(heading)
    #         self.pose.y += dist*np.sin(heading)
    #         self.pose.phi = heading

    # def drive_step(self,timestep):
    #     if self.pose == None:
    #         raise Exception("Pose not initialized")
    #     else:
    #         dist = self.speed*deltaT
    #         direction = self.lane_to_pose[self.current_lane]
    #         heading = np.arctan2(self.nextTask.pose.y-self.pose.y,self.nextTask.pose.x-self.pose.x)
    #         self.pose.x += dist*np.cos(heading)
    #         self.pose.y += dist*np.sin(heading)
    #         self.pose.phi = heading    

    def simple_lights_permision(self,traffic_light):
        if traffic_light == 0:
            return True #ALLOW EVERYONE THROUGH
        elif traffic_light == -1:
            return False ## ALLOW NO ONE THROUGH
        elif self.current_lane == "2-1" or self.current_lane=="12" or self.current_lane == "-21" or self.current_lane=="-1-2":
            return True
        elif self.current_lane == "21" or self.current_lane=="1-2" or self.current_lane == "-2-1" or self.current_lane=="-12":
            return True
        elif traffic_light == "1" and (self.current_lane == "1" or self.current_lane == "-1"):
            return True
        elif traffic_light == "2" and (self.current_lane == "2" or self.current_lane == "-2"):
            return True
        else:
            return False


    def current_lane_to_intersection_lane(self,current_lane,exit_lane):
        intersection_lane = ""
        if self.exit_lane == "-1" and self.current_lane == "2":
            intersection_lane = "2-1"
        elif self.exit_lane == "2" and self.current_lane == "1":
            intersection_lane = "12"
        elif self.exit_lane == "1" and self.current_lane == "-2":
            intersection_lane = "-21"
        elif self.exit_lane == "-2" and self.current_lane == "-1":
            intersection_lane = "-1-2"
        elif self.current_lane == "1" and self.exit_lane == "-2":
            intersection_lane = "1-2"
        elif self.current_lane == "-1" and self.exit_lane == "2":
            intersection_lane = "-12"
        elif self.current_lane == "2" and self.exit_lane == "1":
            intersection_lane = "21"
        elif self.current_lane == "-2" and self.exit_lane == "-1":
            intersection_lane = "-2-1"
        else:
            intersection_lane = self.current_lane ## Not a legal turn, go straight, such as uturn
        #TODO: Add "11", "22"
        return intersection_lane

    def intersection_lane_to_exit_lane(self,current_lane):
        # Convert the current intersection lane to the exit lane
        if current_lane == "1-1":
            exit_lane = "-1"
        elif current_lane == "11":
            exit_lane = "1"
        elif current_lane == "12":
            exit_lane = "2"
        elif current_lane == "1-2":
            exit_lane = "-2"
        elif current_lane == "2-1":
            exit_lane = "-1"
        elif current_lane == "21":
            exit_lane = "1"
        elif current_lane == "22":
            exit_lane = "2"
        elif current_lane == "2-2":
            exit_lane = "-2"       
        elif current_lane == "-1-1":
            exit_lane = "-1"
        elif current_lane == "-11":
            exit_lane = "1"
        elif current_lane == "-12":
            exit_lane = "2"
        elif current_lane == "-1-2":
            exit_lane = "-2"         
        elif current_lane == "-2-1":
            exit_lane = "-1"
        elif current_lane == "-21":
            exit_lane = "1"
        elif current_lane == "-22":
            exit_lane = "2"
        elif current_lane == "-2-2":
            exit_lane = "-2"   
        else:
            raise Exception("Agent:",self.agent_id,"  Unknown Current Lane:", current_lane)
        return exit_lane  

    def turn_direction_to_intersection_lane(self,current_lane,turn_direction,notcommunicating=False):
        # Returns the intersection lane corresponding to current lane taking a turn_direction (L,R,S)
        if (turn_direction != TurnDirection.STRAIGHT) and (turn_direction != TurnDirection.LEFT) and (turn_direction != TurnDirection.RIGHT):
            raise Exception("Unknown turn direction:",turn_direction)
            # intersection_lane = "11"
        elif current_lane == "1":
            if notcommunicating:
                intersection_lane = "1?"              
            elif turn_direction == TurnDirection.STRAIGHT:
                intersection_lane = "11"
            elif turn_direction == TurnDirection.LEFT:
                intersection_lane = "12"
            elif turn_direction == TurnDirection.RIGHT:
                intersection_lane = "1-2"
        elif current_lane == "-1":
            if notcommunicating:
                intersection_lane = "-1?"              
            elif turn_direction == TurnDirection.STRAIGHT:
                intersection_lane = "-1-1"
            elif turn_direction == TurnDirection.LEFT:
                intersection_lane = "-1-2"
            elif turn_direction == TurnDirection.RIGHT:
                intersection_lane = "-12"                
        elif current_lane == "2":
            if notcommunicating:
                intersection_lane = "2?"                
            elif turn_direction == TurnDirection.STRAIGHT:
                intersection_lane = "22"
            elif turn_direction == TurnDirection.LEFT:
                intersection_lane = "2-1"
            elif turn_direction == TurnDirection.RIGHT:
                intersection_lane = "21"                
        elif current_lane == "-2":
            if notcommunicating:
                intersection_lane = "-2?"              
            elif turn_direction == TurnDirection.STRAIGHT:
                intersection_lane = "-2-2"
            elif turn_direction == TurnDirection.LEFT:
                intersection_lane = "-21"
            elif turn_direction == TurnDirection.RIGHT:
                intersection_lane = "-2-1"                   
        else:
            intersection_lane = current_lane
            # print("Weird:",current_lane)    
        return intersection_lane         


    def get_next_step_pose_lane(self,current_lane, distance, current_pose=False):
        if not current_pose:
            current_pose = self.pose
        new_pose = kn.Pose(None,None)
        if current_lane == None:
            print("What is a NONE lane?")
        elif current_lane == "1" or current_lane == "11":
            new_pose.x = current_pose.x + 1*distance
            new_pose.y = current_pose.y + 0
            new_pose.phi = 0
        elif current_lane == "-1" or current_lane == "-1-1":
            new_pose.x = current_pose.x -1*distance
            new_pose.y = current_pose.y + 0
            new_pose.phi = -np.pi
        elif current_lane == "2" or current_lane == "22":
            new_pose.x = current_pose.x + 0
            new_pose.y = current_pose.y + 1*distance
            new_pose.phi = np.pi/2
        elif current_lane == "-2" or current_lane == "-2-2":
            new_pose.x = current_pose.x + 0
            new_pose.y = current_pose.y -1*distance
            new_pose.phi = -np.pi/2
        elif current_lane == "0":
            new_pose.x = current_pose.x + 0
            new_pose.y = current_pose.y + 0
        elif current_lane == "2-1":  #LEFT TURNS
            denom_sqrt =((current_pose.x+self.lane_width)**2+(current_pose.y+self.lane_width)**2)**0.5
            # distance = distance/4.0
            dy = distance * (current_pose.x + self.lane_width) /denom_sqrt
            dx = -distance * (current_pose.y + self.lane_width) / denom_sqrt 
            new_pose.phi = np.arctan2(dy,dx) # I SHOULD DO THE MATH for PHI
            new_pose.x = current_pose.x + dx
            new_pose.y = current_pose.y + dy
        elif current_lane == "12":
            denom_sqrt =((current_pose.x+self.lane_width)**2+(current_pose.y-self.lane_width)**2)**0.5
            # distance = distance/4.0
            dy = distance * (current_pose.x + self.lane_width) /denom_sqrt
            dx = -distance * (current_pose.y - self.lane_width) / denom_sqrt 
            new_pose.phi = np.arctan2(dy,dx) 
            new_pose.x = current_pose.x + dx
            new_pose.y = current_pose.y + dy
        elif current_lane == "-21":
            denom_sqrt =((current_pose.x-self.lane_width)**2+(current_pose.y-self.lane_width)**2)**0.5
            # distance = distance/4.0
            dy = distance * (current_pose.x - self.lane_width) /denom_sqrt
            dx = -distance * (current_pose.y - self.lane_width) / denom_sqrt 
            new_pose.phi = np.arctan2(dy,dx) 
            new_pose.x = current_pose.x + dx
            new_pose.y = current_pose.y + dy   
        elif current_lane == "-1-2":
            denom_sqrt =((current_pose.x-self.lane_width)**2+(current_pose.y+self.lane_width)**2)**0.5
            # distance = distance/4.0
            dy = distance * (current_pose.x - self.lane_width) /denom_sqrt
            dx = -distance * (current_pose.y + self.lane_width) / denom_sqrt 
            new_pose.phi = np.arctan2(dy,dx) 
            new_pose.x = current_pose.x + dx
            new_pose.y = current_pose.y + dy
        elif current_lane == "21":  #RIGHT TURNS
            denom_sqrt =((current_pose.x-self.lane_width)**2+(current_pose.y+self.lane_width)**2)**0.5
            dy = np.abs(distance * (current_pose.x - self.lane_width) /denom_sqrt)
            dx = np.abs(distance * (current_pose.y + self.lane_width) / denom_sqrt)
            new_pose.phi = np.arctan2(dy,dx) 
            new_pose.x = current_pose.x + dx
            new_pose.y = current_pose.y + dy
        elif current_lane == "1-2":
            denom_sqrt =((current_pose.x+self.lane_width)**2+(current_pose.y+self.lane_width)**2)**0.5
            dy = -np.abs(distance * (current_pose.x + self.lane_width) /denom_sqrt)
            dx = np.abs(distance * (current_pose.y + self.lane_width) / denom_sqrt) 
            new_pose.phi = np.arctan2(dy,dx) 
            new_pose.x = current_pose.x + dx
            new_pose.y = current_pose.y + dy
        elif current_lane == "-2-1":
            denom_sqrt =((current_pose.x+self.lane_width)**2+(current_pose.y-self.lane_width)**2)**0.5
            dy = -np.abs(distance * (current_pose.x + self.lane_width) /denom_sqrt)
            dx = -np.abs(distance * (current_pose.y - self.lane_width) / denom_sqrt)
            new_pose.phi = np.arctan2(dy,dx) 
            new_pose.x = current_pose.x + dx
            new_pose.y = current_pose.y + dy   
        elif current_lane == "-12":
            denom_sqrt =((current_pose.x-self.lane_width)**2+(current_pose.y-self.lane_width)**2)**0.5
            dy = np.abs(distance * (current_pose.x - self.lane_width) /denom_sqrt)
            dx = -np.abs(distance * (current_pose.y - self.lane_width) / denom_sqrt)
            new_pose.phi = np.arctan2(dy,dx)
            new_pose.x = current_pose.x + dx
            new_pose.y = current_pose.y + dy                                                           
        else:
            raise Exception("Unrecognized Lane:",current_lane)
        return new_pose

    def get_next_pose_bike(self, turning_angle, speed=None, current_pose=None, time_increment=None):
        u1 = 0 #acceleration
        u2 = turning_angle
        new_pose = kn.Pose(None,None)
        if speed == None:
            speed = self._maxSpeed
        if current_pose == None:
            current_pose = self.pose
        if time_increment == None:
            time_increment = self.time_increment
        V = speed
        psi = current_pose.phi
        X_dot = V*np.cos(psi)
        Y_dot = V*np.sin(psi)
        V_dot = u1
        lr = self.car_length/2.0
        lf = self.car_length/2.0
        beta = np.arctan(np.tan(u2)/2.0)
        psi_dot = V / lr *np.sin(beta)

        new_pose.x = current_pose.x + X_dot*time_increment
        new_pose.y = current_pose.y + Y_dot*time_increment
        new_pose.phi = current_pose.phi + psi_dot*time_increment               
        return new_pose                

    def get_next_step_pose_lane_dubins(self,current_lane, distance, current_pose=False):
        if not current_pose:
            current_pose = self.pose
        new_pose = kn.Pose(None,None)
        if current_lane == "2-1" or current_lane == "12" or current_lane == "-21" or current_lane == "-1-2":  #LEFT TURNS
            # u1 = acceleration
            # u2 = steeting angle
            u1 = 0
            u2 = np.pi/4
        elif current_lane == "21" or current_lane == "1-2" or current_lane == "-2-1" or current_lane == "-12":  #RIGHT TURNS
            u1 = 0 
            u2 = -np.pi/4                                                
        else:
            u1 = 0
            u2 = 0
            # raise Exception("Unrecognized Lane:",current_lane)

        # https://hal-polytechnique.archives-ouvertes.fr/hal-01520869/document
        V = self._maxSpeed
        psi = current_pose.phi
        X_dot = V*np.cos(psi)
        Y_dot = V*np.sin(psi)
        V_dot = u1
        lr = self.car_length/2.0
        lf = self.car_length/2.0
        beta = np.arctan(np.tan(u2)/2.0)
        psi_dot = V / lr *np.sin(beta)

        new_pose.x = current_pose.x + X_dot*self.time_increment
        new_pose.y = current_pose.y + Y_dot*self.time_increment
        new_pose.phi = current_pose.phi + psi_dot*self.time_increment                
        return new_pose        

    def initialize_beginning_lane(self,current_lane, exit_lane=None):
        # +1 represents traveling in the +x direction
        # -2 represents traveling in the -y direction
        self.current_lane = current_lane
        self.initial_lane = current_lane
        if current_lane == None:
            print("What is a 2 lane?")
        elif current_lane == "1":
            self.pose.x = -0.5*self.map_width
            self.pose.y = -0.5*self.lane_width
            self.pose.phi = 0
        elif current_lane == "-1":
            self.pose.x = 0.5*self.map_width
            self.pose.y = 0.5*self.lane_width
            self.pose.phi = -np.pi
        elif current_lane == "2":
            self.pose.x = 0.5*self.lane_width
            self.pose.y = -0.5*self.map_height
            self.pose.phi = np.pi/2
        elif current_lane == "-2":
            self.pose.x = -0.5*self.lane_width
            self.pose.y = 0.5*self.map_height
            self.pose.phi = -np.pi/2
        elif current_lane == "0":
            self.pose.x = 0
            self.pose.y = 0   
        # self.pose.phi = 0
        

    def distance_in_intersection(self, turn_direction):
        if turn_direction == TurnDirection.STRAIGHT:
            distance = 2*self.lane_width
        elif turn_direction == TurnDirection.LEFT:
            distance = np.pi/2.0*self.lane_width*1.5
        elif turn_direction == TurnDirection.RIGHT:
            distance = np.pi/2.0*self.lane_width*0.5
        else:
            print("Unknown turn direction")
            distance = 0
        return distance

    def get_random_turn(self, probability_left_turn, probability_right_turn):
        # Generate a random turn for the car.  Specify the probability of a car turning left and right
        # Probability straight = 1 - p_left - p_right
        
        rand_turn_number = np.random.random()
        new_turn_direction = None
        if rand_turn_number < probability_left_turn:
            new_turn_direction = TurnDirection.LEFT
        elif rand_turn_number < (probability_left_turn + probability_right_turn):
            new_turn_direction = TurnDirection.RIGHT
        else:
            new_turn_direction = TurnDirection.STRAIGHT
        return new_turn_direction        
                                       
        
    def move(self):
        raise NotImplementedError

    def plan(self):
        raise NotImplementedError

    def sense(self):
        raise NotImplementedError

    def broadcast(self,msg):
        raise NotImplementedError

    def listen(self,msg):
        raise NotImplementedError

