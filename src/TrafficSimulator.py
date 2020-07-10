#### Author:  Noam Buckman
#### Description:  TrafficSimulator class runs the simulation by managing
####             the dynamics of the AVs and holds the TrafficCoordinator.
####            Contains methods for plotting the simulation, generating simulations,
####            and intitial vehicles.

import sys, copy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms

# My custom packages
import kinematics as kn
import TrafficCoordinator as tc
import AutonomousVehicle as AV

class TrafficSimulator:
    '''
        TrafficSimulator generates the simulation/traffic environment and 
        manages the evolution of the simulation (dynamics of AVs and running of Coordinator)
        
        Many methods included for plotting the simulation.
    '''
    def __init__(self):

        self.fig = None
        self.axes = None #Is this a good idea?

        self.total_vehicle_list = []
        self.active_vehicles = set()
        self.time = 0
        self.deltaT = 0.01
        self.simulation_over = False
        plt.ion()
        self.previous_time = 0

        self.simulation_iteration = -1

        #MAP DEFAULTS
        self.map_width = 10
        self.map_height = 10
        self.gridsize = 0.01
        self.n_lanes = 4
        self.lane_width = 1.0
        self.entrance_regions = {}
        self.exit_regions = {}
        self.control_regions = {}

        self.posson_lam = 1/10 #one very 10 time steps

        self.traffic_light = 1
        self.occupancy_map = {}
        self.coordinator = tc.MatrixTrafficCoordinator()
        self.next_agent_id = 0
        self.iterations_until_next_car = -1     

        # SAVE HISTORY
        self.history_times = []
        self.history_poses_speeds = []
        self.batch = False

        self.k_max_time_in_control = 45.0 #s 

        self.construction_zones = [] 


    def initialize_map(self,width,height,n_lanes,gridsize=0.01):
        self.map_width = width
        self.map_height = height
        self.n_lanes = n_lanes
        self.gridsize = gridsize

        self.entrance_regions = {} #specifying [x,x] and [y,y]
        self.entrance_regions["-1"] = ((self.lane_width,0.2+self.lane_width),(0,self.lane_width))
        self.entrance_regions["1"] = ((-self.lane_width-0.2,-self.lane_width),(-self.lane_width,0))
        self.entrance_regions["2"] = ((0,self.lane_width),(-self.lane_width-0.2,-self.lane_width))
        self.entrance_regions["-2"] = ((-self.lane_width,0),(self.lane_width,self.lane_width+0.2))       

        self.control_regions = {} #specifying [x,x] and [y,y]
        self.control_regions["-1"] = ((self.lane_width,self.lane_width+0.9*(self.map_width/2-self.lane_width)),(0,self.lane_width))
        self.control_regions["1"] = ((-self.lane_width-0.9*(self.map_width/2-self.lane_width),-self.lane_width),(-self.lane_width,0))
        self.control_regions["2"] = ((0,self.lane_width),(-self.lane_width-0.9*(self.map_height/2-self.lane_width),-self.lane_width))
        self.control_regions["-2"] = ((-self.lane_width,0),(self.lane_width,self.lane_width+(self.map_height/2-self.lane_width)*0.9))  

        self.exit_regions = {}
        self.exit_regions["1"] = ((self.lane_width,self.lane_width+0.2),(-self.lane_width,0))
        self.exit_regions["-1"] = ((-self.lane_width-0.2,-self.lane_width),(0,self.lane_width))
        self.exit_regions["2"] = ((0,self.lane_width),(self.lane_width,self.lane_width+0.2))
        self.exit_regions["-2"] = ((-self.lane_width,0),(-self.lane_width-0.2,-1*self.lane_width))        



    def load_list_vehicles(self,list_vehicles):
        self.total_vehicle_list = []
        for vehicle in list_vehicles:
            self.add_vehicle(vehicle)  

    def get_vehicle_from_id(self,agent_id):
        returned_robot = None
        for robot in self.total_vehicle_list:
            if robot.agent_id == agent_id:
                returned_robot = robot 
        return returned_robot          
    
    def update_robot_map_specs(self, robot):
        robot.lane_width = self.lane_width
        robot.map_width = self.map_width
        robot.map_height = self.map_height
        robot.entrance_regions = self.entrance_regions
        robot.exit_regions = self.exit_regions 
        robot.control_regions = self.control_regions   
    
    def add_vehicle(self, vehicle, id_assigned_at_map_entrance=False):
        ''' Add a robot into the simulator '''
        self.update_robot_map_specs(vehicle)
        
        if id_assigned_at_map_entrance:
            vehicle.agent_id = self.next_agent_id
            self.next_agent_id += 1
        else:
            vehicle.agent_id = -1
        self.total_vehicle_list += [vehicle]
        self.active_vehicles.add(vehicle)
        # self.nTotal = len(self.total_vehicle_list)

    def increment_time(self, delta_t=None):
        ''' Main '''
        if delta_t == None:
            delta_t = self.deltaT
        # self.deltaT = delta_t
        self.time += delta_t
        self.simulation_iteration += 1

        # Assign agent IDs according to arrival time into control region
        vehicles_to_assign_agent_ids = []        
        for vehicle in self.active_vehicles:
            if vehicle.agent_id == -1 and vehicle.time_entered_control > -1 and (vehicle.time_entered_control < self.time - delta_t - 0.00001):
                vehicles_to_assign_agent_ids += [vehicle]
        if len(vehicles_to_assign_agent_ids) > 0:
            self.assign_ids_vehicles(vehicles_to_assign_agent_ids)

        #UPDATE ACTIVE AGENTS
        new_active_robots = set()
        for vehicle in self.active_vehicles:
            if vehicle.active:
                new_active_robots.add(vehicle)
        self.active_vehicles = new_active_robots

        for vehicle in self.active_vehicles:
            vehicle.time = self.time
        self.occupancy_map = self.generate_occupancy_map()
        np.random.shuffle(self.total_vehicle_list)

        for vehicle in self.active_vehicles:
            vehicle.loop(delta_t,self.traffic_light,self.occupancy_map)
            if (vehicle.time_entered_control > 0) and (self.time - vehicle.time_entered_control) > self.k_max_time_in_control:
                raise Exception("Agent", vehicle.agent_id,"in Intersection longer than",self.k_max_time_in_control,"s in intersection")
        
        if self.batch:
            self.coordinator.time = self.time
            self.coordinator.increment(self.deltaT)
        robot_histories = [(vehicle.agent_id,vehicle.pose,vehicle.speed) for vehicle in self.active_vehicles]
        self.history_poses_speeds.append(robot_histories)
        self.history_times.append(self.time)

        return True

    def assign_ids_vehicles(self, vehicles_to_assign_agent_ids):
        # print("Assigning to",[v.initial_lane for v in vehicles_to_assign_agent_ids])

        next_agent_id = self.next_agent_id
        sorted_by_lane = sorted(vehicles_to_assign_agent_ids, key = lambda veh: int(veh.initial_lane))
        for i in range(len(sorted_by_lane)):
            sorted_by_lane[i].agent_id = next_agent_id + i
        self.next_agent_id = next_agent_id + len(sorted_by_lane)

    def generate_occupancy_map(self):
        occupancy_map = {}
        for vehicle in self.active_vehicles:
            occupancy_map[vehicle]=vehicle.pose
        return occupancy_map

    def update_robot_time(self, robot):
        robot.time = self.time #Update robots internal clock

    def updateRobotPose(self,robot):
        return None

    def robotColor(self, robot):
        colors = ['brown','blue','m','y','grey']
        if robot.notcommunicating:
            return 'r'
        else:
            i = robot.agent_id%len(colors)
            return colors[i]

    def add_construction_zone(self, outgoing_lane, relative_lane_position = 0.5, start_time = None, end_time=None):
        if start_time == None:
            start_time = self.time
        if end_time == None:
            end_time = 999999999999999
        
        if outgoing_lane != "1" and outgoing_lane != "-1" and outgoing_lane != "2" and outgoing_lane != "-2":
            raise Exception("Incorrect Outgoing Lane.  Integer lane is required")

        if outgoing_lane == "1":
            construction_x = self.lane_width + relative_lane_position*(self.map_width/2.0 - self.lane_width) 
            construction_y = -self.lane_width/2.0
        elif outgoing_lane == "-1":
            construction_x = -self.lane_width - relative_lane_position*(-self.map_width/2.0 + self.lane_width )
            construction_y = self.lane_width/2.0
        elif outgoing_lane == "2":
            construction_x = self.lane_width/2.0
            construction_y = self.lane_width + relative_lane_position*(self.map_height/2.0 - self.lane_width) 
        elif outgoing_lane == "-2":
            construction_x = -self.lane_width/2.0
            construction_y = -self.lane_width - relative_lane_position*(-self.map_height/2.0 + self.lane_width )
        self.construction_zones += [(start_time, end_time, construction_x,construction_y)]

    def robotColorFromID(self,agent_id):
        colors = ['r','g','brown','blue','m','y','pink','orange','grey']
        i = agent_id%len(colors)
        return colors[i]

    def robotColorSVO(self, robot):
        if 0 - 0.1 <= robot.svo_theta <= 0+0.1:
            color = 'r'
        elif np.pi/6 - 0.1 <= robot.svo_theta <= np.pi/6 + 0.1:
            color = 'purple'
        elif np.pi/4 - 0.1 <= robot.svo_theta <= np.pi/4 + 0.1:
            color = 'b'
        else:
            color = 'k'
            print("Error: No SVO")
        return color


    def plot_robots(self,fsize=(12,12),plot_lanes_flag=True,annotate_flag=True,plot_intersection_region_flag=True,plot_reservation_patches_flag=False,plot_only_intersection_flag=False):
        ''' Plot autonomous vehicles in intersection.  Optional flags included for plotting reservation'''
        #TODO:  Add argument parser for all the different flags


        fig = plt.figure(figsize=fsize)
        ax = fig.add_subplot(111,aspect='equal')
        self.fig = fig
        self.axes = ax
        ax.set_ylim([-self.map_height/2.5,self.map_height/2.5])
        ax.set_xlim([-self.map_width/2.5,self.map_width/2.5])
        ax.set_axis_off()
        

        if plot_lanes_flag:
            self.plot_lanes(ax)
        if annotate_flag:
            self.annotate_axis(ax)
        if plot_intersection_region_flag:
            self.add_intersection_regions(ax)               
        if plot_reservation_patches_flag:
            self.add_reservation_patches(self.coordinator,ax)    
        if plot_only_intersection_flag:
            ax.set_ylim([-self.lane_width,self.lane_width])
            ax.set_xlim([-self.lane_width,self.lane_width])
            ticks = np.arange(-self.lane_width,self.lane_width,self.gridsize)
            ax.set_xticks(ticks)
            ax.set_yticks(ticks)

        ax.axhline(0,-self.map_width/2.0,-self.lane_width,linestyle='--',color='k',linewidth=10)
        ax.axhline(0,self.lane_width, self.map_width/2.0,linestyle='--',color='k')
        ax.axvline(0,self.lane_width, self.map_width/2.0, linestyle='--',color='k')
        ax.axvline(0,-self.map_width/2.0,-self.lane_width,linestyle='--',color='k')


        plot_construction_flag = True
        if plot_construction_flag:
            self.plot_construction(ax)

        addToLegend = False
        for robot in self.active_vehicles:
            if robot.intersection_window[0] > 0 and False:
                edge_color_reservation = 'g'
            else:
                edge_color_reservation = None

            current_color = self.robotColor(robot)
            # current_color = self.robotColorSVO(robot)
            car_patch = patches.Rectangle((robot.pose.x-robot.car_length,robot.pose.y-robot.car_width/2.0),robot.car_length,robot.car_width,facecolor=current_color,edgecolor=edge_color_reservation,linewidth=3)
            t = transforms.Affine2D().rotate_around(robot.pose.x, robot.pose.y, robot.pose.phi)
            car_patch.set_transform(t + ax.transData)
            ax.add_patch(car_patch)
            offset_x = -3 - robot.car_length/2.0*np.cos(robot.pose.phi)*20
            offset_y = -3 - robot.car_length/2.0*np.sin(robot.pose.phi)*20
            new_id = robot.agent_id

            ax.annotate(str(new_id),(robot.pose.x,robot.pose.y),xytext=(offset_x, offset_y),size=8,textcoords='offset points',color='black')
            arrow_direction = robot.turn_direction
            dx, dy = self.turn_direction_to_delta_x_y_arrow(robot)
            arrow_length = 0.1*self.lane_width
            plt.arrow(robot.pose.x,robot.pose.y, dx*arrow_length, dy*arrow_length,width=.05)
            # ax.plot(robot.pose.x,robot.pose.y,'o', c='k',markersize=4)            
            

    
    def plot_construction(self,ax):
        for (start_time, end_time, construction_x, construction_y) in self.construction_zones:
            if start_time < self.time < end_time:
                ax.plot(construction_x, construction_y,'x',color='r')
    
    def turn_direction_to_delta_x_y_arrow(self, car):
        turn_direction = car.turn_direction
        if turn_direction == AV.TurnDirection.RIGHT:
            dx = 0
            dy = -1
        elif turn_direction == AV.TurnDirection.STRAIGHT:
            dx = 1
            dy = 0
        elif turn_direction == AV.TurnDirection.LEFT:
            dx = 0
            dy = 1
            
        dx_prime = dx*np.cos(car.pose.phi) - dy*np.sin(car.pose.phi)
        dy_prime = dx*np.sin(car.pose.phi) + dy*np.cos(car.pose.phi) 
        return dx_prime, dy_prime

    def plot_lanes(self, ax):
        lane_1_patch = patches.Rectangle((-self.map_width/2,-self.lane_width), self.map_width/2.0-self.lane_width, self.lane_width,facecolor='white',edgecolor='k',linewidth=1)
        alane_1_patch = patches.Rectangle((self.lane_width,-self.lane_width), self.map_width/2.0-self.lane_width, self.lane_width,facecolor='white',edgecolor='k', linewidth=1)
        
        
        
        lane_m1_patch = patches.Rectangle((self.lane_width,0), self.map_width/2.0-self.lane_width, self.lane_width,facecolor='white',edgecolor='k',linewidth=1)
        alane_m1_patch = patches.Rectangle((-self.map_width/2,0), self.map_width/2.0-self.lane_width, self.lane_width,facecolor='white',edgecolor='k',linewidth=1)

        lane_2_patch = patches.Rectangle((-self.lane_width,-self.map_height/2),self.lane_width,self.map_height/2.0 - self.lane_width,facecolor='white',edgecolor='k',linewidth=1)
        lane_m2_patch = patches.Rectangle((0,-self.map_height/2),self.lane_width,self.map_height/2.0 - self.lane_width,facecolor='white',edgecolor='k',linewidth=1)

        alane_2_patch = patches.Rectangle((-self.lane_width,self.lane_width),self.lane_width,self.map_height/2.0 - self.lane_width,facecolor='white',edgecolor='k',linewidth=1)
        alane_m2_patch = patches.Rectangle((0,self.lane_width),self.lane_width,self.map_height/2.0 - self.lane_width,facecolor='white',edgecolor='k',linewidth=1)


        ax.add_patch(lane_1_patch)
        ax.add_patch(lane_m1_patch)
        ax.add_patch(lane_2_patch)
        ax.add_patch(lane_m2_patch)


        # alane_2_patch = patches.Rectangle((-self.lane_width,-self.map_height/2),self.lane_width,self.map_height,facecolor='white')
        # alane_m2_patch = patches.Rectangle((0,-self.map_height/2),self.lane_width,self.map_height,facecolor='white')

        ax.add_patch(alane_1_patch)
        ax.add_patch(alane_m1_patch)
        ax.add_patch(alane_2_patch)
        ax.add_patch(alane_m2_patch)

    def add_intersection_regions(self, ax):
        for exit_region in self.exit_regions.values():
            lower_x = exit_region[0][0]
            lower_y = exit_region[1][0]
            width = exit_region[0][1]-exit_region[0][0]
            height = exit_region[1][1]-exit_region[1][0]
            exit_patch = patches.Rectangle((lower_x,lower_y),width,height,facecolor='red',alpha=0.25)
            ax.add_patch(exit_patch)
        for entrance_region in self.entrance_regions.values():
            lower_x = entrance_region[0][0]
            lower_y = entrance_region[1][0]
            width = entrance_region[0][1]-entrance_region[0][0]
            height = entrance_region[1][1]-entrance_region[1][0]
            entrance_patch = patches.Rectangle((lower_x,lower_y),width,height,facecolor='green',alpha=0.25)
            ax.add_patch(entrance_patch)            
        for entrance_region in self.control_regions.values():
            lower_x = entrance_region[0][0]
            lower_y = entrance_region[1][0]
            width = entrance_region[0][1]-entrance_region[0][0]
            height = entrance_region[1][1]-entrance_region[1][0]
            entrance_patch = patches.Rectangle((lower_x,lower_y),width,height,facecolor='grey',alpha=0.25)
            ax.add_patch(entrance_patch)                   

    def annotate_axis(self, ax):
        ax.annotate('%.03f'%self.time,(.4*self.map_width,-0.4*self.map_height))
        ax.annotate("+1 Lane",(-self.map_width/2,-self.lane_width))
        ax.annotate("-1 Lane",(self.map_width/2,self.lane_width))
        ax.annotate("+2 Lane",(self.lane_width,-self.map_height/2))
        ax.annotate("-2 Lane",(-self.lane_width,self.map_height/2))


    def generate_car(self, p_notcomm = 0.1, probability_left_turn=0.33, probability_right_turn=0.33):
        ''' Create new autonomous vehicle.  Returns None if it will collide with car in system'''

        new_car = AV.AutonomousVehicle(-1)
        new_car.set_traffic_coordinator(self.coordinator)
        self.update_robot_map_specs(new_car)
        new_car_lane = np.random.choice(["-2","2","-1","1"])      
        all_lanes = ["2","-1","-2","1"]
        # all_lanes = ["-2","2"]
        new_car_lane = all_lanes[self.next_agent_id%len(all_lanes)]

        new_car.initialize_beginning_lane(new_car_lane)
        new_car.turn_direction = new_car.get_random_turn(probability_left_turn, probability_right_turn)
        r= np.random.random()
        if 0.0001 < r <= p_notcomm:
            new_car.notcommunicating = True
        else:
            new_car.notcommunicating = False
        # if self.next_agent_id%2 == 0:
        #     new_car.notcommunicating = True
        # else:
        #     new_car.notcommunicating = False
        
        # Check if lane is free
        new_pose = new_car.get_next_step_pose_lane(new_car.current_lane,self.deltaT*new_car._maxSpeed)
        no_collision = new_car.collision_free(new_pose,self.occupancy_map,new_car.pose)
        if no_collision:
            return new_car
        else:
            return None
    
    def generate_new_random_car(self, time_between_cars_multiplier=1.0, p_notcomm = 0.1, probability_left_turn=0.33, probability_right_turn=0.33):
        ''' Generate cars according to Poisson process'''

        if self.iterations_until_next_car <= 0:
            new_car = self.generate_car(p_notcomm, probability_left_turn,probability_right_turn)
            if new_car:
                self.add_vehicle(new_car)
            time_between_cars = new_car.time_required_intersection(AV.TurnDirection.STRAIGHT,new_car._maxSpeed)
            number_iterations = time_between_cars_multiplier*time_between_cars/self.deltaT
            posson_lambda = 1/number_iterations        
            self.iterations_until_next_car = np.random.exponential(1/posson_lambda)
        else:
            self.iterations_until_next_car -= 1    

    def add_reservation_patches(self, traffic_coordinator, ax):
        ''' Add axis patches for each reservation square in the reservation matrix'''
        reservation_matrix = traffic_coordinator.get_reservation_matrix()
        m = traffic_coordinator.m
        n = traffic_coordinator.n
        dx = traffic_coordinator.dx
        if m > 0:
            for i in range(m):
                for j in range(n):
                    for (start, end, agent_id) in reservation_matrix[i][j]:
                        time_until_start_reservation = start - self.time
                        k_window_plotting_reservations = 5.0
                        if 0 < time_until_start_reservation < k_window_plotting_reservations:
                            lower_x = -1 - 0.5*dx + dx*j
                            lower_y = 1 - 0.5*dx - dx*i
                            width = dx
                            height = dx
                            reservation_patch = patches.Rectangle((lower_x,lower_y),width,height,facecolor=self.robotColorFromID(agent_id),alpha=1-(time_until_start_reservation/k_window_plotting_reservations))
                            ax.add_patch(reservation_patch)        
    
    def turn_direction_to_delta_x_y_arrow_history(self, phi, turn_direction):
        if turn_direction == AV.TurnDirection.RIGHT:
            dx = 0
            dy = -1
        elif turn_direction == AV.TurnDirection.STRAIGHT:
            dx = 1
            dy = 0
        elif turn_direction == AV.TurnDirection.LEFT:
            dx = 0
            dy = 1
            
        dx_prime = dx*np.cos(phi) - dy*np.sin(phi)
        dy_prime = dx*np.sin(phi) + dy*np.cos(phi) 
        return dx_prime, dy_prime
        
    def plot_robot_histories(self, poses_information, robot_id_map):
        fig = plt.figure(figsize=(4,3))
        ax = fig.add_subplot(111,aspect='equal')
        self.fig = fig
        self.axes = ax
        ax.set_ylim([-self.map_height/2.5,self.map_height/2.5])
        ax.set_xlim([-self.map_width/2.5,self.map_width/2.5])

        edge_color_reservation = None
        self.plot_lanes(ax)
        # self.add_reservation_patches(self.coordinator,ax)    

        # self.annotate_axis(ax)
        self.add_intersection_regions(ax)               
        ax.axhline(0,-self.map_width/2.0,-self.lane_width,linestyle='--',color='k',linewidth=10)
        ax.axhline(0,self.lane_width, self.map_width/2.0,linestyle='--',color='k')
        ax.axvline(0,self.lane_width, self.map_width/2.0, linestyle='--',color='k')
        ax.axvline(0,-self.map_width/2.0,-self.lane_width,linestyle='--',color='k')

        for robot_pose_history in poses_information:
            robot, x, y, phi, svo, turn_direction = robot_pose_history

            # current_color = self.robotColor(robot)
            current_color = self.robotColorSVO(robot)
            car_patch = patches.Rectangle((x-robot.car_length,y-robot.car_width/2.0),robot.car_length,robot.car_width,facecolor=current_color,edgecolor=edge_color_reservation,linewidth=3)
            t = transforms.Affine2D().rotate_around(x, y, phi)
            car_patch.set_transform(t + ax.transData)
            ax.add_patch(car_patch)
            offset_x = -3 - robot.car_length/2.0*np.cos(phi)*20
            offset_y = -3 - robot.car_length/2.0*np.sin(phi)*20
            # new_id = self.get_id_label_experiment_2(robot)
            new_id = robot_id_map[robot.agentID]
            ax.annotate(str(new_id),(x,y),xytext=(offset_x, offset_y),size=8,textcoords='offset points',color='black')
            arrow_direction = turn_direction
            dx, dy = self.turn_direction_to_delta_x_y_arrow_history(phi, turn_direction)
            arrow_length = 0.1*self.lane_width
            plt.arrow(x,y, dx*arrow_length, dy*arrow_length,width=.05)
            # ax.plot(robot.pose.x,robot.pose.y,'o', c='k',markersize=4)    




########################################################################################

def generate_simulation_cars(number_of_agents,  p_left, p_right, mean_arrival_space, lane_choice="Random", p_notcomm=0.5, possible_svo_thetas=[np.pi/4.0]):
    ''' Arrival Time, Turn Direction, Incoming Lane, notcommunicating?
    '''
    arrival_time = 0
    
    all_cars = []
    for agent_id in range(number_of_agents):          
        #Choose a Lane
        all_lanes = ["1","2","-1","-2"]
        if lane_choice == "Sequential":
            start_lane = all_lanes[agent_id%4]
        else:
            start_lane = np.random.choice(all_lanes)
        
        #Choose a Turn Direction
        coin_flip = np.random.random()
        if coin_flip < p_left:
            turn_direction = "Left"
        elif coin_flip < p_right + p_left:
            turn_direction = "Right"
        else:
            turn_direction = "Straight"

        #Choose notcommunicating Type
        coin_flip = np.random.random()
        if coin_flip < p_notcomm:
            notcomm_type = "Not Communicating"
        else:
            notcomm_type = "Communicating"

        
        svo_theta = np.random.choice(possible_svo_thetas)
        
        all_cars += [(arrival_time, agent_id, notcomm_type, svo_theta, turn_direction, start_lane)]
        
        arrival_time += np.random.exponential(mean_arrival_space)
        
    return all_cars


def run_swap_experiments(traffic_simulator, all_cars,  number_cars_to_sim=999999, number_iterations_before_plotting=99999999):
    wait_times_notcomm = []
    thetas_SVO_notcomm = []
    wait_times_AVs = []
    all_swaps = []
    wait_times = []
    exit_times = []
            
    next_agent_id = 0
    (arrival_time, agent_id, notcommunicating_type, svo_theta, turn_direction, start_lane) = all_cars[next_agent_id]
    i = 0
    while traffic_simulator.time < 120.0:
        i+=1
        traffic_simulator.increment_time()

        if i%number_iterations_before_plotting == 0:
            traffic_simulator.plot_robots((3,2))
            plt.show()

        if (i % traffic_simulator.coordinator.k_number_iterations_between_cleaning_time_matrix) == 0:
            traffic_simulator.coordinator.remove_expired_intervals_from_matrix(traffic_simulator.time)
            
        if traffic_simulator.time > arrival_time:
            new_car = AV.AutonomousVehicle(agent_id)
            new_car.set_traffic_coordinator(traffic_simulator.coordinator)
            traffic_simulator.update_robot_map_specs(new_car)
            new_car.initialize_beginning_lane(start_lane)
            
            if turn_direction == 'Left' or turn_direction == "Left" or turn_direction == "LEFT" or turn_direction == 'LEFT':
                new_car.turn_direction = AV.TurnDirection.LEFT
            elif turn_direction == 'Right' or turn_direction == "Right" or turn_direction == "RIGHT" or turn_direction == 'RIGHT':
                new_car.turn_direction = AV.TurnDirection.RIGHT
            else:
                new_car.turn_direction = AV.TurnDirection.STRAIGHT
                
            
            if notcommunicating_type == "Not Communicating" or notcommunicating_type == "not communicating" or notcommunicating_type == "Not communicating":
                new_car.notcommunicating = True
            else:
                new_car.notcommunicating = False
            
            # add 3/22: force notcommunicatings to be ego
            # if new_car.notcommunicating:
            #     svo_theta = 0 
            new_car.svo_theta = svo_theta
            
            new_pose = new_car.get_next_step_pose_lane(new_car.current_lane,traffic_simulator.deltaT*new_car._maxSpeed)
            no_collision = new_car.collision_free(new_pose,traffic_simulator.occupancy_map,new_car.pose)
            if no_collision:
                traffic_simulator.add_vehicle(new_car)
                next_agent_id += 1
                if next_agent_id < len(all_cars) and next_agent_id < number_cars_to_sim:
                    (arrival_time, agent_id, notcommunicating_type, svo_theta, turn_direction, start_lane) = all_cars[next_agent_id]
                else:
                    arrival_time = 99999999999999999999999
            else:
                pass
        if traffic_simulator.time>10.0 and len(traffic_simulator.active_vehicles)==0:
            break                
    exit_times += [car.time_exited_intersection for car in traffic_simulator.total_vehicle_list if car.time_entered_intersection!=-1]
    wait_times += [car.time_entered_intersection-car.time_entered_control for car in traffic_simulator.total_vehicle_list if car.time_entered_intersection!=-1]
    all_swaps += traffic_simulator.coordinator.list_of_swapped_reservations
    wait_times_notcomm += [car.time_entered_intersection-car.time_entered_control for car in traffic_simulator.total_vehicle_list if car.time_entered_intersection!=-1 and car.notcommunicating]
    thetas_SVO_notcomm += [car.svo_theta for car in traffic_simulator.total_vehicle_list if car.time_entered_intersection!=-1 and car.notcommunicating]
    wait_times_AVs += [car.time_entered_intersection-car.time_entered_control for car in traffic_simulator.total_vehicle_list if car.time_entered_intersection!=-1 and not car.notcommunicating]
    
    data_points = [(car.time_entered_intersection-car.time_entered_control, car.notcommunicating, car.turn_direction, car.initial_lane) for car in traffic_simulator.total_vehicle_list if car.time_entered_intersection!=-1]
    # return wait_times, all_swaps, wait_times_notcommunicatings, wait_times_AVs, exit_times, data_points                                        
    return data_points

def run_sim_experiments(traffic_simulator, all_cars, number_cars_to_sim=999999, number_iterations_before_plotting=99999999):
    wait_times_notcomm = []
    thetas_SVO_notcomm = []
    wait_times_AVs = []
    all_swaps = []
    wait_times = []
    exit_times = []
            
    next_agent_id = 0
    (arrival_time, agent_id, notcommunicating_type, svo_theta, turn_direction, start_lane) = all_cars[next_agent_id]
    for i in range(1,1000):
        traffic_simulator.increment_time()

        
        if (i % traffic_simulator.coordinator.k_number_iterations_between_cleaning_time_matrix) == 0:
            traffic_simulator.coordinator.remove_expired_intervals_from_matrix(traffic_simulator.time)
            
        if traffic_simulator.time > arrival_time:
            new_car = AV.AutonomousVehicle(agent_id)
            new_car.set_traffic_coordinator(traffic_simulator.coordinator)
            traffic_simulator.update_robot_map_specs(new_car)
            new_car.initialize_beginning_lane(start_lane)
            
            if turn_direction == 'Left' or turn_direction == "Left" or turn_direction == "LEFT" or turn_direction == 'LEFT':
                new_car.turn_direction = AV.TurnDirection.LEFT
            elif turn_direction == 'Right' or turn_direction == "Right" or turn_direction == "RIGHT" or turn_direction == 'RIGHT':
                new_car.turn_direction = AV.TurnDirection.RIGHT
            else:
                new_car.turn_direction = AV.TurnDirection.STRAIGHT
                
            
            if notcommunicating_type == "Not Communicating" or notcommunicating_type == "not communicating" or notcommunicating_type == "Not communicating":
                new_car.notcommunicating = True
            else:
                new_car.notcommunicating = False
            
            
            new_car.svo_theta = svo_theta
            
            new_pose = new_car.get_next_step_pose_lane(new_car.current_lane,traffic_simulator.deltaT*new_car._maxSpeed)
            no_collision = new_car.collision_free(new_pose,traffic_simulator.occupancy_map,new_car.pose)
            if no_collision:
                traffic_simulator.add_vehicle(new_car)
                next_agent_id += 1
                if next_agent_id < len(all_cars) and next_agent_id < number_cars_to_sim:
                    (arrival_time, agent_id, notcomm_type, svo_theta, turn_direction, start_lane) = all_cars[next_agent_id]
                else:
                    arrival_time = 99999999999999999999999
            else:
                pass
        if traffic_simulator.time>10.0 and len(traffic_simulator.active_vehicles)==0:
            break                
    exit_times += [car.time_exited_intersection for car in traffic_simulator.total_vehicle_list if car.time_entered_intersection!=-1]
    wait_times += [car.time_entered_intersection-car.time_entered_control for car in traffic_simulator.total_vehicle_list if car.time_entered_intersection!=-1]
    all_swaps += traffic_simulator.coordinator.list_of_swapped_reservations
    wait_times_notcomm += [car.time_entered_intersection-car.time_entered_control for car in traffic_simulator.total_vehicle_list if car.time_entered_intersection!=-1 and car.notcommunicating]
    thetas_SVO_notcomm += [car.svo_theta for car in traffic_simulator.total_vehicle_list if car.time_entered_intersection!=-1 and car.notcommunicating]
    wait_times_AVs += [car.time_entered_intersection-car.time_entered_control for car in traffic_simulator.total_vehicle_list if car.time_entered_intersection!=-1 and not car.notcommunicating]
    
    return wait_times, all_swaps, wait_times_notcomm, wait_times_AVs, exit_times
                                        

