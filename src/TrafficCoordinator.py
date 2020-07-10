### Author: Noam Buckman
### Description:  Traffic Coordinator for handling autonomous vehicle reservations
###               as they entire an intersection.

import numpy as np
import queue as queue
import copy as copy
from shapely.geometry import Point, box
from shapely.affinity import rotate


import kinematics as kn
import AutonomousVehicle as AV


class MatrixTrafficCoordinator:
    ''' Generic traffic coordinator that uses a reservation matrix of the intersection
    and collects requests for reservations. The main role of the coordinator is to ensure 
    that there are no collisions through the intersection and provide reservation windows
    to incoming vehicles.  Based on Dresner, Kurt, and Peter Stone AAMAS 2004. 
    '''
    def __init__(self):
    
        self.reservation_counter = 0
        self.agents_with_reservations = {}

        self.queue_cars = queue.Queue()
        self.car_ids_in_queue = set()
        self.next_car_id = -1
        self.current_lane = "11"
        self.priority_tickets_given_out = {"1": -1, "-1": -1, "2": -1, "-2": -1}
        self.next_customer_get_reservation = {"1": 0, "-1": 0, "2": 0, "-2": 0}        
        self.priority_dict = {}
        self.next_available = -1

        self.time_matrix = {} #starts in top left corner of intersection
        self.time = 0
        self.dx = 0.1
        self.time_buffer_interval = 0.02
        # self.time_start_matrix = np.ones((int(2/self.dx)+1,int(2/self.dx)+1))*99999999

        self.m = int(2/self.dx)+1
        self.n = int(2/self.dx)+1
        self.number_tiles_buffer = 2
        self.time_available_matrix = {i: {j: 0 for j in range(self.m)} for i in range(self.n)}

        self.time_matrix_intervals = {i: {j: [] for j in range(self.m)} for i in range(self.n)}

        self.bid_queue = []
        self.discretized_grid_points = [Point(x,y) for x in np.arange(-1,1+self.dx,self.dx) for y in np.arange(-1,1+self.dx,self.dx)]
        self.k_number_iterations_between_cleaning_time_matrix = 50

    def add_interval(self, interval, i, j, agent_id, time_matrix_intervals, consolidate_intervals=False):
        ''' Insert an interval (start_time, end_time) into time_matrix_intervals.'''  

        if type(interval) != tuple:
            raise Exception("Not a tuple")
        elif interval[1] < interval[0]:
            raise Exception("Error:  Interval End Time < Start Time ")
        elif consolidate_intervals:
            interval_with_agent = (interval[0], interval[1], agent_id)
            current_intervals_ij = time_matrix_intervals[i][j]
            if len(time_matrix_intervals[i][j]) == 0:
                time_matrix_intervals[i][j] = [interval_with_agent]
                return time_matrix_intervals
            else:
                for i_interval in range(len(current_intervals_ij)): #THIS DOESN"T WORK PERFECTLY
                    (other_start, other_end, other_agent_id) = current_intervals_ij[i_interval]
                    if interval[0] < other_start: # 
                        if (agent_id == other_agent_id) and interval[1] > other_start:
                            current_intervals_ij  = current_intervals_ij[:i_interval] + [(interval[0], other_end, agent_id)] + current_intervals_ij[i_interval+1:]
                        else:
                            current_intervals_ij = current_intervals_ij[:i_interval] + [interval_with_agent] + current_intervals_ij[i_interval:]
                        time_matrix_intervals[i][j] = current_intervals_ij
                        return time_matrix_intervals
                current_intervals_ij += [interval_with_agent]
                time_matrix_intervals[i][j] = current_intervals_ij
                return time_matrix_intervals                
        else: 
            interval_with_agent = (interval[0], interval[1], agent_id)
            time_matrix_intervals[i][j].append(interval_with_agent)
            return time_matrix_intervals

    def conflicts_in_interval_bool(self, interval, i, j, time_matrix_intervals):
        ''' Checks if an interval of time (start_time,end_time) conflicts at location (i,j) in time_matrix_intervals dictionary '''
        new_start, new_end, new_agent_id = interval
        epsilon = 0.001
        for (start, end, agent_id) in time_matrix_intervals[i][j]:
            if new_agent_id != agent_id:
                if (start-epsilon < new_start < end+epsilon) or (start-epsilon < new_end < end+epsilon):
                    return True
        return False

    def add_points_to_reservation_matrix(self, intersection_points_to_reserve, agent_id, reservation_interval_matrix, reservation_available_matrix):
        ''' Add list of points with start and end times to the interval matrix'''
        #TODO:  Do some pre-processing to consolidate the number of points to add since some intervals will overlap.  Iterate over each window
        for t_start, t_end, i, j in intersection_points_to_reserve:
            if (0 <= i <= self.m-1) and (0 <= j <= self.n-1):      
                reservation_interval_matrix = self.add_interval((t_start,t_end),i,j,agent_id,reservation_interval_matrix)                                        
                reservation_available_matrix[i][j] = max(t_end + self.time_buffer_interval,reservation_available_matrix[i][j]) #I'm adding a fixed buffer size
        return reservation_interval_matrix, reservation_available_matrix

    def remove_expired_intervals_from_matrix(self, current_time=None):
        ''' Helper function to frequently cleanup the matrix interval dictionary with expired intervals'''
        if current_time == None:
            current_time = self.time 
        epsilon = 0.001
        for i in self.time_matrix_intervals.keys():
            for j in self.time_matrix_intervals[i].keys():
                cleaned_list_of_intervals = []
                for (start, end, agent_id) in self.time_matrix_intervals[i][j]:
                    if current_time < (end + epsilon):
                        cleaned_list_of_intervals += [(start, end, agent_id)]
                        # self.time_matrix_intervals[i][j].remove((start,end))
                self.time_matrix_intervals[i][j] = cleaned_list_of_intervals
    
    def forward_simulate(self, start_time, intersection_lane, car_object):
        start_pose = car_object.pose # TODO CURRENT POSE NEEDS TO NOT BE HARD CODED
        if intersection_lane == "22" or intersection_lane == "2-1" or intersection_lane == "21" or intersection_lane == "2?":
            start_pose = kn.Pose(0.5,-1,np.pi/2.0)
        elif intersection_lane == "11" or intersection_lane == "1-2" or intersection_lane == "12" or intersection_lane == "1?":
            start_pose = kn.Pose(-1, -0.5, 0)
        elif intersection_lane == "-1-1" or intersection_lane == "-12" or intersection_lane == "-1-2" or intersection_lane == "-1?":
            start_pose = kn.Pose(1, 0.5, np.pi)
        elif intersection_lane == "-2-2" or intersection_lane == "-2-1" or intersection_lane == "-21" or intersection_lane == "-2?":
            start_pose = kn.Pose(-0.5,1, -np.pi/2.0)    
        else:
            print("Weird current lane,",intersection_lane)                                

        list_of_positions_time = []
        all_intersection_lanes = []
        if intersection_lane[-1] == "?": #notcommunicating DRIVER
            # print("notcommunicating DRIVER!")
            all_intersection_lanes = [
                car_object.turn_direction_to_intersection_lane(car_object.current_lane,AV.TurnDirection.LEFT),
                car_object.turn_direction_to_intersection_lane(car_object.current_lane,AV.TurnDirection.STRAIGHT),
                car_object.turn_direction_to_intersection_lane(car_object.current_lane,AV.TurnDirection.RIGHT),
            ]
        else:
            all_intersection_lanes = [intersection_lane]

        for intersection_lane in all_intersection_lanes:
            current_time = start_time + 0.0
            current_pose = copy.deepcopy(start_pose)
            while not car_object.check_in_intersection_exit(current_pose):
                list_of_positions_time += [(current_pose.x,current_pose.y,current_pose.phi,current_time)]
                dt_simulation =  car_object.time_increment/2.0
                distance = car_object._maxSpeed * dt_simulation #Using max speed
                current_pose = car_object.get_next_step_pose_lane(intersection_lane, distance, current_pose)
                current_time += dt_simulation
                # print('%0.03f'%current_time,'%0.03f'%current_pose.x,",",'%0.03f'%current_pose.y)
                if len(list_of_positions_time)>200*self.n: raise Exception("Vehicle",car_object.agent_id,"Never reaches intersection exit") #TODO: Why 20?
        return list_of_positions_time

    def matrix_index_from_xy(self, x, y):
        x0 = -1 - self.dx/2 #this sould be +self.lane_width, slash come from the map
        y0 = 1 + self.dx/2 #this should be -self.lane_width
        i = int((y0-y)/self.dx)
        j = int((x-x0)/self.dx)
        if i<0 or j<0:
            print("Weird,ij",i,",",j,"xy",x,",",y)
        return (i,j)

    def request_intersection_lane(self, start_time, end_time,intersection_lane, agent_id, car_object, bid=-1):
        self.reservation_counter += 1
        # if agent_id not in self.priority_dict.keys():
        #     next_ticket = self.priority_tickets_given_out[intersection_lane]+1
        #     self.priority_tickets_given_out[intersection_lane] = next_ticket
        #     self.priority_dict[agent_id] = next_ticket
        #     # print("ID",agent_id,"T",next_ticket)
        #     # print("NextCustomer",self.next_customer_get_reservation[intersection_lane])
        if agent_id not in self.car_ids_in_queue:
            self.car_ids_in_queue.add(agent_id)
            self.queue_cars.put(agent_id)
            if self.next_car_id == -1:
                self.next_car_id = self.queue_cars.get_nowait()

        ## FCFS

        if self.next_car_id == agent_id or self.next_car_id == -1: #Preserving priority
            list_of_positions_time = self.forward_simulate(start_time,intersection_lane,car_object)
            conflicting_trajectory = False
            for (x, y, phi, t) in list_of_positions_time:
                (i,j) = self.matrix_index_from_xy(x,y)
                for i_tiles_buffer in range(-self.number_tiles_buffer,self.number_tiles_buffer):
                    for j_tiles_buffer in range(-self.number_tiles_buffer,self.number_tiles_buffer):
                        i_buff = i + i_tiles_buffer
                        j_buff = j + j_tiles_buffer
                        if (0 <= i_buff <= self.m-1) and (0 <= j_buff <= self.n-1):
                            if self.conflicts_in_interval_bool((t-self.time_buffer_interval,t+self.time_buffer_interval,agent_id),i_buff,j_buff, self.time_matrix_intervals):
                                # print("Conflicts!",(t-self.time_buffer_interval,t+self.time_buffer_interval),i_buff,j_buff)
                                return (-1,-1)
            if conflicting_trajectory: #Redudent
                return (-1,-1)
            else:
                for (x,y,phi,t) in list_of_positions_time:
                    (i,j) = self.matrix_index_from_xy(x,y)
                    for i_tiles_buffer in range(-self.number_tiles_buffer,self.number_tiles_buffer):
                        for j_tiles_buffer in range(-self.number_tiles_buffer,self.number_tiles_buffer):
                            i_buff = i + i_tiles_buffer
                            j_buff = j + j_tiles_buffer
                    # for (i_buff,j_buff) in [(i,j),(i+1,j),(i-1,j),(i,j+1),(i,j-1),(i+1,j+1),(i-1,j-1),(i+1,j-1),(i-1,j+1),(i+2,j),(i-2,j),(i,j+2),(i,j-2),(i+2,j+2),(i-2,j-2),(i+2,j-2),(i-2,j+2)]:
                            if (0 <= i_buff <= self.m-1) and (0 <= j_buff <= self.n-1):      
                                self.time_matrix_intervals = self.add_interval((t-self.time_buffer_interval,t+self.time_buffer_interval),i_buff,j_buff,agent_id,self.time_matrix_intervals)                                        
                                self.time_available_matrix[i_buff][j_buff] = t + self.time_buffer_interval #I'm adding a fixed buffer size
                                # self.time_start_matrix[i_buff,j_buff] = t-0.01
                if self.queue_cars.empty():
                    self.next_car_id = -1
                else:
                    self.next_car_id = self.queue_cars.get_nowait()                
                return (start_time, t+self.time_buffer_interval*10) #WHAT DOES THIS DO?
        else:
            return (-1,-1)

        if self.next_customer_get_reservation[intersection_lane] == self.priority_dict[agent_id]:
            if self.current_lane == intersection_lane and (start_time-self.next_available < 0.05):
                self.next_available = end_time
                print("Received Reservation Request #",self.reservation_counter, start_time,end_time)
                self.next_customer_get_reservation[intersection_lane]+=1      
                return (start_time,end_time)
            elif start_time > self.next_available:
                self.next_available = end_time
                print("Received Reservation Request #",self.reservation_counter, start_time,end_time, "Same Lane")   
                self.next_customer_get_reservation[intersection_lane]+=1       
                self.current_lane = intersection_lane  #Update the current lane being used
                return (start_time,end_time)
            else: 
                print('Rejected Reservation Request # %.2f %.2f %.2f Next Avail:  %.2f' % (self.reservation_counter, start_time,end_time, self.next_available))        
                return (-1,-1)
        else:
            print('Rejected Reservation Request # %.2f %.2f %.2f Waiting for Preceding Agents to Get Reservation' % (self.reservation_counter, start_time,end_time))        
            return (-1,-1)

    def return_reservation(self):
        return (-1,-1)

    def get_reservation_matrix(self):
        return self.time_matrix_intervals

    def increment(self, dt):
        self.dt = dt
        return None

    def get_intersection_entrance_free_time(self, incoming_lane, time_available_matrix):
        lane_width = 1.0 #TODO: Do not hardcode the lane_width
        if incoming_lane == "1":
            x_entrance = -lane_width
            y_entrance = -lane_width/2.0
        elif incoming_lane == "-1":
            x_entrance = lane_width
            y_entrance = lane_width/2.0     
        elif incoming_lane == "2":
            x_entrance = lane_width/2.0
            y_entrance = -lane_width     
        elif incoming_lane == "-2":
            x_entrance = -lane_width/2.0
            y_entrance = lane_width                                    
        i_entrance, j_entrance = self.matrix_index_from_xy(x_entrance, y_entrance) 
        return time_available_matrix[i_entrance][j_entrance]  

    def get_points_covered_by_car(self,x,y,phi,car_width,car_length, notcommunicating=False):
        ''' Returns a list of points from the intersection grid that intersect with the car's body'''
        # TODO:  THIS NEEDS TO BE FIXED
        k_number_buffer_tiles = 1.5
        # if notcommunicating: k_number_buffer_tiles = 4 

        minx = x-car_length-k_number_buffer_tiles*self.dx
        maxx = x+k_number_buffer_tiles*self.dx
        miny = y-car_width/2.0-k_number_buffer_tiles*self.dx
        maxy = y+car_width/2.0+k_number_buffer_tiles*self.dx


        buffered_car = box(minx,miny,maxx,maxy)
        rotate_about_pt = Point(x,y)
        rotated_buffered_car = rotate(buffered_car,phi,origin=rotate_about_pt,use_radians=True)
        conflicted_points = []
        for point in self.discretized_grid_points:
            if rotated_buffered_car.contains(point):
                conflicted_points += [point]      
        return conflicted_points            
######################################################################################   
#      
class BatchTrafficCoordinator(MatrixTrafficCoordinator):
    ''' Our SVO coordinator which considers the SVO's of incoming agents
    and allows for swapping positions based on SVO bids.  
    Coordinator batches requests so that it can compare multiple bids at a time.
    '''
    def __init__(self):
        MatrixTrafficCoordinator.__init__(self)
        self.reservation_counter = 0
        
        self.lane_queues = {"1": [], "-1": [], "2": [], "-2": []}
        self.agents_with_reservations = {}
        self.agents_last_attempted_reservation = {}
        self.car_ids_with_reservation = set()

        self.dx = 0.1
        self.time_buffer_interval = 0.02
        # self.time_start_matrix = np.ones((int(2/self.dx)+1,int(2/self.dx)+1))*99999999

        self.number_tiles_buffer = 2
        # self.bid_queue: List[Tuple[float, float, float, AV.TurnDirection, int, AV.AutonomousVehicle]] = []
        self.bid_queue = []
        self.sorted_queue = []
        self.batch_counter = 0
        self.time = 0
        self.last_batch_assignment = 0.0        

        self.max_bid_per_agent = {}
        self.returned_reservations = []
        self.list_of_swapped_reservations = []
        # Tax Information
        self.bid_tax = 0.0 #seconds

        self.svo_squared = False
        self.strict_priority = False
        self.swaps_allowed = True

        # Parameters for the batch assignment
        self.k_replan_increment_sec = 0.5
        self.k_max_retries = 60  # 
        self.k_max_cars_planning_lane_FCFS = 1  #TODO:  This is an important variable        
        self.k_batch_intervals_time = 2.0 # seconds between batch planning
        self.k_min_number_cars_without_reservations = 8
        self.k_max_number_cars_assigned = 8

        self.k_epsilon_bid = 0.001
        self.k_max_start_time_horizon = 45.0        

        self.prob_swap_back_comm = 1.001
        self.prob_swap_back_notcomm = 1.001  #
        self.nocomm_no_swap = False   # If True, notcommunicatings are not allowed to swapped
        self.nocollab_no_swap = False

        self.bid_queue_sort_type = "FCFSlane"


    def increment(self, dt):
        ''' Simulator calls this method at each timestep'''
        self.dt = dt
        self.number_cars_without_reservations = len(self.bid_queue)
        #TODO:  Add something to ensure single agents can be allocated
        batch_assign_flag = False

        for bid in self.bid_queue:
            car = bid[-1]
            if car.intersection_window[0]==-1 and (car.pose.x)**2 + car.pose.y**2 <= 2.0*car.lane_width:
                batch_assign_flag = True
        
        if ((self.time - self.last_batch_assignment) > self.k_batch_intervals_time) and self.number_cars_without_reservations >= self.k_min_number_cars_without_reservations:
            batch_assign_flag = True
        

        if batch_assign_flag:
            self.bid_queue = self.one_bid_per_agent_queue(self.bid_queue)
            # self.new_batch_intersection_FCFS()    
            if len(self.bid_queue) > 0:
                reservations = self.one_swap_sort(self.bid_queue,verbose=False)
            else:
                reservations = []
            self.returned_reservations  += [reservations]
            self.last_batch_assignment = self.time    
            self.bid_queue = []
            self.max_bid_per_agent = {}


    def one_swap_sort(self, bid_queue,verbose=False):
        ''' Looks for single swap improvements for switching the queue.  Iterate through a queue.
            Each agent (first request) can consider swapping with the next agent.  This will only occur if the utility
            for both agents improve (its a consentual swap)
            Used in IROS2019
        '''
        self.lane_queues = self.update_lane_queues(self.lane_queues)  # Make sure lane queue is up-to-date

        sorted_queue = self.sort_queue(bid_queue,self.bid_queue_sort_type)
        final_reservation = []        
        print("Received bids from %d agents"%len(sorted_queue),":",[r[-1].agent_id for r in sorted_queue])
        if len(sorted_queue) == 1:
            first_request = sorted_queue[0]
            requested_start_time1, fake_interval_matrix, fake_available_matrix, fake_lane_queues = self.attempt_one_reservation(first_request)
            if requested_start_time1 > 0:
                self.time_matrix_intervals, self.time_available_matrix, self.lane_queues = fake_interval_matrix, fake_available_matrix, fake_lane_queues
                final_reservation += [(first_request[-1], requested_start_time1)] # Reserve the first request and increment the request   
                for car, reservation_start_time in final_reservation:
                    car.intersection_window = (reservation_start_time, 9999999)   
                print("Reserving intersection for vehicles...",[car_reservation[0].agent_id for car_reservation in final_reservation])
            return final_reservation 
        else:
            first_request = sorted_queue[0]
            for queue_index in range(min(self.k_max_number_cars_assigned,len(sorted_queue)-1)):
                second_request = sorted_queue[queue_index+1]
                #
                u1, u2, requested_start_time1, requested_start_time2, fake_interval_matrix, fake_available_matrix, fake_lane_queues = self.attempt_two_reservations(first_request, second_request)
                if verbose: print("Original Attempted:  Agent %d then %d.  t_s1t= %.02f,  t_s1= %.02f u1= %0.02f u2=%0.02f"%(first_request[-2],second_request[-2],  requested_start_time1, requested_start_time2, u1, u2))
                if requested_start_time1 < 0 or requested_start_time2 < 0:
                    print("Did not return solution for %dth car in queue"%(queue_index+1))
                    break
                
                if first_request[-1].current_lane == second_request[-1].current_lane or (self.nocomm_no_swap and (first_request[-1].notcommunicating or second_request[-1].notcommunicating)) or (self.nocollab_no_swap and (first_request[-1].notcollaborating or second_request[-1].notcollaborating)) :
                    '''Second request in same lane as first request.  Swap is not possible'''
                    self.time_matrix_intervals, self.time_available_matrix, self.lane_queues = fake_interval_matrix, fake_available_matrix, fake_lane_queues
                    final_reservation += [(first_request[-1], requested_start_time1)] # Reserve the first request and increment the request           
                    first_request = second_request               
                    continue
                    
                u1_swap, u2_swap, requested_start_time1_swap, requested_start_time2_swap, fake_interval_matrix_swap, fake_available_matrix_swap, fake_lane_queues_swap = self.attempt_two_reservations(second_request, first_request)
                if requested_start_time1_swap < 0 or requested_start_time2_swap < 0:
                    print("Did not return solution for %dth car in queue in swap"%(queue_index+1))
                    break

                if verbose:
                    print("\n First Agent %d. t_start_original = %.02f t_start_swap = %.02f.   Original U= %.02f Swap U= %.02f"%(first_request[-2], requested_start_time1, requested_start_time2_swap, u1, u2_swap))
                    print("Second Agent %d. t_start_original = %.02f t_start_swap = %.02f.   Original U= %.02f Swap U= %.02f"%(second_request[-2], requested_start_time2, requested_start_time1_swap, u2, u1_swap))
                
                u_epsilon = 0.001

                
                if second_request[-1].notcommunicating:
                    p_swap_back = first_request[-1].prob_swap_back_notcomm # Non-communicating swap back 100%
                else:
                    p_swap_back = first_request[-1].prob_swap_back_comm # communicating

                if (u1_swap - u2 > u_epsilon) and (u2_swap - u1 > u_epsilon) and self.swaps_allowed and np.random.uniform() < p_swap_back:
                    print("SWAP! Agent %d H%r before Agent %d H%r"%(second_request[-1].agent_id, second_request[-1].notcommunicating, first_request[-1].agent_id, first_request[-1].notcommunicating))
                    self.time_matrix_intervals, self.time_available_matrix, self.lane_queues = fake_interval_matrix_swap, fake_available_matrix_swap, fake_lane_queues_swap
                    final_reservation += [(second_request[-1], requested_start_time1_swap)] # Reserve the second request due to swap   
                    n_notcommunicatings = second_request[-1].notcommunicating + first_request[-1].notcommunicating              
                    self.list_of_swapped_reservations += [(u1, u2, u1_swap, u2_swap, first_request[-1].agent_id, second_request[-1].agent_id, n_notcommunicatings)]                
                else:
                    self.time_matrix_intervals, self.time_available_matrix, self.lane_queues = fake_interval_matrix, fake_available_matrix, fake_lane_queues
                    final_reservation += [(first_request[-1], requested_start_time1)] # Reserve the first request and increment the request           
                    first_request = second_request                  
            print("Reserving intersection for vehicles...",[car_reservation[0].agent_id for car_reservation in final_reservation])
            for car, reservation_start_time in final_reservation:
                car.intersection_window = (reservation_start_time, 9999999)
            return final_reservation            
    
    def new_batch_intersection_FCFS(self):
        """Takes the queue of reservation requests and returns a list of return
        reservations.  
        
        1. Ensures that lane queues are up-to-date with cars and existing reservations
        2. For each request, forward simulate car trajectory and check for conflicts in reservation-matrix
        3. If no conflict, add reservation.  Else, repeat 10 times with offset.
        4. Set reservation for each agent from final list of reservations 
        """

        self.lane_queues = self.update_lane_queues(self.lane_queues)  # Make sure lane queue is up-to-date (subroutine?)
        
        # #3/4 edition
        # new_queue = []
        # for bid, requested_start_time, end_time, intersection_lane, agentID, current_car in self.bid_queue:
        #     bid = self.time - current_car.time_entered_control 
        #     new_queue += [(bid, requested_start_time, end_time, intersection_lane, agentID, current_car)]            
        # self.bid_queue = new_queue
        # self.sorted_queue = self.sort_queue(self.bid_queue,"FCFS")

        # ### 3/4 Edition

        final_reservations = []
        
        self.car_ids_with_reservation = set()  #TODO: Is this the right thing to have?
        
        # previous_priority_car_time_entered_control = 0.0 #TODO THIS NEEDS TO GET INFO
        for bid, requested_start_time, end_time, intersection_lane, agent_id, current_car in self.sorted_queue:
            if agent_id in self.car_ids_with_reservation: continue  # Agent already received reservation, maybe we don't need this
            if len(self.lane_queues[current_car.current_lane]) > self.k_max_cars_planning_lane_FCFS: break
            
            time_car_can_enter_intersection = self.get_intersection_available_for_car(current_car.current_lane, current_car, self.lane_queues)
            
            reservation_window_start_time, self.time_matrix_intervals, self.time_available_matrix = self.attempt_reservation(time_car_can_enter_intersection, intersection_lane, current_car, agent_id, self.time_matrix_intervals, self.time_available_matrix)
            
            if reservation_window_start_time < 0:  
                break
            else:
                final_reservations += [(reservation_window_start_time,100000)]
                time_intersection_entrance_free = self.get_intersection_entrance_free_time(current_car.current_lane, self.time_available_matrix)
                self.lane_queues[current_car.current_lane] += [(current_car, time_intersection_entrance_free)] #TODO:  This needs to be when I'm out of the control region
                current_car.intersection_window = (reservation_window_start_time, 100000)        

                self.car_ids_with_reservation.add(agent_id)
        self.bid_queue = []   
        return final_reservations

    def get_intersection_available_for_car(self, incoming_lane, current_car, lane_queues, strict_priority=False):
        ''' Calculates the time at which point car will be at the intersection entrance'''
            
        if len(lane_queues[incoming_lane]) > 0:
            previous_car, previous_car_clears_entrance = lane_queues[incoming_lane][-1]
            distance_to_previous_car = current_car.pose.dist(previous_car.pose)
            car_intersection_start_time = previous_car_clears_entrance + distance_to_previous_car/current_car._maxSpeed
        else: 
            '''No preceeding cars (i.e. start time = time to interesection)'''
            car_intersection_start_time = current_car.time + current_car.get_time_to_intersection()
        if strict_priority:
            all_other_lane_availabilities = [last_time[1] for lane in lane_queues.keys() for last_time in lane_queues[lane] if lane != incoming_lane]  
            start_time_to_preserve_order = max(all_other_lane_availabilities+[0])-current_car.car_length/current_car._maxSpeed
            car_intersection_start_time = max([car_intersection_start_time, start_time_to_preserve_order])          
        if current_car.agent_id in self.agents_last_attempted_reservation.keys():
            car_intersection_start_time = max(car_intersection_start_time, self.agents_last_attempted_reservation[current_car.agent_id])
        return car_intersection_start_time

    def attempt_reservation(self, requested_start_time, intersection_lane, current_car, agent_id, time_matrix_intervals, time_available_matrix, last_entry_time=-1):
        ''' Attempts to reserve the interesection for entering at car_intersection_start_time.  Returns True/False depending on whether it is possible.'''
        new_time_matrix_intervals = copy.deepcopy(time_matrix_intervals)
        new_time_available_matrix = copy.deepcopy(time_available_matrix) 
        current_reservation_attempt = 0
        reservation_start_time = requested_start_time
        # if last_entry_time > 0:
        #     reservation_start_time = last_entry_time # You need to start after the last entry
        while current_reservation_attempt < self.k_max_retries:
            intersection_points_to_reserve = self.calculate_reservation(reservation_start_time, intersection_lane, current_car, agent_id, new_time_matrix_intervals)

            if len(intersection_points_to_reserve) > 0:
                new_time_matrix_intervals, new_time_available_matrix = self.add_points_to_reservation_matrix(intersection_points_to_reserve, agent_id, new_time_matrix_intervals, new_time_available_matrix) #I'm adding a fixed buffer size
                # time_matrix_intervals, time_available_matrix = self.set_reservation(intersection_points_to_reserve, agent_id, current_car, reservation_start_time, time_matrix_intervals, time_available_matrix)
                return reservation_start_time, new_time_matrix_intervals, new_time_available_matrix
            else:
                current_reservation_attempt += 1
            reservation_start_time +=  self.k_replan_increment_sec  #TODO: THIS SHOULD PROBABLY HAPPEN ELSEWHERE?
        self.agents_last_attempted_reservation[agent_id] = reservation_start_time
        print("RetryMaxOut: Car %d, Tried up to starting @ t %.03f"%(agent_id,reservation_start_time))
        return -1, new_time_matrix_intervals, new_time_available_matrix

    def calculate_reservation(self, car_intersection_start_time, intersection_lane, current_car, agent_id, time_matrix_intervals):
        ''' Calculate and returns the positions/intervals required by the vehicle in the intersection.  If there is a conflict, no points are returned
        An additional time_buffer_interval is added when reserving an interval
        '''
        list_of_positions_time = self.forward_simulate(car_intersection_start_time, intersection_lane, current_car)
        intersection_points_to_reserve = []
        for x, y, phi, t in list_of_positions_time:
            points_covered_by_car = self.get_points_covered_by_car(x, y, phi, current_car.car_width, current_car.car_length, current_car.notcommunicating)
            for point in points_covered_by_car:
                i,j = self.matrix_index_from_xy(point.x, point.y)
                if self.conflicts_in_interval_bool((t-self.time_buffer_interval,t+self.time_buffer_interval,agent_id), i, j, time_matrix_intervals):
                    return []
                else:
                    intersection_points_to_reserve += [(t - self.time_buffer_interval, t + self.time_buffer_interval, i, j)]
        return intersection_points_to_reserve
    
    def request_intersection_lane(self,start_time,end_time,intersection_lane,agent_id,car_object,bid=-1):
        ''' Public:  Cars request a reservation by specifying the intersection lane'''
        if bid != -1:
            self.insert_bid_to_queue(start_time,end_time,intersection_lane,agent_id,car_object,bid)
        return (-1,-1)

    def insert_bid_to_queue(self,start_time,end_time,intersection_lane,agent_id,car_object,bid):
        ''' Insert bid into the queue but first only keep one bid per agent'''
        if car_object.agent_id not in self.max_bid_per_agent.keys():
            if car_object.agent_id >= 0:
                self.max_bid_per_agent[car_object.agent_id] = (bid,start_time,end_time,intersection_lane,agent_id,car_object)
        elif bid > self.max_bid_per_agent[car_object.agent_id][0]:
            self.max_bid_per_agent[car_object.agent_id] = (bid,start_time,end_time,intersection_lane,agent_id,car_object)
        self.bid_queue = [self.max_bid_per_agent[agent_id] for agent_id in self.max_bid_per_agent.keys()]

    def sort_queue(self, bid_queue, sorting_string):
        ''' Sorts the queue by predefined sorting.  Assumes queue only has one bid per agent
            sorting_string == "FCFS":  highest bid is first in queue
            sorting_string == "FCFSagent_id":  highest bid is first, ties broken by agent_id
        '''
        sorted_queue = []
        if sorting_string == "FCFS":
           sorted_queue = sorted(bid_queue,key=lambda b: -b[0])
        elif sorting_string == "FCFSagent_id":
            sorted_queue = self.sort_and_break_ties_id(bid_queue)
        elif sorting_string == "FCFSlane":
            sorted_queue = self.sort_and_break_ties_lane(bid_queue)
        else:
            raise Exception("Unknown Sorting String")
        return sorted_queue

    def sort_and_break_ties_id(self, bid_queue, epsilon_bid = -1):
        ''' Sort of list of bids, find the ties, and sort by agent_id'''
        if epsilon_bid == -1:
            epsilon_bid = self.k_epsilon_bid
        
        sorted_queue_by_bids = sorted(bid_queue, key=lambda b: -b[0])
        final_sorted_queue_id_tiebreaker = []

        previous_bid_value = -999999
        previous_tied_bids = []
        for current_bid in sorted_queue_by_bids:
            if abs(current_bid[0]-previous_bid_value) <= epsilon_bid:
                previous_tied_bids += [current_bid]
            else:
                ''' Sort the previously tied bids and use the agent id as the sorting key''' 
                if len(previous_tied_bids) > 0:
                    sorted_bids_by_agent_id = sorted(previous_tied_bids, key=lambda b: b[-1].agent_id)
                    final_sorted_queue_id_tiebreaker += sorted_bids_by_agent_id
                previous_tied_bids = [current_bid]
                previous_bid_value = current_bid[0]
        if len(previous_tied_bids) > 0:
            sorted_bids_by_agent_id = sorted(previous_tied_bids, key=lambda b: b[-1].agent_id)
            final_sorted_queue_id_tiebreaker += sorted_bids_by_agent_id        
        return final_sorted_queue_id_tiebreaker

    def sort_and_break_ties_lane(self, bid_queue, epsilon_bid = -1):
        ''' Sort of list of bids, find the ties, and sort by agent_id'''
        if epsilon_bid == -1:
            epsilon_bid = self.k_epsilon_bid
        
        sorted_queue_by_bids = sorted(bid_queue, key=lambda b: -b[0])
        final_sorted_queue_id_tiebreaker = []

        previous_bid_value = -999999
        previous_tied_bids = []
        for current_bid in sorted_queue_by_bids:
            if abs(current_bid[0]-previous_bid_value) <= epsilon_bid:
                previous_tied_bids += [current_bid]
            else:
                ''' Sort the previously tied bids and use the lane number as the sorting key''' 
                if len(previous_tied_bids) > 0:
                    sorted_bids_by_agent_id = sorted(previous_tied_bids, key=lambda b: int(b[-1].initial_lane))
                    final_sorted_queue_id_tiebreaker += sorted_bids_by_agent_id
                previous_tied_bids = [current_bid]
                previous_bid_value = current_bid[0]
        if len(previous_tied_bids) > 0:
            sorted_bids_by_agent_id = sorted(previous_tied_bids, key=lambda b: int(b[-1].initial_lane))
            final_sorted_queue_id_tiebreaker += sorted_bids_by_agent_id        
        return final_sorted_queue_id_tiebreaker
    
    def attempt_one_reservation(self, request):
        ''' Try to assign a single vehicle to the intersection.  First checks time intersection would be available for vehicle. 
            Then if it is available, reserves the matrix and returns the updated reservation matrices 
        '''
        fake_lane_queues = self.copy_queues(self.lane_queues)
        (bid, requested_start_time, end_time, intersection_lane, agent_id, current_car) = request
        time_car_can_enter_intersection = self.get_intersection_available_for_car(current_car.current_lane, 
                                                    current_car, fake_lane_queues, self.strict_priority)
        start_time, t_interval_matrix_after_reservation, t_available_matrix_after_reservation = self.attempt_reservation(time_car_can_enter_intersection, intersection_lane, current_car, agent_id, self.time_matrix_intervals, self.time_available_matrix)
        if start_time < 0 or (start_time > self.time + self.k_max_start_time_horizon): #NOTE THIS IS RISKY BUSINESS
            return -1, self.time_matrix_intervals, self.time_available_matrix, self.lane_queues
        else:
            time_intersection_entrance_free = self.get_intersection_entrance_free_time(current_car.current_lane, t_available_matrix_after_reservation)
            fake_lane_queues[current_car.current_lane] += [(current_car, time_intersection_entrance_free)] #TODO:  This needs to be when I'm out of the control region
        return start_time, t_interval_matrix_after_reservation, t_available_matrix_after_reservation, fake_lane_queues

    def attempt_two_reservations(self, first_request, second_request):
        '''
        First request, find when the intersection is available for car and then reserve the intersection for that time. 
        Repeat w/ second request, using the (updated) reservation matrices which now include the first request.
        If either request fails, return -1, -1, ... -1
        If both request succeed, return the utilities for both reservations, both reservations, and the updated queues/matrices
        '''
        lane_queues = self.copy_queues(self.lane_queues)
        (bid1, requested_start_time1, end_time1, intersection_lane1, agent_id1, current_car1) = first_request
        (bid2, requested_start_time2, end_time2, intersection_lane2, agent_id2, current_car2) = second_request

        t_enter_intersection_car1 = self.get_intersection_available_for_car(current_car1.current_lane, current_car1, lane_queues, self.strict_priority)
        start_time_1, t_interval_matrix_after_reservation1, t_available_matrix_after_reservation1 = self.attempt_reservation(t_enter_intersection_car1, intersection_lane1, current_car1, agent_id1, self.time_matrix_intervals, self.time_available_matrix)
        if start_time_1 < 0 or start_time_1 > self.time + 45.0: #NOTE THIS IS RISKY BUSINESS
            # print("Couldn't find reservation for 1")
            return -1, -1, -1, -1, -1, -1, -1
            pass
        else:
            time_intersection_entrance_free = self.get_intersection_entrance_free_time(current_car1.current_lane, t_available_matrix_after_reservation1)
            lane_queues[current_car1.current_lane] += [(current_car1, time_intersection_entrance_free)] #TODO:  This needs to be when I'm out of the control region

        time_car2_can_enter_intersection = self.get_intersection_available_for_car(current_car2.current_lane, current_car2, lane_queues, self.strict_priority)
        start_time_2, fake_interval_matrix_after2, fake_available_matrixafter2 = self.attempt_reservation(time_car2_can_enter_intersection, intersection_lane2, current_car2, agent_id2, t_interval_matrix_after_reservation1, t_available_matrix_after_reservation1)  


        if start_time_2 < 0:
            # print("Couldn't find reservation for 2")
            return -1, -1, -1, -1, -1, -1, -1
        else:
            time_intersection_entrance_free = self.get_intersection_entrance_free_time(current_car2.current_lane, fake_available_matrixafter2)
            # fake_lane_queues[current_car2.current_lane] += [(current_car2, time_intersection_entrance_free)] #TODO:  This needs to be when I'm out of the control region
        if self.svo_squared:
            utility1 = self.get_svo_utility_squared(start_time_1, start_time_2, current_car1, current_car2) 
            utility2 = self.get_svo_utility_squared(start_time_2, start_time_1, current_car2, current_car1) 
        else:
            utility1 = self.get_svo_utility(start_time_1, start_time_2, current_car1, current_car2) 
            utility2 = self.get_svo_utility(start_time_2, start_time_1, current_car2, current_car1) 
        return utility1, utility2, start_time_1, start_time_2, t_interval_matrix_after_reservation1, t_available_matrix_after_reservation1, lane_queues

    def get_svo_utility(self, start_time_ego, start_time_2, car_ego, car_2):
        ''' SVO for an ego vehicle, calculated using time in the control regions'''
        time_in_control_ego = start_time_ego - car_ego.time_entered_control
        time_in_control_2 = start_time_2 - car_2.time_entered_control
        utility_ego = time_in_control_ego*np.cos(car_ego.svo_theta) + time_in_control_2*np.sin(car_ego.svo_theta)
        return -utility_ego
    
    def get_svo_utility_squared(self, start_time_ego, start_time_2, car_ego, car_2):
        time_in_control_ego = start_time_ego - car_ego.time_entered_control
        time_in_control_2 = start_time_2 - car_2.time_entered_control        
        utility_ego = (time_in_control_ego**2)*np.cos(car_ego.svo_theta) + (time_in_control_2**2)*np.sin(car_ego.svo_theta)
        return -utility_ego        

    def one_bid_per_agent_queue(self, bid_queue):
        ''' Returns a queue that only has one bid/request per agent'''
        agent_max_requests = {}
        cleaned_queue = []
        for bid, requested_start_time, end_time, intersection_lane, agent_id, current_car in bid_queue:
            if agent_id in agent_max_requests.keys():
                if bid > agent_max_requests[agent_id][0]:
                    agent_max_requests[agent_id] = (bid, requested_start_time, end_time, intersection_lane, agent_id, current_car)
            else:
                agent_max_requests[agent_id] = (bid, requested_start_time, end_time, intersection_lane, agent_id, current_car)
        for agent_id in agent_max_requests.keys():
            cleaned_queue += [agent_max_requests[agent_id]]
        return cleaned_queue

    def update_lane_queues(self, lane_queues):
        """ Remove cars in self.lane_queues whose start time window already began. """
        for lane in lane_queues.keys():
            updated_lane_queue = []
            for (car, start_time) in lane_queues[lane]:
                if start_time >= car.time:
                    updated_lane_queue += [(car,start_time)] #Only keep cars that haven't entered intersection
            lane_queues[lane] = updated_lane_queue
        return lane_queues

    def copy_queues(self, lane_queues):
        new_queues = {}
        for lane in lane_queues.keys():
            new_queues[lane] = []
            for vehicle_time in lane_queues[lane]:
                new_queues[lane] += [vehicle_time]
        return new_queues


    