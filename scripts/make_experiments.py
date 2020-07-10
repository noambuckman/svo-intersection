# make_experiments.py
# Author: Noam Buckman
# Description: Generate Simulation Settings
# This Notebook generates and saves multiple experiment savings for 
# analyzing the effects of different parameters.
# The settings can be varied by % human (non-communicating) vehicles and SVO type.

# Output: Multiple Pickle (.p) files that save the Vehicle settings.

#  Note:  Make sure to change the path and results_directory to match your machine

import sys, os, pickle
import numpy as np

sys.path.append('/Users/noambuckman/git-repos/traffic_simulations/src/')
import TrafficSimulator as ts


results_directory = '/Users/noambuckman/git-repos/traffic_simulations/results/'
if not os.path.exists(results_directory):
    os.makedirs(results_directory)


def assign_human(sim_car_lists, p_human):
    sim_car_lists_with_humans = []
    for list_of_cars in sim_car_lists:
        #Choose Human Type
        list_of_car_with_human_specs = []
        
        for (arrival_time, agentID, human_type, svo_theta, turn_direction, start_lane) in list_of_cars:
            coin_flip = np.random.random()
            if coin_flip < p_human:
                new_human_type = "Human"
            else:
                new_human_type = "AV"
            
            list_of_car_with_human_specs += [(arrival_time, agentID, new_human_type, svo_theta, turn_direction, start_lane)]
        sim_car_lists_with_humans += [list_of_car_with_human_specs]
    return sim_car_lists_with_humans

def assign_svo(sim_car_lists, svo_theta_list):
    sim_car_lists_with_svo = []
    for list_of_cars in sim_car_lists:
        #Choose Human Type
        list_of_car_with_svo_specs = []
        
        for (arrival_time, agentID, human_type, svo_theta, turn_direction, start_lane) in list_of_cars:
            
            
            new_svo_theta = np.random.choice(svo_theta_list)

            
            
            list_of_car_with_svo_specs += [(arrival_time, agentID, human_type, new_svo_theta, turn_direction, start_lane)]
        sim_car_lists_with_svo += [list_of_car_with_svo_specs]
    return sim_car_lists_with_svo    


#### 1. Generate (and save) the arrival times for all vehicles¶
# This will be fixed for all experiments since we don't want to vary the traffic conditions, 
# just the vehicle-specific settings (communicating, SVO)

number_sims = 25
all_cars_arrivals_turns = [ts.generate_simulation_cars(16, 0.33, 0.33, 0.1) for s in range(number_sims)]
pickle.dump(all_cars_arrivals_turns,open(results_directory+'all_cars_arrivals_turns.p','wb'))



#### 2. Vary the percentage of humans
all_cars_h0 = assign_human(all_cars_arrivals_turns, 0.0)
all_cars_h25 = assign_human(all_cars_arrivals_turns, 0.25)
all_cars_h50 = assign_human(all_cars_arrivals_turns, 0.50)
all_cars_h75 = assign_human(all_cars_arrivals_turns, 0.50)
all_cars_h100 = assign_human(all_cars_arrivals_turns, 1.00)

pickle.dump(all_cars_h0,open(results_directory+'all_cars_h0.p','wb'))
pickle.dump(all_cars_h25,open(results_directory+'all_cars_h25.p','wb'))
pickle.dump(all_cars_h50,open(results_directory+'all_cars_h50.p','wb'))
pickle.dump(all_cars_h75,open(results_directory+'all_cars_h75.p','wb'))
pickle.dump(all_cars_h100,open(results_directory+'all_cars_h100.p','wb'))

##### 3.  Vary the distribution of SVOs of the vehicles
all_cars_h0_all_ego = assign_svo(all_cars_h0, [0.0])
all_cars_h0_all_pro = assign_svo(all_cars_h0, [np.pi/4.0])
all_cars_h0_mixed3 = assign_svo(all_cars_h0, [np.pi/4.0, np.pi/6.0, 0])
all_cars_h0_mixed4 = assign_svo(all_cars_h0, [np.pi/4.0, np.pi/6.0, np.pi/12.0, 0])

pickle.dump(all_cars_h0_all_ego,open(results_directory+'all_cars_h0_ego.p','wb'))
pickle.dump(all_cars_h0_all_pro,open(results_directory+'all_cars_h0_all_pro.p','wb'))
pickle.dump(all_cars_h0_mixed3,open(results_directory+'all_cars_h0_mixed3.p','wb'))
pickle.dump(all_cars_h0_mixed4,open(results_directory+'all_cars_h0_mixed4.p','wb'))

all_cars_h25_all_ego = assign_svo(all_cars_h25, [0.0])
all_cars_h25_all_pro = assign_svo(all_cars_h25, [np.pi/4.0])
all_cars_h25_mixed3 = assign_svo(all_cars_h25, [np.pi/4.0, np.pi/6.0, 0])
all_cars_h25_mixed4 = assign_svo(all_cars_h25, [np.pi/4.0, np.pi/6.0, np.pi/12.0, 0])

pickle.dump(all_cars_h25_all_ego,open(results_directory+'all_cars_h25_ego.p','wb'))
pickle.dump(all_cars_h25_all_pro,open(results_directory+'all_cars_h25_all_pro.p','wb'))
pickle.dump(all_cars_h25_mixed3,open(results_directory+'all_cars_h25_mixed3.p','wb'))
pickle.dump(all_cars_h25_mixed4,open(results_directory+'all_cars_h25_mixed4.p','wb'))

all_cars_h50_all_ego = assign_svo(all_cars_h50, [0.0])
all_cars_h50_all_pro = assign_svo(all_cars_h50, [np.pi/4.0])
all_cars_h50_mixed3 = assign_svo(all_cars_h50, [np.pi/4.0, np.pi/6.0, 0])
all_cars_h50_mixed4 = assign_svo(all_cars_h50, [np.pi/4.0, np.pi/6.0, np.pi/12.0, 0])

pickle.dump(all_cars_h50_all_ego,open(results_directory+'all_cars_h50_ego.p','wb'))
pickle.dump(all_cars_h50_all_pro,open(results_directory+'all_cars_h50_all_pro.p','wb'))
pickle.dump(all_cars_h50_mixed3,open(results_directory+'all_cars_h50_mixed3.p','wb'))
pickle.dump(all_cars_h50_mixed4,open(results_directory+'all_cars_h50_mixed4.p','wb'))

all_cars_h75_all_ego = assign_svo(all_cars_h75, [0.0])
all_cars_h75_all_pro = assign_svo(all_cars_h75, [np.pi/4.0])
all_cars_h75_mixed3 = assign_svo(all_cars_h75, [np.pi/4.0, np.pi/6.0, 0])
all_cars_h75_mixed4 = assign_svo(all_cars_h75, [np.pi/4.0, np.pi/6.0, np.pi/12.0, 0])

pickle.dump(all_cars_h75_all_ego,open(results_directory+'all_cars_h75_ego.p','wb'))
pickle.dump(all_cars_h75_all_pro,open(results_directory+'all_cars_h75_all_pro.p','wb'))
pickle.dump(all_cars_h75_mixed3,open(results_directory+'all_cars_h75_mixed3.p','wb'))
pickle.dump(all_cars_h75_mixed4,open(results_directory+'all_cars_h75_mixed4.p','wb'))

all_cars_h100_all_ego = assign_svo(all_cars_h100, [0.0])
all_cars_h100_all_pro = assign_svo(all_cars_h100, [np.pi/4.0])
all_cars_h100_mixed3 = assign_svo(all_cars_h100, [np.pi/4.0, np.pi/6.0, 0])
all_cars_h100_mixed4 = assign_svo(all_cars_h100, [np.pi/4.0, np.pi/6.0, np.pi/12.0, 0])

pickle.dump(all_cars_h100_all_ego,open(results_directory+'all_cars_h100_ego.p','wb'))
pickle.dump(all_cars_h100_all_pro,open(results_directory+'all_cars_h100_all_pro.p','wb'))
pickle.dump(all_cars_h100_mixed3,open(results_directory+'all_cars_h100_mixed3.p','wb'))
pickle.dump(all_cars_h100_mixed4,open(results_directory+'all_cars_h100_mixed4.p','wb'))
