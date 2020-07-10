##### run_experiments.py
##### Author:  Noam Buckman
##### Description:  Loads simulation settings (saved in results_directory) 
#####               and run the simulations under four different variations:
#####           1.  Scrict FCFS Priority + No Pairwise SVO Swapping
#####           2.  Scrict FCFS Priority + Pairwise SVO Swapping
#####           3.  Not Scrict FCFS Priority + No Pairwise SVO Swapping
#####           4.  Not Scrict FCFS Priority + Pairwise SVO Swapping
##### In strict FCFS, even if another agent technically arrives to the stopline first, since they 
##### arrived in the control region and bid first, they get priority.  
##### In pairwise SVO swapping, vehicles may exchange positions with other vehicles in the queue.
##### Results are then saved in the results_directory
 

import sys, os, pickle

sys.path.append('/Users/noambuckman/git-repos/traffic_simulations/src/')
import TrafficSimulator as ts
import TrafficCoordinator as tc


##### Create directory for where simulation results will be saved #####
results_directory = '/Users/noambuckman/git-repos/traffic_simulations/results/'
if not os.path.exists(results_directory):
    os.makedirs(results_directory)

###### Name of experiment must match name of experiment settings that were saved
experiment_name = "all_cars_h25_mixed3" 
all_cars_simulations_list = pickle.load(open(results_directory+experiment_name+".p",'rb'))
all_experiment_results = []
for i_sim in range(len(all_cars_simulations_list)):
    print("Simulation # %d \n "%i_sim)
    
    # Load cars for given simulation
    all_cars = all_cars_simulations_list[i_sim]
    

    ####Cordinator/Simulator Parameters.  
    # A TrafficSimulator is re-generated for each experiment.
    # The following settings are the same across all experiments and sims.
    traffic_simulator = ts.TrafficSimulator()
    traffic_simulator.deltaT = 0.05
    traffic_simulator.batch = True
    traffic_simulator.initialize_map(10,10,4,0.1)
    traffic_simulator.coordinator = tc.BatchTrafficCoordinator()
    traffic_simulator.coordinator.bid_tax = 0
    #IMPORTANT PARAMATERS
    traffic_simulator.time_for_next_car = -1
    traffic_simulator.coordinator.k_batch_intervals_time = 4.0 #s
    traffic_simulator.coordinator.k_replan_increment_sec = 0.5 #s
    traffic_simulator.coordinator.k_max_retries = 60
    traffic_simulator.coordinator.k_min_number_cars_without_reservations = 4
    
    ##### Experiment 1)  Strict Priority + No Swaps
    print("Strict Priority + No Swaps")
    traffic_simulator.coordinator.strict_priority = True
    traffic_simulator.coordinator.swaps_allowed = False
    
    # Experiment 1 Results
    sim1_results = ts.run_sim_experiments(traffic_simulator, all_cars)
    
    
    #### Regenerate the TrafficSimulator for next experiment
    traffic_simulator = ts.TrafficSimulator()
    traffic_simulator.deltaT = 0.05
    traffic_simulator.batch = True
    traffic_simulator.initialize_map(10,10,4,0.1)
    traffic_simulator.coordinator = tc.BatchTrafficCoordinator()
    traffic_simulator.coordinator.bid_tax = 0
    traffic_simulator.time_for_next_car = -1
    traffic_simulator.coordinator.k_batch_intervals_time = 4.0 #s
    traffic_simulator.coordinator.k_replan_increment_sec = 0.5 #s
    traffic_simulator.coordinator.k_max_retries = 60
    traffic_simulator.coordinator.k_min_number_cars_without_reservations = 4
    
    
    
    ##### Experiment 2)  Strict Priority + Swaps
    print("Strict Priority + Swaps")
    traffic_simulator.coordinator.strict_priority = True
    traffic_simulator.coordinator.swaps_allowed = True    

    sim2_results = ts.run_sim_experiments(traffic_simulator, all_cars)
    
    #### Regenerate the TrafficSimulator for next experiment
    traffic_simulator = ts.TrafficSimulator()
    traffic_simulator.deltaT = 0.05
    traffic_simulator.batch = True
    traffic_simulator.initialize_map(10,10,4,0.1)
    traffic_simulator.coordinator = tc.BatchTrafficCoordinator()
    traffic_simulator.coordinator.bid_tax = 0
    traffic_simulator.time_for_next_car = -1
    traffic_simulator.coordinator.k_batch_intervals_time = 4.0 #s
    traffic_simulator.coordinator.k_replan_increment_sec = 0.5 #s
    traffic_simulator.coordinator.k_max_retries = 60
    traffic_simulator.coordinator.k_min_number_cars_without_reservations = 4

    
    ##### Experiment 3)  Relaxed Priority + No Swaps
    print("Relaxed Priority + No Swaps")

    traffic_simulator.coordinator.strict_priority = False
    traffic_simulator.coordinator.swaps_allowed = False

    sim3_results = ts.run_sim_experiments(traffic_simulator, all_cars)
        


    #### Regenerate the TrafficSimulator for next experiment
    traffic_simulator = ts.TrafficSimulator()
    traffic_simulator.deltaT = 0.05
    traffic_simulator.batch = True
    traffic_simulator.initialize_map(10,10,4,0.1)
    traffic_simulator.coordinator = tc.BatchTrafficCoordinator()
    traffic_simulator.coordinator.bid_tax = 0
    traffic_simulator.time_for_next_car = -1
    traffic_simulator.coordinator.k_batch_intervals_time = 4.0 #s
    traffic_simulator.coordinator.k_replan_increment_sec = 0.5 #s
    traffic_simulator.coordinator.k_max_retries = 60
    traffic_simulator.coordinator.k_min_number_cars_without_reservations = 4
    
    
    print("Relaxed Priority + Swaps")
    ##### Experiment 4)  Relaxed Priority + Swaps
    traffic_simulator.coordinator.strict_priority = False
    traffic_simulator.coordinator.swaps_allowed = True    
    
    sim4_results = ts.run_sim_experiments(traffic_simulator, all_cars)
    
    this_simulation_results = [sim1_results, sim2_results, sim3_results, sim4_results]
    all_experiment_results += [this_simulation_results]    

    experiment_file_name = results_directory + experiment_name + '_results.p'
    pickle.dump(all_experiment_results, open(experiment_file_name,'wb'))
    print("Experiment %d done.  Saved at %s"%(i_sim, experiment_file_name))