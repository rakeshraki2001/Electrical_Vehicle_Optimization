# Required Imports

### numpy,copy and product

import numpy as np
from copy import copy
from itertools import product

### Electric Vehicle Class

class EV():
    
    def __init__(self,source_position=None,destination_position=None,initial_battery_charge=None,
                charging_rate=None,discharging_rate=None,maximum_battery_charge=None,average_speed=None,index=None):
        
        #Parameters given in the problem of the assignment (fixed parameters)
        self.source_position        = source_position
        self.destination_position   = destination_position
        self.initial_battery_charge = initial_battery_charge
        self.charging_rate          = charging_rate
        self.discharging_rate       = discharging_rate
        self.maximum_battery_charge = maximum_battery_charge
        self.average_speed          = average_speed
        self.index                  = index
        
        #Additional parameters (variable parameters)
        #parameter                  = intial value of the parameter
        self.current_position       = self.source_position
        self.current_battery_charge = self.initial_battery_charge
        self.charging_status        = None #Boolean Value Expected (True if charging, False if not charging)
        self.path                   = [source_position] #Gives the path list from source "position" to the current "position"
        self.distance_travelled     = 0 #Total distance travelled from initial "position" to the current "position"
        self.time_elapsed           = 0 #Total time elapsed from initial "state" to the current "state"
        self.mode                   = None # 'active' if travelling, 'passive' if it is at rest
        self.charging_time= 0
        self.total_charging_time = 0
        self.total_resting_time = 0
        self.total_travelling_time = 0
        
    def __eq__(self, other):
            if not isinstance(other, EV):
                
                return NotImplemented
            
            return self.index == other.index and self.current_position == other.current_position and self.charging_status == other.charging_status and self.mode == other.mode and self.current_battery_charge == other.current_battery_charge

### To get the map

def get_map(n):
    
    the_map = []
    
    
    for num in range(0,n):
        
        the_map.append([])
        
        
    
    for i in range(0,n):
        
        for j in range(0,n):
            
            the_map[i].append('NaN')
           
            
            
    for i in range(0,n):
        
        for j in range(0,n):
            
            if the_map[i][j] == 'NaN':
                
                if i == j:
                    
                    the_map[i][j] = 0
                elif input(f"Are city_{i} and city_{j} connected?: y/n : ") == 'y':
                    dist_ij = float(input(f"Enter the distance between city_{i} and city_{j}: "))
                    the_map[i][j] = dist_ij
                    the_map[j][i] = dist_ij
                else:
                    the_map[i][j] = np.inf
                    the_map[j][i] = np.inf
            else:
                pass
            
    return the_map

### Travelling function

def travelling(ev,j):
    
    ev_ = copy(ev)
    
    ev_.current_battery_charge = ev.current_battery_charge - (my_map[ev.current_position][j])/ev.discharging_rate
    
    ev_.time_elapsed = ev.time_elapsed + (my_map[ev.current_position][j])/(ev.average_speed)
    
    ev_.total_travelling_time = ev.total_travelling_time + (my_map[ev.current_position][j])/(ev.average_speed)
    
    ev_.distance_travelled = ev.distance_travelled + my_map[ev.current_position][j]
    
    (ev_.path) = ev.path + [j]
    
    ev_.current_position = j
    
    ev_.charging_status = False
    
    if ev_.current_position == ev_.destination_position:
        
        ev_.mode = 'Destination Reached'
    else:
        ev_.mode = 'active'
        
    return ev_

### Resting and getting charged function

def resting_with_getting_charged(ev,x):
    
    ev_ = copy(ev)
    
    ev_.charging_status = True
    
    ev_.mode = 'passive'
    
    ev_.current_battery_charge = (x/100) * ev.maximum_battery_charge 
    
    ev_.time_elapsed = ev.time_elapsed + (ev_.current_battery_charge - ev.current_battery_charge)/ev.charging_rate
    
    ev_.charging_time = (ev_.current_battery_charge - ev.current_battery_charge)/ev.charging_rate
    
    ev_.total_charging_time = ev_.total_charging_time + ev_.charging_time
    
    return ev_

### Waiting for the charging port function

def resting_without_getting_charged(ev):
    
    ev_ = copy(ev)
    
    ev_.charging_status = False
    
    ev_.mode = 'passive'
    
    return ev_

### Destination reached function

def destination_reached(ev):
    
    ev_ = copy(ev)
    
    ev_.mode = 'Destination reached'
    
    return ev_

def get_state(ev_list):
    
    
    ''' The function takes the list of all the EVs of a particular state (valid or otherwise) and gives out a list 
    (state). Each element in that list is a sub-list and the sub-list at index i contains EVs whose current location is
    at city_i'''
    state = []
    
    for i in range (0,number_of_cities):
        
        city_i = []
        
        for ev in ev_list:
            
            if ev.current_position == i:
                city_i.append(ev)
        
        state.append(city_i)
        
    
        
    return state

def check_state(state):
    
    '''Takes in a state and tells whether that state is valid or not'''
    
    for city in state:
        
        for ev in city:
            
            if (ev.current_battery_charge < 0 or ev.current_battery_charge > ev.maximum_battery_charge):
                
                return False
    n_none = 0
            
    for city in state:
        
        for ev in city:
            
            if ev.charging_status == None:
                
                n_none = n_none + 1
    
    if n_none == number_of_evs:
        return True
            
    
    
    for city in state:
           
        number_of_vehicles_getting_charged_in_the_city = 0
        
        
        for ev in city:
            
            if (ev.charging_status == True):
                number_of_vehicles_getting_charged_in_the_city = number_of_vehicles_getting_charged_in_the_city + 1
                
        if number_of_vehicles_getting_charged_in_the_city == 0:
            
            for ev in city:
                
                if ev.mode == 'passive':
                    return False
        if number_of_vehicles_getting_charged_in_the_city > 1:
            
            return False
        
        
    return True

def get_children_of_ev(ev):
    
    children_of_ev = []
    
    if ev.current_position == ev.destination_position:
        
        children_of_ev.append(destination_reached(ev))
        
        
        
    else:
        #RULE 1
        for j  in list(set(cities) - set(ev.path)):
            
            if my_map[ev.current_position][j] != np.inf:
                children_of_ev.append(travelling(ev,j))
                    
                
                
            
        #RULE2
        x = ev.current_battery_charge / ev.maximum_battery_charge
        
        y = int(round(x,2)*100) + 1
        
        for i in range(y,101):
            children_of_ev.append(resting_with_getting_charged(ev,i))
        
        
        
        #RULE3
        children_of_ev.append(resting_without_getting_charged(ev))
    
    
    
            
    return children_of_ev

def get_children_of_all_evs_in_a_specific_city(city):
    
    children_of_all_evs_in_a_specific_city = []
    
    for vehicle in city:
        
        children_of_vehicle = get_children_of_ev(vehicle)
        
        children_of_all_evs_in_a_specific_city.append(children_of_vehicle)
        
    return children_of_all_evs_in_a_specific_city        

def get_children_of_all_evs(state):
    
    children_of_all_evs = []
    
    for i,city in enumerate(state):
        
        children_of_all_evs_in_a_specific_city = get_children_of_all_evs_in_a_specific_city(city)
        
        if children_of_all_evs_in_a_specific_city != []:
            
            children_of_all_evs = children_of_all_evs + children_of_all_evs_in_a_specific_city
        
    return children_of_all_evs

def get_waiting_time(city):
    
    for vehicle in city:
        
        if vehicle.charging_status == True:
            return vehicle.charging_time

# Function to get next states from a state

def get_states(state):
    
    children_of_all_evs = get_children_of_all_evs(state)
    
    all_possible_combinations = list(product(*children_of_all_evs))
    
    list_of_all_possible_states = []
    
    for combination in all_possible_combinations:
        
        ev_list = list(combination)
        
        state = get_state(ev_list)
        
        list_of_all_possible_states.append(state)
        
    list_of_all_valid_states = list(filter(check_state,list_of_all_possible_states))
    
    for state in list_of_all_valid_states:
        
        for city in state:
            
            for vehicle in city:
                
                if vehicle.charging_status == False and vehicle.mode == 'passive':
                    
                    vehicle.total_resting_time = vehicle.total_resting_time + get_waiting(city)                    
                    
                    vehicle.time_elapsed = vehicle.time_elapsed + get_waiting_time(city)
                
    return list_of_all_valid_states       
    

number_of_cities = int(input("Enter the number of cities: "))

print(f"The cities are named as city_i where i ranges from 0 to {number_of_cities-1}")

cities = list(range(0,number_of_cities))

my_map = get_map(number_of_cities)

number_of_evs = int(input('Enter the number of electric vehicles: '))

initial_ev_list = []


for i in range(0,number_of_evs):
     
    source_position = int(input(f'Enter the source position of vehicle_{i}: '))
    destination_position = int(input(f'Enter the destination position of vehicle_{i}: '))
    initial_battery_charge = int(input(f'Enter the initial battery charge of vehicle_{i}: '))
    charging_rate = int(input(f'Enter the charging rate of vehicle_{i}: '))
    discharging_rate = int(input(f'Enter the discharing rate of vehicle_{i}: '))
    maximum_battery_charge = int(input(f'Enter the maximum battery charge of vehicle_{i}: '))
    average_speed = int(input(f'Enter the average speed of vehicle_{i}: '))
    
    ev = EV(source_position,destination_position,initial_battery_charge,charging_rate,discharging_rate,
            maximum_battery_charge,average_speed,i)
    initial_ev_list.append(ev)
    print('\n')

initial_state = get_state(initial_ev_list)

def goal_check(state):
    
    for city in state:
        
        for vehicle in city:
            
            if vehicle.current_position != vehicle.destination_position:
                return False
    return True

# DIJKSTRA'S ALGORITHM 

class City():
    
    def __init__(self,city_index):
        
        self.city_index = city_index
        self.tenative_distance_value = None

def get_all_cities_node_list():
    
    all_cities_node_list = []
    
    for i in range(0,number_of_cities):
        
        all_cities_node_list.append(City(i))
    return all_cities_node_list

def get_city_node(city_index):
    
    for node in all_cities_node_list:
        
        if node.city_index == city_index:
            return node

def get_connected_node_list(node,node_list):
    
    connected_node_list = []
    
    i = node.city_index
    
    for city_node in node_list:
        
        j = city_node.city_index
        
        if i != j and my_map[i][j] != np.inf:
            
            connected_node_list.append(city_node)
            
    return connected_node_list

def get_minimum_tenative_distance(node_list):
    
    distance_list = []
    
    for node in node_list:
        
        distance_list.append(node.tenative_distance_value)
        
    return min(distance_list)

def dijkstra_algorithm(present_node,destination_node):
    
    visited_node_list = []
    unvisited_node_list = []
    
    unvisited_node_list = all_cities_node_list.copy()
    
    for node in unvisited_node_list:
        
        if node == present_node:
            node.tenative_distance_value = 0
        else:
            node.tenative_distance_value = np.inf
            
    current_node = present_node
    
    while True:
        
        for neighbor in get_connected_node_list(current_node,unvisited_node_list):
            
            d1 = current_node.tenative_distance_value
            
            d2 = my_map[current_node.city_index][neighbor.city_index]
            
            d = d1 + d2
            
            if d < neighbor.tenative_distance_value:
                
                neighbor.tenative_distance_value = d
                
        for i,node in enumerate(unvisited_node_list):
            
            if node == current_node:
                visited_node_list.append(unvisited_node_list.pop(i))
                break
        
        if destination_node in visited_node_list:
            return destination_node.tenative_distance_value
        
        get_min_dist_value = get_minimum_tenative_distance(unvisited_node_list)
        
        for node in unvisited_node_list:
            
            if node.tenative_distance_value == get_min_dist_value:
                current_node = node
                break

all_cities_node_list = get_all_cities_node_list()

# HEURISTIC FUNCTION

def heuristic(state):
    
    if goal_check(state):
        return 0
    else:
        
        l = []
        
        for city in state:
            
            for vehicle in city:
                
                min_dist = dijkstra_algorithm(get_city_node(vehicle.current_position),get_city_node(vehicle.destination_position))
                
                l.append(min_dist/vehicle.average_speed)
                
        return max(l)

# PATH COST FUNCTION

def path_cost(state):
    
    l = []
    
    for city in state:
        
        for vehicle in city:
            
            l.append(vehicle.time_elapsed)
            
    return max(l)

# EVALUATION FUNCTION

def evaluation(state):
    
    return heuristic(state) + path_cost(state)

def get_min_f_n(list_of_states):
    
    f_n = []
    
    for state in list_of_states:
        
        f_n.append(evaluation(state))
        
    return min(f_n)

# BEST FIRST SEARCH

def best_first_search(initial_state):
    
    open_list = [initial_state]
    closed_list = []
    
    if open_list == []:
        return 'Open List is Empty'
    else:
        
        while True:
            
            if open_list == []:
                
                return 'Open List is Empty'
            
            
            min_f_n = get_min_f_n(open_list)
            
            for i,s in enumerate(open_list):
                
                if evaluation(s) == min_f_n:
                    
                    state = open_list.pop(i)
            
            if goal_check(state):
                
                return state
            
            else:
                
                states = get_states(state)
                
                closed_list.append(state)
                
                for state_ in states:
                    
                    if state_ not in open_list and state_ not in closed_list:
                        
                        open_list.append(state_)
                        
                    if state_ in open_list:
                        
                        for i,old_state in enumerate(open_list):
                            
                            if old_state == state_:
                                
                                if path_cost(state_) < path_cost(old_state):
                                    
                                    open_list.pop(i)
                                    open_list.append(state_)                                
                                    
            
            
        
        

for city in best_first_search(initial_state):
            for vehicle in city:
                print(f'Vehicle {vehicle.index}')
                print(f"path: {vehicle.path}")
                print(f"time elapsed = {vehicle.time_elapsed}")
                print(f"travel time = {vehicle.total_travelling_time}")
                print(f"total charging time = {vehicle.total_charging_time}")
                print(f"total resting time = {vehicle.total_resting_time}")
                print(f"current battery charge = {vehicle.current_battery_charge}")
                print('\n')