import numpy as np
from copy import copy
from itertools import product

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

### Form of an electric vehicle
    #0 : source_position
    #1 : destination_position
    #2 : initial_battery_charge
    #3 : charging_rate
    #4 : discharging_rate
    #5 : maximum_battery_charge
    #6 : average_speed
    #7 : index
    #8 : current_position
    #9 : current_battery_charge
    #10 : charging_status
    #11 : path
    #12 : distance_travelled
    #13 : time_elapsed
    #14 : mode
    #15 : charging_time
    #16 : total_charging_time
    #17 : total_resting_time
    #18 : total_travelling_time

### Travelling function

def travelling(vehicle,j):
    
    vehicle_ = vehicle.copy()
    
    vehicle_[9] = vehicle[9] - (my_map[vehicle[8]][j])/(vehicle[4])
    
    vehicle_[13] = vehicle[13] + (my_map[vehicle[8]][j])/(vehicle[6])
    
    vehicle_[18] = vehicle[18] + (my_map[vehicle[8]][j])/(vehicle[6])
    
    vehicle_[12] = vehicle[12] + my_map[vehicle[8]][j]
    
    vehicle_[11] = vehicle[11] + [j]
    
    vehicle_[8] = j
    
    vehicle_[10] = False
    
    if vehicle_[8] == vehicle_[1]:
        
        vehicle_[14] = 'Destination Reahced'
    else:
        
        vehicle_[14] = 'active'
        
    return vehicle_

### Resting and getting charged function

def resting_with_getting_charged(vehicle,x):
    
    vehicle_ = vehicle.copy()
    
    vehicle_[10] = True
    
    vehicle_[14] = 'passive'
    
    vehicle_[9] = (x/100) * vehicle[5]
    
    vehicle_[13] = vehicle[13] + (vehicle_[9] - vehicle[9])/vehicle[3]
    
    vehicle_[15] = (vehicle_[9] - vehicle[9])/vehicle[3]
    
    vehicle_[16] = vehicle_[16] + vehicle_[15]
    
    return vehicle_

### Waiting for the charging port function

def resting_without_getting_charged(vehicle):
    
    vehicle_ = vehicle.copy()
    
    vehicle_[10] = False
    
    vehicle_[14] = 'passive'
    
    return vehicle_

### Destination reached function

def destination_reached_function(vehicle):
    
    vehicle_ = vehicle.copy()
    
    vehicle_[14] = 'Destination reached'
    
    return vehicle_

def get_state(vehicle_list):
    
    state =[]
    
    for i in range(0,number_of_cities):
        
        city_i = []
        
        for vehicle in vehicle_list:
            
            if vehicle[8] == i:
                city_i.append(vehicle)
                
        state.append(city_i)
        
        
    return state

def check_state(state):
    
    for city in state:
        
        for vehicle in city:
            
            if vehicle[9] < 0 or vehicle[9] > vehicle[5]:
                
                return False
    n_none = 0
    
    for city in state:
        
        for vehicle in city:
            
            if vehicle[10] == None:
                
                n_none = n_none + 1
                
    if n_none == number_of_vehicles:
        return True
    
    for city in state:
        
        number_of_vehicles_getting_charged_in_the_city = 0
        
        for vehicle in city:
            
            if vehicle[10] == True:
                number_of_vehicles_getting_charged_in_the_city = number_of_vehicles_getting_charged_in_the_city + 1
                
        if number_of_vehicles_getting_charged_in_the_city == 0:
            
            for vehicle in city:
                
                if vehicle[14] == 'passive':
                    return False
        if number_of_vehicles_getting_charged_in_the_city > 1:
            
            return False
        
    return True

def get_children_of_vehicle(vehicle):
    
    children_of_vehicle = []
    
    if vehicle[8] == vehicle[1]:
        
        children_of_vehicle.append(destination_reached(vehicle))
        
        
        
    else:
        #RULE 1
        for j  in list(set(cities) - set(vehicle[11])):
            
            if my_map[vehicle[8]][j] != np.inf:
                children_of_vehicle.append(travelling(vehicle,j))
                    
                
                
            
        #RULE2
        x = vehicle[9] / vehicle[5]
        
        y = int(round(x,2)*100) + 1
        
        for i in range(y,101):
            children_of_vehicle.append(resting_with_getting_charged(vehicle,i))
        
        
        
        #RULE3
        children_of_vehicle.append(resting_without_getting_charged(vehicle))
    
    
    
            
    return children_of_vehicle

def get_children_of_all_vehicles_in_a_specific_city(city):
    
    children_of_all_vehicles_in_a_specific_city = []
    
    for vehicle in city:
        
        children_of_vehicle = get_children_of_vehicle(vehicle)
        
        children_of_all_vehicles_in_a_specific_city.append(children_of_vehicle)
        
    return children_of_all_vehicles_in_a_specific_city        

def get_children_of_all_vehicles(state):
    
    children_of_all_vehicles = []
    
    for city in (state):
        
        children_of_all_vehicles_in_a_specific_city = get_children_of_all_vehicles_in_a_specific_city(city)
                  
        children_of_all_vehicles = children_of_all_vehicles + children_of_all_vehicles_in_a_specific_city
        
    return children_of_all_vehicles

def get_waiting_time(city):
    
    for vehicle in city:
        
        if vehicle[10] == True:
            return vehicle[15]

# Function to get next states from a state

def get_states(state):
    
    children_of_all_vehicles = get_children_of_all_vehicles(state)
    
    all_possible_combinations = list(product(*children_of_all_vehicles))
    
    list_of_all_possible_states = []
    
    for combination in all_possible_combinations:
        
        vehicle_list = list(combination)
        
        state = get_state(vehicle_list)
        
        list_of_all_possible_states.append(state)
        
    list_of_all_valid_states = list(filter(check_state,list_of_all_possible_states))
    
    for state in list_of_all_valid_states:
        
        for city in state:
            
            for vehicle in city:
                
                if vehicle[10] == False and vehicle[14] == 'passive':
                    
                    vehicle[17] = vehicle[17] + get_waiting(city)                    
                    
                    vehicle[13] = vehicle[13] + get_waiting_time(city)
                
    return list_of_all_valid_states       
    

number_of_cities = int(input("Enter the number of cities: "))

print(f"The cities are named as city_i where i ranges from 0 to {number_of_cities-1}")

cities = list(range(0,number_of_cities))

my_map = get_map(number_of_cities)

number_of_vehicles = int(input('Enter the number of electric vehicles: '))

initial_vehicle_list = []

for i in range(0,number_of_vehicles):
    
    vehicle = []
    
    vehicle.append(int(input(f'Enter the source position of vehicle_{i}: ')))
    vehicle.append(int(input(f'Enter the destination position of vehicle_{i}: ')))
    vehicle.append(int(input(f'Enter the initial battery charge of vehicle_{i}: ')))
    vehicle.append(int(input(f'Enter the charging rate of vehicle_{i}: ')))
    vehicle.append(int(input(f'Enter the discharing rate of vehicle_{i}: ')))
    vehicle.append(int(input(f'Enter the maximum battery charge of vehicle_{i}: ')))
    vehicle.append(int(input(f'Enter the average speed of vehicle_{i}: ')))
    
    l = [i,vehicle[0],vehicle[2],None,[vehicle[0]],0,0,None,0,0,0,0]
    
    vehicle = vehicle + l
    
    initial_vehicle_list.append(vehicle)
    print('\n')

initial_state = get_state(initial_vehicle_list)

def goal_check(state):
    
    for city in state:
        
        for vehicle in city:
            
            if vehicle[8] != vehicle[1]:
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

# PATH COST FUNCTION

def path_cost(state):
    
    l = []
    
    for city in state:
        
        for vehicle in city:
            
            l.append(vehicle[13])
            
    return max(l)

# EDGE COST FUNCTION

def edge_cost(m,n):
    
    return path_cost(n) - path_cost(m)

# HEURISTIC FUNCTION

def heuristic(state):
    
    if goal_check(state):
        return 0
    else:
        
        l = []
        
        for city in state:
            
            for vehicle in city:
                
                min_dist = dijkstra_algorithm(get_city_node(vehicle[8]),get_city_node(vehicle[1]))
                                  
                l.append(min_dist/vehicle[6])
                
        h_state =  max(l)
        
        list_of_parent_state_objects = get_list_of_parent_state_objects(state,closed_list)
        
        if list_of_parent_state_objects == []:
            return h_state
        else:
            
            for parent_state_object in list_of_parent_state_objects:
                
                if h_state < abs((parent_state_object.heuristic) - edge_cost(parent_state_object.state,state)):
                    
                    h_state = abs((parent_state_object.heuristic) - edge_cost(parent_state_object.state,state))
                    
                    
            return h_state
    

def get_list_of_parent_state_objects(child_state,list_of_state_objects):
    
    list_of_parent_state_objects = []
    
    for state_object in list_of_state_objects:
        
        if not not_in_list(child_state,get_states(state_object.state)):
            
            list_of_parent_state_objects.append(state_object)
            
    return list_of_parent_state_objects

def not_in_list(state,list_of_states):
    
    for s in list_of_states:
        
        if check_equal_states(s,state):
            return False
        
    return True

class State():
    
    def __init__(self,state):
        
        self.state = state
        self.heuristic = heuristic(self.state)
        self.path_cost = path_cost(self.state)
        self.evaluation = self.heuristic + self.path_cost
        

def get_min_f_n(list_of_state_objects):
    
    f_n = []
    
    for state_object in list_of_state_objects:
        
        f_n.append(state_object.evaluation)
        
    return min(f_n)

def not_in_list_of_state_objects(state,list_of_state_objects):
    
    for state_object in list_of_state_objects:
        
        if check_equal_states(state,state_object.state):
            return False
        
    return True

def check_equal_states(state1,state2):
    
    for i in range(0,number_of_cities):
        
        if len(state1[i]) != len(state2[i]):
            return False
        if len(state1[i]) == len(state2[i]):
            l_i = len(state1[i])
            
            for j in range(0,l_i):
                if state1[i][j][7:11] != state2[i][j][7:11] or state1[i][j][14] != state2[i][j][14]:
                    return False
                
    return True

closed_list = []

# BEST FIRST SEARCH

def best_first_search(initial_state):
    
    initial_state_object = State(initial_state)
    open_list = [initial_state_object]
    
    while True:
        
        if open_list == []:
            return 'Open List is empty'
        
        else:
            
            min_f_n = get_min_f_n(open_list)
            
            for i,state_object in enumerate(open_list):
                
                if state_object.evaluation == min_f_n:
                    
                    new_state_object = open_list.pop(i)
                    
            if goal_check(new_state_object.state):
                return new_state_object
            else:
                
                states = get_states(new_state_object.state)
                closed_list.append(new_state_object)
                
                
                for state_ in states:
                    
                    if not_in_list_of_state_objects(state_,open_list) and not_in_list_of_state_objects(state_,closed_list):
                        
                        state__object = State(state_)
                        
                        open_list.append(state__object)
                        
                        
                    if not not_in_list_of_state_objects(state_,open_list):
                        
                        for i,state_object in enumerate(open_list):
                            
                            if check_equal_states(state_,state_object.state):
                                
                                s_o = State(state_)
                                
                                if s_o.path_cost < state_object.path_cost:
                                    
                                    open_list.pop(i)
                                    open_list.append(s_o)
                                    
        

goal_state = best_first_search(initial_state)


for city in goal_state.state:
    
    for vehicle in city:
        
        if vehicle[16] > 0:
            
            if vehicle.current_battery_charge != 0:
                
                vehicle[16] = vehicle[16] - vehicle[9]/vehicle[3]
                
                vehicle[13] = vehicle[13] - vehicle[9]/vehicle[3]
                
                vehicle[9] = 0




for city in goal_state.state:
            for vehicle in city:
                print(f'Vehicle {vehicle[7]}')
                print(f"path: {vehicle[11]}")
                print(f"time elapsed = {vehicle[13]}")
                print(f"travel time = {vehicle[18]}")
                print(f"total charging time = {vehicle[16]}")
                print(f"total resting time = {vehicle[17]}")
                print(f"current battery charge = {vehicle[9]}")
                print('\n')