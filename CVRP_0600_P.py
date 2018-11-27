from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import pandas as pd

###########################
# Problem Data Definition #
###########################
def create_data_model():
    """Stores the data for the problem"""
    data = {}
    # Locations in block unit
    global places
    global distance_matrix
    global time_matrix

    places = ["Depot","Airoli","Amar Mahal, Mumbai","Saki Naka, Andheri East","Andheri West","Bandra West",
              "Bhayandar","Borivali","Dadar t.t","Dharavi, Mumbai","ghansoli","goregaon East","kalwa","Kharghar",
              "kopri, Thane","Lalbaug, Mumbai","mahim, Mumbai","evershine nagar, malad West","Metro Cinema, Mumbai","Mulund","Powai","sanpada, Mumbai",
              "seawoods station, Mumbai","vasai","virar","Waghbil Naka","wasi naka, Mumbai","Worli Naka"]
    
    df = pd.read_csv('C:\\Users\\sdiva\\Documents\\Python_scripts\\master_distance_matrix_depot.csv')
    df = df.drop(df.columns[0], axis=1)
    lf = pd.read_csv('C:\\Users\\sdiva\\Documents\\Python_scripts\\master_time_matrix.csv')
    lf = lf.drop(lf.columns[0], axis=1)
    
    distance_matrix = df.values.tolist()
    time_matrix = lf.values.tolist()
    data["locations"] = distance_matrix 
    raw_demands = [0,37,60,67,14,4,27,32,8,34,35,19,28,12,
                       106,5,3,24,5,38,15,14,12,34,5,58,9,7]
           
    
    data["vehicle_capacity"] =  [20]*30 +  [12]*5 + [4]*20
    data["num_vehicles"] = len(data["vehicle_capacity"])

    min_capacity = min(data["vehicle_capacity"])
    demand_arr, demand_dict = create_demand_arr_dict(min_capacity, raw_demands)

    data["num_locations"] = len(demand_arr)
    data["demands"] = demand_arr
    data["depot"] = 0
    return data,demand_arr,demand_dict

def create_demand_arr_dict(mul, demand): # Create a virtual demand array considering max capacity of vehicle

    demand_arr = []
    demand_dict = {}
    for idx, i in enumerate(demand):
        if i > mul:
            nmul = i // mul
            rem = i % mul
            for j in range(nmul):
                index = len(demand_arr)
                demand_dict[index] = idx
                demand_arr.append(mul)                
            if rem > 0:
                index = len(demand_arr)
                demand_dict[index] = idx
                demand_arr.append(rem)                           
        else:
            index = len(demand_arr)
            demand_dict[index] = idx
            demand_arr.append(i) 
    return demand_arr, demand_dict
  
#######################
# Problem Constraints #
#######################

def create_distance_evaluator(data, demand_dict):
    """Creates callback to return distance between points."""

    def distance_evaluator(from_node, to_node):
        """Returns the distance between the two nodes"""
        from_index = demand_dict[from_node]
        to_index = demand_dict[to_node]
        return distance_matrix[from_index][to_index]

    return distance_evaluator

def create_demand_evaluator(data):
    """Creates callback to get demands at each location."""
    _demands = data["demands"]

    def demand_evaluator(from_node, to_node):
        """Returns the demand of the current node"""
        del to_node
        return _demands[from_node]

    return demand_evaluator

def add_capacity_constraints(routing, data, demand_evaluator):
    """Adds capacity constraint"""
    capacity = 'Capacity'
    routing.AddDimensionWithVehicleCapacity(
        demand_evaluator,
        0,  # null capacity slack
        data["vehicle_capacity"],
        True,  # start cumul to zero
        capacity)
def create_time_callback(data, demand_dict):
    """Creates callback to get total times between locations."""
    def time_callback(from_node, to_node):
        """Returns the total time between the two nodes"""
        from_index = demand_dict[from_node]
        to_index = demand_dict[to_node]
        return time_matrix[from_index][to_index]
    return time_callback
def add_time_window_constraints(routing, data, time_callback):
    """Add Global Span constraint"""
    time = "Time"
    horizon = 30000
    routing.AddDimension(
        time_callback,
        horizon, # allow waiting time
        horizon, # maximum time per vehicle
        False, # Don't force start cumul to zero. This doesn't have any effect in this example,
               # since the depot has a start window of (0, 0).
        time)    
###########
# Printer #
###########
def print_solution(data, routing, assignment,demand_dict):
    """Prints assignment on console"""
    print('Objective: {}'.format(assignment.ObjectiveValue()))
    print(len(distance_matrix),len(places))
    total_distance = 0
    total_load = 0
    total_time = 0
    cap_arr = data["vehicle_capacity"]
    vehicle_data = [[0 for y in range(4)] for x in range(len(cap_arr))]
    capacity_dimension = routing.GetDimensionOrDie('Capacity') 
    time_dimension = routing.GetDimensionOrDie('Time')   
    for vehicle_id in xrange(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        distance = 0

        flag = routing.IsVehicleUsed(assignment, vehicle_id)
        print("Is vehicle {0} used? - {1}".format(vehicle_id,flag))

        while not routing.IsEnd(index):
            load_var = capacity_dimension.CumulVar(index)
            plan_output += ' {} Load({}) -> '.format(places[demand_dict[routing.IndexToNode(index)]
                ], assignment.Value(load_var))
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            distance += routing.GetArcCostForVehicle(previous_index, index,
                                                     vehicle_id)
        load_var = capacity_dimension.CumulVar(index)
        route_load = assignment.Value(load_var)
        time_var = time_dimension.CumulVar(index)
        route_time = assignment.Value(time_var)

        plan_output += ' {0} Load({1})\n'.format(
            places[demand_dict[routing.IndexToNode(index)]
                ], route_load)
        plan_output += 'Distance of the route: {}km\n'.format(distance/1000)
        plan_output += 'Load of the route: {}\n'.format(
            route_load)
        plan_output += 'Time for the route: {0}s, {1}mins'.format(route_time, route_time//60)
        print(plan_output)
        total_distance += distance
        total_load += route_load
        total_time += route_time
        vehicle_data[vehicle_id] = [distance//1000,route_time//60,cap_arr[vehicle_id],route_load] 
    print('Total Distance of all routes: {}km'.format(total_distance/1000))
    print('Total Load of all routes: {}'.format(total_load))
    print('Total Time of all routes: {}s, {}mins'.format(total_time,total_time//60))
    df=pd.DataFrame(vehicle_data, columns =['Distance','Time','Capacity','Load'])
    df.to_csv("vehcile_data_0600_P.csv",index= False)
    


########
# Main #
########
def main():
    """Entry point of the program"""
    # Instantiate the data problem.
    data, demand_arr, demand_dict = create_data_model()
    # Create Routing Model
    routing = pywrapcp.RoutingModel(data["num_locations"],
                                    data["num_vehicles"], data["depot"])
    # Define weight of each edge
    distance_evaluator = create_distance_evaluator(data, demand_dict)
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
    # Add Capacity constraint
    demand_evaluator = create_demand_evaluator(data)
    add_capacity_constraints(routing, data, demand_evaluator)

    
    time_callback = create_time_callback(data, demand_dict)
    add_time_window_constraints(routing, data, time_callback)

    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)  # pylint: disable=no-member
    search_parameters.time_limit_ms = 10000
    
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    if not assignment:
        print("No solution!")
        
    print_solution(data, routing, assignment, demand_dict)


if __name__ == '__main__':
    main()
