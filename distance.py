from flask import Flask, request, jsonify
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

app = Flask(__name__)

def create_data_model(vehicle_current_lat, vehicle_current_long, orders):
    data = {}
    data['vehicle_current_lat'] = vehicle_current_lat
    data['vehicle_current_long'] = vehicle_current_long
    data['depot'] = (vehicle_current_lat, vehicle_current_long)
    data['orders'] = [(order['order_pickup_lat'], order['order_pickup_long'], order['order_delivery_lat'], order['order_delivery_long']) for order in orders]
    return data

def create_distance_callback(data):
    def distance_callback(from_index, to_index):  # type: ignore
        from_lat, from_long, to_lat, to_long = data['orders'][from_index]
        return int(((from_lat - to_lat)**2 + (from_long - to_long)**2)**0.5)

    return distance_callback

def get_solution(vehicle_current_lat, vehicle_current_long, orders):
    data = create_data_model(vehicle_current_lat, vehicle_current_long, orders)

    manager = pywrapcp.RoutingIndexManager(len(data['orders']), 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    distance_callback = create_distance_callback(data)
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.time_limit.seconds = 1

    solution = routing.SolveWithParameters(search_parameters)

    return manager, routing, solution

def get_order_sequence(vehicle_current_lat, vehicle_current_long, orders):
    manager, routing, solution = get_solution(vehicle_current_lat, vehicle_current_long, orders)

    if not solution:
        return None

    index = routing.Start(0)
    order_sequence = []
    while not routing.IsEnd(index):
        order_sequence.append(orders[manager.IndexToNode(index)])
        index = solution.Value(routing.NextVar(index))

    return order_sequence

@app.route('/get_order_sequence', methods=['POST'])
def calculate_order_sequence():
    try:
        data = request.get_json()
        vehicle_current_lat = data['vehicle_current_lat']
        vehicle_current_long = data['vehicle_current_long']
        orders = data['orders']

        order_sequence = get_order_sequence(vehicle_current_lat, vehicle_current_long, orders)

        return jsonify(order_sequence)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    app.run(port=5002)
