import traci
import sumolib
import time
import random

# Configuration
SUMO_BINARY = "sumo-gui"  # or "sumo" for CLI
CONFIG_FILE = "sumo/map.sumocfg"
AMBULANCE_IDS = ["ambulance_1", "ambulance_2"]  # List of ambulance IDs
JUNCTIONS = ["J1", "J2", "J3", "J4", "J5", "J6"]

# Timing tracking
AMBULANCE_START_TIMES = {}  # Track when each ambulance starts
AMBULANCE_END_TIMES = {}    # Track when each ambulance finishes
PREEMPTION_ENABLED = True   # Toggle for signal preemption

# Emergency codes configuration
AMBULANCE_PRIORITIES = {
    "ambulance_2": "CODE_BLACK",  # Highest priority (from W1)
    "ambulance_1": "CODE_RED"     # Lower priority (from N1)
}

# Pheromone configuration
PHEROMONE_DECAY = 0.95
PHEROMONE_DEPOSIT = 1.0

class Junction:
    def __init__(self, id):
        self.id = id
        self.pheromone = {}  # Stores pheromone values to neighboring junctions
        self.traffic_level = 0
    
    def update_traffic_level(self):
        # Get number of vehicles around this junction
        incoming_edges = traci.trafficlight.getControlledLinks(self.id)
        vehicle_count = 0
        for links in incoming_edges:
            if links:
                edge_id = links[0][0].split('_')[0]  # Remove lane index
                try:
                    vehicle_count += traci.edge.getLastStepVehicleNumber(edge_id)
                except traci.exceptions.TraCIException:
                    # Try with lane index
                    try:
                        vehicle_count += traci.edge.getLastStepVehicleNumber(edge_id + "_0")
                    except:
                        pass
        self.traffic_level = vehicle_count
        
    def update_pheromone(self, next_junction, value):
        if next_junction not in self.pheromone:
            self.pheromone[next_junction] = 0
        self.pheromone[next_junction] = (self.pheromone[next_junction] * PHEROMONE_DECAY) + value

def get_next_junctions(current_junction):
    """Get list of possible next junctions based on network topology"""
    controlled_links = traci.trafficlight.getControlledLinks(current_junction)
    next_junctions = set()
    for links in controlled_links:
        if links:
            to_edge = links[0][1]
            for junction in JUNCTIONS:
                if junction != current_junction and to_edge.endswith(junction):
                    next_junctions.add(junction)
    return list(next_junctions)

def calculate_route_score(from_junction, to_junction, junctions):
    """Calculate score for moving from one junction to another"""
    if to_junction not in junctions[from_junction].pheromone:
        junctions[from_junction].pheromone[to_junction] = 1.0
        
    pheromone = junctions[from_junction].pheromone[to_junction]
    traffic = junctions[to_junction].traffic_level + 1  # Add 1 to avoid division by zero
    
    return (pheromone ** 2) / traffic  # Give more weight to pheromone

def get_ambulance_route_info(ambulance_id):
    """Get ambulance's current and next edge information"""
    if ambulance_id not in traci.vehicle.getIDList():
        return None, None
    
    current_edge = traci.vehicle.getRoadID(ambulance_id)
    if current_edge.endswith("_0"):
        current_edge = current_edge[:-2]
    
    route = traci.vehicle.getRoute(ambulance_id)
    route_index = traci.vehicle.getRouteIndex(ambulance_id)
    next_edge = route[route_index + 1] if route_index < len(route) - 1 else None
    if next_edge and next_edge.endswith("_0"):
        next_edge = next_edge[:-2]
    
    return current_edge, next_edge

def find_best_route(current_junction, junctions):
    """Find best route from current junction using ACO principles"""
    if current_junction not in JUNCTIONS:
        return []
        
    route = [current_junction]
    current = current_junction
    visited = {current}
    
    while len(route) < len(JUNCTIONS):
        next_possible = get_next_junctions(current)
        next_possible = [j for j in next_possible if j not in visited]
        
        if not next_possible:
            break
            
        # Calculate scores for each possible next junction
        scores = [calculate_route_score(current, next_j, junctions) for next_j in next_possible]
        total_score = sum(scores)
        if total_score == 0:
            probabilities = [1/len(scores)] * len(scores)
        else:
            probabilities = [s/total_score for s in scores]
            
        # Choose next junction based on probabilities
        next_junction = random.choices(next_possible, probabilities)[0]
        route.append(next_junction)
        visited.add(next_junction)
        current = next_junction
        
    return route

def set_signals_for_ambulance(green_junction, routes):
    """Set traffic signals considering ambulance's route and emergency code priority"""
    global PREEMPTION_ENABLED
    if not PREEMPTION_ENABLED:
        return
        
    # Get junction's controlled links and current state
    controlled_links = traci.trafficlight.getControlledLinks(green_junction)
    current_state = list(traci.trafficlight.getRedYellowGreenState(green_junction))
    
    # Initialize all states to red
    for i in range(len(current_state)):
        current_state[i] = 'r'
    
    # Get ambulances approaching this junction and sort by priority code
    approaching_ambulances = []
    for ambulance_id in AMBULANCE_IDS:
        current_edge, next_edge = get_ambulance_route_info(ambulance_id)
        if current_edge and next_edge:
            approaching_ambulances.append(ambulance_id)
    
    # Sort ambulances by emergency code priority (CODE_BLACK first)
    approaching_ambulances.sort(
        key=lambda x: 0 if AMBULANCE_PRIORITIES[x] == "CODE_BLACK" else 1
    )
    
    # Track which directions have been set to green
    processed_directions = set()
    
    # Process each ambulance in priority order
    for ambulance_id in approaching_ambulances:
        current_edge, next_edge = get_ambulance_route_info(ambulance_id)
        if not current_edge or not next_edge:
            continue

        # Get this ambulance's route
        route = routes.get(ambulance_id, [])
        current_route_index = route.index(green_junction) if green_junction in route else -1
        next_junction = route[current_route_index + 1] if current_route_index >= 0 and current_route_index < len(route) - 1 else None
        
        # Update state for all directions from ambulance's current road
        for i, links in enumerate(controlled_links):
            if not links:
                continue
                
            from_edge, to_edge, _ = links[0]
            # Remove lane indices for comparison
            from_edge_base = from_edge.split('_')[0]
            to_edge_base = to_edge.split('_')[0]
            
            # Check if this link starts from ambulance's current road
            direction_key = (from_edge_base, to_edge_base)
            if current_edge == from_edge_base and direction_key not in processed_directions:
                if next_junction and (to_edge_base.endswith(next_junction) or 
                                    any(to_edge_base.startswith(j) for j in route)):
                    current_state[i] = 'G'  # Set green for route to next junction
                    processed_directions.add(direction_key)
                    print(f"Priority given to {ambulance_id} ({AMBULANCE_PRIORITIES[ambulance_id]}) at junction {green_junction}")
                elif not next_junction:
                    current_state[i] = 'G'  # If no next junction known, set all directions green
                    processed_directions.add(direction_key)
    
    # Apply the new state
    new_state = ''.join(current_state)
    traci.trafficlight.setRedYellowGreenState(green_junction, new_state)

# Function to format time duration
def format_time(seconds):
    return f"{seconds:.2f} seconds"

# Global variables for simulation state
last_junctions = {}  # Track last junction for each ambulance
current_routes = {}  # Track current route for each ambulance

# Main simulation loop
if __name__ == "__main__":
    import subprocess
    import os
    from datetime import datetime, timedelta
    
    def run_simulation(preemption_enabled=True):
        global PREEMPTION_ENABLED, last_junctions, current_routes
        PREEMPTION_ENABLED = preemption_enabled
        AMBULANCE_START_TIMES.clear()
        AMBULANCE_END_TIMES.clear()
        last_junctions.clear()
        current_routes.clear()
        
        print(f"\nStarting simulation with signal preemption {'enabled' if preemption_enabled else 'disabled'}...")
        sumo_cmd = [SUMO_BINARY, "-c", CONFIG_FILE]
        traci.start(sumo_cmd)
        
        # Initialize simulation variables
        step = 0
        simulation_time = 0
        ambulance_active = False
        junctions = {j_id: Junction(j_id) for j_id in JUNCTIONS}
        active_ambulances = set()
        
        try:
            while traci.simulation.getMinExpectedNumber() > 0:
                traci.simulationStep()
                step += 1
                simulation_time = traci.simulation.getTime()
                
                # Update traffic levels
                for junction in junctions.values():
                    junction.update_traffic_level()
                
                # Get current ambulances and track their times
                current_ambulances = set(aid for aid in AMBULANCE_IDS if aid in traci.vehicle.getIDList())
                
                if current_ambulances:
                    ambulance_active = True
                    
                    # Check for new ambulances
                    for ambulance_id in current_ambulances - active_ambulances:
                        AMBULANCE_START_TIMES[ambulance_id] = simulation_time
                        print(f"\n🚑 Ambulance {ambulance_id} started at {format_time(simulation_time)}")
                        active_ambulances.add(ambulance_id)
                        current_routes[ambulance_id] = []
                        last_junctions[ambulance_id] = None
                    
                    # Process each ambulance
                    for ambulance_id in current_ambulances:
                        try:
                            tls_list = traci.vehicle.getNextTLS(ambulance_id)
                            last_junction = last_junctions.get(ambulance_id)
                            
                            if tls_list:
                                current_junction = tls_list[0][0]
                                if current_junction != last_junction:
                                    if current_junction in JUNCTIONS:
                                        current_routes[ambulance_id] = find_best_route(current_junction, junctions)
                                    set_signals_for_ambulance(current_junction, current_routes)
                                    last_junctions[ambulance_id] = current_junction
                        except traci.exceptions.TraCIException:
                            continue
                    
                    # Check for finished ambulances
                    for ambulance_id in active_ambulances - current_ambulances:
                        AMBULANCE_END_TIMES[ambulance_id] = simulation_time
                        travel_time = simulation_time - AMBULANCE_START_TIMES[ambulance_id]
                        print(f"\n{'='*50}")
                        print(f"🏁 AMBULANCE {ambulance_id} FINISHED")
                        print(f"⏰ Finish time: {format_time(simulation_time)}")
                        print(f"📊 Total travel time: {format_time(travel_time)}")
                        print(f"🚨 Priority level: {AMBULANCE_PRIORITIES[ambulance_id]}")
                        print(f"{'='*50}")
                        active_ambulances.remove(ambulance_id)
                
                elif ambulance_active:
                    # Reset all signals when no ambulances are present
                    for junction in JUNCTIONS:
                        traci.trafficlight.setProgram(junction, "0")
                    ambulance_active = False
                    last_junctions.clear()
                    current_routes.clear()
                
                time.sleep(0.1)  # For simulation stability
                
        except Exception as e:
            print(f"Simulation error: {e}")
        finally:
            print("\n📊 FINAL RESULTS SUMMARY")
            print("="*50)
            for ambulance_id in AMBULANCE_IDS:
                if ambulance_id in AMBULANCE_START_TIMES and ambulance_id in AMBULANCE_END_TIMES:
                    travel_time = AMBULANCE_END_TIMES[ambulance_id] - AMBULANCE_START_TIMES[ambulance_id]
                    print(f"\n🚑 AMBULANCE {ambulance_id}:")
                    print(f"  🏁 Start time: {format_time(AMBULANCE_START_TIMES[ambulance_id])}")
                    print(f"  ⏰ Finish time: {format_time(AMBULANCE_END_TIMES[ambulance_id])}")
                    print(f"  ⌛ Total travel time: {format_time(travel_time)}")
                    print(f"  🚨 Priority: {AMBULANCE_PRIORITIES[ambulance_id]}")
            print("="*50)
            traci.close()
        
        return AMBULANCE_START_TIMES.copy(), AMBULANCE_END_TIMES.copy()
    
    # Run simulation with and without preemption
    print("\n🚦 Running simulation with signal preemption...")
    start_times_with, end_times_with = run_simulation(True)
    
    print("\n🚦 Running simulation without signal preemption...")
    start_times_without, end_times_without = run_simulation(False)
    
    # Compare results
    print("\n📊 Comparison Results:")
    for ambulance_id in AMBULANCE_IDS:
        if (ambulance_id in start_times_with and ambulance_id in end_times_with and
            ambulance_id in start_times_without and ambulance_id in end_times_without):
            time_with = end_times_with[ambulance_id] - start_times_with[ambulance_id]
            time_without = end_times_without[ambulance_id] - start_times_without[ambulance_id]
            time_saved = time_without - time_with
            print(f"\nAmbulance {ambulance_id} ({AMBULANCE_PRIORITIES[ambulance_id]}):")
            print(f"  With preemption: {format_time(time_with)}")
            print(f"  Without preemption: {format_time(time_without)}")
            print(f"  Time saved: {format_time(time_saved)}")
            print(f"  Improvement: {(time_saved/time_without*100):.1f}%")
