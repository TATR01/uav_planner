"""Route optimization algorithms"""

import math
import random
from typing import List, Tuple, Dict, Any
from core.strips import Strip, route_length
from core.geometry import Point, segment_intersection, point_in_polygon
from config.settings import (
    DP_MAX_ITERATIONS, DP_2OPT_THRESHOLD,
    GA_DEFAULT_POPULATION, GA_DEFAULT_GENERATIONS,
    GA_ELITE_RATIO, GA_TOURNAMENT_SIZE,
    GA_MUTATION_INV_RATE, GA_MUTATION_SWAP_RATE,
    ACO_DEFAULT_ANTS, ACO_DEFAULT_ITERATIONS,
    ACO_RHO, ACO_ALPHA, ACO_BETA, ACO_Q, ACO_PH_MAX, ACO_PH_MIN,
    BUFFER_PENALTY
)


def strip_end(strips: List[Strip], strip_idx: int, direction_up: bool) -> Point:
    """Get end point of strip for given direction."""
    strip = strips[strip_idx]
    if not strip.segments:
        return Point(strip.x, 0)
    
    if direction_up:
        _, y1 = strip.segments[-1]
        return Point(strip.x, y1)
    else:
        y0, _ = strip.segments[0]
        return Point(strip.x, y0)


def strip_start(strips: List[Strip], strip_idx: int, direction_up: bool) -> Point:
    """Get start point of strip for given direction."""
    strip = strips[strip_idx]
    if not strip.segments:
        return Point(strip.x, 0)
    
    if direction_up:
        y0, _ = strip.segments[0]
        return Point(strip.x, y0)
    else:
        _, y1 = strip.segments[-1]
        return Point(strip.x, y1)


def segment_hits_buffer(p1: Point, p2: Point, buffer_polygons: List[List[Point]]) -> bool:
    """Check if segment p1-p2 intersects any buffer zone."""
    for buffer_poly in buffer_polygons:
        for i in range(len(buffer_poly)):
            bp1 = buffer_poly[i]
            bp2 = buffer_poly[(i + 1) % len(buffer_poly)]
            if segment_intersection(p1, p2, bp1, bp2):
                return True
        
        if point_in_polygon(p1, buffer_poly) or point_in_polygon(p2, buffer_poly):
            return True
    
    return False


def transition_cost(strips: List[Strip], from_idx: int, from_dir: bool,
                   to_idx: int, to_dir: bool, buffer_polygons: List[List[Point]]) -> float:
    """Calculate cost of transition between two strips."""
    p1 = strip_end(strips, from_idx, from_dir)
    p2 = strip_start(strips, to_idx, to_dir)
    
    distance = p1.distance_to(p2)
    penalty = BUFFER_PENALTY if segment_hits_buffer(p1, p2, buffer_polygons) else 1.0
    
    return distance * penalty


def build_distance_matrix(strips: List[Strip], buffer_polygons: List[List[Point]]) -> List[List[Dict[str, Any]]]:
    """Build transition cost matrix between all strip pairs."""
    n = len(strips)
    dm = [[{'cost': float('inf'), 'from_dir': 0, 'to_dir': 0} for _ in range(n)] for _ in range(n)]
    
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            
            best_cost = float('inf')
            best_from_dir = False
            best_to_dir = False
            
            for from_dir in [False, True]:
                for to_dir in [False, True]:
                    cost = transition_cost(strips, i, from_dir, j, to_dir, buffer_polygons)
                    if cost < best_cost:
                        best_cost = cost
                        best_from_dir = from_dir
                        best_to_dir = to_dir
            
            dm[i][j] = {
                'cost': best_cost,
                'from_dir': best_from_dir,
                'to_dir': best_to_dir
            }
    
    return dm


def algo_dp(strips: List[Strip], buffer_polygons: List[List[Point]]) -> Dict[str, Any]:
    """DP/TSP algorithm: greedy start + 2-opt optimization."""
    n = len(strips)
    if n <= 1:
        return {'orders': [list(range(n))], 'info': 'Trivial'}
    
    dm = build_distance_matrix(strips, buffer_polygons)
    
    # Greedy start
    visited = set()
    order = [0]
    visited.add(0)
    
    while len(order) < n:
        current = order[-1]
        best_next = -1
        best_cost = float('inf')
        
        for j in range(n):
            if j not in visited and dm[current][j]['cost'] < best_cost:
                best_cost = dm[current][j]['cost']
                best_next = j
        
        if best_next >= 0:
            order.append(best_next)
            visited.add(best_next)
    
    # 2-opt
    def route_length_dm(order_list):
        total = 0.0
        for i, idx in enumerate(order_list):
            strip = strips[idx]
            points = strip.points(direction_up=(i % 2 == 0))
            
            for j in range(len(points) - 1):
                total += points[j].distance_to(points[j + 1])
            
            if i > 0:
                total += dm[order_list[i-1]][idx]['cost']
        
        return total
    
    best_length = route_length_dm(order)
    improved = True
    iterations = 0
    
    while improved and iterations < DP_MAX_ITERATIONS:
        improved = False
        iterations += 1
        
        for i in range(n - 1):
            for j in range(i + 2, n):
                new_order = order[:i+1] + order[i+1:j+1][::-1] + order[j+1:]
                new_length = route_length_dm(new_order)
                
                if new_length < best_length - DP_2OPT_THRESHOLD:
                    order = new_order
                    best_length = new_length
                    improved = True
                    break
            
            if improved:
                break
    
    return {'orders': [order], 'info': f'DP/TSP 2-opt, {iterations} iterations'}


def algo_vrp(strips: List[Strip], buffer_polygons: List[List[Point]], k: int = 2) -> Dict[str, Any]:
    """VRP algorithm: split into k routes."""
    n = len(strips)
    if n <= 1:
        return {'orders': [list(range(n))], 'info': 'VRP k=1'}
    
    k = min(k, n)
    dm = build_distance_matrix(strips, buffer_polygons)
    
    indices = list(range(n))
    indices.sort(key=lambda i: strips[i].x)
    
    orders = []
    chunk_size = math.ceil(n / k)
    
    for chunk_idx in range(k):
        chunk = indices[chunk_idx * chunk_size:(chunk_idx + 1) * chunk_size]
        if not chunk:
            continue
        
        visited = set()
        order = [chunk[0]]
        visited.add(chunk[0])
        
        while len(order) < len(chunk):
            current = order[-1]
            best_next = -1
            best_cost = float('inf')
            
            for j in chunk:
                if j not in visited and dm[current][j]['cost'] < best_cost:
                    best_cost = dm[current][j]['cost']
                    best_next = j
            
            if best_next >= 0:
                order.append(best_next)
                visited.add(best_next)
        
        orders.append(order)
    
    return {'orders': orders, 'info': f'VRP: {len(orders)} routes'}


def algo_ga(strips: List[Strip], buffer_polygons: List[List[Point]],
           population: int = 60, generations: int = 150) -> Dict[str, Any]:
    """Genetic Algorithm."""
    n = len(strips)
    if n <= 1:
        return {'orders': [[0]], 'info': ''}
    
    def random_individual():
        ind = list(range(n))
        random.shuffle(ind)
        return ind
    
    def fitness(order):
        total = 0.0
        for i, idx in enumerate(order):
            strip = strips[idx]
            points = strip.points(direction_up=(i % 2 == 0))
            for j in range(len(points) - 1):
                total += points[j].distance_to(points[j + 1])
        return 1.0 / (total + 1)
    
    pop = [random_individual() for _ in range(population)]
    best = min(pop, key=lambda x: -fitness(x))[:]
    
    for gen in range(generations):
        pop.sort(key=lambda x: -fitness(x))
        next_pop = [p[:] for p in pop[:max(2, int(population * 0.1))]]
        
        while len(next_pop) < population:
            p1 = pop[random.randint(0, min(10, population - 1))]
            p2 = pop[random.randint(0, min(10, population - 1))]
            
            child = p1[:]
            for i in range(n):
                if random.random() < 0.5:
                    child[i] = p2[i]
            
            if random.random() < 0.1:
                i, j = random.randint(0, n-1), random.randint(0, n-1)
                child[i], child[j] = child[j], child[i]
            
            next_pop.append(child)
        
        pop = next_pop[:population]
        candidate = max(pop, key=fitness)
        if fitness(candidate) > fitness(best):
            best = candidate[:]
    
    return {'orders': [best], 'info': f'GA: pop={population} gen={generations}'}


def algo_aco(strips: List[Strip], buffer_polygons: List[List[Point]],
            num_ants: int = 25, num_iterations: int = 80) -> Dict[str, Any]:
    """Ant Colony Optimization."""
    n = len(strips)
    if n <= 1:
        return {'orders': [[0]], 'info': ''}
    
    dm = build_distance_matrix(strips, buffer_polygons)
    pheromones = [[1.0] * n for _ in range(n)]
    
    def route_length_list(route_list):
        total = 0.0
        for i, idx in enumerate(route_list):
            strip = strips[idx]
            points = strip.points(direction_up=(i % 2 == 0))
            for j in range(len(points) - 1):
                total += points[j].distance_to(points[j + 1])
            if i > 0:
                total += dm[route_list[i-1]][idx]['cost']
        return total
    
    best_route = list(range(n))
    best_length = route_length_list(best_route)
    
    for it in range(num_iterations):
        for ant in range(num_ants):
            visited = [False] * n
            route = [0]
            visited[0] = True
            
            for _ in range(n - 1):
                current = route[-1]
                probs = []
                
                for j in range(n):
                    if not visited[j]:
                        eta = 1.0 / (dm[current][j]['cost'] + 1e-9)
                        p = (pheromones[current][j] ** 1.2) * (eta ** 2.5)
                        probs.append((j, p))
                
                if not probs:
                    break
                
                total_p = sum(p for _, p in probs)
                r = random.random() * total_p
                acc = 0
                
                for j, p in probs:
                    acc += p
                    if acc >= r:
                        route.append(j)
                        visited[j] = True
                        break
            
            length = route_length_list(route)
            if length < best_length:
                best_length = length
                best_route = route[:]
        
        for i in range(n):
            for j in range(n):
                pheromones[i][j] *= 0.6
        
        for i in range(len(best_route) - 1):
            pheromones[best_route[i]][best_route[i+1]] += 1000 / best_length
    
    return {'orders': [best_route], 'info': f'ACO: {num_ants} ants, {num_iterations} iter'}
