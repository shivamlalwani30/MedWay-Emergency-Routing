import osmnx as ox
import networkx as nx
import heapq


# -------------------------------------------------
# 1. LOAD ROAD NETWORK GRAPH
# -------------------------------------------------
def load_graph():
    print("Downloading map data...")

    # Center point: Malviya Nagar, Jaipur
    center_point = (26.8530, 75.8205)

    graph = ox.graph_from_point(
        center_point,
        dist=2000,          # 2 km radius
        network_type="drive"
    )

    graph = ox.add_edge_speeds(graph)
    graph = ox.add_edge_travel_times(graph)

    print("Map loaded successfully!")
    return graph


# -------------------------------------------------
# 2. DIJKSTRA ROUTING
# -------------------------------------------------
def dijkstra_route(graph, start_lat, start_lon, end_lat, end_lon):
    start_node = ox.distance.nearest_nodes(graph, start_lon, start_lat)
    end_node = ox.distance.nearest_nodes(graph, end_lon, end_lat)

    route = nx.shortest_path(
        graph,
        start_node,
        end_node,
        weight="travel_time"
    )

    travel_time = nx.shortest_path_length(
        graph,
        start_node,
        end_node,
        weight="travel_time"
    )

    return route, travel_time


# -------------------------------------------------
# 3. HEURISTIC FUNCTION (STRAIGHT-LINE DISTANCE)
# -------------------------------------------------
def heuristic(u, v, graph):
    x1, y1 = graph.nodes[u]['x'], graph.nodes[u]['y']
    x2, y2 = graph.nodes[v]['x'], graph.nodes[v]['y']
    return ((x1 - x2)**2 + (y1 - y2)**2) ** 0.5


# -------------------------------------------------
# 4. A* ROUTING
# -------------------------------------------------
def astar_route(graph, start_lat, start_lon, end_lat, end_lon):
    start_node = ox.distance.nearest_nodes(graph, start_lon, start_lat)
    end_node = ox.distance.nearest_nodes(graph, end_lon, end_lat)

    route = nx.astar_path(
        graph,
        start_node,
        end_node,
        heuristic=lambda u, v: heuristic(u, v, graph),
        weight="travel_time"
    )

    travel_time = nx.astar_path_length(
        graph,
        start_node,
        end_node,
        weight="travel_time"
    )

    return route, travel_time


# -------------------------------------------------
# 5. BIDIRECTIONAL A* ROUTING (CUSTOM IMPLEMENTATION)
# -------------------------------------------------
def bidirectional_astar(graph, start_lat, start_lon, end_lat, end_lon):
    start = ox.distance.nearest_nodes(graph, start_lon, start_lat)
    goal = ox.distance.nearest_nodes(graph, end_lon, end_lat)

    open_fwd = []
    open_bwd = []

    heapq.heappush(open_fwd, (0, start))
    heapq.heappush(open_bwd, (0, goal))

    g_fwd = {start: 0}
    g_bwd = {goal: 0}

    parent_fwd = {start: None}
    parent_bwd = {goal: None}

    visited_fwd = set()
    visited_bwd = set()

    meeting_node = None

    while open_fwd and open_bwd:
        # ---------- Forward Search ----------
        _, current_fwd = heapq.heappop(open_fwd)
        visited_fwd.add(current_fwd)

        if current_fwd in visited_bwd:
            meeting_node = current_fwd
            break

        for neighbor in graph.neighbors(current_fwd):
            edge_data = graph.get_edge_data(current_fwd, neighbor)
            min_edge = min(edge_data.values(), key=lambda x: x["travel_time"])
            cost = min_edge["travel_time"]

            tentative_g = g_fwd[current_fwd] + cost

            if neighbor not in g_fwd or tentative_g < g_fwd[neighbor]:
                g_fwd[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal, graph)
                heapq.heappush(open_fwd, (f_score, neighbor))
                parent_fwd[neighbor] = current_fwd

        # ---------- Backward Search ----------
        _, current_bwd = heapq.heappop(open_bwd)
        visited_bwd.add(current_bwd)

        if current_bwd in visited_fwd:
            meeting_node = current_bwd
            break

        for neighbor in graph.predecessors(current_bwd):
            edge_data = graph.get_edge_data(neighbor, current_bwd)
            min_edge = min(edge_data.values(), key=lambda x: x["travel_time"])
            cost = min_edge["travel_time"]

            tentative_g = g_bwd[current_bwd] + cost

            if neighbor not in g_bwd or tentative_g < g_bwd[neighbor]:
                g_bwd[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, start, graph)
                heapq.heappush(open_bwd, (f_score, neighbor))
                parent_bwd[neighbor] = current_bwd

    if meeting_node is None:
        return None, None

    # ---------- Path Reconstruction ----------
    path_fwd = []
    node = meeting_node
    while node is not None:
        path_fwd.append(node)
        node = parent_fwd[node]
    path_fwd.reverse()

    path_bwd = []
    node = parent_bwd[meeting_node]
    while node is not None:
        path_bwd.append(node)
        node = parent_bwd[node]

    full_path = path_fwd + path_bwd
    total_time = g_fwd[meeting_node] + g_bwd[meeting_node]

    return full_path, total_time


# -------------------------------------------------
# 6. MAIN EXECUTION
# -------------------------------------------------
if __name__ == "__main__":
    G = load_graph()

    print("Nodes:", len(G.nodes))
    print("Edges:", len(G.edges))

    # Sample coordinates
    start_lat, start_lon = 26.8535, 75.8200
    end_lat, end_lon = 26.8620, 75.8300

    # Dijkstra
    route_d, time_d = dijkstra_route(
        G, start_lat, start_lon, end_lat, end_lon
    )

    # A*
    route_a, time_a = astar_route(
        G, start_lat, start_lon, end_lat, end_lon
    )

    # Bidirectional A*
    route_bi, time_bi = bidirectional_astar(
        G, start_lat, start_lon, end_lat, end_lon
    )

    print("\n--- ROUTING RESULTS ---")
    print("Dijkstra time (min):", round(time_d / 60, 2))
    print("A* time (min):", round(time_a / 60, 2))
    print("Bidirectional A* time (min):", round(time_bi / 60, 2))

    print("Dijkstra route nodes:", len(route_d))
    print("A* route nodes:", len(route_a))
    print("Bidirectional A* route nodes:", len(route_bi))
