from fastapi import FastAPI
from pydantic import BaseModel
import osmnx as ox
import heapq

# 🔴 THIS LINE IS CRITICAL
app = FastAPI(title="MedWay Emergency Routing API")


class RouteRequest(BaseModel):
    start_lat: float
    start_lon: float
    end_lat: float
    end_lon: float


def heuristic(u, v, graph):
    x1, y1 = graph.nodes[u]['x'], graph.nodes[u]['y']
    x2, y2 = graph.nodes[v]['x'], graph.nodes[v]['y']
    return ((x1 - x2)**2 + (y1 - y2)**2) ** 0.5


def bidirectional_astar(graph, start, goal):
    open_fwd, open_bwd = [], []
    heapq.heappush(open_fwd, (0, start))
    heapq.heappush(open_bwd, (0, goal))

    g_fwd, g_bwd = {start: 0}, {goal: 0}
    parent_fwd, parent_bwd = {start: None}, {goal: None}
    visited_fwd, visited_bwd = set(), set()

    meeting = None

    while open_fwd and open_bwd:
        _, u = heapq.heappop(open_fwd)
        visited_fwd.add(u)
        if u in visited_bwd:
            meeting = u
            break

        for v in graph.neighbors(u):
            edge = min(graph.get_edge_data(u, v).values(),
                       key=lambda x: x["travel_time"])
            cost = g_fwd[u] + edge["travel_time"]
            if v not in g_fwd or cost < g_fwd[v]:
                g_fwd[v] = cost
                heapq.heappush(open_fwd,
                               (cost + heuristic(v, goal, graph), v))
                parent_fwd[v] = u

        _, u = heapq.heappop(open_bwd)
        visited_bwd.add(u)
        if u in visited_fwd:
            meeting = u
            break

        for v in graph.predecessors(u):
            edge = min(graph.get_edge_data(v, u).values(),
                       key=lambda x: x["travel_time"])
            cost = g_bwd[u] + edge["travel_time"]
            if v not in g_bwd or cost < g_bwd[v]:
                g_bwd[v] = cost
                heapq.heappush(open_bwd,
                               (cost + heuristic(v, start, graph), v))
                parent_bwd[v] = u

    if meeting is None:
        return [], 0

    path = []
    n = meeting
    while n:
        path.append(n)
        n = parent_fwd[n]
    path.reverse()

    n = parent_bwd[meeting]
    while n:
        path.append(n)
        n = parent_bwd[n]

    return path, g_fwd[meeting] + g_bwd[meeting]


@app.post("/route")
def compute_route(data: RouteRequest):
    center = (
        (data.start_lat + data.end_lat) / 2,
        (data.start_lon + data.end_lon) / 2
    )

    graph = ox.graph_from_point(center, dist=4000, network_type="drive")
    graph = ox.add_edge_speeds(graph)
    graph = ox.add_edge_travel_times(graph)

    start = ox.distance.nearest_nodes(graph, data.start_lon, data.start_lat)
    end = ox.distance.nearest_nodes(graph, data.end_lon, data.end_lat)

    route, time_sec = bidirectional_astar(graph, start, end)
    coords = [(graph.nodes[n]["y"], graph.nodes[n]["x"]) for n in route]

    return {
        "algorithm": "Bidirectional A*",
        "travel_time_minutes": round(time_sec / 60, 2),
        "route": coords
    }
