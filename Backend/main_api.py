from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import osmnx as ox
import heapq
import math

app = FastAPI(title="MedWay Emergency Routing")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# =======================
# DATA MODELS
# =======================
class RouteRequest(BaseModel):
    patient_lat: float
    patient_lon: float
    severity: str  # LOW / MEDIUM / HIGH

# =======================
# MOCK DATA
# =======================
HOSPITALS = [
    {"name": "SMS Hospital", "lat": 26.9085, "lon": 75.8076, "beds": 10, "icu": True, "trauma": True},
    {"name": "EHCC Hospital", "lat": 26.9009, "lon": 75.7671, "beds": 5, "icu": True, "trauma": True},
    {"name": "Fortis Hospital", "lat": 26.8847, "lon": 75.8243, "beds": 6, "icu": True, "trauma": False},
    {"name": "Apex Hospital", "lat": 26.8659, "lon": 75.8081, "beds": 8, "icu": False, "trauma": False},
]

AMBULANCES = [
    {"id": "AMB-ICU-01", "lat": 26.9150, "lon": 75.8200, "type": "ICU", "available": True},
    {"id": "AMB-ALS-02", "lat": 26.9000, "lon": 75.7900, "type": "ALS", "available": True},
    {"id": "AMB-BLS-03", "lat": 26.8800, "lon": 75.8100, "type": "BLS", "available": True},
]

# =======================
# LOAD MAP ONCE
# =======================
CENTER = (26.9124, 75.7873)
G = ox.graph_from_point(CENTER, dist=10000, network_type="drive")
G = ox.add_edge_speeds(G)
G = ox.add_edge_travel_times(G)

# =======================
# ROUTING
# =======================
def heuristic(a, b):
    return math.hypot(G.nodes[a]["x"] - G.nodes[b]["x"], G.nodes[a]["y"] - G.nodes[b]["y"])

def bidirectional_astar(graph, start, goal):
    fwd, bwd = [(0, start)], [(0, goal)]
    g_f, g_b = {start: 0}, {goal: 0}
    p_f, p_b = {start: None}, {goal: None}
    v_f, v_b = set(), set()
    meet = None

    while fwd and bwd:
        _, u = heapq.heappop(fwd)
        v_f.add(u)
        if u in v_b:
            meet = u
            break

        for v in graph.neighbors(u):
            e = min(graph.get_edge_data(u, v).values(), key=lambda x: x["travel_time"])
            cost = g_f[u] + e["travel_time"]
            if v not in g_f or cost < g_f[v]:
                g_f[v] = cost
                p_f[v] = u
                heapq.heappush(fwd, (cost + heuristic(v, goal), v))

        _, u = heapq.heappop(bwd)
        v_b.add(u)
        if u in v_f:
            meet = u
            break

        for v in graph.predecessors(u):
            e = min(graph.get_edge_data(v, u).values(), key=lambda x: x["travel_time"])
            cost = g_b[u] + e["travel_time"]
            if v not in g_b or cost < g_b[v]:
                g_b[v] = cost
                p_b[v] = u
                heapq.heappush(bwd, (cost + heuristic(v, start), v))

    if not meet:
        return [], 0

    path = []
    n = meet
    while n:
        path.append(n)
        n = p_f[n]
    path.reverse()

    n = p_b[meet]
    while n:
        path.append(n)
        n = p_b[n]

    return path, g_f[meet] + g_b[meet]

# =======================
# HOSPITAL RANKING
# =======================
def rank_hospitals(patient_node, severity):
    ranked = []

    for h in HOSPITALS:
        if h["beds"] <= 0:
            continue

        h_node = ox.distance.nearest_nodes(G, h["lon"], h["lat"])
        _, eta = bidirectional_astar(G, patient_node, h_node)

        penalty = 0
        if severity == "HIGH" and not h["icu"]:
            penalty += 300
        if severity == "HIGH" and not h["trauma"]:
            penalty += 300
        if severity == "MEDIUM" and not h["icu"]:
            penalty += 180

        score = eta + penalty - (h["beds"] * 20)

        ranked.append({
            "name": h["name"],
            "lat": h["lat"],
            "lon": h["lon"],
            "beds": h["beds"],
            "eta_min": round(eta / 60, 2),
            "score": round(score, 2)
        })

    ranked.sort(key=lambda x: x["score"])
    return ranked[:3]

def select_ambulance(patient_node, severity):
    best, best_eta = None, float("inf")

    for a in AMBULANCES:
        if not a["available"]:
            continue
        if severity == "HIGH" and a["type"] != "ICU":
            continue
        if severity == "MEDIUM" and a["type"] == "BLS":
            continue

        a_node = ox.distance.nearest_nodes(G, a["lon"], a["lat"])
        _, eta = bidirectional_astar(G, a_node, patient_node)

        if eta < best_eta:
            best_eta = eta
            best = {
                "id": a["id"],
                "type": a["type"],
                "lat": a["lat"],
                "lon": a["lon"],
                "eta_min": round(eta / 60, 2)
            }

    return best

# =======================
# API
# =======================
@app.post("/route")
def route(data: RouteRequest):
    patient_node = ox.distance.nearest_nodes(G, data.patient_lon, data.patient_lat)

    hospitals = rank_hospitals(patient_node, data.severity)
    best_hospital = hospitals[0]
    ambulance = select_ambulance(patient_node, data.severity)

    amb_node = ox.distance.nearest_nodes(G, ambulance["lon"], ambulance["lat"])
    hosp_node = ox.distance.nearest_nodes(G, best_hospital["lon"], best_hospital["lat"])

    route_amb, _ = bidirectional_astar(G, amb_node, patient_node)
    route_hosp, t_h = bidirectional_astar(G, patient_node, hosp_node)

    return {
        "ambulance": ambulance,
        "best_hospital": best_hospital,
        "hospital_options": hospitals,
        "eta_to_hospital_min": round(t_h / 60, 2),
        "routes": {
            "ambulance_to_patient": [(G.nodes[n]["y"], G.nodes[n]["x"]) for n in route_amb],
            "patient_to_hospital": [(G.nodes[n]["y"], G.nodes[n]["x"]) for n in route_hosp]
        }
    }
