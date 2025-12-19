# MedWay – Emergency Ambulance Routing System 🚑

MedWay is an intelligent emergency response routing system that computes the fastest ambulance routes and selects the most suitable hospital in real time using graph-based algorithms.

## Features
- Bidirectional A* algorithm for fast emergency routing
- Real-time ETA calculation
- Nearest ambulance selection
- Hospital selection based on availability & severity
- Interactive map-based frontend (Leaflet.js)
- Backend powered by FastAPI + OpenStreetMap

## Algorithms Used
- Dijkstra (baseline comparison)
- Bidirectional A* (primary routing engine)

## Tech Stack
- Backend: Python, FastAPI, OSMnx
- Frontend: HTML, CSS, JavaScript, Leaflet.js
- Maps: OpenStreetMap

## System Architecture

1. Road network is extracted from OpenStreetMap and converted into a weighted graph  
2. Edge weights represent travel time  
3. Ambulance and hospital locations are mapped as nodes  
4. Bidirectional A* computes the fastest route  
5. Frontend visualizes routes, ETAs, and entities on an interactive map  


## How to Run Locally
```bash
cd Backend
python -m uvicorn medway_api:app --reload
