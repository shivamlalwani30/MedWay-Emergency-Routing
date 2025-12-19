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

## How to Run Locally
```bash
cd Backend
python -m uvicorn medway_api:app --reload
