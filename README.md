# 🛣️ CityRoute Navigator

CityRoute Navigator is a C++ command-line project that simulates a route navigation system between major Iranian cities using **graph traversal algorithms**. It provides multiple pathfinding strategies such as BFS, DFS, and Dijkstra (shortest path), and supports terminal delays in cities.

---

## 📌 Features

- Support for two types of maps:
  - 20-minute terminal delay
  - 60-minute terminal delay
- Three pathfinding options:
  - 📍 **Shortest path** (Dijkstra)
  - 🧭 **Two different paths** (BFS and DFS)
  - 🚏 **Path with waypoint**
- Accurate calculation of:
  - 🕒 Total travel time (including terminal delays)
  - 📏 Total distance (in km)
- Clean and user-friendly CLI interface.

---

## 🧠 Algorithms Used

- ✅ Breadth-First Search (BFS)
- ✅ Depth-First Search (DFS)
- ✅ Dijkstra's Shortest Path Algorithm

---

## 🏙️ Cities Included

- Tabriz
- Tehran
- Arak
- Isfahan
- Semnan
- Mashhad
- Zahedan
- Birjand
- Ahvaz

---

## 🛠️ How to Run

1. Compile with a C++ compiler (e.g., g++):
   ```bash
   g++ -o cityroute main.cpp
