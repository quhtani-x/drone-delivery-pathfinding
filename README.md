# Autonomous Drone Delivery (A* path planning)

A drone has to fly from the depot to a delivery house across a city, avoiding
buildings and no-fly zones. It plans the shortest **safe** route with the A*
algorithm and then flies it, all in a live visual window.

Click anywhere to drop a new delivery point and it replans the route instantly.
If you box it in completely it tells you there's no safe route.

## features

- A* pathfinding on a city grid with 8-direction (diagonal) movement
- random buildings / no-fly zones each run
- click to set a new delivery target → live replan
- animated drone flying the planned route

## run

```bash
pip install pygame
python sim.py
```

tags: ai, pathfinding, drones, simulation, pygame, robotics

this is the brain a real delivery drone needs - "get there without hitting stuff".
