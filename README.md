# navigation_planner_2d

Simple robot navigation planner

## Current implemented planners

### Global Planner

- Dijkstra
- RRT

### Local Planner

- DWA
- Randomized MPC

## How to install 

1. Download

```bash
git clone git@github.com:kohonda/navigation_planner_2d.git
cd navigation_planner_2d
```

2. Create venv

```bash
python3 -m venv .venv
source .venv/bin/activate
```

3. Install

```bash
cd navigation_planner_2d
pip3 install wheel
pip3 install -e .
```

## How to run simulation

```bash
cd test
python3 test_navigation.py
```

<div align="center">
<img src="https://user-images.githubusercontent.com/50091520/202967798-2d9855d2-f409-49c9-a519-d2cc6d601033.gif">
</div>

- Green/Red circle: Robot (When red, the robot is in collision with obstacles)
- Black: known static object, given as prepared map
- Blue: unknown static/dynamic objects
- Red: LiDAR scan

## How to create new map

```bash
cd script
python3 map_creator.py [config-yaml]
```

Example config is [here](script/example.yaml)
