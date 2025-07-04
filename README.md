# AAA* Pathfinding Algorithm (Anomaly-Aware Adaptive A*)

This project implements AAA*, an enhancement of the A* algorithm that uses anomaly-aware data (e.g. traffic, hazards) to dynamically reweight paths in a city grid.

## Features
- Real-time anomaly heatmap integration
- Dynamic path cost adjustment
- Rerouting around hazards
- Fast execution and low memory footprint

## How to Run
```bash
g++ main.cpp -o aaa_star
./aaa_star
```

## Sample Input Grid (3x4)
```
0 0 0.5 0
0 1.5 1.5 0
0 0 0 0
```

## Output
```
Path: (0,0) (1,0) (2,0) (2,1) (2,2) (2,3)
Grid with path ('*') and anomalies ('#'):
* . . .
* # # .
* * * *
Execution Time: 0 ms
```

## Author
Abdiwali Ahmed

## License
MIT
