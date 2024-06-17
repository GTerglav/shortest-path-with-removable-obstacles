# Shortest path with removable obstacles

Project for MatRač class.

Implementing algorithms from paper *Computing Shortest Paths in the Plane with Removable Obstacles* by P.K. Agarwal, N. Kumar, S. Sintos and S. Suri.

## Requirements
- flask
- plotly
- pickle 



## Project includes:
-  `main.py`: Flask app that allows to run algorithms on example problems.
- `persistentRBTree.py`: An implementation of persistent red-black trees with limited node copying from paper  *Planar point location using persistent search trees* by N. Sarnak in R.E. Tarjan
- `viabilityGraph.py`: Naive algorithm for viability graph 
- `sweepViablityGraph`: "Sweep" algorithm adapted to this version of problem from the book *Computational Geometry: Algorithms and Applications*, chapter 15 
- `sparseGraph.py`: "Sparse" algorithm from chapter 4 of original paper.
- `problems.py`: Examples of problems. Also generated some problems with larger n and saved them in folder generatedProblems using pickle.

