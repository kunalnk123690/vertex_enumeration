# Polytope Vertex Enumeration

A computational tool for enumerating the vertices of convex polytopes.

## Overview

This project implements **vertex enumeration** of convex polytopes using [cddlib](https://github.com/cddlib/cddlib) library which is the implementation double description method in C.

## Background

A convex polytope can be represented in two ways:

- **H-representation (Halfspace):** As the intersection of a finite number of half-spaces, i.e., a system of linear inequalities $Ax \leq b$.
- **V-representation (Vertex):** As the convex hull of a finite set of vertices.

Converting between these two representations is a fundamental problem in computational geometry and has applications in optimization, computational geometry, and combinatorics.

## Getting Started

### Prerequisites

- cdd library
- Open3D for visualization

### Installation
Install cdd library:
```bash
sudo apt install libcdd-dev
```

Install Python Open3D:
```bash
pip install open3d
```

Clone the repository
```bash
git clone https://github.com/kunalnk123690/vertex_enumeration.git --branch=cdd
cd vertex_enumeration
```

### Build

```bash
mkdir build && cd build
cmake ..
make && cd ..
```
From the vertex_enumeration root directory:
```bash
./build/vertex_enumeration
```

This executes the example in `src/main.cpp`, which defines a 16-constraint polytope in $\mathbb{R}^3$ with an interior seed at $(5.0, 11.0, 0.5)$, runs `VertexEnumCDD::con2vert` to enumerate its vertices, and uses `VertexEnumCDD::computeMesh` to triangulate the convex hull and compute the polytope volume.

The program will:
1. Print each computed vertex and the polytope volume.
2. Write the vertex coordinates to `scripts/vertices.txt` as comma-separated `x,y,z` values, one vertex per line.

### Visualize

After running the executable, generate the 3D view from the project root:
```bash
python3 scripts/visualize.py
```

`visualize.py` reads `scripts/vertices.txt`, builds the convex hull, and opens an interactive Open3D window where the polytope can be rotated, zoomed, and inspected.



## Output

After a successful run you will see, in the terminal, the list of computed vertices and the polytope volume, e.g.:

```
Vertices:
  5.0  11.0   1.0
  ...
Volume of Polytope: <value>
Done!
```

and an Open3D window showing the reconstructed polytope when `visualize.py` is run.