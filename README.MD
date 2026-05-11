# Polytope Vertex Enumeration

A C++ tool for enumerating the vertices of a compact polytope from its halfspace description.

## Overview

Given a polytope defined by a system of linear inequalities (H-representation), this project computes all of its vertices (V-representation) using the **polar duality** method, and visualizes the result as a 3D mesh.

## Background

A compact polytope $P \subset \mathbb{R}^n$ admits two equivalent finite descriptions:

- **H-representation:** an intersection of halfspaces, $P = \{\, x \in \mathbb{R}^n \mid A x \leq b \,\}$, where $A \in \mathbb{R}^{m \times n}$ and $b \in \mathbb{R}^m$.
- **V-representation:** the convex hull of a finite vertex set, $P = \mathrm{conv}\{v_1, \dots, v_k\}$.

Translating between the two — the **vertex enumeration problem** — is fundamental in computational geometry and is widely used in optimization, reachability analysis, control invariant set computation, and contact mechanics.

## The Polar Duality Method

The implementation in `VertexEnum::con2vert` solves vertex enumeration by reducing it to a **convex hull** computation in the dual space, which is then handled by [Qhull](http://www.qhull.org/).

### Setup

Polar duality requires the polytope to contain the origin in its interior. As a result, the function aslo requires an interior **seed** point $x_0$ satisfying $A x_0 < b$ strictly, and we work in shifted coordinates $y = x - x_0$. The constraints become

$$A(y + x_0) \leq b \;\;\Longleftrightarrow\;\; A y \leq b - A x_0 \equiv \tilde b,$$

with $\tilde b > 0$ componentwise by construction. Define

$$P' = \{\, y \in \mathbb{R}^n \mid A y \leq \tilde b \,\},$$

so $0 \in \texttt{int}(P')$ and the vertices of $P$ are recovered as $v_i = y_i + x_0$.

### Constructing the polar dual

For a polytope $P'$ containing the origin in its interior, the **polar dual** is

$$(P')^\circ = \{\, z \in \mathbb{R}^n \mid z^\top y \leq 1 \;\; \forall y \in P' \,\}.$$

The key duality theorem states:

> Facets of $P'$ correspond to vertices of $(P')^\circ$, and vertices of $P'$ correspond to facets of $(P')^\circ$.

Each row constraint $a_i^\top y \leq \tilde b_i$ of $P'$ becomes, after dividing by $\tilde b_i > 0$, the constraint $(a_i / \tilde b_i)^\top y \leq 1$. The point

$$d_i = a_i / \tilde b_i \in \mathbb{R}^n$$

is therefore a **vertex of the polar dual** $(P')^\circ$.

### Recovering primal vertices from the dual hull

Computing $\mathrm{conv}(\{d_1, \dots, d_m\}) = (P')^\circ$ via Qhull gives the dual hull as a set of facets, each described by a supporting hyperplane $n^\top z + \beta = 0$ with outward normal $n$ and offset $\beta$. By the duality correspondence, each such facet of $(P')^\circ$ maps to a vertex of $P'$ at

$$y^* = -\frac{n}{\beta},$$

and the corresponding primal vertex is $v = y^* + x_0$.

### Why this is efficient

The polar duality approach reduces vertex enumeration — a problem with no known output-polynomial algorithm in general — to a single convex hull call, which Qhull solves in $O(m \log m)$ expected time in 2D/3D. The construction requires only that an interior seed be known, which is cheaper to obtain (using random sampling).

## Getting Started:

### Project Layout

```
.
├── include/
│   └── vertex_enum.h        # con2vert, computeMesh templates
├── src/
│   └── main.cpp             # example driver
├── scripts/
│   ├── visualize.py         # Open3D viewer
│   └── vertices.txt         # written by the executable at runtime
├── lib/                     # vendored Qhull sources
└── CMakeLists.txt
```

### Prerequisites

- [Eigen 3](https://libeigen.gitlab.io/)
- [Open3D](http://www.open3d.org/) (Python, for visualization)

### Installation

Install Eigen:
```bash
sudo apt install libeigen3-dev
```

Install Python visualization dependencies:
```bash
pip install open3d
```

Clone the repository:
```bash
git clone https://github.com/kunalnk123690/vertex_enumeration.git
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

This executes the example in `src/main.cpp`, which defines a 16-constraint polytope in $\mathbb{R}^3$ with an interior seed at $(5.0, 11.0, 0.5)$, runs `VertexEnum::con2vert` to enumerate its vertices, and uses `VertexEnum::computeMesh` to triangulate the convex hull and compute the polytope volume.

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