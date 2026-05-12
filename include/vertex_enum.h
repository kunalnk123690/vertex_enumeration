#ifndef VERTEX_ENUM_H
#define VERTEX_ENUM_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <stdexcept>
#include <cdd/setoper.h>
#include <cdd/cdd.h>

#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/QhullPoint.h"
#include "libqhullcpp/QhullPoints.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullHyperplane.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullVertexSet.h"

using namespace std;

namespace VertexEnumCDD {

    /**
     * @brief Converts constraints to vertices of a polytope using the double description method.
     * 
     * Given a polytope defined by linear constraints A*x <= b, this function computes
     * the vertices of the polytope using convex hull computation on the dual space.
     * 
     * @tparam T Numeric type (float, double, etc.)
     * @tparam dim Dimension of the polytope (compile-time constant)
     * @param A_column_major Matrix of constraint coefficients (column-major format), shape: (m, dim)
     * @param b Vector of constraint bounds, shape: (m, 1)
     * @param Vertices Output vector to store computed vertices of the polytope
     */    
    template <typename T, int dim>
    inline void con2vert(const Eigen::Matrix<T, -1, dim> &A_column_major, 
                            const Eigen::Matrix<T, -1, 1> &b,
                            std::vector<Eigen::Matrix<T, dim, 1>> &Vertices) {
        Eigen::Matrix<T, -1, dim, Eigen::RowMajor> A = A_column_major;

        // 1. Initialize cdd
        dd_set_global_constants();

        // 2. Prepare H-representation matrix
        long n_constraints = A.rows();
        dd_MatrixPtr H = dd_CreateMatrix(n_constraints, dim + 1);
                        
        H->representation = dd_Inequality;
        H->numbtype = dd_Real;

        for (long i = 0; i < n_constraints; ++i) {
            dd_set_d(H->matrix[i][0], (double)b(i));
            for (long j = 0; j < dim; ++j) {
                dd_set_d(H->matrix[i][j + 1], (double)(-A(i, j)));
            }
        }

        // 3. Convert H-rep to Polyhedron Object
        dd_ErrorType err;
        dd_PolyhedraPtr poly = dd_DDMatrix2Poly(H, &err);

        if (err != dd_NoError || poly == NULL) {
            if (H) dd_FreeMatrix(H);
            return;
        }

        // 4. Extract Generator Matrix (Vertices/Rays) from Polyhedron
        // This is the missing step that caused your error
        dd_MatrixPtr G = dd_CopyGenerators(poly);

        // 5. Extract Vertices
        Vertices.clear();
        for (long i = 0; i < G->rowsize; ++i) {
            double type_indicator = dd_get_d(G->matrix[i][0]);

            // type_indicator: 1.0 = Vertex, 0.0 = Ray
            if (std::abs(type_indicator) > 1e-6) {
                Eigen::Matrix<T, dim, 1> vertex;
                for (long j = 0; j < dim; ++j) {
                    double val = dd_get_d(G->matrix[i][j + 1]);
                    vertex(j) = (T)(val / type_indicator); 
                }
                Vertices.push_back(vertex);
            }
        }

        // 6. Cleanup
        // Note: You must free the Generator matrix, the Polyhedron, and the Input matrix
        dd_FreeMatrix(H);
        dd_FreeMatrix(G);
        dd_FreePolyhedra(poly);
        dd_free_global_constants();
    }    



    /**
     * @brief Generates mesh facets for plotting a polytope and computes its volume.
     * 
     * Given a set of vertices of a polytope, this function computes the convex hull
     * and extracts all facets (edges in 2D, triangles in 3D) necessary for visualization.
     * It also calculates the volume of the polytope.
     * 
     * @tparam T Numeric type (float, double, etc.)
     * @tparam dim Dimension of the polytope (must be 2 or 3)
     * @param vertices Input vector of polytope vertices, shape: (num_vertices, dim)
     * @param mesh Output vector of mesh facets, where each facet is a vector of vertices
     * @return The volume of the polytope
     */    
    template <typename T, int dim>
    inline T computeMesh(const std::vector<Eigen::Matrix<T, dim, 1>> &vertices,
                         std::vector<std::vector<Eigen::Matrix<T, dim, 1>>> &mesh) {
        static_assert(dim == 2 || dim == 3, "Dimension must be 2 or 3");

        int num_points = vertices.size();

        // Map the points vector into an Eigen matrix
        Eigen::Map<const Eigen::Matrix<T, dim, -1>> point_matrix(
            reinterpret_cast<const T *>(vertices.data()), dim, num_points);
        Eigen::Matrix<double, -1, dim, Eigen::RowMajor> points_transpose = point_matrix.transpose().template cast<double>();

        // Construct Qhull object based on dimension
        orgQhull::Qhull qhull;
        if constexpr (dim == 2) {
            qhull.runQhull("", dim, num_points, points_transpose.data(), "d Qt Qz");
        }
        else {
            qhull.runQhull("", dim, num_points, points_transpose.data(), "Qt");
        }
        T volume = qhull.volume();

        // Process the facets of the convex hull
        for(const auto &facet : qhull.facetList()) {
            if (facet.isGood()) {
                std::vector<Eigen::Matrix<T, dim, 1>> simplex;
                std::transform(facet.vertices().begin(), facet.vertices().end(), std::back_inserter(simplex),
                               [](const auto &vertex) {
                                   const auto &point = vertex.point();
                                   if constexpr (dim == 2) {
                                       return Eigen::Matrix<T, 2, 1>(static_cast<T>(point[0]), static_cast<T>(point[1]));
                                   }
                                   else if constexpr (dim == 3) {
                                       return Eigen::Matrix<T, 3, 1>(static_cast<T>(point[0]), static_cast<T>(point[1]), static_cast<T>(point[2]));
                                   }
                               });
                mesh.push_back(simplex);
            }
        }
        return volume;
    }

} // namespace VertexEnumCDD


#endif // VERTEX_ENUM_H