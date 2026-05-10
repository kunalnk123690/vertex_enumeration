#ifndef VERTEX_ENUM_H
#define VERTEX_ENUM_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <stdexcept>

#include "libqhullcpp/Qhull.h"
#include "libqhullcpp/QhullPoint.h"
#include "libqhullcpp/QhullPoints.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullHyperplane.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullVertexSet.h"

using namespace std;

namespace VertexEnum {

    /**
     * @brief Converts constraints to vertices of a polytope using the dual Qhull method.
     * 
     * Given a polytope defined by linear constraints A*x <= b, this function computes
     * the vertices of the polytope using convex hull computation on the dual space.
     * 
     * @tparam T Numeric type (float, double, etc.)
     * @tparam dim Dimension of the polytope (compile-time constant)
     * @param A_column_major Matrix of constraint coefficients (column-major format), shape: (m, dim)
     * @param b Vector of constraint bounds, shape: (m, 1)
     * @param seed A feasible point inside the polytope (must satisfy A*seed < b), shape: (dim, 1)
     * @param Vertices Output vector to store computed vertices of the polytope
     */
    template <typename T, int dim>
    inline void con2vert(const Eigen::Matrix<T, -1, dim> &A_column_major, 
                         const Eigen::Matrix<T, -1, 1> &b, 
                         const Eigen::Matrix<T, dim, 1> &seed, 
                         std::vector<Eigen::Matrix<T, dim, 1>> &Vertices) {
        Eigen::Matrix<T, -1, dim, Eigen::RowMajor> A = A_column_major;

        // Ensure matrix dimensions are compatible
        if(A.rows() != b.size()) {
            throw std::invalid_argument("A and b dimensions do not match.\n");
        }

        // Adjust b by subtracting A * seed
        Eigen::Matrix<T, -1, 1> b_adjusted = b - A * seed;

        // Normalize A by dividing rows by the corresponding elements of b
        Eigen::Matrix<T, -1, -1, Eigen::RowMajor> D = A.array().colwise() / b_adjusted.array();

        // 1. Initialize Qhull
        orgQhull::Qhull qhull;

        // "Qt" = triangulated output, "n" = compute normals
        qhull.runQhull("", dim, D.rows(), D.data(), "Qt n");        

        // Step 6: Extract vertices from the dual (which are facets in primal)
        // Each facet of the convex hull corresponds to a vertex of the polytope
        orgQhull::QhullFacetList facets = qhull.facetList();

        for(auto facet = facets.begin(); facet != facets.end(); ++facet) {
            // Get the hyperplane equation of this facet: n·x + offset = 0
            orgQhull::QhullHyperplane hyperplane = facet->hyperplane();
            T offset = hyperplane.offset();

            // The vertex in the primal is at: -n / offset + seed
            Eigen::Matrix<T, dim, 1> vertex;
            for(int j = 0; j < dim; ++j) {
                vertex(j) = -hyperplane[j] / offset + seed(j);
            }

            Vertices.push_back(vertex);
        }

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


}


#endif // VERTEX_ENUM_H