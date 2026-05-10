#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include "vertex_enum.h"

using namespace std;

int main() {
    
    Eigen::Matrix<double, -1, -1> A(16, 3);
    Eigen::VectorXd b(16);
    A <<   -0.847702,   -0.528588,   0.0446707,
           -0.847555,    -0.52783,  -0.0551829,
          0.00667266,    0.999942,  0.00843881,
          0.00409143,    0.999936,   -0.010557,
          -0.0137297,    0.999884, -0.00665363,
            0.999964,  0.00563098, -0.00630906,
            0.999961,   0.0073058,  0.00492147,
            0.999954,  -0.0032878, -0.00896377,
            0.999948, -0.00775337,  0.00668269,
            0.323013,   -0.946347, -0.00944929,
                   1,           0,           0,
                   0,           1,           0,
                   0,           0,           1,
                  -1,          -0,          -0,
                  -0,          -1,          -0,
                  -0,          -0,          -1;
    b << -8.26957, -8.31089, 12.0239, 11.9977, 11.8841, 11.6213, 11.6429, 11.5348, 11.4999, 2.87184, 20, 20, 1, 0, 0, 0;
    Eigen::Vector3d seed = Eigen::Vector3d(5.0, 11.0, 0.5); // seed satisfies A*seed < b


    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::vector<Eigen::Vector3d>> mesh;

    // Compute vertices of the polytope defined by A and b using the con2vert function
    VertexEnum::con2vert<double, 3>(A, b, seed, vertices);

    // Compute the volume of the polytope and its mesh facets using the computeMesh function
    double volume = VertexEnum::computeMesh<double, 3>(vertices, mesh);
    
    cout << "Vertices:\n";
    for (const auto& vertex : vertices) {
        cout << vertex.transpose() << "\n";
    }

    cout << "Volume of Polytope: " << volume << "\n";
    cout << "Done!\n";

    std::ofstream outfile("../scripts/vertices.txt");
    for (const auto& v : vertices) {
        outfile << v.x() << "," << v.y() << "," << v.z() << "\n";
    }
    outfile.close();

}
