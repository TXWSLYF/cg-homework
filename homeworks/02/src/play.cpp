#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>

using namespace Eigen;

int main()
{
    Vector3f v1 = {1, 2, 0};
    Vector3f v2 = {3.0, 4.0, 0};
    auto v3 = v1.cross(v2);

    std::cout << v1 << '\n'
              << v2 << '\n'
              << v3.z() << '\n'
              << v3[2] << '\n'
              << v3 << std::endl;

    return 0;
}