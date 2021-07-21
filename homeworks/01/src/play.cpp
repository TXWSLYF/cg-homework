#include <eigen3/Eigen/Eigen>
#include <iostream>

int main()
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    std::cout << view << std::endl;

    return 0;
}