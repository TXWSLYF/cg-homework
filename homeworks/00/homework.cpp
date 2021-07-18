#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

// 定义旋转角度
const float angle = 45.0;

int main()
{
    // 定义二维坐标系下 p 点 (2, 1)
    Eigen::Vector3f p(2.0, 1.0, 1);

    std::cout << "定义二维坐标系下 p 点 (2, 1)" << std::endl;
    std::cout << p << std::endl;

    float cos_theta = cos(angle * M_PI / 180);
    float sin_theta = sin(angle * M_PI / 180);

    // 定义旋转矩阵
    Eigen::Matrix3f m1;
    m1 << cos_theta, -sin_theta, 0, sin_theta, cos_theta, 0, 0, 0, 1;
    std::cout << "旋转矩阵" << std::endl;
    std::cout << m1 << std::endl;

    // 定义位移矩阵
    Eigen::Matrix3f m2;
    m2 << 1, 0, 1, 0, 1, 2, 0, 0, 1;
    std::cout << "位移矩阵" << std::endl;
    std::cout << m2 << std::endl;

    std::cout << "矩阵乘法，先旋转，再平移" << std::endl;
    std::cout << m2 * m1 * p << std::endl;

    return 0;
}