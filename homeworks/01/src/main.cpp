#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;

// 默认摄像机坐标系与世界坐标系方向相同？
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    // 单位矩阵
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

/**
 * @description 计算绕任意过原点的轴的旋转变换矩阵
 * @param axis 过原点向量
 * @param angle 旋转角度
 * 罗德里格斯旋转公式矩阵表示
 * https://baike.baidu.com/item/%E7%BD%97%E5%BE%B7%E9%87%8C%E6%A0%BC%E6%97%8B%E8%BD%AC%E5%85%AC%E5%BC%8F/18878562?fr=aladdin#3
 */
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix3f mat1 = Eigen::Matrix3f::Identity();

    double rotation = angle * MY_PI / 180;
    double axis_length = sqrt(pow(axis.x(), 2) + pow(axis.y(), 2) + pow(axis.z(), 2));
    Vector3f axis_1 = (1 / axis_length) * axis;

    Eigen::Matrix3f mat2 = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f mat3 = Eigen::Matrix3f::Identity();
    mat3 << 0, -axis_1.z(), axis_1.y(), axis_1.z(), 0, -axis_1.x(), -axis_1.y(), axis_1.x(), 0;
    mat1 = mat2 * cos(rotation) + (1 - cos(rotation)) * axis_1 * (axis_1.transpose()) + sin(rotation) * mat3;

    Eigen::Matrix4f rotate_martix = Eigen::Matrix4f::Identity();
    // 子矩阵操作 https://blog.csdn.net/zhangqian_shai/article/details/101756145
	rotate_martix.block(0, 0, 3, 3) = mat1;

    return rotate_martix;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotate;
    double rotation = rotation_angle * MY_PI / 180;
    rotate << cos(rotation), -sin(rotation), 0, 0, sin(rotation), cos(rotation), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    model = rotate * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    // 正交变换矩阵
    Eigen::Matrix4f orth1; // 先平移
    Eigen::Matrix4f orth2; // 再缩放
    // 透视->正交矩阵
    Eigen::Matrix4f pertoorth;
    pertoorth << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, zNear + zFar, -(zNear * zFar), 0, 0, 1, 0;
    float halfEyeAngelRadian = eye_fov / 2.0 / 180.0 * MY_PI;
    // zNear 一定是负的
    float t = -zNear * std::tan(halfEyeAngelRadian); //top y轴的最高点
    float r = t * aspect_ratio;                      //right x轴的最大值
    float l = (-1) * r;                              //left x轴最小值
    float b = (-1) * t;                              //bottom y轴的最大值
    orth1 << 1, 0, 0, -(r + l) / 2, 0, 1, 0, -(t + b) / 2, 0, 0, 1, -(zNear + zFar) / 2, 0, 0, 0, 1;
    orth2 << 2 / (r - l), 0, 0, 0, 0, 2 / (t - b), 0, 0, 0, 0, 2 / (zNear - zFar), 0, 0, 0, 0, 1;

    projection = orth2 * orth1 * pertoorth * projection;

    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 20};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    // 顶点顺序
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    Vector3f axis = {1, 1, 1};
    while (key != 27)
    {
        // 清空 color buffer 和 z buffer
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
