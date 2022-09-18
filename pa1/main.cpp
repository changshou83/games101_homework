#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the axis.
    // Then return it.
    angle = angle / 180.0 * MY_PI;
    double sin_angle = sin(angle),
           cos_angle = cos(angle);
    double axis_x = axis.x(),
           axis_y = axis.y(),
           axis_z = axis.z();

    // I + (nhat*nhat)*(1-cos_angle)+sin_angle*nhat
    model << axis_x * axis_x * (1 - cos_angle) + cos_angle, axis_y * axis_x * (1 - cos_angle) - axis_z * sin_angle, axis_z * axis_x * (1 - cos_angle) + axis_y * sin_angle, 0,
        axis_x * axis_y * (1 - cos_angle) + axis_z * sin_angle, axis_y * axis_y * (1 - cos_angle) + cos_angle, axis_z * axis_y * (1 - cos_angle) - axis_x * sin_angle, 0,
        axis_x * axis_z * (1 - cos_angle) - axis_y * sin_angle, axis_y * axis_z * (1 - cos_angle) + axis_x * sin_angle, axis_z * axis_z * (1 - cos_angle) + cos_angle, 0,
        0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    // float angle = rotation_angle / 180.0 * MY_PI;
    // double sina = sin(angle),
    //        cosa = cos(angle);

    // model << cosa, -sina, 0,
    //     sina, cosa, 0,
    //     0, 0, 1;
    Eigen::Vector3f zAxis = {0, 0, 1};
    model = get_rotation(zAxis, rotation_angle);

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f persp2ortho = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f orthoScale = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f orthoTrans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f ortho = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    // Students will implement this function

    persp2ortho << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -1 * zNear * zFar,
        0, 0, 1, 0;

    double halfFov = eye_fov / (2 * 180.0f) * MY_PI;
    double t = -1 * zNear * tan(halfFov), // multiply by -1 to show the result.
        r = t * aspect_ratio;

    // scale into "canonical" cube
    orthoScale << 1 / r, 0, 0, 0,
        0, 1 / t, 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    // center cuboid by translating
    orthoTrans << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;

    matrixOrtho = orthoScale * orthoTrans;
    projection = matrixOrtho * persp2ortho;

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

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

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

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
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
