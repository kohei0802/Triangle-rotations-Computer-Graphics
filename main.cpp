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
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity(); 
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.


    float sin = std::sin(rotation_angle / 180 * M_PI);
    float cos = std::cos(rotation_angle / 180 * M_PI);

    model << cos, -1 * sin, 0, 0, 
             sin, cos, 0, 0, 
             0, 0, 1, 0, 
             0, 0, 0, 1;

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

    eye_fov *= M_PI / 180;

    float t = std::tan(eye_fov / 2) * std::abs(zNear);
    float r = aspect_ratio * t;
    float l = -1 * r;
    float b = -1 * t;

    zNear *= -1;
    zFar *= -1;

    projection << 2 * zNear / (r - l), 0, (l + r) / (l - r), 0, 
                  0, 2 * zNear / (t - b), (b + t) / (b - t), 0, 
                  0, 0, (zFar + zNear) / (zNear - zFar), 2 * zFar * zNear / (zFar - zNear), 
                  0, 0, 1, 0;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) 
{
    angle *= M_PI / 180;

    axis.normalize();

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    Eigen::Matrix3f rodrigues = Eigen::Matrix3f::Identity() * std::cos(angle);
    
    rodrigues += (1 - std::cos(angle)) * axis * axis.transpose();

    rodrigues += std::sin(angle) * (Eigen::Matrix3f() << 0, -1*axis.z(), axis.y(),
                                    axis.z(), 0, -1*axis.x(), 
                                    -1*axis.y(), axis.x(), 0).finished();

    model.block<3,3>(0,0) = rodrigues;

    return model;

}

int main(int argc, const char** argv)
{
    float angle = 0, angle2=0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    Eigen::Vector3f rot_axis = {1, 1, 0};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    // structs that contain the id 0 and 1 respectively
    auto pos_id = r.load_positions(pos); // Store a triangle. 
    auto ind_id = r.load_indices(ind); // Store the 3 indices. pos_id and ind_id probably has different ids

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        // {0}, {1}, 1 (=triangle)
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        r.set_rotation(get_rotation(rot_axis, angle2));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        // frame buffer is passed, c library handles showing to the screen 
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        } else if (key == 'w') {
            angle2 += 10;
        } else if (key == 's') {
            angle2 -= 10;
        }
    }

    return 0;
}
