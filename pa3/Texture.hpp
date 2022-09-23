//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = u * width;
        float v_img = (1 - v) * height;

        // find center point
        int cx = (u_img - (int)u_img) > 0.5 ? std::ceil(u_img) : std::floor(u_img);
        int cy = (v_img - (int)v_img) > 0.5 ? std::ceil(v_img) : std::floor(v_img);

        // find the color of the center point of the four pixels around the center
        // Note: image_data's origin is lower left corner
        //       texture map's origin is higher left corner
        auto u00 = image_data.at<cv::Vec3b>(cy+0.5, cx-0.5);
        auto u01 = image_data.at<cv::Vec3b>(cy-0.5, cx-0.5);
        auto u10 = image_data.at<cv::Vec3b>(cy+0.5, cx+0.5);
        auto u11 = image_data.at<cv::Vec3b>(cy-0.5, cx+0.5);

        // s, t
        auto s = u_img - (cx-0.5);
        auto t = v_img - (cy-0.5);

        // interpolation
        // Unsatisfactory results
        // auto u0 = u00 + s*(u10-u00);
        // auto u1 = u01 + s*(u11-u01);
        // auto color = u0 + t*(u1-u0);
        // better results
        auto u0 = (1-s)*u00 + s*u10;
        auto u1 = (1-s)*u01 + s*u11;
        auto color = (1-t)*u1 + t*u0;

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
