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

    Texture(const std::string& name, int resize)
    {
        image_data = cv::imread(name);
        cv::Size dsize(image_data.cols / resize, image_data.rows / resize);
        cv::resize(image_data, image_data, dsize, 0, 0, cv::INTER_AREA);
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

    Eigen::Vector3f getColorBilinear(float u, float v){
        float u_img = u * width;
        float v_img = (1 - v) * height;
        int u0 = std::floor(u_img), u1 = std::ceil(u_img),
            v0 = std::floor(v_img), v1 = std::ceil(v_img);
        float alpha_u = (u1 - u_img) / (u1 - u0),
              alpha_v = (v1 - v_img) / (v1 - v0);
        
        auto color1 = alpha_u * image_data.at<cv::Vec3b>(v0, u0) + (1 - alpha_u) * image_data.at<cv::Vec3b>(v0, u1),
             color2 = alpha_u * image_data.at<cv::Vec3b>(v1, u0) + (1 - alpha_u) * image_data.at<cv::Vec3b>(v1, u1);
        
        auto color = alpha_v * color1 + (1 - alpha_v) * color2;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
