// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    Vector3f p(x, y, 0);
    std::vector<bool>  d(3, true);
    for (int i=0; i<3; i++){
        Vector3f v1 = _v[(i+1) % 3] - _v[i], v2 = p - _v[i];
        v1.z() = 0; v2.z() = 0;
        d[i] = v1.cross(v2).z() > 0;
    }
    for (int i=1; i<3; i++)
        if (d[i] != d[0]) return false;
    return true;
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

float rst::rasterizer::get_z_value(float x, float y, const Triangle &t){
    auto v = t.toVector4();
    float alpha, beta, gamma;
    std::tie(alpha, beta, gamma) = computeBarycentric2D(x, y, t.v);
    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    z_interpolated *= w_reciprocal;
    return z_interpolated;
}


std::tuple<float, float> rst::rasterizer::super_sampling(int x, int y, const Triangle &t){
    int count = 0;
    float z_value = 0, delta = 1.0/3.0;
    for (int i=1; i<=2; i++)
        for (int j=1; j<=2; j++){
            float xx = x + i * delta, 
                  yy = y + j * delta;
            if (insideTriangle(xx, yy, t.v)) {
                ++count;
                z_value += get_z_value(xx, yy, t);
            }
        }
    return {count/4.0, z_value/count};
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    // TODO : Find out the bounding box of current triangle.
    float minx = this->width, maxx = 0, miny = this->height, maxy = 0;
    for (int i =0; i<3; i++){
        minx = std::min(minx, t.v[i].x());
        miny = std::min(miny, t.v[i].y());
        maxx = std::max(maxx, t.v[i].x());
        maxy = std::max(maxy, t.v[i].y());
    }
    
    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int x=minx; x<=maxx; ++x)
        for (int y=miny; y<=maxy; ++y){
            if (insideTriangle(x+0.5, y+0.5, t.v)){
                // If so, use the following code to get the interpolated z value.
                float alpha, beta, gamma;
                std::tie(alpha, beta, gamma) = computeBarycentric2D(x, y, t.v);
                float z_interpolated = get_z_value(x, y, t);
                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                 int idx = get_index(x, y);
                if (z_interpolated < depth_buf[idx]){
                    depth_buf[idx] = z_interpolated;
                    auto color = t.getColor();
                    set_pixel(Vector3f(x, y, 1.0), color);
                }
            }
                //std::cout<<smp<<" "<<z_interpolated<<std::endl;
            // float smp, z_interpolated;
            // std::tie(smp, z_interpolated) = super_sampling(x, y, t);
            // if (smp > 0){
            //     int idx = get_index(x, y);
            //     if (z_interpolated < depth_buf[idx]){
            //         depth_buf[idx] = z_interpolated;
            //         auto color = t.getColor();
            //         set_pixel(Vector3f(x, y, 1.0), smp * color);
            //     }
            // }
        }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on