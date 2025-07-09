//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <stdexcept>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    // pos_buf stores triangles. And each triangle is composed of 3 points.

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
// draw (build triangle) -> rasterize_wireframe (draw 3 lines) -> draw_line (bresenham)
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    // extract x of the first point
    auto x1 = begin.x();
    // extract y of the first point
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 120, 120};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);

    // the difference between the midpoint and the real line multiplied by 2dx
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    // if x changes more than y
    if(dy1<=dx1)
    {
        // if dx >= 0, set x1 to be the first point
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        // if dx < 0, set x2 to be the first point
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            // update x
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                // update y 
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            // the new "current" coordinate
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);

            // (vector3f coordinate, vector3f color)
            set_pixel(point,line_color);
        }
    }
    // if x changes less than y
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// draw (build triangle) -> rasterize_wireframe (draw 3 lines) -> draw_line (bresenham)
void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type)
{
    if (type != rst::Primitive::Triangle)
    {
        throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
    }

    // return a list of 3d vectors associated with the pos_buffer
    auto& buf = pos_buf[pos_buffer.pos_id];
    // return a list of 3d vector associated with the ind_buffer 
    auto& ind = ind_buf[ind_buffer.ind_id];

    float f1 = (100 - 0.1) / 2.0;
    float f2 = (100 + 0.1) / 2.0;

    // construct mvp matrix
    Eigen::Matrix4f mvp = projection * view * model;

    // extra rotation around any axis 
    mvp *= rotation;

    // Go through the list of 3d vectors. E.g., {0, 1, 2}
    for (auto& i : ind)
    {
        Triangle t;

        // an array of 4d vectors, each appended with 1.0 
        Eigen::Vector4f v[] = {
                // get the first point from a list of 3 points
                mvp * to_vec4(buf[i[0]], 1.0f),
                // get the second point from a list of 3 points
                mvp * to_vec4(buf[i[1]], 1.0f),
                // get the third point from a list of 3 points
                mvp * to_vec4(buf[i[2]], 1.0f)
        };

        // going through each point in the array of 4d vectors 
        for (auto& vec : v) {
            // divide each vector with w, which is the 4th row of the 4d vectors {x, y, z, w}
            vec /= vec.w();
        }

        // going through the vertices {x/w, y/w, z/w, 1}
        for (auto & vert : v)
        {
            // scale x from [-1, 1] to [0, width] 
            vert.x() = 0.5*width*(vert.x()+1.0);
            // scale y from [-1, 1] to [0, height]
            vert.y() = 0.5*height*(vert.y()+1.0);
            // scale z from [-1 1] to [0.1, 100] for occlusions
            vert.z() = vert.z() * f1 + f2;
        }


        for (int i = 0; i < 3; ++i)
        {
            // set the first vertex to be v[0]
            t.setVertex(i, v[i].head<3>());
            // set the second vertex to be v[1]
            t.setVertex(i, v[i].head<3>());
            // set the thir vertex to be v[2]
            t.setVertex(i, v[i].head<3>());
        }

        // set the color of first vertex to be red
        t.setColor(0, 255.0,  0.0,  0.0);
        // set the color of second vertex to be blue
        t.setColor(1, 0.0  ,255.0,  0.0);
        // set the color of third vertex to be green
        t.setColor(2, 0.0  ,  0.0,255.0);


        rasterize_wireframe(t);
    }
}

// draw (build triangle) -> rasterize_wireframe (draw 3 lines) -> draw_line (bresenham)
void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
    draw_line(t.c(), t.a());
    draw_line(t.c(), t.b());
    draw_line(t.b(), t.a());
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

void rst::rasterizer::set_rotation(const Eigen::Matrix4f& r) 
{
    rotation = r;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    // check if color bit is set
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) 
    {
        // fill the whole frame_buff with {0,0,0}
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0}); 
    }
    // check if depth bit is set
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) 
    {
        // fill the entire depth_buffer (z_buffer?) with "infinity" in IEEE 754
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
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;

    // use y * width _+ x to calculate the index in the framebuffer 
    auto ind = (height-point.y())*width + point.x();
    frame_buf[ind] = color;
}

