#ifndef PLUGINLIB_TUTORIAL_POLYGON_PLUGINS_H
#define PLUGINLIB_TUTORIAL_POLYGON_PLUGINS_H

#include <pluginlib_tutorial/polygon_base.h>
#include <cmath>

namespace polygon_plugins
{
class Triangle : public polygon_base::RegularPolygon
{
public:
    Triangle():side_length_(){}

    void intialize(double side_length_)
    {
        side_length_ = side_length_; 
    }
}
}
