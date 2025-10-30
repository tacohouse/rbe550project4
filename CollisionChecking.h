///////////////////////////////////////
// RBE 550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#ifndef COLLISION_CHECKING_H
#define COLLISION_CHECKING_H

#include <vector>

// Simple rectangle obstacle representation
struct Rectangle
{
    double x, y;      // Lower-left corner
    double width, height;
    
    Rectangle(double x_, double y_, double w_, double h_)
        : x(x_), y(y_), width(w_), height(h_) {}
    
    // Check if point (px, py) is inside rectangle
    bool contains(double px, double py) const
    {
        return px >= x && px <= x + width &&
               py >= y && py <= y + height;
    }
};

// Function declarations
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles);
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles);
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles);

#endif  // COLLISION_CHECKING_H