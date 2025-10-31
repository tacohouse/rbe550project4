///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Luis Alzamora Josh Ethan
//////////////////////////////////////

#include "CollisionChecking.h"
#include <cmath>

bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    // Check if point (x, y) is inside any obstacle
    for (const auto &obs : obstacles)
    {
        if (x >= obs.x && x <= obs.x + obs.width &&
            y >= obs.y && y <= obs.y + obs.height)
        {
            return false;  // Point is inside obstacle
        }
    }
    return true;  // Point is free
}

bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    // Check if circle intersects any obstacle
    // Use conservative approximation: check if circle center + radius penetrates rectangle
    for (const auto &obs : obstacles)
    {
        // Find closest point on rectangle to circle center
        double closestX = std::max(obs.x, std::min(x, obs.x + obs.width));
        double closestY = std::max(obs.y, std::min(y, obs.y + obs.height));
        
        // Calculate distance from circle center to closest point
        double distX = x - closestX;
        double distY = y - closestY;
        double distSquared = distX * distX + distY * distY;
        
        // Check if distance is less than radius (collision)
        if (distSquared < radius * radius)
        {
            return false;
        }
    }
    return true;
}

bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{
    // Get the four corners of the square
    double halfSide = sideLength / 2.0;
    double corners[4][2];
    
    // Corners in local frame, then rotate by theta
    double cosT = cos(theta);
    double sinT = sin(theta);
    
    // Corner offsets in local frame
    double localCorners[4][2] = {
        {-halfSide, -halfSide},
        { halfSide, -halfSide},
        { halfSide,  halfSide},
        {-halfSide,  halfSide}
    };
    
    // Transform to world frame
    for (int i = 0; i < 4; i++)
    {
        corners[i][0] = x + localCorners[i][0] * cosT - localCorners[i][1] * sinT;
        corners[i][1] = y + localCorners[i][0] * sinT + localCorners[i][1] * cosT;
    }
    
    // Check if any corner is inside an obstacle
    for (int i = 0; i < 4; i++)
    {
        if (!isValidPoint(corners[i][0], corners[i][1], obstacles))
        {
            return false;
        }
    }
    
    // Check if any obstacle corner is inside the square (more thorough check)
    for (const auto &obs : obstacles)
    {
        double obsCorners[4][2] = {
            {obs.x, obs.y},
            {obs.x + obs.width, obs.y},
            {obs.x + obs.width, obs.y + obs.height},
            {obs.x, obs.y + obs.height}
        };
        
        for (int i = 0; i < 4; i++)
        {
            // Simple check: if obstacle corner is very close to square center
            double dx = obsCorners[i][0] - x;
            double dy = obsCorners[i][1] - y;
            
            // Rotate to square's local frame
            double localX = dx * cosT + dy * sinT;
            double localY = -dx * sinT + dy * cosT;
            
            if (fabs(localX) <= halfSide && fabs(localY) <= halfSide)
            {
                return false;
            }
        }
    }
    
    return true;
}