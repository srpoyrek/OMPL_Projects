///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Shreyas Poyrekar!!!!
//////////////////////////////////////

#include "CollisionChecking.h"


// Intersect the point (x,y) with the set of rectangles. If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
  for (int i = 0; i<obstacles.size(); i++)
  // Check every obstacles in given workspace.
  {
  // get xmax, ymax, xmin, ymin
    double xmin = obstacles[i].x;
    double ymin = obstacles[i].y;
    double xmax = xmin + obstacles[i].width;
    double ymax = ymin + obstacles[i].height;
    
    // if xmin <= x <=xmax and ymin <= y  <= ymax 
    if ((xmin <= x) && (x<= xmax) && (ymin <= y) && (y<= ymax)){
    	return false; // collides in the current obstacle
    	}
    }
   
    return true;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles. If the circle lies outside of all
// obstacles, return true.
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
    return false;
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles. If
// the square lies outside of all obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
    return false;
}

// Add any custom debug / development code here. This code will be executed
// instead of the statistics checker (Project2.cpp). Any code submitted here
// MUST compile, but will not be graded.
void debugMode(const std::vector<Robot> & /*robots*/, const std::vector<Rectangle> & /*obstacles*/,
               const std::vector<bool> & /*valid*/)
{
}
