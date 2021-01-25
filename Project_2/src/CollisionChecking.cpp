///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Shreyas Poyrekar!!!!
//////////////////////////////////////

#include "CollisionChecking.h"
#include <math.h> 

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


double EuclideanNorm(double x1, double y1, double x2, double y2) 
{ 
    // Calculating distance
    return pow(x2 - x1, 2) + pow(y2 - y1, 2);  
} 

// Intersect a circle with center (x,y) and given radius with the set of rectangles. If the circle lies outside of all
// obstacles, return true.
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
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
    double r = radius;
   //  ((x min − r ≤ x ≤ x max + r) and (y min ≤ y ≤ y max ))
    if ((((xmin-r) <= x) && (x <= (xmax+r))) && ((ymin <= y)&&(y <= ymax)))
    {
    	return false; // collides in the current obstacles
    	}
    //((x min ≤ x ≤ x max ) and (y min − r ≤ y ≤ y max + r)) or
    else if (((xmin <=  x) && (x <= xmax)) && (((ymin-r) <= y)&&(y <= (ymax+r))))
    {
    	return false;// collides in the current obstacle
    	}
    	// all four vertexs collisions checking for the circle
    else if (EuclideanNorm(x,y,xmin,ymin)<= r * r)
    {
    	return false;
	}
     else if (EuclideanNorm(x,y,xmin,ymax)<= r * r)
    {
    	return false;
	}
	else if (EuclideanNorm(x,y,xmax,ymin)<= r * r)
    {
    	return false;
	}
	else if (EuclideanNorm(x,y,xmax,ymax)<= r * r)
    {
    	return false;
	}
    }
	
	return true;    
}

// return x rotated co-ordinate
double rotateX(double x, double y, double theta)
{

return ((x*cos(theta)) - (y*sin(theta)));

}

// return y rotated co-ordinate
double rotateY(double x, double y, double theta)
{

return ((x*sin(theta)) + (y*cos(theta)));

}
 
  
// Given three colinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool onSegment(Point p, Point q, Point r) 
{ 
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && 
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y)) 
       return true; 
  
    return false; 
} 
 


// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles. If
// the square lies outside of all obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{	
	// TODO: IMPLEMENT ME!! 
	
	//obstacle vertex variables
	double axo,bxo,cxo,dxo,ayo,byo,cyo,dyo,axor,bxor,cxor,dxor,ayor,byor,cyor,dyor;
	
	Point p1,q1,p2,q2,p3,q3,p4,q4,p5,q5,p6,q6,p7,q7,p8,q8;
	Rectangle squareRobot;
	std::vector<Rectangle> squareRobotVector;
	
	//square robot verters at workspace origin 
	double ax =  - sideLength/2;
	double bx =  - sideLength/2;
	double cx =  + sideLength/2;
	double dx =  + sideLength/2;
	double ay =  - sideLength/2;
	double by =  + sideLength/2;
	double cy =  + sideLength/2;
	double dy =  - sideLength/2;
	
	// non rotated square robot as arectangle struct
	squareRobot.x = x + ax;
	squareRobot.y = y + ay;
	squareRobot.width = sideLength;
	squareRobot.height = sideLength;
	squareRobotVector.push_back(squareRobot);
	
	// rotated at angle theta co-ordinates
	double axr = x + rotateX(ax,ay,theta);
	double ayr = y + rotateY(ax,ay,theta);
	double bxr = x + rotateX(bx,by,theta);
	double byr = y + rotateY(bx,by,theta);
	double cxr = x + rotateX(cx,cy,theta);
	double cyr = y + rotateY(cx,cy,theta);
	double dxr = x + rotateX(dx,dy,theta);
	double dyr = y + rotateY(dx,dy,theta);
				
	p1.x = axr; p1.y = ayr; q1.x = bxr; q1.y = byr;
	p2.x = bxr; p2.y = byr; q2.x = cxr; q1.y = cyr;
	p3.x = cxr; p3.y = cyr; q3.x = dxr; q1.y = dyr;
	p4.x = dxr; p4.y = dyr; q4.x = axr; q1.y = ayr;	
	
	//fast checking if the corners of the square are in any of the obstacles
	if (!isValidPoint(axr,ayr,obstacles))
	{	
		return false;	
	}
	else if (!isValidPoint(bxr,byr,obstacles))
	{	
		return false;	
	}
	else if (!isValidPoint(cxr,cyr,obstacles))
	{	
		return false;	
	}
	else if (!isValidPoint(dxr,dyr,obstacles))
	{	
		return false;	
	}
	else if (!isValidPoint(x,y,obstacles))
	{	
		return false;	
	}
	 
	// check if the obstacle vertex are inside the square
	for (int i=0; i<obstacles.size();i++){
		// get the co-prdinates of the vertex's of the each obstacle
		axo = obstacles[i].x;
		ayo = obstacles[i].y;
		bxo = axo;
		byo = ayo + obstacles[i].height;
		cxo = axo + obstacles[i].width;
		cyo = byo;
		dxo = cxo;
		dyo = ayo;
		
		
		// obstacle vertexs as point structure variables
		p5.x = axo; p5.y = ayo; q5.x = bxo; q5.y = byo;
		p6.x = bxo; p6.y = byo; q6.x = cxo; q6.y = cyo;
		p7.x = cxo; p7.y = cyo; q7.x = dxo; q7.y = dyo;
		p8.x = dxo; p8.y = dyo; q8.x = axo; q8.y = ayo;	
		
		// rotate the vertexs by theta anticlockwise w.r.t to the square centroid
		axor = x + rotateX(axo - x,ayo - y,-theta);
	 	ayor = y + rotateY(axo - x,ayo - y,-theta);
	 	bxor = x + rotateX(bxo - x,byo - y,-theta);
	 	byor = y + rotateY(bxo - x,byo - y,-theta);
	 	cxor = x + rotateX(cxo - x,cyo - y,-theta);	
	 	cyor = y + rotateY(cxo - x,cyo - y,-theta);
	 	dxor = x + rotateX(dxo - x,dyo - y,-theta);
	 	dyor = y + rotateY(dxo - x,dyo - y,-theta);
	 	
	 	// check if each corner of the obstacles is not inside the square
	 	if (!isValidPoint(axor,ayor,squareRobotVector))
		{	
			return false;	
		}
		else if (!isValidPoint(bxor,byor,squareRobotVector))
		{	
			return false;	
		}
		else if (!isValidPoint(cxor,cyor,squareRobotVector))
		{	
			return false;	
		}
		else if (!isValidPoint(dxor,dyor,squareRobotVector))
		{	
			return false;	
		}
		
	}

    
    return true;
}

// Add any custom debug / development code here. This code will be executed
// instead of the statistics checker (Project2.cpp). Any code submitted here
// MUST compile, but will not be graded.
void debugMode(const std::vector<Robot> & /*robots*/, const std::vector<Rectangle> & /*obstacles*/,
               const std::vector<bool> & /*valid*/)
{
}
