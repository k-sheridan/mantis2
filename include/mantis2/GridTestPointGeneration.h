/*
 * GridTestPointGeneration.h
 *
 *  Created on: Jul 18, 2017
 *      Author: kevin
 */

#ifndef MANTIS2_INCLUDE_GRIDTESTPOINTGENERATION_H_
#define MANTIS2_INCLUDE_GRIDTESTPOINTGENERATION_H_

#include <ros/ros.h>

#include "mantis2/Mantis2Parameters.h"


/*
 * generate the grid testpoints
 */
void computeTestPoints()
{

	double minX = -(GRID_WIDTH * GRID_SPACING) / 2.0;
	double maxX = -minX;

	double minY = -(GRID_HEIGHT * GRID_SPACING) / 2.0;
	double maxY = -minY;

	double dr = GRID_SPACING * (1 / (double)TEST_POINT_SPLIT_COUNT);

	int xline = 0;
	for(double x = minX; x <= maxX; x += dr)
	{

		int yline = 0;

		if(xline % TEST_POINT_SPLIT_COUNT) // if we are NOT on a y line
		{
			for(double y = minY; y <= maxY; y += GRID_SPACING)
			{

				if(yline == 0) // red
				{
					red_test_points.push_back(tf::Vector3(x, y, 0.0));
					ROS_DEBUG_STREAM("RED: " << x << ", " << y);
				}
				else if(yline == GRID_HEIGHT)
				{
					green_test_points.push_back(tf::Vector3(x, y, 0.0));
					ROS_DEBUG_STREAM("GREEN: " << x << ", " << y);
				}
				else{
					white_test_points.push_back(tf::Vector3(x, y, 0.0));
					ROS_DEBUG_STREAM("WHITE: " << x << ", " << y);
				}

				yline++;
			}
		}
		else // if we are
		{
			for(double y = minY; y <= maxY; y += dr)
			{

				if(yline == 0) // red
				{
					red_test_points.push_back(tf::Vector3(x, y, 0.0));
					ROS_DEBUG_STREAM("RED: " << x << ", " << y);
				}
				else if(yline == GRID_HEIGHT * TEST_POINT_SPLIT_COUNT)
				{
					green_test_points.push_back(tf::Vector3(x, y, 0.0));
					ROS_DEBUG_STREAM("GREEN: " << x << ", " << y);
				}
				else{
					white_test_points.push_back(tf::Vector3(x, y, 0.0));
					ROS_DEBUG_STREAM("WHITE: " << x << ", " << y);
				}

				yline++;
			}
		}

		xline++;
	}
}


#endif /* MANTIS2_INCLUDE_GRIDTESTPOINTGENERATION_H_ */
