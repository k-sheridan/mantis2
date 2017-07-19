/*
 * HypothesisGeneration.h
 *
 *  Created on: Jun 12, 2017
 *      Author: pauvsi
 */

#ifndef MANTIS_INCLUDE_MANTIS3_HYPOTHESISGENERATION_H_
#define MANTIS_INCLUDE_MANTIS3_HYPOTHESISGENERATION_H_

#include "CoPlanarPoseEstimator.h"

#include "Mantis2Parameters.h"

/*
 * use the initial guess for rotation and z height
 * initial guess must contain the measurement pointer
 */
std::vector<BaseFrameHypothesis> generateAllXYShiftedHypotheses(BaseFrameHypothesis initial_guess)
{
	double minX = -(XY_MARKOV_EDGE_PADDING * XY_MARKOV_RESOLUTION) - ((GRID_WIDTH * GRID_SPACING) / 2.0)  + (XY_MARKOV_RESOLUTION / 2.0); // center of the node
	double maxX = -minX;

	double minY = -(XY_MARKOV_EDGE_PADDING * XY_MARKOV_RESOLUTION) - ((GRID_HEIGHT * GRID_SPACING) / 2.0) + (XY_MARKOV_RESOLUTION / 2.0);
	double maxY = -minY;

	std::vector<BaseFrameHypothesis> hyps;

	for(double x = minX; x < maxX; x += XY_MARKOV_RESOLUTION)
	{
		for(double y = minY; y < maxY; x += XY_MARKOV_RESOLUTION)
		{
			BaseFrameHypothesis bfhyp = initial_guess;

			//change the x an y of the origin
			bfhyp.getW2B().getOrigin().setX(x);
			bfhyp.getW2B().getOrigin().setY(y);

			hyps.push_back(bfhyp);
		}
	}

	ROS_DEBUG_STREAM("generated all possible hypotheses: " << hyps.size());

	return hyps;
}




#endif /* MANTIS_INCLUDE_MANTIS3_HYPOTHESISGENERATION_H_ */
