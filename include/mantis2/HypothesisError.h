/*
 * HypothesisError.h
 *
 *  Created on: Jul 18, 2017
 *      Author: kevin
 */

#ifndef MANTIS2_INCLUDE_MANTIS2_HYPOTHESISERROR_H_
#define MANTIS2_INCLUDE_MANTIS2_HYPOTHESISERROR_H_


#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include "mantis2/Mantis2Parameters.h"

#include "mantis2/Mantis2Types.h"



void evaluateWhitePoints(MantisImage& img, int& projections, double& error)
{
	for(auto e : white_test_points)
	{
		error += img.computePointError(e, WHITE, projections);
	}
}

void evaluateGreenPoints(MantisImage& img, int& projections, double& error)
{
	for(auto e : green_test_points)
	{
		error += img.computePointError(e, GREEN, projections);
	}
}

void evaluateRedPoints(MantisImage& img, int& projections, double& error)
{
	for(auto e : red_test_points)
	{
		error += img.computePointError(e, RED, projections);
	}
}

void evaluateMantisImage(MantisImage& img, int& projections, double& error, bool colorOnly = false){
	evaluateRedPoints(img, projections, error);
	evaluateGreenPoints(img, projections, error);

	if(!colorOnly){evaluateWhitePoints(img, projections, error);}
}

void evaluateBaseFrameHypothesis(BaseFrameHypothesis& hyp, bool colorOnly = false){

	int projections = 0;
	double error = 0; // this will be per pixel

	// set up the local hypothesis for each mantis camera
	hyp.measurement->img1.thisHyp.setW2C(hyp.getW2B() * hyp.measurement->img1.b2c);
	hyp.measurement->img2.thisHyp.setW2C(hyp.getW2B() * hyp.measurement->img2.b2c);
	hyp.measurement->img3.thisHyp.setW2C(hyp.getW2B() * hyp.measurement->img3.b2c);

	evaluateMantisImage(hyp.measurement->img1, projections, error, colorOnly);
	evaluateMantisImage(hyp.measurement->img2, projections, error, colorOnly);
	evaluateMantisImage(hyp.measurement->img3, projections, error, colorOnly);

	ROS_DEBUG_STREAM("evaluated hypothesis. projection count: " << projections);


	//TODO add projection bias
	if(projections == 0)
	{
		hyp.error = DBL_MAX;
	}
	else
	{
		hyp.error = error / (double)projections;
	}

}

void visualizeHypothesis(BaseFrameHypothesis hyp, MantisImage img)
{
	cv::Mat final = img.img;

	img.thisHyp.setW2C(hyp.getW2B() * img.b2c);

	for(auto e : white_test_points)
	{
		cv::drawMarker(final, img.thisHyp.projectPoint(e, img.K), cv::Scalar(0, 255, 255), cv::MarkerTypes::MARKER_CROSS);
	}

	cv::imshow("hypo estimate", final);
	cv::waitKey(30);
}

#endif /* MANTIS2_INCLUDE_MANTIS2_HYPOTHESISERROR_H_ */
