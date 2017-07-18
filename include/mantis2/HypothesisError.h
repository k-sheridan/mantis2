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


void evaluateBaseFrameHypothesis(BaseFrameHypothesis& hyp, bool colorOnly = false){

}

void evaluateMantisImage(MantisImage& img, int& projections, double& error, bool colorOnly = false){

}

void evaluateWhitePoints(MantisImage& img, int& projections, double& error)
{

}

void evaluateGreenPoints(MantisImage& img, int& projections, double& error)
{

}

void evaluateRedPoints(MantisImage& img, int& projections, double& error)
{

}




#endif /* MANTIS2_INCLUDE_MANTIS2_HYPOTHESISERROR_H_ */
