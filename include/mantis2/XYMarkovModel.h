/*
 * XYMarkovModel.h
 *
 *  Created on: Jul 18, 2017
 *      Author: kevin
 */

#ifndef MANTIS2_INCLUDE_MANTIS2_XYMARKOVMODEL_H_
#define MANTIS2_INCLUDE_MANTIS2_XYMARKOVMODEL_H_

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

class XYMarkovModel {
public:

	cv::Mat_<double> P; // this is the model
	double entropy;

	double minX, minY, maxX, maxY;

	XYMarkovModel();
	virtual ~XYMarkovModel();

	void normalize();

	void normalize(cv::Mat_<double>& model);

	void makeUniform();

	void checkAndFixMinimums();

	int XY2index(tf::Vector3& pos);

	cv::Mat_<double> computeSense(std::vector<BaseFrameHypothesis>& hyps);

	cv::Mat_<double> multiply(cv::Mat_<double> P, cv::Mat_<double> sense);

	BaseFrameHypothesis getMostProbableHypothesis(std::vector<BaseFrameHypothesis>& hyps);

	tf::Vector3 getMostProbablePosition();

	void convolve(double dx, double dy);

	double computeEntropy();

	cv::Mat renderModel(cv::Mat_<double> model);
	void viewModel(cv::Mat_<double> model);
};

#endif /* MANTIS2_INCLUDE_MANTIS2_XYMARKOVMODEL_H_ */
