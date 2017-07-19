/*
 * XYMarkovModel.cpp
 *
 *  Created on: Jul 18, 2017
 *      Author: kevin
 */

#include "XYMarkovModel.h"

XYMarkovModel::XYMarkovModel() {
	int cols = ((GRID_WIDTH * GRID_SPACING) / (double)XY_MARKOV_RESOLUTION) + (XY_MARKOV_EDGE_PADDING*2);
	int rows = ((GRID_HEIGHT * GRID_SPACING) / (double)XY_MARKOV_RESOLUTION) + (XY_MARKOV_EDGE_PADDING*2);

	minX = -(GRID_WIDTH*GRID_SPACING)/2.0 - XY_MARKOV_EDGE_PADDING*XY_MARKOV_RESOLUTION;
	maxX = -minX;

	minY = -(GRID_HEIGHT*GRID_SPACING)/2.0 - XY_MARKOV_EDGE_PADDING*XY_MARKOV_RESOLUTION;
	maxY = -minY;

	ROS_DEBUG_STREAM("xy model size: " << rows*cols);

	this->P = cv::Mat(rows, cols, CV_64F); // create the model
	this->makeUniform();
	this->computeEntropy();
	ROS_DEBUG_STREAM("uniform entropy: " << this->entropy);
}

XYMarkovModel::~XYMarkovModel() {

}

void XYMarkovModel::normalize()
{
	double sum = 0;
	int size = this->P.rows * this->P.cols;
	for(int i = 0; i < size; i++)
	{
		sum += this->P(i);
	}

	if(fabs(sum - 1.0) < 0.00001)
	{
		ROS_DEBUG("model already normal");
	}
	else
	{
		for(int i = 0; i < size; i++)
		{
			this->P(i) = this->P(i) / sum;
		}
	}
}

void XYMarkovModel::makeUniform(){
	int size = this->P.rows * this->P.cols;

	double probability = 1 / (double)size;
	for(int i = 0; i < size; i++)
	{
		this->P(i) = probability;
	}
}

double XYMarkovModel::computeEntropy(){
	double sum = 0;
	double sumLog = 0;

	int size = this->P.rows * this->P.cols;
	for(int i = 0; i < size; i++)
	{
		sum += this->P(i);
	}

	for(int i = 0; i < size; i++)
	{
		sumLog += log(this->P(i));
	}

	this->entropy = -sum*sumLog;
	return this->entropy;
}

/*
 * makes sure that no probablility is too low
 */
void XYMarkovModel::checkAndFixMinimums()
{
	double sum = 0;
	int size = this->P.rows*this->P.cols;
	for(int i = 0; i < size; i++)
	{
		if(this->P(i) < XY_MARKOV_MINIMUM_PROBABLILTY)
		{
			this->P(i) = XY_MARKOV_MINIMUM_PROBABLILTY;
			sum += XY_MARKOV_MINIMUM_PROBABLILTY;
		}
		else
		{
			sum += this->P(i);
		}
	}

	for(int i = 0; i < size; i++)
	{
		this->P(i) /= sum;
	}
}

void XYMarkovModel::normalize(cv::Mat_<double>& model)
{
	double sum = 0;
	int size = model.rows * model.cols;
	for(int i = 0; i < size; i++)
	{
		sum += model(i);
	}

	if(fabs(sum - 1.0) < 0.00001)
	{
		ROS_DEBUG("model already normal");
	}
	else
	{
		for(int i = 0; i < size; i++)
		{
			model(i) = model(i) / sum;
		}
	}
}

int XYMarkovModel::XY2index(tf::Vector3 pos)
{

}

cv::Mat_<double> XYMarkovModel::computeSense(std::vector<BaseFrameHypothesis>& hyps){

}

cv::Mat XYMarkovModel::renderModel(cv::Mat_<double> model)
{
	this->normalize(model);

	cv::Mat mono = cv::Mat(model.rows, model.cols, CV_8U);

	int size = model.rows*model.cols;
	for(int i = 0; i < size; i++)
	{
		mono.at<uchar>(i) = (uchar)(model(i)*255);
	}

	return mono;
}

void XYMarkovModel::viewModel(cv::Mat_<double> model)
{
	cv::Mat mono = renderModel(model);
	cv::imshow("markov model", mono);
	cv::waitKey(30);
}
