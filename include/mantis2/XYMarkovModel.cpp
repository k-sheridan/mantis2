/*
 * XYMarkovModel.cpp
 *
 *  Created on: Jul 18, 2017
 *      Author: kevin
 */

#include "XYMarkovModel.h"

XYMarkovModel::XYMarkovModel() {
	int cols = ((GRID_WIDTH * GRID_SPACING) / (double)XY_MARKOV_RESOLUTION) + (XY_MARKOV_EDGE_PADDING);
	int rows = ((GRID_HEIGHT * GRID_SPACING) / (double)XY_MARKOV_RESOLUTION) + (XY_MARKOV_EDGE_PADDING);

	ROS_DEBUG_STREAM("xy model size: " << rows*cols);

	this->P = cv::Mat(rows, cols, CV_64F); // create the model
	this->makeUniform();
	this->computeEntropy();
	ROS_DEBUG_STREAM("uniform entropy: " << this->entropy);
}

XYMarkovModel::~XYMarkovModel() {

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

