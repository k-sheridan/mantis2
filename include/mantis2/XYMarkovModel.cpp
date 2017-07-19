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

	this->P = cv::Mat(rows, cols, CV_64F); // create the model

}

XYMarkovModel::~XYMarkovModel() {

}

void XYMarkovModel::makeUniform(){

}

double XYMarkovModel::computeEntropy(){

}

