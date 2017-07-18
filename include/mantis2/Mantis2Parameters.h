/*
 * Mantis2Parameters.h
 *
 *  Created on: Jul 17, 2017
 *      Author: kevin
 */

#ifndef MANTIS2_INCLUDE_MANTIS2_MANTIS2PARAMETERS_H_
#define MANTIS2_INCLUDE_MANTIS2_MANTIS2PARAMETERS_H_

#define ULTRA_DEBUG true
#define SUPER_DEBUG true

#define BOTTOM_CAMERA_NS "bottom_camera"
#define FRONT_CAMERA_NS "front_camera"
#define BACK_CAMERA_NS "back_camera"

#define BASE_FRAME "base_link"
#define WORLD_FRAME "world"

#define GRID_WIDTH 20
#define GRID_HEIGHT 20
#define GRID_SPACING 1.0
#define LINE_THICKNESS 0.05

//how much to split the nodes by integer
#define TEST_POINT_SPLIT_COUNT 2

#define DO_RED_GREEN_TRICK false

// the inverse scaling factor for quad detection
#define QUAD_DETECTION_INV_SCALE 4

//use erode and dilate t fill gaps
#define FILL_CANNY_GAPS false

//minimum quad area in pixels
#define MINIMUM_CONTOUR_AREA 300

// canny thresh for quad detect
#define CANNY_HYSTERESIS 30

//gaussian blur for canny quad
#define CANNY_BLUR_SIGMA 2.0
#define CANNY_BLUR_KERNEL cv::Size(9, 9)

// polygon eps for quad approx
#define POLYGON_EPSILON 10


//ERROR CALCULATION
// search the surrounding 5 pixels to compute a points error
#define POINT_ERROR_KERNEL_SIZE 5

#define WHITE cv::Vec3i(255, 255, 255)
#define RED cv::Vec3i(0, 0, 255)
#define GREEN cv::Vec3i(0, 255, 0)


//MARKOV
#define XY_MARKOV_RESOLUTION 0.1
// howmany nodes to pad the edges by
#define XY_MARKOV_EDGE_PADDING 10
#define XY_MARKOV_MINIMUM_PROBABLILTY 1e-16





#endif /* MANTIS2_INCLUDE_MANTIS2_MANTIS2PARAMETERS_H_ */
