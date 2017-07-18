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

#define DO_RED_GREEN_TRICK false

// the inverse scaling factor for quad detection
#define QUAD_DETECTION_INV_SCALE 4

//minimum quad area in pixels
#define MINIMUM_CONTOUR_AREA 300

// canny thresh for quad detect
#define CANNY_HYSTERESIS 30

//gaussian blur for canny quad
#define CANNY_BLUR_SIGMA 2.0
#define CANNY_BLUR_KERNEL cv::Size(9, 9)

// polygon eps for quad approx
#define POLYGON_EPSILON 10


#endif /* MANTIS2_INCLUDE_MANTIS2_MANTIS2PARAMETERS_H_ */
