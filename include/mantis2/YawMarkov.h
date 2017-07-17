/*
 * Author: Logesh Roshan Ramadoss
 */
#include <math.h>
#include <array>
#include<float.h>
#include <opencv2/plot.hpp>
#include <opencv2/highgui.hpp>
#include "ros/ros.h"

#include "Mantis2Types.h"
using namespace cv;

#define DEGREES 360
typedef std::array<double, 360> markovPlane;

/*
 * TODO: Make sure yaw coming from hypothesis is correct
 */


class YawMarkovModel{
private:
	markovPlane p;
	double calculateWeight(double x, double mu, double stddev, double y);
	void normalize();
	void normalize(markovPlane& input);
	void normalize(markovPlane input, markovPlane& output);


public:
	YawMarkovModel(markovPlane inp);
	YawMarkovModel(Hypothesis hyp);
	void updateWeights(markovPlane& yaw, double stddev);
	void plotMarkovPlane();
	void plotMarkovPlane(markovPlane yaw);
	void senseFusion(markovPlane sense);
	void senseFusion(Hypothesis hypothesis);
	void updateHypothesis(std::vector<Hypothesis>& hypothesis);
	void convolve(double dTheta, double dt);
	markovPlane getDistrbution();
	double getYaw();

};




/*
 * Returns a weighted value of gaussian distribution
 */
double calculateWeight(double x, double mu, double stddev, double y)
{
	return (y/(stddev*sqrt(2*M_PI)))*exp(-((x-mu)*(x-mu)/(2*stddev*stddev)));
}

void normalize(markovPlane& input)
{
	double sum = 0.0;
	for(int i=0; i<input.size(); ++i)
	{
		sum += input[i];
	}

	for(int i=0; i<input.size(); ++i)
	{
		input[i] = input[i] / sum;
	}
}

void normalize(markovPlane input, markovPlane& output)
{
	double sum = 0.0;
	for(int i=0; i<input.size(); ++i)
	{
		sum += input[i];
	}

	for(int i=0; i<input.size(); ++i)
	{
		output[i] = (input[i] / sum);
	}

}



void updateWeights(markovPlane& yaw, double stddev)
{
	markovPlane aux;
	double max, temp;
	int diff;
	for(int i=0; i<aux.size(); ++i)
	{
		max = 0.0;
		diff = (DEGREES/2)+i;

		/*when i < 180
		 * 	Go right and find new possible weight, then go left(wrap around) and find weight.
		 *  Going left since that displacement is small
		 */
		if(i <= DEGREES/2)
		{
			for(int j=0; j<diff; ++j)
			{
				//if(j == i) continue;
				max += calculateWeight((double)i, (double)j, stddev, yaw[j]);
			}

			for(int j=diff; j<DEGREES; ++j)
			{
				//DEGREES - j is basically looking at the array in the opposite direction (-1, -2 ....)
				max += calculateWeight((double)i, (double)(-(DEGREES-j)), stddev, yaw[j]);
			}
		}
		// when i > 180
		else
		{
			for(int j=i; j<diff; ++j)
			{
				//if(j==i) continue;
				max += calculateWeight((double)i, (double)j, stddev, yaw[j%DEGREES]);
			}

			for(int j=(diff%DEGREES); j<i; ++j)
			{
				max += calculateWeight((double)i, (double)j, stddev, yaw[j]);
			}
		}

		aux[i] = max;
	}

	//normalize and copy
	normalize(aux, yaw);
}


void plotMarkovPlane(markovPlane yaw)
{
	cv::Mat data(DEGREES, 1, CV_64F);
	for(int i=0; i<DEGREES; ++i)
	{
		data.at<double>(i, 0) = yaw[i];
	}

	cv::Mat plot_result;
	cv::Ptr<cv::plot::Plot2d> plot = plot::createPlot2d(data);
	plot->setPlotBackgroundColor( Scalar( 50, 50, 50 ) ); // i think it is not implemented yet
	plot->setPlotLineColor( Scalar( 50, 50, 255 ) );
	plot->render( plot_result );

	imshow( "plot", plot_result );
	waitKey(50);

}

void mergeMarkovPlanes(markovPlane& history, markovPlane newYaw)
{
	ROS_DEBUG_STREAM("merging");
	double newMax = sqrt(DBL_MAX);

	for(int i=0; i<history.size(); ++i)
	{
		//INCREASE PRECISION TO PREVENT IT GOES BELOW DBL_MIN.
		history[i] = history[i]*newMax;
		newYaw[i] = newYaw[i]*newMax;

	}


	for(int i=0; i<history.size(); ++i)
	{
		history[i] *= newYaw[i];
	}

	normalize(history);
	ROS_DEBUG_STREAM("merge done");
}



void updateHypothesis(markovPlane yaw, std::vector<Hypothesis>& hypothesis)
{
	double r, p, y;
	for(int i=0; i<hypothesis.size(); ++i)
	{
		hypothesis[i].getW2C().getBasis().getRPY(r,p,y);
		//TODO: the 1/yaw[] could be extremely large if yaw[] is extremely small. make sure it works
		hypothesis[i].error = hypothesis[i].error * 1/(yaw[(int)y]);
	}
}

