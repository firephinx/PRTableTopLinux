#ifndef __CALIB_H__
#define __CALIB_H__

#include "pcl.h"
#include <libfreenect2/libfreenect2.hpp>

namespace personalRobotics
{
	class Calib
	{
	protected:
		// Calibration
		cv::Mat checkerboard;
		int numCheckerPtsX;
		int numCheckerPtsY;
		cv::Mat homography;
		cv::Point2f projPixelSize;
		cv::Point2f colorPixelSize;
		pcl::ModelCoefficients::Ptr planePtr;
		cv::Mat calibRGB, calibDepth;
		pcl::PointCloud<pcl::PointXYZRGBa> calibPC;

		// Configurations
		int screenWidth;
		int screenHeight;
	 	size_t colorWidth;
		size_t colorHeight;

		/* Protected Functions */
		void computeHomography(bool placeholder);
		void createLookup(Freenect2Device::ColorCameraParams color);
	public:
		// Constructor and destructor
		Calib();
		Calib(size_t ColorWidth, size_t ColorHeight);
		~Calib();

		// Calibration methods
		void findTable();
		void calibrate(bool placeholder=true, int inWidth = DEFAULT_SCREEN_WIDTH, int inHeight = DEFAULT_SCREEN_HEIGHT);

		// Accessors
		cv::Mat getHomography();
		pcl::ModelCoefficients::Ptr getPlanePtr();
		bool isCalibrated();

		// Helper Functions
		void createCheckerboard(cv::Mat& checkerboard, int width, int height, int& numBlocksX, int& numBlocksY);
	};
}

#endif