#ifndef __CALIB_H__
#define __CALIB_H__

#include "pcl.h"
#include <libfreenect2/libfreenect2.hpp>

namespace personalRobotics
{
	class Calib
	{
	protected:
		// Flags
		bool doneCalibrating;
		bool tablePlaneFound;
		bool homographyFound;

		// Calibration
		cv::Mat checkerboard;
		int numCheckerPtsX;
		int numCheckerPtsY;
		cv::Mat homography;
		cv::Point2f projPixelSize;
		cv::Point2f colorPixelSize;
		pcl::ModelCoefficients::Ptr planePtr;
		cv::Mat calibRGB, calibDepth;
		pcl::PointCloud<pcl::PointXYZRGB> calibPC;
		cv::Mat lookupX, lookUpY;

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
		Calib(cv::Mat CalibRGB, cv::Mat CalibDepth, size_t ColorWidth = DEFAULT_COLOR_WIDTH, size_t ColorHeight = DEFAULT_COLOR_HEIGHT);
		~Calib();

		// Calibration methods
		void findTable();
		void createLookup(Freenect2Device::ColorCameraParams color);
		void calibrate(bool placeholder=true, int inWidth = DEFAULT_SCREEN_WIDTH, int inHeight = DEFAULT_SCREEN_HEIGHT);

		// Setters
		void setCalibCloud(pcl::PointCloud<pcl::PointXYZRGB> calibRGBPointCloud);

		// Accessors
		cv::Mat getHomography();
		cv::Mat getLookUpX();
		cv::Mat getLookUpY();
		pcl::ModelCoefficients::Ptr getPlanePtr();
		bool isCalibrated();

		// Helper Functions
		void createCheckerboard(cv::Mat& checkerboard, int width, int height, int& numBlocksX, int& numBlocksY);
	};
}

#endif