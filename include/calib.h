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
		float fx;
  		float fy;
  		float cx;
  		float cy;

		// Configurations
		int screenWidth;
		int screenHeight;
	 	size_t colorWidth;
		size_t colorHeight;

		/* Protected Functions */
		void computeHomography(bool placeholder);
	public:
		// Constructor and destructor
		Calib(cv::Mat CalibRGB, cv::Mat CalibDepth, libfreenect2::Freenect2Device::ColorCameraParams color, size_t ColorWidth = DEFAULT_COLOR_WIDTH, size_t ColorHeight = DEFAULT_COLOR_HEIGHT);
		~Calib();

		// Calibration methods
		void createLookup();
		void findTable();
		void createLookup(libfreenect2::Freenect2Device::ColorCameraParams color);
		void calibrate(bool placeholder=true, int inWidth = DEFAULT_SCREEN_WIDTH, int inHeight = DEFAULT_SCREEN_HEIGHT);

		// Setters
		void inputNewFrames(cv::Mat CalibRGB, cv::Mat CalibDepth);

		// Accessors
		cv::Mat getHomography();
		cv::Mat getLookUpX();
		cv::Mat getLookUpY();
		pcl::ModelCoefficients::Ptr getPlanePtr();
		bool isCalibrated();

		// Helper Functions
		void createCheckerboard(cv::Mat& checkerboard, int width, int height, int& numBlocksX, int& numBlocksY);
	
		// Conversion Utility Functions
		void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) const;
	    void convertPointXYZRGBToPoint2f(pcl::PointXYZRGB pointXYZRGB, cv::Point2f *colorPoint);
	};
}

#endif
