#ifndef __CALIB_H__
#define __CALIB_H__

#include <libfreenect2/libfreenect2.hpp>
#include <string>
#include "pcl.h"
#include "opencv.h"
#include "settings.h"
#include "utilities.h"

namespace personalRobotics
{
	class Calib
	{
	protected:
		// Flags
		bool doneCalibrating;
		bool tablePlaneFound;
		bool homographyFound;

		// Config params
		int maxRansacIters;					//!< Maximum iterations for the RANSAC for estimating the table plane.
		float ransacMargin;					//!< Threshold to distinguish inliers from outliers. Any point is a inlier to a hypothesised plane if the distance between the plane and the point is less than this threshold.
		float distCutoff;					//!< Distance margin within which a point is consider to belong to the table plane.
		float radialThreshold;				//!< [DEPRECATED] The radius of the circle beyond which the depth pixel are ignored. The distances are computed after accounting for the intrinsics of the depth camera matrix(d = sqrt((X/Z)^2 + (Y/Z)^2)).
		float clusterTolerance;				//!< Threshold for including a point in a cluster. The point is included in the cluster if the distance between the point and its closest point in the cluster is less this threshold.
		int minClusterSize;					//!< Minimum size of the cluster below which the cluster is drpped. The size of the cluster is the number of points that belong to that cluster.
		int maxClusterSize;					//!< Maazimum size of the cluster beyond which the cluster is dropped for computaional purposes. The size of the cluster is the number of points that belong to that cluster.
		float minThreshold;					//!< Minimum depth(Z) below which the points are rejected.
		float maxThreshold;					//!< Maximum depth(Z) beyond which the points are rejected.
		float objectDifferenceThreshold;	//!< Threshold score below which two entities in consecutive frames are considered the same. The score is computed by calculateEntityDifferences() function.
		float objectMovementThreshold;		//!< Minimum distance the centroid of an entity needs to translate in two cosecutive frames for the entity to be considered non-static.

		// Calibration
		cv::Mat checkerboard;
		int numCheckerPtsX;
		int numCheckerPtsY;
		cv::Mat homography;
		cv::Point2f projPixelSize;
		cv::Point2f rgbPixelSize;
		pcl::ModelCoefficients::Ptr planePtr;
		cv::Mat calibRGB, calibDepth;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr calibPC;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclPtr;
		cv::Mat lookupX, lookupY;
		float fx;
  		float fy;
  		float cx;
  		float cy;

		// Configurations
		int screenWidth;
		int screenHeight;
	 	size_t colorWidth;
		size_t colorHeight;
		size_t depthWidth;
		size_t depthHeight;

		/* Protected Functions */
		void computeHomography(bool placeholder);
	public:
		// Constructor and destructor
		Calib(const cv::Mat CalibRGB, const cv::Mat CalibDepth, libfreenect2::Freenect2Device::ColorCameraParams color);
		~Calib();

		// Calibration methods
		void createLookup();
		void findTable();
		void createLookup(libfreenect2::Freenect2Device::ColorCameraParams color);
		void calibrate(bool placeholder=true, int inWidth = DEFAULT_SCREEN_WIDTH, int inHeight = DEFAULT_SCREEN_HEIGHT);

		// Setters
		void inputNewFrames(const cv::Mat CalibRGB, const cv::Mat CalibDepth);

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
	    void convertPointXYZToPoint2f(pcl::PointXYZ pointXYZ, cv::Point2f *colorPoint);
	};
}

#endif
