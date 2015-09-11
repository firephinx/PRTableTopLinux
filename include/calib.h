#ifndef __CALIB_H__
#define __CALIB_H__

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

		// Configurations
		int screenWidth;
		int screenHeight;
	public:
		// Constructor and destructor
		Calib();
		~Calib();
		void resetCalibration();

		// Calibration methods
		void findTable();
		void computeHomography(bool placeholder);
		void calibrate(bool placeholder=true, int inWidth = DEFAULT_SCREEN_WIDTH, int inHeight = DEFAULT_SCREEN_HEIGHT);

		// Helper Functions
		void createCheckerboard(cv::Mat& checkerboard, int width, int height, int& numBlocksX, int& numBlocksY);
	};
}

#endif