#include "calib.h"

// Constructor and Destructor
personalRobotics::Calib::Calib()
{

}

personalRobotics::Calib::Calib(size_t ColorWidth, size_t ColorHeight)
{
	colorWidth = ColorWidth;
	colorHeight = ColorHeight; 
}

personalRobotics::Calib::~Calib()
{

}

// Calibration methods
void personalRobotics::Calib::findTable()
{
	size_t numPoints = depthHeight*depthWidth;
	pclPtr->clear();
	pclPtr->resize(numPoints);
	size_t dstPoint = 0;
	for (size_t point = 0; point < numPoints; point++)
	{
		if (pointCloudPtr[point].Z > minThreshold && pointCloudPtr[point].Z < maxThreshold)
		{
			pclPtr.get()->points[dstPoint].x = pointCloudPtr[point].X;
			pclPtr.get()->points[dstPoint].y = pointCloudPtr[point].Y;
			pclPtr.get()->points[dstPoint].z = pointCloudPtr[point].Z;
			dstPoint++;
		}
	}
	pointCloudMutex.unlock();
	pclPtr->resize(dstPoint);

	// Segment out the plane using least squares and RANSAC
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxRansacIters);
	seg.setDistanceThreshold(ransacMargin);
	seg.setInputCloud(pclPtr);
	seg.segment(*inliers, *planePtr);

	// Change the sign of all number so that 'd' in ax+by+cz+d is always positive
	float signOfD = (planePtr->values[3]) / abs(planePtr->values[3]);
	planePtr->values[0] = planePtr->values[0] * signOfD;
	planePtr->values[1] = planePtr->values[1] * signOfD;
	planePtr->values[2] = planePtr->values[2] * signOfD;
	planePtr->values[3] = planePtr->values[3] * signOfD;

	std::cout << "findTablePlane() computed plane values: "
		  << planePtr->values[0] << " "
		  << planePtr->values[1] << " "
		  << planePtr->values[2] << " "
		  << planePtr->values[3] << std::endl;
	
	// Find pixel size
	CameraSpacePoint keyPoints[3];
	ColorSpacePoint projectedKeyPoints[3];
	keyPoints[0] = { 0, 0, (-1 * (planePtr->values[3] + planePtr->values[0] * 0 + planePtr->values[1] * 0) / planePtr->values[2]) };		//(0  , 0   ,z1)
	keyPoints[1] = { 0.1, 0, (-1 * (planePtr->values[3] + planePtr->values[0] * 0.1 + planePtr->values[1] * 0) / planePtr->values[2]) };	//(0.1, 0   ,z2)
	keyPoints[2] = { 0, 0.1, (-1 * (planePtr->values[3] + planePtr->values[0] * 0 + planePtr->values[1] * 0.1) / planePtr->values[2]) };	//(0  , 0.1 ,z3)
	coordinateMapperPtr->MapCameraPointsToColorSpace(3, keyPoints, 3, projectedKeyPoints);
	double delX = sqrt((projectedKeyPoints[1].X - projectedKeyPoints[0].X)*(projectedKeyPoints[1].X - projectedKeyPoints[0].X) + (projectedKeyPoints[1].Y - projectedKeyPoints[0].Y)*(projectedKeyPoints[1].Y - projectedKeyPoints[0].Y));
	double delY = sqrt((projectedKeyPoints[2].X - projectedKeyPoints[0].X)*(projectedKeyPoints[2].X - projectedKeyPoints[0].X) + (projectedKeyPoints[2].Y - projectedKeyPoints[0].Y)*(projectedKeyPoints[2].Y - projectedKeyPoints[0].Y));

	// Value of 0.1 used above is in meters, to get pixel size in mm, divide 100 by delX and delY
	rgbPixelSize.x = 100 / delX;
	rgbPixelSize.y = 100 / delY;

	std::cout << "findTablePlane() computed rgbPixelSize: (" << rgbPixelSize.x << ", " << rgbPixelSize.y <<")"<< std::endl;
	// Return
	return true;
}

/* computeHomography takes in an RGB image that contains a checkerboard pattern in the image and returns a homography */
cv::Mat personalRobotics::Calib::computeHomography(bool usePlaceHolder, cv::Mat colorImg)
{
	if(!usePlaceHolder)
	{
		cv::Mat tempColorImg = colorImg.clone();
		std::vector<cv::Point2f> detectedCorners, checkerboardCorners;
		bool foundCorners = findChessboardCorners(tempColorImg, cv::Size(numCheckerPtsX, numCheckerPtsY), detectedCorners, CV_CALIB_CB_ADAPTIVE_THRESH);
		std::cout << "Size of checkerboard being used in findHomography is: (" << checkerboard.cols << ", " << checkerboard.rows << ")" << std::endl;
		bool foundProjectedCorners = findChessboardCorners(checkerboard, cv::Size(numCheckerPtsX, numCheckerPtsY), checkerboardCorners, CV_CALIB_CB_ADAPTIVE_THRESH);

		if (foundCorners && foundProjectedCorners)
		{
			homography = cv::findHomography(detectedCorners, checkerboardCorners, CV_RANSAC);
			homographyFound.set(true);
		}
		else
		{
			std::cout << "Failed to find chessboard corners in the image.\n";
		}
	}
	else
	{
		std::cout << "Performing a placeholder calibration" << std::endl;
		homography = (cv::Mat_<double>(3, 3) << 1.7456, 0.0337, -837.4711, 0.0143, 1.7469, -331.6242, 0.0, 0.0, 1) * (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
		homographyFound.set(true);
	}
}

void personalRobotics::Calib::createLookup(Freenect2Device::ColorCameraParams color)
{
  const float fx = color.fx;
  const float fy = color.fy;
  const float cx = color.cx;
  const float cy = color.cy;
  float *it;

  lookupY = cv::Mat(1, colorHeight, CV_32F);
  it = lookupY.ptr<float>();
  for(size_t r = 0; r < colorHeight; ++r, ++it)
  {
    *it = (r - cy) * fy;
  }

  lookupX = cv::Mat(1, colorWidth, CV_32F);
  it = lookupX.ptr<float>();
  for(size_t c = 0; c < colorWidth; ++c, ++it)
  {
    *it = (c - cx) * fx;
  }
}

/* Calibration method for the kinect using a checkerboard. */
void personalRobotics::Calib::calibrate(bool usePlaceholder, int inWidth, int inHeight)
{
	
	// Setup
	screenWidth = inWidth;
	screenHeight = inHeight;
	numCheckerPtsX = 0;
	numCheckerPtsY = 0;
	createCheckerboard(checkerboard, screenWidth, screenHeight, numCheckerPtsX, numCheckerPtsY);
	
	// Calibration
	int attempts = 0;
	int maxAttempts = 5;
	std::cout << "Starting calibration with\nwidth: " << screenWidth << " height: " << screenHeight << " numXCorners: " << numCheckerPtsX << " numYCorners: " << numCheckerPtsY << std::endl;
	while (isCalibrating.get())
	{	    
	  if (!tablePlaneFound.get() && segmentor.isDepthAllocated.get()) {
	         std::cout << "calling findTable()" << std::endl;
			findTable();
	  }				

		if (!homographyFound.get() && segmentor.isColorAllocated.get())
		{
		  	    std::cout << "calling findHomography()" << std::endl;
			findHomography(usePlaceholder);
			attempts++;
			if (attempts > maxAttempts)
			{
				std::cout << "Calibration failed " << attempts-1 << " times. Performing a placeholder cailbration.\n";
				findHomography(true);
			}
		}
		

		if (tablePlaneFound.get() && homographyFound.get())
		{
			isCalibrating.set(false);
			segmentor.setHomography(homography, screenWidth, screenHeight);

			// Map size of rgb and projector pixels
			std::vector<cv::Point2f> rgbKeyPoints;
			std::vector<cv::Point2f> projKeyPoints;
			rgbKeyPoints.push_back(cv::Point2f(0, 0));
			rgbKeyPoints.push_back(cv::Point2f(10, 0));
			rgbKeyPoints.push_back(cv::Point2f(0, 10));
			cv::perspectiveTransform(rgbKeyPoints, projKeyPoints, homography);
			float delX = 10.f / cv::norm(projKeyPoints[1] - projKeyPoints[0]);
			float delY = 10.f / cv::norm(projKeyPoints[2] - projKeyPoints[0]);
			colorPixelSize = *(segmentor.getRGBpixelSize());
			projPixelSize.x = colorPixelSize.x * delX;
			projPixelSize.y = colorPixelSize.y * delY;
			std::cout << "Calibration complete. Projector pixel size is: (" << projPixelSize.x << "," << projPixelSize.y << ")\n";
		}
	}
}

//Accessors
/* Accessor function that returns the homography */
cv::Mat personalRobotics::Calib::getHomography()
{
	return homography;
}

/* Accessor function that returns the model coefficients of the plane */
pcl::ModelCoefficients::Ptr personalRobotics::Calib::getPlanePtr()
{
	return planePtr;
}

// Helper Functions
void personalRobotics::Calib::createCheckerboard(cv::Mat& checkerboard, int width, int height, int& numBlocksX, int& numBlocksY)
{
	checkerboard.create(height, width, CV_8UC1);
	int blockSize = (personalRobotics::gcd(height, width));
	numBlocksX = width / blockSize;
	numBlocksY = height / blockSize;
	while (numBlocksX < 6 || numBlocksY < 6)
	{
		if (blockSize % 2 != 0)
			std::cout << "Deadlock! check or change the resolution of display" << std::endl;
		else
		{
			blockSize /= 2;
			numBlocksX = width / blockSize;
			numBlocksY = height / blockSize;
		}
	}
	std::cout << "Creating checkerboard with width: " << width << " height: " << height << " block size: " << blockSize << std::endl;
	int color = 0;
	for (int row = 0; row<(numBlocksY); row++)
	{
		if (color == 0)
			color = 255;
		else
			color = 0;
		for (int col = 0; col<(numBlocksX); col++)
		{
			if (color == 0)
				color = 255;
			else
				color = 0;
			cv::Scalar cvcolor(color);
			cv::Point p1(col*blockSize, row*blockSize);
			cv::Point p2((col + 1)*blockSize, (row + 1)*blockSize);
			if (row == 0 || col == 0 || row == numBlocksY - 1 || col == numBlocksX - 1)
				rectangle(checkerboard, p1, p2, cv::Scalar(255), CV_FILLED);
			else
				rectangle(checkerboard, p1, p2, cvcolor, CV_FILLED);
		}
	}
	numBlocksX -= 3;
	numBlocksY -= 3;
}
