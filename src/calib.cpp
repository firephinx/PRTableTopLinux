#include "calib.h"

// Constructor and Destructor
personalRobotics::Calib::Calib(cv::Mat CalibRGB, cv::Mat CalibDepth, libfreenect2::Freenect2Device::ColorCameraParams color)
{
	CalibRGB.copyTo(calibRGB);
	CalibDepth.copyTo(calibDepth);
	colorWidth = DEFAULT_COLOR_WIDTH;
	colorHeight = DEFAULT_COLOR_HEIGHT; 
	depthWidth = DEFAULT_DEPTH_WIDTH;
	depthHeight = DEFAULT_DEPTH_HEIGHT; 
	minThreshold = DEFAULT_MIN_DEPTH_LIMIT;
	maxThreshold = DEFAULT_MAX_DEPTH_LIMIT;
	maxRansacIters = DEFAULT_MAX_RANSAC_ITERATIONS;
	ransacMargin = DEFAULT_DEPTH_MARGIN;
	distCutoff = DEFAULT_DISTANCE_CUTOFF;
	radialThreshold = (DEFAULT_RADIAL_CUTOFF)*(DEFAULT_RADIAL_CUTOFF);
	clusterTolerance = DEFAULT_CLUSTER_TOLERANCE;
	minClusterSize = DEFAULT_MINIMUM_CLUSTER_SIZE;
	maxClusterSize = DEFAULT_MAXIMUM_CLUSTER_SIZE;
	objectDifferenceThreshold = OBJECT_DIFFERENCE_THRESHOLD;
	objectMovementThreshold = OBJECT_MOVEMENT_THRESHOLD;
	fx = color.fx;
  	fy = color.fy;
  	cx = color.cx;
  	cy = color.cy;
	doneCalibrating = false;
	tablePlaneFound = false;
	homographyFound = false;
	pclPtr = (pcl::PointCloud<pcl::PointXYZ>::Ptr) new pcl::PointCloud<pcl::PointXYZ>;
	createLookup();
	createCloud(CalibDepth, CalibRGB, &calibPC);
}

personalRobotics::Calib::~Calib()
{

}

// Calibration methods
void personalRobotics::Calib::findTable()
{
	size_t numCalibPoints = calibPC.points.size();
	size_t numPoints = depthHeight*depthWidth;
	pclPtr->clear();
	pclPtr->width = depthWidth;
	pclPtr->height = depthHeight;
	pclPtr->resize(numPoints);
	size_t dstPoint = 0;
	for (size_t point = 0; point < numCalibPoints; point++)
	{
		if (calibPC[point].z > minThreshold && calibPC[point].z < maxThreshold)
		{
			pclPtr->points[dstPoint].x = calibPC[point].x;
			pclPtr->points[dstPoint].y = calibPC[point].y;
			pclPtr->points[dstPoint].z = calibPC[point].z;
			dstPoint++;
		}
	}
	pclPtr->resize(dstPoint);
	
	std::cout << "checkpoint 3 " << numPoints << " numCalibPoints =  " << numCalibPoints << " " << dstPoint << std::endl;
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
	std::cout << "checkpoint 4" << std::endl;
	
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
	pcl::PointXYZ keyPoints[3];
	cv::Point2f *projectedKeyPoints[3];
	keyPoints[0] = { 0, 0, (-1 * (planePtr->values[3] + planePtr->values[0] * 0 + planePtr->values[1] * 0) / planePtr->values[2]) };		//(0  , 0   ,z1)
	keyPoints[1] = { 0.1, 0, (-1 * (planePtr->values[3] + planePtr->values[0] * 0.1 + planePtr->values[1] * 0) / planePtr->values[2]) };	//(0.1, 0   ,z2)
	keyPoints[2] = { 0, 0.1, (-1 * (planePtr->values[3] + planePtr->values[0] * 0 + planePtr->values[1] * 0.1) / planePtr->values[2]) };	//(0  , 0.1 ,z3)
	for(int i = 0; i < 3; i++)
	{
		personalRobotics::Calib::convertPointXYZToPoint2f(keyPoints[i], projectedKeyPoints[i]);
	}
	double delX = sqrt((projectedKeyPoints[1]->x - projectedKeyPoints[0]->x)*(projectedKeyPoints[1]->x - projectedKeyPoints[0]->x) + (projectedKeyPoints[1]->y - projectedKeyPoints[0]->y)*(projectedKeyPoints[1]->y - projectedKeyPoints[0]->y));
	double delY = sqrt((projectedKeyPoints[2]->x - projectedKeyPoints[0]->x)*(projectedKeyPoints[2]->x - projectedKeyPoints[0]->x) + (projectedKeyPoints[2]->y - projectedKeyPoints[0]->y)*(projectedKeyPoints[2]->y - projectedKeyPoints[0]->y));

	// Value of 0.1 used above is in meters, to get pixel size in mm, divide 100 by delX and delY
	rgbPixelSize.x = 100 / delX;
	rgbPixelSize.y = 100 / delY;

	std::cout << "findTablePlane() computed rgbPixelSize: (" << rgbPixelSize.x << ", " << rgbPixelSize.y <<")"<< std::endl;
	// Return
	return;
}

/* computeHomography takes in an RGB image that contains a checkerboard pattern in the image and returns a homography */
void personalRobotics::Calib::computeHomography(bool usePlaceHolder)
{
	if(!usePlaceHolder)
	{
		cv::Mat tempColorImg = calibRGB.clone();
		std::vector<cv::Point2f> detectedCorners, checkerboardCorners;
		bool foundCorners = findChessboardCorners(tempColorImg, cv::Size(numCheckerPtsX, numCheckerPtsY), detectedCorners, CV_CALIB_CB_ADAPTIVE_THRESH);
		std::cout << "Size of checkerboard being used in findHomography is: (" << checkerboard.cols << ", " << checkerboard.rows << ")" << std::endl;
		bool foundProjectedCorners = findChessboardCorners(checkerboard, cv::Size(numCheckerPtsX, numCheckerPtsY), checkerboardCorners, CV_CALIB_CB_ADAPTIVE_THRESH);

		if (foundCorners && foundProjectedCorners)
		{
			homography = cv::findHomography(detectedCorners, checkerboardCorners, CV_RANSAC);
			homographyFound = true;
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
		homographyFound = true;
	}
}

void personalRobotics::Calib::createLookup()
{
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
	while (!doneCalibrating)
	{	    
		if (!tablePlaneFound) {
	    	std::cout << "calling findTable()" << std::endl;
			findTable();
	  	}				

		if (!homographyFound)
		{
		  	std::cout << "calling findHomography()" << std::endl;
			computeHomography(usePlaceholder);
			attempts++;
			if (attempts > maxAttempts)
			{
				std::cout << "Calibration failed " << attempts-1 << " times. Performing a placeholder cailbration.\n";
				computeHomography(true);
			}
		}

		if (tablePlaneFound && homographyFound)
		{
			doneCalibrating = true;

			// Map size of rgb and projector pixels
			std::vector<cv::Point2f> rgbKeyPoints;
			std::vector<cv::Point2f> projKeyPoints;
			rgbKeyPoints.push_back(cv::Point2f(0, 0));
			rgbKeyPoints.push_back(cv::Point2f(10, 0));
			rgbKeyPoints.push_back(cv::Point2f(0, 10));
			cv::perspectiveTransform(rgbKeyPoints, projKeyPoints, homography);
			/*float delX = 10.f / cv::norm(projKeyPoints[1] - projKeyPoints[0]);
			float delY = 10.f / cv::norm(projKeyPoints[2] - projKeyPoints[0]);
			colorPixelSize = *(segmentor.getRGBpixelSize());
			projPixelSize.x = colorPixelSize.x * delX;
			projPixelSize.y = colorPixelSize.y * delY;
			std::cout << "Calibration complete. Projector pixel size is: (" << projPixelSize.x << "," << projPixelSize.y << ")\n";*/
		}
	}
}

//Setters
void personalRobotics::Calib::inputNewFrames(cv::Mat CalibRGB, cv::Mat CalibDepth)
{
	calibRGB = CalibRGB;
	calibDepth = CalibDepth;
}

//Accessors
/* Accessor function that returns the homography */
cv::Mat personalRobotics::Calib::getHomography()
{
	return homography;
}

cv::Mat personalRobotics::Calib::getLookUpX()
{
	return lookupX;
}
cv::Mat personalRobotics::Calib::getLookUpY()
{
	return lookupY;
}

/* Accessor function that returns the model coefficients of the plane */
pcl::ModelCoefficients::Ptr personalRobotics::Calib::getPlanePtr()
{
	return planePtr;
}

bool personalRobotics::Calib::isCalibrated()
{
	return doneCalibrating;
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

void personalRobotics::Calib::createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) const
{
  const float badPoint = std::numeric_limits<float>::quiet_NaN();

  printf("num depth rows = %d, num depth cols = %d", depth.rows, depth.cols);
  #pragma omp parallel for
  for(int r = 0; r < depth.rows; ++r)
  {
    pcl::PointXYZRGB *itP = &cloud->points[r * depth.cols];
    const uint16_t *itD = depth.ptr<uint16_t>(r);
    const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
    const float y = lookupY.at<float>(0, r);
    const float *itX = lookupX.ptr<float>();

    for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
    {
      register const float depthValue = *itD / 1000.0f;
      // Check for invalid measurements
      if(isnan(depthValue) || depthValue <= 0.001)
      {
        // not valid
        itP->x = itP->y = itP->z = badPoint;
        itP->rgba = 0;
        continue;
      }
      itP->z = depthValue;
      itP->x = *itX * depthValue;
      itP->y = y * depthValue;
      itP->b = itC->val[0];
      itP->g = itC->val[1];
      itP->r = itC->val[2];
    }
  }
}

void personalRobotics::Calib::convertPointXYZToPoint2f(pcl::PointXYZ pointXYZ, cv::Point2f *colorPoint)
{
  float x = pointXYZ.x;
  float y = pointXYZ.y;
  float z = pointXYZ.z;
  float newX = ((x/z)/fx) + cx;
  float newY = ((y/z)/fy) + cy;
  cv::Point2f colorXY(newX, newY);
  *colorPoint = colorXY;  
}

/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr personalRobotics::convertRegisteredDepthToXYZRGBPointCloud (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);

  cloud->height = depth_height_;
  cloud->width = depth_width_;
  cloud->is_dense = false;

  cloud->points.resize (cloud->height * cloud->width);

  //I inserted 525 as Julius Kammerl said as focal length
  register float constant = 1.0f / device_->getDepthFocalLength (depth_width_);

  //the frame id is completely ignored
  if (device_->isDepthRegistered ())
    cloud->header.frame_id = rgb_frame_id_;
  else
    cloud->header.frame_id = depth_frame_id_;

  register int centerX = (cloud->width >> 1);
  int centerY = (cloud->height >> 1);

  //I also ignore invalid values completely
  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  //this section is used to get the depth data array, I replaced it with my code (and you did it with your code)
  register const unsigned short* depth_map = depth_image->getDepthMetaData ().Data ();
  if (depth_image->getWidth() != depth_width_ || depth_image->getHeight () != depth_height_)
  {
    static unsigned buffer_size = 0;
    static boost::shared_array<unsigned short> depth_buffer (0);

    if (buffer_size < depth_width_ * depth_height_)
    {
      buffer_size = depth_width_ * depth_height_;
      depth_buffer.reset (new unsigned short [buffer_size]);
    }
    depth_image->fillDepthImageRaw (depth_width_, depth_height_, depth_buffer.get ());
    depth_map = depth_buffer.get ();
  }

  //these for loops are mostly used as they are
  register int depth_idx = 0;
  for (int v = -centerY; v < centerY; ++v)
  {
    for (register int u = -centerX; u < centerX; ++u, ++depth_idx)
    {
      pcl::PointXYZ& pt = cloud->points[depth_idx];
     
      //This part is used for invalid measurements, I removed it
      if (depth_map[depth_idx] == 0 ||
          depth_map[depth_idx] == depth_image->getNoSampleValue () ||
          depth_map[depth_idx] == depth_image->getShadowValue ())
      {
        // not valid
        pt.x = pt.y = pt.z = bad_point;
        continue;
      }
      pt.z = depth_map[depth_idx] * 0.001f;
      pt.x = static_cast<float> (u) * pt.z * constant;
      pt.y = static_cast<float> (v) * pt.z * constant;
    }
  }
  cloud->sensor_origin_.setZero ();
  cloud->sensor_orientation_.w () = 0.0f;
  cloud->sensor_orientation_.x () = 1.0f;
  cloud->sensor_orientation_.y () = 0.0f;
  cloud->sensor_orientation_.z () = 0.0f;  
  return (cloud);
} 
*/
