/* Compute Homography Function that takes in an RGB image and returns a homography */
cv::Mat personalRobotics::EyeKin::computeHomography(bool usePlaceHolder, cv::Mat colorImg)
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

/* Calibration method for the kinect using a checkerboard. */
void personalRobotics::EyeKin::calibrate(bool usePlaceholder, int inWidth, int inHeight)
{
	// Reset
	reset();
	
	// Setup
	screenWidth = inWidth;
	screenHeight = inHeight;
	numCheckerPtsX = 0;
	numCheckerPtsY = 0;
	personalRobotics::createCheckerboard(checkerboard, screenWidth, screenHeight, numCheckerPtsX, numCheckerPtsY);
	
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
			if (segmentorEverStarted.get())
			{
				segmentor.resumeSegmentor();
			}
			else
			{
				segmentorEverStarted.set(true);
				segmentor.startSegmentor();
			}
			
		}
		else
			Sleep(25);
	}
}