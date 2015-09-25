#ifndef __CONVERSIONS_H__
#define __CONVERSIONS_H__

#include "pcl.h"

namespace personalRobotics
{
	class Conversions
	{
		protected:
		// Calibration
		cv::Mat lookupX,lookupY;
		public:
		// Constructor and destructor
		Conversions(const cv::Mat LookupX, const cv::Mat LookupY);
		~Conversions();
	
		// Setters
		void setLookUpX(const cv::Mat LookupX);
		void setLookUpY(const cv::Mat LookupY);

		// Functions
		void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const;
	    cv::Point convertPointCloudPointToRGBCoordinates(pcl::PointXYZRGB pointXYZRGB);

    	//void convertRegisteredDepthToXYZRGBPointCloud (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const;
	};
}
#endif