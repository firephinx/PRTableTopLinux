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
		Conversions();
		Conversions(const cv::Mat LookupX, const cv::Mat LookupY);
		~Conversions();
	
		void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const;
	    cv::Point convertPointCloudPointToRGBCoordinates(pcl::PointXYZRGB);

    	//void convertRegisteredDepthToXYZRGBPointCloud (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const;
	};
}
#endif