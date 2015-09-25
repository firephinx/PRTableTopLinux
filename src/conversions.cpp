#include "conversions.h"

// Constructor and Destructor
personalRobotics::Conversions::Conversions(const cv::Mat LookupX, const cv::Mat LookupY)
{
  lookupX = LookupX;
  lookupY = LookupY;
}

personalRobotics::Conversions::~Conversions()
{

}

personalRobotics::Conversions::setLookUpX(const cv::Mat LookupX)
{
  lookupX = LookupX;
}
personalRobotics::Conversions::setLookUpY(const cv::Mat LookupY)
{
  lookupY = LookupY;
}

void personalRobotics::Conversions::createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) const
{
  const float badPoint = std::numeric_limits<float>::quiet_NaN();

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


cv::Point2f personalRobotics::Conversions::convertPointCloudPointToRGBCoordinates(pcl::PointXYZRGB pointXYZRGB)
{
  float x = pointXYZRGB.x;
  float y = pointXYZRGB.y;
  float z = pointXYZRGB.z;
  float newX = x/z;
  float newY = y/z;
  cv::Point2f colorXY;
  const float y = lookupY.at<float>(0, r);
  const float *itX = lookupX.ptr<float>();


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