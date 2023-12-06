#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  public:
    ImageConverter(): it_(nh_)
    {
      // Subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/iiwa/camera1/image_raw", 1, &ImageConverter::imageCb, this);
      image_pub_ = it_.advertise("/image_converter/output_video", 1);

      cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // Read image
      cv::Mat image_grayscale;
      cv::cvtColor(cv_ptr->image, image_grayscale, cv::COLOR_BGR2GRAY);

      // Set up the detector with default parameters.
      cv::SimpleBlobDetector::Params params;
      cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params); 
 
      // Detect blobs.
      std::vector<cv::KeyPoint> keypoints;
      detector->detect(image_grayscale, keypoints );
 
      // Draw detected blobs as red circles.
      // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
      cv::Mat image_with_keypoints;
      cv::drawKeypoints(cv_ptr->image, keypoints, image_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
 
      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, image_with_keypoints);
      cv::waitKey(3);

      // Output modified video stream
      image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}