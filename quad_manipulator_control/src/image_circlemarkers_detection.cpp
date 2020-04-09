#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <iostream>



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher markers_pub_;
  // detection key points
  std::vector<cv::KeyPoint> keypoints;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/quad_manipulation/camera1/image_raw", 1, &ImageConverter::imageCb, this);
    // publish the detection image to the ros
    image_pub_ = it_.advertise("/quad_manipulation/camera1_detection/image_raw", 1);

    //publish the marker points to the ros
    markers_pub_ = nh_.advertise<std_msgs::Int16MultiArray>("image/detection_markers", 100);

    cv::namedWindow("Image window");
  }

  ~ImageConverter()
  {
    cv::destroyWindow("Image window");
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

    //blob detection
    cv::Mat outimage;
    unsigned int ret = BlobDetection(cv_ptr->image, outimage, keypoints);
    cv_ptr->image = outimage;

    // Update GUI Window
    cv::imshow("Image window", cv_ptr->image);
    cv::waitKey(3);

    // publish modified video stream to ros
    image_pub_.publish(cv_ptr->toImageMsg());

    //publish detection markers to ros
    const unsigned int data_sz = 2*keypoints.size();
    std_msgs::Int16MultiArray msgdata;
    msgdata.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msgdata.layout.dim[0].size = data_sz;
    msgdata.layout.dim[0].stride = 1;
    msgdata.layout.dim[0].label = "markers";
    msgdata.data.resize(data_sz);
    for(unsigned int i=0; i<keypoints.size(); i++)
    {
      msgdata.data[2*i] = keypoints[i].pt.x;
      msgdata.data[2*i+1] = keypoints[i].pt.y;
    }
    markers_pub_.publish(msgdata);

  }

  /*
   * Blob point detection function
  */
  unsigned int BlobDetection(cv::Mat inputimage, cv::Mat& outimage, std::vector<cv::KeyPoint>& keypoints)
  {
    //declar
    unsigned int ret = 0;

    cv::Mat srcGrayImage;

    if (inputimage.channels() == 3)
    {
        cv::cvtColor(inputimage, srcGrayImage, CV_RGB2GRAY);
    }
    else
    {
        inputimage.copyTo(srcGrayImage);
    }

    cv::SimpleBlobDetector::Params params;
    //threshodhold
    params.minThreshold = 10;
    params.maxThreshold = 50;
    //circle
    params.minArea = 10;
    params.maxArea = 10000;
    params.filterByCircularity = true;
    //color
    params.filterByColor = true;
    params.blobColor = 0;

    cv::Ptr<cv::SimpleBlobDetector> sbd = cv::SimpleBlobDetector::create(params);

    sbd->detect(srcGrayImage, keypoints);

    cv::drawKeypoints(srcGrayImage, keypoints, outimage, cv::Scalar(0,0,255),cv::DrawMatchesFlags::DEFAULT);

    //return
    return(ret);
  }
};




int main(int argc,char* argv[])
{
  //ros init
  ros::init(argc, argv, "circlemarkers_detection_");

  //image blob detection
  ImageConverter ic;

  //ros spin
  ros::spin();

  return 0;
}
