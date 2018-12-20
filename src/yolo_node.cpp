#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "object_detection_yolo.h"

static const std::string OPENCV_WINDOW = "lizi view";

class YoloNode
{
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  Net net;
  Mat blob;

  ros::NodeHandle * nh_ = nullptr;

public:
  YoloNode(ros::NodeHandle & nh)
    : it_(nh)
  {
    nh_ = &nh;
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/yolo_node/input", 1,
      &YoloNode::imageCb, this);
    image_pub_ = it_.advertise("/yolo_node/output", 1);

    std::string NAMES, CFG, WEIGHTS;
    nh_->getParam("names", NAMES);
    nh_->getParam("cfg_file", CFG);
    nh_->getParam("weights", WEIGHTS);

    // Load names of classes
    string classesFile = NAMES;
    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line)) classes.push_back(line);

    // Give the configuration and weight files for the model
    String modelConfiguration = CFG;
    String modelWeights = WEIGHTS;

    // Load the network
    net = readNetFromDarknet(modelConfiguration, modelWeights);
    net.setPreferableTarget(DNN_TARGET_CPU);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);

    // cv::namedWindow(OPENCV_WINDOW);
  }

  ~YoloNode()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
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

  // Stop the program if reached end of video
      if (cv_ptr->image.empty()) {
          cout << "Done processing !!!" << endl;
          waitKey(3000);
          return;
      }
      // Create a 4D blob from a cv_ptr->
      blobFromImage(cv_ptr->image, blob, 1/255.0, cvSize(inpWidth, inpHeight), Scalar(0,0,0), true, false);

      //Sets the input to the network
      net.setInput(blob);

      // Runs the forward pass to get output of the output layers
      vector<Mat> outs;
      net.forward(outs, getOutputsNames(net));

      // Remove the bounding boxes with low confidence
      postprocess(cv_ptr->image, outs);

      // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
      vector<double> layersTimes;
      double freq = getTickFrequency() / 1000;
      double t = net.getPerfProfile(layersTimes) / freq;
      string label = format("Inference time for a frame : %.2f ms", t);
      putText(cv_ptr->image, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));

      // Write the frame with the detection boxes
      Mat detectedFrame;
      cv_ptr->image.convertTo(detectedFrame, CV_8U);

    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolo_node");
  ros::NodeHandle nh("~");
  YoloNode yolo_node(nh);
  ros::spin();
  return 0;
}
