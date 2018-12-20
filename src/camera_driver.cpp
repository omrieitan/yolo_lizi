
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>


using namespace cv;
using namespace std;


bool record_video =false;
float video_delay=0;

std::string camera_name;

image_transport::Publisher image_pub;
int width,height,fps;

cv_bridge::CvImage out_msg;



void do_frame(Mat frame) {

    out_msg.image = frame;
    out_msg.header.stamp=ros::Time::now();
    image_pub.publish(out_msg.toImageMsg());
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

    do_frame(cv_ptr->image);

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle n("/");

    if (ros::param::get("~width", width)) ROS_INFO("Got width param: %d", width);
    else ROS_ERROR("Failed to get width param");

    if (ros::param::get("~height", height)) ROS_INFO("Got height param: %d", height);
    else ROS_ERROR("Failed to get height param");

    if (ros::param::get("~fps", fps)) ROS_INFO("Got fps param: %d", fps);
    else ROS_ERROR("Failed to get fps param");

    string topic="";
    if (ros::param::get("~topic", topic))  cout <<"Got topic param: "<<topic<<endl;
    else ROS_ERROR("Failed to get topic param");

    std::string ns = ros::this_node::getNamespace();
    ns.erase(0, 2);
    camera_name=ns;

    image_transport::ImageTransport it(n);
    out_msg.encoding = "bgr8";
    out_msg.header.frame_id=camera_name;
    image_pub = it.advertise(topic, 1);

    string video_device="";
    if (ros::param::get("~video_device", video_device))  cout <<"Got video_device param: "<<video_device<<endl;
    else ROS_ERROR("Failed to get video_device param");

    VideoCapture cap;

    bool use_webcam=video_device.length()<=2;
    bool use_file=video_device[video_device.length()-4]=='.';

    if (video_device.substr(0,5)!="/dev/" && !use_file && !use_webcam) {
        puts("Using topic");
        ros::Subscriber img_sub = n.subscribe(video_device, 1, imageCb);
        ros::spin();
    }

    else {

        if (use_webcam) {
            puts("Using webcam");
            cap.open(stoi(video_device));
        }

        else if (use_file) {
            puts("Using file");
            cap.open(video_device);

        }

        else {
            char gst[200];
            puts("Using v4l2src");
            sprintf(gst,"v4l2src device=%s ! video/x-raw, width=%d, height=%d ! appsink",(char *)video_device.c_str(),width,height);
            cap.open(gst);
        }



        if (!cap.isOpened())
        {
            std::cerr << "can not open camera or video file" << std::endl;
            return -1;
        }

        long frame_counter;
        long frameCnt;
if (use_file) {
     frame_counter = 0;
     frameCnt = cap.get(CV_CAP_PROP_FRAME_COUNT);
}
        while (ros::ok()) {
            Mat frame;

            if (cap.read(frame)) {
                if (frame.cols>2500) resize(frame, frame, Size(), 0.5, 0.5, INTER_LINEAR );
                do_frame(frame);
                if (use_file) {
                    ros::Duration(1.0/fps).sleep();
                    if (frame_counter++>frameCnt-10) {
                        cap.set(CV_CAP_PROP_POS_FRAMES, 0);
                        frame_counter=0;
                    }
                }
                ros::spinOnce();

            }
        }
    }

    return 0;
}

