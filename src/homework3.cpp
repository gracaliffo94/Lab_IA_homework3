#include <ros/ros.h>

#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

class Min_distance
{
public:
  Min_distance()
  {
    //Topic you want to publish
    pub_ = n_.advertise<std_msgs::Float32>("/min_distance", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("base_scan", 1000, &Min_distance::callback,this);
  }

  void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    using namespace cv;
    using namespace std;
    ros::NodeHandle n2;
    int i=0;
    float anglemax     = msg->angle_max;
    float anglemin     = msg->angle_min;
    float angleinc     = msg->angle_increment;
    float length       = (anglemax-anglemin)/angleinc;
    int intlength      = int(length)+1;
    float min_distance = msg->ranges[0];

 /*   for (i=0; i<intlength;i++){
     if(min_distance>msg->ranges[i]){
      min_distance=msg->ranges[i];
      ROS_INFO("i %d",i);
     }
    }*/

/*    std_msgs::Float32 msg2;
    msg2.data= min_distance;
    pub_.publish(msg2);
    ROS_INFO("MINIMUM DISTANCE: %f", msg2.data);
*/
    const Scalar black=Scalar(0,0,0);
    const Scalar red=Scalar(0,0,255);
    int size=600;
    cv::Mat M(size,size, CV_8UC3, cv::Scalar(255,255,255));
    int rescale=20;
    int min_distance_index;
    Point prev=Point( (size/2) + cos(anglemin)*(msg->ranges[0])*rescale , (size/2) + sin(anglemin)*(msg->ranges[0])*rescale);
    float angle_min_distance=anglemin;
    for(int i=0;i<intlength;++i){
	Point next = Point( (size/2) + cos(anglemin+i*angleinc)*(msg->ranges[i])*rescale , (size/2) + sin(anglemin+i*angleinc)*(msg->ranges[i])*rescale);
	line(M,prev,next,black,1,LINE_8,0);
	if(min_distance > msg->ranges[i]){
                min_distance_index=i;
		min_distance=msg->ranges[i];
		ROS_INFO("i %d",i);
	}
        prev=next;
    }
    angle_min_distance=anglemin+min_distance_index*angleinc;
    circle(M,Point((size/2)+rescale*min_distance*cos(angle_min_distance),(size/2)+rescale*min_distance*sin(angle_min_distance)),5,red,1, LINE_8, 0);
    ROS_INFO("--->%f",angle_min_distance);
    imshow("Scan", M);
    waitKey(1);
}
private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "Mini");

  //Create an object of class SubscribeAndPublish that will take care of everything
  Min_distance SAPObject;

  ros::spin();

  return 0;
}

