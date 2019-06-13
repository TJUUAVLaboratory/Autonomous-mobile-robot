
#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<nav_msgs/Odometry.h>
#include<std_msgs/String.h>
#include "cJSON.h"
#include<string>
#include <sstream>

using namespace std;

struct point {
    float x;
    float y;

} ;

void onMouse(int event, int x,int y ,int flags,void *p){
    IplImage *img1;
    img1=cvCloneImage((IplImage*)p);
    CvFont font;
    uchar *ptr;
    cvInitFont (&font,CV_FONT_HERSHEY_PLAIN,0.8,0.8,0,1,8);
 
    if ((img1->nChannels == 1)&&(event==CV_EVENT_LBUTTONDOWN)){
        char label[50];
        ptr=cvPtr2D(img1, y,x,NULL);
        sprintf(label, "(%d, %d: %d)", x, y, ptr[0]);
        std::cout << label << endl;
        cvRectangle(img1, cvPoint(x,y-12),cvPoint(x+100, y+4),
            cvScalar(255),CV_FILLED, 8,0);
        cvPutText(img1,label,cvPoint(x,y),&font,cvScalar(0) );
        cvShowImage("src",img1);
    }
    else if(event==CV_EVENT_LBUTTONDOWN){
        char label[50];    
        ptr=cvPtr2D(img1, y,x,NULL);
        sprintf(label, "(%d, %d: %d, %d, %d)", x, y, ptr[0],ptr[1],ptr[2]);
        cout << label <<endl;
        cvRectangle(img1, cvPoint(x,y-12),cvPoint(x+150, y+4),
            CV_RGB(255,0,0),CV_FILLED, 8,0);
        cvPutText(img1,label,cvPoint(x,y),&font,CV_RGB(255,255,255) );
        cvShowImage("src",img1);
    }
}

std::string getStringFromFloat(float f)
  {
      std::ostringstream buffer;
      buffer << f;
      return buffer.str();
 }

//  request map save to path
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fake_odom_node");
    ros::NodeHandle nh;

    ros::Publisher fake_odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1, true);
    ros::Publisher fake_goal_pub = nh.advertise<std_msgs::String>("/aibee_navi", 1, true);
    
    std::string image_path = "/home/aibee/aibee_navi/aibee_navi_0529/exp0528/pathfind/full_map.png";
    cv::Mat  map_image = cv::imread(image_path);
//     cv::imshow("Original Image", map_image);
    ROS_INFO_STREAM( "the image size: rows: " << map_image.rows << " cols: "<< map_image.cols);
//    // cv::setMouseCallback("Original Image", onMouse, reinterpret_cast<void *>(&map_image));
//     //注册回调函数，表示将函数onMouse与名为“Original Image”的窗口进行关联
//     cv::waitKey(0);  
        point start; start.x = 427, start.y = 280;
        point goal;  goal.x = 348, goal.y = 313;

        float param[] = {-17.2203, -13.7435, 41, 39.7};
        float resolution = param[3]/map_image.cols;

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "/odom";
        odom_msg.pose.pose.position.x = start.x*resolution + param[0];
        odom_msg.pose.pose.position.y = start.y*resolution + param[1];
        odom_msg.pose.pose.position.z = 0;

        odom_msg.pose.pose.orientation.x = 0;
        odom_msg.pose.pose.orientation.y = 0;
        odom_msg.pose.pose.orientation.z = 0;
        odom_msg.pose.pose.orientation.w = 1;

        point target;
        target.x = goal.x * resolution + param[0];
        target.y = goal.y * resolution + param[1];
        std_msgs::String target_msg;

        string  target_x, target_y;
        target_x = getStringFromFloat(target.x);
        target_y = getStringFromFloat(target.y);

    
        string msg_string = "[\"start\",\"pathfind/full_map.png\",[ -17.2203, -13.7435, 41, 39.7 ],[" + target_x + ","+target_y+"]]";
        cJSON*  json = cJSON_Parse(msg_string.c_str());
        target_msg.data = cJSON_Print(json);

        ros::Rate loop_rate(50);
        while (ros::ok())
        {
            fake_odom_pub.publish(odom_msg);
            
            fake_goal_pub.publish(target_msg);
            loop_rate.sleep();
        }
        

    return 0;

}

