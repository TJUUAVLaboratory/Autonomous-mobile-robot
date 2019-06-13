
#include<ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include<nav_msgs/Odometry.h>
#include<std_msgs/String.h>

using namespace std;


// void onMouse(int event, int x, int y, int flags, void *param)
// {
// cv::Mat *im = reinterpret_cast<cv::Mat*>(param);
// switch (event)   //调度事件
//     {
//     case EVENT_LBUTTONDOWN:  //鼠标左键按下事件
//     //显示像素值
//     std::cout << "at(" << x << "," << y << ") value is:" << static_cast<int>(im->at<uchar>(cv::Point(x, y))) << std::endl;
//     //使用Mat对象的at方法来获取(x,y)的像素值
//     break;

//     }
// }


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



//  request map save to path
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fake_odom_node");
    ros::NodeHandle nh;

    ros::Publisher fake_odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50, true);
    ros::Publisher fake_goal_pub = nh.advertise<std_msgs::String>("/aibee_navi", 50, true);
    
    std::string image_path = "/home/aibee/aibee_navi/aibee_navi_0529/exp0528/pathfind/full_map.png";
    cv::Mat  map_image = cv::imread(image_path);

    cv::imshow("Original Image", map_image);
    ROS_INFO_STREAM( "the image size: rows: " << map_image.rows << " cols: "<< map_image.cols);

   // cv::setMouseCallback("Original Image", onMouse, reinterpret_cast<void *>(&map_image));
//注册回调函数，表示将函数onMouse与名为“Original Image”的窗口进行关联
    cv::waitKey(0);    

    return 0;

}

