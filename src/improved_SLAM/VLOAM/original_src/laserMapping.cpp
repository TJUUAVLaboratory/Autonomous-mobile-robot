/* Before we start, remember the big picture. After the last node, laserOdometry, we already have a 
   odometry estimate. So why this node? 
   
   The answer is, due to the unavoidable measurement noise!  

   Also, this node is very computationally expensive, so it is called at 1 Hz task rate.

   In LOAM paper, Fig 8 is a very helpful visual of what's going on here. However, the LOAM paper does not really
   explain at all what is going on. 
 */

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

/* Many parameters that can be tuned in KITTI */
const double PI = 3.1415926;

const float scanPeriod = 0.1;

const int stackFrameNum = 1;
const int mapFrameNum = 5;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast = false;
bool newLaserCloudFullRes = false;
bool newLaserOdometry = false;

/* Especially here */
/* The below has to do with dividing the map point cloud Q_{k-1} into a cubes */
int laserCloudCenWidth = 10;
int laserCloudCenHeight = 5;
int laserCloudCenDepth = 10;
const int laserCloudWidth = 21;
const int laserCloudHeight = 11;
const int laserCloudDepth = 21;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

int laserCloudValidInd[125];
int laserCloudSurroundInd[125];

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerStack(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfStack(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerStack2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfStack2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>());
//pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSel(new pcl::PointCloud<pcl::PointXYZI>());
//pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCorr(new pcl::PointCloud<pcl::PointXYZI>());
//pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudProj(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerArray[laserCloudNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfArray[laserCloudNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerArray2[laserCloudNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfArray2[laserCloudNum];

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>());

float transformSum[6] = {0};
float transformIncre[6] = {0};
float transformTobeMapped[6] = {0};
float transformBefMapped[6] = {0};
float transformAftMapped[6] = {0};

int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 200;

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};

void transformAssociateToMap()
{
  float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
           - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
  float y1 = transformBefMapped[4] - transformSum[4];
  float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
           + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

  float x2 = x1;
  float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
  float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

  transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
  transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
  transformIncre[5] = z2;

  float sbcx = sin(transformSum[0]);
  float cbcx = cos(transformSum[0]);
  float sbcy = sin(transformSum[1]);
  float cbcy = cos(transformSum[1]);
  float sbcz = sin(transformSum[2]);
  float cbcz = cos(transformSum[2]);

  float sblx = sin(transformBefMapped[0]);
  float cblx = cos(transformBefMapped[0]);
  float sbly = sin(transformBefMapped[1]);
  float cbly = cos(transformBefMapped[1]);
  float sblz = sin(transformBefMapped[2]);
  float cblz = cos(transformBefMapped[2]);

  float salx = sin(transformAftMapped[0]);
  float calx = cos(transformAftMapped[0]);
  float saly = sin(transformAftMapped[1]);
  float caly = cos(transformAftMapped[1]);
  float salz = sin(transformAftMapped[2]);
  float calz = cos(transformAftMapped[2]);

  float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
            - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
            - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
            - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
            - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
  transformTobeMapped[0] = -asin(srx);

  float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
               - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
               - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
               - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
               + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
               - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
               - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
               - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
               + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]), 
                                 crycrx / cos(transformTobeMapped[0]));
  
  float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) 
               - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx) 
               - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly) 
               + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx) 
               - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz 
               + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) 
               + calx*cblx*salz*sblz);
  float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) 
               - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx) 
               + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx) 
               + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) 
               + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly 
               - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz) 
               - calx*calz*cblx*sblz);
  transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]), 
                                 crzcrx / cos(transformTobeMapped[0]));

  x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
  y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
  z1 = transformIncre[5];

  x2 = x1;
  y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
  z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

  transformTobeMapped[3] = transformAftMapped[3] 
                         - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
  transformTobeMapped[4] = transformAftMapped[4] - y2;
  transformTobeMapped[5] = transformAftMapped[5] 
                         - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void transformUpdate()
{
  if (imuPointerLast >= 0) {
    float imuRollLast = 0, imuPitchLast = 0;
    while (imuPointerFront != imuPointerLast) {
      if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
        break;
      }
      imuPointerFront = (imuPointerFront + 1) % imuQueLength;
    }

    if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
      imuRollLast = imuRoll[imuPointerFront];
      imuPitchLast = imuPitch[imuPointerFront];
    } else {
      int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
      float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack]) 
                       / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod) 
                      / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

      imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
      imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
    }

    transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
    transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
  }

  //transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * transformSum[0];
  //transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * transformSum[2];

  for (int i = 0; i < 6; i++) {
    transformBefMapped[i] = transformSum[i];
    transformAftMapped[i] = transformTobeMapped[i];
  }
}

void pointAssociateToMap(pcl::PointXYZI *pi, pcl::PointXYZI *po)
{
  float x1 = cos(transformTobeMapped[2]) * pi->x
           - sin(transformTobeMapped[2]) * pi->y;
  float y1 = sin(transformTobeMapped[2]) * pi->x
           + cos(transformTobeMapped[2]) * pi->y;
  float z1 = pi->z;

  float x2 = x1;
  float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
  float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

  po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
        + transformTobeMapped[3];
  po->y = y2 + transformTobeMapped[4];
  po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
        + transformTobeMapped[5];
  po->intensity = pi->intensity;
}

void pointAssociateTobeMapped(pcl::PointXYZI *pi, pcl::PointXYZI *po)
{
  float x1 = cos(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3]) 
           - sin(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);
  float y1 = pi->y - transformTobeMapped[4];
  float z1 = sin(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3]) 
           + cos(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);

  float x2 = x1;
  float y2 = cos(transformTobeMapped[0]) * y1 + sin(transformTobeMapped[0]) * z1;
  float z2 = -sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

  po->x = cos(transformTobeMapped[2]) * x2
        + sin(transformTobeMapped[2]) * y2;
  po->y = -sin(transformTobeMapped[2]) * x2
        + cos(transformTobeMapped[2]) * y2;
  po->z = z2;
  po->intensity = pi->intensity;
}

void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2)
{
  timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();

  laserCloudCornerLast->clear();
  pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);

  newLaserCloudCornerLast = true;
}

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfLast2)
{
  timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();

  laserCloudSurfLast->clear();
  pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurfLast);

  newLaserCloudSurfLast = true;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
  timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

  laserCloudFullRes->clear();
  pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);

  newLaserCloudFullRes = true;
}

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  timeLaserOdometry = laserOdometry->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  transformSum[0] = -pitch;
  transformSum[1] = -yaw;
  transformSum[2] = roll;

  transformSum[3] = laserOdometry->pose.pose.position.x;
  transformSum[4] = laserOdometry->pose.pose.position.y;
  transformSum[5] = laserOdometry->pose.pose.position.z;

  newLaserOdometry = true;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  imuPointerLast = (imuPointerLast + 1) % imuQueLength;

  imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>
                                            ("/laser_cloud_corner_last", 2, laserCloudCornerLastHandler);

  ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>
                                          ("/laser_cloud_surf_last", 2, laserCloudSurfLastHandler);

  ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry> 
                                     ("/laser_odom_to_init", 5, laserOdometryHandler);

  ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2> 
                                         ("/velodyne_cloud_3", 2, laserCloudFullResHandler);

  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);

  ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2> 
                                         ("/laser_cloud_surround", 1);

  ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2> 
                                        ("/velodyne_cloud_registered", 2);

  //ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pc1", 2);

  //ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc2", 2);

  //ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pc3", 2);

  //ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc4", 2);

  ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);
  nav_msgs::Odometry odomAftMapped;
  odomAftMapped.header.frame_id = "/camera_init";
  odomAftMapped.child_frame_id = "/aft_mapped";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform aftMappedTrans;
  aftMappedTrans.frame_id_ = "/camera_init";
  aftMappedTrans.child_frame_id_ = "/aft_mapped";

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  pcl::PointXYZI pointOri, pointSel, pointProj, coeff;

  cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
  cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

  cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

  bool isDegenerate = false;
  cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterCorner;
  downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
  downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterMap;
  downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);

  for (int i = 0; i < laserCloudNum; i++) {
    laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudCornerArray2[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfArray2[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  /* frameCount initializes as 0, mapFrameCount initializes as 4 */
  int frameCount = stackFrameNum - 1;
  int mapFrameCount = mapFrameNum - 1;
  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes && newLaserOdometry &&
        fabs(timeLaserCloudCornerLast - timeLaserOdometry) < 0.005 &&
        fabs(timeLaserCloudSurfLast - timeLaserOdometry) < 0.005 &&
        fabs(timeLaserCloudFullRes - timeLaserOdometry) < 0.005) {
      newLaserCloudCornerLast = false;
      newLaserCloudSurfLast = false;
      newLaserCloudFullRes = false;
      newLaserOdometry = false;

      frameCount++;

	  /* Coordinate transform here */
      if (frameCount >= stackFrameNum) {
        transformAssociateToMap();

        int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        for (int i = 0; i < laserCloudCornerLastNum; i++) {
          pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);
          laserCloudCornerStack2->push_back(pointSel);
        }

        int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
        for (int i = 0; i < laserCloudSurfLastNum; i++) {
          pointAssociateToMap(&laserCloudSurfLast->points[i], &pointSel);
          laserCloudSurfStack2->push_back(pointSel);
        }
      }

	  /* Optimization step */
	  /* WTF is this code */

	  /* From LOAM paper:
	     To find correspondences for the feature points, we store the point cloud 
		 on the map, Q_{k-1} , in 10m cubic areas. The points in the cubes that 
		 intersect with \bar{Q}_{k} are extracted and stored in a 3D KD-tree in {W}. 
		 We find the points in Q_{k-1} within  a certain region (10cm × 10cm × 10cm) around the feature points.
	  */
      if (frameCount >= stackFrameNum) {
        frameCount = 0;

		/* Find which subcube the currently estimated lidar pose corresponds to. i,j,k are the indices of the cube.
		 * For example when the coordinates are (-25,25), the cube would be centered at (10,5,10).
		 */
        pcl::PointXYZI pointOnYAxis; /* A point on the y-axis of the current Lidar coordinate system {L} (0,10,0) */
        pointOnYAxis.x = 0.0;
        pointOnYAxis.y = 10.0;
        pointOnYAxis.z = 0.0;
        pointAssociateToMap(&pointOnYAxis, &pointOnYAxis); /* Transforms to the world frame {W} */

		/* index at the center of the cube */
        int centerCubeI = int((transformTobeMapped[3] + 25.0) / 50.0) + laserCloudCenWidth;
        int centerCubeJ = int((transformTobeMapped[4] + 25.0) / 50.0) + laserCloudCenHeight;
        int centerCubeK = int((transformTobeMapped[5] + 25.0) / 50.0) + laserCloudCenDepth;

        if (transformTobeMapped[3] + 25.0 < 0) centerCubeI--;
        if (transformTobeMapped[4] + 25.0 < 0) centerCubeJ--;
        if (transformTobeMapped[5] + 25.0 < 0) centerCubeK--;

		/* If the subcube is taken at the edge of the largest cube,
		 * the index of the cube corresponding to the point is moved 1 unit toward the center. 
		   UNCLEAR - why?
		   Do this for every index I J K
		 */
        while (centerCubeI < 3) {

		  /* Moves the pointer to the center  */
          for (int j = 0; j < laserCloudHeight; j++) {
            for (int k = 0; k < laserCloudDepth; k++) {
              int i = laserCloudWidth - 1;
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer = 
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer = 
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; i >= 1; i--) {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i - 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

		  /* Incrementing the cube indices */
          centerCubeI++;
          laserCloudCenWidth++;
        }

        while (centerCubeI >= laserCloudWidth - 3) {
          for (int j = 0; j < laserCloudHeight; j++) {
            for (int k = 0; k < laserCloudDepth; k++) {
              int i = 0;
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer = 
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer = 
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; i < laserCloudWidth - 1; i++) {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeI--;
          laserCloudCenWidth--;
        }

        while (centerCubeJ < 3) {
          for (int i = 0; i < laserCloudWidth; i++) {
            for (int k = 0; k < laserCloudDepth; k++) {
              int j = laserCloudHeight - 1;
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer = 
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer = 
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; j >= 1; j--) {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth*(j - 1) + laserCloudWidth * laserCloudHeight*k];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }
 
          centerCubeJ++;
          laserCloudCenHeight++;
        } 

        while (centerCubeJ >= laserCloudHeight - 3) {
          for (int i = 0; i < laserCloudWidth; i++) {
            for (int k = 0; k < laserCloudDepth; k++) {
              int j = 0;
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer = 
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer = 
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; j < laserCloudHeight - 1; j++) {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth*(j + 1) + laserCloudWidth * laserCloudHeight*k];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeJ--;
          laserCloudCenHeight--;
        }

        while (centerCubeK < 3) {
          for (int i = 0; i < laserCloudWidth; i++) {
            for (int j = 0; j < laserCloudHeight; j++) {
              int k = laserCloudDepth - 1;
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer = 
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer = 
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; k >= 1; k--) {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k - 1)];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeK++;
          laserCloudCenDepth++;
        }
      
        while (centerCubeK >= laserCloudDepth - 3) {
          for (int i = 0; i < laserCloudWidth; i++) {
            for (int j = 0; j < laserCloudHeight; j++) {
              int k = 0;
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeCornerPointer = 
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCubeSurfPointer = 
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
              for (; k < laserCloudDepth - 1; k++) {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k + 1)];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
              }
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeCornerPointer;
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = 
              laserCloudCubeSurfPointer;
              laserCloudCubeCornerPointer->clear();
              laserCloudCubeSurfPointer->clear();
            }
          }

          centerCubeK--;
          laserCloudCenDepth--;
        }



		/* After processing the edge points, the next step is to find the corresponding registration point
		   in the 5x5x5 neighbors of the acquired cube */
        int laserCloudValidNum = 0;
        int laserCloudSurroundNum = 0;
        for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
          for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
            for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++) {
              if (i >= 0 && i < laserCloudWidth && 
                  j >= 0 && j < laserCloudHeight && 
                  k >= 0 && k < laserCloudDepth) {

                float centerX = 50.0 * (i - laserCloudCenWidth);
                float centerY = 50.0 * (j - laserCloudCenHeight);
                float centerZ = 50.0 * (k - laserCloudCenDepth);

				/* Takes neighboring 8 points */
                bool isInLaserFOV = false;
                for (int ii = -1; ii <= 1; ii += 2) {
                  for (int jj = -1; jj <= 1; jj += 2) {
                    for (int kk = -1; kk <= 1; kk += 2) {
                      float cornerX = centerX + 25.0 * ii;
                      float cornerY = centerY + 25.0 * jj;
                      float cornerZ = centerZ + 25.0 * kk;

                      float squaredSide1 = (transformTobeMapped[3] - cornerX) 
                                         * (transformTobeMapped[3] - cornerX) 
                                         + (transformTobeMapped[4] - cornerY) 
                                         * (transformTobeMapped[4] - cornerY)
                                         + (transformTobeMapped[5] - cornerZ) 
                                         * (transformTobeMapped[5] - cornerZ);

                      float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX) 
                                         + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
                                         + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

                      float check1 = 100.0 + squaredSide1 - squaredSide2
                                   - 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                      float check2 = 100.0 + squaredSide1 - squaredSide2
                                   + 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                      if (check1 < 0 && check2 > 0) {
                        isInLaserFOV = true;
                      }
                    }
                  }
                }

                if (isInLaserFOV) {
                  laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j 
                                                       + laserCloudWidth * laserCloudHeight * k;
                  laserCloudValidNum++;
                }
                laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j 
                                                             + laserCloudWidth * laserCloudHeight * k;
                laserCloudSurroundNum++;
              }
            }
          }
        }

		/* The benefit of doing all of the above is that we no longer need to process huge point clouds
		   We just need to process the neighboring cube regions. */
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        for (int i = 0; i < laserCloudValidNum; i++) {
          *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
          *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
        }
        int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size(); /* The number of points on the feature edge */
        int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size(); /* The number of points on the active feature surface */

		/* Transfers the current frame feature point in the world coordinate system to the current Lidar coordinate syste */
        int laserCloudCornerStackNum2 = laserCloudCornerStack2->points.size();
        for (int i = 0; i < laserCloudCornerStackNum2; i++) {
          pointAssociateTobeMapped(&laserCloudCornerStack2->points[i], &laserCloudCornerStack2->points[i]);
        }

        int laserCloudSurfStackNum2 = laserCloudSurfStack2->points.size();
        for (int i = 0; i < laserCloudSurfStackNum2; i++) {
          pointAssociateTobeMapped(&laserCloudSurfStack2->points[i], &laserCloudSurfStack2->points[i]);
        }

		/* Filter the features points in the current frame */
        laserCloudCornerStack->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerStack2);
        downSizeFilterCorner.filter(*laserCloudCornerStack);
        int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

        laserCloudSurfStack->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfStack2);
        downSizeFilterSurf.filter(*laserCloudSurfStack);
        int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

        laserCloudCornerStack2->clear();
        laserCloudSurfStack2->clear();

		/* So far, we have all the map feature points near the current location of the lidar
		   and the point cloud features of the current frame. Next we again use KD-tree to find the 
		   nearest 5 points.. */

		/* This involves Principle Component Analysis of the point cloud covariance matrix.
		 * 
		 * If the 5 points are distributed on a straight line, the eigenvalue of the covariance matrix
		 * contains 1 element that is significantly larger than the others. The eigenvector associated 
		 * with that eigenvalue represents the directorion of the straight line. 		
		 *
		 * If the 5 points are distributed on a plane, there is a significantly SMALLER element of the eigenvalue 
		 * of the covariance matrix. The associated eigenvector represents 
		 * the *normal* direction of the plane in which it lies. Thus we can find 2 points on the straight line
		 * to construct a optimization problem via the the distance from point to line (See LOAM paper..)
		 */

		/* The plane features use the same idea as above, and after the optimization problem is fully built,
		 * it is again solved by L-M iterative method. This is almost identical to laserOdometry code so there
		 * are no more comments here.
		 */

        if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100) {
          kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
          kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

          for (int iterCount = 0; iterCount < 10; iterCount++) {
            laserCloudOri->clear();
            //laserCloudSel->clear();
            //laserCloudCorr->clear();
            //laserCloudProj->clear();
            coeffSel->clear();

            for (int i = 0; i < laserCloudCornerStackNum; i++) {
              pointOri = laserCloudCornerStack->points[i];
              pointAssociateToMap(&pointOri, &pointSel);
              kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
              
              if (pointSearchSqDis[4] < 1.0) {
                float cx = 0;
                float cy = 0; 
                float cz = 0;
                for (int j = 0; j < 5; j++) {
                  cx += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
                  cy += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
                  cz += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
                }
                cx /= 5;
                cy /= 5; 
                cz /= 5;

                float a11 = 0;
                float a12 = 0; 
                float a13 = 0;
                float a22 = 0;
                float a23 = 0; 
                float a33 = 0;
                for (int j = 0; j < 5; j++) {
                  float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - cx;
                  float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - cy;
                  float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - cz;

                  a11 += ax * ax;
                  a12 += ax * ay;
                  a13 += ax * az;
                  a22 += ay * ay;
                  a23 += ay * az;
                  a33 += az * az;
                }
                a11 /= 5;
                a12 /= 5; 
                a13 /= 5;
                a22 /= 5;
                a23 /= 5; 
                a33 /= 5;

                matA1.at<float>(0, 0) = a11;
                matA1.at<float>(0, 1) = a12;
                matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12;
                matA1.at<float>(1, 1) = a22;
                matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13;
                matA1.at<float>(2, 1) = a23;
                matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                  float x0 = pointSel.x;
                  float y0 = pointSel.y;
                  float z0 = pointSel.z;
                  float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                  float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                  float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                  float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                  float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                  float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                  float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                             * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                             + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                             * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                             + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                             * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                  float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                  float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                           + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                  float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                           - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                  float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                           + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                  float ld2 = a012 / l12;

                  pointProj = pointSel;
                  pointProj.x -= la * ld2;
                  pointProj.y -= lb * ld2;
                  pointProj.z -= lc * ld2;

                  float s = 1 - 0.9 * fabs(ld2);

                  coeff.x = s * la;
                  coeff.y = s * lb;
                  coeff.z = s * lc;
                  coeff.intensity = s * ld2;

                  if (s > 0.1) {
                    laserCloudOri->push_back(pointOri);
                    //laserCloudSel->push_back(pointSel);
                    //laserCloudProj->push_back(pointProj);
                    //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[0]]);
                    //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[1]]);
                    //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[2]]);
                    //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[3]]);
                    //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[4]]);
                    coeffSel->push_back(coeff);
                  }
                }
              }
            }

            for (int i = 0; i < laserCloudSurfStackNum; i++) {
              pointOri = laserCloudSurfStack->points[i];
              pointAssociateToMap(&pointOri, &pointSel); 
              kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

              if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                  matA0.at<float>(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                  matA0.at<float>(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                  matA0.at<float>(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                }
                cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

                float pa = matX0.at<float>(0, 0);
                float pb = matX0.at<float>(1, 0);
                float pc = matX0.at<float>(2, 0);
                float pd = 1;
 
                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                  if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                      pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                      pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2) {
                    planeValid = false;
                    break;
                  }
                }

                if (planeValid) {
                  float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                  pointProj = pointSel;
                  pointProj.x -= pa * pd2;
                  pointProj.y -= pb * pd2;
                  pointProj.z -= pc * pd2;

                  float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                          + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                  coeff.x = s * pa;
                  coeff.y = s * pb;
                  coeff.z = s * pc;
                  coeff.intensity = s * pd2;

                  if (s > 0.1) {
                    laserCloudOri->push_back(pointOri);
                    //laserCloudSel->push_back(pointSel);
                    //laserCloudProj->push_back(pointProj);
                    //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[0]]);
                    //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[1]]);
                    //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[2]]);
                    //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[3]]);
                    //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[4]]);
                    coeffSel->push_back(coeff);
                  }
                }
              }
            }

            float srx = sin(transformTobeMapped[0]);
            float crx = cos(transformTobeMapped[0]);
            float sry = sin(transformTobeMapped[1]);
            float cry = cos(transformTobeMapped[1]);
            float srz = sin(transformTobeMapped[2]);
            float crz = cos(transformTobeMapped[2]);

            int laserCloudSelNum = laserCloudOri->points.size();
            if (laserCloudSelNum < 50) {
              continue;
            }

            cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
            cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
            cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
            cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
            for (int i = 0; i < laserCloudSelNum; i++) {
              pointOri = laserCloudOri->points[i];
              coeff = coeffSel->points[i];

              float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                        + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                        + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

              float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                        + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                        + ((-cry*crz - srx*sry*srz)*pointOri.x 
                        + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

              float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                        + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                        + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

              matA.at<float>(i, 0) = arx;
              matA.at<float>(i, 1) = ary;
              matA.at<float>(i, 2) = arz;
              matA.at<float>(i, 3) = coeff.x;
              matA.at<float>(i, 4) = coeff.y;
              matA.at<float>(i, 5) = coeff.z;
              matB.at<float>(i, 0) = -coeff.intensity;
            }
            cv::transpose(matA, matAt);
            matAtA = matAt * matA;
            matAtB = matAt * matB;
            cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

            if (iterCount == 0) {
              cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
              cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
              cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

              cv::eigen(matAtA, matE, matV);
              matV.copyTo(matV2);

              isDegenerate = false;
              float eignThre[6] = {100, 100, 100, 100, 100, 100};
              for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                  for (int j = 0; j < 6; j++) {
                    matV2.at<float>(i, j) = 0;
                  }
                  isDegenerate = true;
                } else {
                  break;
                }
              }
              matP = matV.inv() * matV2;
            }

            if (isDegenerate /*&& 0*/) {
              cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
              matX.copyTo(matX2);
              matX = matP * matX2;

              //ROS_INFO ("laser mapping degenerate");
            }

            transformTobeMapped[0] += matX.at<float>(0, 0);
            transformTobeMapped[1] += matX.at<float>(1, 0);
            transformTobeMapped[2] += matX.at<float>(2, 0);
            transformTobeMapped[3] += matX.at<float>(3, 0);
            transformTobeMapped[4] += matX.at<float>(4, 0);
            transformTobeMapped[5] += matX.at<float>(5, 0);

            float deltaR = sqrt(matX.at<float>(0, 0) * 180 / PI * matX.at<float>(0, 0) * 180 / PI
                         + matX.at<float>(1, 0) * 180 / PI * matX.at<float>(1, 0) * 180 / PI
                         + matX.at<float>(2, 0) * 180 / PI * matX.at<float>(2, 0) * 180 / PI);
            float deltaT = sqrt(matX.at<float>(3, 0) * 100 * matX.at<float>(3, 0) * 100
                         + matX.at<float>(4, 0) * 100 * matX.at<float>(4, 0) * 100
                         + matX.at<float>(5, 0) * 100 * matX.at<float>(5, 0) * 100);

            if (deltaR < 0.05 && deltaT < 0.05) {
              break;
            }

            //ROS_INFO ("iter: %d, deltaR: %f, deltaT: %f", iterCount, deltaR, deltaT);
          }

          transformUpdate();
        }

        for (int i = 0; i < laserCloudCornerStackNum; i++) {
          pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

          int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
          int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
          int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

          if (pointSel.x + 25.0 < 0) cubeI--;
          if (pointSel.y + 25.0 < 0) cubeJ--;
          if (pointSel.z + 25.0 < 0) cubeK--;

          if (cubeI >= 0 && cubeI < laserCloudWidth && 
              cubeJ >= 0 && cubeJ < laserCloudHeight && 
              cubeK >= 0 && cubeK < laserCloudDepth) {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            laserCloudCornerArray[cubeInd]->push_back(pointSel);
          }
        }

        for (int i = 0; i < laserCloudSurfStackNum; i++) {
          pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

          int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
          int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
          int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

          if (pointSel.x + 25.0 < 0) cubeI--;
          if (pointSel.y + 25.0 < 0) cubeJ--;
          if (pointSel.z + 25.0 < 0) cubeK--;

          if (cubeI >= 0 && cubeI < laserCloudWidth && 
              cubeJ >= 0 && cubeJ < laserCloudHeight && 
              cubeK >= 0 && cubeK < laserCloudDepth) {
            int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
            laserCloudSurfArray[cubeInd]->push_back(pointSel);
          }
        }

        for (int i = 0; i < laserCloudValidNum; i++) {
          int ind = laserCloudValidInd[i];

          laserCloudCornerArray2[ind]->clear();
          downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
          downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

          laserCloudSurfArray2[ind]->clear();
          downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
          downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
          laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
          laserCloudCornerArray2[ind] = laserCloudTemp;

          laserCloudTemp = laserCloudSurfArray[ind];
          laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
          laserCloudSurfArray2[ind] = laserCloudTemp;
        }

        mapFrameCount++;
        if (mapFrameCount >= mapFrameNum) {
          mapFrameCount = 0;

          laserCloudSurround2->clear();
          for (int i = 0; i < laserCloudSurroundNum; i++) {
            int ind = laserCloudSurroundInd[i];
            *laserCloudSurround2 += *laserCloudCornerArray[ind];
            *laserCloudSurround2 += *laserCloudSurfArray[ind];
          }

          laserCloudSurround->clear();
          downSizeFilterCorner.setInputCloud(laserCloudSurround2);
          downSizeFilterCorner.filter(*laserCloudSurround);

		  /* I lied, 1 more comment is that in order to keep the real time operation going,
		   * the environment cloud is only published every 5 frames.
		   */


          sensor_msgs::PointCloud2 laserCloudSurround3;
          pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
          laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
          laserCloudSurround3.header.frame_id = "/camera_init";
          pubLaserCloudSurround.publish(laserCloudSurround3);
        }

        int laserCloudFullResNum = laserCloudFullRes->points.size();
        for (int i = 0; i < laserCloudFullResNum; i++) {
          pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        laserCloudFullRes3.header.frame_id = "/camera_init";
        pubLaserCloudFullRes.publish(laserCloudFullRes3);

        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                  (transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

        odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
        odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
        odomAftMapped.pose.pose.orientation.z = geoQuat.x;
        odomAftMapped.pose.pose.orientation.w = geoQuat.w;
        odomAftMapped.pose.pose.position.x = transformAftMapped[3];
        odomAftMapped.pose.pose.position.y = transformAftMapped[4];
        odomAftMapped.pose.pose.position.z = transformAftMapped[5];
        odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
        odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
        odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
        odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
        odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
        odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
        pubOdomAftMapped.publish(odomAftMapped);

        aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
        aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3], 
                                             transformAftMapped[4], transformAftMapped[5]));
        tfBroadcaster.sendTransform(aftMappedTrans);

        /*sensor_msgs::PointCloud2 pc12;
        pcl::toROSMsg(*laserCloudCornerStack, pc12);
        pc12.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        pc12.header.frame_id = "/camera";
        pub1.publish(pc12);

        sensor_msgs::PointCloud2 pc22;
        pcl::toROSMsg(*laserCloudSurfStack, pc22);
        pc22.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        pc22.header.frame_id = "/camera";
        pub2.publish(pc22);

        sensor_msgs::PointCloud2 pc32;
        pcl::toROSMsg(*laserCloudSel, pc32);
        pc32.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        pc32.header.frame_id = "/camera";
        pub3.publish(pc32);

        sensor_msgs::PointCloud2 pc42;
        pcl::toROSMsg(*laserCloudProj, pc42);
        pc42.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        pc42.header.frame_id = "/camera";
        pub4.publish(pc42);*/
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
