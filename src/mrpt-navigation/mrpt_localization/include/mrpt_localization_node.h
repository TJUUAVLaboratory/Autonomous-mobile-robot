/***********************************************************************************
 * Revised BSD License *
 * Copyright (c) 2014, Markus Bader <markus.bader@tuwien.ac.at> *
 * All rights reserved. *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without *
 * modification, are permitted provided that the following conditions are met: *
 *     * Redistributions of source code must retain the above copyright *
 *       notice, this list of conditions and the following disclaimer. *
 *     * Redistributions in binary form must reproduce the above copyright *
 *       notice, this list of conditions and the following disclaimer in the *
 *       documentation and/or other materials provided with the distribution. *
 *     * Neither the name of the Vienna University of Technology nor the *
 *       names of its contributors may be used to endorse or promote products *
 *       derived from this software without specific prior written permission. *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 **
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE *
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 **
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 **
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **                       *
 ***********************************************************************************/

#ifndef MRPT_LOCALIZATION_NODE_H
#define MRPT_LOCALIZATION_NODE_H

#include <cstring>  // size_t

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <dynamic_reconfigure/server.h>

#include <mrpt/version.h>
#include <mrpt/obs/CObservationOdometry.h>
using mrpt::obs::CObservationOdometry;

#include "mrpt_localization/MotionConfig.h"
#include "mrpt_localization/mrpt_localization.h"
#include "mrpt_msgs/ObservationRangeBeacon.h"

/// ROS Node
class PFLocalizationNode : public PFLocalization
{
	MRPT_ROS_LOG_MACROS;

   public:
	struct Parameters : public PFLocalization::Parameters
	{
		static const int MOTION_MODEL_GAUSSIAN = 0;
		static const int MOTION_MODEL_THRUN = 1;
		Parameters(PFLocalizationNode* p);
		ros::NodeHandle node;
		void callbackParameters(
			mrpt_localization::MotionConfig& config, uint32_t level);
		dynamic_reconfigure::Server<mrpt_localization::MotionConfig>
			reconfigure_server_;
		dynamic_reconfigure::Server<
			mrpt_localization::MotionConfig>::CallbackType reconfigure_cb_;
		void update(const unsigned long& loop_count);
		double rate;
		double transform_tolerance;  ///< projection into the future added to
		/// the published tf to extend its validity
		double no_update_tolerance;  ///< maximum time without updating we keep
		/// using filter time instead of Time::now
		double no_inputs_tolerance;  ///< maximum time without any observation
		/// we wait before start complaining
		int parameter_update_skip;
		int particlecloud_update_skip;
		int map_update_skip;
		std::string tf_prefix;
		std::string base_frame_id;
		std::string odom_frame_id;
		std::string global_frame_id;
		bool update_while_stopped;
		bool pose_broadcast;
		bool tf_broadcast;
		bool use_map_topic;
		bool first_map_only;
	};

	PFLocalizationNode(ros::NodeHandle& n);
	virtual ~PFLocalizationNode();
	void init();
	void loop();
	void callbackLaser(const sensor_msgs::LaserScan&);
	void callbackBeacon(const mrpt_msgs::ObservationRangeBeacon&);
	void callbackRobotPose(const geometry_msgs::PoseWithCovarianceStamped&);
	void odometryForCallback(
		CObservationOdometry::Ptr&, const std_msgs::Header&);
	void callbackInitialpose(const geometry_msgs::PoseWithCovarianceStamped&);
	void callbackOdometry(const nav_msgs::Odometry&);
	void callbackMap(const nav_msgs::OccupancyGrid&);
	void updateMap(const nav_msgs::OccupancyGrid&);
	void publishTF();
	void publishPose();

   private:
	ros::NodeHandle nh_;
	bool first_map_received_;
	ros::Time time_last_input_;
	unsigned long long loop_count_;
	nav_msgs::GetMap::Response resp_;
	ros::Subscriber sub_init_pose_;
	ros::Subscriber sub_odometry_;
	std::vector<ros::Subscriber> sub_sensors_;
	ros::Subscriber sub_map_;
	ros::ServiceClient client_map_;
	ros::Publisher pub_particles_;
	ros::Publisher pub_map_;
	ros::Publisher pub_metadata_;
	ros::Publisher pub_pose_;
	ros::ServiceServer service_map_;
	tf::TransformListener tf_listener_;
	tf::TransformBroadcaster tf_broadcaster_;
	std::map<std::string, mrpt::poses::CPose3D> laser_poses_;
	std::map<std::string, mrpt::poses::CPose3D> beacon_poses_;

	// methods
	Parameters* param();
	void update();
	void updateSensorPose(std::string frame_id);

	void publishParticles();
	void useROSLogLevel();

	bool waitForTransform(
		mrpt::poses::CPose3D& des, const std::string& target_frame,
		const std::string& source_frame, const ros::Time& time,
		const ros::Duration& timeout,
		const ros::Duration& polling_sleep_duration = ros::Duration(0.01));
	bool mapCallback(
		nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);
	void publishMap();
	virtual bool waitForMap();
};

#endif  // MRPT_LOCALIZATION_NODE_H
