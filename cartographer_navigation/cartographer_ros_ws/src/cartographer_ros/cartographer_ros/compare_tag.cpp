//
// Created by galyean on 19-4-3.
//

#include <iomanip>
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer_ros/time_conversion.h"
#include <fstream>
#include <Eigen/Geometry>
#include <cartographer/transform/transform_interpolation_buffer.h>
#include<iostream>
DEFINE_string(pbstream_filename, "",
              "Proto stream file containing the pose graph.");
DEFINE_string(tag_pose_name, "",
              "tag pose file contains pose from tag.");
struct Pose{
    Pose():time_stamp(0){};
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation;
    long time_stamp;
};
namespace cartographer_ros {

    namespace carto = ::cartographer;

    void ExportPbstream(const std::string& pbstream_filename,const std::string& tag_pose_name) {
        carto::io::ProtoStreamReader reader(pbstream_filename);
        carto::io::ProtoStreamDeserializer deserializer(&reader);


        carto::mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();

        std::cout<<"trajectory_id nums : "<<pose_graph_proto.trajectory().size()<<std::endl;
        //for (size_t trajectory_id = 1; trajectory_id < pose_graph_proto.trajectory().size();
        //   ++trajectory_id) {
        const carto::mapping::proto::Trajectory& trajectory_proto =
                pose_graph_proto.trajectory(0);
                auto& node = trajectory_proto.node(0);
                Eigen::Vector3d initial_position(node.pose().translation().x(),
                node.pose().translation().y(),
                node.pose().translation().z());
        double total_length=0;
         for (int i = 1; i < trajectory_proto.node_size(); ++i) {
                auto& node = trajectory_proto.node(i);
                Eigen::Vector3d now_position(node.pose().translation().x(),
                node.pose().translation().y(),
                node.pose().translation().z());
                total_length+=(now_position-initial_position).norm();
                initial_position = now_position;
            }
            std::cout<<"total length : "<<total_length<<std::endl;
        const ::cartographer::transform::TransformInterpolationBuffer transform_interpolation_buffer(trajectory_proto);

        std::vector<Pose> tag_poses;
        std::ifstream read_tag_pose(tag_pose_name);
        char buffer[1024];
        while(read_tag_pose.getline(buffer,1024)){
            std::stringstream ss;
            ss<<buffer;
            Pose tag_pose;
                    ss >>tag_pose.time_stamp;
        double q[4];
        for(size_t i=0;i<4;i++){
            ss>>q[i];
        }
        Eigen::Quaterniond q_x(0,1,0,0);
        tag_pose.rotation = Eigen::Quaterniond(q[0],q[1],q[2],q[3]);
        for(size_t i=0;i<3;i++){
            ss>>q[i];
        }
        tag_pose.translation = Eigen::Vector3d(q[0],q[1],q[2]);
            tag_poses.push_back(tag_pose);
        }
        std::cout<<"tag pose size: "<<tag_poses.size()<<std::endl;
        std::vector<Pose> cartographer_poses;
        cartographer_poses.resize(tag_poses.size());
        size_t start_id =0;
        for(size_t i=0;i<tag_poses.size();++i){
            ::ros::Time ros_time(double(tag_poses[i].time_stamp/1.e9));
            ::cartographer::common::Time car_time = ::cartographer_ros::FromRos(ros_time);
             if (!transform_interpolation_buffer.Has(car_time)) {
            std::cout << "car_time " << i << " time not valid" << std::endl;
            start_id = i+1;
            if(start_id>tag_poses.size()-1){
                std::cerr<<" start id is wrong!"<<std::endl;
                exit(1);
            }
            continue;
          }
            ::cartographer::transform::Rigid3d T_I_G = transform_interpolation_buffer.Lookup(car_time);
            Pose car_pose;
            
            car_pose.rotation = T_I_G.rotation();
            car_pose.translation = T_I_G.translation();
            cartographer_poses[i]=car_pose;
        }
            Eigen::Matrix4d T_GTag_imu = Eigen::Matrix4d::Identity();
    T_GTag_imu.block(0,0,3,3) = tag_poses[start_id].rotation.toRotationMatrix();
    T_GTag_imu.block(0,3,3,1) = tag_poses[start_id].translation;
    Eigen::Matrix4d T_imu_GCar = Eigen::Matrix4d::Identity();
    T_imu_GCar.block(0,0,3,3) = cartographer_poses[start_id].rotation.toRotationMatrix();
    T_imu_GCar.block(0,3,3,1) = cartographer_poses[start_id].translation;

    Eigen::Matrix4d T_GTag_G_car = T_imu_GCar*T_GTag_imu;

    for(size_t i=start_id;i<cartographer_poses.size();++i){
        Eigen::Matrix4d T_GTag_imu = Eigen::Matrix4d::Identity();
        T_GTag_imu.block(0,0,3,3) = tag_poses[i].rotation.toRotationMatrix();
        T_GTag_imu.block(0,3,3,1) = tag_poses[i].translation;
        Eigen::Matrix4d T_align = T_GTag_G_car*T_GTag_imu.inverse();
        std::cout<<std::setprecision(15)<<tag_poses[i].time_stamp<<"    "<<T_align.block(0,3,3,1).transpose()<<"              "<<cartographer_poses[i].translation.transpose()<<std::endl;
        Eigen::Matrix3d rotation_align = T_align.block(0,0,3,3);
        Eigen::Quaterniond q_(rotation_align);

        std::cout<<q_.angularDistance(cartographer_poses[i].rotation)*180/3.1415926<<std::endl;
/*        Eigen::Quaterniond q_i_1_tag = tag_poses[0].rotation*tag_poses[i].rotation.conjugate();
        Eigen::Quaterniond q_i_1_car = (cartographer_poses[i].rotation.conjugate()*cartographer_poses[0].rotation).conjugate();
        Eigen::Vector3d t_i_1_tag = tag_poses[0].translation-q_i_1_tag.toRotationMatrix()*tag_poses[i].translation;
        Eigen::Vector3d t_i_1_car = cartographer_poses[0].rotation.conjugate()*(cartographer_poses[i].translation-cartographer_poses[0].translation);
        std::cout<<"delta rotation: "<<(q_i_1_car*q_i_1_tag.conjugate()).coeffs().transpose()<<std::endl;
        std::cout<<"delta translation: "<<(t_i_1_car-t_i_1_tag).transpose()<<std::endl;*/
    }


    }

}


int main(int argc, char** argv) {
    FLAGS_alsologtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::SetUsageMessage(
            "\n\n"
            "This is a function to get "
            "last trajectory from a pbstream file.\n");
    google::ParseCommandLineFlags(&argc, &argv, true);
    CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";
    CHECK(!FLAGS_tag_pose_name.empty()) << "-tag_pose_name is missing.";
    ::cartographer_ros::ExportPbstream(FLAGS_pbstream_filename,FLAGS_tag_pose_name);
}


