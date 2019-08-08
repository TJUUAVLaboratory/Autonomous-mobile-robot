//
// Created by galyean on 19-3-25.
//

#include <iomanip>
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/pose_graph.h"
#include <fstream>
#include <Eigen/Geometry>
#include <cartographer/transform/transform_interpolation_buffer.h>

DEFINE_string(pbstream_filename, "",
              "Proto stream file containing the pose graph.");

namespace cartographer_ros {

    namespace carto = ::cartographer;

    void ExportPbstream(const std::string& pbstream_filename) {
        carto::io::ProtoStreamReader reader(pbstream_filename);
        carto::io::ProtoStreamDeserializer deserializer(&reader);
        std::ofstream write_pose("/home/galyean/workspace/experiment_result/pose.txt");

        carto::mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();

        //std::cout<<"trajectory_id nums : "<<pose_graph_proto.trajectory().size()<<std::endl;
        //for (size_t trajectory_id = 1; trajectory_id < pose_graph_proto.trajectory().size();
          //   ++trajectory_id) {
            const carto::mapping::proto::Trajectory& trajectory_proto =
                    pose_graph_proto.trajectory(0);
        const ::cartographer::transform::TransformInterpolationBuffer transform_interpolation_buffer(trajectory_proto);

            for (int i = 0; i < trajectory_proto.node_size(); ++i) {
                const auto& node = trajectory_proto.node(i);
                Eigen::Quaterniond rotation(node.pose().rotation().w(),
                                            node.pose().rotation().x(),
                                            node.pose().rotation().y(),
                                            node.pose().rotation().z());
                double angle = 2*std::acos(node.pose().rotation().w());
                if(node.pose().rotation().w()<0){
                    angle=-1*(angle-3.1415926/2.f);
                }
                Eigen::Vector3d axis(node.pose().rotation().x(),
                                     node.pose().rotation().y(),
                                     node.pose().rotation().z());
                axis = axis/axis.norm();

                axis = -axis*angle;

                Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
                Eigen::Vector3d euler_rotation = rotation_matrix.eulerAngles(2,1,0);
                // node.pose() contains the pose...
                write_pose<<std::fixed<<std::setprecision(6)<<(double)node.timestamp()/1e7
                <<" "<<0<<" "<<0<<" "<<0<<" "
                <<(double)node.timestamp()/1e7<<" "
                <<euler_rotation[2]<<" "<<euler_rotation[1]<<" "<<euler_rotation[0]<<std::endl;
                std::cout<<"z position"<<"   "<<node.pose().translation().z()<<"\n";

                //std::cout<<node.pose().translation().x()<<std::endl;
                //std::cout<<node.pose().rotation().x()<<std::endl;
            }
        write_pose.close();


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
    ::cartographer_ros::ExportPbstream(FLAGS_pbstream_filename);
}
