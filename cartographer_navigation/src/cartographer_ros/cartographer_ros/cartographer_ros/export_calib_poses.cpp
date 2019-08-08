//
// Created by galyean on 19-4-15.
//

#include <iomanip>
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/pose_graph.h"
#include <fstream>
#include <vector>
#include <sstream>
#include <Eigen/Geometry>
#include <cartographer/transform/transform_interpolation_buffer.h>
#include "cartographer_ros/time_conversion.h"


DEFINE_string(pbstream_filename, "",
              "Proto stream file containing the pose graph.");

namespace cartographer {
    struct CalibTime{
        double time_stamp;
        Eigen::Quaterniond rotation;
        Eigen::Vector3d translation;
        int tag_id_1;
        int tag_id_2;
    };

    namespace carto = ::cartographer;

    void ExportPbstream(const std::string& pbstream_filename,const std::string& calib_timestamp) {
        // firstly, read timestamp from calib_time.txt
        std::vector<CalibTime> calib_times;
        std::ifstream read_time(calib_timestamp);
        char buffer[1024];
        while(read_time.getline(buffer,1024)){
            CalibTime calib_time;
            std::stringstream ss;
            ss<<buffer;
            ss>>calib_time.time_stamp;
            ss>>calib_time.tag_id_1;
            ss>>calib_time.tag_id_2;
            calib_times.push_back(calib_time);
        }

        carto::io::ProtoStreamReader reader(pbstream_filename);
        carto::io::ProtoStreamDeserializer deserializer(&reader);
        carto::mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();

        std::cout<<"trajectory_id nums : "<<pose_graph_proto.trajectory().size()<<std::endl;
        //for (size_t trajectory_id = 1; trajectory_id < pose_graph_proto.trajectory().size();
        //   ++trajectory_id) {
        const carto::mapping::proto::Trajectory& trajectory_proto =
                pose_graph_proto.trajectory(0);
        const ::cartographer::transform::TransformInterpolationBuffer transform_interpolation_buffer(trajectory_proto);

        std::vector<CalibTime> calib_times_out;
        for(size_t i=0;i<calib_times.size();++i){
            ::ros::Time ros_time(double(calib_times[i].time_stamp));
            ::cartographer::common::Time car_time = ::cartographer_ros::FromRos(ros_time);
             if (!transform_interpolation_buffer.Has(car_time)) {
                std::cout << "car_time " << calib_times[i].time_stamp << " time not valid" << std::endl;
             }else{
                 ::cartographer::transform::Rigid3d T_G_I = transform_interpolation_buffer.Lookup(car_time);
                 CalibTime out_calib = calib_times[i];
                 out_calib.rotation = T_G_I.rotation();
                 out_calib.translation = T_G_I.translation();
                 calib_times_out.push_back(out_calib);
             }
        }

        std::ofstream write_result("/media/galyean/DATA/wheel_speed0/2019.05.27.20.16.46/lidar_wheel_odo.txt");
        for(size_t i=0;i!=calib_times_out.size();++i){
            write_result<<calib_times_out[i].time_stamp<<' '
            <<calib_times_out[i].rotation.x()
            <<' '<<calib_times_out[i].rotation.y()<<' '<<calib_times_out[i].rotation.z()<<' '
            <<calib_times_out[i].rotation.w()
            <<' '<<calib_times_out[i].translation.transpose()<<std::endl;
        }
        write_result.close();

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
    ::cartographer::ExportPbstream(FLAGS_pbstream_filename,"/media/galyean/DATA/wheel_speed0/2019.05.27.20.16.46/wheel_odo.txt");
}
