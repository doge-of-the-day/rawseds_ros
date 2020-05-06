#ifndef RAWSEEDS_ROS_TF_LOGGER_NODE_H
#define RAWSEEDS_ROS_TF_LOGGER_NODE_H

#include <cslibs_utility/logger/csv_writer.hpp>
#include <cslibs_math_ros/tf/tf_listener.hpp>
#include <ros/ros.h>
#include <optional>
#include <tf/tfMessage.h>
namespace rawseeds_ros {

class TFLoggerNode {
 public:
  TFLoggerNode() = default;
  ~TFLoggerNode();

  bool setup();
  void run();

 private:
  /// for writing 2d pose data
  using csv_writer_2d_t =
      cslibs_utility::logger::CSVWriter<double, double, double, double>;
  /// for writing 3d pose data
  using csv_writer_3d_t =
      cslibs_utility::logger::CSVWriter<double, double, double, double, double,
                                        double, double>;

  ros::NodeHandle nh_{"~"};
  ros::Subscriber sub_tf_;
  ros::Time last_stamp_{ros::TIME_MIN};
  std::string moving_frame_;
  std::string fixed_frame_;
  std::unique_ptr<cslibs_math_ros::tf::TFListener> tf_listener_;

  std::optional<csv_writer_2d_t> csv_2d_;
  std::optional<csv_writer_3d_t> csv_3d_;
  std::optional<std::string> csv_output_file_aligned_;
  std::optional<std::string> csv_groundtruth_file_;

  void update2D(const tf::tfMessage::ConstPtr &tf_msg);
  void update3D(const tf::tfMessage::ConstPtr &tf_msg);
  bool getMovingFrameStamp(const tf::tfMessage::ConstPtr &tf_msg, ros::Time &stamp);

};
}  // namespace rawseeds_ros

#endif // RAWSEEDS_ROS_TF_LOGGER_NODE_H