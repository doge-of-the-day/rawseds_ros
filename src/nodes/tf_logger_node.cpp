#include <rawseeds_ros/nodes/tf_logger_node.h>

#include <rawseeds_ros/tools/dataset_aligner.hpp>

namespace rawseeds_ros {

TFLoggerNode::~TFLoggerNode() {
  if (csv_output_file_aligned_.has_value()) {
    if (csv_2d_.has_value()) {
      const auto path = csv_2d_.value().path();

      csv_2d_.reset();
      DatasetAligner<cslibs_math_2d::Transform2d>::align(
          csv_groundtruth_file_.value(), path,
          csv_output_file_aligned_.value());
    }
    if (csv_3d_.has_value()) {
      const auto path = csv_2d_.value().path();
      csv_3d_.reset();
      DatasetAligner<cslibs_math_3d::Transform3d>::align(
          csv_groundtruth_file_.value(), path,
          csv_output_file_aligned_.value());
    }
  }
}

bool TFLoggerNode::setup() {
  const auto csv_output_file = nh_.param<std::string>("csv_output_file", "");
  const auto csv_output_file_aligned =
      nh_.param<std::string>("csv_output_file_aligned", "");
  const auto csv_groundtruth_file =
      nh_.param<std::string>("csv_groundtruth_file", "");
  const auto log_3d = nh_.param<bool>("log_3d", false);
  const auto queue_size = nh_.param<int>("queue_size", 10);
  moving_frame_ = nh_.param<std::string>("moving_frame", "base_link");
  fixed_frame_ = nh_.param<std::string>("fixed_frame", "map");

  if (csv_output_file == "") {
    ROS_ERROR_STREAM("Output file parameter was not set!");
    return false;
  }

  if (csv_output_file_aligned != "") {
    csv_output_file_aligned_ = csv_output_file_aligned;
  }
  if (csv_groundtruth_file != "") {
    csv_groundtruth_file_ = csv_groundtruth_file;
  } else {
    csv_output_file_aligned_.reset();
  }

  tf_listener_.reset(new cslibs_math_ros::tf::TFListener);

  if (log_3d) {
    sub_tf_ = nh_.subscribe("/tf", queue_size, &TFLoggerNode::update3D, this);
    csv_3d_.emplace(csv_output_file);
  } else {
    sub_tf_ = nh_.subscribe("/tf", queue_size, &TFLoggerNode::update2D, this);
    csv_2d_.emplace(csv_output_file);
  }

  return true;
}

void TFLoggerNode::run() { ros::spin(); }

void TFLoggerNode::update2D(const tf::tfMessage::ConstPtr& tf_msg) {
  ros::Time stamp;
  if (getMovingFrameStamp(tf_msg, stamp)) {
    cslibs_math_2d::Transform2d transform;
    if (tf_listener_->lookupTransform(fixed_frame_, moving_frame_, stamp,
                                      transform, ros::Duration{0.1})) {
      csv_2d_.value().write(stamp.toSec(), transform.tx(), transform.ty(),
                            transform.yaw());
      last_stamp_ = stamp;
    }
  }
}

void TFLoggerNode::update3D(const tf::tfMessage::ConstPtr& tf_msg) {
  ros::Time stamp;
  if (getMovingFrameStamp(tf_msg, stamp)) {
    cslibs_math_3d::Transform3d transform;
    if (tf_listener_->lookupTransform(fixed_frame_, moving_frame_, stamp,
                                      transform, ros::Duration{0.1})) {
      csv_3d_.value().write(stamp.toSec(), transform.tx(), transform.ty(),
                            transform.tz(), transform.roll(), transform.pitch(),
                            transform.yaw());
      last_stamp_ = stamp;
    }
  }
}

bool TFLoggerNode::getMovingFrameStamp(const tf::tfMessage::ConstPtr& tf_msg,
                                       ros::Time& stamp) {
  for (const auto& transform : tf_msg->transforms) {
    if (transform.header.frame_id == moving_frame_ ||
        transform.child_frame_id == moving_frame_) {
      stamp = transform.header.stamp;
      return true;
    }
  }
  return false;
}

}  // namespace rawseeds_ros

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rawseeds_tf_logger_node");

  rawseeds_ros::TFLoggerNode node;
  if (node.setup()) {
    node.run();
  }

  return 0;
}
