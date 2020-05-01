#include "tf_logger_node.h"

namespace rawseeds_ros {

bool TFLoggerNode::setup() {
  const auto out_filename = nh_.param<std::string>("csv_output_file", "");
  const auto log_3d = nh_.param<bool>("log_3d", false);
  const auto queue_size = nh_.param<int>("queue_size", 10);
  moving_frame_ = nh_.param<std::string>("moving_frame", "/base_link");
  fixed_frame_ = nh_.param<std::string>("fixed_frame", "/map");

  if (out_filename == "") {
    ROS_ERROR_STREAM("Output file parameter was not set!");
    return false;
  }

  tf_listener_.reset(new cslibs_math_ros::tf::TFListener);

  if (log_3d) {
    sub_tf_ = nh_.subscribe("/tf", queue_size, &TFLoggerNode::update3D, this);
    csv_3d_.emplace(csv_writer_3d_t{out_filename});
  } else {
    sub_tf_ = nh_.subscribe("/tf", queue_size, &TFLoggerNode::update2D, this);
    csv_2d_.emplace(csv_writer_2d_t{out_filename});
  }

  return true;
}

void TFLoggerNode::run() { ros::spin(); }

void TFLoggerNode::update2D(const tf::tfMessage::ConstPtr& tf_msg) {
  ros::Time stamp;
  if (getMovingFrameStamp(tf_msg, stamp)) {
    /// query and write out

    last_stamp_ = stamp;
  }
}

void TFLoggerNode::update3D(const tf::tfMessage::ConstPtr& tf_msg) {
  ros::Time stamp;
  if (getMovingFrameStamp(tf_msg, stamp)) {
    /// query and write out

    last_stamp_ = stamp;
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

int main(int argc, char * argv[]) {
  ros::init(argc, argv, "rawseeds_tf_logger_node");

  rawseeds_ros::TFLoggerNode node;
  if (node.setup()) {
    node.run();
  }

  return 0;
}
