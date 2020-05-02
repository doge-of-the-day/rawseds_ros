#ifndef RAWSEEDS_ROS_DATASET_ALIGNER_HPP
#define RAWSEEDS_ROS_DATASET_ALIGNER_HPP

#include <cslibs_utility/logger/csv_reader.hpp>
#include <cslibs_math_2d/linear/transform.hpp>
#include <cslibs_math_3d/lineart

namespace rawseeds_ros {
class DatasetAligner {
 public:
  using Ptr = std::unique_ptr<DatasetAligner>;
  virtual ~DatasetAligner() = default;

};

// struct Deserializer2D {
//   using csv_reader_t =
//       cslibs_utility::logger::CSVReader<double, double, double, double>;

// };
// struct Deserializer3D {
//   using csv_reader_t =
//       cslibs_utility::logger::CSVReader<double, double, double, double, double,
//                                         double, double>;
//   using transform_t = cslibs_math_3d::linear::Transform3d;
// };
} // namespace rawseeds_ros
#endif