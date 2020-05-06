#ifndef RAWSEEDS_ROS_DATASET_ALIGNER_HPP
#define RAWSEEDS_ROS_DATASET_ALIGNER_HPP

#include <cslibs_math_2d/linear/transform.hpp>
#include <cslibs_math_3d/linear/transform.hpp>
#include <cslibs_time/time.hpp>
#include <cslibs_utility/logger/csv_reader.hpp>
#include <cslibs_utility/logger/csv_writer.hpp>
#include <map>

namespace rawseeds_ros {
namespace dataset_aligner {
template <typename Transform_T>
struct CSVReader {
  static_assert(!std::is_same<Transform_T, Transform_T>::value,
                "CSVReader trait has not been overriden.");
};
template <typename Transform_T>
struct CSVWriter {
  static_assert(!std::is_same<Transform_T, Transform_T>::value,
                "CSVWriter trait has not been overriden.");
};

template <typename Transform_T>
inline void write(const Transform_T &t, const cslibs_time::Time &s,
                  typename CSVWriter<Transform_T>::type &w) {}

template <typename Transform_T>
inline void read(typename CSVReader<Transform_T>::type &r,
                 std::map<cslibs_time::Time, Transform_T,
                          std::less<cslibs_time::Time>,
                          Eigen::aligned_allocator<std::pair<const cslibs_time::Time, Transform_T>>> &data) {}
}  // namespace dataset_aligner

template <typename Transform_T>
class DatasetAligner {
 public:
  static void align(const std::string &path_groundtruth,
                    const std::string &path_original,
                    const std::string &path_aligned) {
    reader_t reader_ground_truth{path_groundtruth};
    reader_t reader_dataset{path_original};
    writer_t writer_dataset{path_aligned};

    dataset_t dataset;
    dataset_aligner::read(reader_dataset, dataset);

    for (const auto &e : reader_ground_truth.getData()) {
      const auto stamp = cslibs_time::Time{std::get<0>(e)};
      Transform_T transform;
      if (query(stamp, dataset, transform)) {
        dataset_aligner::write(transform, stamp, writer_dataset);
      }
    }
  }

 private:
  using reader_t = typename dataset_aligner::CSVReader<Transform_T>::type;
  using writer_t = typename dataset_aligner::CSVWriter<Transform_T>::type;
  using dataset_t = typename dataset_aligner::CSVReader<Transform_T>::dataset_t;

  inline static bool query(const cslibs_time::Time &stamp,
                           const dataset_t &dataset, Transform_T &transform) {
    const auto none = dataset.end();
    auto ta = none;
    auto tb = none;
    for (auto it = dataset.begin(); it != dataset.end(); ++it) {
      if (it->first < stamp) {
        ta = it;
      }
      if (it->first >= stamp) {
        tb = it;
        break;
      }
    }

    if (tb == none) {
      return false;
    }
    if (ta == none) {
      ta = tb;
    }

    const auto duration = tb->first - ta->first;
    const auto ratio = duration.seconds() == 0.0 ? 0.0 :
        (stamp - ta->first).seconds() / duration.seconds();

    std::cout << ratio << " " << ta->second << " " << tb->second << std::endl;
    transform = ta->second.interpolate(tb->second, ratio);
    return true;
  }
};

namespace dataset_aligner {
template <>
struct CSVWriter<cslibs_math_2d::Transform2d> {
  using type =
      cslibs_utility::logger::CSVWriter<double, double, double, double>;
};
template <>
struct CSVReader<cslibs_math_2d::Transform2d> {
  using type =
      cslibs_utility::logger::CSVReader<double, double, double, double>;
  using dataset_entry_t = std::pair<const cslibs_time::Time, cslibs_math_2d::Transform2d>;
  using dataset_t = std::map<cslibs_time::Time, cslibs_math_2d::Transform2d, std::less<cslibs_time::Time>, Eigen::aligned_allocator<dataset_entry_t>>;
};

template <>
inline void write(const cslibs_math_2d::Transform2d &t,
                  const cslibs_time::Time &s,
                  typename CSVWriter<cslibs_math_2d::Transform2d>::type &w) {
  w.write(s.seconds(), t.tx(), t.ty(), t.yaw());
}

template <>
inline void read<cslibs_math_2d::Transform2d>(
    typename CSVReader<cslibs_math_2d::Transform2d>::type &r,
    typename CSVReader<cslibs_math_2d::Transform2d>::dataset_t &data) {
  for (auto &e : r.getData()) {
    data[cslibs_time::Time{std::get<0>(e)}] = cslibs_math_2d::Transform2d{
        std::get<1>(e), std::get<2>(e), std::get<3>(e)};
  }
}

template <>
struct CSVWriter<cslibs_math_3d::Transform3d> {
  using type = cslibs_utility::logger::CSVWriter<double, double, double, double,
                                                 double, double, double>;
};
template <>
struct CSVReader<cslibs_math_3d::Transform3d> {
  using type = cslibs_utility::logger::CSVReader<double, double, double, double,
                                                 double, double, double>;
  using dataset_entry_t = std::pair<const cslibs_time::Time, cslibs_math_3d::Transform3d>;
  using dataset_t = std::map<cslibs_time::Time, cslibs_math_3d::Transform3d, std::less<cslibs_time::Time>, Eigen::aligned_allocator<dataset_entry_t>>;
};

template <>
inline void write<cslibs_math_3d::Transform3d>(
    const cslibs_math_3d::Transform3d &t, const cslibs_time::Time &s,
    typename CSVWriter<cslibs_math_3d::Transform3d>::type &writer) {}

template <>
inline void read<cslibs_math_3d::Transform3d>(
    typename CSVReader<cslibs_math_3d::Transform3d>::type &r,
    typename CSVReader<cslibs_math_3d::Transform3d>::dataset_t &data) {
  for (auto &e : r.getData()) {
    data[cslibs_time::Time{std::get<0>(e)}] = cslibs_math_3d::Transform3d{
        std::get<1>(e), std::get<2>(e), std::get<3>(e),
        std::get<4>(e), std::get<5>(e), std::get<6>(e)};
  }
}

}  // namespace dataset_aligner
}  // namespace rawseeds_ros
#endif