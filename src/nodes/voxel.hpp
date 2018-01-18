#ifndef RAWSEEDS_ROS_VOXEL_HPP
#define RAWSEEDS_ROS_VOXEL_HPP

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree.hpp>

#include <tf/tf.h>

namespace cis = cslibs_indexed_storage;

namespace rawseeds_ros {
class Voxel {
public:
    using index_t = std::array<int, 3>;
    using point_t = tf::Point;
    using color_t = tf::Point;

    inline Voxel() :
        n_(1),
        n_1_(0)
    {
    }

    inline Voxel(const point_t &pt,
                const color_t &c) :
        n_(2),
        n_1_(1),
        mean_(pt),
        color_(c)
    {
    }

    inline virtual ~Voxel() = default;

    inline Voxel(const Voxel &other) :
        mean_(other.mean_),
        color_(other.color_)
    {
    }

   inline  Voxel(Voxel &&other) :
        mean_(std::move(other.mean_)),
        color_(std::move(other.color_))
    {
    }

    inline Voxel& operator = (const Voxel &other)
    {
        mean_ = other.mean_;
        color_  = other.color_;
        return *this;
    }

    inline Voxel& operator = (Voxel &&other)
    {
        mean_ = std::move(other.mean_);
        color_ = std::move(other.color_);
        return *this;
    }

    inline point_t const & mean() const
    {
        return mean_;
    }

    inline color_t const & color() const
    {
        return color_;
    }

    inline void merge(const Voxel &other)
    {
        const std::size_t   _n    = n_1_ + other.n_1_;
        const point_t       _pt = (mean_ * n_1_ + other.mean_ * other.n_1_) / static_cast<float>(_n);
        const color_t       _c  = (color_ * n_1_ + other.color_ * other.n_1_) / static_cast<float>(_n);
        n_                        = _n + 1;
        n_1_                      = _n;
        mean_                       = _pt;
        color_                        = _c;
    }

private:
    std::size_t n_;
    std::size_t n_1_;

    point_t     mean_;
    color_t     color_;

};
using VoxelGrid = cis::Storage<Voxel, Voxel::index_t, cis::backend::kdtree::KDTree>;
}
#endif // RAWSEEDS_ROS_VOXEL_HPP
