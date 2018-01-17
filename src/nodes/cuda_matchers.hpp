#ifndef RAWSEEDS_ROS_CUDA_MATCHERS_HPP
#define RAWSEEDS_ROS_CUDA_MATCHERS_HPP

#include <opencv2/cudastereo.hpp>
#include <memory>

namespace rawseeds_ros {
namespace cuda {

class Matcher
{
public:
    using Ptr = std::shared_ptr<Matcher>;

    virtual ~Matcher()  = default;

    void compute(const cv::Mat &left,
                 const cv::Mat &right,
                 cv::Mat &disparity)
    {
        assert(left.type() == CV_8UC1);
        assert(right.type() == CV_8UC1);
        assert(left.rows == right.rows);
        assert(left.cols == right.cols);

        disparity_ = cv::cuda::GpuMat(left.rows, right.rows, CV_8UC1);

        left_.upload(left);
        right_.upload(right);
        doCompute();
        disparity_.download(disparity);
    }

protected:
    Matcher() = default;

    virtual void doCompute() = 0;

    cv::cuda::GpuMat left_;
    cv::cuda::GpuMat right_;
    cv::cuda::GpuMat disparity_;

};

template<typename cuda_matcher_t>
class MatcherImpl : public Matcher
{
public:
    MatcherImpl(const cv::Ptr<cuda_matcher_t> &m)
    {
        matcher_ = m;
    }
private:
    cv::Ptr<cuda_matcher_t> matcher_;

protected:
    void doCompute() override
    {
        matcher_->compute(left_, right_, disparity_);
    }
};
}
}

#endif // RAWSEEDS_ROS_CUDA_MATCHERS_HPP
