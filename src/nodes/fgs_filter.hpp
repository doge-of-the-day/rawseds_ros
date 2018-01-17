#ifndef FGS_FILTER_HPP
#define FGS_FILTER_HPP

#include <opencv2/core.hpp>

namespace cv
{
namespace ximgproc
{

/** @brief Interface for implementations of Fast Global Smoother filter.
For more details about this filter see @cite Min2014 and @cite Farbman2008 .
*/
class CV_EXPORTS_W FastGlobalSmootherFilter : public Algorithm
{
public:
    /** @brief Apply smoothing operation to the source image.
    @param src source image for filtering with unsigned 8-bit or signed 16-bit or floating-point 32-bit depth and up to 4 channels.
    @param dst destination image.
    */
    CV_WRAP virtual void filter(InputArray src, OutputArray dst) = 0;
};

/** @brief Factory method, create instance of FastGlobalSmootherFilter and execute the initialization routines.
@param guide image serving as guide for filtering. It should have 8-bit depth and either 1 or 3 channels.
@param lambda parameter defining the amount of regularization
@param sigma_color parameter, that is similar to color space sigma in bilateralFilter.
@param lambda_attenuation internal parameter, defining how much lambda decreases after each iteration. Normally,
it should be 0.25. Setting it to 1.0 may lead to streaking artifacts.
@param num_iter number of iterations used for filtering, 3 is usually enough.
For more details about Fast Global Smoother parameters, see the original paper @cite Min2014. However, please note that
there are several differences. Lambda attenuation described in the paper is implemented a bit differently so do not
expect the results to be identical to those from the paper; sigma_color values from the paper should be multiplied by 255.0 to
achieve the same effect. Also, in case of image filtering where source and guide image are the same, authors
propose to dynamically update the guide image after each iteration. To maximize the performance this feature
was not implemented here.
*/
CV_EXPORTS_W Ptr<FastGlobalSmootherFilter> createFastGlobalSmootherFilter(InputArray guide, double lambda, double sigma_color, double lambda_attenuation=0.25, int num_iter=3);

/** @brief Simple one-line Fast Global Smoother filter call. If you have multiple images to filter with the same
guide then use FastGlobalSmootherFilter interface to avoid extra computations.
@param guide image serving as guide for filtering. It should have 8-bit depth and either 1 or 3 channels.
@param src source image for filtering with unsigned 8-bit or signed 16-bit or floating-point 32-bit depth and up to 4 channels.
@param dst destination image.
@param lambda parameter defining the amount of regularization
@param sigma_color parameter, that is similar to color space sigma in bilateralFilter.
@param lambda_attenuation internal parameter, defining how much lambda decreases after each iteration. Normally,
it should be 0.25. Setting it to 1.0 may lead to streaking artifacts.
@param num_iter number of iterations used for filtering, 3 is usually enough.
*/
CV_EXPORTS_W void fastGlobalSmootherFilter(InputArray guide, InputArray src, OutputArray dst, double lambda, double sigma_color, double lambda_attenuation=0.25, int num_iter=3);

/** @brief Global image smoothing via L0 gradient minimization.
@param src source image for filtering with unsigned 8-bit or signed 16-bit or floating-point depth.
@param dst destination image.
@param lambda parameter defining the smooth term weight.
@param kappa parameter defining the increasing factor of the weight of the gradient data term.
For more details about L0 Smoother, see the original paper @cite xu2011image.
*/
CV_EXPORTS_W void l0Smooth(InputArray src, OutputArray dst, double lambda = 0.02, double kappa = 2.0);
//! @}
}
}
#endif // FGS_FILTER_HPP
