#ifndef __COMMON_H__
#define __COMMON_H__
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace xb
{

struct Intrinsics_Paramter
{
    double fx, fy, cx, cy, k1, k2;
};

struct Contour
{
    std::vector<cv::Point2f> points;
    std::vector<float> direction;
    std::vector<float> response;
};

struct EllipseContour
{
    cv::RotatedRect retval;
    std::vector<Eigen::Vector3f> poles;

    EllipseContour(const cv::RotatedRect& ret) :
                retval(ret)
    {}
};

} // namespace xb

#endif

