#ifndef __UTIL_H__
#define __UTIL_H__
#include "CameraCalibration/common.h"

namespace xb
{

Eigen::Matrix3f getCoincMatrix(const cv::RotatedRect& ret)
{
    double x = ret.center.x;
    double y = ret.center.y;
    double a = ret.size.height / 2;
    double b = ret.size.width / 2;
    double theta = ret.angle / 180 * M_PI;
    double A = pow(a, 2) * pow(sin(theta), 2) + pow(b, 2) * pow(cos(theta), 2);
    double B = 2 * (pow(b, 2) - pow(a, 2)) * sin(theta) * cos(theta);
    double C = pow(a, 2) * pow(cos(theta), 2) + pow(b, 2) * pow(sin(theta), 2);
    double D = -2 * A * x - B * y;
    double E = -B * x - 2 * C * y;
    double F = A * pow(x, 2) + B * x * y + C * pow(y, 2) - pow(a, 2) * pow(b, 2);
    Eigen::Matrix3f paramter;
    paramter << A, B / 2, D / 2,
            B / 2, C, E / 2,
            D / 2, E / 2, F;
    return paramter;
}

Eigen::Vector3f getLineVec(cv::Vec4f& center_line)
{
    double a = center_line[1] / center_line[0];
    double b = -1;
    double c = center_line[3] - a * center_line[2];
    Eigen::Vector3f vec(a, b, c);
    return vec;
}

double cross_ratio_axis(double big_axis, double small_axis)
{
    double AC = big_axis / 2 + small_axis / 2;
    double AD = big_axis;
    double BC = small_axis;
    double BD = big_axis / 2 + small_axis / 2;
    return (AC * BD) / (BC * AD);
}

double Point2Line_dist(Eigen::Vector3f& line, cv::Point2f& point)
{
    return abs(line[0] * point.x + line[1] * point.y + line[2]) / sqrt(line[0] * line[0] + line[1] * line[1]);
}

double Point2Line_cost(Eigen::Vector3f& line, cv::Point2f& point)
{
    return pow(Point2Line_dist(line, point), 2);
}

double P2P_dist(const cv::Point2f& p1, const cv::Point2f& p2)
{
    return cv::norm(p1 - p2);
}

bool isOnLine(Eigen::Vector3f& line, cv::Point2f& point)
{
    double error = 100;
    if(Point2Line_dist(line, point) < error)    return true;
    return false;
}

// std::vector<Eigen::Vector3f> getCirclePoints(int radius, int number)
// {
    
// }

} // namespace xb

#endif