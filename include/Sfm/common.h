#ifndef __COMMON_H__
#define __COMMOM_H_
#include <opencv2/opencv.hpp>

namespace xb
{

typedef std::vector<cv::KeyPoint> Keypoints;
typedef std::vector<cv::Point2f>  Points2f;
typedef std::vector<cv::DMatch> Matching;
typedef std::pair<size_t, size_t> ImagePair;
typedef std::vector<std::vector<Matching> > MatchMatrix;

struct Features {
    Keypoints keyPoints;
    // Points2f  points;
    cv::Mat   descriptors;
};



} // namespace xb

#endif