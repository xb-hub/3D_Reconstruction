#ifndef __EXTRACTFEATURES_H__
#define __EXTRACTFEATURES_H__
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "Sfm/common.h"

namespace xb
{
    
class SfmFeatures
{
private:
    /* data */
public:
    SfmFeatures(/* args */);
    ~SfmFeatures();

    Features extractFeatures(const cv::Mat& image);
    static Matching matchFeatures(const Features& featuresLeft, const Features& featuresRight);


private:
    cv::Ptr<cv::Feature2D>         mDetector_;
    cv::Ptr<cv::DescriptorMatcher> mMatcher_;
    const double NN_MATCH_RATIO = 0.8f; 

};

} // namespace xb

#endif