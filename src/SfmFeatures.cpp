#include "Sfm/SfmFeatures.h"
using namespace xb;

SfmFeatures::SfmFeatures()
{
    // mDetector_ = cv::ORB::create();
    mDetector_ = cv::SIFT::create(500);
    mMatcher_ = cv::FlannBasedMatcher matcher; 
}

Features::~Features() {}

Features SfmFeatures::extractFeatures(const cv::Mat& image)
{
    Features features;
    mDetector_->detectAndCompute(image, cv::noArray(), features.keyPoints, features.descriptors);
    return features;
}

Matching SfmFeatures::matchFeatures(const Features& featuresLeft, const Features& featuresRight)
{
    std::vector<Matching> initialMatching;

    mMatcher_->knnMatch(featuresLeft.descriptors, featuresRight.descriptors, initialMatching, 2);

    //prune the matching using the ratio test
    Matching prunedMatching;
    for(unsigned i = 0; i < initialMatching.size(); i++) {
        if(initialMatching[i][0].distance < NN_MATCH_RATIO * initialMatching[i][1].distance) {
            prunedMatching.push_back(initialMatching[i][0]);
        }
    }

    return prunedMatching;
}