#ifndef __SFM_H__
#define __SFM_H__
#include <vector>
#include "Sfm/common.h"
#include "Sfm/SfmFeatures.h"

namespace xb
{

class Sfm
{
public:
    void setImagePath(const std::string path) { mImagePath_ = path; }
    bool ReadImages();
    Sfm(/* args */);
    ~Sfm();

private:
    void extractFeatures();
    void createFeatureMatchMatrix();
    void findBaselineTriangulation();

private:
    std::string mImagePath_;
    std::vector<Features> mImageFeatures_;
    std::vector<cv::Mat> mImages_;
    MatchMatrix mFeatureMatchMatrix;

    SfmFeatures mFeatures_;

};

} // namespace xb

#endif