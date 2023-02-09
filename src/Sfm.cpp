#include "Sfm/Sfm.h"

#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
using namespace xb;

Sfm::Sfm()
{}

Sfm::~Sfm() {}

bool Sfm::ReadImages()
{
    if(mImagePath_ == "")   return false;
    std::vector<cv::String> path;
    cv::glob(mImagePath_, path);

    for(const auto& it : path)
    {
        mImages_.push_back(cv::imread(it));
    }
    return true;
}

void Sfm::extractFeatures()
{
    for(auto& it : mImages_)
    {
        mImageFeatures_.push_back(mFeatures_.extractFeatures(it));
    }
}

void Sfm::createFeatureMatchMatrix() {
    const size_t numImages = mImages_.size();
    mFeatureMatchMatrix.resize(numImages, std::vector<Matching>(numImages));

    //prepare image pairs to match concurrently
    std::vector<ImagePair> pairs;
    for (size_t i = 0; i < numImages; i++) {
        for (size_t j = i + 1; j < numImages; j++) {
            pairs.push_back({ i, j });
        }
    }

    std::vector<std::thread> threads;

    //find out how many threads are supported, and how many pairs each thread will work on
    const int numThreads = std::thread::hardware_concurrency() - 1;
    const int numPairsForThread = (numThreads > pairs.size()) ? 1 : (int)ceilf((float)(pairs.size()) / numThreads);

    //invoke each thread with its pairs to process (if less pairs than threads, invoke only #pairs threads with 1 pair each)
    for (size_t threadId = 0; threadId < MIN(numThreads, pairs.size()); threadId++) {
        threads.push_back(std::thread([&, threadId] {
            const int startingPair = numPairsForThread * threadId;

            for (int j = 0; j < numPairsForThread; j++) {
                const int pairId = startingPair + j;
                if (pairId >= pairs.size()) { //make sure threads don't overflow the pairs
                    break;
                }
                const ImagePair& pair = pairs[pairId];

                mFeatureMatchMatrix[pair.first][pair.second] = SfmFeatures::matchFeatures(mImageFeatures_[pair.first], mImageFeatures_[pair.second]);
            }
        }));
    }

    //wait for threads to complete
    for (auto& t : threads) {
        t.join();
    }
}

void Sfm::findBaselineTriangulation()
{
    
}