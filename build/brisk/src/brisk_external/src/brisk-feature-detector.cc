/*
 Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger, Simon Lynen and Margarita Chli.

 Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 All rights reserved.

 This is the Author's implementation of BRISK: Binary Robust Invariant
 Scalable Keypoints [1]. Various (partly unpublished) extensions are provided,
 some of which are described in [2].

 [1] Stefan Leutenegger, Margarita Chli and Roland Siegwart. BRISK: Binary
     Robust Invariant Scalable Keypoints. In Proceedings of the IEEE
     International Conference on Computer Vision (ICCV), 2011.

 [2] Stefan Leutenegger. Unmanned Solar Airplanes: Design and Algorithms for
     Efficient and Robust Autonomous Operation. Doctoral dissertation, 2014.

 This file is part of BRISK.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <stdexcept>

#include <brisk/brisk-feature-detector.h>
#include <brisk/internal/brisk-scale-space.h>

namespace {
void RemoveInvalidKeyPoints(const cv::Mat& mask,
                            std::vector<cv::KeyPoint>* keypoints) {
  if(keypoints==nullptr){
    throw std::runtime_error("keypoints NULL");
  }
  if (mask.empty())
    return;

  std::function<bool(const cv::KeyPoint& key_pt)> masking =
      [&mask](const cv::KeyPoint& key_pt)->bool {
        const float& keypoint_x = cv::KeyPointX(key_pt);
        const float& keypoint_y = cv::KeyPointY(key_pt);
        return mask.at<unsigned char>(static_cast<int>(keypoint_y + 0.5f),
            static_cast<int>(keypoint_x + 0.5f)) == 0;
  };

  keypoints->erase(
      std::remove_if(keypoints->begin(), keypoints->end(),
                     masking), keypoints->end());
}
}  // namespace

namespace brisk {
BriskFeatureDetector::BriskFeatureDetector(int threshold, int octaves,
                                           bool suppressScaleNonmaxima) {
  _threshold = threshold;
  _octaves = octaves;
  _suppressScaleNonmaxima = suppressScaleNonmaxima;
}

void BriskFeatureDetector::detectImpl(const cv::Mat& image,
                                      std::vector<cv::KeyPoint>& keypoints,
                                      const cv::Mat& mask) const {
  if(!image.isContinuous() || image.channels()!=1 || image.elemSize()!=1) {
    throw std::runtime_error("BRISK requires continuous 1-channel 8-bit images");
  }
  keypoints.clear();
  brisk::BriskScaleSpace briskScaleSpace(_octaves, _suppressScaleNonmaxima);
  briskScaleSpace.ConstructPyramid(image, _threshold);
  briskScaleSpace.GetKeypoints(&keypoints);
  RemoveInvalidKeyPoints(mask, &keypoints);
}

void BriskFeatureDetector::ComputeScale(
    const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints) const {
  BriskScaleSpace briskScaleSpace(_octaves, _suppressScaleNonmaxima);
  briskScaleSpace.ConstructPyramid(image, _threshold, 0);
  briskScaleSpace.GetKeypoints(&keypoints);
}
}  // namespace brisk
