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

#include <fstream>  // NOLINT
#include <iomanip>
#include <iostream>  //NOLINT
#include <list>
#include <vector>
#include <map>
#include <stdexcept>

#include <brisk/brisk.h>
#include <brisk/brute-force-matcher.h>
#include <brisk/command-line-parser.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char ** argv)
{
  // Process command line args.
  brisk::CommandLineParser parser;
  parser.setOptions({
      { "image1", { brisk::CommandLineParser::ArgType::String, BRISK_IMAGE_PATH"/img1.ppm", "Path to first image." } },
      { "image2", { brisk::CommandLineParser::ArgType::String, BRISK_IMAGE_PATH"/img2.ppm", "Path to second image." } },
      { "detectorType", { brisk::CommandLineParser::ArgType::String, "BRISK", "The keypoint detector type." } },
      { "detectorThreshold", { brisk::CommandLineParser::ArgType::Num, "34.0", "The keypoint detector threshold." } },
      { "descriptorType", { brisk::CommandLineParser::ArgType::String, "BRISK2", "The keypoint descriptor type." } },
      { "matchingThreshold", { brisk::CommandLineParser::ArgType::Num, "0.0", "The threshold for the radius matcher. 0 means automatic." } },
      { "rotationInvariant", { brisk::CommandLineParser::ArgType::Bool, "true", "If set to false, keypoints are assumed upright." } },
      { "scaleInvariant", { brisk::CommandLineParser::ArgType::Bool, "true", "If set to false, keypoints are all assigned the same scale." } },
      { "computeTiming", { brisk::CommandLineParser::ArgType::Bool, "false", "If set to true, the timing is computed as an average over 100 executions." } } }
  );
  parser.setInfo("Demo application for different keypoint detectors and descriptors.");
  if (!parser.parse(argc, argv)) {
    return 1;
  }

  // Names of the two image files.
  std::string fname1;
  std::string fname2;
  cv::Mat imgRGB1;
  cv::Mat imgRGB2;
  cv::Mat imgRGB3;
  bool do_rot = false;
  fname1 = parser.optionAsString("image1");
  imgRGB1 = cv::imread(fname1);
  if (imgRGB1.empty()) {
    std::cout << "image not found at " << fname1 << std::endl;
    return 2;
  }
  fname2 = parser.optionAsString("image2");
  imgRGB2 = cv::imread(fname2);
  if (imgRGB2.empty()) {
    std::cout << "image not found at " << fname2 << std::endl;
    return 2;
  }

  // Convert to grayscale.
  cv::Mat imgGray1;
  cv::cvtColor(imgRGB1, imgGray1, CV_BGR2GRAY);
  cv::Mat imgGray2;
  if (!do_rot) {
    cv::cvtColor(imgRGB2, imgGray2, CV_BGR2GRAY);
  }

  // Run FAST in first image.
  std::vector<cv::KeyPoint> keypoints1, keypoints2;

  // Create the detector:
  cv::Ptr < cv::FeatureDetector > detector;
  std::string detectorType = parser.optionAsString("detectorType");
  float threshold = parser.optionAsNum("detectorThreshold");
  bool scaleInvariant = parser.optionAsBool("scaleInvariant");
  int octaves = 4;
  if (!scaleInvariant) {
    octaves = 0;
  }
  if ("FAST"==detectorType) {
#if CV_MAJOR_VERSION >= 3
    throw std::runtime_error("OpenCV version 3 FAST not supported");
#else
    detector = new cv::FastFeatureDetector(threshold, true);
#endif
  } else if ("AGAST" == detectorType) {
    detector = new brisk::BriskFeatureDetector(threshold, 0);
  } else if ("BRISK" == detectorType) {
    bool suppressScaleNonmaxima = true;
    detector = new brisk::BriskFeatureDetector(threshold, octaves, suppressScaleNonmaxima);
#ifndef __ARM_NEON__
  } else if ("HARRISSCALESPACE" == detectorType) {
    detector = new brisk::HarrisScaleSpaceFeatureDetector(threshold, octaves, 100);
#endif
  } else if ("ORB" == detectorType) {
#if CV_MAJOR_VERSION >= 3
    throw std::runtime_error("OpenCV version 3 ORB seems broken");
#else
    detector = new cv::OrbFeatureDetector(threshold);
#endif
  } else if ("SURF" == detectorType) {
    throw std::runtime_error("SURF requested -- not available in free OpenCV.");
  } else if ("SIFT" == detectorType) {
    throw std::runtime_error("SIFT requested -- not available in free OpenCV.");
  } else {
#if CV_MAJOR_VERSION >= 3
    throw std::runtime_error("OpenCV feature factory not available any longer");
#else
    detector = cv::FeatureDetector::create(detectorType.c_str());
#endif
  }
  if (detector.empty()) {
    std::cout << "Detector " << detectorType << " not recognized. Check spelling!" << std::endl;
    return 3;
  }

  // Run the detector:
  int testits = 100;
  bool computeTiming = parser.optionAsBool("computeTiming");
  if (computeTiming) {
    double tt = cv::getTickCount();
    detector->detect(imgGray2, keypoints2);
    for (int i = 0; i < testits; ++i) {
      keypoints1.clear();
      detector->detect(imgGray1, keypoints1);
    }
    tt = cv::getTickCount() - tt;
    std::cout << std::setprecision(4);
    std::cout << "Detection time: "
        << tt / (static_cast<double>(cv::getTickFrequency()) * testits) * 1000. << "ms for "
        << keypoints1.size() << " keypoints; "
        << tt / (static_cast<double>(cv::getTickFrequency()) * testits * (keypoints1.size())) * 1000
        << "ms per keypoint." << std::endl;
  } else {
    detector->detect(imgGray1, keypoints1);
    detector->detect(imgGray2, keypoints2);
  }

  // Now the extractor:
  bool hamming = true;
  cv::Ptr < cv::DescriptorExtractor > descriptorExtractor;
  std::string descriptorType = parser.optionAsString("descriptorType");
  bool rotationInvariant = parser.optionAsBool("rotationInvariant");
  // Now the extractor:
  if (descriptorType == "BRISK2") {
    descriptorExtractor = new brisk::BriskDescriptorExtractor(
        rotationInvariant, scaleInvariant, brisk::BriskDescriptorExtractor::Version::briskV2);
  } else if (descriptorType == "BRISK1") {
    descriptorExtractor = new brisk::BriskDescriptorExtractor(
        rotationInvariant, scaleInvariant, brisk::BriskDescriptorExtractor::Version::briskV1);
  } else if (descriptorType == "BRIEF") {
#if CV_MAJOR_VERSION >= 3
    throw std::runtime_error("OpenCV version 3 BRIEF not supported");
#else
    descriptorExtractor = new cv::BriefDescriptorExtractor(64);
#endif
  } else if (descriptorType == "ORB") {
#if CV_MAJOR_VERSION >= 3
    throw std::runtime_error("OpenCV version 3 ORB not supported");
#else
    if (!rotationInvariant) {
      std::cout << "Warning: ignoring not rotation invariant for ORB" << std::endl;
    }
    if (scaleInvariant) {
      std::cout << "Warning: ORB is multi-scale, but not detecting scale" << std::endl;
    }
    descriptorExtractor = new cv::OrbDescriptorExtractor();
#endif
  } else if (descriptorType == "SURF") {
    throw std::runtime_error("SURF requested -- not available in free OpenCV.");
    hamming = false;
  } else if (descriptorType == "SIFT") {
    throw std::runtime_error("SIFT requested -- not available in free OpenCV.");
    hamming = false;
  } else {
#if CV_MAJOR_VERSION >= 3
    throw std::runtime_error("OpenCV feature factory not available any longer");
#else
    descriptorExtractor = cv::DescriptorExtractor::create(descriptorType);
#endif
  }
  if (descriptorExtractor.empty()) {
    hamming = false;
    std::cout << "Descriptor " << descriptorType << " not recognized. Check spelling!" << std::endl;
    return 4;
  }

  // Get the descriptors.
  cv::Mat descriptors1, descriptors2;
  std::vector < cv::DMatch > indices;

  // First image.
  descriptorExtractor->compute(imgGray2, keypoints2, descriptors2);
  double tt = cv::getTickCount();
  if (computeTiming) {
    for (int i = 0; i < testits; ++i) {
      // And the second one.
      descriptorExtractor->compute(imgGray1, keypoints1, descriptors1);
    }
    tt = cv::getTickCount() - tt;
    std::cout << std::setprecision(4);
    std::cout << "Description time: "
        << tt / (static_cast<double>(cv::getTickFrequency()) * testits) * 1000. << "ms for "
        << keypoints1.size() << " keypoints; "
        << tt / (static_cast<double>(cv::getTickFrequency()) * testits * (keypoints1.size())) * 1000
        << "ms per keypoint." << std::endl;
  } else {
    descriptorExtractor->compute(imgGray1, keypoints1, descriptors1);
  }

  // Matching.
  std::vector < std::vector<cv::DMatch> > matches;
  cv::Ptr < cv::BFMatcher > descriptorMatcher;
  float matchingThreshold = parser.optionAsNum("matchingThreshold");

  if (hamming) {
    brisk::BruteForceMatcher matcher;
    if(matchingThreshold<1.0e-12){
      matchingThreshold = 55.0 * descriptors1.cols / 48.0;
    }
    matcher.radiusMatch(descriptors2, descriptors1, matches, matchingThreshold);
  } else {
    cv::BFMatcher matcher(cv::NORM_L2);
    if(matchingThreshold<1.0e-12){
      matchingThreshold = 0.21;
    }
    matcher.radiusMatch(descriptors2, descriptors1, matches, matchingThreshold);
  }
  cv::Mat outimg;

  // Drawing.
  drawMatches(imgRGB2, keypoints2, imgRGB1, keypoints1, matches, outimg, cv::Scalar(0, 255, 0),
              cv::Scalar(0, 0, 255), std::vector<std::vector<char> >(),
              cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::namedWindow("Matches");
  cv::imshow("Matches", outimg);
  cv::waitKey();

  return 0;
}
