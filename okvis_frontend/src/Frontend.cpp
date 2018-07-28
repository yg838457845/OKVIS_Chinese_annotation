/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Mar 27, 2015
 *      Author: Andreas Forster (an.forster@gmail.com)
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file Frontend.cpp
 * @brief Source file for the Frontend class.
 * @author Andreas Forster
 * @author Stefan Leutenegger
 */

#include <okvis/Frontend.hpp>

#include <brisk/brisk.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <glog/logging.h>

#include <okvis/ceres/ImuError.hpp>
#include <okvis/VioKeyframeWindowMatchingAlgorithm.hpp>
#include <okvis/IdProvider.hpp>

// cameras and distortions
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>

// Kneip RANSAC
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/FrameAbsolutePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/FrameRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/FrameRotationOnlySacProblem.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// Constructor.
Frontend::Frontend(size_t numCameras)
    : isInitialized_(false),
      numCameras_(numCameras),
      briskDetectionOctaves_(0),
      briskDetectionThreshold_(50.0),
      briskDetectionAbsoluteThreshold_(800.0),
      briskDetectionMaximumKeypoints_(450),
      briskDescriptionRotationInvariance_(true),
      briskDescriptionScaleInvariance_(false),
      briskMatchingThreshold_(60.0),
      matcher_(
          std::unique_ptr<okvis::DenseMatcher>(new okvis::DenseMatcher(4))),
      keyframeInsertionOverlapThreshold_(0.6),
      keyframeInsertionMatchingRatioThreshold_(0.2) {
  // create mutexes for feature detectors and descriptor extractors
  for (size_t i = 0; i < numCameras_; ++i) {
    featureDetectorMutexes_.push_back(
        std::unique_ptr<std::mutex>(new std::mutex()));
  }
  initialiseBriskFeatureDetectors();
}

// Detection and descriptor extraction on a per image basis.
// 特征提取和描述子计算函数
//cameraIndex表示相机的序号
//frameOut相机拍摄的图像
//T_WC第cameraIndex相机的位姿
//keypoints关键点
bool Frontend::detectAndDescribe(size_t cameraIndex,
                                 std::shared_ptr<okvis::MultiFrame> frameOut,
                                 const okvis::kinematics::Transformation& T_WC,
                                 const std::vector<cv::KeyPoint> * keypoints) {
  OKVIS_ASSERT_TRUE_DBG(Exception, cameraIndex < numCameras_, "Camera index exceeds number of cameras.");
  std::lock_guard<std::mutex> lock(*featureDetectorMutexes_[cameraIndex]);//给当前步骤上锁

  // check there are no keypoints here
  OKVIS_ASSERT_TRUE(Exception, keypoints == nullptr, "external keypoints currently not supported")

  frameOut->setDetector(cameraIndex, featureDetectors_[cameraIndex]);//设置特征提取算子
  frameOut->setExtractor(cameraIndex, descriptorExtractors_[cameraIndex]);//计算描述子

  frameOut->detect(cameraIndex);//探测特征点

  // ExtractionDirection == gravity direction in camera frame
  Eigen::Vector3d g_in_W(0, 0, -1);//重力方向
  Eigen::Vector3d extractionDirection = T_WC.inverse().C() * g_in_W;//将g_in_W进行旋转,到相机坐标系C
  frameOut->describe(cameraIndex, extractionDirection);//计算描述子

  // set detector/extractor to nullpointer? TODO
  return true;
}

// Matching as well as initialization of landmarks and state.
// 特征匹配函数以及地标点初始化函数
///形参
/// 状态求解器
/// 当前位姿
/// 参数
/// 地图
/// 输入的当前帧
/// 是否为关键帧
bool Frontend::dataAssociationAndInitialization(
    okvis::Estimator& estimator,
    okvis::kinematics::Transformation& /*T_WS_propagated*/, // TODO sleutenegger: why is this not used here?
    const okvis::VioParameters &params,
    const std::shared_ptr<okvis::MapPointVector> /*map*/, // TODO sleutenegger: why is this not used here?
    std::shared_ptr<okvis::MultiFrame> framesInOut,
    bool *asKeyframe) {
  // match new keypoints to existing landmarks/keypoints
  // initialise new landmarks (states)
  // outlier rejection by consistency check
  // RANSAC (2D2D / 3D2D)
  // decide keyframe
  // left-right stereo match & init

  // find distortion type
  //确定畸变模型
  okvis::cameras::NCameraSystem::DistortionType distortionType = params.nCameraSystem
      .distortionType(0);
  for (size_t i = 1; i < params.nCameraSystem.numCameras(); ++i) {
    OKVIS_ASSERT_TRUE(Exception,
                      distortionType == params.nCameraSystem.distortionType(i),
                      "mixed frame types are not supported yet");
  }
  int num3dMatches = 0;

  // first frame? (did do addStates before, so 1 frame minimum in estimator)
  //状态求解器中有至少两个状态
  if (estimator.numFrames() > 1) {

    int requiredMatches = 5;//需要的最小匹配对数

    double uncertainMatchFraction = 0;
    bool rotationOnly = false;

    // match to last keyframe
    // 定义一个计时变量
    TimerSwitchable matchKeyframesTimer("2.4.1 matchToKeyframes");
    //数据集用的是RadialTangential
    switch (distortionType) {
      case okvis::cameras::NCameraSystem::RadialTangential: {
        num3dMatches = matchToKeyframes<
            VioKeyframeWindowMatchingAlgorithm<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::RadialTangentialDistortion> > >(
            estimator, params, framesInOut->id(), rotationOnly, false,
            &uncertainMatchFraction);
        break;
      }
      case okvis::cameras::NCameraSystem::Equidistant: {
        num3dMatches = matchToKeyframes<
            VioKeyframeWindowMatchingAlgorithm<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::EquidistantDistortion> > >(
            estimator, params, framesInOut->id(), rotationOnly, false,
            &uncertainMatchFraction);
        break;
      }
      case okvis::cameras::NCameraSystem::RadialTangential8: {
        num3dMatches = matchToKeyframes<
            VioKeyframeWindowMatchingAlgorithm<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::RadialTangentialDistortion8> > >(
            estimator, params, framesInOut->id(), rotationOnly, false,
            &uncertainMatchFraction);
        break;
      }
      default:
        OKVIS_THROW(Exception, "Unsupported distortion type.")
        break;
    }
    matchKeyframesTimer.stop();
    //如果没有初始化，一般会在第二帧进行
    if (!isInitialized_) {
      //不是纯旋转
      if (!rotationOnly) {
        isInitialized_ = true;//初始化为真（只要不是纯旋转）
        LOG(INFO) << "Initialized!";
      }
    }
    //关键帧匹配的特征点数量，requiredMatches=5
    if (num3dMatches <= requiredMatches) {
      LOG(WARNING) << "Tracking failure. Number of 3d2d-matches: " << num3dMatches;
    }

    // keyframe decision, at the moment only landmarks that match with keyframe are initialised
    //观测是否为关键帧,framesInOut为输入的关键帧
    *asKeyframe = *asKeyframe || doWeNeedANewKeyframe(estimator, framesInOut);

    // match to last frame
    //和上一帧进行匹配
    TimerSwitchable matchToLastFrameTimer("2.4.2 matchToLastFrame");
    switch (distortionType) {
      case okvis::cameras::NCameraSystem::RadialTangential: {
        //数据集所用的函数
        matchToLastFrame<
            VioKeyframeWindowMatchingAlgorithm<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::RadialTangentialDistortion> > >(
            estimator, params, framesInOut->id(),
            false);
        break;
      }
      case okvis::cameras::NCameraSystem::Equidistant: {
        matchToLastFrame<
            VioKeyframeWindowMatchingAlgorithm<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::EquidistantDistortion> > >(
            estimator, params, framesInOut->id(),
            false);
        break;
      }
      case okvis::cameras::NCameraSystem::RadialTangential8: {
        matchToLastFrame<
            VioKeyframeWindowMatchingAlgorithm<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::RadialTangentialDistortion8> > >(
            estimator, params, framesInOut->id(),
            false);

        break;
      }
      default:
        OKVIS_THROW(Exception, "Unsupported distortion type.")
        break;
    }
    matchToLastFrameTimer.stop();
  } else
     //第一帧默认是初始关键帧,只有一帧,即第一帧
    *asKeyframe = true;  // first frame needs to be keyframe

  // do stereo match to get new landmarks
  TimerSwitchable matchStereoTimer("2.4.3 matchStereo");
  switch (distortionType) {
    case okvis::cameras::NCameraSystem::RadialTangential: {
      //进行双目间的立体匹配
      matchStereo<
          VioKeyframeWindowMatchingAlgorithm<
              okvis::cameras::PinholeCamera<
                  okvis::cameras::RadialTangentialDistortion> > >(estimator,
                                                                  framesInOut);
      break;
    }
    case okvis::cameras::NCameraSystem::Equidistant: {
      matchStereo<
          VioKeyframeWindowMatchingAlgorithm<
              okvis::cameras::PinholeCamera<
                  okvis::cameras::EquidistantDistortion> > >(estimator,
                                                             framesInOut);
      break;
    }
    case okvis::cameras::NCameraSystem::RadialTangential8: {
      matchStereo<
          VioKeyframeWindowMatchingAlgorithm<
              okvis::cameras::PinholeCamera<
                  okvis::cameras::RadialTangentialDistortion8> > >(estimator,
                                                                   framesInOut);
      break;
    }
    default:
      OKVIS_THROW(Exception, "Unsupported distortion type.")
      break;
  }
  matchStereoTimer.stop();

  return true;
}

// Propagates pose, speeds and biases with given IMU measurements.
// 利用IMU的数据预测位姿、速度和偏置
bool Frontend::propagation(const okvis::ImuMeasurementDeque & imuMeasurements,
                           const okvis::ImuParameters & imuParams,
                           okvis::kinematics::Transformation& T_WS_propagated,
                           okvis::SpeedAndBias & speedAndBiases,
                           const okvis::Time& t_start, const okvis::Time& t_end,
                           Eigen::Matrix<double, 15, 15>* covariance,
                           Eigen::Matrix<double, 15, 15>* jacobian) const {
  //IMU给出的观测信息过少
  if (imuMeasurements.size() < 2) {
    LOG(WARNING)
        << "- Skipping propagation as only one IMU measurement has been given to frontend."
        << " Normal when starting up.";
    return 0;
  }
  //利用IMU观测进行预积分，预测求解IMU的状态
  int measurements_propagated = okvis::ceres::ImuError::propagation(
      imuMeasurements, imuParams, T_WS_propagated, speedAndBiases, t_start,
      t_end, covariance, jacobian);

  return measurements_propagated > 0;
}

// Decision whether a new frame should be keyframe or not.
// 判断一个新的帧是不是关键帧
bool Frontend::doWeNeedANewKeyframe(
    const okvis::Estimator& estimator,
    std::shared_ptr<okvis::MultiFrame> currentFrame) {
   //状态空间中小于2两个帧，只有一个关键帧，则需要
  if (estimator.numFrames() < 2) {
    // just starting, so yes, we need this as a new keyframe
    return true;
  }
  //如果还没有初始化，则不能添加关键帧
  if (!isInitialized_)
    return false;

  double overlap = 0.0;
  double ratio = 0.0;

  // go through all the frames and try to match the initialized keypoints
  //numFrames（）表示相机的数量
  for (size_t im = 0; im < currentFrame->numFrames(); ++im) {

    // get the hull of all keypoints in current frame
    std::vector<cv::Point2f> frameBPoints, frameBHull;
    std::vector<cv::Point2f> frameBMatches, frameBMatchesHull;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > frameBLandmarks;

    const size_t numB = currentFrame->numKeypoints(im);//当前帧第im个相机特征点的数量
    frameBPoints.reserve(numB);//初始化特征点容器
    frameBLandmarks.reserve(numB);//初始化地标点容器
    Eigen::Vector2d keypoint;
    for (size_t k = 0; k < numB; ++k) {
      currentFrame->getKeypoint(im, k, keypoint);//提取第im个相机中第k个特征点
      // insert it
      frameBPoints.push_back(cv::Point2f(keypoint[0], keypoint[1]));//加入到特征点集合中
      // also remember matches
      //特征点已经存在地标点标号
      if (currentFrame->landmarkId(im, k) != 0) {
        frameBMatches.push_back(cv::Point2f(keypoint[0], keypoint[1]));//加入到已经匹配完成的特征点集合中
      }
    }

    if (frameBPoints.size() < 3)
      continue;
    cv::convexHull(frameBPoints, frameBHull);
    if (frameBMatches.size() < 3)
      continue;
    cv::convexHull(frameBMatches, frameBMatchesHull);

    // areas
    double frameBArea = cv::contourArea(frameBHull);//计算特征点轮廓的面积
    double frameBMatchesArea = cv::contourArea(frameBMatchesHull);//计算有匹配地标点的特征点轮廓的面积

    // overlap area
    double overlapArea = frameBMatchesArea / frameBArea;//面积的比值
    // matching ratio inside overlap area: count
    int pointsInFrameBMatchesArea = 0;
    if (frameBMatchesHull.size() > 2) {
      for (size_t k = 0; k < frameBPoints.size(); ++k) {
        if (cv::pointPolygonTest(frameBMatchesHull, frameBPoints[k], false)
            > 0) {
          pointsInFrameBMatchesArea++;//在匹配点轮廓中特征点(内点)的数量
        }
      }
    }
    double matchingRatio = double(frameBMatches.size())
        / double(pointsInFrameBMatchesArea);//匹配点的数量比内点的数量

    // calculate overlap score
    overlap = std::max(overlapArea, overlap);//匹配特征点轮廓的面积/所有特征点轮廓的面积
    ratio = std::max(matchingRatio, ratio);//匹配点的数量/特征点在轮廓中的数量
  }

  // take a decision
  //overlap表示匹配点轮廓面积的占比,ratio表示匹配点的占比
  //匹配点较多,则认为不需要添加关键帧
  if (overlap > keyframeInsertionOverlapThreshold_
      && ratio > keyframeInsertionMatchingRatioThreshold_)
    return false;
  else
    return true;
}

// Match a new multiframe to existing keyframes
//匹配一个新的帧和已经存在的关键帧
//模板MATCHING_ALGORITHM===VioKeyframeWindowMatchingAlgorithm<okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion> >
template<class MATCHING_ALGORITHM>
int Frontend::matchToKeyframes(okvis::Estimator& estimator,
                               const okvis::VioParameters & params,
                               const uint64_t currentFrameId,
                               bool& rotationOnly,
                               bool usePoseUncertainty,
                               double* uncertainMatchFraction,
                               bool removeOutliers) {
  rotationOnly = true;
  //状态求解器中的帧数小于2,直接返回,因为我们需要第一帧作为初始关键帧
  if (estimator.numFrames() < 2) {
    // just starting, so yes, we need this as a new keyframe
    return 0;
  }

  int retCtr = 0;
  int numUncertainMatches = 0;

  // go through all the frames and try to match the initialized keypoints
  //该循环执行了3个关键帧
  size_t kfcounter = 0;
  for (size_t age = 1; age < estimator.numFrames(); ++age) {
    uint64_t olderFrameId = estimator.frameIdByAge(age);//olderFrameId表示比当前帧早age的帧的id
    //如果这个早的帧不是关键帧,直接跳过
    if (!estimator.isKeyframe(olderFrameId))
      continue;
    for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
      //对每一目相机的图像都进行匹配, 3d-2d的匹配算法初始化
      ///形参
      /// estimator------状态求解器
      /// Match3D2D为1,储存在一个枚举类中
      /// briskMatchingThreshold_为60
      /// usePoseUncertainty为false
      MATCHING_ALGORITHM matchingAlgorithm(estimator,
                                           MATCHING_ALGORITHM::Match3D2D,
                                           briskMatchingThreshold_,
                                           usePoseUncertainty);
      //设置算法需要进行匹配的两帧的id
      matchingAlgorithm.setFrames(olderFrameId, currentFrameId, im, im);

      // match 3D-2D
      //开始匹配
      matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);
      retCtr += matchingAlgorithm.numMatches();//确定的匹配点数量
      numUncertainMatches += matchingAlgorithm.numUncertainMatches();//不确定的匹配点数量

    }
    kfcounter++;//经历了几个关键帧
    if (kfcounter > 2)
      break;
  }

  kfcounter = 0;
  bool firstFrame = true;
  //该循环匹配了两个关键帧
  for (size_t age = 1; age < estimator.numFrames(); ++age) {
    uint64_t olderFrameId = estimator.frameIdByAge(age);//olderFrameId表示比当前帧早age的帧的id
    //如果不是关键帧,直接跳过
    if (!estimator.isKeyframe(olderFrameId))
      continue;
    //遍历每一个相机
    for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
      //初始化匹配算法
      MATCHING_ALGORITHM matchingAlgorithm(estimator,
                                           MATCHING_ALGORITHM::Match2D2D,
                                           briskMatchingThreshold_,
                                           usePoseUncertainty);
      //设置算法匹配需要用到的两帧的id
      matchingAlgorithm.setFrames(olderFrameId, currentFrameId, im, im);

      // match 2D-2D for initialization of new (mono-)correspondences
      //进行2d-2d的匹配
      matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);
      retCtr += matchingAlgorithm.numMatches();//统计所有的匹配点
      numUncertainMatches += matchingAlgorithm.numUncertainMatches();//统计所有的不确定的匹配点
    }

    // remove outliers
    // only do RANSAC 3D2D with most recent KF
    //表示已经完成了初始化且是第一个搜索到的关键帧
    if (kfcounter == 0 && isInitialized_)
      runRansac3d2d(estimator, params.nCameraSystem,
                    estimator.multiFrame(currentFrameId), removeOutliers);//执行ransac---3d-2d

    bool rotationOnly_tmp = false;
    // do RANSAC 2D2D for initialization only
    // 还没有初始化,只有在第二帧输入时
    if (!isInitialized_) {
      runRansac2d2d(estimator, params, currentFrameId, olderFrameId, true,
                    removeOutliers, rotationOnly_tmp);//执行ransac----2d-2d
    }
    //是匹配到的第一个关键帧吗
    if (firstFrame) {
      rotationOnly = rotationOnly_tmp;//判断是不是纯旋转
      firstFrame = false;
    }

    kfcounter++;//找到的关键帧数量+1
    if (kfcounter > 1)
      break;
  }

  // calculate fraction of safe matches
  if (uncertainMatchFraction) {
    *uncertainMatchFraction = double(numUncertainMatches) / double(retCtr);//不确定的匹配点与总共匹配点的比值
  }

  return retCtr;//返回匹配点对数
}

// Match a new multiframe to the last frame.
//和上一帧进行匹配,usePoseUncertainty为false
template<class MATCHING_ALGORITHM>
int Frontend::matchToLastFrame(okvis::Estimator& estimator,
                               const okvis::VioParameters& params,
                               const uint64_t currentFrameId,
                               bool usePoseUncertainty,
                               bool removeOutliers) {
  //状态求解器中只一帧的情况,直接返回,因为第一帧必定是关键帧
  if (estimator.numFrames() < 2) {
    // just starting, so yes, we need this as a new keyframe
    return 0;
  }

  uint64_t lastFrameId = estimator.frameIdByAge(1);//表示上一帧的序号
  //判断它是不是关键帧
  if (estimator.isKeyframe(lastFrameId)) {
    // already done
    return 0;
  }

  int retCtr = 0;
  //遍历每个相机
  for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
    MATCHING_ALGORITHM matchingAlgorithm(estimator,
                                         MATCHING_ALGORITHM::Match3D2D,
                                         briskMatchingThreshold_,
                                         usePoseUncertainty);
    matchingAlgorithm.setFrames(lastFrameId, currentFrameId, im, im);//

    // match 3D-2D
    matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);
    retCtr += matchingAlgorithm.numMatches();//匹配点对数
  }

  runRansac3d2d(estimator, params.nCameraSystem,
                estimator.multiFrame(currentFrameId), removeOutliers);//执行3d-2d的RANSAC,筛选能被当前帧观测到的地标点

  for (size_t im = 0; im < params.nCameraSystem.numCameras(); ++im) {
    MATCHING_ALGORITHM matchingAlgorithm(estimator,
                                         MATCHING_ALGORITHM::Match2D2D,
                                         briskMatchingThreshold_,
                                         usePoseUncertainty);
    matchingAlgorithm.setFrames(lastFrameId, currentFrameId, im, im);

    // match 2D-2D for initialization of new (mono-)correspondences
    matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);
    retCtr += matchingAlgorithm.numMatches();//匹配点对数
  }

  // remove outliers
  bool rotationOnly = false;
  //还未初始化
  if (!isInitialized_)
    runRansac2d2d(estimator, params, currentFrameId, lastFrameId, false,
                  removeOutliers, rotationOnly);//执行2d-2d的匹配

  return retCtr;//返回匹配点对数
}

// Match the frames inside the multiframe to each other to initialise new landmarks.
// 立体视觉匹配
template<class MATCHING_ALGORITHM>
void Frontend::matchStereo(okvis::Estimator& estimator,
                           std::shared_ptr<okvis::MultiFrame> multiFrame) {

  const size_t camNumber = multiFrame->numFrames();//相机数量
  const uint64_t mfId = multiFrame->id();//初始帧id

  for (size_t im0 = 0; im0 < camNumber; im0++) {
    for (size_t im1 = im0 + 1; im1 < camNumber; im1++) {
      // first, check the possibility for overlap
      // FIXME: implement this in the Multiframe...!!
      //im0表示左目
        //im1表示右目
      // check overlap
      if(!multiFrame->hasOverlap(im0, im1)){
        continue;
      }

      MATCHING_ALGORITHM matchingAlgorithm(estimator,
                                           MATCHING_ALGORITHM::Match2D2D,
                                           briskMatchingThreshold_,
                                           false);  // TODO: make sure this is changed when switching back to uncertainty based matching
      matchingAlgorithm.setFrames(mfId, mfId, im0, im1);  // newest frame

      // match 2D-2D
      matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);//执行2D到2D的匹配

      // match 3D-2D
      matchingAlgorithm.setMatchingType(MATCHING_ALGORITHM::Match3D2D);
      matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);//执行左3D到右2D的匹配

      // match 2D-3D
      matchingAlgorithm.setFrames(mfId, mfId, im1, im0);  // newest frame
      matcher_->match<MATCHING_ALGORITHM>(matchingAlgorithm);//执行右3d到左2d的匹配
    }
  }

  // TODO: for more than 2 cameras check that there were no duplications!

  // TODO: ensure 1-1 matching.

  // TODO: no RANSAC ?

  for (size_t im = 0; im < camNumber; im++) {
    const size_t ksize = multiFrame->numKeypoints(im);//第im个相机图片的特征点数量
    for (size_t k = 0; k < ksize; ++k) {
        //特征点已经存在了地标点,直接跳过
      if (multiFrame->landmarkId(im, k) != 0) {
        continue;  // already identified correspondence
      }
      //设置地标点id
      multiFrame->setLandmarkId(im, k, okvis::IdProvider::instance().newId());
    }
  }
}

// Perform 3D/2D RANSAC.
// 执行3D到2D的RANSAC,对当前帧能观测到的地标点进行RANSAC筛选
int Frontend::runRansac3d2d(okvis::Estimator& estimator,
                            const okvis::cameras::NCameraSystem& nCameraSystem,
                            std::shared_ptr<okvis::MultiFrame> currentFrame,
                            bool removeOutliers) {
  //如果状态器中的帧数量小于2,直接跳过
  if (estimator.numFrames() < 2) {
    // nothing to match against, we are just starting up.
    return 1;
  }

  /////////////////////
  //   KNEIP RANSAC
  /////////////////////
  int numInliers = 0;

  // absolute pose adapter for Kneip toolchain
  // 初始化适应器
  opengv::absolute_pose::FrameNoncentralAbsoluteAdapter adapter(estimator,
                                                                nCameraSystem,
                                                                currentFrame);

  size_t numCorrespondences = adapter.getNumberCorrespondences();//得到当前帧currentFrame能观测到的地标点数量
  //地标点数量过少
  if (numCorrespondences < 5)
    return numCorrespondences;

  // create a RelativePoseSac problem and RANSAC
  opengv::sac::Ransac<
      opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem> ransac;
  std::shared_ptr<
      opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem> absposeproblem_ptr(
      new opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem(
          adapter,
          opengv::sac_problems::absolute_pose::FrameAbsolutePoseSacProblem::Algorithm::GP3P));
  //ransc的参数
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ = 9;
  ransac.max_iterations_ = 50;
  // initial guess not needed...
  // run the ransac
  // 运行RANSC
  ransac.computeModel(0);

  // assign transformation
  numInliers = ransac.inliers_.size();//ransc内点数
  if (numInliers >= 10) {

    // kick out outliers:
   //标记哪些地标点是真（内点）
    std::vector<bool> inliers(numCorrespondences, false);
    for (size_t k = 0; k < ransac.inliers_.size(); ++k) {
      inliers.at(ransac.inliers_.at(k)) = true;
    }

    for (size_t k = 0; k < numCorrespondences; ++k) {
      if (!inliers[k]) {
        //地标点不符合ransc
        // get the landmark id:
        size_t camIdx = adapter.camIndex(k);
        size_t keypointIdx = adapter.keypointIndex(k);
        uint64_t lmId = currentFrame->landmarkId(camIdx, keypointIdx);

        // reset ID:
        currentFrame->setLandmarkId(camIdx, keypointIdx, 0);//重置camIdx相机中第keypointIdx特征点对应的地标点的id

        // remove observation
        if (removeOutliers) {
          estimator.removeObservation(lmId, currentFrame->id(), camIdx,
                                      keypointIdx);//移除观测
        }
      }
    }
  }
  return numInliers;
}

// Perform 2D/2D RANSAC.
// 执行2D到2D的RANSC
//RANSAC筛选出去不是内点的二维三角化后的点
//并优化了初始第二帧的位姿
int Frontend::runRansac2d2d(okvis::Estimator& estimator,
                            const okvis::VioParameters& params,
                            uint64_t currentFrameId, uint64_t olderFrameId,
                            bool initializePose,
                            bool removeOutliers,
                            bool& rotationOnly) {
  // match 2d2d
  rotationOnly = false;
  const size_t numCameras = params.nCameraSystem.numCameras();

  size_t totalInlierNumber = 0;
  bool rotation_only_success = false;
  bool rel_pose_success = false;

  // run relative RANSAC
  // 遍历每个相机
  for (size_t im = 0; im < numCameras; ++im) {

    // relative pose adapter for Kneip toolchain
    opengv::relative_pose::FrameRelativeAdapter adapter(estimator,
                                                        params.nCameraSystem,
                                                        olderFrameId, im,
                                                        currentFrameId, im);

    size_t numCorrespondences = adapter.getNumberCorrespondences();//匹配的点对数
    //匹配点对数
    if (numCorrespondences < 10)
      continue;  // won't generate meaningful results. let's hope the few correspondences we have are all inliers!!

    // try both the rotation-only RANSAC and the relative one:

    // create a RelativePoseSac problem and RANSAC
    //匹配点对数>10
    typedef opengv::sac_problems::relative_pose::FrameRotationOnlySacProblem FrameRotationOnlySacProblem;
    opengv::sac::Ransac<FrameRotationOnlySacProblem> rotation_only_ransac;//纯旋转RANSAC模式
    std::shared_ptr<FrameRotationOnlySacProblem> rotation_only_problem_ptr(
        new FrameRotationOnlySacProblem(adapter));
    rotation_only_ransac.sac_model_ = rotation_only_problem_ptr;
    rotation_only_ransac.threshold_ = 9;
    rotation_only_ransac.max_iterations_ = 50;

    // run the ransac
    //直接RANSAC
    rotation_only_ransac.computeModel(0);

    // get quality
    int rotation_only_inliers = rotation_only_ransac.inliers_.size();
    float rotation_only_ratio = float(rotation_only_inliers)
        / float(numCorrespondences);//RANSAC内点与匹配点对数

    // now the rel_pose one:
    typedef opengv::sac_problems::relative_pose::FrameRelativePoseSacProblem FrameRelativePoseSacProblem;
    opengv::sac::Ransac<FrameRelativePoseSacProblem> rel_pose_ransac;//相对位姿的RANSAC模式
    std::shared_ptr<FrameRelativePoseSacProblem> rel_pose_problem_ptr(
        new FrameRelativePoseSacProblem(
            adapter, FrameRelativePoseSacProblem::STEWENIUS));
    rel_pose_ransac.sac_model_ = rel_pose_problem_ptr;
    rel_pose_ransac.threshold_ = 9;     //(1.0 - cos(0.5/600));
    rel_pose_ransac.max_iterations_ = 50;

    // run the ransac
    //执行RANSAC
    rel_pose_ransac.computeModel(0);

    // assess success
    int rel_pose_inliers = rel_pose_ransac.inliers_.size();
    float rel_pose_ratio = float(rel_pose_inliers) / float(numCorrespondences);//RANSAC内点与匹配点对数

    // decide on success and fill inliers
    std::vector<bool> inliers(numCorrespondences, false);
    //纯旋转的RANSAC分量大
    if (rotation_only_ratio > rel_pose_ratio || rotation_only_ratio > 0.8) {
      if (rotation_only_inliers > 10) {
        rotation_only_success = true;
      }
      rotationOnly = true;
      totalInlierNumber += rotation_only_inliers;//内点的数量
      for (size_t k = 0; k < rotation_only_ransac.inliers_.size(); ++k) {
        inliers.at(rotation_only_ransac.inliers_.at(k)) = true;//哪个内点为真(旋转模式)
      }
    } else {
      if (rel_pose_inliers > 10) {
        rel_pose_success = true;
      }
      totalInlierNumber += rel_pose_inliers;
      for (size_t k = 0; k < rel_pose_ransac.inliers_.size(); ++k) {
        inliers.at(rel_pose_ransac.inliers_.at(k)) = true;//哪个内点为真(相对位置模式)
      }
    }

    // failure?
    if (!rotation_only_success && !rel_pose_success) {
      continue;
    }

    // otherwise: kick out outliers!
    std::shared_ptr<okvis::MultiFrame> multiFrame = estimator.multiFrame(
        currentFrameId);//当前帧

    for (size_t k = 0; k < numCorrespondences; ++k) {
      size_t idxB = adapter.getMatchKeypointIdxB(k);//第k个匹配点对应的B帧特征点序号
      //表示第k个匹配点不是RANSAC内点
      if (!inliers[k]) {

        //uint64_t lmId = multiFrame->landmarkId(im, k);
        uint64_t lmId = multiFrame->landmarkId(im, idxB);
        // reset ID:
        //multiFrame->setLandmarkId(im, k, 0);//重置当前帧中对应的地标点
        multiFrame->setLandmarkId(im, idxB, 0);//重置当前帧中对应的地标点
        // remove observation
        if (removeOutliers) {
          if (lmId != 0 && estimator.isLandmarkAdded(lmId)){
            estimator.removeObservation(lmId, currentFrameId, im, idxB);//移除观测
          }
        }
      }
    }

    // initialize pose if necessary
    //initializePose默认为真,且未被初始化
    if (initializePose && !isInitialized_) {
      if (rel_pose_success)
        LOG(INFO)
            << "Initializing pose from 2D-2D RANSAC";
      else
        LOG(INFO)
            << "Initializing pose from 2D-2D RANSAC: orientation only";

      Eigen::Matrix4d T_C1C2_mat = Eigen::Matrix4d::Identity();

      okvis::kinematics::Transformation T_SCA, T_WSA, T_SC0, T_WS0;
      uint64_t idA = olderFrameId;
      uint64_t id0 = currentFrameId;
      estimator.getCameraSensorStates(idA, im, T_SCA);//得到外参
      estimator.get_T_WS(idA, T_WSA);//得到IMU的位姿
      estimator.getCameraSensorStates(id0, im, T_SC0);
      estimator.get_T_WS(id0, T_WS0);
      if (rel_pose_success) {
        // update pose
        // if the IMU is used, this will be quickly optimized to the correct scale. Hopefully.
        T_C1C2_mat.topLeftCorner<3, 4>() = rel_pose_ransac.model_coefficients_;

        //initialize with projected length according to motion prior.

        okvis::kinematics::Transformation T_C1C2 = T_SCA.inverse()
            * T_WSA.inverse() * T_WS0 * T_SC0;//得到C0到CA的转换关系
        T_C1C2_mat.topRightCorner<3, 1>() = T_C1C2_mat.topRightCorner<3, 1>()
            * std::max(
                0.0,
                double(
                    T_C1C2_mat.topRightCorner<3, 1>().transpose()
                        * T_C1C2.r()));//将平移进行放大,即尺度的放大
      } else {
        // rotation only assigned...
        // 纯旋转,只对旋转信息进行了赋值
        T_C1C2_mat.topLeftCorner<3, 3>() = rotation_only_ransac
            .model_coefficients_;
      }

      // set.
      estimator.set_T_WS(
          id0,
          T_WSA * T_SCA * okvis::kinematics::Transformation(T_C1C2_mat)
              * T_SC0.inverse());//重新确定了当前帧IMU的位姿
    }
  }

  if (rel_pose_success || rotation_only_success)
    return totalInlierNumber;//总共的内点数量
  else {
    //表示只有旋转,没有任何的相对平移
    rotationOnly = true;  // hack...
    return -1;
  }

  return 0;
}

// (re)instantiates feature detectors and descriptor extractors. Used after settings changed or at startup.
// 特征提取和描述子的提取
void Frontend::initialiseBriskFeatureDetectors() {
  //遍历每一个相机的锁
  for (auto it = featureDetectorMutexes_.begin();
      it != featureDetectorMutexes_.end(); ++it) {
    (*it)->lock();//上锁
  }
  featureDetectors_.clear();//特征点提取类的容器,提取brisk的特征点
  descriptorExtractors_.clear();//描述子计算类的容器
  for (size_t i = 0; i < numCameras_; ++i) {
    featureDetectors_.push_back(
        std::shared_ptr<cv::FeatureDetector>(
#ifdef __ARM_NEON__
            new cv::GridAdaptedFeatureDetector( 
            new cv::FastFeatureDetector(briskDetectionThreshold_),
                briskDetectionMaximumKeypoints_, 7, 4 ))); // from config file, except the 7x4...
#else
            new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
                briskDetectionThreshold_, briskDetectionOctaves_, 
                briskDetectionAbsoluteThreshold_,
                briskDetectionMaximumKeypoints_)));//探测阈值(briskDetectionThreshold_=50),探测层数为(briskDetectionOctaves_=0),
                //探测绝对阈值(briskDetectionAbsoluteThreshold_=800),最多提取特征点的数量(450)
#endif
    descriptorExtractors_.push_back(
        std::shared_ptr<cv::DescriptorExtractor>(
            new brisk::BriskDescriptorExtractor(
                briskDescriptionRotationInvariance_,
                briskDescriptionScaleInvariance_)));//旋转不变性为真,尺度不变形为false
  }
  for (auto it = featureDetectorMutexes_.begin();
      it != featureDetectorMutexes_.end(); ++it) {
    (*it)->unlock();//解锁
  }
}

}  // namespace okvis
