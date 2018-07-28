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
 *  Created on: Oct 17, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file VioKeyframeWindowMatchingAlgorithm.cpp
 * @brief Source file for the VioKeyframeWindowMatchingAlgorithm class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <okvis/VioKeyframeWindowMatchingAlgorithm.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/IdProvider.hpp>
#include <okvis/cameras/CameraBase.hpp>
#include <okvis/MultiFrame.hpp>

// cameras and distortions
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>

#include <opencv2/features2d/features2d.hpp> // for cv::KeyPoint

/// \brief okvis Main namespace of this package.
namespace okvis {

// Constructor.
//CAMERA_GEOMETRY_T= okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion>
template<class CAMERA_GEOMETRY_T>
VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::VioKeyframeWindowMatchingAlgorithm(
    okvis::Estimator& estimator, int matchingType, float distanceThreshold,
    bool usePoseUncertainty) {
  matchingType_ = matchingType;
  distanceThreshold_ = distanceThreshold;
  estimator_ = &estimator;
  usePoseUncertainty_ = usePoseUncertainty;
}

template<class CAMERA_GEOMETRY_T>
VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::~VioKeyframeWindowMatchingAlgorithm() {

}

// Set which frames to match.
//设置两个帧进行匹配
template<class CAMERA_GEOMETRY_T>
void VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::setFrames(
    uint64_t mfIdA, uint64_t mfIdB, size_t camIdA, size_t camIdB) {

  OKVIS_ASSERT_TRUE(Exception, !(mfIdA == mfIdB && camIdA == camIdB),
                    "trying to match identical frames.");

  // remember indices
  mfIdA_ = mfIdA;
  mfIdB_ = mfIdB;
  camIdA_ = camIdA;
  camIdB_ = camIdB;
  // frames and related information
  frameA_ = estimator_->multiFrame(mfIdA_);
  frameB_ = estimator_->multiFrame(mfIdB_);

  // focal length
  fA_ = frameA_->geometryAs<CAMERA_GEOMETRY_T>(camIdA_)->focalLengthU();
  fB_ = frameB_->geometryAs<CAMERA_GEOMETRY_T>(camIdB_)->focalLengthU();

  // calculate the relative transformations and uncertainties
  // TODO donno, if and what we need here - I'll see
  estimator_->getCameraSensorStates(mfIdA_, camIdA, T_SaCa_);//得到帧mfIdA_的第camIdA个相机的外参矩阵（相机到IMU的旋转）
  estimator_->getCameraSensorStates(mfIdB_, camIdB, T_SbCb_);//得到帧mfIdB_的第camIdB个相机的外参矩阵(相机到IMU的旋转)
  estimator_->get_T_WS(mfIdA_, T_WSa_);//得到帧mfIdA_的位姿转换矩阵Tws
  estimator_->get_T_WS(mfIdB_, T_WSb_);//得到帧mfIdB_的位姿转换矩阵Tws
  T_SaW_ = T_WSa_.inverse();//A帧的IMU到W的位姿转换矩阵
  T_SbW_ = T_WSb_.inverse();//B帧的IMU到W的位姿转换矩阵
  T_WCa_ = T_WSa_ * T_SaCa_;//得到a相机(A帧中)到W的位姿转换矩阵
  T_WCb_ = T_WSb_ * T_SbCb_;//得到b相机(B帧中)到W的位姿转换矩阵
  T_CaW_ = T_WCa_.inverse();//得到W到a相机的位姿转换矩阵
  T_CbW_ = T_WCb_.inverse();//得到W到b相机的位姿转换矩阵
  T_CaCb_ = T_WCa_.inverse() * T_WCb_;//得到b相机到a相机的位姿转换矩阵
  T_CbCa_ = T_CaCb_.inverse();//得到a相机到b相机的位姿转换矩阵

  validRelativeUncertainty_ = false;
}

// Set the matching type.
template<class CAMERA_GEOMETRY_T>
void VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::setMatchingType(
    int matchingType) {
  matchingType_ = matchingType;
}

// This will be called exactly once for each call to DenseMatcher::match().
// 运行match()函数的时候可以调用该函数
template<class CAMERA_GEOMETRY_T>
void VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::doSetup() {

  // setup stereo triangulator
  // first, let's get the relative uncertainty.
  okvis::kinematics::Transformation T_CaCb;
  Eigen::Matrix<double, 6, 6> UOplus = Eigen::Matrix<double, 6, 6>::Zero();
  //usePoseUncertainty_=false
  if (usePoseUncertainty_) {
    OKVIS_THROW(Exception, "No pose uncertainty use currently supported");
  } else {
    UOplus.setIdentity();//设置成单位阵
    UOplus.bottomRightCorner<3, 3>() *= 1e-8;//对矩阵的右下角3*3矩阵赋值
    uint64_t currentId = estimator_->currentFrameId();//当前帧的id
    //如果当前帧在IMU的窗口(当前帧状态的exist为真)中
    if (estimator_->isInImuWindow(currentId) && (mfIdA_ != mfIdB_)) {
      okvis::SpeedAndBias speedAndBias;
      estimator_->getSpeedAndBias(currentId, 0, speedAndBias);//提取当前帧对应的速度和偏差(0表示第0个IMU,唯一的IMU)
      double scale = std::max(1.0, speedAndBias.head<3>().norm());//提取速度的模值
      UOplus.topLeftCorner<3, 3>() *= (scale * scale) * 1.0e-2;//对矩阵左上角进行赋值
    } else {
      UOplus.topLeftCorner<3, 3>() *= 4e-8;//直接对矩阵的左上角赋值
    }
  }

  // now set the frames and uncertainty
  //UOplus前三行表示位置的变化量
  //UOplus后三行表示角度的变化量
  //设置立体三角化的可能值，这里进行了pose_error和相关雅各比的计算
  probabilisticStereoTriangulator_.resetFrames(frameA_, frameB_, camIdA_,
                                               camIdB_, T_CaCb_, UOplus);

  // reset the match counter
  numMatches_ = 0;
  numUncertainMatches_ = 0;

  const size_t numA = frameA_->numKeypoints(camIdA_);//A帧中A相机的特征点数量
  skipA_.clear();
  skipA_.resize(numA, false);//大小为numA一个逻辑容器
  raySigmasA_.resize(numA);//大小为numA一个double容器
  // calculate projections only once
  if (matchingType_ == Match3D2D) {
    // allocate a matrix to store projections
    //sizeA（）表示A帧中A相机的特征点数量
    projectionsIntoB_ = Eigen::Matrix<double, Eigen::Dynamic, 2>::Zero(sizeA(),
                                                                       2);
    projectionsIntoBUncertainties_ =
        Eigen::Matrix<double, Eigen::Dynamic, 2>::Zero(sizeA() * 2, 2);

    // do the projections for each keypoint, if applicable
    for (size_t k = 0; k < numA; ++k) {
      uint64_t lm_id = frameA_->landmarkId(camIdA_, k);//提取A帧中A相机对应的第k个地标点
      //该特征点没有地标点，直接跳过
      if (lm_id == 0 || !estimator_->isLandmarkAdded(lm_id)) {
        // this can happen, if you called the 2D-2D version just before,
        // without inserting the landmark into the graph
        skipA_[k] = true;
        continue;
      }

      okvis::MapPoint landmark;
      estimator_->getLandmark(lm_id, landmark);//得到地标点
      Eigen::Vector4d hp_W = landmark.point;//地标点的齐次坐标
      //判断地标点是否被初始化
      if (!estimator_->isLandmarkInitialized(lm_id)) {
        skipA_[k] = true;
        continue;
      }

      // project (distorted)
      Eigen::Vector2d kptB;
      const Eigen::Vector4d hp_Cb = T_CbW_ * hp_W;//投影到b相机上
      //将空间点从相机坐标系中投影到像素坐标系上，经过了畸变
      //投影失败则直接跳过
      if (frameB_->geometryAs<CAMERA_GEOMETRY_T>(camIdB_)->projectHomogeneous(
          hp_Cb, &kptB)
          != okvis::cameras::CameraBase::ProjectionStatus::Successful) {
        skipA_[k] = true;
        continue;
      }
      //地标点被观测的次数小于2,直接跳过
      if (landmark.observations.size() < 2) {
        estimator_->setLandmarkInitialized(lm_id, false);
        skipA_[k] = true;
        continue;
      }

      // project and get uncertainty
      Eigen::Matrix<double, 2, 4> jacobian;
      Eigen::Matrix4d P_C = Eigen::Matrix4d::Zero();
      //速度模值的对角阵对P_C进行赋值
      P_C.topLeftCorner<3, 3>() = UOplus.topLeftCorner<3, 3>();  // get from before -- velocity scaled
      //将hp_Cb投影到B相机的像素坐标系kptB中,jacobian为像素坐标kptB对相机坐标系中点hp_Cb的雅各比矩阵
      frameB_->geometryAs<CAMERA_GEOMETRY_T>(camIdB_)->projectHomogeneous(
          hp_Cb, &kptB, &jacobian);
      ///这个矩阵的意思是:
      /// 雅各比矩阵*速度模值对角阵*雅各比矩阵T
      projectionsIntoBUncertainties_.block<2, 2>(2 * k, 0) = jacobian * P_C
          * jacobian.transpose();
      projectionsIntoB_.row(k) = kptB;//储存投影像素点(A相机第k个特征点对应的匹配点在B相机上的像素坐标)

      // precalculate ray uncertainties
      double keypointAStdDev;
      frameA_->getKeypointSize(camIdA_, k, keypointAStdDev);//该特征点的直径大小
      keypointAStdDev = 0.8 * keypointAStdDev / 12.0;
      //计算特征点的偏差
      raySigmasA_[k] = sqrt(sqrt(2)) * keypointAStdDev / fA_;  // (sqrt(MeasurementCovariance.norm()) / _fA)
    }
  } else {
    //表示2D-2D的特征点匹配
    for (size_t k = 0; k < numA; ++k) {
      double keypointAStdDev;
      frameA_->getKeypointSize(camIdA_, k, keypointAStdDev);//得到A帧中A相机图像中特征点k的直径大小
      keypointAStdDev = 0.8 * keypointAStdDev / 12.0;//特征点对应的标准差
      raySigmasA_[k] = sqrt(sqrt(2)) * keypointAStdDev / fA_;
      //表示特征点没有对应的地标点
      if (frameA_->landmarkId(camIdA_, k) == 0) {
        continue;
      }
      if (estimator_->isLandmarkAdded(frameA_->landmarkId(camIdA_, k))) {
        if (estimator_->isLandmarkInitialized(
            frameA_->landmarkId(camIdA_, k))) {
          //表示特征点存在地标点且地标点已经被初始化，则匹配过程直接跳过该特征点
          skipA_[k] = true;
        }
      }
    }
  }
  const size_t numB = frameB_->numKeypoints(camIdB_);//得到B帧B相机特征点的数量
  skipB_.clear();
  skipB_.reserve(numB);
  raySigmasB_.resize(numB);
  // do the projections for each keypoint, if applicable
  //如果是3D-2D的匹配
  if (matchingType_ == Match3D2D) {
    for (size_t k = 0; k < numB; ++k) {
      okvis::MapPoint landmark;
      //如果B帧B相机中存在特征点对应的地标点且地标点已经被添加到了状态求解器中
      if (frameB_->landmarkId(camIdB_, k) != 0
          && estimator_->isLandmarkAdded(frameB_->landmarkId(camIdB_, k))) {
        estimator_->getLandmark(frameB_->landmarkId(camIdB_, k), landmark);//得到该地标点
        //如果地标点已经可以被B帧B相机观测到，且就对应第k个特征点，则认为skipB_为真
        skipB_.push_back(
            landmark.observations.find(
                okvis::KeypointIdentifier(mfIdB_, camIdB_, k))
                != landmark.observations.end());
      } else {
        skipB_.push_back(false);//其他情况都不能跳过对该特征点的检测
      }
      double keypointBStdDev;
      frameB_->getKeypointSize(camIdB_, k, keypointBStdDev);//得到B帧B相机特征点k的像素半径
      keypointBStdDev = 0.8 * keypointBStdDev / 12.0;//计算特征点的标准差
      raySigmasB_[k] = sqrt(sqrt(2)) * keypointBStdDev / fB_;
    }
  } else {
    //2D-2D的匹配
    for (size_t k = 0; k < numB; ++k) {
      double keypointBStdDev;
      frameB_->getKeypointSize(camIdB_, k, keypointBStdDev);//得到特征点k的像素半径
      keypointBStdDev = 0.8 * keypointBStdDev / 12.0;//特征点的标准差
      raySigmasB_[k] = sqrt(sqrt(2)) * keypointBStdDev / fB_;//特征点的sigma
      //特征点还没有对应的地标点，则认为不能跳过
      if (frameB_->landmarkId(camIdB_, k) == 0) {
        skipB_.push_back(false);
        continue;
      }
      //求解器中已经添加了k特征点对应的地标点
      if (estimator_->isLandmarkAdded(frameB_->landmarkId(camIdB_, k))) {
        //如果已经初始化了，则认为可以跳过
        skipB_.push_back(
            estimator_->isLandmarkInitialized(frameB_->landmarkId(camIdB_, k)));  // old: isSet - check.
      } else {
        skipB_.push_back(false);//求解器中还没有该特征点对应的地标点，则认为不能跳过
      }
    }
  }

}

// What is the size of list A?
template<class CAMERA_GEOMETRY_T>
size_t VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::sizeA() const {
  return frameA_->numKeypoints(camIdA_);
}
// What is the size of list B?
template<class CAMERA_GEOMETRY_T>
size_t VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::sizeB() const {
  return frameB_->numKeypoints(camIdB_);
}

// Set the distance threshold for which matches exceeding it will not be returned as matches.
template<class CAMERA_GEOMETRY_T>
void VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::setDistanceThreshold(
    float distanceThreshold) {
  distanceThreshold_ = distanceThreshold;
}

// Get the distance threshold for which matches exceeding it will not be returned as matches.
template<class CAMERA_GEOMETRY_T>
float VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::distanceThreshold() const {
  return distanceThreshold_;
}

// Geometric verification of a match.
///用几何模型对匹配点对进行筛选
template<class CAMERA_GEOMETRY_T>
bool VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::verifyMatch(
    size_t indexA, size_t indexB) const {

  if (matchingType_ == Match2D2D) {

    // potential 2d2d match - verify by triangulation
    Eigen::Vector4d hP;
    bool isParallel;
    bool valid = probabilisticStereoTriangulator_.stereoTriangulate(
        indexA, indexB, hP, isParallel, std::max(raySigmasA_[indexA], raySigmasB_[indexB]));
    if (valid) {
      return true;
    }
  } else {
    // get projection into B
    // 3D-2D
    Eigen::Vector2d kptB = projectionsIntoB_.row(indexA);

    // uncertainty
    double keypointBStdDev;
    frameB_->getKeypointSize(camIdB_, indexB, keypointBStdDev);//得到B帧中特征点的直径
    keypointBStdDev = 0.8 * keypointBStdDev / 12.0;//B帧中特征点的标准差
    //U表示B帧中特征点的协方差+由于速度引起的投影误差（B特征点的一个信息矩阵）
    Eigen::Matrix2d U = Eigen::Matrix2d::Identity() * keypointBStdDev
        * keypointBStdDev
        + projectionsIntoBUncertainties_.block<2, 2>(2 * indexA, 0);

    Eigen::Vector2d keypointBMeasurement;
    frameB_->getKeypoint(camIdB_, indexB, keypointBMeasurement);
    Eigen::Vector2d err = kptB - keypointBMeasurement;
    const int chi2 = err.transpose() * U.inverse() * err;//衡量A帧中特征点到B帧投影与B帧中原始点的偏差
    //偏差过小，则认为匹配成功
    if (chi2 < 4.0) {
      return true;
    }
  }
  return false;
}

// A function that tells you how many times setMatching() will be called.
template<class CAMERA_GEOMETRY_T>
void VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::reserveMatches(
    size_t /*numMatches*/) {
  //_triangulatedPoints.clear();
}

// Get the number of matches.
template<class CAMERA_GEOMETRY_T>
size_t VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::numMatches() {
  return numMatches_;
}

// Get the number of uncertain matches.
template<class CAMERA_GEOMETRY_T>
size_t VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::numUncertainMatches() {
  return numUncertainMatches_;
}

// At the end of the matching step, this function is called once
// for each pair of matches discovered.
// 设置最佳匹配并储存,形参
///A帧中特征点序号
/// B帧中特征点序号
/// 最佳匹配距离
template<class CAMERA_GEOMETRY_T>
void VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::setBestMatch(
    size_t indexA, size_t indexB, double /*distance*/) {

  // assign correspondences
  uint64_t lmIdA = frameA_->landmarkId(camIdA_, indexA);//提取特征点对应的地标点序号
  uint64_t lmIdB = frameB_->landmarkId(camIdB_, indexB);//提取特征点对应的地标点序号

  if (matchingType_ == Match2D2D) {

    // check that not both are set
    // 表示两个特征点都已经了初始化,不需要再进行匹配
    if (lmIdA != 0 && lmIdB != 0) {
      return;
    }

    // re-triangulate...
    // potential 2d2d match - verify by triangulation
    Eigen::Vector4d hP_Ca;
    bool canBeInitialized;
    //三角化检测,hP_Ca为匹配点在A帧相机坐标系中的齐次坐标(归一化后的)
    //canBeInitialized表示是否可以准确初始化
    bool valid = probabilisticStereoTriangulator_.stereoTriangulate(
        indexA, indexB, hP_Ca, canBeInitialized,
        std::max(raySigmasA_[indexA], raySigmasB_[indexB]));
    if (!valid) {
      return;
    }

    // get the uncertainty
    //为真,为真表示三角化中矩阵可逆,不平行（不是纯旋转）,三角化存在不确定性
    //getUncertainty中得到了重投影误差和相关雅各比矩阵
    if (canBeInitialized) {  // know more exactly
      Eigen::Matrix3d pointUOplus_A;
      probabilisticStereoTriangulator_.getUncertainty(indexA, indexB, hP_Ca,
                                                      pointUOplus_A,
                                                      canBeInitialized);
    }

    // check and adapt landmark status
    bool insertA = lmIdA == 0;
    bool insertB = lmIdB == 0;
    bool insertHomogeneousPointParameterBlock = false;
    uint64_t lmId = 0;  // 0 just to avoid warning
    //未创建特征点
    if (insertA && insertB) {
      // ok, we need to assign a new Id...
      lmId = okvis::IdProvider::instance().newId();
      frameA_->setLandmarkId(camIdA_, indexA, lmId);//设置特征点对应地标点的序号
      frameB_->setLandmarkId(camIdB_, indexB, lmId);
      lmIdA = lmId;
      lmIdB = lmId;
      // and add it to the graph
      insertHomogeneousPointParameterBlock = true;
    } else {
       //表示A帧特征点已经有对应的地标点
      if (!insertA) {
        lmId = lmIdA;
        //状态空间中如果未添加A帧特征点
        if (!estimator_->isLandmarkAdded(lmId)) {
          // add landmark and observation to the graph
          insertHomogeneousPointParameterBlock = true;//需要再添加一次
          insertA = true;//需要再添加一次
        }
      }
      //表示B帧特征点已经有对应的地标点
      if (!insertB) {
        lmId = lmIdB;
        //如果状态空间中未添加B帧特征点
        if (!estimator_->isLandmarkAdded(lmId)) {
          // add landmark and observation to the graph
          insertHomogeneousPointParameterBlock = true;//需要再添加一次
          insertB = true;//需要再添加一次
        }
      }
    }
    // add landmark to graph if necessary
    if (insertHomogeneousPointParameterBlock) {
      estimator_->addLandmark(lmId, T_WCa_ * hP_Ca);//添加地标点
      OKVIS_ASSERT_TRUE(Exception, estimator_->isLandmarkAdded(lmId),
                        lmId<<" not added, bug");
      estimator_->setLandmarkInitialized(lmId, canBeInitialized);//初始化地标点
    } else {
      //如果该对匹配点已经被添加为了地标点
      // update initialization status, set better estimate, if possible
      if (canBeInitialized) {
        estimator_->setLandmarkInitialized(lmId, true);
        estimator_->setLandmark(lmId, T_WCa_ * hP_Ca);//重置地标点
      }
    }

    // in image A
    // A帧还没有对应的地标点
    okvis::MapPoint landmark;
    //A帧还没有对应的地标点，且地标点在A帧中没有对应特征点的观测
    if (insertA
        && landmark.observations.find(
            okvis::KeypointIdentifier(mfIdA_, camIdA_, indexA))
            == landmark.observations.end()) {  // ensure no double observations...
            // TODO hp_Sa NOT USED!
      Eigen::Vector4d hp_Sa(T_SaCa_ * hP_Ca);//转换到IMU坐标系中
      hp_Sa.normalize();//归一化
      frameA_->setLandmarkId(camIdA_, indexA, lmId);//给A帧中设置地标点的id
      lmIdA = lmId;//
      // initialize in graph
      OKVIS_ASSERT_TRUE(Exception, estimator_->isLandmarkAdded(lmId),
                        "landmark id=" << lmId<<" not added");
      estimator_->addObservation<camera_geometry_t>(lmId, mfIdA_, camIdA_,
                                                    indexA);//给第lmId个地标点添加A帧indexA特征点的观测
    }

    // in image B
    // B帧中该特征点还没有对应地标点，且该地标点在B帧中没有对应特征点的观测
    if (insertB
        && landmark.observations.find(
            okvis::KeypointIdentifier(mfIdB_, camIdB_, indexB))
            == landmark.observations.end()) {  // ensure no double observations...
      Eigen::Vector4d hp_Sb(T_SbCb_ * T_CbCa_ * hP_Ca);//转换到IMU坐标系中
      hp_Sb.normalize();//归一化
      frameB_->setLandmarkId(camIdB_, indexB, lmId);//给B帧设置地标点的id
      lmIdB = lmId;
      // initialize in graph
      OKVIS_ASSERT_TRUE(Exception, estimator_->isLandmarkAdded(lmId),
                        "landmark " << lmId << " not added");
      estimator_->addObservation<camera_geometry_t>(lmId, mfIdB_, camIdB_,
                                                    indexB);//给第lmId个地标点添加B帧index特征点的观测
    }

    // let's check for consistency with other observations:
    okvis::ceres::HomogeneousPointParameterBlock point(T_WCa_ * hP_Ca, 0);
    if(canBeInitialized)
      estimator_->setLandmark(lmId, point.estimate());//更新地标点

  } else {
    OKVIS_ASSERT_TRUE_DBG(Exception,lmIdB==0,"bug. Id in frame B already set.");
    // 3D-2D匹配
    // get projection into B
    Eigen::Vector2d kptB = projectionsIntoB_.row(indexA);//A帧中第indexA个特征点投影在B帧上的像素坐标
    Eigen::Vector2d keypointBMeasurement;
    frameB_->getKeypoint(camIdB_, indexB, keypointBMeasurement);//B帧中特征点的观测值（真实值）

    Eigen::Vector2d err = kptB - keypointBMeasurement;//偏差
    double keypointBStdDev;
    frameB_->getKeypointSize(camIdB_, indexB, keypointBStdDev);//B帧中特征点的直径
    keypointBStdDev = 0.8 * keypointBStdDev / 12.0;
    Eigen::Matrix2d U_tot = Eigen::Matrix2d::Identity() * keypointBStdDev
        * keypointBStdDev
        + projectionsIntoBUncertainties_.block<2, 2>(2 * indexA, 0);//B帧中特征点的信息矩阵

    const double chi2 = err.transpose().eval() * U_tot.inverse() * err;//得到一个衡量投影误差的标准
    //误差过大
    if (chi2 > 4.0) {
      return;
    }

    // saturate allowed image uncertainty
    // 协方差矩阵的模值过大，则认为不确定
    if (U_tot.norm() > 25.0 / (keypointBStdDev * keypointBStdDev * sqrt(2))) {
      numUncertainMatches_++;
      //return;
    }

    frameB_->setLandmarkId(camIdB_, indexB, lmIdA);//B帧中添加地标点
    lmIdB = lmIdA;
    okvis::MapPoint landmark;
    estimator_->getLandmark(lmIdA, landmark);//得到地标点的信息

    // initialize in graph
    if (landmark.observations.find(
        okvis::KeypointIdentifier(mfIdB_, camIdB_, indexB))
        == landmark.observations.end()) {  // ensure no double observations...
      OKVIS_ASSERT_TRUE(Exception, estimator_->isLandmarkAdded(lmIdB),
                        "not added");
      estimator_->addObservation<camera_geometry_t>(lmIdB, mfIdB_, camIdB_,
                                                    indexB);//对该地标点添加B帧第indexB个特征点的观测
    }

  }
  numMatches_++;//匹配数量加一
}

template class VioKeyframeWindowMatchingAlgorithm<
    okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion> > ;

template class VioKeyframeWindowMatchingAlgorithm<
    okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion> > ;

template class VioKeyframeWindowMatchingAlgorithm<
    okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion8> > ;

}
