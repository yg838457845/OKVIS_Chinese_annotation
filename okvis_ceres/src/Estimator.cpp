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
 *  Created on: Dec 30, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file Estimator.cpp
 * @brief Source file for the Estimator class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <glog/logging.h>
#include <okvis/Estimator.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/ImuError.hpp>
#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/RelativePoseError.hpp>
#include <okvis/ceres/SpeedAndBiasError.hpp>
#include <okvis/IdProvider.hpp>
#include <okvis/MultiFrame.hpp>
#include <okvis/assert_macros.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// Constructor if a ceres map is already available.
Estimator::Estimator(
    std::shared_ptr<okvis::ceres::Map> mapPtr)
    : mapPtr_(mapPtr),
      referencePoseId_(0),
      cauchyLossFunctionPtr_(new ::ceres::CauchyLoss(1)),
      huberLossFunctionPtr_(new ::ceres::HuberLoss(1)),
      marginalizationResidualId_(0)
{
}

// The default constructor.
Estimator::Estimator()
    : mapPtr_(new okvis::ceres::Map()),
      referencePoseId_(0),
      cauchyLossFunctionPtr_(new ::ceres::CauchyLoss(1)),
      huberLossFunctionPtr_(new ::ceres::HuberLoss(1)),
      marginalizationResidualId_(0)
{
}

Estimator::~Estimator()
{
}

// Add a camera to the configuration. Sensors can only be added and never removed.
int Estimator::addCamera(
    const ExtrinsicsEstimationParameters & extrinsicsEstimationParameters)
{
  extrinsicsEstimationParametersVec_.push_back(extrinsicsEstimationParameters);
  return extrinsicsEstimationParametersVec_.size() - 1;
}

// Add an IMU to the configuration.
int Estimator::addImu(const ImuParameters & imuParameters)
{
  if(imuParametersVec_.size()>1){
    LOG(ERROR) << "only one IMU currently supported";
    return -1;
  }
  imuParametersVec_.push_back(imuParameters);//添加IMU参数到容器imuParametersVec_中
  return imuParametersVec_.size() - 1;
}

// Remove all cameras from the configuration
void Estimator::clearCameras(){
  extrinsicsEstimationParametersVec_.clear();
}

// Remove all IMUs from the configuration.
void Estimator::clearImus(){
  imuParametersVec_.clear();
}

// Add a pose to the state.
//状态添加函数
bool Estimator::addStates(
    okvis::MultiFramePtr multiFrame,
    const okvis::ImuMeasurementDeque & imuMeasurements,
    bool asKeyframe)
{
  // note: this is before matching...
  // TODO !!
  okvis::kinematics::Transformation T_WS;
  okvis::SpeedAndBias speedAndBias;
  if (statesMap_.empty()) {
    // in case this is the first frame ever, let's initialize the pose:
    bool success0 = initPoseFromImu(imuMeasurements, T_WS);//利用惯导数据初始化姿态
    OKVIS_ASSERT_TRUE_DBG(Exception, success0,
        "pose could not be initialized from imu measurements.");
    if (!success0)
      return false;
    speedAndBias.setZero();//速度与偏差的函数
    speedAndBias.segment<3>(6) = imuParametersVec_.at(0).a0;//a0表示先验加速度计偏差的平均值
  } else {
    // get the previous states
    uint64_t T_WS_id = statesMap_.rbegin()->second.id;//提取状态空间中最后一个状态量对应的id
    //提取状态空间中最后一个状态量对应的传感器IMU数组，数组中的第一个容器，容器中表示速度參差的元素，元素的序号
    uint64_t speedAndBias_id = statesMap_.rbegin()->second.sensors.at(SensorStates::Imu)
        .at(0).at(ImuSensorStates::SpeedAndBias).id;
    OKVIS_ASSERT_TRUE_DBG(Exception, mapPtr_->parameterBlockExists(T_WS_id),
                       "this is an okvis bug. previous pose does not exist.");
    //static_pointer_cast指针转换函数，将ParameterBlock转换为PoseParameterBlock，这样estimate（）输出的就是位姿信息
    T_WS = std::static_pointer_cast<ceres::PoseParameterBlock>(
        mapPtr_->parameterBlockPtr(T_WS_id))->estimate();//提取对应的位姿估计值
    //OKVIS_ASSERT_TRUE_DBG(
    //    Exception, speedAndBias_id,
    //    "this is an okvis bug. previous speedAndBias does not exist.");
    //提取相应的速度与偏差的估计值
    speedAndBias =
        std::static_pointer_cast<ceres::SpeedAndBiasParameterBlock>(
            mapPtr_->parameterBlockPtr(speedAndBias_id))->estimate();

    // propagate pose and speedAndBias
    //迭代更新位姿和速度、偏差
    int numUsedImuMeasurements = ceres::ImuError::propagation(
        imuMeasurements, imuParametersVec_.at(0), T_WS, speedAndBias,
        statesMap_.rbegin()->second.timestamp, multiFrame->timestamp());
    OKVIS_ASSERT_TRUE_DBG(Exception, numUsedImuMeasurements > 1,
                       "propagation failed");
    if (numUsedImuMeasurements < 1){
      LOG(INFO) << "numUsedImuMeasurements=" << numUsedImuMeasurements;
      return false;
    }
  }


  // create a states object:
  //初始化一个状态量
  States states(asKeyframe, multiFrame->id(), multiFrame->timestamp());

  // check if id was used before
  OKVIS_ASSERT_TRUE_DBG(Exception,
      statesMap_.find(states.id)==statesMap_.end(),
      "pose ID" <<states.id<<" was used before!");

  // create global states
  //创建位姿的一个类
  std::shared_ptr<okvis::ceres::PoseParameterBlock> poseParameterBlock(
      new okvis::ceres::PoseParameterBlock(T_WS, states.id,
                                           multiFrame->timestamp()));
  //状态中的global是一个6维的数组，每一维元素都是一个stateinfo
  states.global.at(GlobalStates::T_WS).exists = true;
  states.global.at(GlobalStates::T_WS).id = states.id;

  if(statesMap_.empty())
  {
    referencePoseId_ = states.id; // set this as reference pose创建一个參考的ID
    //在地图指针mapPtr_中添加位姿块poseParameterBlock，以李代数的形式
    if (!mapPtr_->addParameterBlock(poseParameterBlock,ceres::Map::Pose6d)) {
      return false;
    }
  } else {
    if (!mapPtr_->addParameterBlock(poseParameterBlock,ceres::Map::Pose6d)) {
      return false;
    }
  }

  // add to buffer
  statesMap_.insert(std::pair<uint64_t, States>(states.id, states));//将状态加入状态空间
  multiFramePtrMap_.insert(std::pair<uint64_t, okvis::MultiFramePtr>(states.id, multiFrame));//将帧加入帧类地图

  // the following will point to the last states:
  std::map<uint64_t, States>::reverse_iterator lastElementIterator = statesMap_.rbegin();//状态地图中的最后一个元素（id，状态）
  lastElementIterator++;//迭代器前移一位,指向前一个状态,即新加入状态的上一个状态

  // initialize new sensor states
  // cameras:
  //相机状态的初始化
  //extrinsicsEstimationParametersVec_中储存了两个相机的外参
  for (size_t i = 0; i < extrinsicsEstimationParametersVec_.size(); ++i) {

    SpecificSensorStatesContainer cameraInfos(2);//初始化一个2维大小的容器，容器中的元素类型为StateInfo（状态的信息类）
    cameraInfos.at(CameraSensorStates::T_SCi).exists=true;//二维容器的第一维（T_SCi状态）赋值已存在
    cameraInfos.at(CameraSensorStates::Intrinsics).exists=false;//二维容器中的第二维(内参状态)赋值不存在
    //第i个相机的相对位移和相对旋转的标准差小于阈值
    if(((extrinsicsEstimationParametersVec_.at(i).sigma_c_relative_translation<1e-12)||
        (extrinsicsEstimationParametersVec_.at(i).sigma_c_relative_orientation<1e-12))&&
        (statesMap_.size() > 1)){
      // use the same block...
      //将状态地图倒数第二个状态的相机传感器数组中第i个相机的T_SCi位姿对应元素的id赋值给相机信息容器
      cameraInfos.at(CameraSensorStates::T_SCi).id =
          lastElementIterator->second.sensors.at(SensorStates::Camera).at(i).at(CameraSensorStates::T_SCi).id;
    } else {
      const okvis::kinematics::Transformation T_SC = *multiFrame->T_SC(i);
      uint64_t id = IdProvider::instance().newId();//赋值个新的id
      std::shared_ptr<okvis::ceres::PoseParameterBlock> extrinsicsParameterBlockPtr(
          new okvis::ceres::PoseParameterBlock(T_SC, id,
                                               multiFrame->timestamp()));//创建一个新的位姿块变量
      //往地图中添加位姿块
      if(!mapPtr_->addParameterBlock(extrinsicsParameterBlockPtr,ceres::Map::Pose6d)){
        return false;
      }
      cameraInfos.at(CameraSensorStates::T_SCi).id = id;//赋值相机信息容器中T_SCi对应元素中id的值(新值)
    }
    // update the states info
    //将相机信息容器添加到新添加状态的相机传感器容器中
    statesMap_.rbegin()->second.sensors.at(SensorStates::Camera).push_back(cameraInfos);
    states.sensors.at(SensorStates::Camera).push_back(cameraInfos);//新建的状态也跟着更新
  }

  // IMU states are automatically propagated.
  //IMU状态的更新
  for (size_t i=0; i<imuParametersVec_.size(); ++i){
    SpecificSensorStatesContainer imuInfo(2);//初始化一个2维大小的容器，容器中的元素类型为StateInfo（状态的信息类）
    imuInfo.at(ImuSensorStates::SpeedAndBias).exists = true;//初始化容器中速度\偏差对应元素(状态)的存在性
    uint64_t id = IdProvider::instance().newId();//赋值一个新的id
    //初始化一个新的速度\偏差的块
    std::shared_ptr<okvis::ceres::SpeedAndBiasParameterBlock> speedAndBiasParameterBlock(
        new okvis::ceres::SpeedAndBiasParameterBlock(speedAndBias, id, multiFrame->timestamp()));
    //往地图中添加速度\偏差的块
    if(!mapPtr_->addParameterBlock(speedAndBiasParameterBlock)){
      return false;
    }
    imuInfo.at(ImuSensorStates::SpeedAndBias).id = id;//赋值IMU信息容器中速度\偏差对应元素(状态)的id
    //更新状态地图的信息
    statesMap_.rbegin()->second.sensors.at(SensorStates::Imu).push_back(imuInfo);
    //更新状态的信息
    states.sensors.at(SensorStates::Imu).push_back(imuInfo);
  }

  // depending on whether or not this is the very beginning, we will add priors or relative terms to the last state:
  //表示在最开始,刚刚只添加了一维状态
  if (statesMap_.size() == 1) {
    // let's add a prior
    //添加相机位姿的残差
    Eigen::Matrix<double,6,6> information = Eigen::Matrix<double,6,6>::Zero();//信息矩阵
    information(5,5) = 1.0e8; information(0,0) = 1.0e8; information(1,1) = 1.0e8; information(2,2) = 1.0e8;
    std::shared_ptr<ceres::PoseError > poseError(new ceres::PoseError(T_WS, information));//初始化位姿误差的指针
    /*auto id2= */ mapPtr_->addResidualBlock(poseError,NULL,poseParameterBlock);//往地图添加參差块
    //mapPtr_->isJacobianCorrect(id2,1.0e-6);

    // sensor states
    //相机的状态,主要是外参(SCI)的残差
    for (size_t i = 0; i < extrinsicsEstimationParametersVec_.size(); ++i) {
      double translationStdev = extrinsicsEstimationParametersVec_.at(i).sigma_absolute_translation;//第i个相机的绝对平移标准差
      double translationVariance = translationStdev*translationStdev;//求取方差
      double rotationStdev = extrinsicsEstimationParametersVec_.at(i).sigma_absolute_orientation;//第i个相机的绝对旋转标准差
      double rotationVariance = rotationStdev*rotationStdev;//求取方差
      //如果方差过大
      if(translationVariance>1.0e-16 && rotationVariance>1.0e-16){
        const okvis::kinematics::Transformation T_SC = *multiFrame->T_SC(i);//当前帧的外参位姿
        //初始化第i相机的SC矩阵（外参）的误差
        std::shared_ptr<ceres::PoseError > cameraPoseError(
              new ceres::PoseError(T_SC, translationVariance, rotationVariance));
        // add to map
        mapPtr_->addResidualBlock(
            cameraPoseError,
            NULL,
            mapPtr_->parameterBlockPtr(
                states.sensors.at(SensorStates::Camera).at(i).at(CameraSensorStates::T_SCi).id));//提取当前状态第i个相机对应T_SCI(参数)的ID, 然后得到对应id个数据块
        //mapPtr_->isJacobianCorrect(id,1.0e-6);
      }
      else {
        mapPtr_->setParameterBlockConstant(
            states.sensors.at(SensorStates::Camera).at(i).at(CameraSensorStates::T_SCi).id);//提取当前状态第i个相机对应T_SCI(参数)的ID,设置该id对应的数据块在优化中保持不变
      }
    }
    //添加速度与偏置的误差
    for (size_t i = 0; i < imuParametersVec_.size(); ++i) {
      Eigen::Matrix<double,6,1> variances;
      // get these from parameter file
      const double sigma_bg = imuParametersVec_.at(0).sigma_bg;//Initial gyroscope bias.
      const double sigma_ba = imuParametersVec_.at(0).sigma_ba;//Initial accelerometer bias
      //初始化速度和偏置的误差
      std::shared_ptr<ceres::SpeedAndBiasError > speedAndBiasError(
            new ceres::SpeedAndBiasError(
                speedAndBias, 1.0, sigma_bg*sigma_bg, sigma_ba*sigma_ba));
      // add to map
      //向地图中添加速度和偏置的误差模块
      mapPtr_->addResidualBlock(
          speedAndBiasError,
          NULL,
          mapPtr_->parameterBlockPtr(
              states.sensors.at(SensorStates::Imu).at(i).at(ImuSensorStates::SpeedAndBias).id));
      //mapPtr_->isJacobianCorrect(id,1.0e-6);
    }
  }
  else{
    // add IMU error terms
    // 添加IMU的误差
    for (size_t i = 0; i < imuParametersVec_.size(); ++i) {
      //初始化ImuError
      std::shared_ptr<ceres::ImuError> imuError(
          new ceres::ImuError(imuMeasurements, imuParametersVec_.at(i),
                              lastElementIterator->second.timestamp,
                              states.timestamp));
      //往地图中添加IMU的残差块
      ///形参为：
      /// IMU的残差块
      /// NULL
      /// 地图中倒数第二个状态的(位姿)ID---->数据块
      /// 地图中倒数第二个状态的第i个IMU传感器的速度与偏置部分对应的ID----->数据块
      /// 地图中最新一个状态的(位姿)ID----->数据块
      /// 地图中最新一个状态的第i个IMU传感器的速度与偏置部分对应的ID------->数据块
      /*::ceres::ResidualBlockId id = */mapPtr_->addResidualBlock(
          imuError,
          NULL,
          mapPtr_->parameterBlockPtr(lastElementIterator->second.id),
          mapPtr_->parameterBlockPtr(
              lastElementIterator->second.sensors.at(SensorStates::Imu).at(i).at(
                  ImuSensorStates::SpeedAndBias).id),
          mapPtr_->parameterBlockPtr(states.id),
          mapPtr_->parameterBlockPtr(
              states.sensors.at(SensorStates::Imu).at(i).at(
                  ImuSensorStates::SpeedAndBias).id));
      //imuError->setRecomputeInformation(false);
      //mapPtr_->isJacobianCorrect(id,1.0e-9);
      //imuError->setRecomputeInformation(true);
    }

    // add relative sensor state errors
    // 添加相机外参(SC)的误差
    for (size_t i = 0; i < extrinsicsEstimationParametersVec_.size(); ++i) {
      //首先需要判断上一帧状态的外参ID和当前帧状态的外参ID是否相等,相等即表示相机的外参未进行更新
      if(lastElementIterator->second.sensors.at(SensorStates::Camera).at(i).at(CameraSensorStates::T_SCi).id !=
          states.sensors.at(SensorStates::Camera).at(i).at(CameraSensorStates::T_SCi).id){
        // i.e. they are different estimated variables, so link them with a temporal error term
        // 计算两帧间的时间间隔
        double dt = (states.timestamp - lastElementIterator->second.timestamp)
            .toSec();
        //第i个相机的相对平移误差
        double translationSigmaC = extrinsicsEstimationParametersVec_.at(i)
            .sigma_c_relative_translation;
        //平移的协方差
        double translationVariance = translationSigmaC * translationSigmaC * dt;
        //第i个相机的相对旋转误差
        double rotationSigmaC = extrinsicsEstimationParametersVec_.at(i)
            .sigma_c_relative_orientation;
        //旋转的协方差
        double rotationVariance = rotationSigmaC * rotationSigmaC * dt;
        //相对的外参误差
        std::shared_ptr<ceres::RelativePoseError> relativeExtrinsicsError(
            new ceres::RelativePoseError(translationVariance,
                                         rotationVariance));
        //地图中添加误差块
        ///形参
        /// 相对外参误差
        /// NULL
        /// 上一帧第i个相机对应T_SC的序号---->数据块
        /// 当前帧第i个相机对应T_SC的序号---->数据块
        mapPtr_->addResidualBlock(
            relativeExtrinsicsError,
            NULL,
            mapPtr_->parameterBlockPtr(
                lastElementIterator->second.sensors.at(SensorStates::Camera).at(
                    i).at(CameraSensorStates::T_SCi).id),
            mapPtr_->parameterBlockPtr(
                states.sensors.at(SensorStates::Camera).at(i).at(
                    CameraSensorStates::T_SCi).id));
        //mapPtr_->isJacobianCorrect(id,1.0e-6);
      }
    }
    // only camera. this is slightly inconsistent, since the IMU error term contains both
    // a term for global states as well as for the sensor-internal ones (i.e. biases).
    // TODO: magnetometer, pressure, ...
  }

  return true;
}

// Add a landmark.
// 添加地标点
bool Estimator::addLandmark(uint64_t landmarkId,
                            const Eigen::Vector4d & landmark) {
  std::shared_ptr<okvis::ceres::HomogeneousPointParameterBlock> pointParameterBlock(
      new okvis::ceres::HomogeneousPointParameterBlock(landmark, landmarkId));
  //cere地图中添加地标点的数据块
  if (!mapPtr_->addParameterBlock(pointParameterBlock,
                                  okvis::ceres::Map::HomogeneousPoint)) {
    return false;
  }

  // remember
  double dist = std::numeric_limits<double>::max();
  //计算欧式距离
  if(fabs(landmark[3])>1.0e-8){
    dist = (landmark/landmark[3]).head<3>().norm(); // euclidean distance
  }
  //将地标点插入地标点地图（landmarkId，MapPoint）中
  landmarksMap_.insert(
      std::pair<uint64_t, MapPoint>(
          landmarkId, MapPoint(landmarkId, landmark, 0.0, dist)));
  OKVIS_ASSERT_TRUE_DBG(Exception, isLandmarkAdded(landmarkId), "bug: inconsistend landmarkdMap_ with mapPtr_.");
  return true;
}

// Remove an observation from a landmark.
bool Estimator::removeObservation(::ceres::ResidualBlockId residualBlockId) {
  const ceres::Map::ParameterBlockCollection parameters = mapPtr_->parameters(residualBlockId);
  const uint64_t landmarkId = parameters.at(1).first;
  // remove in landmarksMap
  MapPoint& mapPoint = landmarksMap_.at(landmarkId);
  for(std::map<okvis::KeypointIdentifier, uint64_t >::iterator it = mapPoint.observations.begin();
      it!= mapPoint.observations.end(); ){
    if(it->second == uint64_t(residualBlockId)){

      it = mapPoint.observations.erase(it);
    } else {
      it++;
    }
  }
  // remove residual block
  mapPtr_->removeResidualBlock(residualBlockId);
  return true;
}

// Remove an observation from a landmark, if available.
bool Estimator::removeObservation(uint64_t landmarkId, uint64_t poseId,
                                  size_t camIdx, size_t keypointIdx) {
  if(landmarksMap_.find(landmarkId) == landmarksMap_.end()){
    for (PointMap::iterator it = landmarksMap_.begin(); it!= landmarksMap_.end(); ++it) {
      LOG(INFO) << it->first<<", no. obs = "<<it->second.observations.size();
    }
    LOG(INFO) << landmarksMap_.at(landmarkId).id;
  }
  OKVIS_ASSERT_TRUE_DBG(Exception, isLandmarkAdded(landmarkId),
                     "landmark not added");

  okvis::KeypointIdentifier kid(poseId,camIdx,keypointIdx);
  MapPoint& mapPoint = landmarksMap_.at(landmarkId);
  std::map<okvis::KeypointIdentifier, uint64_t >::iterator it = mapPoint.observations.find(kid);
  if(it == landmarksMap_.at(landmarkId).observations.end()){
    return false; // observation not present
  }

  // remove residual block
  mapPtr_->removeResidualBlock(reinterpret_cast< ::ceres::ResidualBlockId>(it->second));

  // remove also in local map
  mapPoint.observations.erase(it);

  return true;
}

/**
 * @brief Does a vector contain a certain element.
 * @tparam Class of a vector element.
 * @param vector Vector to search element in.
 * @param query Element to search for.
 * @return True if query is an element of vector.
 */
template<class T>
bool vectorContains(const std::vector<T> & vector, const T & query){
  for(size_t i=0; i<vector.size(); ++i){
    if(vector[i] == query){
      return true;
    }
  }
  return false;
}

// Applies the dropping/marginalization strategy according to the RSS'13/IJRR'14 paper.
// The new number of frames in the window will be numKeyframes+numImuFrames.
///边缘化函数
/// numKeyframes=5
/// numImuFrames=3
bool Estimator::applyMarginalizationStrategy(
    size_t numKeyframes, size_t numImuFrames,
    okvis::MapPointVector& removedLandmarks)
{
  // keep the newest numImuFrames
  std::map<uint64_t, States>::reverse_iterator rit = statesMap_.rbegin();
  //状态空间中的状态数目大于numImuFrames（3）
  for(size_t k=0; k<numImuFrames; k++){
    rit++;
    if(rit==statesMap_.rend()){
      // nothing to do.
      return true;
    }
  }

  // remove linear marginalizationError, if existing
  // 移除残差块，如果需要的话
  if (marginalizationErrorPtr_ && marginalizationResidualId_) {
    bool success = mapPtr_->removeResidualBlock(marginalizationResidualId_);//移除边缘化残差块
    OKVIS_ASSERT_TRUE_DBG(Exception, success,
                       "could not remove marginalization error");
    marginalizationResidualId_ = 0;
    if (!success)
      return false;
  }

  // these will keep track of what we want to marginalize out.
  std::vector<uint64_t> paremeterBlocksToBeMarginalized;
  std::vector<bool> keepParameterBlocks;
  //重置指针marginalizationErrorPtr_
  if (!marginalizationErrorPtr_) {
    marginalizationErrorPtr_.reset(
        new ceres::MarginalizationError(*mapPtr_.get()));
  }

  // distinguish if we marginalize everything or everything but pose
  std::vector<uint64_t> removeFrames;
  std::vector<uint64_t> removeAllButPose;
  std::vector<uint64_t> allLinearizedFrames;
  size_t countedKeyframes = 0;
  ///removeAllButPose表示窗口（3帧）之前的所有未剔除的状态（帧），这些帧（包括关键帧）除了位姿之外的所有数据块（速度/偏置）都被进行边缘化
  /// removeFrames表示窗口之前的所有未剔除的状态，但是不包括临近窗口的5个关键帧，这些帧的（位姿，外参）数据块都被进行边缘化
  /// allLinearizedFrames表示窗口（3帧）之前的所有为剔除的状态（帧）
  while (rit != statesMap_.rend()) {
    //如果rit帧不是关键帧或者关键帧的数量大于5
    if (!rit->second.isKeyframe || countedKeyframes >= numKeyframes) {
      removeFrames.push_back(rit->second.id);//添加满足条件的帧到可移除帧(边缘化）集合(包括不为关键帧的c-3）
    } else {
      countedKeyframes++;//是关键帧，关键帧数量加一
    }
    removeAllButPose.push_back(rit->second.id);//添加3帧之前（包括c-3）的所有帧到该集合
    allLinearizedFrames.push_back(rit->second.id);//添加3帧之前（包括c-3）的所有帧到该集合
    ++rit;// check the next frame
  }

  // marginalize everything but pose:
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 边缘化所有除了位姿
  for(size_t k = 0; k<removeAllButPose.size(); ++k){
    std::map<uint64_t, States>::iterator it = statesMap_.find(removeAllButPose[k]);//得到需要边缘化的状态
    //遍历每一个状态的global类型（6维的数组）
    ///好像在OKVIS中只有状态的global变量只有GlobalStates::T_WS位置有值
    /// 除了状态位姿，其他global类对应残差的添加
    for (size_t i = 0; i < it->second.global.size(); ++i) {
      if (i == GlobalStates::T_WS) {
        continue; // we do not remove the pose here.
      }
      //不存在直接跳过
      if (!it->second.global[i].exists) {
        continue; // if it doesn't exist, we don't do anything.
      }
      //设置为TWS数据块为固定，直接跳过
      //global的id和状态的id相等，而状态的id和位姿（TWS）数据块的id相等
      if (mapPtr_->parameterBlockPtr(it->second.global[i].id)->fixed()) {
        continue;  // we never eliminate fixed blocks.
      }
      std::map<uint64_t, States>::iterator checkit = it;
      checkit++;
      // only get rid of it, if it's different
      // 与下一个状态id保持一致，则直接跳过
      if(checkit->second.global[i].exists &&
          checkit->second.global[i].id == it->second.global[i].id){
        continue;
      }
      ///状态的存在性定义为false
      it->second.global[i].exists = false; // remember we removed
      ///添加状态第i个全局量（TSW，等等）id（即是数据块的id）到容器中
      paremeterBlocksToBeMarginalized.push_back(it->second.global[i].id);
      keepParameterBlocks.push_back(false);
      ceres::Map::ResidualBlockCollection residuals = mapPtr_->residuals(
          it->second.global[i].id);//提取状态中第i个数据块对应的残差块集合
      for (size_t r = 0; r < residuals.size(); ++r) {
        std::shared_ptr<ceres::ReprojectionErrorBase> reprojectionError =
            std::dynamic_pointer_cast<ceres::ReprojectionErrorBase>(
            residuals[r].errorInterfacePtr);//得到重投影误差（强制转换）
        //没有重投影误差
        if(!reprojectionError){   // we make sure no reprojection errors are yet included.
          marginalizationErrorPtr_->addResidualBlock(residuals[r].residualBlockId);//直接添加残差到误差集合中
        }
      }
    }
    // add all error terms of the sensor states.
    ///除了相机外参的数据块相关的残差的添加
    /// 需要边缘化帧的传感器类别
    /// 第一行：遍历所有传感器
    /// 第二行：遍历该传感器的数量
    /// 第三行：第i种第j个传感器的状态数量
    for (size_t i = 0; i < it->second.sensors.size(); ++i) {
      for (size_t j = 0; j < it->second.sensors[i].size(); ++j) {
        for (size_t k = 0; k < it->second.sensors[i][j].size(); ++k) {
          //遇见相机的外参，直接跳过
          if (i == SensorStates::Camera && k == CameraSensorStates::T_SCi) {
            continue; // we do not remove the extrinsics pose here.
          }
          //该状态不存在的直接跳过
          if (!it->second.sensors[i][j][k].exists) {
            continue;
          }
          //对应的（外参或速度与偏置）数据块是固定的
          if (mapPtr_->parameterBlockPtr(it->second.sensors[i][j][k].id)
              ->fixed()) {
            continue;  // we never eliminate fixed blocks.
          }
          std::map<uint64_t, States>::iterator checkit = it;
          checkit++;//下一个状态量
          // only get rid of it, if it's different
          //相比下一个状态量，未发生变化，直接跳过
          if(checkit->second.sensors[i][j][k].exists &&
              checkit->second.sensors[i][j][k].id == it->second.sensors[i][j][k].id){
            continue;
          }
          //该状态量的存在情况赋值为错
          it->second.sensors[i][j][k].exists = false; // remember we removed
          //需要被边缘化的传感器状态（外参或速度偏置数据块）的序号
          paremeterBlocksToBeMarginalized.push_back(it->second.sensors[i][j][k].id);
          keepParameterBlocks.push_back(false);
          ceres::Map::ResidualBlockCollection residuals = mapPtr_->residuals(
              it->second.sensors[i][j][k].id);//得到该数据块的残差集合
          for (size_t r = 0; r < residuals.size(); ++r) {
            std::shared_ptr<ceres::ReprojectionErrorBase> reprojectionError =
                std::dynamic_pointer_cast<ceres::ReprojectionErrorBase>(
                residuals[r].errorInterfacePtr);//得到重投影误差
            if(!reprojectionError){   // we make sure no reprojection errors are yet included.
              marginalizationErrorPtr_->addResidualBlock(residuals[r].residualBlockId);//添加残差到系统
            }
          }
        }
      }
    }
  }
  // marginalize ONLY pose now:
  ///仅边缘化位姿，对象是removeFrames（表示没有关键帧,可以边缘化位姿）
  bool reDoFixation = false;
  for(size_t k = 0; k<removeFrames.size(); ++k){
    std::map<uint64_t, States>::iterator it = statesMap_.find(removeFrames[k]);//确定了状态空间中可移除的状态
    ///////////////////状态it的位姿TWS
    // schedule removal - but always keep the very first frame.
    //if(it != statesMap_.begin()){
    if(true){ /////DEBUG
      //可移除状态的T_WS存在定义为false
      it->second.global[GlobalStates::T_WS].exists = false; // remember we removed
      //可移除状态对应的T_WS（数据块）序号添加到paremeterBlocksToBeMarginalized
      paremeterBlocksToBeMarginalized.push_back(it->second.global[GlobalStates::T_WS].id);
      keepParameterBlocks.push_back(false);
    }

    // add remaing error terms
    ///TWS数据块对应的残差添加
    //得到了可移除状态的TWS数据块对应的残差块集合
    ceres::Map::ResidualBlockCollection residuals = mapPtr_->residuals(
        it->second.global[GlobalStates::T_WS].id);
    //遍历残差块集合
    for (size_t r = 0; r < residuals.size(); ++r) {
      //如果是位姿偏差
      if(std::dynamic_pointer_cast<ceres::PoseError>(
           residuals[r].errorInterfacePtr)){ // avoids linearising initial pose error
                mapPtr_->removeResidualBlock(residuals[r].residualBlockId);//只是删除待边缘化位姿的位姿误差
				reDoFixation = true;
        continue;
      }
      //如果是二次投影误差
      std::shared_ptr<ceres::ReprojectionErrorBase> reprojectionError =
          std::dynamic_pointer_cast<ceres::ReprojectionErrorBase>(
          residuals[r].errorInterfacePtr);//如果是二次投影误差
      if(!reprojectionError){   // we make sure no reprojection errors are yet included.
        marginalizationErrorPtr_->addResidualBlock(residuals[r].residualBlockId);//两个都不是，重新添加偏差
      }
    }

    // add remaining error terms of the sensor states.
    ///相机外参对应的残差添加
    size_t i = SensorStates::Camera;//表示提取相机的状态项
    //j表示遍历相机的个数
    for (size_t j = 0; j < it->second.sensors[i].size(); ++j) {
      size_t k = CameraSensorStates::T_SCi;
      //状态的外参如果不存在，直接跳过
      if (!it->second.sensors[i][j][k].exists) {
        continue;
      }
      //该外参对应的数据块如果被设置为固定
      if (mapPtr_->parameterBlockPtr(it->second.sensors[i][j][k].id)
          ->fixed()) {
        continue;  // we never eliminate fixed blocks.
      }
      std::map<uint64_t, States>::iterator checkit = it;
      checkit++;//下一个状态
      // only get rid of it, if it's different
      //该状态的外参与下一个状态的外参相同，直接跳过
      if(checkit->second.sensors[i][j][k].exists &&
          checkit->second.sensors[i][j][k].id == it->second.sensors[i][j][k].id){
        continue;
      }
      //第j个相机的外参标记为false
      it->second.sensors[i][j][k].exists = false; // remember we removed
      paremeterBlocksToBeMarginalized.push_back(it->second.sensors[i][j][k].id);//将外参（对应数据块）的序号添加入可移除集合中
      keepParameterBlocks.push_back(false);
      ceres::Map::ResidualBlockCollection residuals = mapPtr_->residuals(
          it->second.sensors[i][j][k].id);//提取该外参数据块对应的偏差集合
      for (size_t r = 0; r < residuals.size(); ++r) {
        std::shared_ptr<ceres::ReprojectionErrorBase> reprojectionError =
            std::dynamic_pointer_cast<ceres::ReprojectionErrorBase>(
            residuals[r].errorInterfacePtr);//看偏差类型能否转化为二次投影误差
        if(!reprojectionError){   // we make sure no reprojection errors are yet included.
          marginalizationErrorPtr_->addResidualBlock(residuals[r].residualBlockId);//不是二次投影误差
        }
      }
    }

    // now finally we treat all the observations.
    OKVIS_ASSERT_TRUE_DBG(Exception, allLinearizedFrames.size()>0, "bug");
    uint64_t currentKfId = allLinearizedFrames.at(0);//（c-3)帧的id
    ///地标点残差的添加
    {
      //遍历地标点集合
      for(PointMap::iterator pit = landmarksMap_.begin();
          pit != landmarksMap_.end(); ){

        ceres::Map::ResidualBlockCollection residuals = mapPtr_->residuals(pit->first);//提取与地标点有关的残差集合
        //std::cout << "我想看看能不能到这里"<<residuals.size()<< std::endl;
        // first check if we can skip
        bool skipLandmark = true;
        bool hasNewObservations = false;
        bool justDelete = false;
        bool marginalize = true;
        bool errorTermAdded = false;
        std::map<uint64_t,bool> visibleInFrame;
        size_t obsCount = 0;
        //遍历与该地标点有关的偏差
        for (size_t r = 0; r < residuals.size(); ++r) {
          std::shared_ptr<ceres::ReprojectionErrorBase> reprojectionError =
              std::dynamic_pointer_cast<ceres::ReprojectionErrorBase>(
                  residuals[r].errorInterfacePtr);//转换为二次投影误差的形式
          //是二次投影误差
          if (reprojectionError) {
            uint64_t poseId = mapPtr_->parameters(residuals[r].residualBlockId).at(0).first;//提取第r个二次投影误差对应的TWS数据块（状态）的id
            //std::cout << "我想看看能不能到这里"<< mapPtr_->parameters(residuals[r].residualBlockId).size()<< std::endl;
            // since we have implemented the linearisation to account for robustification,
            // we don't kick out bad measurements here any more like
            // if(vectorContains(allLinearizedFrames,poseId)){ ...
            //   if (error.transpose() * error > 6.0) { ... removeObservation ... }
            // }
            //边缘化帧集合中存在帧可以观测到该地标点，(包括不为关键帧的c-3）
            if(vectorContains(removeFrames,poseId)){
              skipLandmark = false;
            }
            //最新三帧（包括C-3）可以观测到该地标点
            if(poseId>=currentKfId){
              marginalize = false;//边缘化赋值为false
              hasNewObservations = true;//判断有了新的观测
            }
            //三帧之前(包括C-3）存在帧可以观测到该地标点
            if(vectorContains(allLinearizedFrames, poseId)){
              visibleInFrame.insert(std::pair<uint64_t,bool>(poseId,true));//表示能观测到该特征点的所有帧（状态）的集合
              obsCount++;//可以被三帧之前的多少帧观测到
            }
          }
        }
        //如果无该特征点有关的残差项
        if(residuals.size()==0){
          mapPtr_->removeParameterBlock(pit->first);//地图中移除地标点
          removedLandmarks.push_back(pit->second);//移除地标点的集合
          pit = landmarksMap_.erase(pit);//从地标点地图中移除地标点
          continue;
        }
         ///skipLandmark为真，表示边缘化关键帧都不能观测到该地标点，比如例子中的地标点4
        //直接跳过边缘化过程，因为不需要边缘化
        if(skipLandmark) {
          pit++;
          continue;
        }

        // so, we need to consider it.
        for (size_t r = 0; r < residuals.size(); ++r) {
          std::shared_ptr<ceres::ReprojectionErrorBase> reprojectionError =
              std::dynamic_pointer_cast<ceres::ReprojectionErrorBase>(
                  residuals[r].errorInterfacePtr);//判断与该地标点关联的残差是不是二次投影误差
          if (reprojectionError) {
            ///地标点可以被第poseId个帧观测到
            uint64_t poseId = mapPtr_->parameters(residuals[r].residualBlockId).at(0).first;//提起该二次投影误差对应的TWS（帧的id）
            ///（第poseid帧属于边缘化集合且最新三帧(包括c-3）中有帧可以观测到该地标点）或
            ///（第poseid帧不为三帧之前的帧且最新的三帧(包括c-3）也不能观测到该地标点）,比如论文中地标点2,3
            if((vectorContains(removeFrames,poseId) && hasNewObservations) ||
                (!vectorContains(allLinearizedFrames,poseId) && marginalize)){
              // ok, let's ignore the observation.
              removeObservation(residuals[r].residualBlockId);//移除该观测（二次投影）
              residuals.erase(residuals.begin() + r);//将该二次投影偏差都删掉
              r--;
            } else if(marginalize && vectorContains(allLinearizedFrames,poseId)) {
              // TODO: consider only the sensible ones for marginalization
              ///第poseid帧为三帧之前的帧且最新的三帧(包括c-3）不能观测到该地标点,比如论文中地标点1
              //被allLinearizedFrames中帧观测到的次数小于2
              if(obsCount<2){ //visibleInFrame.size()
                removeObservation(residuals[r].residualBlockId);//移除该观测（第r个二次投影）
                residuals.erase(residuals.begin() + r);//将之前的二次投影偏差都删掉
                r--;
              } else {
                // add information to be considered in marginalization later.
                // 添加该观测进入残差空间中
                errorTermAdded = true;

                //添加该二次投影误差进入状态中
                marginalizationErrorPtr_->addResidualBlock(
                    residuals[r].residualBlockId, false);//该地标点的第r个二次投影误差
              }
            }
            // check anything left
            //检查该地标点的二次投影误差是否被剔除完了
            if (residuals.size() == 0) {
              justDelete = true;
              marginalize = false;
            }
          }
        }
        //地标点的观测全部被删除完了
        if(justDelete){
          mapPtr_->removeParameterBlock(pit->first);//移除地标点数据块
          removedLandmarks.push_back(pit->second);//移除地标点的集合
          pit = landmarksMap_.erase(pit);//从landmarksMap_中删去地标点
          continue;
        }
        //最新的三帧(包括C-3）不能观测到该地标点且之前有大于2帧观测到了该地标点
        //边缘化为真且可以被之前2帧观测到
        if(marginalize&&errorTermAdded){
          paremeterBlocksToBeMarginalized.push_back(pit->first);//添加该地标点到待边缘化集合
          keepParameterBlocks.push_back(false);
          removedLandmarks.push_back(pit->second);//添加到将被移除的地标点集合
          pit = landmarksMap_.erase(pit);//将其从地标点地图中删去
          continue;
        }

        pit++;
      }
    }

    // update book-keeping and go to the next frame
    //if(it != statesMap_.begin()){ // let's remember that we kept the very first pose
    if(true) { ///// DEBUG
      multiFramePtrMap_.erase(it->second.id);//从帧地图中删去这个可以移除的帧
      statesMap_.erase(it->second.id);//从状态地图中删除这个可以移除的状态
    }
  }

  // now apply the actual marginalization
  //执行边缘化
  //paremeterBlocksToBeMarginalized为需要边缘化的数据块id的集合
  if(paremeterBlocksToBeMarginalized.size()>0){
    std::vector< ::ceres::ResidualBlockId> addedPriors;
    ///paremeterBlocksToBeMarginalized为待边缘化数据块的id
    ///keepParameterBlocks为维数与paremeterBlocksToBeMarginalized相等的容器（false）
    marginalizationErrorPtr_->marginalizeOut(paremeterBlocksToBeMarginalized, keepParameterBlocks);//边缘化处理
  }

  // update error computation
  // 边缘化误差的更新
  if(paremeterBlocksToBeMarginalized.size()>0){
    marginalizationErrorPtr_->updateErrorComputation();//更新
  }

  // add the marginalization term again
  // 重置边缘化误差
  if(marginalizationErrorPtr_->num_residuals()==0){
    marginalizationErrorPtr_.reset();
  }
  if (marginalizationErrorPtr_) {
  std::vector<std::shared_ptr<okvis::ceres::ParameterBlock> > parameterBlockPtrs;
  marginalizationErrorPtr_->getParameterBlockPtrs(parameterBlockPtrs);//得到未被边缘化掉的所有数据块
  //std::cout<<parameterBlockPtrs.size()<<std::endl;
  /// 我认为这里很重要,因为根据递推关系,地标点残差的添加应该在这里
  marginalizationResidualId_ = mapPtr_->addResidualBlock(
      marginalizationErrorPtr_, NULL, parameterBlockPtrs);//将未被边缘化的偏差加入优化地图中
  OKVIS_ASSERT_TRUE_DBG(Exception, marginalizationResidualId_,
                     "could not add marginalization error");
  if (!marginalizationResidualId_)
    return false;
  }
    //如果删除了位姿误差
	if(reDoFixation){
	  // finally fix the first pose properly
		//mapPtr_->resetParameterization(statesMap_.begin()->first, ceres::Map::Pose3d);
		okvis::kinematics::Transformation T_WS_0;
		get_T_WS(statesMap_.begin()->first, T_WS_0);
	  Eigen::Matrix<double,6,6> information = Eigen::Matrix<double,6,6>::Zero();
	  information(5,5) = 1.0e14; information(0,0) = 1.0e14; information(1,1) = 1.0e14; information(2,2) = 1.0e14;
      std::shared_ptr<ceres::PoseError > poseError(new ceres::PoseError(T_WS_0, information));//初始位姿误差
      mapPtr_->addResidualBlock(poseError,NULL,mapPtr_->parameterBlockPtr(statesMap_.begin()->first));//重新补一个初始位姿误差
	}

  return true;
}

// Prints state information to buffer.
void Estimator::printStates(uint64_t poseId, std::ostream & buffer) const {
  buffer << "GLOBAL: ";
  for(size_t i = 0; i<statesMap_.at(poseId).global.size(); ++i){
    if(statesMap_.at(poseId).global.at(i).exists) {
      uint64_t id = statesMap_.at(poseId).global.at(i).id;
      if(mapPtr_->parameterBlockPtr(id)->fixed())
        buffer << "(";
      buffer << "id="<<id<<":";
      buffer << mapPtr_->parameterBlockPtr(id)->typeInfo();
      if(mapPtr_->parameterBlockPtr(id)->fixed())
        buffer << ")";
      buffer <<", ";
    }
  }
  buffer << "SENSOR: ";
  for(size_t i = 0; i<statesMap_.at(poseId).sensors.size(); ++i){
    for(size_t j = 0; j<statesMap_.at(poseId).sensors.at(i).size(); ++j){
      for(size_t k = 0; k<statesMap_.at(poseId).sensors.at(i).at(j).size(); ++k){
        if(statesMap_.at(poseId).sensors.at(i).at(j).at(k).exists) {
          uint64_t id = statesMap_.at(poseId).sensors.at(i).at(j).at(k).id;
          if(mapPtr_->parameterBlockPtr(id)->fixed())
            buffer << "(";
          buffer << "id="<<id<<":";
          buffer << mapPtr_->parameterBlockPtr(id)->typeInfo();
          if(mapPtr_->parameterBlockPtr(id)->fixed())
            buffer << ")";
          buffer <<", ";
        }
      }
    }
  }
  buffer << std::endl;
}

// Initialise pose from IMU measurements. For convenience as static.
//从IMU观测中恢复初始位姿
bool Estimator::initPoseFromImu(
    const okvis::ImuMeasurementDeque & imuMeasurements,
    okvis::kinematics::Transformation & T_WS)
{
  // set translation to zero, unit rotation
  //将T_WS的旋转矩阵置为单位阵, 平移响亮赋值为0向量
  T_WS.setIdentity();

  if (imuMeasurements.size() == 0)//如果没有IMU数据
    return false;

  // acceleration vector
  Eigen::Vector3d acc_B = Eigen::Vector3d::Zero();
  for (okvis::ImuMeasurementDeque::const_iterator it = imuMeasurements.begin();
      it < imuMeasurements.end(); ++it) {
    acc_B += it->measurement.accelerometers;//提取IMU观测数据的加速度值,并进行累加
  }
  acc_B /= double(imuMeasurements.size());//求平均
  Eigen::Vector3d e_acc = acc_B.normalized();//求acc_B的单位向量

  // align with ez_W:
  Eigen::Vector3d ez_W(0.0, 0.0, 1.0);
  Eigen::Matrix<double, 6, 1> poseIncrement;
  poseIncrement.head<3>() = Eigen::Vector3d::Zero();//poseIncrement向量的前三个元素
  poseIncrement.tail<3>() = ez_W.cross(e_acc).normalized();//poseIncrement向量的后三个元素,ez_W与e_acc的叉乘
  double angle = std::acos(ez_W.transpose() * e_acc);//求解ez_W与e_acc的夹角
  poseIncrement.tail<3>() *= angle;//得到欧拉角（旋转轴和旋转角度）
  T_WS.oplus(-poseIncrement);//将T_WS与-poseIncrement相加

  return true;
}

// Start ceres optimization.
// 开始ceres的优化
///迭代次数
/// 线程数量
#ifdef USE_OPENMP
void Estimator::optimize(size_t numIter, size_t numThreads,
                                 bool verbose)
#else
void Estimator::optimize(size_t numIter, size_t /*numThreads*/,
                                 bool verbose) // avoid warning since numThreads unused
#warning openmp not detected, your system may be slower than expected
#endif

{
  // assemble options
  mapPtr_->options.linear_solver_type = ::ceres::SPARSE_SCHUR;//设置求解器的类型
  //mapPtr_->options.initial_trust_region_radius = 1.0e4;
  //mapPtr_->options.initial_trust_region_radius = 2.0e6;
  //mapPtr_->options.preconditioner_type = ::ceres::IDENTITY;
  mapPtr_->options.trust_region_strategy_type = ::ceres::DOGLEG;//设置求解器信赖域策略类型
  //mapPtr_->options.trust_region_strategy_type = ::ceres::LEVENBERG_MARQUARDT;
  //mapPtr_->options.use_nonmonotonic_steps = true;
  //mapPtr_->options.max_consecutive_nonmonotonic_steps = 10;
  //mapPtr_->options.function_tolerance = 1e-12;
  //mapPtr_->options.gradient_tolerance = 1e-12;
  //mapPtr_->options.jacobi_scaling = false;
#ifdef USE_OPENMP
    mapPtr_->options.num_threads = numThreads;//求解器线程的数量
#endif
  mapPtr_->options.max_num_iterations = numIter;//求解器最大的迭代数量

  if (verbose) {
    mapPtr_->options.minimizer_progress_to_stdout = true;
  } else {
    mapPtr_->options.minimizer_progress_to_stdout = false;
  }

  // call solver
  // 回调求解函数
  mapPtr_->solve();

  // update landmarks
  ///利用优化求解器中的数据块来更新地标点的信息,和判断地标点的质量
  {
    for(auto it = landmarksMap_.begin(); it!=landmarksMap_.end(); ++it){
      Eigen::MatrixXd H(3,3);
      mapPtr_->getLhs(it->first,H);//计算海瑟矩阵
      Eigen::SelfAdjointEigenSolver< Eigen::Matrix3d > saes(H);//对称矩阵求解器
      Eigen::Vector3d eigenvalues = saes.eigenvalues();//计算特征值
      const double smallest = (eigenvalues[0]);//最小特征值
      const double largest = (eigenvalues[2]);//最大特征值
      if(smallest<1.0e-12){
        // this means, it has a non-observable depth
        it->second.quality = 0.0;
      } else {
        // OK, well constrained
        it->second.quality = sqrt(smallest)/sqrt(largest);//计算地标点的质量
      }

      // update coordinates
      it->second.point = std::static_pointer_cast<okvis::ceres::HomogeneousPointParameterBlock>(
          mapPtr_->parameterBlockPtr(it->first))->estimate();//重新赋值地标点的坐标
    }
  }

  // summary output
  if (verbose) {
    LOG(INFO) << mapPtr_->summary.FullReport();
  }
}

// Set a time limit for the optimization process.
bool Estimator::setOptimizationTimeLimit(double timeLimit, int minIterations) {
  if(ceresCallback_ != nullptr) {
    if(timeLimit < 0.0) {
      // no time limit => set minimum iterations to maximum iterations
      ceresCallback_->setMinimumIterations(mapPtr_->options.max_num_iterations);
      return true;
    }
    ceresCallback_->setTimeLimit(timeLimit);
    ceresCallback_->setMinimumIterations(minIterations);
    return true;
  }
  else if(timeLimit >= 0.0) {
    ceresCallback_ = std::unique_ptr<okvis::ceres::CeresIterationCallback>(
          new okvis::ceres::CeresIterationCallback(timeLimit,minIterations));
    mapPtr_->options.callbacks.push_back(ceresCallback_.get());
    return true;
  }
  // no callback yet registered with ceres.
  // but given time limit is lower than 0, so no callback needed
  return true;
}

// getters
// Get a specific landmark.
bool Estimator::getLandmark(uint64_t landmarkId,
                                    MapPoint& mapPoint) const
{
  std::lock_guard<std::mutex> l(statesMutex_);
  if (landmarksMap_.find(landmarkId) == landmarksMap_.end()) {
    OKVIS_THROW_DBG(Exception,"landmark with id = "<<landmarkId<<" does not exist.")
    return false;
  }
  mapPoint = landmarksMap_.at(landmarkId);
  return true;
}

// Checks whether the landmark is initialized.
bool Estimator::isLandmarkInitialized(uint64_t landmarkId) const {
  OKVIS_ASSERT_TRUE_DBG(Exception, isLandmarkAdded(landmarkId),
                     "landmark not added");
  return std::static_pointer_cast<okvis::ceres::HomogeneousPointParameterBlock>(
      mapPtr_->parameterBlockPtr(landmarkId))->initialized();
}

// Get a copy of all the landmarks as a PointMap.
size_t Estimator::getLandmarks(PointMap & landmarks) const
{
  std::lock_guard<std::mutex> l(statesMutex_);
  landmarks = landmarksMap_;
  return landmarksMap_.size();
}

// Get a copy of all the landmark in a MapPointVector. This is for legacy support.
// Use getLandmarks(okvis::PointMap&) if possible.
size_t Estimator::getLandmarks(MapPointVector & landmarks) const
{
  std::lock_guard<std::mutex> l(statesMutex_);
  landmarks.clear();
  landmarks.reserve(landmarksMap_.size());
  for(PointMap::const_iterator it=landmarksMap_.begin(); it!=landmarksMap_.end(); ++it){
    landmarks.push_back(it->second);
  }
  return landmarksMap_.size();
}

// Get pose for a given pose ID.
bool Estimator::get_T_WS(uint64_t poseId,
                                 okvis::kinematics::Transformation & T_WS) const
{
  if (!getGlobalStateEstimateAs<ceres::PoseParameterBlock>(poseId,
                                                           GlobalStates::T_WS,
                                                           T_WS)) {
    return false;
  }

  return true;
}

// Feel free to implement caching for them...
// Get speeds and IMU biases for a given pose ID.
bool Estimator::getSpeedAndBias(uint64_t poseId, uint64_t imuIdx,
                                okvis::SpeedAndBias & speedAndBias) const
{
  if (!getSensorStateEstimateAs<ceres::SpeedAndBiasParameterBlock>(
      poseId, imuIdx, SensorStates::Imu, ImuSensorStates::SpeedAndBias,
      speedAndBias)) {
    return false;
  }
  return true;
}

// Get camera states for a given pose ID.
bool Estimator::getCameraSensorStates(
    uint64_t poseId, size_t cameraIdx,
    okvis::kinematics::Transformation & T_SCi) const
{
  return getSensorStateEstimateAs<ceres::PoseParameterBlock>(
      poseId, cameraIdx, SensorStates::Camera, CameraSensorStates::T_SCi, T_SCi);
}

// Get the ID of the current keyframe.
uint64_t Estimator::currentKeyframeId() const {
  for (std::map<uint64_t, States>::const_reverse_iterator rit = statesMap_.rbegin();
      rit != statesMap_.rend(); ++rit) {
    if (rit->second.isKeyframe) {
      return rit->first;
    }
  }
  OKVIS_THROW_DBG(Exception, "no keyframes existing...");
  return 0;
}

// Get the ID of an older frame.
// 获得比当前帧早age的帧的id
uint64_t Estimator::frameIdByAge(size_t age) const {
  std::map<uint64_t, States>::const_reverse_iterator rit = statesMap_.rbegin();
  for(size_t i=0; i<age; ++i){
    ++rit;
    OKVIS_ASSERT_TRUE_DBG(Exception, rit != statesMap_.rend(),
                       "requested age " << age << " out of range.");
  }
  return rit->first;
}

// Get the ID of the newest frame added to the state.
uint64_t Estimator::currentFrameId() const {
  OKVIS_ASSERT_TRUE_DBG(Exception, statesMap_.size()>0, "no frames added yet.")
  return statesMap_.rbegin()->first;
}

// Checks if a particular frame is still in the IMU window
// 检测帧是否仍在IMU的窗口中
bool Estimator::isInImuWindow(uint64_t frameId) const {
  if(statesMap_.at(frameId).sensors.at(SensorStates::Imu).size()==0){
    return false; // no IMU added
  }
  return statesMap_.at(frameId).sensors.at(SensorStates::Imu).at(0).at(ImuSensorStates::SpeedAndBias).exists;
}

// Set pose for a given pose ID.
bool Estimator::set_T_WS(uint64_t poseId,
                                 const okvis::kinematics::Transformation & T_WS)
{
  if (!setGlobalStateEstimateAs<ceres::PoseParameterBlock>(poseId,
                                                           GlobalStates::T_WS,
                                                           T_WS)) {
    return false;
  }

  return true;
}

// Set the speeds and IMU biases for a given pose ID.
bool Estimator::setSpeedAndBias(uint64_t poseId, size_t imuIdx, const okvis::SpeedAndBias & speedAndBias)
{
  return setSensorStateEstimateAs<ceres::SpeedAndBiasParameterBlock>(
      poseId, imuIdx, SensorStates::Imu, ImuSensorStates::SpeedAndBias, speedAndBias);
}

// Set the transformation from sensor to camera frame for a given pose ID.
bool Estimator::setCameraSensorStates(
    uint64_t poseId, size_t cameraIdx,
    const okvis::kinematics::Transformation & T_SCi)
{
  return setSensorStateEstimateAs<ceres::PoseParameterBlock>(
      poseId, cameraIdx, SensorStates::Camera, CameraSensorStates::T_SCi, T_SCi);
}

// Set the homogeneous coordinates for a landmark.
bool Estimator::setLandmark(
    uint64_t landmarkId, const Eigen::Vector4d & landmark)
{
  std::shared_ptr<ceres::ParameterBlock> parameterBlockPtr = mapPtr_
      ->parameterBlockPtr(landmarkId);
#ifndef NDEBUG
  std::shared_ptr<ceres::HomogeneousPointParameterBlock> derivedParameterBlockPtr =
  std::dynamic_pointer_cast<ceres::HomogeneousPointParameterBlock>(parameterBlockPtr);
  if(!derivedParameterBlockPtr) {
    OKVIS_THROW_DBG(Exception,"wrong pointer type requested.")
    return false;
  }
  derivedParameterBlockPtr->setEstimate(landmark);;
#else
  std::static_pointer_cast<ceres::HomogeneousPointParameterBlock>(
      parameterBlockPtr)->setEstimate(landmark);
#endif

  // also update in map
  landmarksMap_.at(landmarkId).point = landmark;
  return true;
}

// Set the landmark initialization state.
void Estimator::setLandmarkInitialized(uint64_t landmarkId,
                                               bool initialized) {
  OKVIS_ASSERT_TRUE_DBG(Exception, isLandmarkAdded(landmarkId),
                     "landmark not added");
  std::static_pointer_cast<okvis::ceres::HomogeneousPointParameterBlock>(
      mapPtr_->parameterBlockPtr(landmarkId))->setInitialized(initialized);
}

// private stuff
// getters
bool Estimator::getGlobalStateParameterBlockPtr(
    uint64_t poseId, int stateType,
    std::shared_ptr<ceres::ParameterBlock>& stateParameterBlockPtr) const
{
  // check existence in states set
  if (statesMap_.find(poseId) == statesMap_.end()) {
    OKVIS_THROW(Exception,"pose with id = "<<poseId<<" does not exist.")
    return false;
  }

  // obtain the parameter block ID
  uint64_t id = statesMap_.at(poseId).global.at(stateType).id;
  if (!mapPtr_->parameterBlockExists(id)) {
    OKVIS_THROW(Exception,"pose with id = "<<id<<" does not exist.")
    return false;
  }

  stateParameterBlockPtr = mapPtr_->parameterBlockPtr(id);
  return true;
}
template<class PARAMETER_BLOCK_T>
bool Estimator::getGlobalStateParameterBlockAs(
    uint64_t poseId, int stateType,
    PARAMETER_BLOCK_T & stateParameterBlock) const
{
  // convert base class pointer with various levels of checking
  std::shared_ptr<ceres::ParameterBlock> parameterBlockPtr;
  if (!getGlobalStateParameterBlockPtr(poseId, stateType, parameterBlockPtr)) {
    return false;
  }
#ifndef NDEBUG
  std::shared_ptr<PARAMETER_BLOCK_T> derivedParameterBlockPtr =
  std::dynamic_pointer_cast<PARAMETER_BLOCK_T>(parameterBlockPtr);
  if(!derivedParameterBlockPtr) {
    LOG(INFO) << "--"<<parameterBlockPtr->typeInfo();
    std::shared_ptr<PARAMETER_BLOCK_T> info(new PARAMETER_BLOCK_T);
    OKVIS_THROW_DBG(Exception,"wrong pointer type requested: requested "
                 <<info->typeInfo()<<" but is of type"
                 <<parameterBlockPtr->typeInfo())
    return false;
  }
  stateParameterBlock = *derivedParameterBlockPtr;
#else
  stateParameterBlock = *std::static_pointer_cast<PARAMETER_BLOCK_T>(
      parameterBlockPtr);
#endif
  return true;
}
template<class PARAMETER_BLOCK_T>
bool Estimator::getGlobalStateEstimateAs(
    uint64_t poseId, int stateType,
    typename PARAMETER_BLOCK_T::estimate_t & state) const
{
  PARAMETER_BLOCK_T stateParameterBlock;
  if (!getGlobalStateParameterBlockAs(poseId, stateType, stateParameterBlock)) {
    return false;
  }
  state = stateParameterBlock.estimate();
  return true;
}

bool Estimator::getSensorStateParameterBlockPtr(
    uint64_t poseId, int sensorIdx, int sensorType, int stateType,
    std::shared_ptr<ceres::ParameterBlock>& stateParameterBlockPtr) const
{
  // check existence in states set
  if (statesMap_.find(poseId) == statesMap_.end()) {
    OKVIS_THROW_DBG(Exception,"pose with id = "<<poseId<<" does not exist.")
    return false;
  }

  // obtain the parameter block ID
  uint64_t id = statesMap_.at(poseId).sensors.at(sensorType).at(sensorIdx).at(
      stateType).id;
  if (!mapPtr_->parameterBlockExists(id)) {
    OKVIS_THROW_DBG(Exception,"pose with id = "<<poseId<<" does not exist.")
    return false;
  }
  stateParameterBlockPtr = mapPtr_->parameterBlockPtr(id);
  return true;
}
template<class PARAMETER_BLOCK_T>
bool Estimator::getSensorStateParameterBlockAs(
    uint64_t poseId, int sensorIdx, int sensorType, int stateType,
    PARAMETER_BLOCK_T & stateParameterBlock) const
{
  // convert base class pointer with various levels of checking
  std::shared_ptr<ceres::ParameterBlock> parameterBlockPtr;
  if (!getSensorStateParameterBlockPtr(poseId, sensorIdx, sensorType, stateType,
                                       parameterBlockPtr)) {
    return false;
  }
#ifndef NDEBUG
  std::shared_ptr<PARAMETER_BLOCK_T> derivedParameterBlockPtr =
  std::dynamic_pointer_cast<PARAMETER_BLOCK_T>(parameterBlockPtr);
  if(!derivedParameterBlockPtr) {
    std::shared_ptr<PARAMETER_BLOCK_T> info(new PARAMETER_BLOCK_T);
    OKVIS_THROW_DBG(Exception,"wrong pointer type requested: requested "
                     <<info->typeInfo()<<" but is of type"
                     <<parameterBlockPtr->typeInfo())
    return false;
  }
  stateParameterBlock = *derivedParameterBlockPtr;
#else
  stateParameterBlock = *std::static_pointer_cast<PARAMETER_BLOCK_T>(
      parameterBlockPtr);
#endif
  return true;
}
template<class PARAMETER_BLOCK_T>
bool Estimator::getSensorStateEstimateAs(
    uint64_t poseId, int sensorIdx, int sensorType, int stateType,
    typename PARAMETER_BLOCK_T::estimate_t & state) const
{
  PARAMETER_BLOCK_T stateParameterBlock;
  if (!getSensorStateParameterBlockAs(poseId, sensorIdx, sensorType, stateType,
                                      stateParameterBlock)) {
    return false;
  }
  state = stateParameterBlock.estimate();
  return true;
}

template<class PARAMETER_BLOCK_T>
bool Estimator::setGlobalStateEstimateAs(
    uint64_t poseId, int stateType,
    const typename PARAMETER_BLOCK_T::estimate_t & state)
{
  // check existence in states set
  if (statesMap_.find(poseId) == statesMap_.end()) {
    OKVIS_THROW_DBG(Exception,"pose with id = "<<poseId<<" does not exist.")
    return false;
  }

  // obtain the parameter block ID
  uint64_t id = statesMap_.at(poseId).global.at(stateType).id;
  if (!mapPtr_->parameterBlockExists(id)) {
    OKVIS_THROW_DBG(Exception,"pose with id = "<<poseId<<" does not exist.")
    return false;
  }

  std::shared_ptr<ceres::ParameterBlock> parameterBlockPtr = mapPtr_
      ->parameterBlockPtr(id);
#ifndef NDEBUG
  std::shared_ptr<PARAMETER_BLOCK_T> derivedParameterBlockPtr =
  std::dynamic_pointer_cast<PARAMETER_BLOCK_T>(parameterBlockPtr);
  if(!derivedParameterBlockPtr) {
    OKVIS_THROW_DBG(Exception,"wrong pointer type requested.")
    return false;
  }
  derivedParameterBlockPtr->setEstimate(state);
#else
  std::static_pointer_cast<PARAMETER_BLOCK_T>(parameterBlockPtr)->setEstimate(
      state);
#endif
  return true;
}

template<class PARAMETER_BLOCK_T>
bool Estimator::setSensorStateEstimateAs(
    uint64_t poseId, int sensorIdx, int sensorType, int stateType,
    const typename PARAMETER_BLOCK_T::estimate_t & state)
{
  // check existence in states set
  if (statesMap_.find(poseId) == statesMap_.end()) {
    OKVIS_THROW_DBG(Exception,"pose with id = "<<poseId<<" does not exist.")
    return false;
  }

  // obtain the parameter block ID
  uint64_t id = statesMap_.at(poseId).sensors.at(sensorType).at(sensorIdx).at(
      stateType).id;
  if (!mapPtr_->parameterBlockExists(id)) {
    OKVIS_THROW_DBG(Exception,"pose with id = "<<poseId<<" does not exist.")
    return false;
  }

  std::shared_ptr<ceres::ParameterBlock> parameterBlockPtr = mapPtr_
      ->parameterBlockPtr(id);
#ifndef NDEBUG
  std::shared_ptr<PARAMETER_BLOCK_T> derivedParameterBlockPtr =
  std::dynamic_pointer_cast<PARAMETER_BLOCK_T>(parameterBlockPtr);
  if(!derivedParameterBlockPtr) {
    OKVIS_THROW_DBG(Exception,"wrong pointer type requested.")
    return false;
  }
  derivedParameterBlockPtr->setEstimate(state);
#else
  std::static_pointer_cast<PARAMETER_BLOCK_T>(parameterBlockPtr)->setEstimate(
      state);
#endif
  return true;
}

}  // namespace okvis


