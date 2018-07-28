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
 *  Created on: Aug 21, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file ThreadedKFVio.cpp
 * @brief Source file for the ThreadedKFVio class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <map>

#include <glog/logging.h>

#include <okvis/ThreadedKFVio.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/ceres/ImuError.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

static const int max_camera_input_queue_size = 10;
static const okvis::Duration temporal_imu_data_overlap(0.02);  // overlap of imu data before and after two consecutive frames [seconds]

#ifdef USE_MOCK
// Constructor for gmock.
ThreadedKFVio::ThreadedKFVio(okvis::VioParameters& parameters, okvis::MockVioBackendInterface& estimator,
    okvis::MockVioFrontendInterface& frontend)
    : speedAndBiases_propagated_(okvis::SpeedAndBias::Zero()),
      imu_params_(parameters.imu),
      repropagationNeeded_(false),
      frameSynchronizer_(okvis::FrameSynchronizer(parameters)),
      lastAddedImageTimestamp_(okvis::Time(0, 0)),
      optimizationDone_(true),
      estimator_(estimator),
      frontend_(frontend),
      parameters_(parameters),
      maxImuInputQueueSize_(60) {
  init();
}
#else
// Constructor.
//构造函数（主函数进入SLAM算法最重要的接口）
ThreadedKFVio::ThreadedKFVio(okvis::VioParameters& parameters)
    : speedAndBiases_propagated_(okvis::SpeedAndBias::Zero()),//9维的列向量
      imu_params_(parameters.imu),//IMU参数的赋值
      repropagationNeeded_(false),//是否需要反向传播
      frameSynchronizer_(okvis::FrameSynchronizer(parameters)),//帧同步器
      lastAddedImageTimestamp_(okvis::Time(0, 0)),//上一次添加图像的时间戳
      optimizationDone_(true),//是否进行优化
      estimator_(),//初始化估计求解器
      frontend_(parameters.nCameraSystem.numCameras()),//前端求解器的初始化
      parameters_(parameters),//系统参数的赋值
      //在IMU数据被遗弃前，队列中相机帧数的最大值
      maxImuInputQueueSize_(
          2 * max_camera_input_queue_size * parameters.imu.rate
              / parameters.sensors_information.cameraRate) {
   //Set the blocking variable that indicates whether the addMeasurement() functions
   // should return immediately (blocking=false), or only when the processing is complete.
   ///设置blocking变量，判断addMeasurement函数是立即返回（false），还是过程完成后才返回
  setBlocking(false);
  init();//初始化
}
#endif

// Initialises settings and calls startThreads().
//初始化设置，开始线程
void ThreadedKFVio::init() {
  assert(parameters_.nCameraSystem.numCameras() > 0);//其作用是如果它的条件返回错误，则终止程序执行
  numCameras_ = parameters_.nCameraSystem.numCameras();//相机的数目
  numCameraPairs_ = 1;

  frontend_.setBriskDetectionOctaves(parameters_.optimization.detectionOctaves);//特征点探测半径
  frontend_.setBriskDetectionThreshold(parameters_.optimization.detectionThreshold);//特征点探测的图像层数
  frontend_.setBriskDetectionMaximumKeypoints(parameters_.optimization.maxNoKeypoints);//每张图像最大的特征点数目
  //lastOptimizedStateTimestamp_上一次优化时（状态）的时间戳
  lastOptimizedStateTimestamp_ = okvis::Time(0.0) + temporal_imu_data_overlap;  // s.t. last_timestamp_ - overlap >= 0 (since okvis::time(-0.02) returns big number)
  //lastAddedStateTimestamp_上一次添加状态时的时间戳
  lastAddedStateTimestamp_ = okvis::Time(0.0) + temporal_imu_data_overlap;  // s.t. last_timestamp_ - overlap >= 0 (since okvis::time(-0.02) returns big number)
  //观测器中添加IMU的参数信息
  estimator_.addImu(parameters_.imu);
  for (size_t i = 0; i < numCameras_; ++i) {
    // parameters_.camera_extrinsics is never set (default 0's)...
    // do they ever change?
    estimator_.addCamera(parameters_.camera_extrinsics);//添加相机的外参信息
    ///Camera measurement input queues（cameraMeasurementsReceived_为相机观测的队列,每一个元素都是ThreadSafeQueue）
    /// emplace_back为容器中的元素添加函数（相当于push_back加类的构造函数）
    /// typedef Measurement<CameraData> CameraMeasurement;(其中Measurement为一个结构体,CameraData为一个模板,包含图像的信息)
    /// CameraMeasurement中包括了图像信息和特征点的信息
    cameraMeasurementsReceived_.emplace_back(
          std::shared_ptr<threadsafe::ThreadSafeQueue<std::shared_ptr<okvis::CameraMeasurement> > >
          (new threadsafe::ThreadSafeQueue<std::shared_ptr<okvis::CameraMeasurement> >()));
  }
  
  // set up windows so things don't crash on Mac OS
  // 设置窗口的名字
  if(parameters_.visualization.displayImages){
    for (size_t im = 0; im < parameters_.nCameraSystem.numCameras(); im++) {
      std::stringstream windowname;
      windowname << "OKVIS camera " << im;
  	  cv::namedWindow(windowname.str());
    }
  }
  
  startThreads();//开始线程
}

// Start all threads.
//开始所有线程
void ThreadedKFVio::startThreads() {

  // consumer threads
  // 处理帧的线程容器,每一个线程调用frameConsumerLoop,不过输入参数不一样
  for (size_t i = 0; i < numCameras_; ++i) {
    frameConsumerThreads_.emplace_back(&ThreadedKFVio::frameConsumerLoop, this, i);
  }
  // 处理特征点的线程容器,每一个线程都调用matchingLoop
  for (size_t i = 0; i < numCameraPairs_; ++i) {
    keypointConsumerThreads_.emplace_back(&ThreadedKFVio::matchingLoop, this);
  }
  // 处理IMU的线程
  imuConsumerThread_ = std::thread(&ThreadedKFVio::imuConsumerLoop, this);
  // 求取位置的线程
  positionConsumerThread_ = std::thread(&ThreadedKFVio::positionConsumerLoop,
                                        this);
  //GPS处理的线程
  gpsConsumerThread_ = std::thread(&ThreadedKFVio::gpsConsumerLoop, this);
  //磁力处理的线程
  magnetometerConsumerThread_ = std::thread(
      &ThreadedKFVio::magnetometerConsumerLoop, this);
  //不同处理的线程
  differentialConsumerThread_ = std::thread(
      &ThreadedKFVio::differentialConsumerLoop, this);

  // algorithm threads
  //可视化线程
  visualizationThread_ = std::thread(&ThreadedKFVio::visualizationLoop, this);
  //优化线程
  optimizationThread_ = std::thread(&ThreadedKFVio::optimizationLoop, this);
  //消息发布线程
  publisherThread_ = std::thread(&ThreadedKFVio::publisherLoop, this);
}

// Destructor. This calls Shutdown() for all threadsafe queues and joins all threads.
ThreadedKFVio::~ThreadedKFVio() {
  for (size_t i = 0; i < numCameras_; ++i) {
    cameraMeasurementsReceived_.at(i)->Shutdown();
  }
  keypointMeasurements_.Shutdown();
  matchedFrames_.Shutdown();
  imuMeasurementsReceived_.Shutdown();
  optimizationResults_.Shutdown();
  visualizationData_.Shutdown();
  imuFrameSynchronizer_.shutdown();
  positionMeasurementsReceived_.Shutdown();

  // consumer threads
  for (size_t i = 0; i < numCameras_; ++i) {
    frameConsumerThreads_.at(i).join();
  }
  for (size_t i = 0; i < numCameraPairs_; ++i) {
    keypointConsumerThreads_.at(i).join();
  }
  imuConsumerThread_.join();
  positionConsumerThread_.join();
  gpsConsumerThread_.join();
  magnetometerConsumerThread_.join();
  differentialConsumerThread_.join();
  visualizationThread_.join();
  optimizationThread_.join();
  publisherThread_.join();

  /*okvis::kinematics::Transformation endPosition;
  estimator_.get_T_WS(estimator_.currentFrameId(), endPosition);
  std::stringstream s;
  s << endPosition.r();
  LOG(INFO) << "Sensor end position:\n" << s.str();
  LOG(INFO) << "Distance to origin: " << endPosition.r().norm();*/
#ifndef DEACTIVATE_TIMERS
  LOG(INFO) << okvis::timing::Timing::print();
#endif
}

// Add a new image.
//状态求解器中添加图片
bool ThreadedKFVio::addImage(const okvis::Time & stamp, size_t cameraIndex,
                             const cv::Mat & image,
                             const std::vector<cv::KeyPoint> * keypoints,
                             bool* /*asKeyframe*/) {
  assert(cameraIndex<numCameras_);//相机序号是0,1,小于2
  //这个图像时间比上一刻添加的图像还要早，且相差过多（大于0.005s），即报错
  if (lastAddedImageTimestamp_ > stamp
      && fabs((lastAddedImageTimestamp_ - stamp).toSec())
          > parameters_.sensors_information.frameTimestampTolerance) {
    LOG(ERROR)
        << "Received image from the past. Dropping the image.";
    return false;
  }
  lastAddedImageTimestamp_ = stamp;//更新上一次添加图片的时间戳
  //相机观测量的指针
  std::shared_ptr<okvis::CameraMeasurement> frame = std::make_shared<
      okvis::CameraMeasurement>();
  frame->measurement.image = image;//赋值图像信息
  frame->timeStamp = stamp;//赋值时间戳
  frame->sensorId = cameraIndex;//赋值传感器ID（即第几个相机）

  if (keypoints != nullptr) {
    frame->measurement.deliversKeypoints = true;//特征点可交付（存在特征点）
    frame->measurement.keypoints = *keypoints;//赋值特征点
  } else {
    frame->measurement.deliversKeypoints = false;//不存在特征点
  }
  ///Push to the queue if the size is less than max_queue_size, else block.
  /// PushBlockingIfFull函数的意思是如果队列的长度大于1,则阻止，队列长度小于1,则添加
  /// 在cameraMeasurementsReceived_中有一个CameraMeasurement类型的队列queue
  if (blocking_) {
    cameraMeasurementsReceived_[cameraIndex]->PushBlockingIfFull(frame, 1);
    return true;
  } else {
    ///Push to the queue. If full, drop the oldest entry.
    /// PushNonBlockingDroppingIfFull函数的意思是如果队列长度大于max_camera_input_queue_size（10），则从队列顶端删除一个元素再进行添加操作
    cameraMeasurementsReceived_[cameraIndex]->PushNonBlockingDroppingIfFull(
        frame, max_camera_input_queue_size);
    return cameraMeasurementsReceived_[cameraIndex]->Size() == 1;//返回队列长度是否等于1
  }
}

// Add an abstracted image observation.
// 向状态求解器中添加特征点
bool ThreadedKFVio::addKeypoints(
    const okvis::Time & /*stamp*/, size_t /*cameraIndex*/,
    const std::vector<cv::KeyPoint> & /*keypoints*/,
    const std::vector<uint64_t> & /*landmarkIds*/,
    const cv::Mat & /*descriptors*/,
    bool* /*asKeyframe*/) {
  OKVIS_THROW(
      Exception,
      "ThreadedKFVio::addKeypoints() not implemented anymore since changes to _keypointMeasurements queue.");
  return false;
}

// Add an IMU measurement.
// 向状态求解器中添加IMU的观测信息
bool ThreadedKFVio::addImuMeasurement(const okvis::Time & stamp,
                                      const Eigen::Vector3d & alpha,
                                      const Eigen::Vector3d & omega) {

  okvis::ImuMeasurement imu_measurement;//IMU的观测状态，包括加速度，角速度和时间戳
  imu_measurement.measurement.accelerometers = alpha;// 赋值加速度
  imu_measurement.measurement.gyroscopes = omega;//赋值角速度
  imu_measurement.timeStamp = stamp;//赋值时间戳
  //imuMeasurementsReceived_的观测序列
  if (blocking_) {
    imuMeasurementsReceived_.PushBlockingIfFull(imu_measurement, 1);//添加IMU观测数据，必须满足原序列中元素的个数小于1
    return true;
  } else {
    imuMeasurementsReceived_.PushNonBlockingDroppingIfFull(
        imu_measurement, maxImuInputQueueSize_);//添加IMU观测数据，原序列中元素个数大于maxImuInputQueueSize_时，必须先从序列顶端删除一个元素在添加新元素
    return imuMeasurementsReceived_.Size() == 1;
  }
}

// Add a position measurement.
// 添加位置信息
void ThreadedKFVio::addPositionMeasurement(const okvis::Time & stamp,
                                           const Eigen::Vector3d & position,
                                           const Eigen::Vector3d & positionOffset,
                                           const Eigen::Matrix3d & positionCovariance) {
  okvis::PositionMeasurement position_measurement;//位姿状态量
  position_measurement.measurement.position = position;//赋值位姿
  position_measurement.measurement.positionOffset = positionOffset;//赋值位姿偏差
  position_measurement.measurement.positionCovariance = positionCovariance;//赋值协方差
  position_measurement.timeStamp = stamp;//赋值时间戳

  if (blocking_) {
    positionMeasurementsReceived_.PushBlockingIfFull(position_measurement, 1);//添加位姿数据，必须满足原序列中元素的个数小于1
    return;
  } else {
    positionMeasurementsReceived_.PushNonBlockingDroppingIfFull(
        position_measurement, maxPositionInputQueueSize_);//添加位姿数据，原序列中元素个数大于maxPositionInputQueueSize_时，必须先从序列顶端删除一个元素在添加新元素
    return;
  }
}

// Add a GPS measurement.
void ThreadedKFVio::addGpsMeasurement(const okvis::Time &, double, double,
                                      double, const Eigen::Vector3d &,
                                      const Eigen::Matrix3d &) {
  OKVIS_THROW(Exception, "GPS measurements not supported")
}

// Add a magnetometer measurement.
void ThreadedKFVio::addMagnetometerMeasurement(const okvis::Time &,
                                               const Eigen::Vector3d &, double) {
  OKVIS_THROW(Exception, "Magnetometer measurements not supported")
}

// Add a static pressure measurement.
void ThreadedKFVio::addBarometerMeasurement(const okvis::Time &, double, double) {

  OKVIS_THROW(Exception, "Barometer measurements not supported")
}

// Add a differential pressure measurement.
void ThreadedKFVio::addDifferentialPressureMeasurement(const okvis::Time &,
                                                       double, double) {

  OKVIS_THROW(Exception, "Differential pressure measurements not supported")
}

// Set the blocking variable that indicates whether the addMeasurement() functions
// should return immediately (blocking=false), or only when the processing is complete.
void ThreadedKFVio::setBlocking(bool blocking) {
  blocking_ = blocking;
  // disable time limit for optimization
  if(blocking_) {
    std::lock_guard<std::mutex> lock(estimator_mutex_);
    estimator_.setOptimizationTimeLimit(-1.0,parameters_.optimization.max_iterations);
  }
}

// Loop to process frames from camera with index cameraIndex
//处理第Index个相机拍摄的图像
void ThreadedKFVio::frameConsumerLoop(size_t cameraIndex) {
  std::shared_ptr<okvis::CameraMeasurement> frame;//定义相机一帧数据的指针
  std::shared_ptr<okvis::MultiFrame> multiFrame;//定义一帧图像,特征点,描述子,位姿的指针
  //TimerSwitchable的类型就是Timer,且将每一个string字符串对应一个数字(句柄)
  TimerSwitchable beforeDetectTimer("1.1 frameLoopBeforeDetect"+std::to_string(cameraIndex),true);
  TimerSwitchable waitForFrameSynchronizerMutexTimer("1.1.1 waitForFrameSynchronizerMutex"+std::to_string(cameraIndex),true);
  TimerSwitchable addNewFrameToSynchronizerTimer("1.1.2 addNewFrameToSynchronizer"+std::to_string(cameraIndex),true);
  TimerSwitchable waitForStateVariablesMutexTimer("1.1.3 waitForStateVariablesMutex"+std::to_string(cameraIndex),true);
  TimerSwitchable propagationTimer("1.1.4 propagationTimer"+std::to_string(cameraIndex),true);
  TimerSwitchable detectTimer("1.2 detectAndDescribe"+std::to_string(cameraIndex),true);
  TimerSwitchable afterDetectTimer("1.3 afterDetect"+std::to_string(cameraIndex),true);
  TimerSwitchable waitForFrameSynchronizerMutexTimer2("1.3.1 waitForFrameSynchronizerMutex2"+std::to_string(cameraIndex),true);
  TimerSwitchable waitForMatchingThreadTimer("1.4 waitForMatchingThread"+std::to_string(cameraIndex),true);


  for (;;) {
    // get data and check for termination request
    // 提取第cameraIndex个相机拍摄的信息，最老的一帧，储存在frame中
    if (cameraMeasurementsReceived_[cameraIndex]->PopBlocking(&frame) == false) {
      return;
    }
    beforeDetectTimer.start();//开始beforeDetectTimer,运行状态为真,且m_time为当前时间
    {  // lock the frame synchronizer
      waitForFrameSynchronizerMutexTimer.start();//开始waitForFrameSynchronizerMutexTimer,运行状态为真,且m_time为当前时间
      std::lock_guard<std::mutex> lock(frameSynchronizer_mutex_);
      waitForFrameSynchronizerMutexTimer.stop();//停止waitForFrameSynchronizerMutexTimer,运行状态为假,且保存了从start到stop的时间段,并将对应句柄和时间间隔进行存储
      // add new frame to frame synchronizer and get the MultiFrame containing it
      addNewFrameToSynchronizerTimer.start();//开始addNewFrameToSynchronizerTimer，运行状态为真，且m_time为当前时间
      multiFrame = frameSynchronizer_.addNewFrame(frame);//将帧frame添加到多帧同步器(包含双目图像和校正后时间戳的类)中
      addNewFrameToSynchronizerTimer.stop();//停止addNewFrameToSynchronizerTimer,运行状态为假,且保存了从start到stop的时间段,并将对应句柄和时间间隔进行储存
    }  // unlock frameSynchronizer only now as we can be sure that not two states are added for the same timestamp
    okvis::kinematics::Transformation T_WS;
    okvis::Time lastTimestamp;
    okvis::SpeedAndBias speedAndBiases;
    // copy last state variables
    {
      waitForStateVariablesMutexTimer.start();//开始waitForStateVariablesMutexTimer,运行状态为真,且m_time为当前时间
      std::lock_guard<std::mutex> lock(lastState_mutex_);
      waitForStateVariablesMutexTimer.stop();//停止waitForStateVariablesMutexTimer,运行状态为假,且保存了从start到stop的时间段,并将对应句柄和时间间隔进行存储
      T_WS = lastOptimized_T_WS_;//最后一次优化后的位姿结果
      speedAndBiases = lastOptimizedSpeedAndBiases_;//最后一次优化后的速度和偏差
      lastTimestamp = lastOptimizedStateTimestamp_;//最后一次优化后的时间戳
    }

    // -- get relevant imu messages for new state
    //得到IMU数据的结束时间(当前帧的时间戳+0.02）
    okvis::Time imuDataEndTime = multiFrame->timestamp()
        + temporal_imu_data_overlap;//temporal_imu_data_overlap=0.02
    //得到IMU数据的开始时间(上一帧的时间戳-0.02)
    okvis::Time imuDataBeginTime = lastTimestamp - temporal_imu_data_overlap;

    OKVIS_ASSERT_TRUE_DBG(Exception,imuDataBeginTime < imuDataEndTime,"imu data end time is smaller than begin time.");

    // wait until all relevant imu messages have arrived and check for termination request
    //IMU的时间同步器
    if (imuFrameSynchronizer_.waitForUpToDateImuData(
      okvis::Time(imuDataEndTime)) == false)  {
      return;
    }

    OKVIS_ASSERT_TRUE_DBG(Exception,
                          imuDataEndTime < imuMeasurements_.back().timeStamp,
                          "Waiting for up to date imu data seems to have failed!");//提示一直等待到imuDataEndTime时刻
    //获得时间段(imuDataBeginTime, imuDataEndTime)内的IMU观测值
    okvis::ImuMeasurementDeque imuData = getImuMeasurments(imuDataBeginTime,
                                                           imuDataEndTime);

    // if imu_data is empty, either end_time > begin_time or
    // no measurements in timeframe, should not happen, as we waited for measurements
    //如果没有满足时间要求的IMU观测数据
    if (imuData.size() == 0) {
      beforeDetectTimer.stop();//停止,并记录时间
      continue;
    }
    //如果IMU数据序列中第一个IMU数据的时间戳大于相机帧的时间戳
    ///IMU第一帧数据之前的那一帧Image也抛弃，下一帧Image（第一帧Frame）才进行特征检测处理。
    if (imuData.front().timeStamp > frame->timeStamp) {
      LOG(WARNING) << "Frame is newer than oldest IMU measurement. Dropping it.";
      beforeDetectTimer.stop();
      continue;
    }

    // get T_WC(camIndx) for detectAndDescribe()
    ///状态求解器中状态的数目为空
    ///第一帧之前的IMU数据会用来计算pose（该函数返回值永远是true，因此initPose是否准确完全依赖IMU给出的读数）
    if (estimator_.numFrames() == 0) {
      // first frame ever
      bool success = okvis::Estimator::initPoseFromImu(imuData, T_WS);//利用imu初始化初始位姿
      {
        std::lock_guard<std::mutex> lock(lastState_mutex_);
        lastOptimized_T_WS_ = T_WS;
        lastOptimizedSpeedAndBiases_.setZero();
        lastOptimizedSpeedAndBiases_.segment<3>(6) = imu_params_.a0;//IMU中加速度计的初始偏差,  x(6+1 : 6+3)
        lastOptimizedStateTimestamp_ = multiFrame->timestamp();
      }
      OKVIS_ASSERT_TRUE_DBG(Exception, success,
          "pose could not be initialized from imu measurements.");
      if (!success) {
        beforeDetectTimer.stop();
        continue;
      }
    } else {
      /// get old T_WS. 不是初始化
      /// 第一帧之后的IMU数据进行propagation（注意multiframe在单目情形下就是frame），注意到这里propagation的covariance和jacobian均为0，仅仅用于预测，对特征点检测提供先验的T_WC：
      propagationTimer.start();//开始propagationTimer
      okvis::ceres::ImuError::propagation(imuData, parameters_.imu, T_WS,
                                          speedAndBiases, lastTimestamp,
                                          multiFrame->timestamp());//迭代优化
      propagationTimer.stop();//停止propagationTimer,并计时
    }
    okvis::kinematics::Transformation T_WC = T_WS
        * (*parameters_.nCameraSystem.T_SC(frame->sensorId));//求取相机坐标系到世界坐标系的转换
    beforeDetectTimer.stop();//停止beforeDetectTimer
    detectTimer.start();//开始探测,detectTimer
    frontend_.detectAndDescribe(frame->sensorId, multiFrame, T_WC, nullptr);//特征提取
    detectTimer.stop();//停止探测,并计时
    afterDetectTimer.start();//开始afterDetectTimer

    bool push = false;
    {  // we now tell frame synchronizer that detectAndDescribe is done for MF with our timestamp
      waitForFrameSynchronizerMutexTimer2.start();//开始waitForFrameSynchronizerMutexTimer2
      std::lock_guard<std::mutex> lock(frameSynchronizer_mutex_);//上锁
      waitForFrameSynchronizerMutexTimer2.stop();//停止waitForFrameSynchronizerMutexTimer2,并计时
      // 表示当前帧（左或右）完成了特征提取
      frameSynchronizer_.detectionEndedForMultiFrame(multiFrame->id());//Inform the synchronizer that a frame in the multiframe has completed keypoint detection and description.
        //表示当前帧（左和右）完成了特征提取
      if (frameSynchronizer_.detectionCompletedForAllCameras(
          multiFrame->id())) {
//        LOG(INFO) << "detection completed for multiframe with id "<< multi_frame->id();
        push = true;
      }
    }  // unlocking frame synchronizer
    afterDetectTimer.stop();//停止afterDetectTimer,并计时
    //push为真，表示左右目图像都完成了特征提取
    if (push) {
      // use queue size 1 to propagate a congestion to the _cameraMeasurementsReceived queue
      // and check for termination request
      waitForMatchingThreadTimer.start();//开始waitForMatchingThreadTimer
      //将multiFrame添加到keypointMeasurements_的队列中
      ///keypointMeasurements_为类型ThreadSafeQueue<std::shared_ptr<okvis::MultiFrame> >
      //PushBlockingIfFull为ThreadSafeQueue的成员函数,MultiFrame为模板
      if (keypointMeasurements_.PushBlockingIfFull(multiFrame, 1) == false) {
        return;
      }
      waitForMatchingThreadTimer.stop();//停止waitForMatchingThreadTimer,并计时
    }
  }
}

// Loop that matches frames with existing frames.
/// 第二个线程， 帧间匹配线程
void ThreadedKFVio::matchingLoop() {
    //TimerSwitchable表示将字符串表示成一个时间戳
  TimerSwitchable prepareToAddStateTimer("2.1 prepareToAddState",true);
  TimerSwitchable waitForOptimizationTimer("2.2 waitForOptimization",true);
  TimerSwitchable addStateTimer("2.3 addState",true);
  TimerSwitchable matchingTimer("2.4 matching",true);

  for (;;) {
    // get new frame
    //定义双目帧类
    std::shared_ptr<okvis::MultiFrame> frame;

    // get data and check for termination request
    //keypointMeasurements_表示特征提取完成了的双目帧类
    if (keypointMeasurements_.PopBlocking(&frame) == false)
      return;

    prepareToAddStateTimer.start();//prepareToAddStateTimer开始
    // -- get relevant imu messages for new state
    okvis::Time imuDataEndTime = frame->timestamp() + temporal_imu_data_overlap;//当前帧的时间戳+0.02作为IMU结束的时间戳
    okvis::Time imuDataBeginTime = lastAddedStateTimestamp_
        - temporal_imu_data_overlap;

    OKVIS_ASSERT_TRUE_DBG(Exception,imuDataBeginTime < imuDataEndTime,
        "imu data end time is smaller than begin time." <<
        "current frametimestamp " << frame->timestamp() << " (id: " << frame->id() <<
        "last timestamp         " << lastAddedStateTimestamp_ << " (id: " << estimator_.currentFrameId());

    // wait until all relevant imu messages have arrived and check for termination request
    //等待所有IMU数据的到来
    if (imuFrameSynchronizer_.waitForUpToDateImuData(
        okvis::Time(imuDataEndTime)) == false)
      return; OKVIS_ASSERT_TRUE_DBG(Exception,
        imuDataEndTime < imuMeasurements_.back().timeStamp,
        "Waiting for up to date imu data seems to have failed!");
    //提取时间区间中的IMU数据
    okvis::ImuMeasurementDeque imuData = getImuMeasurments(imuDataBeginTime,
                                                           imuDataEndTime);

    prepareToAddStateTimer.stop();//停止prepareToAddStateTimer
    // if imu_data is empty, either end_time > begin_time or
    // no measurements in timeframe, should not happen, as we waited for measurements
    if (imuData.size() == 0)
      continue;

    // make sure that optimization of last frame is over.
    // TODO If we didn't actually 'pop' the _matchedFrames queue until after optimization this would not be necessary
    {
      waitForOptimizationTimer.start();//开始waitForOptimizationTimer
      std::unique_lock<std::mutex> l(estimator_mutex_);
      //如果不进行优化，则一直等着
      while (!optimizationDone_)
        optimizationNotification_.wait(l);
      waitForOptimizationTimer.stop();//停止waitForOptimizationTimer
      addStateTimer.start();//开始addStateTimer
      okvis::Time t0Matching = okvis::Time::now();//提取当前时间
      bool asKeyframe = false;
      //添加estimator的状态
      if (estimator_.addStates(frame, imuData, asKeyframe)) {
        lastAddedStateTimestamp_ = frame->timestamp();//将当前帧的时间戳赋值给lastAddedStateTimestamp_
        addStateTimer.stop();//停止addStateTimer
      } else {
        LOG(ERROR) << "Failed to add state! will drop multiframe.";
        addStateTimer.stop();
        continue;
      }

      // -- matching keypoints, initialising landmarks etc.
      okvis::kinematics::Transformation T_WS;
      //得到当前帧的位姿矩阵
      estimator_.get_T_WS(frame->id(), T_WS);
      matchingTimer.start();//匹配matchingTimer开始
      //匹配函数+地标点初始化创建函数+RANSAC函数+判断关键帧函数
      frontend_.dataAssociationAndInitialization(estimator_, T_WS, parameters_,
                                                 map_, frame, &asKeyframe);
      matchingTimer.stop();
      if (asKeyframe)
        estimator_.setKeyframe(frame->id(), asKeyframe);
      // Set the blocking variable that indicates whether the addMeasurement() functions
      // should return immediately (blocking=false), or only when the processing is complete.
      if(!blocking_) {
        double timeLimit = parameters_.optimization.timeLimitForMatchingAndOptimization
                           -(okvis::Time::now()-t0Matching).toSec();//计算剩余的时间
        estimator_.setOptimizationTimeLimit(std::max<double>(0.0, timeLimit),
                                            parameters_.optimization.min_iterations);
      }
      optimizationDone_ = false;//它为false，一直等待
    }  // unlock estimator_mutex_

    // use queue size 1 to propagate a congestion to the _matchedFrames queue
    //将匹配好的双目帧保存到matchedFrames_
    if (matchedFrames_.PushBlockingIfFull(frame, 1) == false)
      return;
  }
}

// Loop to process IMU measurements.
//IMU 帧处理函数
void ThreadedKFVio::imuConsumerLoop() {
  okvis::ImuMeasurement data;
  TimerSwitchable processImuTimer("0 processImuMeasurements",true);
  for (;;) {
    // get data and check for termination request
      //提取IMU的一帧数据
    if (imuMeasurementsReceived_.PopBlocking(&data) == false)
      return;
    processImuTimer.start();//表示开始一个processImuTimer
    okvis::Time start;
    const okvis::Time* end;  // do not need to copy end timestamp
    {
      std::lock_guard<std::mutex> imuLock(imuMeasurements_mutex_);
      OKVIS_ASSERT_TRUE(Exception,
                        imuMeasurements_.empty()
                        || imuMeasurements_.back().timeStamp < data.timeStamp,
                        "IMU measurement from the past received");
      //Should the state that is propagated with IMU messages be published? Or just the optimized ones?
      //IMU信息积分得到的状态是否被打印
      if (parameters_.publishing.publishImuPropagatedState) {
        if (!repropagationNeeded_ && imuMeasurements_.size() > 0) {
          start = imuMeasurements_.back().timeStamp;//赋值起始时间为IMU观测最后一个的时间戳
        } else if (repropagationNeeded_) {
          //repropagationNeeded_为真表示执行了优化
          std::lock_guard<std::mutex> lastStateLock(lastState_mutex_);//需要重积分
          start = lastOptimizedStateTimestamp_;
          T_WS_propagated_ = lastOptimized_T_WS_;
          speedAndBiases_propagated_ = lastOptimizedSpeedAndBiases_;
          repropagationNeeded_ = false;
        } else
          start = okvis::Time(0, 0);
        end = &data.timeStamp;//结束时间为当前IMU的时间戳
      }
      imuMeasurements_.push_back(data);//添加IMU数据到观测序列
    }  // unlock _imuMeasurements_mutex

    // notify other threads that imu data with timeStamp is here.
    // 标记新来的一帧数据的时间戳已经大于要求的时间界限
    imuFrameSynchronizer_.gotImuData(data.timeStamp);
    //如果要求IMU信息积分得到的状态被打印
    if (parameters_.publishing.publishImuPropagatedState) {
      Eigen::Matrix<double, 15, 15> covariance;
      Eigen::Matrix<double, 15, 15> jacobian;
      //IMU状态的积分和预测
      frontend_.propagation(imuMeasurements_, imu_params_, T_WS_propagated_,
                            speedAndBiases_propagated_, start, *end, &covariance,
                            &jacobian);
      ///我觉得应该是给了优化求解器一个初始值(近似估计值)
      OptimizationResults result;//优化变量
      result.stamp = *end;//赋值时间戳
      result.T_WS = T_WS_propagated_;//赋值IMU预积分得到的位姿
      result.speedAndBiases = speedAndBiases_propagated_;//赋值速度和偏差的预积分
      result.omega_S = imuMeasurements_.back().measurement.gyroscopes
          - speedAndBiases_propagated_.segment<3>(3);//赋值最后一个IMU数据的角速度
      //赋值两个相机的外参
      for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
        result.vector_of_T_SCi.push_back(
            okvis::kinematics::Transformation(
                *parameters_.nCameraSystem.T_SC(i)));
      }
      result.onlyPublishLandmarks = false;//赋值仅打印地标点为错
      optimizationResults_.PushNonBlockingDroppingIfFull(result,1);//如果序列optimizationResults_中的值大于1,则需要pop掉最老的一个，然后才可以添加result
    }
    processImuTimer.stop();
  }
}

// Loop to process position measurements.
// 处理位姿的线程
void ThreadedKFVio::positionConsumerLoop() {
  okvis::PositionMeasurement data;
  for (;;) {
    // get data and check for termination request
    if (positionMeasurementsReceived_.PopBlocking(&data) == false)
      return;
    // collect
    {
      std::lock_guard<std::mutex> positionLock(positionMeasurements_mutex_);
      positionMeasurements_.push_back(data);
    }
  }
}

// Loop to process GPS measurements.
void ThreadedKFVio::gpsConsumerLoop() {
}

// Loop to process magnetometer measurements.
void ThreadedKFVio::magnetometerConsumerLoop() {
}

// Loop to process differential pressure measurements.
void ThreadedKFVio::differentialConsumerLoop() {
}

// Loop that visualizes completed frames.
// 可视化处理线程
void ThreadedKFVio::visualizationLoop() {
  okvis::VioVisualizer visualizer_(parameters_);
  for (;;) {
    VioVisualizer::VisualizationData::Ptr new_data;
    //visualizationData_的添加与更新在优化的线程中
    if (visualizationData_.PopBlocking(&new_data) == false)
      return;
    //visualizer_.showDebugImages(new_data);
    std::vector<cv::Mat> out_images(parameters_.nCameraSystem.numCameras());
    for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
      out_images[i] = visualizer_.drawMatches(new_data, i);//绘制第几个图
    }
    //序列displayImages_中如果元素数量大于1,则需要先剔除一个最老的元素，然后才添加新的元素
    displayImages_.PushNonBlockingDroppingIfFull(out_images,1);//展示图像的序列添加图像
  }
}

// trigger display (needed because OSX won't allow threaded display)
void ThreadedKFVio::display() {
  std::vector<cv::Mat> out_images;
  if (displayImages_.Size() == 0)
	return;
  if (displayImages_.PopBlocking(&out_images) == false)
    return;
  // draw
  // 展示图像并设置窗口句柄
  for (size_t im = 0; im < parameters_.nCameraSystem.numCameras(); im++) {
    std::stringstream windowname;
    windowname << "OKVIS camera " << im;
    cv::imshow(windowname.str(), out_images[im]);
  }
  cv::waitKey(1);
}

// Get a subset of the recorded IMU measurements.
//获得满足时间范围的IMU数据
okvis::ImuMeasurementDeque ThreadedKFVio::getImuMeasurments(
    okvis::Time& imuDataBeginTime, okvis::Time& imuDataEndTime) {
  // sanity checks:
  // if end time is smaller than begin time, return empty queue.
  // if begin time is larger than newest imu time, return empty queue.
  // IMU数据还没来得及跟上, imuMeasurements_为IMU数据序列
  if (imuDataEndTime < imuDataBeginTime
      || imuDataBeginTime > imuMeasurements_.back().timeStamp)
    return okvis::ImuMeasurementDeque();

  std::lock_guard<std::mutex> lock(imuMeasurements_mutex_);
  // get iterator to imu data before previous frame
  okvis::ImuMeasurementDeque::iterator first_imu_package = imuMeasurements_
      .begin();
  okvis::ImuMeasurementDeque::iterator last_imu_package =
      imuMeasurements_.end();
  // TODO go backwards through queue. Is probably faster.
  for (auto iter = imuMeasurements_.begin(); iter != imuMeasurements_.end();
      ++iter) {
    // move first_imu_package iterator back until iter->timeStamp is higher than requested begintime
    //开始阶段前的IMU数据
    if (iter->timeStamp <= imuDataBeginTime)
      first_imu_package = iter;//寻找开始时的迭代器

    // set last_imu_package iterator as soon as we hit first timeStamp higher than requested endtime & break
    //结束阶段后的IMU数据
    if (iter->timeStamp >= imuDataEndTime) {
      last_imu_package = iter;
      // since we want to include this last imu measurement in returned Deque we
      // increase last_imu_package iterator once.
      ++last_imu_package;
      break;
    }
  }

  // create copy of imu buffer
  //返回满足时间段要求的IMU数据
  return okvis::ImuMeasurementDeque(first_imu_package, last_imu_package);
}

// Remove IMU measurements from the internal buffer.
// 删除掉IMU测量数据中eraseUntil之前的数据
int ThreadedKFVio::deleteImuMeasurements(const okvis::Time& eraseUntil) {
  std::lock_guard<std::mutex> lock(imuMeasurements_mutex_);
  if (imuMeasurements_.front().timeStamp > eraseUntil)
    return 0;

  okvis::ImuMeasurementDeque::iterator eraseEnd;
  int removed = 0;
  for (auto it = imuMeasurements_.begin(); it != imuMeasurements_.end(); ++it) {
    eraseEnd = it;
    if (it->timeStamp >= eraseUntil)
      break;
    ++removed;
  }

  imuMeasurements_.erase(imuMeasurements_.begin(), eraseEnd);

  return removed;
}

// Loop that performs the optimization and marginalisation.
// 优化和边缘化线程
void ThreadedKFVio::optimizationLoop() {
  TimerSwitchable optimizationTimer("3.1 optimization",true);
  TimerSwitchable marginalizationTimer("3.2 marginalization",true);
  TimerSwitchable afterOptimizationTimer("3.3 afterOptimization",true);

  for (;;) {
    std::shared_ptr<okvis::MultiFrame> frame_pairs;
    VioVisualizer::VisualizationData::Ptr visualizationDataPtr;
    okvis::Time deleteImuMeasurementsUntil(0, 0);
    //提取一帧已经匹配好了的帧（经过了match线程）
    if (matchedFrames_.PopBlocking(&frame_pairs) == false)
      return;
    OptimizationResults result;
    {
      std::lock_guard<std::mutex> l(estimator_mutex_);
      optimizationTimer.start();
      //if(frontend_.isInitialized()){
        estimator_.optimize(parameters_.optimization.max_iterations, 2, false);//2个线程，优化函数
      //}
      /*if (estimator_.numFrames() > 0 && !frontend_.isInitialized()){
        // undo translation
        for(size_t n=0; n<estimator_.numFrames(); ++n){
          okvis::kinematics::Transformation T_WS_0;
          estimator_.get_T_WS(estimator_.frameIdByAge(n),T_WS_0);
          Eigen::Matrix4d T_WS_0_mat = T_WS_0.T();
          T_WS_0_mat.topRightCorner<3,1>().setZero();
          estimator_.set_T_WS(estimator_.frameIdByAge(n),okvis::kinematics::Transformation(T_WS_0_mat));
          okvis::SpeedAndBias sb_0 = okvis::SpeedAndBias::Zero();
          if(estimator_.getSpeedAndBias(estimator_.frameIdByAge(n), 0, sb_0)){
            sb_0.head<3>().setZero();
            estimator_.setSpeedAndBias(estimator_.frameIdByAge(n), 0, sb_0);
          }
        }
      }*/

      optimizationTimer.stop();

      // get timestamp of last frame in IMU window. Need to do this before marginalization as it will be removed there (if not keyframe)
      /// 确定需要删除多少个IMU的观测值
      /// numImuFrames=3
      if (estimator_.numFrames()
          > size_t(parameters_.optimization.numImuFrames)) {
        deleteImuMeasurementsUntil = estimator_.multiFrame(
            estimator_.frameIdByAge(parameters_.optimization.numImuFrames))
            ->timestamp() - temporal_imu_data_overlap;//3帧前的时间戳减去重叠时间
      }

      marginalizationTimer.start();//开始边缘化
      estimator_.applyMarginalizationStrategy(
          parameters_.optimization.numKeyframes,
          parameters_.optimization.numImuFrames, result.transferredLandmarks);
      marginalizationTimer.stop();
      afterOptimizationTimer.start();

      // now actually remove measurements
      deleteImuMeasurements(deleteImuMeasurementsUntil);//删除deleteImuMeasurementsUntil之前的IMU观测值

      // saving optimized state and saving it in OptimizationResults struct
      {
        std::lock_guard<std::mutex> lock(lastState_mutex_);
        estimator_.get_T_WS(frame_pairs->id(), lastOptimized_T_WS_);//得到最优的T_WS
        estimator_.getSpeedAndBias(frame_pairs->id(), 0,
                                   lastOptimizedSpeedAndBiases_);//得到优化后的速度和偏置
        lastOptimizedStateTimestamp_ = frame_pairs->timestamp();//上一个优化的时间戳

        // if we publish the state after each IMU propagation we do not need to publish it here.
        // 如果我们打印状态
        if (!parameters_.publishing.publishImuPropagatedState) {
          result.T_WS = lastOptimized_T_WS_;
          result.speedAndBiases = lastOptimizedSpeedAndBiases_;
          result.stamp = lastOptimizedStateTimestamp_;
          result.onlyPublishLandmarks = false;//只打印地标点
        }
        else
          result.onlyPublishLandmarks = true;
        estimator_.getLandmarks(result.landmarksVector);//得到地标点

        repropagationNeeded_ = true;
      }

      if (parameters_.visualization.displayImages) {
        // fill in information that requires access to estimator.
        visualizationDataPtr = VioVisualizer::VisualizationData::Ptr(
            new VioVisualizer::VisualizationData());
        visualizationDataPtr->observations.resize(frame_pairs->numKeypoints());
        okvis::MapPoint landmark;
        okvis::ObservationVector::iterator it = visualizationDataPtr
            ->observations.begin();
        for (size_t camIndex = 0; camIndex < frame_pairs->numFrames();
            ++camIndex) {
          for (size_t k = 0; k < frame_pairs->numKeypoints(camIndex); ++k) {
            OKVIS_ASSERT_TRUE_DBG(Exception,it != visualizationDataPtr->observations.end(),"Observation-vector not big enough");
            it->keypointIdx = k;
            frame_pairs->getKeypoint(camIndex, k, it->keypointMeasurement);
            frame_pairs->getKeypointSize(camIndex, k, it->keypointSize);
            it->cameraIdx = camIndex;
            it->frameId = frame_pairs->id();
            it->landmarkId = frame_pairs->landmarkId(camIndex, k);
            if (estimator_.isLandmarkAdded(it->landmarkId)) {
              estimator_.getLandmark(it->landmarkId, landmark);
              it->landmark_W = landmark.point;
              if (estimator_.isLandmarkInitialized(it->landmarkId))
                it->isInitialized = true;
              else
                it->isInitialized = false;
            } else {
              it->landmark_W = Eigen::Vector4d(0, 0, 0, 0);  // set to infinity to tell visualizer that landmark is not added
            }
            ++it;
          }
        }
        visualizationDataPtr->keyFrames = estimator_.multiFrame(
            estimator_.currentKeyframeId());
        estimator_.get_T_WS(estimator_.currentKeyframeId(),
                            visualizationDataPtr->T_WS_keyFrame);
      }

      optimizationDone_ = true;
    }  // unlock mutex
    optimizationNotification_.notify_all();

    if (!parameters_.publishing.publishImuPropagatedState) {
      // adding further elements to result that do not access estimator.
      for (size_t i = 0; i < parameters_.nCameraSystem.numCameras(); ++i) {
        result.vector_of_T_SCi.push_back(
            okvis::kinematics::Transformation(
                *parameters_.nCameraSystem.T_SC(i)));
      }
    }
    optimizationResults_.Push(result);

    // adding further elements to visualization data that do not access estimator
    if (parameters_.visualization.displayImages) {
      visualizationDataPtr->currentFrames = frame_pairs;
      visualizationData_.PushNonBlockingDroppingIfFull(visualizationDataPtr, 1);
    }
    afterOptimizationTimer.stop();
  }
}

// Loop that publishes the newest state and landmarks.
void ThreadedKFVio::publisherLoop() {
  for (;;) {
    // get the result data
    OptimizationResults result;
    if (optimizationResults_.PopBlocking(&result) == false)
      return;

    // call all user callbacks
    if (stateCallback_ && !result.onlyPublishLandmarks)
      stateCallback_(result.stamp, result.T_WS);
    if (fullStateCallback_ && !result.onlyPublishLandmarks)
      fullStateCallback_(result.stamp, result.T_WS, result.speedAndBiases,
                         result.omega_S);
    if (fullStateCallbackWithExtrinsics_ && !result.onlyPublishLandmarks)
      fullStateCallbackWithExtrinsics_(result.stamp, result.T_WS,
                                       result.speedAndBiases, result.omega_S,
                                       result.vector_of_T_SCi);
    if (landmarksCallback_ && !result.landmarksVector.empty())
      landmarksCallback_(result.stamp, result.landmarksVector,
                         result.transferredLandmarks);  //TODO(gohlp): why two maps?
  }
}

}  // namespace okvis
