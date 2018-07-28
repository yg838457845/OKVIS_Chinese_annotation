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
 *  Created on: Sep 14, 2014
 *      Author: Pascal Gohl
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file FrameSynchronizer.cpp
 * @brief Source file for the FrameSynchronizer class.
 * @author Pascal Gohl
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <glog/logging.h>

#include <okvis/FrameSynchronizer.hpp>
#include <okvis/IdProvider.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

static const int max_frame_sync_buffer_size = 3;

// Constructor. Calls init().
//初始化帧的同步器
FrameSynchronizer::FrameSynchronizer(okvis::VioParameters& parameters)
: parameters_(parameters),
  numCameras_(0),
  timeTol_(parameters.sensors_information.frameTimestampTolerance),
  lastCompletedFrameId_(0)
{
  if(parameters.nCameraSystem.numCameras() > 0) {
    init(parameters);//初始化参数,相机的个数,时间戳的容忍度
  }
  //frameBuffer_表示内存缓冲器,大小为3,类行为pair<MultiFrame,size_t>
  frameBuffer_.resize(max_frame_sync_buffer_size,
                      std::pair<std::shared_ptr<okvis::MultiFrame>,size_t>(nullptr,0));
  bufferPosition_ = 0;//缓存器位置
}

// Trivial destructor.
FrameSynchronizer::~FrameSynchronizer() {
}

// Initialise the synchronizer with new parameters. Is called in the constructor.
// 帧同步器的初始化
void FrameSynchronizer::init(okvis::VioParameters& parameters) {
  parameters_ = parameters;//参数的赋值
  numCameras_ = parameters.nCameraSystem.numCameras();//相机的个数
  timeTol_ = parameters.sensors_information.frameTimestampTolerance;//时间戳的容忍度
  // TODO(gohlp): this fails if camera id's are not consecutive
}

// Adds a new frame to the internal buffer and returns the Multiframe containing the frame.
//添加新的一帧到内容缓存区, 并且返回多帧的类(包含左右图像和校正后的时间戳)
std::shared_ptr<okvis::MultiFrame> FrameSynchronizer::addNewFrame(std::shared_ptr<okvis::CameraMeasurement>& frame) {
  assert(numCameras_ > 0);//numCameras_>0才可以执行下去
  okvis::Time frame_stamp = frame->timeStamp;//帧的时间戳
  std::shared_ptr<okvis::MultiFrame> multiFrame;//多帧类的容器
  int position;
  //在帧缓冲器frameBuffer_中寻找时间戳为frame_stamp的帧序号,储存在position中, 一般用于第二目相机图像的输入
  if(findFrameByTime(frame_stamp,position)) {
    multiFrame = frameBuffer_[position].first;//提取对应时间戳的帧
    OKVIS_ASSERT_TRUE_DBG(Exception,multiFrame->image(frame->sensorId).empty(),
                       "Frame for this camera has already been added to multiframe!");
    //输入帧的时间戳与匹配到帧时间戳不相等
    if(frame_stamp != multiFrame->timestamp()) {
      // timestamps do not agree. setting timestamp to middlepoint
      frame_stamp += (multiFrame->timestamp()-frame_stamp)*0.5;//计算时间戳差的中值
      multiFrame->setTimestamp(frame_stamp);//重新赋值已存在frame的时间戳
    }
    multiFrame->setImage(frame->sensorId,frame->measurement.image);//赋值已存在frame的传感器标记和其观测图像
  }
  else {
    //初始化一个多帧类multiFrame,newId()表示id++, 该部分用于第一目相机图像的输入
    multiFrame = std::shared_ptr<okvis::MultiFrame>(new okvis::MultiFrame(parameters_.nCameraSystem,frame_stamp,
                                                                          okvis::IdProvider::instance().newId()));
    multiFrame->setImage(frame->sensorId,frame->measurement.image);//赋值multiframe的传感器标记和其观测图像
    bufferPosition_ = (bufferPosition_+1) % max_frame_sync_buffer_size;//缓冲器位置加1
    //如果缓存器该位置已经有值或者相机的数目不对
    if(frameBuffer_[bufferPosition_].first != nullptr
       && frameBuffer_[bufferPosition_].second != numCameras_) {
     LOG(ERROR) << "Dropping frame with id " << frameBuffer_[bufferPosition_].first->id();
    }
    //添加元素（只包括双目的左目图像）到帧缓存器frameBuffer_
    frameBuffer_[bufferPosition_].first = multiFrame;
    frameBuffer_[bufferPosition_].second= 0;
  }
  return multiFrame;
}

// Inform the synchronizer that a frame in the multiframe has completed keypoint detection and description.
// 通知类Synchronizer类，一个frame已经完成了特征点的观测和描述子（左目和右目各运行一次）
bool FrameSynchronizer::detectionEndedForMultiFrame(uint64_t multiFrameId) {
  int position;
  bool found = findFrameById(multiFrameId,position);//寻找对应ID的缓冲器中的帧
  if(found) {
    ++frameBuffer_[position].second;//如果找到了，加一，表示该位置的相机个数加一
    OKVIS_ASSERT_TRUE_DBG(Exception,frameBuffer_[position].second<=numCameras_,
                       "Completion counter is larger than the amount of cameras in the system!");
  }
  return found;
}

// This will return true if the internal counter on how many times detectionEndedForMultiFrame()
// has been called for this multiframe equals the number of cameras in the system.
///统计运行detectionEndedForMultiFrame的次数是否等于相机个个数（即左右相机都完成了特征提取）
bool FrameSynchronizer::detectionCompletedForAllCameras(uint64_t multiFrameId) {
  int position;
  if(findFrameById(multiFrameId,position)) {
      //如果缓存器相应位置储存的相机个数等于2, 表示两个相机都完成了特征的提取
    if(frameBuffer_[position].second == numCameras_) {
      OKVIS_ASSERT_TRUE(Exception,frameBuffer_[position].first->timestamp() > lastCompletedFrameTimestamp_
                            && (lastCompletedFrameId_==0 || frameBuffer_[position].first->id() > lastCompletedFrameId_) ,
                     "wrong order!\ntimestamp last: " << lastCompletedFrameTimestamp_
                     << "\ntimestamp new:  " << frameBuffer_[position].first->timestamp()
                     << "\nid last: " << lastCompletedFrameId_
                     << "\nid new:  " << frameBuffer_[position].first->id());
      lastCompletedFrameId_ = frameBuffer_[position].first->id();
      lastCompletedFrameTimestamp_ = frameBuffer_[position].first->timestamp();
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

// Find a multiframe in the buffer that has a timestamp within the tolerances of the given one. The tolerance
// is given as a parameter in okvis::VioParameters::sensors_information::frameTimestampTolerance
// 找到与时间戳timestamp对应的多帧类multiframe
bool FrameSynchronizer::findFrameByTime(const okvis::Time& timestamp, int& position) const{
  bool found = false;
  //max_frame_sync_buffer_size=3
  //frameBuffer_为帧的缓存器,储存pair<MultiFrame,size_t>
  for(int i=0; i < max_frame_sync_buffer_size; ++i) {
    position = (bufferPosition_+i)%max_frame_sync_buffer_size;//缓存器的位置
    //第position位置的多帧同步器MultiFrame
    //第position位置帧时间戳与时间戳参数timestamp的差值小于timeTol_,或相等,或者还没有值
    if(frameBuffer_[position].first != nullptr &&
       (frameBuffer_[position].first->timestamp() == timestamp ||
        fabs((frameBuffer_[position].first->timestamp()-timestamp).toSec()) < timeTol_)) {
      found = true;//寻找到了
      break;
    }
  }
  return found;
}

// Find a multiframe in the buffer for a given multiframe ID.
// 根据ID搜索匹配缓冲器frameBuffer_中储存的
bool FrameSynchronizer::findFrameById(uint64_t mfId, int& position) const {
  bool found = false;
  //相当于从bufferPosition_位置
  for(int i=0; i < max_frame_sync_buffer_size; ++i) {
    position = (bufferPosition_+i)%max_frame_sync_buffer_size;
    if(frameBuffer_[position].first != nullptr &&
       frameBuffer_[position].first->id() == mfId) {
      found = true;
      break;
    }
  }
  return found;
}


} /* namespace okvis */
