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
 *  Created on: Jun 17, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file VioParametersReader.cpp
 * @brief Source file for the VioParametersReader class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <algorithm>

#include <glog/logging.h>

#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>

#include <opencv2/core/core.hpp>

#include <okvis/VioParametersReader.hpp>

#ifdef HAVE_LIBVISENSOR
  #include <visensor/visensor_api.hpp>
#endif

/// \brief okvis Main namespace of this package.
namespace okvis {

// The default constructor.
VioParametersReader::VioParametersReader()
    : useDriver(false),
      readConfigFile_(false) {
  vioParameters_.publishing.publishRate = 0;
}

// The constructor. This calls readConfigFile().
//在主函数中调用了该构造函数
VioParametersReader::VioParametersReader(const std::string& filename) {
  // reads
  readConfigFile(filename);
}

// Read and parse a config file.
//读取相机参数文件
void VioParametersReader::readConfigFile(const std::string& filename) {
  vioParameters_.optimization.useMedianFilter = false;//使用中值滤波为false
  vioParameters_.optimization.timeReserve.fromSec(0.005);//开始和结束阶段的时间缓冲

  // reads
  cv::FileStorage file(filename, cv::FileStorage::READ);//读取文件

  OKVIS_ASSERT_TRUE(Exception, file.isOpened(),
                    "Could not open config file: " << filename);//打开错误
  LOG(INFO) << "Opened configuration file: " << filename;

  // number of keyframes
  //优化窗口中关键帧的数量
  if (file["numKeyframes"].isInt()) {
    file["numKeyframes"] >> vioParameters_.optimization.numKeyframes;
  } else {
    LOG(WARNING)
        << "numKeyframes parameter not provided. Setting to default numKeyframes=5.";
    vioParameters_.optimization.numKeyframes = 5;
  }
  // number of IMU frames
  //优化窗口中IMU的数量
  if (file["numImuFrames"].isInt()) {
    file["numImuFrames"] >> vioParameters_.optimization.numImuFrames;
  } else {
    LOG(WARNING)
        << "numImuFrames parameter not provided. Setting to default numImuFrames=2.";
    vioParameters_.optimization.numImuFrames = 2;
  }
  // minimum ceres iterations
  //最小优化迭代次数
  if (file["ceres_options"]["minIterations"].isInt()) {
    file["ceres_options"]["minIterations"]
        >> vioParameters_.optimization.min_iterations;
  } else {
    LOG(WARNING)
        << "ceres_options: minIterations parameter not provided. Setting to default minIterations=1";
    vioParameters_.optimization.min_iterations = 1;
  }
  // maximum ceres iterations
  //最大优化迭代次数
  if (file["ceres_options"]["maxIterations"].isInt()) {
    file["ceres_options"]["maxIterations"]
        >> vioParameters_.optimization.max_iterations;
  } else {
    LOG(WARNING)
        << "ceres_options: maxIterations parameter not provided. Setting to default maxIterations=10.";
    vioParameters_.optimization.max_iterations = 10;
  }
  // ceres time limit
  //优化时间限制
  if (file["ceres_options"]["timeLimit"].isReal()) {
    file["ceres_options"]["timeLimit"] >> vioParameters_.optimization.timeLimitForMatchingAndOptimization;
  } else {
    LOG(WARNING)
        << "ceres_options: timeLimit parameter not provided. Setting no time limit.";
    vioParameters_.optimization.timeLimitForMatchingAndOptimization = -1.0;
  }

  // do we use the direct driver?
  //是否使用驱动，判断结果储存在useDriver
  bool success = parseBoolean(file["useDriver"], useDriver);
  OKVIS_ASSERT_TRUE(Exception, success,
                    "'useDriver' parameter missing in configuration file.");

  // display images?
  //是否展示图像，判断结果储存在vioParameters_.visualization.displayImages
  success = parseBoolean(file["displayImages"],
                         vioParameters_.visualization.displayImages);
  OKVIS_ASSERT_TRUE(Exception, success,
                    "'displayImages' parameter missing in configuration file.");

  // detection threshold
  //特征点探测半径
  success = file["detection_options"]["threshold"].isReal();
  OKVIS_ASSERT_TRUE(
      Exception, success,
      "'detection threshold' parameter missing in configuration file.");
  file["detection_options"]["threshold"] >> vioParameters_.optimization.detectionThreshold;

  // detection octaves
  //特征点探测的图像层数
  success = file["detection_options"]["octaves"].isInt();
  OKVIS_ASSERT_TRUE(
      Exception, success,
      "'detection octaves' parameter missing in configuration file.");
  file["detection_options"]["octaves"] >> vioParameters_.optimization.detectionOctaves;
  OKVIS_ASSERT_TRUE(Exception,
                    vioParameters_.optimization.detectionOctaves >= 0,
                    "Invalid parameter value.");

  // maximum detections
  //一张图像中最多可以有多少个特征点
  success = file["detection_options"]["maxNoKeypoints"].isInt();
  OKVIS_ASSERT_TRUE(
      Exception, success,
      "'detection maxNoKeypoints' parameter missing in configuration file.");
  file["detection_options"]["maxNoKeypoints"] >> vioParameters_.optimization.maxNoKeypoints;
  OKVIS_ASSERT_TRUE(Exception,
                    vioParameters_.optimization.maxNoKeypoints >= 0,
                    "Invalid parameter value.");

  // image delay
  //图像的延迟
  success = file["imageDelay"].isReal();
  OKVIS_ASSERT_TRUE(Exception, success,
                    "'imageDelay' parameter missing in configuration file.");
  file["imageDelay"] >> vioParameters_.sensors_information.imageDelay;
  LOG(INFO) << "imageDelay=" << vioParameters_.sensors_information.imageDelay;

  // camera rate
  //相机的帧率
  success = file["camera_params"]["camera_rate"].isInt();
  OKVIS_ASSERT_TRUE(
      Exception, success,
      "'camera_params: camera_rate' parameter missing in configuration file.");
  file["camera_params"]["camera_rate"]
      >> vioParameters_.sensors_information.cameraRate;

  // timestamp tolerance
  //时间戳的容忍度
  if (file["camera_params"]["timestamp_tolerance"].isReal()) {
    file["camera_params"]["timestamp_tolerance"]
        >> vioParameters_.sensors_information.frameTimestampTolerance;
    OKVIS_ASSERT_TRUE(
        Exception,
        vioParameters_.sensors_information.frameTimestampTolerance
            < 0.5 / vioParameters_.sensors_information.cameraRate,
        "Timestamp tolerance for stereo frames is larger than half the time between frames.");
    OKVIS_ASSERT_TRUE(
        Exception,
        vioParameters_.sensors_information.frameTimestampTolerance >= 0.0,
        "Timestamp tolerance is smaller than 0");
  } else {
    vioParameters_.sensors_information.frameTimestampTolerance = 0.2
        / vioParameters_.sensors_information.cameraRate;
    LOG(WARNING)
        << "No timestamp tolerance for stereo frames specified. Setting to "
        << vioParameters_.sensors_information.frameTimestampTolerance;
  }

  // camera params
  //The standard deviation of the camera extrinsics translation
  //相机外参平移的标准差
  if (file["camera_params"]["sigma_absolute_translation"].isReal()) {
    file["camera_params"]["sigma_absolute_translation"]
        >> vioParameters_.camera_extrinsics.sigma_absolute_translation;
  } else {
    vioParameters_.camera_extrinsics.sigma_absolute_translation = 0.0;
    LOG(WARNING)
        << "camera_params: sigma_absolute_translation parameter not provided. Setting to default 0.0";
  }
  //相机外参旋转的标准差
  if (file["camera_params"]["sigma_absolute_orientation"].isReal()) {
    file["camera_params"]["sigma_absolute_orientation"]
        >> vioParameters_.camera_extrinsics.sigma_absolute_orientation;
  } else {
    vioParameters_.camera_extrinsics.sigma_absolute_orientation = 0.0;
    LOG(WARNING)
        << "camera_params: sigma_absolute_orientation parameter not provided. Setting to default 0.0";
  }
  //相机两帧间平移的标准差
  if (file["camera_params"]["sigma_c_relative_translation"].isReal()) {
    file["camera_params"]["sigma_c_relative_translation"]
        >> vioParameters_.camera_extrinsics.sigma_c_relative_translation;
  } else {
    vioParameters_.camera_extrinsics.sigma_c_relative_translation = 0.0;
    LOG(WARNING)
        << "camera_params: sigma_c_relative_translation parameter not provided. Setting to default 0.0";
  }
  //相机两帧间旋转的标准差
  if (file["camera_params"]["sigma_c_relative_orientation"].isReal()) {
    file["camera_params"]["sigma_c_relative_orientation"]
        >> vioParameters_.camera_extrinsics.sigma_c_relative_orientation;
  } else {
    vioParameters_.camera_extrinsics.sigma_c_relative_orientation = 0.0;
    LOG(WARNING)
        << "camera_params: sigma_c_relative_orientation parameter not provided. Setting to default 0.0";
  }
   //rate at which odometry updates are published only works properly if imu_rate/publish_rate is an integer!!
  //历程计更新发布的频率，必须要求-----IMU频率/该频率为整数
  if(file["publishing_options"]["publish_rate"].isInt()) {
    file["publishing_options"]["publish_rate"] 
        >> vioParameters_.publishing.publishRate;
  }
  //地标点精确度要求的最低阈值
  if (file["publishing_options"]["landmarkQualityThreshold"].isReal()) {
    file["publishing_options"]["landmarkQualityThreshold"]
        >> vioParameters_.publishing.landmarkQualityThreshold;
  }
  //有最高精度的地标点将会被打印为最强烈的颜色
  if (file["publishing_options"]["maximumLandmarkQuality"].isReal()) {
    file["publishing_options"]["maximumLandmarkQuality"]
        >> vioParameters_.publishing.maxLandmarkQuality;
  }
   //最大的路径长度
  if (file["publishing_options"]["maxPathLength"].isInt()) {
    vioParameters_.publishing.maxPathLength =
        (int) (file["publishing_options"]["maxPathLength"]);
  }
  //Should the state that is propagated with IMU messages be published? Or just the optimized ones?
  //IMU信息预积分的状态是否打印
  parseBoolean(file["publishing_options"]["publishImuPropagatedState"],
                   vioParameters_.publishing.publishImuPropagatedState);
   //是否打印地标点
  parseBoolean(file["publishing_options"]["publishLandmarks"],
                   vioParameters_.publishing.publishLandmarks);
   //提供定制的世界坐标系
  cv::FileNode T_Wc_W_ = file["publishing_options"]["T_Wc_W"];
  if(T_Wc_W_.isSeq()) {
    Eigen::Matrix4d T_Wc_W_e;
    T_Wc_W_e << T_Wc_W_[0], T_Wc_W_[1], T_Wc_W_[2], T_Wc_W_[3], 
                T_Wc_W_[4], T_Wc_W_[5], T_Wc_W_[6], T_Wc_W_[7],
                T_Wc_W_[8], T_Wc_W_[9], T_Wc_W_[10], T_Wc_W_[11], 
                T_Wc_W_[12], T_Wc_W_[13], T_Wc_W_[14], T_Wc_W_[15];

    vioParameters_.publishing.T_Wc_W = okvis::kinematics::Transformation(T_Wc_W_e);//转换为四元数和平移向量
    std::stringstream s;
    s << vioParameters_.publishing.T_Wc_W.T();//输出齐次矩阵
    LOG(INFO) << "Custom World frame provided T_Wc_W=\n" << s.str();
  }
 //设置跟踪那个坐标系
  if (file["publishing_options"]["trackedBodyFrame"].isString()) {
    std::string frame = (std::string)file["publishing_options"]["trackedBodyFrame"];
    // cut out first word. str currently contains everything including comments
    frame = frame.substr(0, frame.find(" "));
    if (frame.compare("B") == 0)
      vioParameters_.publishing.trackedBodyFrame=FrameName::B;
    else if (frame.compare("S") == 0)
      vioParameters_.publishing.trackedBodyFrame=FrameName::S;
    else {
      LOG(WARNING) << frame << " unknown/invalid frame for trackedBodyFrame, setting to B";
      vioParameters_.publishing.trackedBodyFrame=FrameName::B;
    }
  }
    //速度投影在哪个坐标系
  if (file["publishing_options"]["velocitiesFrame"].isString()) {
    std::string frame = (std::string)file["publishing_options"]["velocitiesFrame"];
    // cut out first word. str currently contains everything including comments
    frame = frame.substr(0, frame.find(" "));
    if (frame.compare("B") == 0)
      vioParameters_.publishing.velocitiesFrame=FrameName::B;
    else if (frame.compare("S") == 0)
      vioParameters_.publishing.velocitiesFrame=FrameName::S;
    else if (frame.compare("Wc") == 0)
      vioParameters_.publishing.velocitiesFrame=FrameName::Wc;
    else {
      LOG(WARNING) << frame << " unknown/invalid frame for velocitiesFrame, setting to Wc";
      vioParameters_.publishing.velocitiesFrame=FrameName::Wc;
    }
  }

  // camera calibration
  //定义容器，类型为CameraCalibration，而aligned_allocator是定义内存管理类
  std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> calibrations;
  //参数内容保存在容器calibrations中
  if(!getCameraCalibration(calibrations, file))
    LOG(FATAL) << "Did not find any calibration!";

  size_t camIdx = 0;
  //遍历每个相机
  for (size_t i = 0; i < calibrations.size(); ++i) {
    //用相机相对IMU的平移和旋转初始化指针T_SC_okvis_ptr
    std::shared_ptr<const okvis::kinematics::Transformation> T_SC_okvis_ptr(
          new okvis::kinematics::Transformation(calibrations[i].T_SC.r(),
                                                calibrations[i].T_SC.q().normalized()));
    //如果distortionType的字符串内容与equidistant相等
    if (strcmp(calibrations[i].distortionType.c_str(), "equidistant") == 0) {
      vioParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          //PinholeCamera为CameraBase的继承函数，EquidistantDistortion为PinholeCamera中声明的模板类
          //Equidistant表示一种畸变矫正的方法
          std::shared_ptr<const okvis::cameras::CameraBase>(
              new okvis::cameras::PinholeCamera<
                  okvis::cameras::EquidistantDistortion>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  okvis::cameras::EquidistantDistortion(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3])/*, id ?*/)),
          okvis::cameras::NCameraSystem::Equidistant/*, computeOverlaps ?*/);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      //信息展示
      LOG(INFO) << "Equidistant pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else if (strcmp(calibrations[i].distortionType.c_str(), "radialtangential") == 0
               || strcmp(calibrations[i].distortionType.c_str(), "plumb_bob") == 0) {
      //另一种畸变矫正的方式
      vioParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          //PinholeCamera为CameraBase的继承函数，RadialTangentialDistortion为PinholeCamera中声明的模板类
          std::shared_ptr<const okvis::cameras::CameraBase>(
              new okvis::cameras::PinholeCamera<
                  okvis::cameras::RadialTangentialDistortion>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  okvis::cameras::RadialTangentialDistortion(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3])/*, id ?*/)),
          okvis::cameras::NCameraSystem::RadialTangential/*, computeOverlaps ?*/);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Radial tangential pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else if (strcmp(calibrations[i].distortionType.c_str(), "radialtangential8") == 0
               || strcmp(calibrations[i].distortionType.c_str(), "plumb_bob8") == 0) {
       //第三种矫正方法
      vioParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::shared_ptr<const okvis::cameras::CameraBase>(
              new okvis::cameras::PinholeCamera<
                  okvis::cameras::RadialTangentialDistortion8>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  okvis::cameras::RadialTangentialDistortion8(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3],
                    calibrations[i].distortionCoefficients[4],
                    calibrations[i].distortionCoefficients[5],
                    calibrations[i].distortionCoefficients[6],
                    calibrations[i].distortionCoefficients[7])/*, id ?*/)),
          okvis::cameras::NCameraSystem::RadialTangential8/*, computeOverlaps ?*/);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Radial tangential 8 pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else {
      LOG(ERROR) << "unrecognized distortion type " << calibrations[i].distortionType;
    }
    ++camIdx;//相机计数加一
  }

  vioParameters_.sensors_information.imuIdx = 0;//IMU的标号

  cv::FileNode T_BS_ = file["imu_params"]["T_BS"];//IMU到机体的转换矩阵
  OKVIS_ASSERT_TRUE(
      Exception,
      T_BS_.isSeq(),
      "'T_BS' parameter missing in the configuration file or in the wrong format.")

  Eigen::Matrix4d T_BS_e;
  T_BS_e << T_BS_[0], T_BS_[1], T_BS_[2], T_BS_[3], T_BS_[4], T_BS_[5], T_BS_[6], T_BS_[7], T_BS_[8], T_BS_[9], T_BS_[10], T_BS_[11], T_BS_[12], T_BS_[13], T_BS_[14], T_BS_[15];//赋值给矩阵

  vioParameters_.imu.T_BS = okvis::kinematics::Transformation(T_BS_e);//赋值给系统参数
  std::stringstream s;
  s << vioParameters_.imu.T_BS.T();
  LOG(INFO) << "IMU with transformation T_BS=\n" << s.str();

  // the IMU parameters
  //IMU的参数
  cv::FileNode imu_params = file["imu_params"];
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["a_max"].isReal(),
      "'imu_params: a_max' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["g_max"].isReal(),
      "'imu_params: g_max' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["sigma_g_c"].isReal(),
      "'imu_params: sigma_g_c' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["sigma_a_c"].isReal(),
      "'imu_params: sigma_a_c' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
       Exception, imu_params["sigma_bg"].isReal(),
       "'imu_params: sigma_bg' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
       Exception, imu_params["sigma_ba"].isReal(),
       "'imu_params: sigma_ba' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["sigma_gw_c"].isReal(),
      "'imu_params: sigma_gw_c' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["sigma_g_c"].isReal(),
      "'imu_params: sigma_g_c' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["tau"].isReal(),
      "'imu_params: tau' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(Exception, imu_params["g"].isReal(),
                    "'imu_params: g' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(Exception, imu_params["a0"].isSeq(),
                    "'imu_params: a0' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["imu_rate"].isInt(),
      "'imu_params: imu_rate' parameter missing in configuration file.");
  imu_params["a_max"] >> vioParameters_.imu.a_max;//加速度计饱和度
  imu_params["g_max"] >> vioParameters_.imu.g_max;//陀螺仪饱和度
  imu_params["sigma_g_c"] >> vioParameters_.imu.sigma_g_c;//陀螺仪的噪声密度
  imu_params["sigma_a_c"] >> vioParameters_.imu.sigma_a_c;//加速度计的噪声密度
  imu_params["sigma_bg"] >> vioParameters_.imu.sigma_bg;//初始陀螺仪偏差
  imu_params["sigma_ba"] >> vioParameters_.imu.sigma_ba;//初始加速度计偏差
  imu_params["sigma_gw_c"] >> vioParameters_.imu.sigma_gw_c;//陀螺飘逸噪声密度
  imu_params["sigma_aw_c"] >> vioParameters_.imu.sigma_aw_c;//加速度计漂移噪声密度
  imu_params["imu_rate"] >> vioParameters_.imu.rate;//IMU的频率
  imu_params["tau"] >> vioParameters_.imu.tau;//加速度计偏差的反转时间常数
  imu_params["g"] >> vioParameters_.imu.g;//重力加速度
  //先验的加速度计偏差的平均值
  vioParameters_.imu.a0 = Eigen::Vector3d((double) (imu_params["a0"][0]),
                                          (double) (imu_params["a0"][1]),
                                          (double) (imu_params["a0"][2]));

  readConfigFile_ = true;//赋值读取文件成功的标志
}

// Parses booleans from a cv::FileNode. OpenCV sadly has no implementation like this.
bool VioParametersReader::parseBoolean(cv::FileNode node, bool& val) const {
    //node中是int型数据
  if (node.isInt()) {
    val = (int) (node) != 0;//判断node中的整形变量是否为0
    return true;
  }
  //node中是string变量
  if (node.isString()) {
    std::string str = (std::string) (node);//提取node中的值
    // cut out first word. str currently contains everything including comments
    str = str.substr(0,str.find(" "));//提取str中最开始到空格的字符串
    // transform it to all lowercase
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);//transform函数是将某操作应用于指定范围的每个元素，tolower功能是把字母字符转换成小写
    /* from yaml.org/type/bool.html:
     * Booleans are formatted as English words
     * (“true”/“false”, “yes”/“no” or “on”/“off”)
     * for readability and may be abbreviated as
     * a single character “y”/“n” or “Y”/“N”. */
    //如果str（参数文件中的word）为false、no、n、off，val都为false
    if (str.compare("false")  == 0
        || str.compare("no")  == 0
        || str.compare("n")   == 0
        || str.compare("off") == 0) {
      val = false;
      return true;
    }
    //如果str为true、yes、y、on，val都为true
    if (str.compare("true")   == 0
        || str.compare("yes") == 0
        || str.compare("y")   == 0
        || str.compare("on")  == 0) {
      val = true;
      return true;
    }
  }
  return false;
}

//获得相机的标定参数，将结果保存在容器calibrations（2维）
bool VioParametersReader::getCameraCalibration(
    std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations,
    cv::FileStorage& configurationFile) {

  bool success = getCalibrationViaConfig(calibrations, configurationFile["cameras"]);

#ifdef HAVE_LIBVISENSOR
  if (useDriver && !success) {
    // start up sensor
    viSensor = std::shared_ptr<void>(
          new visensor::ViSensorDriver());
    try {
      // use autodiscovery to find sensor. TODO: specify IP in config?
      std::static_pointer_cast<visensor::ViSensorDriver>(viSensor)->init();
    } catch (Exception const &ex) {
      LOG(ERROR) << ex.what();
      exit(1);
    }

    success = getCalibrationViaVisensorAPI(calibrations);
  }
#endif

  return success;
}

// Get the camera calibration via the configuration file.
//通过cfg文件获取相机参数，cameraNode为文件提取的结果
bool VioParametersReader::getCalibrationViaConfig(
    std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations,
    cv::FileNode cameraNode) const {

  calibrations.clear();
  bool gotCalibration = false;
  // first check if calibration is available in config file
  if (cameraNode.isSeq()
     && cameraNode.size() > 0) {
    size_t camIdx = 0;
    //遍历cfg文件的提取结果（左右相机各对应一个结构体，遍历每一个结构体）
    for (cv::FileNodeIterator it = cameraNode.begin();
        it != cameraNode.end(); ++it) {
        //其中所有元素都存在且准确赋值
      if ((*it).isMap()
          && (*it)["T_SC"].isSeq()
          && (*it)["image_dimension"].isSeq()
          && (*it)["image_dimension"].size() == 2
          && (*it)["distortion_coefficients"].isSeq()
          && (*it)["distortion_coefficients"].size() >= 4
          && (*it)["distortion_type"].isString()
          && (*it)["focal_length"].isSeq()
          && (*it)["focal_length"].size() == 2
          && (*it)["principal_point"].isSeq()
          && (*it)["principal_point"].size() == 2) {
        LOG(INFO) << "Found calibration in configuration file for camera " << camIdx;
        gotCalibration = true;//标记得到了相机的内参
      } else {
        LOG(WARNING) << "Found incomplete calibration in configuration file for camera " << camIdx
                     << ". Will not use the calibration from the configuration file.";
        return false;
      }
      ++camIdx;
    }
  }
  else
    LOG(INFO) << "Did not find a calibration in the configuration file.";
   //如果得到了内参
  if (gotCalibration) {
      //遍历每一个相机参数的结构体
    for (cv::FileNodeIterator it = cameraNode.begin();
        it != cameraNode.end(); ++it) {

      CameraCalibration calib;//存储标定结果

      cv::FileNode T_SC_node = (*it)["T_SC"];//提取文件中的相机到IMU的旋转信息
      cv::FileNode imageDimensionNode = (*it)["image_dimension"];//图像的大小，分辨率
      cv::FileNode distortionCoefficientNode = (*it)["distortion_coefficients"];//畸变系数
      cv::FileNode focalLengthNode = (*it)["focal_length"];//焦距（像素）
      cv::FileNode principalPointNode = (*it)["principal_point"];//光心坐标（像素）

      // extrinsics
      //将cfg文件中的数据储存到变量calib中
      Eigen::Matrix4d T_SC;
      T_SC << T_SC_node[0], T_SC_node[1], T_SC_node[2], T_SC_node[3], T_SC_node[4], T_SC_node[5], T_SC_node[6], T_SC_node[7], T_SC_node[8], T_SC_node[9], T_SC_node[10], T_SC_node[11], T_SC_node[12], T_SC_node[13], T_SC_node[14], T_SC_node[15];
      calib.T_SC = okvis::kinematics::Transformation(T_SC);//Transformation中内置将齐次矩阵变为四元数和平移向量

      calib.imageDimension << imageDimensionNode[0], imageDimensionNode[1];
      calib.distortionCoefficients.resize(distortionCoefficientNode.size());
      for(size_t i=0; i<distortionCoefficientNode.size(); ++i) {
        calib.distortionCoefficients[i] = distortionCoefficientNode[i];
      }
      calib.focalLength << focalLengthNode[0], focalLengthNode[1];
      calib.principalPoint << principalPointNode[0], principalPointNode[1];
      calib.distortionType = (std::string)((*it)["distortion_type"]);

      calibrations.push_back(calib);
    }
  }
  return gotCalibration;
}

// Get the camera calibrations via the visensor API.
bool VioParametersReader::getCalibrationViaVisensorAPI(
    std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations) const{
#ifdef HAVE_LIBVISENSOR
  if (viSensor == nullptr) {
    LOG(ERROR) << "Tried to get calibration from the sensor. But the sensor is not set up.";
    return false;
  }

  calibrations.clear();

  std::vector<visensor::SensorId::SensorId> listOfCameraIds =
      std::static_pointer_cast<visensor::ViSensorDriver>(viSensor)->getListOfCameraIDs();

  for (auto it = listOfCameraIds.begin(); it != listOfCameraIds.end(); ++it) {
    visensor::ViCameraCalibration calibrationFromAPI;
    okvis::VioParametersReader::CameraCalibration calibration;
    if(!std::static_pointer_cast<visensor::ViSensorDriver>(viSensor)->getCameraCalibration(*it,calibrationFromAPI)) {
      LOG(ERROR) << "Reading the calibration via the sensor API failed.";
      calibrations.clear();
      return false;
    }
    LOG(INFO) << "Reading the calbration for camera " << size_t(*it) << " via API successful";
    double* R = calibrationFromAPI.R;
    double* t = calibrationFromAPI.t;
    // getCameraCalibration apparently gives T_CI back.
    //(Confirmed by comparing it to output of service)
    Eigen::Matrix4d T_CI;
    T_CI << R[0], R[1], R[2], t[0],
            R[3], R[4], R[5], t[1],
            R[6], R[7], R[8], t[2],
            0,    0,    0,    1;
    okvis::kinematics::Transformation T_CI_okvis(T_CI);
    calibration.T_SC = T_CI_okvis.inverse();

    calibration.focalLength << calibrationFromAPI.focal_point[0],
                               calibrationFromAPI.focal_point[1];
    calibration.principalPoint << calibrationFromAPI.principal_point[0],
                                  calibrationFromAPI.principal_point[1];
    calibration.distortionCoefficients.resize(4); // FIXME: 8 coeff support?
    calibration.distortionCoefficients << calibrationFromAPI.dist_coeff[0],
                                          calibrationFromAPI.dist_coeff[1],
                                          calibrationFromAPI.dist_coeff[2],
                                          calibrationFromAPI.dist_coeff[3];
    calibration.imageDimension << 752, 480;
    calibration.distortionType = "plumb_bob";
    calibrations.push_back(calibration);
  }

  return calibrations.empty() == false;
#else
  static_cast<void>(calibrations); // unused
  LOG(ERROR) << "Tried to get calibration directly from the sensor. However libvisensor was not found.";
  return false;
#endif
}


}  // namespace okvis
