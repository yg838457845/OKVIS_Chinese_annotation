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
 *  Created on: Jun 26, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file okvis_app_synchronous.cpp
 * @brief This file processes a dataset.
 
 This node goes through a dataset in order and waits until all processing is done
 before adding a new message to algorithm

 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <memory>
#include <functional>
#include <atomic>

#include <Eigen/Core>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
#include <okvis/VioParametersReader.hpp>
#include <okvis/ThreadedKFVio.hpp>

#include <boost/filesystem.hpp>

//绘图类
class PoseViewer
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  constexpr static const double imageSize = 500.0;
  PoseViewer()
  {
    cv::namedWindow("OKVIS Top View");//定义窗口
    _image.create(imageSize, imageSize, CV_8UC3);//创建空图像
    drawing_ = false;//原子变量
    showing_ = false;//原子变量
  }
  // this we can register as a callback
  //轨迹图绘制函数（ROS版本下好像没有）
  void publishFullStateAsCallback(
      const okvis::Time & /*t*/, const okvis::kinematics::Transformation & T_WS,
      const Eigen::Matrix<double, 9, 1> & speedAndBiases,
      const Eigen::Matrix<double, 3, 1> & /*omega_S*/)
  {

    // just append the path
    Eigen::Vector3d r = T_WS.r();//S到W的平移向量，投影表示在W中
    Eigen::Matrix3d C = T_WS.C();//Returns the rotation matrix ，S到W的旋转矩阵
    _path.push_back(cv::Point2d(r[0], r[1]));//路径
    _heights.push_back(r[2]);//高度
    // maintain scaling（实际场景的边界确定）
    if (r[0] - _frameScale < _min_x)
      _min_x = r[0] - _frameScale;//x方向的边界
    if (r[1] - _frameScale < _min_y)
      _min_y = r[1] - _frameScale;//y方向的边界
    if (r[2] < _min_z)
      _min_z = r[2];//高度的边界
    if (r[0] + _frameScale > _max_x)
      _max_x = r[0] + _frameScale;
    if (r[1] + _frameScale > _max_y)
      _max_y = r[1] + _frameScale;
    if (r[2] > _max_z)
      _max_z = r[2];
    _scale = std::min(imageSize / (_max_x - _min_x), imageSize / (_max_y - _min_y));//图像与实际场景的放大倍数

    // draw it
    //等待showing_的信号，showing为原子变量；而且此处应该省略了{std::this_thread::yield();}
    //showing为true的时候，将可以将本线程的CPU时间片放弃，并允许其他线程(dispaly)运行, 停止绘制
    while (showing_) {
    }
    drawing_ = true;//drawing_为true的时候, 线程(display)放弃运行,停止显示
    // erase
    _image.setTo(cv::Scalar(10, 10, 10));//填充背景颜色
    drawPath();//绘制路径
    // draw axes
    //绘制坐标系
    Eigen::Vector3d e_x = C.col(0);
    Eigen::Vector3d e_y = C.col(1);
    Eigen::Vector3d e_z = C.col(2);
    //back返回当前vector容器中末尾元素的引用。
    cv::line(
        _image,
        convertToImageCoordinates(_path.back()),
        convertToImageCoordinates(
            _path.back() + cv::Point2d(e_x[0], e_x[1]) * _frameScale),
        cv::Scalar(0, 0, 255), 1, CV_AA);//画x轴，颜色为蓝色
    cv::line(
        _image,
        convertToImageCoordinates(_path.back()),
        convertToImageCoordinates(
            _path.back() + cv::Point2d(e_y[0], e_y[1]) * _frameScale),
        cv::Scalar(0, 255, 0), 1, CV_AA);//画y轴，颜色为绿
    cv::line(
        _image,
        convertToImageCoordinates(_path.back()),
        convertToImageCoordinates(
            _path.back() + cv::Point2d(e_z[0], e_z[1]) * _frameScale),
        cv::Scalar(255, 0, 0), 1, CV_AA);//画z轴，颜色为红色

    // some text:
    std::stringstream postext;
    //赋值位置信息
    postext << "position = [" << r[0] << ", " << r[1] << ", " << r[2] << "]";
    //在窗口的底栏显示
    cv::putText(_image, postext.str(), cv::Point(15,15),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1);
    std::stringstream veltext;
    //赋值速度信息
    veltext << "velocity = [" << speedAndBiases[0] << ", " << speedAndBiases[1] << ", " << speedAndBiases[2] << "]";
    cv::putText(_image, veltext.str(), cv::Point(15,35),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,255,255), 1);
    //drawing_为false,则线程display开始争抢当前线程publishFullStateAsCallback的时间,开启显示
    drawing_ = false; // notify
  }
  //展示函数
  void display()
  {
      //当drawing_为true时,将display线程的时间让给其他线程(绘图线程)
    while (drawing_) {
    }
    showing_ = true;//showing为真,意味着将线程publishFullStateAsCallback的时间让给当前线程(display), 不进行绘制
    cv::imshow("OKVIS Top View", _image);//绘制图像
    showing_ = false;//showing_为假意味着线程publishFullStateAsCallback开始争抢当前线程(display)的时间, 开始绘制
    cv::waitKey(1);//1s显示一次
  }
 private:
  //将公尺米为单位的坐标转化到像素坐标系中
  cv::Point2d convertToImageCoordinates(const cv::Point2d & pointInMeters) const
  {
    cv::Point2d pt = (pointInMeters - cv::Point2d(_min_x, _min_y)) * _scale;
    //将y方向反转，因为一般我们视角是从上向下看得
    return cv::Point2d(pt.x, imageSize - pt.y); // reverse y for more intuitive top-down plot
  }
  //路径绘制函数
  void drawPath()
  {
      //注意, 绘制的速度要落后一帧
    for (size_t i = 0; i + 1 < _path.size(); ) {
      cv::Point2d p0 = convertToImageCoordinates(_path[i]);
      cv::Point2d p1 = convertToImageCoordinates(_path[i + 1]);
      cv::Point2d diff = p1-p0;//两帧位姿间的平移向量
      //平移量过小
      if(diff.dot(diff)<2.0){
        _path.erase(_path.begin() + i + 1);  // clean short segment
        _heights.erase(_heights.begin() + i + 1);
        continue;
      }
      //两帧相对深度值,求平均,并求在最大高度范围内的比例
      double rel_height = (_heights[i] - _min_z + _heights[i + 1] - _min_z)
                      * 0.5 / (_max_z - _min_z);
      //难道是用颜色表示深度?
      cv::line(
          _image,
          p0,
          p1,
          rel_height * cv::Scalar(255, 0, 0)
              + (1.0 - rel_height) * cv::Scalar(0, 0, 255),
          1, CV_AA);
      i++;
    }
  }
  cv::Mat _image;
  std::vector<cv::Point2d> _path;
  std::vector<double> _heights;
  double _scale = 1.0;
  double _min_x = -0.5;
  double _min_y = -0.5;
  double _min_z = -0.5;
  double _max_x = 0.5;
  double _max_y = 0.5;
  double _max_z = 0.5;
  const double _frameScale = 0.2;  // [m]
  //如果我们在多个线程中对这些类型的共享资源进行操作，
  //编译器将保证这些操作都是原子性的，也就是说，确保任意时刻只有一个线程对这个资源进行访问，
  //编译器将保证，多个线程访问这个共享资源的正确性。从而避免了锁的使用，提高了效率。
  std::atomic_bool drawing_;
  std::atomic_bool showing_;
};

// this is just a workbench. most of the stuff here will go into the Frontend class.
int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);//argv[0]可执行程序的路径，比用其他方式方便的多
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  if (argc != 3 && argc != 4) {
    LOG(ERROR)<<
    "Usage: ./" << argv[0] << " configuration-yaml-file dataset-folder [skip-first-seconds]";
    return -1;
  }

  okvis::Duration deltaT(0.0);
  if (argc == 4) {
    deltaT = okvis::Duration(atof(argv[3]));//atof表示将字符串转换为浮点数,并在类Duration中将时间戳的前十位和后9位分开
  }

  // read configuration file
  std::string configFilename(argv[1]);//读取参数文件

  okvis::VioParametersReader vio_parameters_reader(configFilename);//读取参数文件到类中
  okvis::VioParameters parameters;
  vio_parameters_reader.getParameters(parameters);//将读取到的参数赋值给变量parameters

  okvis::ThreadedKFVio okvis_estimator(parameters);//用参数构造ThreadedKFVio类，成功后SLAM系统开始运行

  PoseViewer poseViewer;//定义画图工具
  ///bind表示绑定函数publishFullStateAsCallback, 其是poseViewer中的成员函数, 其余四个参数固定, 延迟到我们需要的时候调用
  okvis_estimator.setFullStateCallback(
      std::bind(&PoseViewer::publishFullStateAsCallback, &poseViewer,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4));

  okvis_estimator.setBlocking(true);//设置okvis_estimator类中的变量blocking_为真, 且执行时间限制优化

  // the folder path
  std::string path(argv[2]);//数据文件所在的路径

  const unsigned int numCameras = parameters.nCameraSystem.numCameras();//return The number of cameras.

  // open the IMU file
  std::string line;
  std::ifstream imu_file(path + "/imu0/data.csv");//IMU文件的路径
  //没有的话则直接检测错误
  if (!imu_file.good()) {
    LOG(ERROR)<< "no imu file found at " << path+"/imu0/data.csv";
    return -1;
  }
  int number_of_lines = 0;
  while (std::getline(imu_file, line))//一行一行的读取数据
    ++number_of_lines;//累积,记录总行数
  LOG(INFO)<< "No. IMU measurements: " << number_of_lines-1;//一共有number_of_lines-1行
  if (number_of_lines - 1 <= 0) {
    LOG(ERROR)<< "no imu messages present in " << path+"/imu0/data.csv";
    return -1;
  }
  // set reading position to second line
  imu_file.clear();//清楚eof的标记
  //C++中seekg函数的功能是：设置输入文件流的文件流指针位置
  imu_file.seekg(0, std::ios::beg);//输入流定位到流的开始位置
  std::getline(imu_file, line);//第一行,是菜单栏目

  std::vector<okvis::Time> times;//定义时间戳容器
  okvis::Time latest(0);
  int num_camera_images = 0;
  //image_names为一个2维的容器,每一维又是一个记录图像信息的容器
  std::vector < std::vector < std::string >> image_names(numCameras);
  for (size_t i = 0; i < numCameras; ++i) {
    num_camera_images = 0;
    std::string folder(path + "/cam" + std::to_string(i) + "/data");//储存图像信息的文件夹
    //遍历文件夹下所有的图像文件名
    for (auto it = boost::filesystem::directory_iterator(folder);
        it != boost::filesystem::directory_iterator(); it++) {
      if (!boost::filesystem::is_directory(it->path())) {  //we eliminate directories,如果该文件名对应的路径不是一个目录
        num_camera_images++;//图像数量加一
        image_names.at(i).push_back(it->path().filename().string());//储存文件名到容器image_names中
      } else {
        continue;//跳过目录
      }
    }

    if (num_camera_images == 0) {
      LOG(ERROR)<< "no images at " << folder;//数据集没有图像,直接跳过
      return 1;
    }

    LOG(INFO)<< "No. cam " << i << " images: " << num_camera_images;//第i个相机中有多少个图像
    // the filenames are not going to be sorted. So do this here
    std::sort(image_names.at(i).begin(), image_names.at(i).end());//对文件名按照时间顺序进行排序
  }

  std::vector < std::vector < std::string > ::iterator
      > cam_iterators(numCameras);//定义一个2维的容器,每一维都是一个迭代器
  for (size_t i = 0; i < numCameras; ++i) {
    cam_iterators.at(i) = image_names.at(i).begin();//每一维的迭代器都从第一帧开始
  }

  int counter = 0;
  okvis::Time start(0.0);//从0开始
  //开始程序运行函数
  while (true) {
      //窗口显示函数的调用
    okvis_estimator.display();
    poseViewer.display();

    // check if at the end
    for (size_t i = 0; i < numCameras; ++i) {
        //当迭代器迭代到了容器的最后
      if (cam_iterators[i] == image_names[i].end()) {
        std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
        cv::waitKey();
        return 0;
      }
    }

    /// add images
    okvis::Time t;

    for (size_t i = 0; i < numCameras; ++i) {
      cv::Mat filtered = cv::imread(
          path + "/cam" + std::to_string(i) + "/data/" + *cam_iterators.at(i),
          cv::IMREAD_GRAYSCALE);//读取图像到矩阵filtered中
      //读取小于秒单位(小数点后)的时间戳
      std::string nanoseconds = cam_iterators.at(i)->substr(
          cam_iterators.at(i)->size() - 13, 9);
      //读取以秒为单位的时间戳
      std::string seconds = cam_iterators.at(i)->substr(
          0, cam_iterators.at(i)->size() - 13);
      t = okvis::Time(std::stoi(seconds), std::stoi(nanoseconds));//stoi表示字符串转数值
      if (start == okvis::Time(0.0)) {
        start = t;//将初始第一帧的时间戳赋值给start
      }

      // get all IMU measurements till then
      //start为初始帧的时间戳
      okvis::Time t_imu = start;
      do {
          //IMU数据读取错误
        if (!std::getline(imu_file, line)) {
          std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
          cv::waitKey();
          return 0;
        }

        std::stringstream stream(line);//一行IMU数据
        std::string s;
        std::getline(stream, s, ',');//以","为分隔点读取一个数据(时间戳)
        std::string nanoseconds = s.substr(s.size() - 9, 9);//秒后的数据,(小数点后的时间)
        std::string seconds = s.substr(0, s.size() - 9);//秒单位的数据

        Eigen::Vector3d gyr;
        for (int j = 0; j < 3; ++j) {
          std::getline(stream, s, ',');
          gyr[j] = std::stof(s);//三维陀螺仪测量值
        }

        Eigen::Vector3d acc;
        for (int j = 0; j < 3; ++j) {
          std::getline(stream, s, ',');
          acc[j] = std::stof(s);//三维加速度计的值
        }

        t_imu = okvis::Time(std::stoi(seconds), std::stoi(nanoseconds));//将IMU的时间戳赋值给t_imu

        // add the IMU measurement for (blocking) processing
        //imu比初始帧图像早小于1s,或者imu比初始帧图像晚
        if (t_imu - start + okvis::Duration(1.0) > deltaT) {
          okvis_estimator.addImuMeasurement(t_imu, acc, gyr);//添加IMU数据到okvis_estimator中
        }

      } while (t_imu <= t);//当imu比当前帧晚,直接跳出循环(由于是左右图都进行一次,所以一般一帧图像会对应其前面临近几帧和后面两帧IMU)

      // add the image to the frontend for (blocking) processing
      //初始帧之后的所有帧都加入到okvis_estimator中
      if (t - start > deltaT) {
        okvis_estimator.addImage(t, i, filtered);
      }

      cam_iterators[i]++;//图像序列的迭代器加一, 即对应下一帧图像
    }
    ++counter;//计数运行到了第几帧

    // display progress
    //进行进度, (百分比)
    if (counter % 20 == 0) {
      std::cout << "\rProgress: "
          << int(double(counter) / double(num_camera_images) * 100) << "%  "
          << std::flush;
    }

  }

  std::cout << std::endl << std::flush;
  return 0;
}
