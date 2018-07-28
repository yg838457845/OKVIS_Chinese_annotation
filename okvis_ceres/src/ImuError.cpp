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
 *  Created on: Sep 3, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file ImuError.cpp
 * @brief Source file for the ImuError class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <thread>

#include <glog/logging.h>

#include <okvis/kinematics/operators.hpp>
#include <okvis/ceres/ImuError.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/assert_macros.hpp>
#include <okvis/ceres/ode/ode.hpp>
#include <okvis/kinematics/Transformation.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Construct with measurements and parameters.
/// 初始IMU的误差
/// imuMeasurements为从上一帧到当前帧之间所有的IMU观测值
/// imuParameters为IMU的参数信息
ImuError::ImuError(const okvis::ImuMeasurementDeque & imuMeasurements,
                   const okvis::ImuParameters & imuParameters,
                   const okvis::Time& t_0, const okvis::Time& t_1) {
  setImuMeasurements(imuMeasurements);
  setImuParameters(imuParameters);
  setT0(t_0);
  setT1(t_1);

  OKVIS_ASSERT_TRUE_DBG(Exception,
                     t_0 >= imuMeasurements.front().timeStamp,
                     "First IMU measurement included in ImuError is not old enough!");
  OKVIS_ASSERT_TRUE_DBG(Exception,
                     t_1 <= imuMeasurements.back().timeStamp,
                     "Last IMU measurement included in ImuError is not new enough!");
}

// Propagates pose, speeds and biases with given IMU measurements.
/// 预积分位置,速度和偏差
int ImuError::redoPreintegration(const okvis::kinematics::Transformation& /*T_WS*/,
                                 const okvis::SpeedAndBias & speedAndBiases) const {

  // ensure unique access
  std::lock_guard<std::mutex> lock(preintegrationMutex_);

  // now the propagation
  okvis::Time time = t0_;
  okvis::Time end = t1_;

  // sanity check:
  assert(imuMeasurements_.front().timeStamp<=time);
  ///要求IMU观测较多
  if (!(imuMeasurements_.back().timeStamp >= end))
    return -1;  // nothing to do...

  // increments (initialise with identity)
  Delta_q_ = Eigen::Quaterniond(1, 0, 0, 0);
  C_integral_ = Eigen::Matrix3d::Zero();
  C_doubleintegral_ = Eigen::Matrix3d::Zero();
  acc_integral_ = Eigen::Vector3d::Zero();
  acc_doubleintegral_ = Eigen::Vector3d::Zero();

  // cross matrix accumulatrion
  cross_ = Eigen::Matrix3d::Zero();

  // sub-Jacobians
  dalpha_db_g_ = Eigen::Matrix3d::Zero();
  dv_db_g_ = Eigen::Matrix3d::Zero();
  dp_db_g_ = Eigen::Matrix3d::Zero();

  // the Jacobian of the increment (w/o biases)
  P_delta_ = Eigen::Matrix<double, 15, 15>::Zero();

  //Eigen::Matrix<double, 15, 15> F_tot;
  //F_tot.setIdentity();

  double Delta_t = 0;
  bool hasStarted = false;
  int i = 0;
  //遍历IMU的观测
  for (okvis::ImuMeasurementDeque::const_iterator it = imuMeasurements_.begin();
      it != imuMeasurements_.end(); ++it) {

    Eigen::Vector3d omega_S_0 = it->measurement.gyroscopes;
    Eigen::Vector3d acc_S_0 = it->measurement.accelerometers;
    Eigen::Vector3d omega_S_1 = (it + 1)->measurement.gyroscopes;
    Eigen::Vector3d acc_S_1 = (it + 1)->measurement.accelerometers;

    // time delta
    okvis::Time nexttime;
    if ((it + 1) == imuMeasurements_.end()) {
      nexttime = t1_;
    } else
      nexttime = (it + 1)->timeStamp;
    double dt = (nexttime - time).toSec();

    if (end < nexttime) {
      double interval = (nexttime - it->timeStamp).toSec();
      nexttime = t1_;
      dt = (nexttime - time).toSec();
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();
    }

    if (dt <= 0.0) {
      continue;
    }
    Delta_t += dt;

    if (!hasStarted) {
      hasStarted = true;
      const double r = dt / (nexttime - it->timeStamp).toSec();
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();
    }

    // ensure integrity
    double sigma_g_c = imuParameters_.sigma_g_c;
    double sigma_a_c = imuParameters_.sigma_a_c;

    if (fabs(omega_S_0[0]) > imuParameters_.g_max
        || fabs(omega_S_0[1]) > imuParameters_.g_max
        || fabs(omega_S_0[2]) > imuParameters_.g_max
        || fabs(omega_S_1[0]) > imuParameters_.g_max
        || fabs(omega_S_1[1]) > imuParameters_.g_max
        || fabs(omega_S_1[2]) > imuParameters_.g_max) {
      sigma_g_c *= 100;
      LOG(WARNING)<< "gyr saturation";
    }

    if (fabs(acc_S_0[0]) > imuParameters_.a_max || fabs(acc_S_0[1]) > imuParameters_.a_max
        || fabs(acc_S_0[2]) > imuParameters_.a_max
        || fabs(acc_S_1[0]) > imuParameters_.a_max
        || fabs(acc_S_1[1]) > imuParameters_.a_max
        || fabs(acc_S_1[2]) > imuParameters_.a_max) {
      sigma_a_c *= 100;
      LOG(WARNING)<< "acc saturation";
    }

    // actual propagation
    // orientation:
    ///得到omega_S_0,omega_S_1,acc_S_0,acc_S_1
    Eigen::Quaterniond dq;
    const Eigen::Vector3d omega_S_true = (0.5 * (omega_S_0 + omega_S_1)
        - speedAndBiases.segment < 3 > (3));//计算平均的角速度
    const double theta_half = omega_S_true.norm() * 0.5 * dt;
    const double sinc_theta_half = ode::sinc(theta_half);
    const double cos_theta_half = cos(theta_half);
    dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;
    dq.w() = cos_theta_half;
    ///dq表示更新的角度,Delta_q_为初始的位姿,Delta_q_1为更新后的位姿
    Eigen::Quaterniond Delta_q_1 = Delta_q_ * dq;
    // rotation matrix integral:
    const Eigen::Matrix3d C = Delta_q_.toRotationMatrix();//前一个IMU数据对应的旋转矩阵
    const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();//后一个IMU数据对应的旋转矩阵
    const Eigen::Vector3d acc_S_true = (0.5 * (acc_S_0 + acc_S_1)
        - speedAndBiases.segment < 3 > (6));//计算平均的加速度
    const Eigen::Matrix3d C_integral_1 = C_integral_ + 0.5 * (C + C_1) * dt;//旋转矩阵的积分
    const Eigen::Vector3d acc_integral_1 = acc_integral_
        + 0.5 * (C + C_1) * acc_S_true * dt;//加速度的积分
    // rotation matrix double integral:
    C_doubleintegral_ += C_integral_ * dt + 0.25 * (C + C_1) * dt * dt;//旋转矩阵的二重积分
    acc_doubleintegral_ += acc_integral_ * dt
        + 0.25 * (C + C_1) * acc_S_true * dt * dt;//加速度的二重积分

    // Jacobian parts
    dalpha_db_g_ += C_1 * okvis::kinematics::rightJacobian(omega_S_true * dt) * dt;//角度对偏置的求导
    const Eigen::Matrix3d cross_1 = dq.inverse().toRotationMatrix() * cross_
        + okvis::kinematics::rightJacobian(omega_S_true * dt) * dt;
    const Eigen::Matrix3d acc_S_x = okvis::kinematics::crossMx(acc_S_true);
    Eigen::Matrix3d dv_db_g_1 = dv_db_g_
        + 0.5 * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);//速度对偏置的求导
    dp_db_g_ += dt * dv_db_g_
        + 0.25 * dt * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);//位置对偏置的求导

    // covariance propagation
    Eigen::Matrix<double, 15, 15> F_delta =
        Eigen::Matrix<double, 15, 15>::Identity();
    // transform
    F_delta.block<3, 3>(0, 3) = -okvis::kinematics::crossMx(
        acc_integral_ * dt + 0.25 * (C + C_1) * acc_S_true * dt * dt);
    F_delta.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
    F_delta.block<3, 3>(0, 9) = dt * dv_db_g_
        + 0.25 * dt * dt * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    F_delta.block<3, 3>(0, 12) = -C_integral_ * dt
        + 0.25 * (C + C_1) * dt * dt;
    F_delta.block<3, 3>(3, 9) = -dt * C_1;
    F_delta.block<3, 3>(6, 3) = -okvis::kinematics::crossMx(
        0.5 * (C + C_1) * acc_S_true * dt);
    F_delta.block<3, 3>(6, 9) = 0.5 * dt
        * (C * acc_S_x * cross_ + C_1 * acc_S_x * cross_1);
    F_delta.block<3, 3>(6, 12) = -0.5 * (C + C_1) * dt;
    P_delta_ = F_delta * P_delta_ * F_delta.transpose();
    // add noise. Note that transformations with rotation matrices can be ignored, since the noise is isotropic.
    //F_tot = F_delta*F_tot;
    const double sigma2_dalpha = dt * sigma_g_c
        * sigma_g_c;
    P_delta_(3, 3) += sigma2_dalpha;
    P_delta_(4, 4) += sigma2_dalpha;
    P_delta_(5, 5) += sigma2_dalpha;
    const double sigma2_v = dt * sigma_a_c * sigma_a_c;
    P_delta_(6, 6) += sigma2_v;
    P_delta_(7, 7) += sigma2_v;
    P_delta_(8, 8) += sigma2_v;
    const double sigma2_p = 0.5 * dt * dt * sigma2_v;
    P_delta_(0, 0) += sigma2_p;
    P_delta_(1, 1) += sigma2_p;
    P_delta_(2, 2) += sigma2_p;
    const double sigma2_b_g = dt * imuParameters_.sigma_gw_c * imuParameters_.sigma_gw_c;
    P_delta_(9, 9) += sigma2_b_g;
    P_delta_(10, 10) += sigma2_b_g;
    P_delta_(11, 11) += sigma2_b_g;
    const double sigma2_b_a = dt * imuParameters_.sigma_aw_c * imuParameters_.sigma_aw_c;
    P_delta_(12, 12) += sigma2_b_a;
    P_delta_(13, 13) += sigma2_b_a;
    P_delta_(14, 14) += sigma2_b_a;

    // memory shift
    Delta_q_ = Delta_q_1;//更新角度
    C_integral_ = C_integral_1;
    acc_integral_ = acc_integral_1;//速度增量的更新
    cross_ = cross_1;
    dv_db_g_ = dv_db_g_1;
    time = nexttime;

    ++i;
    //当更新到到了结束的阈值
    if (nexttime == t1_)
      break;

  }

  // store the reference (linearisation) point
  speedAndBiases_ref_ = speedAndBiases;//将初始的速度与偏置设置为参考

  // get the weighting:
  // enforce symmetric
  P_delta_ = 0.5 * P_delta_ + 0.5 * P_delta_.transpose().eval();//正交化

  // calculate inverse
  information_ = P_delta_.inverse();
  information_ = 0.5 * information_ + 0.5 * information_.transpose().eval();

  // square root
  Eigen::LLT<information_t> lltOfInformation(information_);
  squareRootInformation_ = lltOfInformation.matrixL().transpose();

  //std::cout << F_tot;

  return i;
}

// Propagates pose, speeds and biases with given IMU measurements.
/// 利用IMU观测值更新pose、speeds、biase。
/// 注意到这里propagation的covariance和jacobian均为0，仅仅用于预测，对特征点检测提供先验的T_WC：
/// t_start为上一帧的时间戳，t_end为当前帧的时间戳
/// T_WS、speedAndBiases为上一帧的位姿、速度和偏差
int ImuError::propagation(const okvis::ImuMeasurementDeque & imuMeasurements,
                          const okvis::ImuParameters & imuParams,
                          okvis::kinematics::Transformation& T_WS,
                          okvis::SpeedAndBias & speedAndBiases,
                          const okvis::Time & t_start,
                          const okvis::Time & t_end, covariance_t* covariance,
                          jacobian_t* jacobian) {

  // now the propagation
  okvis::Time time = t_start;
  okvis::Time end = t_end;

  // sanity check:
  assert(imuMeasurements.front().timeStamp<=time);//IMU观测数据集中第一个元素的时间戳小于开始的时间
  //需要IMU观测数据集中最后一个元素的时间戳大于结束的时间
  if (!(imuMeasurements.back().timeStamp >= end))
    return -1;  // nothing to do...

  // initial condition
  Eigen::Vector3d r_0 = T_WS.r();//初始的平移
  Eigen::Quaterniond q_WS_0 = T_WS.q();//初始的四元数
  Eigen::Matrix3d C_WS_0 = T_WS.C();//旋转矩阵

  // increments (initialise with identity)
  //初始化增量
  Eigen::Quaterniond Delta_q(1,0,0,0);
  Eigen::Matrix3d C_integral = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d C_doubleintegral = Eigen::Matrix3d::Zero();
  Eigen::Vector3d acc_integral = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_doubleintegral = Eigen::Vector3d::Zero();

  // cross matrix accumulatrion
  Eigen::Matrix3d cross = Eigen::Matrix3d::Zero();

  // sub-Jacobians
  //初始化雅各比矩阵
  Eigen::Matrix3d dalpha_db_g = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dv_db_g = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dp_db_g = Eigen::Matrix3d::Zero();

  // the Jacobian of the increment (w/o biases)
  //初始化雅各比矩阵的增量
  Eigen::Matrix<double,15,15> P_delta = Eigen::Matrix<double,15,15>::Zero();

  double Delta_t = 0;
  bool hasStarted = false;
  int i = 0;
  //遍历IMU观测值
  for (okvis::ImuMeasurementDeque::const_iterator it = imuMeasurements.begin();
        it != imuMeasurements.end(); ++it) {

    Eigen::Vector3d omega_S_0 = it->measurement.gyroscopes;//前一个IMU帧的角速度
    Eigen::Vector3d acc_S_0 = it->measurement.accelerometers;//前一个IMU帧的加速度
    Eigen::Vector3d omega_S_1 = (it + 1)->measurement.gyroscopes;//后一个IMU帧的角速度
    Eigen::Vector3d acc_S_1 = (it + 1)->measurement.accelerometers;//后一个IMU帧的加速度

    // time delta
    okvis::Time nexttime;
    if ((it + 1) == imuMeasurements.end()) {
      nexttime = t_end;//运行到最后一个IMU观测值，赋值nexttime为结束时间
    } else
      nexttime = (it + 1)->timeStamp;//没有运行到最后，赋值为后一帧的时间戳
    double dt = (nexttime - time).toSec();//后一IMU帧到开始时的时间间隔（第一次），之后表示临接IMU帧的时间段

    //满足该条件的是位于当前帧时间戳之后的IMU数据
    //当 end 小于 nexttime 时，end 处 IMU 的测量值通过插值得到
    if (end < nexttime) {
      double interval = (nexttime - it->timeStamp).toSec();//两帧间的时间间隔
      nexttime = t_end;//赋值nexttime为结束时间
      dt = (nexttime - time).toSec();//上一帧到结束的时间，变为秒为单位
      const double r = dt / interval;
      omega_S_1 = ((1.0 - r) * omega_S_0 + r * omega_S_1).eval();//插值计算t_end的角速度
      acc_S_1 = ((1.0 - r) * acc_S_0 + r * acc_S_1).eval();//插值计算t_end的加速度
    }
    //这个条件约束了在两个端点的中间选取IMU数据
    if (dt <= 0.0) {
      continue;
    }
    Delta_t += dt;//叠加
    //第一次
    if (!hasStarted) {
      hasStarted = true;//只运行一次
      const double r = dt / (nexttime - it->timeStamp).toSec();//插值参数
      omega_S_0 = (r * omega_S_0 + (1.0 - r) * omega_S_1).eval();//eval避免等号两边有一样类型的变量
      acc_S_0 = (r * acc_S_0 + (1.0 - r) * acc_S_1).eval();//插值计算start时的加速度
    }

    // ensure integrity
    //IMU参数
    double sigma_g_c = imuParams.sigma_g_c;///陀螺仪的噪声密度
    double sigma_a_c = imuParams.sigma_a_c;///加速度计的噪声密度
    //角速度大于陀螺仪饱和度，给警告
    if (fabs(omega_S_0[0]) > imuParams.g_max
        || fabs(omega_S_0[1]) > imuParams.g_max
        || fabs(omega_S_0[2]) > imuParams.g_max
        || fabs(omega_S_1[0]) > imuParams.g_max
        || fabs(omega_S_1[1]) > imuParams.g_max
        || fabs(omega_S_1[2]) > imuParams.g_max) {
      sigma_g_c *= 100;//陀螺仪的噪声密度乘100
      LOG(WARNING) << "gyr saturation";
    }
    //加速度大于加速度计饱和度，给警告
    if (fabs(acc_S_0[0]) > imuParams.a_max || fabs(acc_S_0[1]) > imuParams.a_max
        || fabs(acc_S_0[2]) > imuParams.a_max
        || fabs(acc_S_1[0]) > imuParams.a_max
        || fabs(acc_S_1[1]) > imuParams.a_max
        || fabs(acc_S_1[2]) > imuParams.a_max) {
      sigma_a_c *= 100;//加速度计的噪声密度乘100
      LOG(WARNING) << "acc saturation";
    }

    // actual propagation
    // orientation:
    //由角速度测量值和时间间隔积分得到四元数
    Eigen::Quaterniond dq;//定义四元数
    const Eigen::Vector3d omega_S_true = (0.5*(omega_S_0+omega_S_1) - speedAndBiases.segment<3>(3));//角速度设为时间 t0 和 t1 平均值减去偏差
    const double theta_half = omega_S_true.norm() * 0.5 * dt;//对（角速度*时间）求取模值
    const double sinc_theta_half = ode::sinc(theta_half);//计算四元数的虚部系数
    const double cos_theta_half = cos(theta_half);//计算四元数的实部
    dq.vec() = sinc_theta_half * omega_S_true * 0.5 * dt;//计算虚部
    dq.w() = cos_theta_half;//计算实部
    Eigen::Quaterniond Delta_q_1 = Delta_q * dq;//迭代更新四元数
    // rotation matrix integral:
    //利用加速度计计算
    const Eigen::Matrix3d C = Delta_q.toRotationMatrix();//更新前的四元数变为旋转矩阵，初始值为单位阵，表示没有（旋转的变化量）
    const Eigen::Matrix3d C_1 = Delta_q_1.toRotationMatrix();//更新后的四元数变为旋转矩阵，（旋转的变化量）
    const Eigen::Vector3d acc_S_true = (0.5*(acc_S_0+acc_S_1) - speedAndBiases.segment<3>(6));//加速度设为时间t0和t1平均值减去偏差
    const Eigen::Matrix3d C_integral_1 = C_integral + 0.5*(C + C_1)*dt;//旋转分量的积分
    const Eigen::Vector3d acc_integral_1 = acc_integral + 0.5*(C + C_1)*acc_S_true*dt;//加速度的积分
    // rotation matrix double integral:
    C_doubleintegral += C_integral*dt + 0.25*(C + C_1)*dt*dt;//二重积分
    acc_doubleintegral += acc_integral*dt + 0.25*(C + C_1)*acc_S_true*dt*dt;//加速度的二重积分

    // Jacobian parts
    //雅各比矩阵
    dalpha_db_g += dt*C_1;//欧拉角对角速度偏置的求导
    //cross好像是角度雅各比的积分~  像是旋转矩阵的累乘
    const Eigen::Matrix3d cross_1 = dq.inverse().toRotationMatrix()*cross +
        okvis::kinematics::rightJacobian(omega_S_true*dt)*dt;
    const Eigen::Matrix3d acc_S_x = okvis::kinematics::crossMx(acc_S_true);//crossMx求反对称矩阵
    Eigen::Matrix3d dv_db_g_1 = dv_db_g + 0.5*dt*(C*acc_S_x*cross + C_1*acc_S_x*cross_1);//计算速度对角速度偏置的求导
    dp_db_g += dt*dv_db_g + 0.25*dt*dt*(C*acc_S_x*cross + C_1*acc_S_x*cross_1);//计算位移对角速度偏置的求导

    // covariance propagation
    if (covariance) {
      Eigen::Matrix<double,15,15> F_delta = Eigen::Matrix<double,15,15>::Identity();
      // transform
      // F_delta为error(p,delta,v,bg,ba)对(p,delta,v)的雅各比,在dt上的积分
      F_delta.block<3,3>(0,3) = -okvis::kinematics::crossMx(acc_integral*dt + 0.25*(C + C_1)*acc_S_true*dt*dt);
      F_delta.block<3,3>(0,6) = Eigen::Matrix3d::Identity()*dt;
      F_delta.block<3,3>(0,9) = dt*dv_db_g + 0.25*dt*dt*(C*acc_S_x*cross + C_1*acc_S_x*cross_1);
      F_delta.block<3,3>(0,12) = -C_integral*dt + 0.25*(C + C_1)*dt*dt;
      F_delta.block<3,3>(3,9) = -dt*C_1;
      F_delta.block<3,3>(6,3) = -okvis::kinematics::crossMx(0.5*(C + C_1)*acc_S_true*dt);
      F_delta.block<3,3>(6,9) = 0.5*dt*(C*acc_S_x*cross + C_1*acc_S_x*cross_1);
      F_delta.block<3,3>(6,12) = -0.5*(C + C_1)*dt;
      P_delta = F_delta*P_delta*F_delta.transpose();
      // add noise. Note that transformations with rotation matrices can be ignored, since the noise is isotropic.
      //F_tot = F_delta*F_tot;
      const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
      P_delta(3,3) += sigma2_dalpha;
      P_delta(4,4) += sigma2_dalpha;
      P_delta(5,5) += sigma2_dalpha;
      const double sigma2_v = dt * sigma_a_c * imuParams.sigma_a_c;
      P_delta(6,6) += sigma2_v;
      P_delta(7,7) += sigma2_v;
      P_delta(8,8) += sigma2_v;
      const double sigma2_p = 0.5*dt*dt*sigma2_v;
      P_delta(0,0) += sigma2_p;
      P_delta(1,1) += sigma2_p;
      P_delta(2,2) += sigma2_p;
      const double sigma2_b_g = dt * imuParams.sigma_gw_c * imuParams.sigma_gw_c;
      P_delta(9,9)   += sigma2_b_g;
      P_delta(10,10) += sigma2_b_g;
      P_delta(11,11) += sigma2_b_g;
      const double sigma2_b_a = dt * imuParams.sigma_aw_c * imuParams.sigma_aw_c;
      P_delta(12,12) += sigma2_b_a;
      P_delta(13,13) += sigma2_b_a;
      P_delta(14,14) += sigma2_b_a;
    }

    // memory shift
    Delta_q = Delta_q_1;
    C_integral = C_integral_1;
    acc_integral = acc_integral_1;
    cross = cross_1;
    dv_db_g = dv_db_g_1;
    time = nexttime;

    ++i;

    if (nexttime == t_end)
      break;

  }

  // actual propagation output:
  //位姿和速度的迭代~
  const Eigen::Vector3d g_W = imuParams.g * Eigen::Vector3d(0, 0, 6371009).normalized();
  T_WS.set(r_0+speedAndBiases.head<3>()*Delta_t
             + C_WS_0*(acc_doubleintegral/*-C_doubleintegral*speedAndBiases.segment<3>(6)*/)
             - 0.5*g_W*Delta_t*Delta_t,
             q_WS_0*Delta_q);
  speedAndBiases.head<3>() += C_WS_0*(acc_integral/*-C_integral*speedAndBiases.segment<3>(6)*/)-g_W*Delta_t;

  // assign Jacobian, if requested
  if (jacobian) {
    Eigen::Matrix<double,15,15> & F = *jacobian;
    // F为error(p,delta,v,bg,ba)对(p,delta,v)的雅各比,在Delta_t上的积分
    F.setIdentity(); // holds for all states, including d/dalpha, d/db_g, d/db_a
    F.block<3,3>(0,3) = -okvis::kinematics::crossMx(C_WS_0*acc_doubleintegral);
    F.block<3,3>(0,6) = Eigen::Matrix3d::Identity()*Delta_t;
    F.block<3,3>(0,9) = C_WS_0*dp_db_g;
    F.block<3,3>(0,12) = -C_WS_0*C_doubleintegral;
    F.block<3,3>(3,9) = -C_WS_0*dalpha_db_g;
    F.block<3,3>(6,3) = -okvis::kinematics::crossMx(C_WS_0*acc_integral);
    F.block<3,3>(6,9) = C_WS_0*dv_db_g;
    F.block<3,3>(6,12) = -C_WS_0*C_integral;
  }

  // overall covariance, if requested
  if (covariance) {
    Eigen::Matrix<double,15,15> & P = *covariance;
    // transform from local increments to actual states
    Eigen::Matrix<double,15,15> T = Eigen::Matrix<double,15,15>::Identity();
    T.topLeftCorner<3,3>() = C_WS_0;
    T.block<3,3>(3,3) = C_WS_0;
    T.block<3,3>(6,6) = C_WS_0;
    P = T * P_delta * T.transpose();
  }
  return i;
}

// This evaluates the error term and additionally computes the Jacobians.
bool ImuError::Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
//计算误差式和计算最小雅各比矩阵
///parameters表示相对位姿(平移向量和四元数)
///residuals为残差
/// 雅各比矩阵
/// 最小雅各比矩阵
bool ImuError::EvaluateWithMinimalJacobians(double const* const * parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobiansMinimal) const {

  // get poses
  const okvis::kinematics::Transformation T_WS_0(
      Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]),
      Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]));

  const okvis::kinematics::Transformation T_WS_1(
      Eigen::Vector3d(parameters[2][0], parameters[2][1], parameters[2][2]),
      Eigen::Quaterniond(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]));

  // get speed and bias
  okvis::SpeedAndBias speedAndBiases_0;
  okvis::SpeedAndBias speedAndBiases_1;
  for (size_t i = 0; i < 9; ++i) {
    speedAndBiases_0[i] = parameters[1][i];
    speedAndBiases_1[i] = parameters[3][i];
  }

  // this will NOT be changed:
  const Eigen::Matrix3d C_WS_0 = T_WS_0.C();
  const Eigen::Matrix3d C_S0_W = C_WS_0.transpose();

  // call the propagation
  const double Delta_t = (t1_ - t0_).toSec();
  Eigen::Matrix<double, 6, 1> Delta_b;
  // ensure unique access
  {
    std::lock_guard<std::mutex> lock(preintegrationMutex_);
    Delta_b = speedAndBiases_0.tail<6>()
          - speedAndBiases_ref_.tail<6>();
  }
  redo_ = redo_ || (Delta_b.head<3>().norm() * Delta_t > 0.0001);//(角速度偏置*时间)的偏差过大
  if (redo_) {
    redoPreintegration(T_WS_0, speedAndBiases_0);
    redoCounter_++;
    Delta_b.setZero();
    redo_ = false;
    /*if (redoCounter_ > 1) {
      std::cout << "pre-integration no. " << redoCounter_ << std::endl;
    }*/
  }

  // actual propagation output:
  {
    std::lock_guard<std::mutex> lock(preintegrationMutex_); // this is a bit stupid, but shared read-locks only come in C++14
    const Eigen::Vector3d g_W = imuParameters_.g * Eigen::Vector3d(0, 0, 6371009).normalized();

    // assign Jacobian w.r.t. x0
    Eigen::Matrix<double,15,15> F0 =
        Eigen::Matrix<double,15,15>::Identity(); // holds for d/db_g, d/db_a
    const Eigen::Vector3d delta_p_est_W =
        T_WS_0.r() - T_WS_1.r() + speedAndBiases_0.head<3>()*Delta_t - 0.5*g_W*Delta_t*Delta_t;
    const Eigen::Vector3d delta_v_est_W =
        speedAndBiases_0.head<3>() - speedAndBiases_1.head<3>() - g_W*Delta_t;
    ///Dq相当于是Delta_q_利用偏置更新的值,Delta_q_表示角度四元数的增量
    const Eigen::Quaterniond Dq = okvis::kinematics::deltaQ(-dalpha_db_g_*Delta_b.head<3>())*Delta_q_;
    F0.block<3,3>(0,0) = C_S0_W;
    F0.block<3,3>(0,3) = C_S0_W * okvis::kinematics::crossMx(delta_p_est_W);
    F0.block<3,3>(0,6) = C_S0_W * Eigen::Matrix3d::Identity()*Delta_t;
    F0.block<3,3>(0,9) = dp_db_g_;
    F0.block<3,3>(0,12) = -C_doubleintegral_;
    F0.block<3,3>(3,3) = (okvis::kinematics::plus(Dq*T_WS_1.q().inverse()) *
        okvis::kinematics::oplus(T_WS_0.q())).topLeftCorner<3,3>();
    F0.block<3,3>(3,9) = (okvis::kinematics::oplus(T_WS_1.q().inverse()*T_WS_0.q())*
        okvis::kinematics::oplus(Dq)).topLeftCorner<3,3>()*(-dalpha_db_g_);
    F0.block<3,3>(6,3) = C_S0_W * okvis::kinematics::crossMx(delta_v_est_W);
    F0.block<3,3>(6,6) = C_S0_W;
    F0.block<3,3>(6,9) = dv_db_g_;
    F0.block<3,3>(6,12) = -C_integral_;

    // assign Jacobian w.r.t. x1
    Eigen::Matrix<double,15,15> F1 =
        -Eigen::Matrix<double,15,15>::Identity(); // holds for the biases
    F1.block<3,3>(0,0) = -C_S0_W;
    F1.block<3,3>(3,3) = -(okvis::kinematics::plus(Dq) *
        okvis::kinematics::oplus(T_WS_0.q()) *
        okvis::kinematics::plus(T_WS_1.q().inverse())).topLeftCorner<3,3>();
    F1.block<3,3>(6,6) = -C_S0_W;

    // the overall error vector
    Eigen::Matrix<double, 15, 1> error;
    error.segment<3>(0) =  C_S0_W * delta_p_est_W + acc_doubleintegral_ + F0.block<3,6>(0,9)*Delta_b;
    error.segment<3>(3) = 2*(Dq*(T_WS_1.q().inverse()*T_WS_0.q())).vec(); //2*T_WS_0.q()*Dq*T_WS_1.q().inverse();//
    error.segment<3>(6) = C_S0_W * delta_v_est_W + acc_integral_ + F0.block<3,6>(6,9)*Delta_b;
    error.tail<6>() = speedAndBiases_0.tail<6>() - speedAndBiases_1.tail<6>();

    // error weighting
    Eigen::Map<Eigen::Matrix<double, 15, 1> > weighted_error(residuals);
    weighted_error = squareRootInformation_ * error;//加权误差

    // get the Jacobians
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        // Jacobian w.r.t. minimal perturbance
        Eigen::Matrix<double, 15, 6> J0_minimal = squareRootInformation_
            * F0.block<15, 6>(0, 0);//残差对TWS0的偏导

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor> > J0(
            jacobians[0]);
        J0 = J0_minimal * J_lift;

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[0] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor> > J0_minimal_mapped(
                jacobiansMinimal[0]);
            J0_minimal_mapped = J0_minimal;//存到形参中
          }
        }
      }
      if (jacobians[1] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> > J1(
            jacobians[1]);
        J1 = squareRootInformation_ * F0.block<15, 9>(0, 6);//残差对速度与偏置0的偏导

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[1] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> > J1_minimal_mapped(
                jacobiansMinimal[1]);
            J1_minimal_mapped = J1;
          }
        }
      }
      if (jacobians[2] != NULL) {
        // Jacobian w.r.t. minimal perturbance
        Eigen::Matrix<double, 15, 6> J2_minimal = squareRootInformation_
                    * F1.block<15, 6>(0, 0);//残差对TWS1的偏导

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[2], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor> > J2(
            jacobians[2]);
        J2 = J2_minimal * J_lift;

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[2] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor> > J2_minimal_mapped(
                jacobiansMinimal[2]);
            J2_minimal_mapped = J2_minimal;//赋值到形参
          }
        }
      }
      if (jacobians[3] != NULL) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> > J3(jacobians[3]);
        J3 = squareRootInformation_ * F1.block<15, 9>(0, 6);//残差对速度与偏置1的偏导

        // if requested, provide minimal Jacobians
        if (jacobiansMinimal != NULL) {
          if (jacobiansMinimal[3] != NULL) {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor> > J3_minimal_mapped(
                jacobiansMinimal[3]);
            J3_minimal_mapped = J3;
          }
        }
      }
    }
  }
  return true;
}

}  // namespace ceres
}  // namespace okvis
