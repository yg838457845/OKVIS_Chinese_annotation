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
 *  Created on: Sep 10, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file PoseError.cpp
 * @brief Source file for the PoseError class.
 * @author Stefan Leutenegger
 */

#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Construct with measurement and information matrix.
PoseError::PoseError(const okvis::kinematics::Transformation & measurement,
                     const Eigen::Matrix<double, 6, 6> & information) {
  setMeasurement(measurement);
  setInformation(information);
}

// Construct with measurement and variance.
PoseError::PoseError(const okvis::kinematics::Transformation & measurement,
                     double translationVariance, double rotationVariance) {
  setMeasurement(measurement);

  information_t information;//6*6大小的矩阵
  information.setZero();
  //初始化信息矩阵
  information.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 1.0
      / translationVariance;
  information.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * 1.0
      / rotationVariance;

  setInformation(information);
}

// Set the information.
void PoseError::setInformation(const information_t & information) {
  information_ = information;
  covariance_ = information.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  Eigen::LLT<information_t> lltOfInformation(information_);//对information_进行LLT分解
  squareRootInformation_ = lltOfInformation.matrixL().transpose();//squareRootInformation_即分解得到的L阵
}

// This evaluates the error term and additionally computes the Jacobians.
bool PoseError::Evaluate(double const* const * parameters, double* residuals,
                         double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
// 求取最小雅各比矩阵
///parameters表示位姿（四元数和平移）
/// residuals表示残差
/// jacobians表示雅各比矩阵
/// jacobiansMinimal表示最小雅各比矩阵
bool PoseError::EvaluateWithMinimalJacobians(double const* const * parameters,
                                             double* residuals,
                                             double** jacobians,
                                             double** jacobiansMinimal) const {

  // compute error
  // 将位姿转换为位姿矩阵
  okvis::kinematics::Transformation T_WS(
      Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]),
      Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4],
                         parameters[0][5]));
  // delta pose
  // 输入残差的位姿和函数形参位姿逆的乘积
  okvis::kinematics::Transformation dp = measurement_ * T_WS.inverse();
  // get the error
  Eigen::Matrix<double, 6, 1> error;
  const Eigen::Vector3d dtheta = 2 * dp.q().coeffs().head<3>();//选取dp中四元数的虚部
  error.head<3>() = measurement_.r() - T_WS.r();//平移的误差
  error.tail<3>() = dtheta;//旋转的误差

  // weigh it
  // 将数组residuals映射成一个6*1的矩阵
  Eigen::Map<Eigen::Matrix<double, 6, 1> > weighted_error(residuals);
  //计算加入权重后的误差，squareRootInformation_为LLT分解后的信息矩阵
  weighted_error = squareRootInformation_ * error;

  // compute Jacobian...
  // 计算雅各比矩阵
  if (jacobians != NULL) {
    if (jacobians[0] != NULL) {
      //行优先，将jacobians转换为6*7的矩阵
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor> > J0(
          jacobians[0]);
      Eigen::Matrix<double, 6, 6, Eigen::RowMajor> J0_minimal;
      J0_minimal.setIdentity();//单位化
      J0_minimal *= -1.0;//反向
      //plus表示将四元数转化为右乘矩阵，右下3*3子矩阵表示虚部的反对称阵+实部的对角阵
      J0_minimal.block<3, 3>(3, 3) = -okvis::kinematics::plus(dp.q())
          .topLeftCorner<3, 3>();
      //信息矩阵的L阵乘J0_minimal，eval函数防止混淆
      J0_minimal = (squareRootInformation_ * J0_minimal).eval();

      // pseudo inverse of the local parametrization Jacobian:
      Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
      //计算左雅各比矩阵
      PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

      // hallucinate Jacobian w.r.t. state
      // 计算误差对parameter的左乘雅各比矩阵
      J0 = J0_minimal * J_lift;

      if (jacobiansMinimal != NULL) {
        if (jacobiansMinimal[0] != NULL) {
          //将数组jacobiansMinimal的元素变为矩阵J0_minimal_mapped
          Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > J0_minimal_mapped(
              jacobiansMinimal[0]);
          J0_minimal_mapped = J0_minimal;
        }
      }
    }
  }

  return true;
}

}  // namespace ceres
}  // namespace okvis
