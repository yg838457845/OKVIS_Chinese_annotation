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
 *  Created on: Oct 18, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file ProbabilisticStereoTriangulator.cpp
 * @brief Source file for the ProbabilisticStereoTriangulator class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <okvis/triangulation/stereo_triangulation.hpp>
#include <okvis/triangulation/ProbabilisticStereoTriangulator.hpp>
#include <okvis/ceres/ReprojectionError.hpp>
#include <okvis/ceres/PoseError.hpp>
#include <okvis/ceres/PoseParameterBlock.hpp>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>

// cameras and distortions
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
namespace triangulation {

// Default constructor; make sure to call resetFrames before triangulation!
template<class CAMERA_GEOMETRY_T>
ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::ProbabilisticStereoTriangulator(
    double pixelSigma)
    : camIdA_(-1),
      camIdB_(-1) {
  // relative transformation - have a local copy
  T_AB_.setIdentity();
  // relative uncertainty - have a local copy
  UOplus_.setZero();

  sigmaRay_ = pixelSigma / 300.0;
}

// Constructor to set frames and relative transformation.
template<class CAMERA_GEOMETRY_T>
ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::ProbabilisticStereoTriangulator(
    std::shared_ptr<okvis::MultiFrame> frameA_ptr,
    std::shared_ptr<okvis::MultiFrame> frameB_ptr, size_t camIdA, size_t camIdB,
    const okvis::kinematics::Transformation& T_AB,
    const Eigen::Matrix<double, 6, 6>& UOplus, double pixelSigma)
    : frameA_(frameA_ptr),
      frameB_(frameB_ptr),
      camIdA_(camIdA),
      camIdB_(camIdB),
      T_AB_(T_AB),
      UOplus_(UOplus) {
  T_BA_ = T_AB_.inverse();
  // also do all backprojections
//	_frameA_ptr->computeAllBackProjections(false);
//	_frameB_ptr->computeAllBackProjections(false);
  // prepare the pose prior, since this will not change.
  ::okvis::ceres::PoseError poseError(T_AB_, UOplus_.inverse());
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> J_minimal;  // Jacobian
  Eigen::Matrix<double, 7, 7, Eigen::RowMajor> J;  // Jacobian
  poseA_ = ::okvis::ceres::PoseParameterBlock(
      okvis::kinematics::Transformation(), 0, okvis::Time(0));
  poseB_ = ::okvis::ceres::PoseParameterBlock(T_AB_, 0, okvis::Time(0));
  extrinsics_ = ::okvis::ceres::PoseParameterBlock(
      okvis::kinematics::Transformation(), 0, okvis::Time(0));
  double residuals[6];
  // evaluate to get the jacobian
  double* parameters = poseB_.parameters();//parameters为一个数组
  double* jacobians = J.data();
  double* jacobians_minimal = J_minimal.data();
  ///有一点很困惑~   parameters代表的位姿和poseError中的测量参数measurement是一模一样的
  /// residuals对parameters的雅各比矩阵(jacobians,jacobians_minimal)
  poseError.EvaluateWithMinimalJacobians(&parameters, &residuals[0], &jacobians,
                                         &jacobians_minimal);
  // prepare lhs of Gauss-Newton:
  H_.setZero();
  H_.topLeftCorner<6, 6>() = J_minimal.transpose() * J_minimal;

  sigmaRay_ = pixelSigma
      / std::min(
          frameA_->geometryAs<CAMERA_GEOMETRY_T>(camIdA_)->focalLengthU(),
          frameB_->geometryAs<CAMERA_GEOMETRY_T>(camIdB_)->focalLengthU());
}

// Reset frames and relative transformation.
//重置帧和相关位移
//CAMERA_GEOMETRY_T==okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion>
template<class CAMERA_GEOMETRY_T>
void ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::resetFrames(
    std::shared_ptr<okvis::MultiFrame> frameA_ptr,
    std::shared_ptr<okvis::MultiFrame> frameB_ptr, size_t camIdA, size_t camIdB,
    const okvis::kinematics::Transformation& T_AB,
    const Eigen::Matrix<double, 6, 6>& UOplus) {
  T_AB_ = T_AB;//B到A的位姿转移
  T_BA_ = T_AB_.inverse();//A到B的位姿转移

  frameA_ = frameA_ptr;//A帧
  frameB_ = frameB_ptr;//B帧
  camIdA_ = camIdA;//A相机(A帧中)
  camIdB_ = camIdB;//B相机(B帧中)

  UOplus_ = UOplus;
  // also do all backprojections
//	_frameA_ptr->computeAllBackProjections(false);
//	_frameB_ptr->computeAllBackProjections(false);
  // prepare the pose prior, since this will not change.
  ::okvis::ceres::PoseError poseError(T_AB_, UOplus_.inverse());//设置位姿误差,UOplus为信息矩阵
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> J_minimal;  // Jacobian
  Eigen::Matrix<double, 7, 7, Eigen::RowMajor> J;  // Jacobian
  //得到数据块A
  poseA_ = ::okvis::ceres::PoseParameterBlock(
      okvis::kinematics::Transformation(), 0, okvis::Time(0));
  //得到数据块B,以T_AB为B相对A的位姿
  poseB_ = ::okvis::ceres::PoseParameterBlock(T_AB_, 0, okvis::Time(0));
  extrinsics_ = ::okvis::ceres::PoseParameterBlock(
      okvis::kinematics::Transformation(), 0, okvis::Time(0));
  double residuals[6];
  // evaluate to get the jacobian
  double* parameters = poseB_.parameters();//得到位姿的参数,四元数的形式
  double* jacobians = J.data();//转换为数组jacobians
  double* jacobians_minimal = J_minimal.data();//转换为数组jacobians_minimal
  poseError.EvaluateWithMinimalJacobians(&parameters, &residuals[0], &jacobians,
                                         &jacobians_minimal);
  // prepare lhs of Gauss-Newton:
  H_.setZero();
  H_.topLeftCorner<6, 6>() = J_minimal.transpose() * J_minimal;//计算Hassens矩阵

  sigmaRay_ = 0.5
      / std::min(
          frameA_->geometryAs<CAMERA_GEOMETRY_T>(camIdA_)->focalLengthU(),
          frameB_->geometryAs<CAMERA_GEOMETRY_T>(camIdB_)->focalLengthU());

}

// Default destructor.
template<class CAMERA_GEOMETRY_T>
ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::~ProbabilisticStereoTriangulator() {
}

// Triangulation.
// 三角化（检查特征匹配的时候会用到）
template<class CAMERA_GEOMETRY_T>
bool ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::stereoTriangulate(
    size_t keypointIdxA, size_t keypointIdxB,
    Eigen::Vector4d& outHomogeneousPoint_A,
    bool & outCanBeInitializedInaccuarate,
    double sigmaRay) const {

  OKVIS_ASSERT_TRUE_DBG(Exception,frameA_&&frameB_,"initialize with frames before use!");

  // chose the source of uncertainty
  double sigmaR = sigmaRay;
  if (sigmaR == -1.0)
    sigmaR = sigmaRay_;

  // call triangulation
  bool isValid;
  bool isParallel;
  Eigen::Vector2d keypointCoordinatesA, keypointCoordinatesB;
  Eigen::Vector3d backProjectionDirectionA_inA, backProjectionDirectionB_inA;
  //提取A帧a相机第keypointIdxA个特征点的二维坐标：keypointCoordinatesA
  frameA_->getKeypoint(camIdA_, keypointIdxA, keypointCoordinatesA);
  //提取B帧b相机第keypointIdxB个特征点的二维坐标：keypointCoordinatesB
  frameB_->getKeypoint(camIdB_, keypointIdxB, keypointCoordinatesB);
  //将特征点投影到相机坐标系中（Z轴为1)
  frameA_->geometryAs<CAMERA_GEOMETRY_T>(camIdA_)->backProject(
      keypointCoordinatesA, &backProjectionDirectionA_inA);
  frameB_->geometryAs<CAMERA_GEOMETRY_T>(camIdB_)->backProject(
      keypointCoordinatesB, &backProjectionDirectionB_inA);  // direction in frame B
  //将B帧b相机坐标系中的点旋转到A帧a相机坐标系中（投影）
  backProjectionDirectionB_inA = T_AB_.C() * backProjectionDirectionB_inA;
  ///三角化，形参
  /// p1表示A帧中A系的原点
  /// e1表示特征点在A帧相机坐标系中的投影
  /// p2表示B帧原点在A系中的坐标
  /// e2表示特征点在B帧相机坐标系中的投影，然后旋转到A帧
  /// sigma为特征点的不确定度
  Eigen::Vector4d hpA = triangulateFast(
      Eigen::Vector3d(0, 0, 0),  // center of A in A coordinates (0,0,0)
      backProjectionDirectionA_inA.normalized(), T_AB_.r(),  // center of B in A coordinates
      backProjectionDirectionB_inA.normalized(), sigmaR, isValid, isParallel);
  outCanBeInitializedInaccuarate = !isParallel;//如果平行(满足条件的纯旋转-----不可逆)，则表示初始化地标点准确，如果不平行(存在一定程度的不确定---可逆)，则表示初始化地标点不精确
  //不有效
  if (!isValid) {
    return false;
  }

  // check reprojection:
  double errA, errB;
  //计算误差
  isValid = computeReprojectionError4(frameA_, camIdA_, keypointIdxA, hpA,
                                      errA);
  if (!isValid) {
    return false;
  }
  Eigen::Vector4d outHomogeneousPoint_B = T_BA_ * Eigen::Vector4d(hpA);//转移到B帧中
  //计算B帧中的误差
  if (!computeReprojectionError4(frameB_, camIdB_, keypointIdxB,
                                 outHomogeneousPoint_B, errB)) {
    isValid = false;
    return false;
  }
  //两个误差过大
  if (errA > 4.0 || errB > 4.0) {
    isValid = false;
  }

  // assign output
  // 特征点在A帧相机坐标系中的坐标（注意时归一化后的）
  outHomogeneousPoint_A = Eigen::Vector4d(hpA);

  return isValid;
}

// Triangulation.
template<class CAMERA_GEOMETRY_T>
bool ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::stereoTriangulate(
    size_t keypointIdxA, size_t keypointIdxB,
    Eigen::Vector4d& outHomogeneousPoint_A, Eigen::Matrix3d& outPointUOplus_A,
    bool& outCanBeInitialized, double sigmaRay) const {
  OKVIS_ASSERT_TRUE_DBG(Exception,frameA_&&frameB_,"initialize with frames before use!");

  // get the triangulation
  bool canBeInitialized;
  if (!stereoTriangulate(keypointIdxA, keypointIdxB, outHomogeneousPoint_A, canBeInitialized,
                         sigmaRay)){
    return false;
  }

  // and get the uncertainty /
  getUncertainty(keypointIdxA, keypointIdxB, outHomogeneousPoint_A,
                 outPointUOplus_A, outCanBeInitialized);
  outCanBeInitialized &= canBeInitialized; // be conservative -- if the initial one failed, the 2nd should, too...
  return true;
}

// Get triangulation uncertainty.
// 得到三角化的不确定度
template<class CAMERA_GEOMETRY_T>
void ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::getUncertainty(
    size_t keypointIdxA, size_t keypointIdxB,
    const Eigen::Vector4d& homogeneousPoint_A,
    Eigen::Matrix3d& outPointUOplus_A, bool& outCanBeInitialized) const {
  OKVIS_ASSERT_TRUE_DBG(Exception,frameA_&&frameB_,"initialize with frames before use!");

  // also get the point in the other coordinate representation
  //Eigen::Vector4d& homogeneousPoint_B=_T_BA*homogeneousPoint_A;
  Eigen::Vector4d hPA = homogeneousPoint_A;//匹配点在A相机坐标系中的齐次坐标(归一化后)

  // calculate point uncertainty by constructing the lhs of the Gauss-Newton equation system.
  // note: the transformation T_WA is assumed constant and identity w.l.o.g.
  Eigen::Matrix<double, 9, 9> H = H_;

  //	keypointA_t& kptA = _frameA_ptr->keypoint(keypointIdxA);
  //	keypointB_t& kptB = _frameB_ptr->keypoint(keypointIdxB);
  Eigen::Vector2d kptA, kptB;
  frameA_->getKeypoint(camIdA_, keypointIdxA, kptA);//获取匹配点坐标
  frameB_->getKeypoint(camIdB_, keypointIdxB, kptB);

  // assemble the stuff from the reprojection errors
  double keypointStdDev;
  frameA_->getKeypointSize(camIdA_, keypointIdxA, keypointStdDev);//A特征点的直径
  keypointStdDev = 0.8 * keypointStdDev / 12.0;//A特征点的标准差
  Eigen::Matrix2d inverseMeasurementCovariance = Eigen::Matrix2d::Identity()
      * (1.0 / (keypointStdDev * keypointStdDev));//A特征点的协方差矩阵
  ::okvis::ceres::ReprojectionError<CAMERA_GEOMETRY_T> reprojectionErrorA(
      frameA_->geometryAs<CAMERA_GEOMETRY_T>(camIdA_), 0, kptA,
      inverseMeasurementCovariance);//形参分别为相机模型,相机的id,特征观测值,特征点的信息矩阵
  //typename keypointA_t::measurement_t residualA;
  Eigen::Matrix<double, 2, 1> residualA;
  Eigen::Matrix<double, 2, 4, Eigen::RowMajor> J_hpA;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_hpA_min;
  double* jacobiansA[3];
  jacobiansA[0] = 0;  // do not calculate, T_WA is fixed identity transform
  jacobiansA[1] = J_hpA.data();
  jacobiansA[2] = 0;  // fixed extrinsics
  double* jacobiansA_min[3];
  jacobiansA_min[0] = 0;  // do not calculate, T_WA is fixed identity transform
  jacobiansA_min[1] = J_hpA_min.data();
  jacobiansA_min[2] = 0;  // fixed extrinsics
  const double* parametersA[3];
  //const double* test = _poseA.parameters();
  parametersA[0] = poseA_.parameters();//A帧的位姿四元数
  parametersA[1] = hPA.data();//地标点在A相机中的坐标(归一化)
  parametersA[2] = extrinsics_.parameters();//外参的参数
  reprojectionErrorA.EvaluateWithMinimalJacobians(parametersA, residualA.data(),
                                                  jacobiansA, jacobiansA_min);//表示偏差residualA对parametersA的雅各比,即jacobiansA和jacobiansA_min

  inverseMeasurementCovariance.setIdentity();
  frameB_->getKeypointSize(camIdB_, keypointIdxB, keypointStdDev);//提取B帧中特征点的直径
  keypointStdDev = 0.8 * keypointStdDev / 12.0;//标准差
  inverseMeasurementCovariance *= 1.0 / (keypointStdDev * keypointStdDev);//协方差矩阵

  ::okvis::ceres::ReprojectionError<CAMERA_GEOMETRY_T> reprojectionErrorB(
      frameB_->geometryAs<CAMERA_GEOMETRY_T>(camIdB_), 0, kptB,
      inverseMeasurementCovariance);//设置重投影误差模型
  Eigen::Matrix<double, 2, 1> residualB;
  Eigen::Matrix<double, 2, 7, Eigen::RowMajor> J_TB;
  Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J_TB_min;
  Eigen::Matrix<double, 2, 4, Eigen::RowMajor> J_hpB;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_hpB_min;
  double* jacobiansB[3];
  jacobiansB[0] = J_TB.data();
  jacobiansB[1] = J_hpB.data();
  jacobiansB[2] = 0;  // fixed extrinsics
  double* jacobiansB_min[3];
  jacobiansB_min[0] = J_TB_min.data();
  jacobiansB_min[1] = J_hpB_min.data();
  jacobiansB_min[2] = 0;  // fixed extrinsics
  const double* parametersB[3];
  parametersB[0] = poseB_.parameters();//B相对A的位姿
  parametersB[1] = hPA.data();//地标点在A相机坐标系中的位置
  parametersB[2] = extrinsics_.parameters();//外参
  reprojectionErrorB.EvaluateWithMinimalJacobians(parametersB, residualB.data(),
                                                  jacobiansB, jacobiansB_min);//计算重投影误差和雅各比矩阵

  // evaluate again closer:
  //目的是~ 将hPA点沿着AB连线的中线向下移动了0.8左右，不懂这里的目的
  hPA.head<3>() = 0.8 * (hPA.head<3>() - T_AB_.r() / 2.0 * hPA[3])
      + T_AB_.r() / 2.0 * hPA[3];
  reprojectionErrorB.EvaluateWithMinimalJacobians(parametersB, residualB.data(),
                                                  jacobiansB, jacobiansB_min);//在计算一遍雅各比矩阵
  //误差过大，则认为不能初始化
  if (residualB.transpose() * residualB < 4.0)
    outCanBeInitialized = false;
  else
    outCanBeInitialized = true;

  // now add to H:
  // 计算视觉部分的海瑟矩阵
  H.bottomRightCorner<3, 3>() += J_hpA_min.transpose() * J_hpA_min;
  H.topLeftCorner<6, 6>() += J_TB_min.transpose() * J_TB_min;
  H.topRightCorner<6, 3>() += J_TB_min.transpose() * J_hpB_min;
  H.bottomLeftCorner<3, 6>() += J_hpB_min.transpose() * J_TB_min;
  H.bottomRightCorner<3, 3>() += J_hpB_min.transpose() * J_hpB_min;

  // invert (if invertible) to get covariance:
  Eigen::Matrix<double, 9, 9> cov;
  //H帧不满秩
  if (H.colPivHouseholderQr().rank() < 9) {
    outCanBeInitialized = false;
    return;
  }
  //信息矩阵
  cov = H.inverse();  // FIXME: use the QR decomposition for this...
  outPointUOplus_A = cov.bottomRightCorner<3, 3>();//与残差关于特征点在A帧相机坐标系位置的海瑟矩阵有关
}

// Compute the reprojection error.
template<class CAMERA_GEOMETRY_T>
bool ProbabilisticStereoTriangulator<CAMERA_GEOMETRY_T>::computeReprojectionError4(
    const std::shared_ptr<okvis::MultiFrame>& frame, size_t camId,
    size_t keypointId, const Eigen::Vector4d& homogeneousPoint,
    double& outError) const {

  OKVIS_ASSERT_LT_DBG(Exception, keypointId, frame->numKeypoints(camId),
      "Index out of bounds");
  Eigen::Vector2d y;
  okvis::cameras::CameraBase::ProjectionStatus status = frame
      ->geometryAs<CAMERA_GEOMETRY_T>(camId)->projectHomogeneous(
      homogeneousPoint, &y);//投影到图像像素中
  if (status == okvis::cameras::CameraBase::ProjectionStatus::Successful) {
    Eigen::Vector2d k;
    Eigen::Matrix2d inverseCov = Eigen::Matrix2d::Identity();
    double keypointStdDev;
    frame->getKeypoint(camId, keypointId, k);
    frame->getKeypointSize(camId, keypointId, keypointStdDev);
    keypointStdDev = 0.8 * keypointStdDev / 12.0;//特征点的标准差
    inverseCov *= 1.0 / (keypointStdDev * keypointStdDev);//关于特征点x和y的协方差矩阵

    y -= k;//投影点和真实点的偏差
    outError = y.dot(inverseCov * y);//计算误差统计值
    return true;
  } else
    return false;
}

template class ProbabilisticStereoTriangulator<
    okvis::cameras::PinholeCamera<okvis::cameras::EquidistantDistortion> > ;
template class ProbabilisticStereoTriangulator<
    okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion> > ;
template class ProbabilisticStereoTriangulator<
    okvis::cameras::PinholeCamera<okvis::cameras::RadialTangentialDistortion8> > ;

}  // namespace triangulation
}  // namespace okvis
