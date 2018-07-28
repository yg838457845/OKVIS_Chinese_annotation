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
 *  Created on: Sep 12, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file MarginalizationError.cpp
 * @brief Source file for the MarginalizationError class.
 * @author Stefan Leutenegger
 */

#include <functional>

#include <okvis/ceres/MarginalizationError.hpp>
#include <okvis/ceres/LocalParamizationAdditionalInterfaces.hpp>
#include <okvis/assert_macros.hpp>

//#define USE_NEW_LINEARIZATION_POINT

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

inline void conservativeResize(Eigen::MatrixXd& matrixXd, int rows, int cols) {
  Eigen::MatrixXd tmp(rows, cols);
  const int common_rows = std::min(rows, (int) matrixXd.rows());
  const int common_cols = std::min(cols, (int) matrixXd.cols());
  tmp.topLeftCorner(common_rows, common_cols) = matrixXd.topLeftCorner(
      common_rows, common_cols);
  matrixXd.swap(tmp);
}

inline void conservativeResize(Eigen::VectorXd& vectorXd, int size) {
  if (vectorXd.rows() == 1) {
    Eigen::VectorXd tmp(size);  //Eigen::VectorXd tmp = Eigen::VectorXd::Zero(size,Eigen::RowMajor);
    const int common_size = std::min((int) vectorXd.cols(), size);
    tmp.head(common_size) = vectorXd.head(common_size);
    vectorXd.swap(tmp);
  } else {
    Eigen::VectorXd tmp(size);  //Eigen::VectorXd tmp = Eigen::VectorXd::Zero(size);
    const int common_size = std::min((int) vectorXd.rows(), size);
    tmp.head(common_size) = vectorXd.head(common_size);
    vectorXd.swap(tmp);
  }
}

// Default constructor. Initialises a new okvis::ceres::Map.
MarginalizationError::MarginalizationError() {
  mapPtr_ = 0;
  denseIndices_ = 0;
  residualBlockId_ = 0;
  errorComputationValid_ = false;
}

// Default constructor from okvis::ceres::Map.
MarginalizationError::MarginalizationError(Map& map) {
  setMap(map);
  denseIndices_ = 0;
  residualBlockId_ = 0;
  errorComputationValid_ = false;
}

MarginalizationError::MarginalizationError(
    Map& map, std::vector< ::ceres::ResidualBlockId> & residualBlockIds) {
  setMap(map);
  denseIndices_ = 0;
  residualBlockId_ = 0;
  errorComputationValid_ = false;
  bool success = addResidualBlocks(residualBlockIds);
  OKVIS_ASSERT_TRUE(
      Exception,
      success,
      "residual blocks supplied or their connected parameter blocks were not properly added to the map");
}

// Set the underlying okvis::ceres::Map.
void MarginalizationError::setMap(Map& map) {
  mapPtr_ = &map;
  residualBlockId_ = 0;  // reset.
}

// Add some residuals to this marginalisation error. This means, they will get linearised.
bool MarginalizationError::addResidualBlocks(
    const std::vector< ::ceres::ResidualBlockId> & residualBlockIds,
    const std::vector<bool> & keepResidualBlocks) {
  // add one block after the other
  for (size_t i = 0; i < residualBlockIds.size(); ++i) {
    bool keep = false;
    if (keepResidualBlocks.size() == residualBlockIds.size()) {
      keep = keepResidualBlocks[i];
    }
    if (!addResidualBlock(residualBlockIds[i], keep))
      return false;
  }
  return true;
}

// Add some residuals to this marginalisation error. This means, they will get linearised.
/// 边缘化误差中添加残差块
/// 将残差residualBlockId对应的数据块添加进信息容器parameterBlockInfos_和parameterBlockId2parameterBlockInfoIdx_中
/// 计算残差residualBlockId与其数据块对应的雅各比,并利用雅各比计算海瑟矩阵,找到合适的位置插入H_,b_中
/// 如果keep为false,则从优化地图中移除该残差
bool MarginalizationError::addResidualBlock(
    ::ceres::ResidualBlockId residualBlockId, bool keep) {

  // get the residual block & check
  std::shared_ptr<ErrorInterface> errorInterfacePtr =
      mapPtr_->errorInterfacePtr(residualBlockId);//提取第residualBlockId个残差块的残差
  OKVIS_ASSERT_TRUE_DBG(Exception, errorInterfacePtr,
      "residual block id does not exist.");
  if (errorInterfacePtr == 0) {
    return false;
  }

  errorComputationValid_ = false;  // flag that the error computation is invalid

  // get the parameter blocks
  Map::ParameterBlockCollection parameters = mapPtr_->parameters(
      residualBlockId);//残差residualBlockId对应的数据块集合

  // insert into parameter block ordering book-keeping
  for (size_t i = 0; i < parameters.size(); ++i) {
    Map::ParameterBlockSpec parameterBlockSpec = parameters[i];//提取每一个（数据块id，数据块）

    // does it already exist as a parameter block connected?
    // parameterBlockId2parameterBlockInfoIdx_中储存了数据块在信息容器中的位置(数据块id,在信息容器中的位置)
    ParameterBlockInfo info;//数据块的信息
    std::map<uint64_t, size_t>::iterator it =
        parameterBlockId2parameterBlockInfoIdx_.find(parameterBlockSpec.first);
    //如果在parameterBlockId2parameterBlockInfoIdx_中没有搜索到数据块
    if (it == parameterBlockId2parameterBlockInfoIdx_.end()) {  // not found. add it.
        // let's see, if it is actually a landmark, because then it will go into the sparse part
      bool isLandmark = false;
      //判断数据块是不是地标点
      if (std::dynamic_pointer_cast<HomogeneousPointParameterBlock>(
          parameterBlockSpec.second) != 0) {
        isLandmark = true;
      }

      // resize equation system
      const size_t origSize = H_.cols();
      size_t additionalSize = 0;
      //判断数据块是否固定
      if (!parameterBlockSpec.second->fixed()) ////////DEBUG
        additionalSize = parameterBlockSpec.second->minimalDimension();//数据块的最小维数
      size_t denseSize = 0;
      //parameterBlockInfos_为数据块信息的集合
      //orderingIdx表示一个数据块在海瑟矩阵中起始位置的行(列)
      //denseIndices_表示一个数据块在信息容器中的位置
      if (denseIndices_ > 0)
        denseSize = parameterBlockInfos_.at(denseIndices_ - 1).orderingIdx
            + parameterBlockInfos_.at(denseIndices_ - 1).minimalDimension;
       ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /////////////////////////////计算H的维数和对H阵进行扩维和更新
      if(additionalSize>0) {
        //添加了数据块且不是地标点
        if (!isLandmark) {
          // insert
          // lhs
          Eigen::MatrixXd H01 = H_.topRightCorner(denseSize,
                                                  origSize - denseSize);//denseSize为左上角的维数
          Eigen::MatrixXd H10 = H_.bottomLeftCorner(origSize - denseSize,
                                                    denseSize);//
          Eigen::MatrixXd H11 = H_.bottomRightCorner(origSize - denseSize,
                                                     origSize - denseSize);
          // rhs
          Eigen::VectorXd b1 = b0_.tail(origSize - denseSize);//b1为b的后几位
          //conservativeResize为一个函数：
          //表示：重置H_,H_的维数为(origSize + additionalSize,origSize + additionalSize),且H_保留左上角的值
          conservativeResize(H_, origSize + additionalSize,
                             origSize + additionalSize);  // lhs
          //重置b0_的维数(origSize + additionalSize),且保留b0_上面的值
          conservativeResize(b0_, origSize + additionalSize);  // rhs
          //denseSize为左上角的维数
          H_.topRightCorner(denseSize, origSize - denseSize) = H01;//赋值右上角
          H_.bottomLeftCorner(origSize - denseSize, denseSize) = H10;//赋值左下角
          H_.bottomRightCorner(origSize - denseSize, origSize - denseSize) = H11;//赋值右下角
          H_.block(0, denseSize, H_.rows(), additionalSize).setZero();//起始于(0, denseSize),块的大小(H_.rows(), additionalSize)
          H_.block(denseSize, 0, additionalSize, H_.rows()).setZero();//起始于(denseSize, 0),块的大小(additionalSize, H_.rows())

          b0_.tail(origSize - denseSize) = b1;//b0的尾部
          b0_.segment(denseSize, additionalSize).setZero();//起始于denseSize,维数大小为additionalSize
        } else {
          //重置H_的维数为(origSize + additionalSize,origSize + additionalSize)
          conservativeResize(H_, origSize + additionalSize,
                             origSize + additionalSize);  // lhs
          //重置b0_的维数为origSize + additionalSize
          conservativeResize(b0_, origSize + additionalSize);  // rhs
          // just append
          b0_.tail(additionalSize).setZero();//b0_的大小为additionalSize的尾部为0
          H_.bottomRightCorner(H_.rows(), additionalSize).setZero();//纵向右下角为0
          H_.bottomRightCorner(additionalSize, H_.rows()).setZero();//横向右下角为0
        }
      }

      // update book-keeping
      //不是地标点
      if (!isLandmark) {
        //利用(数据块的id,数据块,denseSize,是否地标点)对数据块的信息进行赋值
        //denseSize为对应数据块在H阵中的起点
        info = ParameterBlockInfo(parameterBlockSpec.first,
                                  parameterBlockSpec.second, denseSize,
                                  isLandmark);
        parameterBlockInfos_.insert(
            parameterBlockInfos_.begin() + denseIndices_, info);//将数据块信息插入信息容器中
        //parameterBlockId2parameterBlockInfoIdx_中储存(数据块id,数据块的状态在信息容器中的位置)
        parameterBlockId2parameterBlockInfoIdx_.insert(
            std::pair<uint64_t, size_t>(parameterBlockSpec.first,
                                        denseIndices_));

        //  update base_t book-keeping
        base_t::mutable_parameter_block_sizes()->insert(
            base_t::mutable_parameter_block_sizes()->begin() + denseIndices_,
            info.dimension);//插入数据块的维度
        //denseIndices_表示当前数据块在信息容器中的位置
        denseIndices_++;  // remember we increased the dense part of the problem

        // also increase the rest
        // 遍历后面的数据块信息,因为新插入了一个信息,所以后面的信息的位置都加一
        for (size_t j = denseIndices_; j < parameterBlockInfos_.size(); ++j) {
          parameterBlockInfos_.at(j).orderingIdx += additionalSize;//orderingIdx应该为数据块在H阵中的起始位置
          ///parameterBlockInfos_.at(j)表示提取第j个数据块信息
          /// ->id()表示第j个数据块信心对应数据块的id
          /// 最终含义时是数据块信息在信息容器中对应的位置加一
          parameterBlockId2parameterBlockInfoIdx_[parameterBlockInfos_.at(j)
              .parameterBlockPtr->id()] += 1;//后续数据块的denseIndices_值加一
        }
      } else {
        // just add at the end
        // 是地标点
        info = ParameterBlockInfo(
            parameterBlockSpec.first,
            parameterBlockSpec.second,
            parameterBlockInfos_.back().orderingIdx
                + parameterBlockInfos_.back().minimalDimension,
            isLandmark);//初始化地标点信息
        parameterBlockInfos_.push_back(info);//添加地标点
        //parameterBlockId2parameterBlockInfoIdx_中插入(地标点序号,其状态信息的位置)
        parameterBlockId2parameterBlockInfoIdx_.insert(
            std::pair<uint64_t, size_t>(parameterBlockSpec.first,
                                        parameterBlockInfos_.size() - 1));

        //  update base_t book-keeping
        base_t::mutable_parameter_block_sizes()->push_back(info.dimension);//添加地标点的维度信息
      }
      assert(
          parameterBlockInfos_[parameterBlockId2parameterBlockInfoIdx_[parameterBlockSpec
              .first]].parameterBlockId == parameterBlockSpec.first);//验证状态信息向量中的数据块id和原本数据块id相等
    } else {

#ifdef USE_NEW_LINEARIZATION_POINT
      // switch linearization point - easy to do on the linearized part...
      // 表示信息向量中已经有了当前数据块(表示该数据块多了一个对应的残差)
      /// 姑且认为数据块parameterBlockPtr是会变的,所以当新的一个一样的数据块来临时,它与原始的linearizationPoint会产生误差
      size_t i = it->second;//i表示当前数据块在信息向量中的位置
      Eigen::VectorXd Delta_Chi_i(parameterBlockInfos_[i].minimalDimension);//以数据块的维数创建一个向量
      parameterBlockInfos_[i].parameterBlockPtr->minus(
          parameterBlockInfos_[i].linearizationPoint.get(),
          parameterBlockInfos_[i].parameterBlockPtr->parameters(),
          Delta_Chi_i.data());
      //更新b0
      b0_ -=
      H_.block(0,parameterBlockInfos_[i].orderingIdx,H_.rows(),parameterBlockInfos_[i].minimalDimension)*
      Delta_Chi_i;
      //因为是重新输入了一个和之前一样的数据块,所以重新更新数据块信息
      parameterBlockInfos_[i].resetLinearizationPoint( parameterBlockInfos_[i].parameterBlockPtr);//将数据块的数据赋值给线性状态
#endif
      info = parameterBlockInfos_.at(it->second);//得到该数据块对应的信息
    }
  }

  // update base_t book-keeping on residuals
  base_t::set_num_residuals(H_.cols());//设置X的维数(HX=b)

  double** parametersRaw = new double*[parameters.size()];//待添加残差对应的数据块数量
  Eigen::VectorXd residualsEigen(errorInterfacePtr->residualDim());//定义一个残差维数的向量
  double* residualsRaw = residualsEigen.data();//将指针residualsRaw与向量residualsEigen关联

  double** jacobiansRaw = new double*[parameters.size()];//定义二维向量,维数和数据块个数相同
  std::vector<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
      Eigen::aligned_allocator<
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > > jacobiansEigen(
      parameters.size());//雅各比矩阵,一个维数和数据块个数相同的容器

  double** jacobiansMinimalRaw = new double*[parameters.size()];
  std::vector<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
      Eigen::aligned_allocator<
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > > jacobiansMinimalEigen(
      parameters.size());//最小雅各比矩阵,一个维数和数据块个数相同的容器

  for (size_t i = 0; i < parameters.size(); ++i) {
    OKVIS_ASSERT_TRUE_DBG(Exception, isParameterBlockConnected(parameters[i].first),
        "okvis bug: no linearization point, since not connected.");
    //数据块的线性(原始)数据
    parametersRaw[i] =
        parameterBlockInfos_[parameterBlockId2parameterBlockInfoIdx_[parameters[i]
            .first]].linearizationPoint.get();  // first estimate Jacobian!!

    jacobiansEigen[i].resize(errorInterfacePtr->residualDim(),
                             parameters[i].second->dimension());//第i个数据块的雅各比矩阵,矩阵的行为残差的维度,矩阵的列为数据块的维度
    jacobiansRaw[i] = jacobiansEigen[i].data();//矩阵转指针
    jacobiansMinimalEigen[i].resize(errorInterfacePtr->residualDim(),
                                    parameters[i].second->minimalDimension());//第i个数据块的最小雅各比矩阵,矩阵的行为残差的维度,矩阵的列为数据块的最小维度
    jacobiansMinimalRaw[i] = jacobiansMinimalEigen[i].data();//矩阵转指针
  }

  // evaluate residual block
  //计算residualsRaw对parametersRaw的雅各比矩阵
  errorInterfacePtr->EvaluateWithMinimalJacobians(parametersRaw, residualsRaw,
                                                  jacobiansRaw,
                                                  jacobiansMinimalRaw);


  // correct for loss function if applicable
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  //这个玩意应该是核函数
  //利用核函数修正雅各比矩阵和误差项
  ::ceres::LossFunction* lossFunction = mapPtr_
      ->residualBlockId2ResidualBlockSpecMap().find(residualBlockId)->second
      .lossFunctionPtr;//提取残差的损失函数
  if (lossFunction) {
    OKVIS_ASSERT_TRUE_DBG(
        Exception,
        mapPtr_->residualBlockId2ResidualBlockSpecMap().find(residualBlockId)
        != mapPtr_->residualBlockId2ResidualBlockSpecMap().end(),
        "???");

    // following ceres in internal/ceres/corrector.cc
    const double sq_norm = residualsEigen.transpose() * residualsEigen;//残差的模值
    double rho[3];
    lossFunction->Evaluate(sq_norm, rho);//rho中储存了(value,一阶导,二阶导)
    const double sqrt_rho1 = sqrt(rho[1]);
    double residual_scaling;
    double alpha_sq_norm;
    if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
      residual_scaling = sqrt_rho1;
      alpha_sq_norm = 0.0;

    } else {
      // Calculate the smaller of the two solutions to the equation
      //
      // 0.5 *  alpha^2 - alpha - rho'' / rho' *  z'z = 0.
      //
      // Start by calculating the discriminant D.
      const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];

      // Since both rho[1] and rho[2] are guaranteed to be positive at
      // this point, we know that D > 1.0.

      const double alpha = 1.0 - sqrt(D);
      OKVIS_ASSERT_TRUE_DBG(Exception, !std::isnan(alpha), "??");

      // Calculate the constants needed by the correction routines.
      residual_scaling = sqrt_rho1 / (1 - alpha);
      alpha_sq_norm = alpha / sq_norm;
    }

    // correct Jacobians (Equation 11 in BANS)
    for (size_t i = 0; i < parameters.size(); ++i) {
      jacobiansMinimalEigen[i] = sqrt_rho1
          * (jacobiansMinimalEigen[i]
              - alpha_sq_norm * residualsEigen
                  * (residualsEigen.transpose() * jacobiansMinimalEigen[i]));
    }

    // correct residuals (caution: must be after "correct Jacobians"):
    residualsEigen *= residual_scaling;
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // add blocks to lhs and rhs
  for (size_t i = 0; i < parameters.size(); ++i) {
    Map::ParameterBlockSpec parameterBlockSpec = parameters[i];//(第i个数据块的序号,第i个数据块)

    ParameterBlockInfo parameterBlockInfo_i = parameterBlockInfos_.at(
        parameterBlockId2parameterBlockInfoIdx_[parameters[i].first]);//提取数据块的信息

    OKVIS_ASSERT_TRUE_DBG(
        Exception,
        parameterBlockInfo_i.parameterBlockId == parameters[i].second->id(),
        "okvis bug: inconstistent okvis ordering");
    //数据块信息的维数为0,则直接跳过
    if (parameterBlockInfo_i.minimalDimension == 0)
      continue;

    OKVIS_ASSERT_TRUE_DBG(Exception,H_.allFinite(),"WTF1");
    //H中第i个数据块和自身的海瑟矩阵(i数据块~i数据块)
    H_.block(parameterBlockInfo_i.orderingIdx, parameterBlockInfo_i.orderingIdx,
             parameterBlockInfo_i.minimalDimension,
             parameterBlockInfo_i.minimalDimension) += jacobiansMinimalEigen.at(
        i).transpose().eval() * jacobiansMinimalEigen.at(i);
    b0_.segment(parameterBlockInfo_i.orderingIdx,
                parameterBlockInfo_i.minimalDimension) -= jacobiansMinimalEigen
        .at(i).transpose().eval() * residualsEigen;//b中第i个数据块的值

    OKVIS_ASSERT_TRUE_DBG(Exception,H_.allFinite(),
                      "WTF2 " <<jacobiansMinimalEigen.at(i).transpose().eval() * jacobiansMinimalEigen.at(i));
    //第i个数据块之前的数据块j
    for (size_t j = 0; j < i; ++j) {
      ParameterBlockInfo parameterBlockInfo_j = parameterBlockInfos_.at(
          parameterBlockId2parameterBlockInfoIdx_[parameters[j].first]);//提取第j个数据块对应的信息

      OKVIS_ASSERT_TRUE_DBG(
          Exception,
          parameterBlockInfo_j.parameterBlockId == parameters[j].second->id(),
          "okvis bug: inconstistent okvis ordering");
      //维数为0直接跳过
      if (parameterBlockInfo_j.minimalDimension == 0)
        continue;

      // upper triangular:
      // 数据块(i,j)对应的海瑟矩阵块
      H_.block(parameterBlockInfo_i.orderingIdx,
               parameterBlockInfo_j.orderingIdx,
               parameterBlockInfo_i.minimalDimension,
               parameterBlockInfo_j.minimalDimension) += jacobiansMinimalEigen
          .at(i).transpose().eval() * jacobiansMinimalEigen.at(j);
      // lower triangular:
      // 数据块(j,i)对应的海瑟矩阵块
      H_.block(parameterBlockInfo_j.orderingIdx,
               parameterBlockInfo_i.orderingIdx,
               parameterBlockInfo_j.minimalDimension,
               parameterBlockInfo_i.minimalDimension) += jacobiansMinimalEigen
          .at(j).transpose().eval() * jacobiansMinimalEigen.at(i);
    }
  }

  // finally, we also have to delete the nonlinear residual block from the map:
  // 从优化地图中剔除残差块
  if (!keep) {
    mapPtr_->removeResidualBlock(residualBlockId);
  }

  // cleanup temporarily allocated stuff
  delete[] parametersRaw;
  delete[] jacobiansRaw;
  delete[] jacobiansMinimalRaw;

  check();

  return true;
}

// Info: is this parameter block connected to this marginalization error?
bool MarginalizationError::isParameterBlockConnected(
    uint64_t parameterBlockId) {
  OKVIS_ASSERT_TRUE_DBG(Exception, mapPtr_->parameterBlockExists(parameterBlockId),
      "this parameter block does not even exist in the map...");
  std::map<uint64_t, size_t>::iterator it =
      parameterBlockId2parameterBlockInfoIdx_.find(parameterBlockId);
  if (it == parameterBlockId2parameterBlockInfoIdx_.end())
    return false;
  else
    return true;
}

// Checks the internal datastructure (debug)
/// 检查数据结构
void MarginalizationError::check() {
// check basic sizes
  OKVIS_ASSERT_TRUE_DBG(
      Exception,
      base_t::parameter_block_sizes().size()==parameterBlockInfos_.size(),
      "check failed"); OKVIS_ASSERT_TRUE_DBG(
      Exception,
      parameterBlockId2parameterBlockInfoIdx_.size()==parameterBlockInfos_.size(),
      "check failed"); OKVIS_ASSERT_TRUE_DBG(Exception, base_t::num_residuals()==H_.cols(), "check failed"); OKVIS_ASSERT_TRUE_DBG(Exception, base_t::num_residuals()==H_.rows(), "check failed"); OKVIS_ASSERT_TRUE_DBG(Exception, base_t::num_residuals()==b0_.rows(),
      "check failed"); OKVIS_ASSERT_TRUE_DBG(Exception, parameterBlockInfos_.size()>=denseIndices_,
      "check failed");
  int totalsize = 0;
  // check parameter block sizes
  // 遍历数据块信息容器
  for (size_t i = 0; i < parameterBlockInfos_.size(); ++i) {
    totalsize += parameterBlockInfos_[i].minimalDimension;//数据块的维度
    OKVIS_ASSERT_TRUE_DBG(
        Exception,
        //数据块维度==数据块的维度
        parameterBlockInfos_[i].dimension==size_t(base_t::parameter_block_sizes()[i]),
        "check failed"); OKVIS_ASSERT_TRUE_DBG(
        Exception,
        //地图中数据块存在
        mapPtr_->parameterBlockExists(parameterBlockInfos_[i].parameterBlockId),
        "check failed"); OKVIS_ASSERT_TRUE_DBG(
        Exception,
        //信息容器中存在数据块
        parameterBlockId2parameterBlockInfoIdx_[parameterBlockInfos_[i].parameterBlockId]==i,
        "check failed");
    if (i < denseIndices_) {
      OKVIS_ASSERT_TRUE_DBG(Exception, !parameterBlockInfos_[i].isLandmark,
          "check failed");
    } else {
      OKVIS_ASSERT_TRUE_DBG(Exception, parameterBlockInfos_[i].isLandmark,
                        "check failed");
    }

  }
  // check contiguous
  for (size_t i = 1; i < parameterBlockInfos_.size(); ++i) {
    OKVIS_ASSERT_TRUE_DBG(
        Exception,
        parameterBlockInfos_[i-1].orderingIdx+parameterBlockInfos_[i-1].minimalDimension==parameterBlockInfos_[i].orderingIdx,
        "check failed "<<parameterBlockInfos_[i-1].orderingIdx<<"+"<<parameterBlockInfos_[i-1].minimalDimension<<"=="<<parameterBlockInfos_[i].orderingIdx);
  }
// check dimension again
  OKVIS_ASSERT_TRUE_DBG(Exception, base_t::num_residuals()==totalsize, "check failed");
}

// Call this in order to (re-)add this error term after whenever it had been modified.
/// 得到数据块~
void MarginalizationError::getParameterBlockPtrs(
    std::vector<std::shared_ptr<okvis::ceres::ParameterBlock> >& parameterBlockPtrs) {
//OKVIS_ASSERT_TRUE_DBG(Exception,_errorComputationValid,"Call updateErrorComputation() before addToMap!");
  OKVIS_ASSERT_TRUE_DBG(Exception, mapPtr_!=0, "no Map object passed ever!");
  for (size_t i = 0; i < parameterBlockInfos_.size(); ++i) {
    parameterBlockPtrs.push_back(parameterBlockInfos_[i].parameterBlockPtr);//提取剩余数据块
  }
}

// Marginalise out a set of parameter blocks.
/// 边缘化所有的数据块
/// 通过shur消元更新H_和b_矩阵
/// 从parameterBlockInfos_和parameterBlockId2parameterBlockInfoIdx_删除待边缘化数据块的元素
bool MarginalizationError::marginalizeOut(
    const std::vector<uint64_t>& parameterBlockIds,
    const std::vector<bool> & keepParameterBlocks) {
  if (parameterBlockIds.size() == 0) {
    return false;
  }

  // copy so we can manipulate
  // 待边缘化数据块id的集合的副本
  std::vector<uint64_t> parameterBlockIdsCopy = parameterBlockIds;
  if (parameterBlockIds.size() != keepParameterBlocks.size()) {
    OKVIS_ASSERT_TRUE_DBG(
        Exception,
        keepParameterBlocks.size() == 0,
        "input vectors must either be of same size or omit optional parameter keepParameterBlocks: "<<
        parameterBlockIds.size()<<" vs "<<keepParameterBlocks.size());
  }
  //添加（数据块序号、keep）到keepParameterBlocksCopy中
  std::map<uint64_t, bool> keepParameterBlocksCopy;
  for (size_t i = 0; i < parameterBlockIdsCopy.size(); ++i) {
    bool keep = false;
    if (i < keepParameterBlocks.size()) {
      keep = keepParameterBlocks.at(i);
    }
    keepParameterBlocksCopy.insert(
        std::pair<uint64_t, bool>(parameterBlockIdsCopy.at(i), keep));
  }

  /* figure out which blocks need to be marginalized out */
  std::vector<std::pair<int, int> > marginalizationStartIdxAndLengthPairslandmarks;
  std::vector<std::pair<int, int> > marginalizationStartIdxAndLengthPairsDense;
  size_t marginalizationParametersLandmarks = 0;
  size_t marginalizationParametersDense = 0;

  // make sure no duplications...
  std::sort(parameterBlockIdsCopy.begin(), parameterBlockIdsCopy.end());//排序（升序），数据块序号的从小到大排序
  //parameterBlockIdsCopy为数据块序号（升序）的集合
  for (size_t i = 1; i < parameterBlockIdsCopy.size(); ++i) {
    if (parameterBlockIdsCopy[i] == parameterBlockIdsCopy[i - 1]) {
      parameterBlockIdsCopy.erase(parameterBlockIdsCopy.begin() + i);//删除相同的数据块
      --i;
    }
  }
  for (size_t i = 0; i < parameterBlockIdsCopy.size(); ++i) {
    std::map<uint64_t, size_t>::iterator it =
        parameterBlockId2parameterBlockInfoIdx_.find(parameterBlockIdsCopy[i]);//寻找数据块对应的数据块信息的位置（在parameterBlockInfos_中的位置）

    // sanity check - are we trying to marginalize stuff that is not connected to this error term?
    OKVIS_ASSERT_TRUE(
        Exception,
        it != parameterBlockId2parameterBlockInfoIdx_.end(),
        "trying to marginalize out unconnected parameter block id = "<<parameterBlockIdsCopy[i])
    if (it == parameterBlockId2parameterBlockInfoIdx_.end())
      return false;

    // distinguish dense and landmark (sparse) part for more efficient pseudo-inversion later on
    size_t startIdx = parameterBlockInfos_.at(it->second).orderingIdx;//提取数据块对应的数据块信息-----在H阵的起始位置
    size_t minDim = parameterBlockInfos_.at(it->second).minimalDimension;//提取数据块对应的数据块信息-----最小维数
    //如果是地标点
    if (parameterBlockInfos_.at(it->second).isLandmark) {
      marginalizationStartIdxAndLengthPairslandmarks.push_back(
          std::pair<int, int>(startIdx, minDim));//添加（地标点数据块在H阵的起始位置、地标点数据块的最小维数）到集合中
      marginalizationParametersLandmarks += minDim;//地标点最小维数的累加
    } else {
      marginalizationStartIdxAndLengthPairsDense.push_back(
          std::pair<int, int>(startIdx, minDim));//添加（非地标点数据块在H阵的起始位置、非地标点数据块的最小维数）到集合中
      marginalizationParametersDense += minDim;//非地标点最小维数的累加
    }
  }

  // make sure the marginalization pairs are ordered
  // 将集合中数据（地标点数据块在H阵的起始位置）升序排序
  std::sort(marginalizationStartIdxAndLengthPairslandmarks.begin(),
            marginalizationStartIdxAndLengthPairslandmarks.end(),
            [](std::pair<int,int> left, std::pair<int,int> right) {
              return left.first < right.first;
            });
  //将集合中数据（非地标点数据块在H阵的起始位置）升序排序
  std::sort(marginalizationStartIdxAndLengthPairsDense.begin(),
            marginalizationStartIdxAndLengthPairsDense.end(),
            [](std::pair<int,int> left, std::pair<int,int> right) {
              return left.first < right.first;
            });

  // unify contiguous marginalization requests
  // （地标点数据块在H阵的起始位置、地标点数据块的最小维数）的集合
  for (size_t m = 1; m < marginalizationStartIdxAndLengthPairslandmarks.size();
      ++m) {
    //如果上一个数据块（待边缘化的）在H阵的起点+上一个数据块（待边缘化）的最小维数=当前数据块（带边缘化）的起点（连续）
    //注意待边缘化的地标点不一定是连续的
    if (marginalizationStartIdxAndLengthPairslandmarks.at(m - 1).first
        + marginalizationStartIdxAndLengthPairslandmarks.at(m - 1).second
        == marginalizationStartIdxAndLengthPairslandmarks.at(m).first) {
      marginalizationStartIdxAndLengthPairslandmarks.at(m - 1).second +=
          marginalizationStartIdxAndLengthPairslandmarks.at(m).second;//连续地标点维度的累加
      marginalizationStartIdxAndLengthPairslandmarks.erase(
          marginalizationStartIdxAndLengthPairslandmarks.begin() + m);//删除地标点的信息（连续地标点的后面几个）
      --m;
    }
  }
  //（非地标点数据块在H阵的起始位置、非地标点数据块的最小维数）的集合
  for (size_t m = 1; m < marginalizationStartIdxAndLengthPairsDense.size();
      ++m) {
    //如果上一个数据块（待边缘化的）在H阵的起点+上一个数据块（待边缘化）的最小维数=当前数据块（带边缘化）的起点（连续）
    //注意待边缘化的非地标点数据块不一定是连续的
    if (marginalizationStartIdxAndLengthPairsDense.at(m - 1).first
        + marginalizationStartIdxAndLengthPairsDense.at(m - 1).second
        == marginalizationStartIdxAndLengthPairsDense.at(m).first) {
      marginalizationStartIdxAndLengthPairsDense.at(m - 1).second +=
          marginalizationStartIdxAndLengthPairsDense.at(m).second;//连续非地标点数据块维度的累加
      marginalizationStartIdxAndLengthPairsDense.erase(
          marginalizationStartIdxAndLengthPairsDense.begin() + m);//删除非地标点数据块的信息（连续非地标点数据块的后面几个）
      --m;
    }
  }

  errorComputationValid_ = false;  // flag that the error computation is invalid

  // include in the fix rhs part deviations from linearization point of the parameter blocks to be marginalized
  // corrected: this is not necessary, will cancel itself

  /* landmark part (if existing) */
  if (marginalizationStartIdxAndLengthPairslandmarks.size() > 0) {

    // preconditioner
    //提取H_矩阵的对角线上元素,并且元素如果大于1.0e-9,选择 p=H_矩阵对角线上元素的平方根
    Eigen::VectorXd p = (H_.diagonal().array() > 1.0e-9).select(H_.diagonal().cwiseSqrt(),1.0e-3);
    Eigen::VectorXd p_inv = p.cwiseInverse();//点乘求逆

    // scale H and b
    //调整H_的尺度
    H_ = p_inv.asDiagonal() * H_ * p_inv.asDiagonal();
    b0_ = p_inv.asDiagonal() * b0_;
    //marginalizationParametersLandmarks为待边缘化地标点的总维度
    Eigen::MatrixXd U(H_.rows() - marginalizationParametersLandmarks,
                      H_.rows() - marginalizationParametersLandmarks);//U矩阵的维度比H_小,减少待边缘化地标点的总维度
    Eigen::MatrixXd V(marginalizationParametersLandmarks,
                      marginalizationParametersLandmarks);//V矩阵的维度为待边缘化地标点的总维度
    Eigen::MatrixXd W(H_.rows() - marginalizationParametersLandmarks,
                      marginalizationParametersLandmarks);
    Eigen::VectorXd b_a(H_.rows() - marginalizationParametersLandmarks);//b_a的维度为(原来维度-待边缘化地标点的总维度)
    Eigen::VectorXd b_b(marginalizationParametersLandmarks);//b_b的维度为(待边缘化地标点的总维度)

    // split preconditioner
    Eigen::VectorXd p_a(H_.rows() - marginalizationParametersLandmarks);//减少地标点后的维度
    Eigen::VectorXd p_b(marginalizationParametersLandmarks);//待边缘化地标点的总维度
    ///分裂p为p_a,p_b, p_a为不涉及边缘化的p中元素, p_b为需要边缘化的p中元素
    splitVector(marginalizationStartIdxAndLengthPairslandmarks, p, p_a, p_b);  // output

    // split lhs
    ///分裂H_矩阵,其中U是不涉及边缘化的H中的元素,W是待边缘化数据块和保留数据块的交叉海瑟矩阵,V为待边缘化数据块的海瑟矩阵
    splitSymmetricMatrix(marginalizationStartIdxAndLengthPairslandmarks, H_, U,
                         W, V);  // output

    // split rhs
    ///分裂b向量,b_a为不涉及边缘化的b中元素,b_b为待边缘化的b中元素
    splitVector(marginalizationStartIdxAndLengthPairslandmarks, b0_, b_a, b_b);  // output


    // invert the marginalization block
    //地标点的最小维数,3
    static const int sdim =
        ::okvis::ceres::HomogeneousPointParameterBlock::MinimalDimension;
    b0_.resize(b_a.rows());
    b0_ = b_a;//剔除之后的b0_
    H_.resize(U.rows(), U.cols());
    H_ = U;//剔除之后的H_
    const size_t numBlocks = V.cols() / sdim;//需要剔除地标点的数量
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> delta_H(
        numBlocks);//矩阵容器,维度为待边缘化地标点的数量
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> delta_b(
        numBlocks);
    Eigen::MatrixXd M1(W.rows(), W.cols());
    size_t idx = 0;
    for (size_t i = 0; int(i) < V.cols(); i += sdim) {
      Eigen::Matrix<double, sdim, sdim> V_inv_sqrt;//3*3的矩阵
      Eigen::Matrix<double, sdim, sdim> V1 = V.block(i, i, sdim, sdim);//V中该地标点对应的矩阵块
      MarginalizationError::pseudoInverseSymmSqrt(V1, V_inv_sqrt);//V_inv_sqrt为V1的广义逆
      Eigen::MatrixXd M = W.block(0, i, W.rows(), sdim) * V_inv_sqrt;//W中该地标点对应矩阵块*V中矩阵块的逆
      Eigen::MatrixXd M1 = W.block(0, i, W.rows(), sdim) * V_inv_sqrt
          * V_inv_sqrt.transpose();
      // accumulate
      delta_H.at(idx).resize(U.rows(), U.cols());//赋值delta_H的维度
      delta_b.at(idx).resize(b_a.rows());//赋值delta_b的维度
      if (i == 0) {
        delta_H.at(idx) = M * M.transpose();//初始化delta_H
        delta_b.at(idx) = M1 * b_b.segment<sdim>(i);//初始化delta_b
      } else {
        delta_H.at(idx) = delta_H.at(idx - 1) + M * M.transpose();//更新delta_H
        delta_b.at(idx) = delta_b.at(idx - 1) + M1 * b_b.segment<sdim>(i);//更新delta_b
      }
      ++idx;
    }
    // Schur
    // Schur消元,相当于通过高斯消元,将待边缘化的地标点变量从线性方程组中剔除
    b0_ -= delta_b.at(idx - 1);
    H_ -= delta_H.at(idx - 1);

    // unscale
    // 利用特征值统一化尺度,不至于H中数值的数目过大
    H_ = p_a.asDiagonal() * H_ * p_a.asDiagonal();
    b0_ = p_a.asDiagonal() * b0_;
  }

  /* dense part (if existing) */
  if (marginalizationStartIdxAndLengthPairsDense.size() > 0) {
    ///非地标点数据块的边缘化
    // preconditioner
    Eigen::VectorXd p = (H_.diagonal().array() > 1.0e-9).select(H_.diagonal().cwiseSqrt(),1.0e-3);//将H_矩阵的特征值平方根提取出来作为向量
    Eigen::VectorXd p_inv = p.cwiseInverse();//点乘求逆

    // scale H and b
    //利用特征值缩小H阵的尺度
    H_ = p_inv.asDiagonal() * H_ * p_inv.asDiagonal();
    b0_ = p_inv.asDiagonal() * b0_;

    Eigen::MatrixXd U(H_.rows() - marginalizationParametersDense,
                      H_.rows() - marginalizationParametersDense);//U为H中除去待边缘化数据块(非地标点)对应元素的子矩阵
    Eigen::MatrixXd V(marginalizationParametersDense,
                      marginalizationParametersDense);//V为H中待边缘化数据块(非地标点)对应元素的子矩阵
    Eigen::MatrixXd W(H_.rows() - marginalizationParametersDense,
                      marginalizationParametersDense);//W为H中两种数据块交叉作用的子矩阵
    Eigen::VectorXd b_a(H_.rows() - marginalizationParametersDense);//b中未边缘化的元素
    Eigen::VectorXd b_b(marginalizationParametersDense);//b中待边缘化的元素

    // split preconditioner
    Eigen::VectorXd p_a(H_.rows() - marginalizationParametersDense);
    Eigen::VectorXd p_b(marginalizationParametersDense);
    //分割向量p,根据是否边缘化
    splitVector(marginalizationStartIdxAndLengthPairsDense, p, p_a, p_b);  // output

    // split lhs
    //分割矩阵H
    splitSymmetricMatrix(marginalizationStartIdxAndLengthPairsDense, H_, U, W,
                         V);  // output

    // split rhs
    //分割向量b0_
    splitVector(marginalizationStartIdxAndLengthPairsDense, b0_, b_a, b_b);  // output

    // invert the marginalization block
    Eigen::MatrixXd V_inverse_sqrt(V.rows(), V.cols());
    Eigen::MatrixXd V1 = 0.5 * (V + V.transpose());//正交化
    pseudoInverseSymmSqrt(V1, V_inverse_sqrt);//求V的广义逆

    // Schur
    ///schur高斯消元
    Eigen::MatrixXd M = W * V_inverse_sqrt;
    // rhs
    b0_.resize(b_a.rows());
    b0_ = (b_a - M * V_inverse_sqrt.transpose() * b_b);
    // lhs
    H_.resize(U.rows(), U.cols());

    H_ = (U - M * M.transpose());

    // unscale
    // 尺度均衡
    H_ = p_a.asDiagonal() * H_ * p_a.asDiagonal();
    b0_ = p_a.asDiagonal() * b0_;
  }

  // also adapt the ceres-internal size information
  base_t::set_num_residuals(
      base_t::num_residuals() - marginalizationParametersDense
          - marginalizationParametersLandmarks);//更新X的维数

  /* delete all the book-keeping */
  ///parameterBlockIdsCopy为升序排列的数据块序号的集合,无重复
  for (size_t i = 0; i < parameterBlockIdsCopy.size(); ++i) {
    size_t idx = parameterBlockId2parameterBlockInfoIdx_.find(
        parameterBlockIdsCopy[i])->second;//得到该数据块对应的信息位置(在parameterBlockInfos_中)
    int margSize = parameterBlockInfos_.at(idx).minimalDimension;//提取数据块的维数
    parameterBlockInfos_.erase(parameterBlockInfos_.begin() + idx);//将数据块对应的信息从信息容器中删除

    for (size_t j = idx; j < parameterBlockInfos_.size(); ++j) {
      parameterBlockInfos_.at(j).orderingIdx -= margSize;//更新信息容器中后面数据块的起始位置(在H矩阵中的起始行和列)
      parameterBlockId2parameterBlockInfoIdx_.at(
          parameterBlockInfos_.at(j).parameterBlockId) -= 1;//数据块对应的信息容器中的位置减一
    }

    parameterBlockId2parameterBlockInfoIdx_.erase(parameterBlockIdsCopy[i]);//删除该数据块在parameterBlockId2parameterBlockInfoIdx_中对应的值

    // also adapt the ceres-internal book-keepin
    //mutable_parameter_block_sizes函数返回一个容器,储存数据块对应的维度
    base_t::mutable_parameter_block_sizes()->erase(
        mutable_parameter_block_sizes()->begin() + idx);
  }

  /* assume everything got dense */
  // this is a conservative assumption, but true in particular when marginalizing
  // poses w/o landmarks
  ///假设所有的未被删除的数据块都不是地标点
  /// parameterBlockInfos_的维数应该和parameterBlockIdsCopy略大
  /// 比如论文中的地标点1可以被1,2,3关键帧看到,所以其对应三个二次投影误差,分别添加三个投影误差时,会给parameterBlockInfos_添加七个数据块,如:-------
  /// ----------------1,2,3关键帧的位姿,外参,以及地标点1的坐标
  /// 而parameterBlockIdsCopy只有地标点1和关键帧1的位姿和外参
  denseIndices_ = parameterBlockInfos_.size();
  for (size_t i = 0; i < parameterBlockInfos_.size(); ++i) {
    if (parameterBlockInfos_.at(i).isLandmark) {
      parameterBlockInfos_.at(i).isLandmark = false;
    }
  }

  // check if the removal is safe
  for (size_t i = 0; i < parameterBlockIdsCopy.size(); ++i) {
    Map::ResidualBlockCollection residuals = mapPtr_->residuals(
        parameterBlockIdsCopy[i]);//如果被边缘化的数据块还存在关联的残差关系
    if (residuals.size() != 0
        && keepParameterBlocksCopy.at(parameterBlockIdsCopy[i]) == false)
      mapPtr_->printParameterBlockInfo(parameterBlockIdsCopy[i]);//打印数据块和其对应的残差信息
    OKVIS_ASSERT_TRUE_DBG(
        Exception,
        residuals.size()==0 || keepParameterBlocksCopy.at(parameterBlockIdsCopy[i]) == true,
        "trying to marginalize out a parameterBlock that is still connected to other error terms."
        <<" keep = "<<int(keepParameterBlocksCopy.at(parameterBlockIdsCopy[i])));
  }
  for (size_t i = 0; i < parameterBlockIdsCopy.size(); ++i) {
    if (keepParameterBlocksCopy.at(parameterBlockIdsCopy[i])) {
      OKVIS_THROW(Exception,"unmarginalizeLandmark(parameterBlockIdsCopy[i]) not implemented.")
    } else {
      mapPtr_->removeParameterBlock(parameterBlockIdsCopy[i]);//从地图中移除数据块
    }
  }

  check();

  return true;
}

// This must be called before optimization after adding residual blocks and/or marginalizing,
// since it performs all the lhs and rhs computations on from a given _H and _b.
/// 更新H和b，在优化之前，添加残差块和边缘化之后
void MarginalizationError::updateErrorComputation() {
  //errorComputationValid_初始值为false
  if (errorComputationValid_)
    return;  // already done.

  // now we also know the error dimension:
  base_t::set_num_residuals(H_.cols());//设置残差的维度

  // preconditioner
  // 对H_的对角线上值求平方根
  Eigen::VectorXd p = (H_.diagonal().array() > 1.0e-9).select(H_.diagonal().cwiseSqrt(),1.0e-3);
  Eigen::VectorXd p_inv = p.cwiseInverse();//求p的逆

  // lhs SVD: _H = J^T*J = _U*S*_U^T
  // 正规化H_矩阵,之后设置对称矩阵的求解器
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(
      0.5  * p_inv.asDiagonal() * (H_ + H_.transpose())  * p_inv.asDiagonal() );

  static const double epsilon = std::numeric_limits<double>::epsilon();//运行编译程序的计算机所能识别的最小非零浮点数。
  double tolerance = epsilon * H_.cols()
      * saes.eigenvalues().array().maxCoeff();//最大特征值*H_的维度*最小非零浮点数
  S_ = Eigen::VectorXd(
      (saes.eigenvalues().array() > tolerance).select(
          saes.eigenvalues().array(), 0));//特征值对角阵
  S_pinv_ = Eigen::VectorXd(
      (saes.eigenvalues().array() > tolerance).select(
          saes.eigenvalues().array().inverse(), 0));//特征值倒数的对角阵

  S_sqrt_ = S_.cwiseSqrt();//S_的平方根
  S_pinv_sqrt_ = S_pinv_.cwiseSqrt();//S_pinv_的平方根

  // assign Jacobian
  /// 通过SVD计算sqrt(H),即J.
  J_ = (p.asDiagonal() * saes.eigenvectors() * (S_sqrt_.asDiagonal())).transpose();

  // constant error (residual) _e0 := (-pinv(J^T) * _b):
  /// 通过SVD的逆计算J-1
  Eigen::MatrixXd J_pinv_T = (S_pinv_sqrt_.asDiagonal())
      * saes.eigenvectors().transpose()  *p_inv.asDiagonal() ;
  /// 从b推导出e
  e0_ = (-J_pinv_T * b0_);

  // reconstruct. TODO: check if this really improves quality --- doesn't seem so...
  //H_ = J_.transpose() * J_;
  //b0_ = -J_.transpose() * e0_;
  errorComputationValid_ = true;
}

// Computes the linearized deviation from the references (linearization points)
bool MarginalizationError::computeDeltaChi(Eigen::VectorXd& DeltaChi) const {
  DeltaChi.resize(H_.rows());
  for (size_t i = 0; i < parameterBlockInfos_.size(); ++i) {
    // stack Delta_Chi vector
    if (!parameterBlockInfos_[i].parameterBlockPtr->fixed()) {
      Eigen::VectorXd Delta_Chi_i(parameterBlockInfos_[i].minimalDimension);
      parameterBlockInfos_[i].parameterBlockPtr->minus(
          parameterBlockInfos_[i].linearizationPoint.get(),
          parameterBlockInfos_[i].parameterBlockPtr->parameters(),
          Delta_Chi_i.data());
      DeltaChi.segment(parameterBlockInfos_[i].orderingIdx,
                       parameterBlockInfos_[i].minimalDimension) = Delta_Chi_i;
			}
  }
  return true;
}

// Computes the linearized deviation from the references (linearization points)
bool MarginalizationError::computeDeltaChi(double const* const * parameters,
                                           Eigen::VectorXd& DeltaChi) const {
  DeltaChi.resize(H_.rows());
  for (size_t i = 0; i < parameterBlockInfos_.size(); ++i) {
    // stack Delta_Chi vector
    if (!parameterBlockInfos_[i].parameterBlockPtr->fixed()) {
      Eigen::VectorXd Delta_Chi_i(parameterBlockInfos_[i].minimalDimension);
      parameterBlockInfos_[i].parameterBlockPtr->minus(
          parameterBlockInfos_[i].linearizationPoint.get(), parameters[i],
          Delta_Chi_i.data());
      DeltaChi.segment(parameterBlockInfos_[i].orderingIdx,
                       parameterBlockInfos_[i].minimalDimension) = Delta_Chi_i;
			}
  }
  return true;
}

//This evaluates the error term and additionally computes the Jacobians.
bool MarginalizationError::Evaluate(double const* const * parameters,
                                    double* residuals,
                                    double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
/// 计算最小雅各比矩阵
bool MarginalizationError::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobiansMinimal) const {
  OKVIS_ASSERT_TRUE_DBG(
      Exception,
      errorComputationValid_,
      "trying to opmimize, but updateErrorComputation() was not called after adding residual blocks/marginalizing");

  Eigen::VectorXd Delta_Chi;
  computeDeltaChi(parameters, Delta_Chi);

  for (size_t i = 0; i < parameterBlockInfos_.size(); ++i) {

    // decompose the jacobians: minimal ones are easy
    if (jacobiansMinimal != NULL) {
      if (jacobiansMinimal[i] != NULL) {
        Eigen::Map<
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                Eigen::RowMajor> > Jmin_i(
            jacobiansMinimal[i], e0_.rows(),
            parameterBlockInfos_[i].minimalDimension);
        Jmin_i = J_.block(0, parameterBlockInfos_[i].orderingIdx, e0_.rows(),
                          parameterBlockInfos_[i].minimalDimension);
      }
    }

    // hallucinate the non-minimal Jacobians
    if (jacobians != NULL) {
      if (jacobians[i] != NULL) {
        Eigen::Map<
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                Eigen::RowMajor> > J_i(jacobians[i], e0_.rows(),
                                       parameterBlockInfos_[i].dimension);
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Jmin_i =
            J_.block(0, parameterBlockInfos_[i].orderingIdx, e0_.rows(),
                     parameterBlockInfos_[i].minimalDimension);

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> J_lift(
            parameterBlockInfos_[i].parameterBlockPtr->minimalDimension(),
            parameterBlockInfos_[i].parameterBlockPtr->dimension());
        parameterBlockInfos_[i].parameterBlockPtr->liftJacobian(
            parameterBlockInfos_[i].linearizationPoint.get(), J_lift.data());

        J_i = Jmin_i * J_lift;
      }
    }
  }

  // finally the error (residual) e = (-pinv(J^T) * _b + _J*Delta_Chi):
  Eigen::Map<Eigen::VectorXd> e(residuals, e0_.rows());
  e = e0_ + J_ * Delta_Chi;

  return true;
}

}  // namespace ceres
}  // namespace okvis

