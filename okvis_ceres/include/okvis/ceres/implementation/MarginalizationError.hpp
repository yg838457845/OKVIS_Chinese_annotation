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
 *  Created on: Sep 11, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/MarginalizationError.hpp
 * @brief Header implementation file for the MarginalizationError class.
 * @author Stefan Leutenegger
 */

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Split for Schur complement op.
/// 分割反对称矩阵
/// marginalizationStartIdxAndLengthPairs为储存----待边缘化数据块在H阵中起点和维度的pair型容器
template<typename Derived_A, typename Derived_U, typename Derived_W,
    typename Derived_V>
void MarginalizationError::splitSymmetricMatrix(
    const std::vector<std::pair<int, int> >& marginalizationStartIdxAndLengthPairs,
    const Eigen::MatrixBase<Derived_A>& A,  // input
    const Eigen::MatrixBase<Derived_U>& U,  // output
    const Eigen::MatrixBase<Derived_W>& W,  // output
    const Eigen::MatrixBase<Derived_V>& V) {  // output

  // sanity check
  const int size = A.cols();//A的列
  OKVIS_ASSERT_TRUE_DBG(Exception, size == A.rows(), "matrix not symmetric");
  OKVIS_ASSERT_TRUE_DBG(Exception, V.cols() == V.rows(),
                        "matrix not symmetric");
  OKVIS_ASSERT_TRUE_DBG(Exception, V.cols() == W.cols(),
                        "matrix not symmetric");
  OKVIS_ASSERT_TRUE_DBG(Exception, U.rows() == W.rows(),
                        "matrix not symmetric");
  OKVIS_ASSERT_TRUE_DBG(
      Exception,
      V.rows() + U.rows() == size,
      "matrices supplied to be split into do not form exact upper triangular blocks of the original one");
  OKVIS_ASSERT_TRUE_DBG(Exception, U.cols() == U.rows(),
                        "matrix not symmetric");
  //将输入的容器重新储存,后面加入(size,0)
  std::vector<std::pair<int, int> > marginalizationStartIdxAndLengthPairs2 =
      marginalizationStartIdxAndLengthPairs;
  marginalizationStartIdxAndLengthPairs2.push_back(
      std::pair<int, int>(size, 0));

  const size_t length = marginalizationStartIdxAndLengthPairs2.size();

  int lastIdx_row = 0;
  int start_a_i = 0;
  int start_b_i = 0;
  for (size_t i = 0; i < length; ++i) {
    int lastIdx_col = 0;
    int start_a_j = 0;
    int start_b_j = 0;
    int thisIdx_row = marginalizationStartIdxAndLengthPairs2[i].first;//待边缘化的数据块的起点
    const int size_b_i = marginalizationStartIdxAndLengthPairs2[i].second;  // 待边缘化的数据块的维度
    const int size_a_i = thisIdx_row - lastIdx_row;  // 当前数据块在H中的行起点减去上一个数据块在H中的行终点
    for (size_t j = 0; j < length; ++j) {
      int thisIdx_col = marginalizationStartIdxAndLengthPairs2[j].first;
      const int size_a_j = thisIdx_col - lastIdx_col;  // 当前数据块在H中的列起点减去上一个数据块在H中的列终点
      const int size_b_j = marginalizationStartIdxAndLengthPairs2[j].second;  // 待边缘化的数据块的维度

      // the kept part - only access and copy if non-zero
      if (size_a_j > 0 && size_a_i > 0) {
         //U对应未涉及边缘化数据块的H矩阵的元素
        const_cast<Eigen::MatrixBase<Derived_U>&>(U).block(start_a_i, start_a_j,
                                                           size_a_i, size_a_j) =
            A.block(lastIdx_row, lastIdx_col, size_a_i, size_a_j);
      }

      // now the mixed part
      if (size_b_j > 0 && size_a_i > 0) {
         //W对应边缘化数据块和未边缘化数据块在H阵中交叉的元素
        const_cast<Eigen::MatrixBase<Derived_W>&>(W).block(start_a_i, start_b_j,
                                                           size_a_i, size_b_j) =
            A.block(lastIdx_row, thisIdx_col, size_a_i, size_b_j);
      }

      // and finally the marginalized part
      if (size_b_j > 0 && size_b_i > 0) {
         //V对应边缘化数据块在H矩阵中的元素
        const_cast<Eigen::MatrixBase<Derived_V>&>(V).block(start_b_i, start_b_j,
                                                           size_b_i, size_b_j) =
            A.block(thisIdx_row, thisIdx_col, size_b_i, size_b_j);
      }

      lastIdx_col = thisIdx_col + size_b_j;  // 列的终点
      start_a_j += size_a_j;//起点的更新
      start_b_j += size_b_j;
    }
    lastIdx_row = thisIdx_row + size_b_i;  // 行的终点
    start_a_i += size_a_i;//起点的更新
    start_b_i += size_b_i;
  }
}

// Split for Schur complement op.
/// 分割向量
/// marginalizationStartIdxAndLengthPairs为储存----待边缘化数据块在H阵中起点和维度的pair型容器
template<typename Derived_b, typename Derived_b_a, typename Derived_b_b>
void MarginalizationError::splitVector(
    const std::vector<std::pair<int, int> >& marginalizationStartIdxAndLengthPairs,
    const Eigen::MatrixBase<Derived_b>& b,  // input
    const Eigen::MatrixBase<Derived_b_a>& b_a,  // output
    const Eigen::MatrixBase<Derived_b_b>& b_b) {  // output

  const int size = b.rows();//b的行
  // sanity check
  OKVIS_ASSERT_TRUE_DBG(Exception, b.cols() == 1, "supplied vector not x-by-1");
  OKVIS_ASSERT_TRUE_DBG(Exception, b_a.cols() == 1,
                        "supplied vector not x-by-1");
  OKVIS_ASSERT_TRUE_DBG(Exception, b_b.cols() == 1,
                        "supplied vector not x-by-1");
  OKVIS_ASSERT_TRUE_DBG(
      Exception,
      b_a.rows() + b_b.rows() == size,
      "vector supplied to be split into cannot be concatenated to the original one");
 ///将marginalizationStartIdxAndLengthPairs赋值给marginalizationStartIdxAndLengthPairs2
  std::vector<std::pair<int, int> > marginalizationStartIdxAndLengthPairs2 =
      marginalizationStartIdxAndLengthPairs;
  ///在容器最后添加上一个新的元素(size,0)
  marginalizationStartIdxAndLengthPairs2.push_back(
      std::pair<int, int>(size, 0));

  const size_t length = marginalizationStartIdxAndLengthPairs2.size();//长度

  int lastIdx_row = 0;
  int start_a_i = 0;
  int start_b_i = 0;
  for (size_t i = 0; i < length; ++i) {
    int thisIdx_row = marginalizationStartIdxAndLengthPairs2[i].first;//第i个数据块在H阵中的起点
    const int size_b_i = marginalizationStartIdxAndLengthPairs2[i].second;  // 第i个数据块的维度
    const int size_a_i = thisIdx_row - lastIdx_row;  // 当前数据块的起点和上一个数据块终点的差

    // the kept part - only access and copy if non-zero
    /// 如果前后两个数据块间有空隙的地带
    /// b_a中存储这个空隙地带对应的b值(b_a的数据不涉及边缘化)
    if (size_a_i > 0) {
      const_cast<Eigen::MatrixBase<Derived_b_a>&>(b_a).segment(start_a_i,
                                                               size_a_i) = b
          .segment(lastIdx_row, size_a_i);
    }

    // and finally the marginalized part
    ///数据块的维度大于零
    /// b_b中储存需要被边缘化数据块对应的b值
    if (size_b_i > 0) {
      const_cast<Eigen::MatrixBase<Derived_b_b>&>(b_b).segment(start_b_i,
                                                               size_b_i) = b
          .segment(thisIdx_row, size_b_i);
    }

    lastIdx_row = thisIdx_row + size_b_i;  // remember,表示数据块在H帧中的终点
    start_a_i += size_a_i;//b_a中的起始位置
    start_b_i += size_b_i;//b_b中的起始位置
  }
}

// Pseudo inversion of a symmetric matrix.
// attention: this uses Eigen-decomposition, it assumes the input is symmetric positive semi-definite
// (negative Eigenvalues are set to zero)
///求取一个对称矩阵的伪逆矩阵
template<typename Derived>
bool MarginalizationError::pseudoInverseSymm(
    const Eigen::MatrixBase<Derived>&a, const Eigen::MatrixBase<Derived>&result,
    double epsilon, int * rank) {
  //保证a是方阵
  OKVIS_ASSERT_TRUE_DBG(Exception, a.rows() == a.cols(),
                        "matrix supplied is not quadratic");
  ///认为a是埃尔米特矩阵,即实数空间的对称矩阵
  Eigen::SelfAdjointEigenSolver<Derived> saes(a);

  typename Derived::Scalar tolerance = epsilon * a.cols()
      * saes.eigenvalues().array().maxCoeff();
  //利用埃尔米特矩阵的性质求逆
  const_cast<Eigen::MatrixBase<Derived>&>(result) = (saes.eigenvectors())
      * Eigen::VectorXd(
          (saes.eigenvalues().array() > tolerance).select(
              saes.eigenvalues().array().inverse(), 0)).asDiagonal()
      * (saes.eigenvectors().transpose());

  if (rank) {
    *rank = 0;
    for (int i = 0; i < a.rows(); ++i) {
      if (saes.eigenvalues()[i] > tolerance)
        (*rank)++;
    }
  }

  return true;
}

// Pseudo inversion and square root (Cholesky decomposition) of a symmetric matrix.
// attention: this uses Eigen-decomposition, it assumes the input is symmetric positive semi-definite
// (negative Eigenvalues are set to zero)
template<typename Derived>
bool MarginalizationError::pseudoInverseSymmSqrt(
    const Eigen::MatrixBase<Derived>&a, const Eigen::MatrixBase<Derived>&result,
    double epsilon, int * rank) {

  OKVIS_ASSERT_TRUE_DBG(Exception, a.rows() == a.cols(),
                        "matrix supplied is not quadratic");

  Eigen::SelfAdjointEigenSolver<Derived> saes(a);

  typename Derived::Scalar tolerance = epsilon * a.cols()
      * saes.eigenvalues().array().maxCoeff();

  const_cast<Eigen::MatrixBase<Derived>&>(result) = (saes.eigenvectors())
      * Eigen::VectorXd(
          Eigen::VectorXd(
              (saes.eigenvalues().array() > tolerance).select(
                  saes.eigenvalues().array().inverse(), 0)).array().sqrt())
          .asDiagonal();

  if (rank) {
    *rank = 0;
    for (int i = 0; i < a.rows(); ++i) {
      if (saes.eigenvalues()[i] > tolerance)
        (*rank)++;
    }
  }

  return true;
}

// Block-wise pseudo inversion of a symmetric matrix with non-zero diagonal blocks.
// attention: this uses Eigen-decomposition, it assumes the input is symmetric positive semi-definite
// (negative Eigenvalues are set to zero)
template<typename Derived, int blockDim>
void MarginalizationError::blockPinverse(
    const Eigen::MatrixBase<Derived>& M_in,
    const Eigen::MatrixBase<Derived>& M_out, double epsilon) {

  OKVIS_ASSERT_TRUE_DBG(Exception, M_in.rows() == M_in.cols(),
                        "matrix supplied is not quadratic");

  const_cast<Eigen::MatrixBase<Derived>&>(M_out).resize(M_in.rows(),
                                                        M_in.rows());
  const_cast<Eigen::MatrixBase<Derived>&>(M_out).setZero();
  for (int i = 0; i < M_in.cols(); i += blockDim) {
    Eigen::Matrix<double, blockDim, blockDim> inv;
    const Eigen::Matrix<double, blockDim, blockDim> in = M_in
        .template block<blockDim, blockDim>(i, i);
    //const Eigen::Matrix<double,blockDim,blockDim> in1=0.5*(in+in.transpose());
    pseudoInverseSymm(in, inv, epsilon);
    const_cast<Eigen::MatrixBase<Derived>&>(M_out)
        .template block<blockDim, blockDim>(i, i) = inv;
  }
}

// Block-wise pseudo inversion and square root (Cholesky decomposition)
// of a symmetric matrix with non-zero diagonal blocks.
// attention: this uses Eigen-decomposition, it assumes the input is symmetric positive semi-definite
// (negative Eigenvalues are set to zero)
template<typename Derived, int blockDim>
void MarginalizationError::blockPinverseSqrt(
    const Eigen::MatrixBase<Derived>& M_in,
    const Eigen::MatrixBase<Derived>& M_out, double epsilon) {

  OKVIS_ASSERT_TRUE_DBG(Exception, M_in.rows() == M_in.cols(),
                        "matrix supplied is not quadratic");

  const_cast<Eigen::MatrixBase<Derived>&>(M_out).resize(M_in.rows(),
                                                        M_in.rows());
  const_cast<Eigen::MatrixBase<Derived>&>(M_out).setZero();
  for (int i = 0; i < M_in.cols(); i += blockDim) {
    Eigen::Matrix<double, blockDim, blockDim> inv;
    const Eigen::Matrix<double, blockDim, blockDim> in = M_in
        .template block<blockDim, blockDim>(i, i);
    //const Eigen::Matrix<double,blockDim,blockDim> in1=0.5*(in+in.transpose());
    pseudoInverseSymmSqrt(in, inv, epsilon);
    const_cast<Eigen::MatrixBase<Derived>&>(M_out)
        .template block<blockDim, blockDim>(i, i) = inv;
  }
}

}
}
