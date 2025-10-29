#pragma once

#include <initializer_list>
#include <string>
#include <utility>


/**
 * If you need NavToolkit's plug-and-play sensor interface to estimate a value type besides
 * double (such as float or int), use compiler flags to define this macro to your desired type.
 */
#ifndef NAVTK_SCALAR
#	define NAVTK_SCALAR double
#endif
/**
 * The NavToolkit namespace.
 */
namespace navtk {
/**
 * Scalar type definition.
 */
typedef NAVTK_SCALAR Scalar;
}  // namespace navtk

#ifdef NAVTK_PYTHON_TENSOR
#	include <xtensor-python/pytensor.hpp>
namespace navtk {
template <std::size_t Dims, typename T = Scalar>
using Tensor = xt::pytensor<T, Dims>;
// xtensor-python doesn't have a `_fixed` equivalent
template <int R, int C, typename T = Scalar>
using MatrixN = xt::pytensor<T, 2>;
template <int N, typename T = Scalar>
using VectorN = xt::pytensor<T, 1>;
}  // namespace navtk
#else
#	include <xtensor/containers/xfixed.hpp>
#	include <xtensor/containers/xtensor.hpp>
namespace navtk {
/**
 * Tensor type definition.
 * @tparam Dims The number of dimensions in the tensor.
 * @tparam T The type of the element stored in the tensor.
 */
template <std::size_t Dims, typename T = Scalar>
using Tensor = xt::xtensor<T, Dims>;
/**
 * Matrix of shape `R`, `C` type definition.
 * @tparam R The number of rows in the matrix.
 * @tparam C The number of columns in the matrix.
 * @tparam T The type of the element stored in the matrix.
 */
template <int R, int C, typename T = Scalar>
using MatrixN = xt::xtensor_fixed<T, xt::xshape<R, C> >;
/**
 * Vector of length `N` type definition.
 * @tparam N The length of the vector.
 * @tparam T The type of the element stored in the vector.
 */
template <int N, typename T = Scalar>
using VectorN = xt::xtensor_fixed<T, xt::xshape<N> >;
}  // namespace navtk
#endif

#include <xtensor/views/xview.hpp>

namespace navtk {

/**
 * Matrix type definition.
 */
typedef Tensor<2> Matrix;
/**
 * Matrix of type `T` type definition.
 * @tparam T The type of the element stored in the matrix.
 */
template <typename T>
using MatrixT = Tensor<2, T>;
/**
 * Vector type definition.
 */
typedef Tensor<1> Vector;
/**
 * 3 by 3 Matrix type definition.
 */
typedef MatrixN<3, 3> Matrix3;
/**
 * Vector of length 3 type definition.
 */
typedef VectorN<3> Vector3;
/**
 * Vector of length 4 type definition.
 */
typedef VectorN<4> Vector4;

/**
 * Vector of type `T` type definition.
 * @tparam T The type of the element stored in the vector.
 */
template <typename T>
using VectorT = Tensor<1, T>;

using namespace xt::placeholders;  // required for `_` to work
/**
 * Matrix size type definition.
 */
using Size = Matrix::size_type;

}  // namespace navtk
