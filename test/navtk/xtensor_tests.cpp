#include <type_traits>

#include <gtest/gtest.h>
#include <tensor_assert.hpp>

#include <navtk/tensors.hpp>
#include "tensor_assert.hpp"

using navtk::Vector;

TEST(XtensorTests, concatenate) {
	Vector foo      = {1, 2, 3};
	Vector bar      = {4, 5};
	foo             = xt::concatenate(xt::xtuple(foo, bar));
	Vector expected = {1, 2, 3, 4, 5};
	ASSERT_ALLCLOSE(foo, expected);
}
