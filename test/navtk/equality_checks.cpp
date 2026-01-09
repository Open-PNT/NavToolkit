#include <equality_checks.hpp>

#include <gtest/gtest.h>
#include <tensor_assert.hpp>

#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>

namespace navtk {
namespace filtering {
namespace testing {

void verify_sol(const inertial::StandardPosVelAtt& sol1,
                const inertial::StandardPosVelAtt& sol2,
                double rtol,
                double atol) {
	ASSERT_EQ(sol1.time_validity, sol2.time_validity);
	ASSERT_ALLCLOSE_EX(sol1.get_llh(), sol2.get_llh(), rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1.get_vned(), sol2.get_vned(), rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1.get_C_s_to_ned(), sol2.get_C_s_to_ned(), rtol, atol);
}

void verify_sol(const NavSolution& sol1, const NavSolution& sol2, double rtol, double atol) {
	ASSERT_EQ(sol1.time, sol2.time);
	ASSERT_ALLCLOSE_EX(sol1.pos, sol2.pos, rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1.vel, sol2.vel, rtol, atol);
	ASSERT_ALLCLOSE_EX(sol1.rot_mat, sol2.rot_mat, rtol, atol);
}
}  // namespace testing
}  // namespace filtering
}  // namespace navtk
