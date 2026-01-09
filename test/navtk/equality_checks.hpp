#pragma once

#include <navtk/filtering/containers/NavSolution.hpp>
#include <navtk/inertial/StandardPosVelAtt.hpp>

namespace navtk {
namespace filtering {
namespace testing {

/*
 * Verifies two position, velocity and attitude and time representations are suitably close to one
 * another.
 *
 * @param sol1 First solution to check.
 * @param sol2 Solution to compare to sol1.
 * @param rtol Allowable relative tolerance between the two position, velocity and attitude members.
 * @param atol Allowable absolute tolerance between the two position, velocity and attitude members.
 */
void verify_sol(const inertial::StandardPosVelAtt& sol1,
                const inertial::StandardPosVelAtt& sol2,
                double rtol = 1e-5,
                double atol = 1e-8);
/*
 * Verifies two position, velocity and attitude and time representations are suitably close to one
 * another.
 *
 * @param sol1 First solution to check.
 * @param sol2 Solution to compare to sol1.
 * @param rtol Allowable relative tolerance between the two position, velocity and attitude members.
 * @param atol Allowable absolute tolerance between the two position, velocity and attitude members.
 */
void verify_sol(const NavSolution& sol1,
                const NavSolution& sol2,
                double rtol = 1e-5,
                double atol = 1e-8);
}  // namespace testing
}  // namespace filtering
}  // namespace navtk
