#include <error_mode_assert.hpp>

// Memory location for ExpressionThrowTester's long-lived memory pins.
namespace testing {
namespace detail {
std::forward_list<OpaquePtr> pins = {};
std::forward_list<OpaquePtr>& get_pins() { return pins; }
}  // namespace detail
}  // namespace testing
