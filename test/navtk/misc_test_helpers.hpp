#pragma once

#include <sstream>
#include <string>
#include <vector>

#include <spdlog_assert.hpp>  // to get ::fmt included properly

// Uses the given macro to define a test within the given suite which makes sure the given enum's
// values can all be printed using `ostream <<` operators. The generated test is written such that
// missing one of the enum's values in the argument list will trigger a failure.
//
// The __VA_ARGS__ list should be a list of qualified enum values, so for enum Foo, an example call
// might look like:
//
//     ENUM_PRINT_TEST(TEST, FooTests, Foo, Foo::BAR, Foo::BAZ)
//
#define ENUM_PRINT_TEST(macro, suite, enum_name, ...)                         \
	macro(suite, enum_name##__ostream_operator) {                             \
		int values_seen = 0;                                                  \
		::std::string expected{#__VA_ARGS__};                                 \
		::std::vector<enum_name> values{__VA_ARGS__};                         \
		::std::stringstream ss;                                               \
		for (const auto& value : values) {                                    \
			if (values_seen++) ss << ", ";                                    \
			ss << value;                                                      \
		}                                                                     \
		auto stringified_enum_values = ss.str();                              \
		EXPECT_EQ(expected, stringified_enum_values);                         \
		::std::stringstream invalid_enum_value;                               \
		invalid_enum_value << static_cast<enum_name>(values_seen);            \
		EXPECT_EQ(::fmt::format("<INVALID {}({})>", #enum_name, values_seen), \
		          invalid_enum_value.str());                                  \
	}
