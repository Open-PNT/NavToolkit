#pragma once

#include <algorithm>
#include <regex>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>

// When this is true, log messages that are expected by tests (and would therefore otherwise be
// suppressed) get displayed in the test output.
static constexpr bool SHOW_EXPECTED_LOGS =
#ifdef SHOW_EXPECTED_LOGS
    true
#else
    false
#endif
    ;

namespace testing {

namespace detail {

// A polyfill for std::void_t on C++14
#if __cplusplus <= 201402L
template <typename T>
using void_t = void;
#else
using std::void_t;
#endif

// Macro for generating EnableIf-style helper types
#define DECLARE_SFINAE_ENABLE_IF(type_name) \
	template <typename T, bool v>           \
	using EnableIf##type_name = typename std::enable_if<v == type_name<T>::value>::type*

// Macro for generating SFINAE helper types based on the validity of an expression
#define DECLARE_SFINAE_BY_EXPR_VALIDITY(type_name, expr)             \
	template <typename T, typename = void>                           \
	struct type_name : std::false_type {};                           \
	template <typename T>                                            \
	struct type_name<T, void_t<decltype(expr)>> : std::true_type {}; \
	DECLARE_SFINAE_ENABLE_IF(type_name)

// SFINAE struct for things that will cause a compiler error if we try to return a non-reference
// type. This is needed for testing things like the ValidationContext, which deletes its copy and
// move constructors but has many methods that return references to itself. 99% of the time,
// allowing the ExpressionThrowTester lambda to return a reference means we're returning a reference
// to a stack-allocated slot on the lambda itself (stack-use-after-free), so we use this SFINAE to
// avoid doing that wherever possible.
/*
// On a modern compiler, this short version works. On older gcc's, deleted constructors don't honor
// SFINAE but somehow std::is_constructible works, so we need the workarounds below.
DECLARE_SFINAE_BY_EXPR_VALIDITY(
    ExprCanBeForwarded,
    std::remove_reference_t<T>(std::forward<T>(static_cast<T>(std::declval<T>()))));
*/
template <
    typename T,
    std::enable_if_t<std::is_constructible<T, decltype(static_cast<T>(std::declval<T>()))>::value,
                     void*> = nullptr>
constexpr decltype(std::forward<T>(static_cast<T>(std::declval<T>()))) forwarded_declval();

DECLARE_SFINAE_BY_EXPR_VALIDITY(ExprCanBeForwardedPartial, forwarded_declval<T>());

template <typename T, EnableIfExprCanBeForwardedPartial<T, true> = nullptr>
std::is_constructible<std::remove_reference_t<T>, decltype(forwarded_declval<T>())>
can_be_forwarded_v();

template <typename T, EnableIfExprCanBeForwardedPartial<T, false> = nullptr>
std::false_type can_be_forwarded_v();

template <typename T>
struct ExprCanBeForwarded : decltype(can_be_forwarded_v<T>()) {};

template <>
struct ExprCanBeForwarded<void> : std::false_type {};

DECLARE_SFINAE_ENABLE_IF(ExprCanBeForwarded);

}  // namespace detail

struct CaughtLog {
	spdlog::level::level_enum level;
	std::string text;
	CaughtLog(spdlog::level::level_enum level, std::string text);
};

std::string spdlog_level_to_macro_name(int level);


// A helper type that allows the LOGCATCHER_ASSERT_ to be defined once for both void- and non-void
// expressions. It works because it's impossible to overload operator,(void, whatever), so you get
// the default comma behavior of discarding the void expression and returning the VoidWorkaround
// itself. inspired by
// https://www.reddit.com/r/cpp/comments/52o99u/a_generic_workaround_for_voidreturning_functions/
struct VoidWorkaround {
	template <typename T, detail::EnableIfExprCanBeForwarded<T, false> = nullptr>
	constexpr friend auto operator,(T&& t, VoidWorkaround) -> decltype(t) {
		return static_cast<decltype(t)>(t);
	}
	template <typename T, detail::EnableIfExprCanBeForwarded<T, true> = nullptr>
	constexpr friend std::remove_reference_t<T> operator,(T&& t, VoidWorkaround) {
		return std::forward<T>(t);
	}
};

using Mutex = spdlog::details::null_mutex;


void result_to_message(const AssertionResult& assertion_result,
                       bool fatal,
                       const char* file,
                       int line);

class LogCatcher {
private:
	class Sink : public spdlog::sinks::base_sink<Mutex> {
	public:
		LogCatcher* catcher;
		Sink(LogCatcher* catcher);

	protected:
		void flush_() override;
		void sink_it_(const spdlog::details::log_msg& msg) override;
	};

	struct SinkManager {
		LogCatcher* catcher;
		std::remove_reference<decltype(spdlog::default_logger()->sinks())>::type backup_sinks;
		bool have_detached = false;
		bool forward_unexpected;
		SinkManager(LogCatcher* catcher);
		~SinkManager();
		void detach();
	};

public:
	const char* file;
	int line;
	int level;
	std::string action;
	std::string pattern;

	bool abort;
	std::shared_ptr<spdlog::logger> logger;
	std::vector<CaughtLog> logs;
	std::regex re;
	bool have_log;
	std::unique_ptr<SinkManager> manager;

	LogCatcher(const char* file,
	           int line,
	           int level,
	           const std::string& action,
	           const std::string& pattern,
	           bool abort = false);

	~LogCatcher();

	AssertionResult build_result() const;
	void detach_and_send_result(bool fatal) const;

	template <typename Result>
	auto pass_through_and_assert(Result&& result, bool fatal) const -> decltype(result) {
		this->detach_and_send_result(fatal);
		return static_cast<decltype(result)>(result);
	}

	template <typename Result>
	auto ignore(Result&& result) -> decltype(result) {
		this->manager->detach();
		return static_cast<decltype(result)>(result);
	}

	// RAII type that prevents the given logger from suppressing exceptions. When destructed,
	// restores the default error-handling behavior.
	class SpdlogExceptionForwarder {
	public:
		SpdlogExceptionForwarder(std::shared_ptr<spdlog::logger> logger);
		~SpdlogExceptionForwarder();

	private:
		std::shared_ptr<spdlog::logger> logger;
	};

	// Special exception type thrown when in "abort mode". Don't use this directly, use
	// error_mode_assert.hpp's EXPECT_UB_OR_DIE.
	struct Aborting {
		std::shared_ptr<SpdlogExceptionForwarder> spdlog_hack;
	};
};

}  // namespace testing

#define LOGCATCHER_ASSERT_(failtype, level, action, ...)                   \
	::testing::LogCatcher(__FILE__, __LINE__, level, #action, __VA_ARGS__) \
	    .pass_through_and_assert(((action), ::testing::VoidWorkaround{}), failtype)
#define SPDLOG_ASSERT_NO_LOG -2
#define ASSERT_NO_LOG(action) LOGCATCHER_ASSERT_(true, SPDLOG_ASSERT_NO_LOG, action, "")
#define ASSERT_LOG(action, message) LOGCATCHER_ASSERT_(true, SPDLOG_LEVEL_OFF, action, message)
#define ASSERT_TRACE(action, message) LOGCATCHER_ASSERT_(true, SPDLOG_LEVEL_TRACE, action, message)
#define ASSERT_DEBUG(action, message) LOGCATCHER_ASSERT_(true, SPDLOG_LEVEL_DEBUG, action, message)
#define ASSERT_INFO(action, message) LOGCATCHER_ASSERT_(true, SPDLOG_LEVEL_INFO, action, message)
#define ASSERT_WARN(action, message) LOGCATCHER_ASSERT_(true, SPDLOG_LEVEL_WARN, action, message)
#define ASSERT_ERROR(action, message) LOGCATCHER_ASSERT_(true, SPDLOG_LEVEL_ERROR, action, message)
#define ASSERT_CRITICAL(action, message) \
	LOGCATCHER_ASSERT_(true, SPDLOG_LEVEL_CRITICAL, action, message)
#define EXPECT_NO_LOG(action) LOGCATCHER_ASSERT_(false, SPDLOG_ASSERT_NO_LOG, action, "")
#define EXPECT_LOG(action, message) LOGCATCHER_ASSERT_(false, SPDLOG_LEVEL_OFF, action, message)
#define EXPECT_TRACE(action, message) LOGCATCHER_ASSERT_(false, SPDLOG_LEVEL_TRACE, action, message)
#define EXPECT_DEBUG(action, message) LOGCATCHER_ASSERT_(false, SPDLOG_LEVEL_DEBUG, action, message)
#define EXPECT_INFO(action, message) LOGCATCHER_ASSERT_(false, SPDLOG_LEVEL_INFO, action, message)
#define EXPECT_WARN(action, message) LOGCATCHER_ASSERT_(false, SPDLOG_LEVEL_WARN, action, message)
#define EXPECT_ERROR(action, message) LOGCATCHER_ASSERT_(false, SPDLOG_LEVEL_ERROR, action, message)
#define EXPECT_CRITICAL(action, message) \
	LOGCATCHER_ASSERT_(false, SPDLOG_LEVEL_CRITICAL, action, message)

#define IGNORE_LOGS(action)                                                       \
	::testing::LogCatcher(__FILE__, __LINE__, SPDLOG_LEVEL_CRITICAL, #action, "") \
	    .ignore(((action), ::testing::VoidWorkaround{}))

#define LOGCATCHER_ASSERT_WHEN_(cond, failtype, level, action, ...)  \
	(cond ? LOGCATCHER_ASSERT_(failtype, level, action, __VA_ARGS__) \
	      : ((action), ::testing::VoidWorkaround{}))
#define ASSERT_NO_LOG_WHEN(cond, action) \
	LOGCATCHER_ASSERT_WHEN_(cond, true, SPDLOG_ASSERT_NO_LOG, action, "")
#define ASSERT_LOG_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, true, SPDLOG_LEVEL_OFF, action, message)
#define ASSERT_TRACE_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, true, SPDLOG_LEVEL_TRACE, action, message)
#define ASSERT_DEBUG_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, true, SPDLOG_LEVEL_DEBUG, action, message)
#define ASSERT_INFO_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, true, SPDLOG_LEVEL_INFO, action, message)
#define ASSERT_WARN_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, true, SPDLOG_LEVEL_WARN, action, message)
#define ASSERT_ERROR_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, true, SPDLOG_LEVEL_ERROR, action, message)
#define ASSERT_CRITICAL_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, true, SPDLOG_LEVEL_CRITICAL, action, message)
#define EXPECT_NO_LOG_WHEN(cond, action) \
	LOGCATCHER_ASSERT_WHEN_(cond, false, SPDLOG_ASSERT_NO_LOG, action, "")
#define EXPECT_LOG_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, false, SPDLOG_LEVEL_OFF, action, message)
#define EXPECT_TRACE_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, false, SPDLOG_LEVEL_TRACE, action, message)
#define EXPECT_DEBUG_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, false, SPDLOG_LEVEL_DEBUG, action, message)
#define EXPECT_INFO_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, false, SPDLOG_LEVEL_INFO, action, message)
#define EXPECT_WARN_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, false, SPDLOG_LEVEL_WARN, action, message)
#define EXPECT_ERROR_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, false, SPDLOG_LEVEL_ERROR, action, message)
#define EXPECT_CRITICAL_WHEN(cond, action, message) \
	LOGCATCHER_ASSERT_WHEN_(cond, false, SPDLOG_LEVEL_CRITICAL, action, message)
