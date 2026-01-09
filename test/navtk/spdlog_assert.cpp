#include <spdlog_assert.hpp>

#include <gtest/gtest-spi.h>

using spdlog::level::level_enum;
using std::regex;
using std::regex_search;
using std::shared_ptr;
using std::string;

namespace testing {

CaughtLog::CaughtLog(level_enum level, string text) : level(level), text(std::move(text)) {}

#define CASE_TO_STR(X) \
	case X:            \
		return #X
string spdlog_level_to_macro_name(int level) {
	switch (level) {
		CASE_TO_STR(SPDLOG_LEVEL_TRACE);
		CASE_TO_STR(SPDLOG_LEVEL_DEBUG);
		CASE_TO_STR(SPDLOG_LEVEL_INFO);
		CASE_TO_STR(SPDLOG_LEVEL_WARN);
		CASE_TO_STR(SPDLOG_LEVEL_ERROR);
		CASE_TO_STR(SPDLOG_LEVEL_CRITICAL);
	// Special case because we use SPDLOG_LEVEL_OFF to mean "any log"
	case SPDLOG_LEVEL_OFF:
		return "a";
	}
	std::ostringstream oss;
	oss << "<INVALID LOG LEVEL: " << level << ">";
	return oss.str();
}



void result_to_message(const AssertionResult& assertion_result,
                       bool fatal,
                       const char* file,
                       int line) {
	auto fail_result_type = fatal ? ::testing::TestPartResult::kFatalFailure
	                              : ::testing::TestPartResult::kNonFatalFailure;
	auto result_type = assertion_result ? ::testing::TestPartResult::kSuccess : fail_result_type;
	auto fail_msg    = assertion_result.message();
	GTEST_MESSAGE_AT_(file, line, fail_msg, result_type);
	if (fatal) throw AssertionException(TestPartResult(result_type, file, line, fail_msg));
}


void LogCatcher::Sink::flush_() {}
LogCatcher::Sink::Sink(LogCatcher* catcher) : spdlog::sinks::base_sink<Mutex>(), catcher(catcher) {}
void LogCatcher::Sink::sink_it_(const spdlog::details::log_msg& msg) {
	if (nullptr != catcher) {
		spdlog::memory_buf_t formatted;
		spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);
		catcher->logs.emplace_back(msg.level, fmt::to_string(formatted));
		auto& log = catcher->logs.back();
		if (catcher->level == SPDLOG_ASSERT_NO_LOG && !catcher->manager->forward_unexpected)
			catcher->have_log = true;
		else if ((log.level == catcher->level || catcher->level == level_enum::off) &&
		         regex_search(log.text, catcher->re))
			catcher->have_log = true;
		else if (catcher->manager->forward_unexpected)
			for (const auto& other_sink : catcher->manager->backup_sinks)
				if (other_sink.get() != this) other_sink->log(msg);
		// catcher->abort is used by error_mode_assert.hpp's expect_ub_or_die() to test functions
		// that we expect to log a message but then have undefined behavior. To guarantee that
		// undefined behavior isn't executed, we need to unwind the stack rather than allowing
		// execution to continue. To achieve that, we use `throw` with a specially-constructed
		// exception type that spdlog won't report as a "logger error."
		//
		// An alternative approach (potentially safer) would be to evaluate the function under test
		// within a forke()ed subprocess, but:
		// - Not all platforms can fork() (I'm looking at you, Windows.)
		// - We have issues with memory usage under ARM; fork() makes a copy of the memory space.
		if (catcher->have_log && catcher->abort) {
			throw LogCatcher::Aborting{
			    std::make_shared<LogCatcher::SpdlogExceptionForwarder>(catcher->logger)};
		}
	}
}


template <typename T, typename R>
bool any_is_type(const std::vector<std::shared_ptr<R>>& items) {
	for (const auto& item : items)
		if (nullptr != std::dynamic_pointer_cast<T>(item)) return true;
	return false;
}


LogCatcher::SinkManager::SinkManager(LogCatcher* catcher)
    : catcher(catcher),
      backup_sinks(catcher->logger->sinks()),
      forward_unexpected(!::SHOW_EXPECTED_LOGS && any_is_type<LogCatcher::Sink>(backup_sinks)) {
	if (!::SHOW_EXPECTED_LOGS) catcher->logger->sinks().clear();
	catcher->logger->sinks().push_back(std::make_shared<Sink>(catcher));
}

LogCatcher::SinkManager::~SinkManager() { detach(); }

void LogCatcher::SinkManager::detach() {
	if (have_detached) return;
	catcher->logger->sinks() = backup_sinks;
	have_detached            = true;
}


LogCatcher::LogCatcher(const char* file,
                       int line,
                       int level,
                       const std::string& action,
                       const std::string& pattern,
                       bool abort)
    : file(file),
      line(line),
      level(level),
      action(action),
      pattern(pattern),
      abort(abort),
      logger(spdlog::default_logger()),
      logs(),
      re(pattern, regex::icase),
      have_log(false),
      manager(std::make_unique<SinkManager>(this)) {}


LogCatcher::~LogCatcher() {
	if (!manager->have_detached) {
		detach_and_send_result(false);
	}
}

AssertionResult LogCatcher::build_result() const {
	if (have_log != (SPDLOG_ASSERT_NO_LOG == level)) return AssertionSuccess();
	auto out = AssertionFailure();
	if (SPDLOG_ASSERT_NO_LOG == level)
		out = out << "Expected no logs to be written";
	else
		out = out << "Expected " << spdlog_level_to_macro_name(level) << " log matching /"
		          << pattern << "/i";
	out = out << " from running { " << action << " }\n";
	if (!logs.empty()) {
		out = out << "Actual logs:\n";
		for (const auto& log : logs) out = out << log.text;
	} else
		out = out << "Actual logs: (none)";
	return out;
}

void LogCatcher::detach_and_send_result(bool fatal) const {
	const AssertionResult gtest_ar = this->build_result();
	manager->detach();
	if (abort || !gtest_ar) result_to_message(gtest_ar, fatal, file, line);
}

void spdlog_passthru_error_handler(const std::string&) { throw; }

LogCatcher::SpdlogExceptionForwarder::SpdlogExceptionForwarder(
    std::shared_ptr<spdlog::logger> logger)
    : logger(std::move(logger)) {
	this->logger->set_error_handler(&spdlog_passthru_error_handler);
}

LogCatcher::SpdlogExceptionForwarder::~SpdlogExceptionForwarder() {
	this->logger->set_error_handler(nullptr);
}

}  // namespace testing


namespace {

class ImprobableException {};

void throw_improbable_exception() {
	spdlog::error("This is an error");
	throw ImprobableException{};
}

void expect_error_wrapped_in_expect_throw(bool should_fail) {
	EXPECT_THROW(
	    EXPECT_ERROR(throw_improbable_exception(), should_fail ? "the wrong error" : "an error"),
	    ImprobableException);
}

}  // namespace


TEST(SpdlogAssertTests, ExpectWorksEvenWhenScopeAbortedByException) {
	expect_error_wrapped_in_expect_throw(false);
	EXPECT_NONFATAL_FAILURE(expect_error_wrapped_in_expect_throw(true),
	                        "log matching /the wrong error");
}
