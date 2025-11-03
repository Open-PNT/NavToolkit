#include <chrono>
#include <ostream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>
#include <misc_test_helpers.hpp>
#include <spdlog_assert.hpp>

#include <navtk/errors.hpp>

using namespace navtk;
using std::thread;
using std::this_thread::sleep_for;
using std::this_thread::yield;
using namespace std::chrono_literals;

// Because tests in this file tamper with the global error state, this test fixture saves and
// restores its value out of an abundance of caution.
class ErrorsTests : public ::testing::Test {
protected:
	ErrorMode original_global;
	void SetUp() override { original_global = get_global_error_mode(); }
	void TearDown() override { set_global_error_mode(original_global); }

public:
	// This member is used to make sure an ERROR_MODE_SENSITIVE_TEST can still (indirectly) access
	// stuff defined on the test fixture.
	std::string word = "hydrocephalic";
};

// unlock()/relock() are protected because they only exist for python support.
struct TestableErrorModeLock : ErrorModeLock {
	using ErrorModeLock::ErrorModeLock;
	using ErrorModeLock::relock;
	using ErrorModeLock::unlock;
};


ENUM_PRINT_TEST(TEST_F, ErrorsTests, ErrorMode, ErrorMode::OFF, ErrorMode::LOG, ErrorMode::DIE)

// The log_or_throw template takes varidic arguments and should use those to generate a formatted
// error message.
TEST_F(ErrorsTests, log_or_throw__BehavesLikeSpdlog) {
	EXPECT_ERROR(log_or_throw(ErrorMode::LOG, "one {} two {} three {}", 1, 2, 3),
	             "one 1 two 2 three 3");
}


// When an exception is thrown, its error message is also generated using the format string and
// arguments.
TEST_F(ErrorsTests, log_or_throw__GeneratesHumanReadableExceptions) {
	try {
		EXPECT_ERROR(log_or_throw(ErrorMode::DIE, "one {} two {} three {}", 1, 2, 3), "");
	} catch (std::runtime_error& re) {
		EXPECT_EQ(std::string(re.what()), "one 1 two 2 three 3");
	}
}


// Passing a custom template parameter can change the exception type
TEST_F(ErrorsTests, log_or_throw__CanThrowAnyType) {
	EXPECT_THROW(EXPECT_ERROR(log_or_throw<std::range_error>(ErrorMode::DIE, "kaboom"), "kaboom"),
	             std::range_error);

	// Check again using the "default mode" overload.
	{
		auto guard = ErrorModeLock(ErrorMode::DIE);
		EXPECT_THROW(EXPECT_ERROR(log_or_throw<std::range_error>("kaboom2"), "kaboom2"),
		             std::range_error);
	}
}

// Passing both an exception class and a level also works
TEST_F(ErrorsTests, log_or_throw__AcceptsBothTemplateParamsAtOnce) {
	EXPECT_THROW(EXPECT_INFO((log_or_throw<std::range_error, spdlog::level::level_enum::info>(
	                             ErrorMode::DIE, "highly customized")),
	                         "custom"),
	             std::range_error);

	// Check again using the "default mode" overload
	{
		auto guard = ErrorModeLock(ErrorMode::DIE);
		EXPECT_THROW(EXPECT_INFO((log_or_throw<std::range_error, spdlog::level::level_enum::info>(
		                             "highly customized")),
		                         "custom"),
		             std::range_error);
	}
}

// spdlog and fmt have some weird linker-related edge case with std::vector. This test ensures
// errors.hpp includes the workaround.
TEST_F(ErrorsTests, log_or_throw__HandlesVectors) {
	std::vector<std::string> vec{"one", "two", "three"};

	EXPECT_ERROR(log_or_throw(ErrorMode::LOG, "vec: {}", vec), "three");
}


// Make sure we can log a type that has operator<< but no direct support from fmt. This fails
// a static assertion unless errors.hpp also includes fmt/ostream.h
struct TypeWithLtLt {};
std::ostream& operator<<(std::ostream& os, const TypeWithLtLt&) { return os << "krimber"; }
TEST_F(ErrorsTests, log_or_throw__HandlesTypesWithOstreamLtLtOperator) {
	TypeWithLtLt target;
	EXPECT_ERROR(log_or_throw(ErrorMode::LOG, "{}", fmt::streamed(target)), "krimber");
}


// When no mode is passed in to log_or_throw, it should honor the global error mode.
TEST_F(ErrorsTests, log_or_throw__HonorsDefaultErrorMode) {
	{
		auto guard = ErrorModeLock(ErrorMode::DIE);
		EXPECT_THROW(EXPECT_ERROR(log_or_throw("kaboom"), "kaboom"), std::runtime_error);
	}

	{
		auto guard = ErrorModeLock(ErrorMode::LOG);
		EXPECT_ERROR(log_or_throw("whine"), "whine");
	}

	{
		auto guard = ErrorModeLock(ErrorMode::OFF);
		EXPECT_NO_LOG(log_or_throw("something"));
	}
}


// Passing in a mode as the first parameter to log_or_throw overrides the global error mode.
TEST_F(ErrorsTests, log_or_throw__OverrideDefaultErrorMode) {
	auto guard = ErrorModeLock(ErrorMode::OFF);
	EXPECT_THROW(EXPECT_ERROR(log_or_throw(ErrorMode::DIE, "kaboom"), "kaboom"),
	             std::runtime_error);
	EXPECT_ERROR(log_or_throw(ErrorMode::LOG, "whine"), "whine");
}


// Background thread used in ErrorsTests.set_global_error_mode__HonorsLock
void set_error_mode_when_unlocked(ErrorMode mode,
                                  std::shared_ptr<std::mutex> execution_order_mutex) {
	std::lock_guard<std::mutex> guard{*execution_order_mutex};
	set_global_error_mode(mode);
}

// Setting the global error mode blocks when other threads are holding ErrorModeLocks.
TEST_F(ErrorsTests, set_global_error_mode__HonorsLock) {
	// Setting ErrorMode::LOG here makes sure neither of the EXPECT_EQ's below can match by
	// accident.
	set_global_error_mode(ErrorMode::LOG);

	// This mutex is used to make sure my background thread code runs in the correct order. It's
	// initially locked, preventing the background thread we're about to create from doing anything.
	auto execution_order_mutex = std::make_shared<std::mutex>();
	std::unique_lock<std::mutex> execution_order_lock{*execution_order_mutex};

	// The set_global_error_mode call inside this thread should block as long as the guard
	// introduced in the next brace exists.
	thread bg(set_error_mode_when_unlocked, ErrorMode::OFF, execution_order_mutex);

	{
		// Hold this lock, preventing ErrorMode from changing to something other than ErrorMode::DIE
		auto guard = ErrorModeLock(ErrorMode::DIE);

		// Releasing execution_order_lock allows set_error_mode_when_unlocked's guard to lock the
		// mutex, meaning its call to set_global_error_mode can proceed. If it is correctly honoring
		// locks, this will have no effect until guard goes out of scope.
		execution_order_lock.unlock();

		// Make sure the background thread has started by waiting until we're unable to immediately
		// lock the mutex.
		for (yield(); execution_order_lock.try_lock(); sleep_for(1ms))
			execution_order_lock.unlock();

		// Because we're still holding a lock on the global ErrorMode, the background thread should
		// get stuck trying to set ErrorMode::OFF.
		EXPECT_EQ(get_global_error_mode(), ErrorMode::DIE);

		// This closing brace releases the guard, which unlocks the global error mode mutex.
	}

	// Give the background thread time to finish.
	bg.join();

	// And so we expect the error mode to change by the time we get here.
	EXPECT_EQ(get_global_error_mode(), ErrorMode::OFF);
}

// ErrorModeLock undoes its changes to the global error mode when it goes out of scope.
TEST_F(ErrorsTests, ErrorModeLock__SaveAndRestore) {
	set_global_error_mode(ErrorMode::OFF);
	{
		ErrorModeLock guard{ErrorMode::LOG};
		EXPECT_EQ(get_global_error_mode(), ErrorMode::LOG);
	}
	EXPECT_EQ(get_global_error_mode(), ErrorMode::OFF);
}


// In order to support the python bindings context manager behavior (__exit__), we have to expose an
// explicit .unlock() method.
TEST_F(ErrorsTests, ErrorModeLock__ExplicitUnlock) {
	set_global_error_mode(ErrorMode::OFF);

	auto guard = TestableErrorModeLock(ErrorMode::LOG);
	EXPECT_EQ(get_global_error_mode(), ErrorMode::LOG);

	// Trying to change the error mode from another thread should block until our guard is unlocked.
	thread bg(set_error_mode_when_unlocked, ErrorMode::DIE, std::make_shared<std::mutex>());

	// Make sure the background thread has had a chance to get stuck.
	sleep_for(2ms);
	EXPECT_EQ(get_global_error_mode(), ErrorMode::LOG);

	// Unlock should behave the same as destroying the guard, since we have no reliable way of
	// controlling when the destructor is called.
	guard.unlock();

	// If the .unlock didn't work, the test will deadlock on this line.
	bg.join();
	EXPECT_EQ(get_global_error_mode(), ErrorMode::DIE);
}


// Because there's no way to stop python users from re-using a context manager, __enter__ must work
// even if __exit__ has already been called. The protected relock method supports this case.
TEST_F(ErrorsTests, ErrorModeLock__ExplicitRelock) {
	set_global_error_mode(ErrorMode::OFF);

	auto guard = std::make_unique<TestableErrorModeLock>(ErrorMode::LOG);
	EXPECT_EQ(get_global_error_mode(), ErrorMode::LOG);
	guard->unlock();
	EXPECT_EQ(get_global_error_mode(), ErrorMode::OFF);

	// Setting the global error mode in a background thread should just work, immediately, because
	// we've unlocked the guard.
	{
		thread bg(set_error_mode_when_unlocked, ErrorMode::DIE, std::make_shared<std::mutex>());
		bg.join();
		EXPECT_EQ(get_global_error_mode(), ErrorMode::DIE);
	}

	guard->relock();

	// Re-lock should've re-set the global error mode to the target mode passed in at the
	// constructor.
	EXPECT_EQ(get_global_error_mode(), ErrorMode::LOG);

	// It should also be holding the lock, causing a new background thread to block rather than set
	// a new error mode.
	thread bg(set_error_mode_when_unlocked, ErrorMode::DIE, std::make_shared<std::mutex>());
	// Make sure the background thread has had a chance to get stuck.
	sleep_for(2ms);
	EXPECT_EQ(get_global_error_mode(), ErrorMode::LOG);

	// Delete the guard to release the lock.
	guard.reset();

	// If deleting the guard didn't release the lock, the test will deadlock on this line.
	bg.join();
	EXPECT_EQ(get_global_error_mode(), ErrorMode::DIE);
}


// ErrorModeLock's should be std::move-able
TEST_F(ErrorsTests, ErrorModeLock__std_move) {
	set_global_error_mode(ErrorMode::OFF);
	auto from = std::make_unique<ErrorModeLock>(ErrorMode::LOG, true);
	auto to   = std::make_unique<ErrorModeLock>(std::move(*from));

	// Moving the lock should not have changed the error mode.
	EXPECT_EQ(get_global_error_mode(), ErrorMode::LOG);

	// *from has now been moved-from, so even though restore is enabled, destroying it shouldn't
	// restore yet.
	from.reset();
	EXPECT_EQ(get_global_error_mode(), ErrorMode::LOG);

	// deleting *to should trigger the original restore
	to.reset();
	EXPECT_EQ(get_global_error_mode(), ErrorMode::OFF);
}


// Setting the global error mode from the same thread that holds an ErrorModeLock works, and doesn't
// break ErrorModeLock's mode-restoring feature.
TEST_F(ErrorsTests, set_global_Error_mode__SameThreadAsLock) {
	set_global_error_mode(ErrorMode::OFF);
	{
		auto guard = ErrorModeLock(ErrorMode::LOG);
		EXPECT_EQ(get_global_error_mode(), ErrorMode::LOG);

		set_global_error_mode(ErrorMode::DIE);
		EXPECT_EQ(get_global_error_mode(), ErrorMode::DIE);
	}
	EXPECT_EQ(get_global_error_mode(), ErrorMode::OFF);
}


// A function that holds an ErrorModeLock should be able to call another function that holds an
// ErrorModeLock without causing a deadlock.
TEST_F(ErrorsTests, ErrorModeLock__RecursiveLock) {
	auto outer_guard = ErrorModeLock(ErrorMode::LOG);
	EXPECT_EQ(get_global_error_mode(), ErrorMode::LOG);
	{
		// A deadlock here represents a failure of the test because the wrong lock type is being
		// used behind the scenes. This is deadlocking with outer_guard above.
		auto inner_guard = ErrorModeLock(ErrorMode::DIE);
		EXPECT_EQ(get_global_error_mode(), ErrorMode::DIE);
	}
	// Because the inner guard has gone out-of-scope, the previous global error mode should've been
	// restored.
	EXPECT_EQ(get_global_error_mode(), ErrorMode::LOG);
}


// Everything above was written in terms of gtest-provided macros. Here's a single test written
// using the ERROR_MODE_SENSITIVE_TEST and EXPECT_HONORS_MODE macros, just to make sure the macros
// work. Uncomment lines to test various failure modes.
ERROR_MODE_SENSITIVE_TEST(TEST_F, ErrorsTests, log_or_throw__TestUsingMagicMacros) {
	// Fails LOG and DIE conditions because 3 neither logs nor dies.
	// EXPECT_HONORS_MODE(3, "anything");

	// Fails OFF and DIE conditions because the log message is always written, but no exception
	// is ever thrown
	// EXPECT_HONORS_MODE(spdlog::error("nooooo"), "no");

	// Fails all three, because the exception is supposed to also trigger a log message.
	// EXPECT_HONORS_MODE(throw std::runtime_error("sprained ankle"), "ankle");

	// Finally, the correct behavior
	EXPECT_HONORS_MODE(log_or_throw("magic word: {}", test.word), "hydro");
}


// EXPECT_HONORS_MODE_PARAM expects its action to ignore the global mode and honor its parameter
// instead.
ERROR_MODE_SENSITIVE_TEST(TEST_F, ErrorsTests, log_or_throw__TestExpectHonorsModeParam) {
	EXPECT_HONORS_MODE_PARAM(log_or_throw(mode, "magic word: {}", test.word), "hydro");
}


// Create a type that is impossible to construct to make sure error_mode_assert can deal with
// uncooperative return types
class ImpossibleType {
private:
	ImpossibleType()                      = default;
	ImpossibleType(const ImpossibleType&) = delete;
	ImpossibleType(ImpossibleType&&)      = delete;
};

// A method that, to the compiler, looks like it returns ImpossibleType, used in the expression
// passed to the EXPECT_HONORS_MODE call below to force the ExpressionThrowTester to handle types
// that it can't instantiate.
ImpossibleType pretend_to_return_impossible() {
	log_or_throw("expected");
	throw std::runtime_error("Shouldn't reach this line.");
}

// Call EXPECT_HONORS_MODE but prevent it from aborting the test. Return true if it tried to abort.
bool expect_honors_mode_tries_to_abort() {
	try {
		EXPECT_HONORS_MODE(pretend_to_return_impossible(), "expected");
		return false;
	} catch (::testing::AssertionException&) {
		return true;
	}
}

// When EXPECT_HONORS_MODE encounters the exception it was expecting,
TEST_F(ErrorsTests, ExpressionThrowTester__can_cope_with_non_constructable_value) {
	auto guard = ErrorModeLock{ErrorMode::DIE};
	EXPECT_TRUE(EXPECT_WARN(expect_honors_mode_tries_to_abort(), "cannot be default-constructed"));
}


// Create a type that fmt::format can't handle (for testing error returns) and a static instance of
// that type for testing reference returns.
struct OpaqueType {
	int member_variable = -1;

	// Delete copy & move to make sure my code below can _only_ deal in references.
	OpaqueType()                  = default;
	OpaqueType(const OpaqueType&) = delete;
	OpaqueType(OpaqueType&&)      = delete;
};

// Some globals for return_a_reference to return
static OpaqueType the_opaque_thing;
static std::string the_non_opaque_thing = "I'm a string!";

// Returns a reference type to make sure reference types can pass through the ExpressionThrowTester
auto& return_a_reference() {
	// Comment this out to deliberately fail can_cope_with_reference_types to allow you to inspect
	// the generated message (to test `describe_return_value`)
	log_or_throw("expected");

	// Return the_opaque_thing if you'd like to see the failure message when format::fmt can't print
	// objects of the returned type. Return the_non_opaque_thing to see the failure message when the
	// return type is printable.
	return the_opaque_thing /* or the_non_opaque_thing */;
}

// Some helper functions that allow the test below to "work" regardless of which lines of
// return_a_reference are commented/uncommented
auto address_of_global(const OpaqueType&) { return &the_opaque_thing; }
auto address_of_global(const std::string&) { return &the_non_opaque_thing; }
bool hasnt_corrupted_memory(const OpaqueType& value) { return value.member_variable == -1; }
bool hasnt_corrupted_memory(const std::string& value) {
	// in ErrorMode::DIE we're expecting a default-constructed string. Otherwise, the global.
	return value == (get_global_error_mode() == ErrorMode::DIE ? "" : "I'm a string!");
}

// When EXPECT_HONORS_MODE is called with an expression that returns a reference type, it needs to
// pass through that reference type unmodified whenever it can. When it encounters an exception, it
// needs to default-construct a value that has a long-lived location in memory (to prevent it from
// accidentally returning a reference to its own stack).
//
// See the comments of return_a_reference to inspect messages in various failure modes.
ERROR_MODE_SENSITIVE_TEST(TEST_F, ErrorsTests, ExpressionThrowTester__reference_types) {
	decltype(auto) passed_through_opaque = EXPECT_HONORS_MODE(return_a_reference(), "expected");
	if (mode == ErrorMode::DIE) {
		// We expect that return_a_reference has thrown rather than returned, so
		// passed_through_opaque should be a default-constructed value generated by the tester
		// itself.
		EXPECT_NE(&passed_through_opaque, address_of_global(passed_through_opaque));
	} else {
		// Whenever return_a_reference returns, we want the exact memory address to match.
		EXPECT_EQ(&passed_through_opaque, address_of_global(passed_through_opaque));
	}

	// Early attempts at solving the ExpressionThrowTester's problem with reference types caused
	// abundant stack-use-after-return issues. This catches those even on platforms without ASAN.
	EXPECT_TRUE(hasnt_corrupted_memory(passed_through_opaque));
}


// This function logs a warning, and then does a log_or_throw. It's used to make sure the harness
// has a way of dealing with such functions.
void warn_then_explode() {
	spdlog::warn("I'm warning you!");
	navtk::log_or_throw("Well, that's it for me.");
}

// The inner expect swallows the warning, but passes anything else through to the outer expect.
ERROR_MODE_SENSITIVE_TEST(TEST_F, ErrorsTests, ExpressionThrowTester__nested_log_first) {
	EXPECT_HONORS_MODE(EXPECT_WARN(warn_then_explode(), "you"), "me");
}

// Testing the behavior when the order is reversed.
void explode_then_warn() {
	navtk::log_or_throw("Well, that's it for me.");
	spdlog::warn("I'm warning you!");
}

// In this case, the warning doesn't get written in ErrorMode::DIE because the line is unreachable.
ERROR_MODE_SENSITIVE_TEST(TEST_F, ErrorsTests, ExpressionThrowTester__nested_log_last) {
	EXPECT_WARN_WHEN(mode != ErrorMode::DIE, EXPECT_HONORS_MODE(explode_then_warn(), "me"), "you");
}
