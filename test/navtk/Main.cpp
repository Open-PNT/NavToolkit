#include <unistd.h>
#include <cstring>

#include <gtest/gtest.h>
#include <spdlog/cfg/argv.h>

#include <navtk/errors.hpp>

// Gtest #define's GTEST_SKIP as a 'return' expression rather than a 'throw',
// which means that it won't skip tests when called by helper functions, just
// exit the helper function. This listener causes an exception to be thrown
// when GTEST_SKIP is called regardless of where in the stack the call is made.
struct StopTestOnSkip : testing::EmptyTestEventListener {
	void OnTestPartResult(const testing::TestPartResult& result) override {
		if (result.type() == ::testing::TestPartResult::kSkip)
			throw testing::AssertionException(result);
	}
};


int main(int argc, char** argv) {
	spdlog::cfg::load_argv_levels(argc, argv);

	// Some tests depend on the presence of file hour1190.18n, which will be copied to the same
	// folder as this test program, so do the c++ equivalent of cd $(dirname $0) so the tests work
	// even if executed from a different folder.
	if (argc) {
		// walk through argv[0] backwards, looking for any slashes that would indicate a subfolder
		for (auto ii = strlen(argv[0]); ii-- > 0;) {
			if (argv[0][ii] == '/') {
				argv[0][ii] = '\0';  // Change the / to an end-of-string marker. poor-man's dirname
				auto result = chdir(argv[0]);
				if (result != 0) {
					perror("chdir() failed");
				}
				argv[0][ii] = '/';  // change it back in case later code reads argv
				break;
			}
			// Handle mingw case also
			else if (argv[0][ii] == '\\') {
				argv[0][ii] = '\0';  // Change the / to an end-of-string marker. poor-man's dirname
				auto result = chdir(argv[0]);
				if (result != 0) {
					perror("chdir() failed");
				}
				argv[0][ii] = '\\';  // change it back in case later code reads argv
				break;
			}
		}
	}

	// Disable error checking here to fail legacy tests. A test that depends on error behavior
	// should hold an ErrorModeLock or use one of the special error testing macros.
	::navtk::set_global_error_mode(::navtk::ErrorMode::OFF);

	::testing::InitGoogleTest(&argc, argv);
	::testing::UnitTest::GetInstance()->listeners().Append(new StopTestOnSkip);
	return RUN_ALL_TESTS();
}
