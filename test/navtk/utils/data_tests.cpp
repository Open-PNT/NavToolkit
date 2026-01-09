#include <functional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <error_mode_assert.hpp>

#include <navtk/utils/data.hpp>

using navtk::ErrorMode;
using std::function;
using std::string;
using std::vector;
using namespace navtk::utils;
using namespace navtk::utils::detail;

// These forward-declares allow me to unit test helper functions in navtk/utils/data.cpp without
// exposing them to the API. Ordinarily, one would not unit-test implementation details like this,
// but the behavior of navtk::utils::open_data_file is influenced by operating system, build
// flags, environment variables, and user accounts. None of those things can be set to fixed, known
// values from gtest in a cross-platform way, so this is a best-effort to cover as much of the code
// as possible from whatever environment this test's compiler targets.
namespace navtk {
namespace utils {
namespace detail {
string getenv_str(const string& key);
string home_folder();
string expand_user(const string& path);
vector<string> split_paths(const string& paths, char delim);
vector<string> env_path_or_default(const string& env_key, const string& fallback);
string label_to_env_key(const string& label);
bool is_absolute_path(const string& path);
string join_paths(const string& left, const string& right);
}  // namespace detail
}  // namespace utils
}  // namespace navtk

// Some platform-specific nonsense. We have to define our own setenv/unsetenv alternative because
// Windows calls setenv `_putenv_s`, and mingw tries to "help" by `#define setenv _putenv_s` even
// though the signatures aren't a perfect match.
#ifdef _WIN32
#	define DIR_SLASH "\\"
#	define ROOT_FOLDER "q:" DIR_SLASH
#	define PATH_DELIMITER ';'
inline static int set_env(const char* name, const char* value) {
	if (nullptr == value) return _putenv_s(name, "");
	return _putenv_s(name, value);
}
#	define setenv _putenv_s
#else
#	define DIR_SLASH "/"
#	define ROOT_FOLDER DIR_SLASH
#	define PATH_DELIMITER ':'
inline static int set_env(const char* name, const char* value) {
	if (nullptr == value || value[0] == '\0') return unsetenv(name);
	return setenv(name, value, 1);
}
#endif


// -----------------------------------------------------------------------------------------------


// Make sure I can actually read environment variables on this platform
TEST(DataUtils, getenv_str__ReturnsEnvironmentVariable) {
	set_env("TEST_ENV_VAR", "Rhapsodic");
	EXPECT_EQ("Rhapsodic", getenv_str("TEST_ENV_VAR"));
}

// Make sure unset environment variables aren't crashy
TEST(DataUtils, getenv_str__ReturnsEmptyStringForUnset) {
	const char* var_key = "HIGHLY_IMPROBABLE_THIS_IS_SET_IN_YOUR_ENVIRONMENT";
	set_env(var_key, nullptr);
	EXPECT_EQ("", getenv_str(var_key));
}


// -----------------------------------------------------------------------------------------------


// There's no reasonable way to unit-test navtk::utils::home_folder(). There is an attempt, in the
// python bindings test, to use open_data_file to indirectly compare home_folder() to
// `os.path.expanduser('~')`, but in the context of C++ there's no good source of truth.
static const string HOME = home_folder();


// -----------------------------------------------------------------------------------------------


// In the simplest case, expand_user gets called with a path that does not contain any of the magic
// strings it is supposed to expand, and it just returns the path that was passed in.
TEST(DataUtils, expand_user__PassesThroughPathsWithNoUser) {
	EXPECT_EQ("", expand_user(""));
	EXPECT_EQ("foo", expand_user("foo"));
	EXPECT_EQ("/usr/local/bin", expand_user("/usr/local/bin"));
}

// If the path consists _entirely_ of a $HOME reference, expand_user should be equivalent to
// home_folder.
TEST(DataUtils, expand_user__HandlesHomeOnlyPath) {
	EXPECT_EQ(HOME, expand_user("~"));
	EXPECT_EQ(HOME, expand_user("$HOME"));
	EXPECT_EQ(HOME, expand_user("%UserProfile%"));
}

// expand_user("$HOME/") should preserve the trailing slash
TEST(DataUtils, expand_user__PreservesTrailingSlash) {
	auto expected = ::fmt::format("{}" DIR_SLASH, HOME);
	EXPECT_EQ(expected, expand_user("~" DIR_SLASH));
	EXPECT_EQ(expected, expand_user("$HOME" DIR_SLASH));
	EXPECT_EQ(expected, expand_user("%UserProfile%" DIR_SLASH));
}

// Because Windows environment variables are case-insensitive, all of these different forms of
// %UserProfile% need to work.
TEST(DataUtils, expand_user__AllowsUserProfileToBeOddlyCased) {
	EXPECT_EQ(HOME, expand_user("%USERPROFILE%"));
	EXPECT_EQ(HOME, expand_user("%userprofile%"));
	EXPECT_EQ(HOME, expand_user("%UsErPrOfIlE%"));
}

// A home reference in the middle of the path makes no sense and should be ignored.
TEST(DataUtils, expand_user__IgnoresNonLeadingReferences) {
	vector<string> forms{"~", "$HOME", "%UserProfile%"};
	for (const auto& home_name : forms) {
		string malformed =
		    fmt::format("{0}path{1}with{1}{2}{1}in{1}it", ROOT_FOLDER, DIR_SLASH, home_name);
		EXPECT_EQ(malformed, expand_user(malformed));
	}
}

// A more practical use is something like expand_user("~/.local/share"), a longer path of which the
// home folder is only the first part.
TEST(DataUtils, expand_user__ExpandsLongerPaths) {
	vector<string> forms{"~", "$HOME", "%UserProfile%"};
	for (const auto& home_name : forms) {
		string supplied = fmt::format("{0}{1}used{1}correctly", home_name, DIR_SLASH);
		string expected = fmt::format("{0}{1}used{1}correctly", HOME, DIR_SLASH);
		EXPECT_EQ(expected, expand_user(supplied)) << "Using name " << home_name;
	}
}

// While this is a valid bashism, we explicitly don't want to support ~name user lookups, since, in
// context, it would mean somebody is building a system which depends on one user being allowed to
// trash another user's $HOME, and that's just anarchy.
TEST(DataUtils, expand_user__RefusesToLookUpOtherUsers) {
	EXPECT_EQ("~root/.bashrc", expand_user("~root/.bashrc"));
}


// -----------------------------------------------------------------------------------------------

// If split_paths encounters an empty string, it returns an empty vector (not a vector containing an
// empty string)
TEST(DataUtils, split_paths__EmptyVectorForEmptyPath) { EXPECT_EQ(0, split_paths("", ':').size()); }

// Given an input string that does not contain a delimiter, split_paths returns a single-element
// vector containing that string.
TEST(DataUtils, split_paths__SingleItem) {
	EXPECT_EQ("foo/bar/baz", split_paths("foo/bar/baz", ':').at(0));
}

// Here's the expected use case of split_paths -- input is multiple paths separated by a delimiter;
// output is a vector containing each individual path.
TEST(DataUtils, split_paths__MultiItem) {
	auto vec = split_paths("/foo/bar:/usr/foo/bar", ':');
	EXPECT_EQ(2, vec.size());
	EXPECT_EQ("/foo/bar", vec.at(0));
	EXPECT_EQ("/usr/foo/bar", vec.at(1));
}

// Empty paths are meaningless, and usually indicate somebody goofed up their profile variables, so
// split_paths should remove them from its results.
TEST(DataUtils, split_paths__ElidesEmpty) {
	// the empty path in the middle is meaningless, so it gets removed
	auto empty_in_middle = split_paths("/foo/bar::/usr/foo/bar", ':');
	EXPECT_EQ(2, empty_in_middle.size());
	EXPECT_EQ("/foo/bar", empty_in_middle.at(0));
	EXPECT_EQ("/usr/foo/bar", empty_in_middle.at(1));

	// Same again, but this time the empty string is at the end
	auto empty_at_end = split_paths("/foo/bar:/usr/foo/bar:", ':');
	EXPECT_EQ(2, empty_at_end.size());
	EXPECT_EQ("/foo/bar", empty_at_end.at(0));
	EXPECT_EQ("/usr/foo/bar", empty_at_end.at(1));

	// Empty string at the beginning
	auto empty_at_start = split_paths(":/foo/bar:/usr/foo/bar", ':');
	EXPECT_EQ(2, empty_at_start.size());
	EXPECT_EQ("/foo/bar", empty_at_start.at(0));
	EXPECT_EQ("/usr/foo/bar", empty_at_start.at(1));

	// That one user who always manages to do something like this
	auto silly = split_paths("::::/foo/bar::::::/usr/foo/bar:::::", ':');
	EXPECT_EQ(2, silly.size());
	EXPECT_EQ("/foo/bar", silly.at(0));
	EXPECT_EQ("/usr/foo/bar", silly.at(1));
}

// To make sure the "elides empty" logic isn't off-by-one, here's a . in the path at multiple
// positions.
TEST(DataUtils, split_paths__SingleCharPath) {
	EXPECT_EQ(3, split_paths("/a:/b:.", ':').size());
	EXPECT_EQ(3, split_paths(".:/a:/b", ':').size());
	EXPECT_EQ(3, split_paths("/a:.:/b", ':').size());
}


// -----------------------------------------------------------------------------------------------


// When both env var and fallback are provided, and the environment variable is set, its value is
// the preferred return for env_path_or_default.
TEST(DataUtils, env_path_or_default__PrefersEnvVar) {
	auto env_name = "ENV_VAR_NAME";
	set_env(env_name, "facepalm");
	auto vec = env_path_or_default(env_name, "fallback");
	EXPECT_EQ(1, vec.size());
	EXPECT_EQ("facepalm", vec.at(0));
}

// When both are provided but the env var is unset, the fallback value is used
TEST(DataUtils, env_path_or_default__FallsBackWhenUnsetEnv) {
	auto env_name = "ENV_VAR_NAME";
	set_env(env_name, nullptr);
	auto vec = env_path_or_default(env_name, "fallback");
	EXPECT_EQ(1, vec.size());
	EXPECT_EQ("fallback", vec.at(0));
}

// When the env var isn't provided, the fallback value is used.
TEST(DataUtils, env_path_or_default__FallsBackWhenNoEnvVarSpecified) {
	auto env_name = "";
	set_env(env_name, "facepalm");  // probably has no effect, but just in case.
	auto vec = env_path_or_default(env_name, "fallback");
	EXPECT_EQ(1, vec.size());
	EXPECT_EQ("fallback", vec.at(0));
}

// If env_path_or_default encounters a path that contains the os's path delimiters, whether it be
// env var or fallback, its return is split on the delimiter.
TEST(DataUtils, env_path_or_default__SplitsPaths) {
	auto env_value  = fmt::format("foo{0}bar{0}baz", PATH_DELIMITER);
	auto fall_value = fmt::format("FOO{0}BAR{0}BAZ", PATH_DELIMITER);
	auto env_name   = "ENV_VAR_NAME";

	// Try once with the env var set
	{
		set_env(env_name, env_value.c_str());
		auto env_vec = env_path_or_default(env_name, fall_value);
		EXPECT_EQ(3, env_vec.size());
		EXPECT_EQ("foo", env_vec.at(0));
		EXPECT_EQ("bar", env_vec.at(1));
		EXPECT_EQ("baz", env_vec.at(2));
	}

	// And once via the fallback
	{
		set_env(env_name, nullptr);
		auto fallback_vec = env_path_or_default(env_name, fall_value);
		EXPECT_EQ(3, fallback_vec.size());
		EXPECT_EQ("FOO", fallback_vec.at(0));
		EXPECT_EQ("BAR", fallback_vec.at(1));
		EXPECT_EQ("BAZ", fallback_vec.at(2));
	}
}


// -----------------------------------------------------------------------------------------------


// The environment variable for "the foo file" is NAVTK_FOO_PATH
TEST(DataUtils, label_to_env_key__WrapsWithPrefixSuffix) {
	EXPECT_EQ("NAVTK_FOO_PATH", label_to_env_key("foo"));
}

// Because labels passed into open_data_file are intended to be human readable, they may be more
// than one word and may include punctuation, but those things are invalid as parts of an
// environment variable name.
TEST(DataUtils, label_to_env_key__UnderscoresOutBadCharacters) {
	// "Cannot open Dr. Raquet's Matlab file"
	EXPECT_EQ("NAVTK_DR_RAQUET_S_MATLAB_PATH", label_to_env_key("Dr. Raquet's MATLAB"));
}


// -----------------------------------------------------------------------------------------------


// Every wheel must be reinvented
TEST(DataUtils, is_absolute_path__KnowsWhetherAPathIsAbsolute) {
	EXPECT_FALSE(is_absolute_path("foo" DIR_SLASH "bar"));
	EXPECT_TRUE(is_absolute_path(ROOT_FOLDER "foo" DIR_SLASH "bar"));
}


// -----------------------------------------------------------------------------------------------


// The naive approach to join_paths is to just return left + slash + right. Sometimes that's the
// correct behavior.
TEST(DataUtils, join_paths__ObviousCase) {
	EXPECT_EQ("foo" DIR_SLASH "bar", join_paths("foo", "bar"));
}

// If the left path already ends with a slash, join shouldn't add its own slash
TEST(DataUtils, join_paths__WhenLeftHasTrailingSlash) {
	EXPECT_EQ("foo" DIR_SLASH "bar", join_paths("foo" DIR_SLASH, "bar"));
}

// If either path is empty, join shouldn't add a slash
TEST(DataUtils, join_paths__OnePathEmpty) {
	EXPECT_EQ("foo", join_paths("", "foo"));
	EXPECT_EQ("foo", join_paths("foo", ""));
}

// If path on the right is absolute, then sticking something else in front of it is invalid.
TEST(DataUtils, join_paths__AbsolutePath) {
#ifdef _WIN32
	auto abspath = "C:\\Windows\\System32";
#else
	auto abspath = "/usr/lib";
#endif
	EXPECT_EQ(abspath, join_paths("something-else", abspath));
	EXPECT_EQ(abspath, join_paths(abspath, abspath));
	EXPECT_EQ(fmt::format("{}{}something-else", abspath, DIR_SLASH),
	          join_paths(abspath, "something-else"));
}


// -----------------------------------------------------------------------------------------------


// When the single-file environment variable isn't set, visit_possible_file_paths emits an error
// message that includes both the human-readable label, and the expected basename it sought.
ERROR_MODE_SENSITIVE_TEST(TEST, DataUtils, visit_possible_file_paths__NoMatchNoEnvVar) {
	set_env("NAVTK_SOME_LABEL_PATH", nullptr);
	EXPECT_HONORS_MODE_PARAM(
	    visit_possible_file_paths(
	        mode, "some label", "foo.txt", [](const string&) -> bool { return false; }),
	    "some label.*foo.txt");
}

// When the single-file environment variable _is_ set, but points at something we can't open,
// visit_possible_file_paths emits an error that includes the env var's value.
ERROR_MODE_SENSITIVE_TEST(TEST, DataUtils, visit_possible_file_paths__NoMatchWithEnvVar) {
	set_env("NAVTK_SOME_LABEL_PATH", ROOT_FOLDER "fake" DIR_SLASH "file.txt");
	EXPECT_HONORS_MODE_PARAM(
	    visit_possible_file_paths(
	        mode, "some label", "foo.txt", [](const string&) -> bool { return false; }),
	    "file.txt");
}

// When the single-file path var is set, it is the only path searched for.
TEST(DataUtils, visit_possible_file_paths__WhenEnvVarSetOnlyChecksThere) {
	vector<string> places_searched;
	set_env("NAVTK_SOME_LABEL_PATH", ROOT_FOLDER "fake" DIR_SLASH "file.txt");
	visit_possible_file_paths(
	    ErrorMode::OFF, "some label", "foo.txt", [&](const string& path) -> bool {
		    places_searched.emplace_back(path);
		    return false;
	    });
	EXPECT_EQ(1, places_searched.size());
	EXPECT_EQ(ROOT_FOLDER "fake" DIR_SLASH "file.txt", places_searched.at(0));
}

// When the single-file path var is unset, one or more paths (depending entirely on the system) get
// searched.  This test is imprecise because we can predict very little about what is correct for
// the user's environment.
TEST(DataUtils, visit_possible_file_paths__WhenEnvVarUnset) {
	vector<string> places_searched;
	set_env("NAVTK_SOME_LABEL_PATH", nullptr);
	visit_possible_file_paths(
	    ErrorMode::OFF, "some label", "foo.txt", [&](const string& path) -> bool {
		    places_searched.emplace_back(path);
		    return false;
	    });

	// Because there's a hardcoded "current working directory" in there, we know there must be at
	// least one path searched, or something has gone horribly wrong.
	EXPECT_GT(places_searched.size(), 0);

	// Only the single-file path variable is allowed to override the file's basename, so every path
	// we checked should've ended with "/foo.txt"
	for (const auto& place : places_searched)
		EXPECT_TRUE(std::regex_search(place, std::regex("\\" DIR_SLASH "foo.txt$")));
}


// -----------------------------------------------------------------------------------------------


// When asked to look up a file that doesn't exist, log or throw based on the given error mode.
ERROR_MODE_SENSITIVE_TEST(TEST, DataUtils, open_data_file__HonorsModeParam) {
	auto improbable_filename = "aoteuhosnetuhaosnetuhaosnetuhaoesnuthoasnetuhoa";
	EXPECT_EQ(nullptr,
	          EXPECT_HONORS_MODE_PARAM(open_data_file(mode, "foo bar", improbable_filename),
	                                   "NAVTK_FOO_BAR_PATH"));
}

// If no error mode was given, use the global error mode.
ERROR_MODE_SENSITIVE_TEST(TEST, DataUtils, open_data_file__HonorsGlobalMode) {
	auto improbable_filename = "aoteuhosnetuhaosnetuhaosnetuhaoesnuthoasnetuhoa";
	EXPECT_EQ(
	    nullptr,
	    EXPECT_HONORS_MODE(open_data_file("foo bar", improbable_filename), "NAVTK_FOO_BAR_PATH"));
}
