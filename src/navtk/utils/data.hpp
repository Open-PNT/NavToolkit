#pragma once

#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <navtk/errors.hpp>

/**
 * Name of the environment variable to read when looking for a `NAVTK_DATA_DIR`. If this
 * environment variable is unset, or set to a folder that does not contain the file being searched
 * for, navtk::utils::open_data_file will check a series of default locations.
 */
#define NAVTK_DATA_DIR_ENV_VAR "NAVTK_DATA_DIR"

/**
 * Prefix for file-specific environment variables. If navtk::utils::open_data_file tries to open a
 * file labeled "foo", it will check for the presence of an environment variable called
 * `NAVTK_FOO_PATH`
 */
#define NAVTK_DATA_FILE_ENV_VAR_PREFIX "NAVTK_"

/**
 * Suffix for file-specific environment variables. If navtk::utils::open_data_file tries to open a
 * file labeled "foo", it will check for the presence of an environment variable called
 * `NAVTK_FOO_PATH`
 */
#define NAVTK_DATA_FILE_ENV_VAR_SUFFIX "_PATH"

/**
 * Subfolder of the various OS data folders (such as `/usr/share` etc.) to search for navtk data
 * files.
 */
#define NAVTK_OS_DATA_DIR_SUBFOLDER_NAME "navtk"

namespace navtk {
namespace utils {

// This is exposed in the API rather than forward-declared because it's used by the Python bindings
// to reimplement navtk::utils::open_data_file in terms of Python file objects rather than
// ifstreams.
namespace detail {

/**
 * Pass a series of possible paths for a data file to the given `try_path` function, stopping if it
 * ever returns `true`. If it always returned `false` and there are no more paths to try, log or
 * throw an error message.
 *
 * @param error_mode Whether to log, throw, or silently ignore when no path passed to `try_path`
 * succeeds.
 * @param label A human-readable description of the desired file. This will be incorporated in the
 * generated error message in the context `cannot open {} file`. It will also be used to generate
 * the file-specific environment variable.
 * @param basename The expected name of the file, which will be looked for in `NAVTK_DATA_DIR`
 * @param try_path A callback to examine each path and return true if it contains a usable file.
 */
void visit_possible_file_paths(ErrorMode error_mode,
                               const std::string& label,
                               const std::string& basename,
                               std::function<bool(const std::string&)> try_path);

}  // namespace detail


// The odd quoting syntax in this docstring ``%%APPDATA% `` is a workaround for a bug in Exhale
// that swallows the leading % in Windows environment variable names otherwise.
/**
 * Search well-known locations for a data file and open it if possible.
 *
 * This function will check for an environment variable named like `"NAVTK_" + label + "_PATH"`,
 * with an upper-cased, identifier-safe version of the label parameter. In other words, if label is
 * `"elevation map"`, the environment variable will be `NAVTK_ELEVATION_MAP_PATH`. If this
 * variable is set, it will be used as the only possible location of the file (causing an error
 * message if the file is not readable). Otherwise, the following locations will be searched in
 * order, looking for a file with the given `basename`:
 * - The value of `NAVTK_DATA_DIR`, if set.
 * - On debug builds, the `subprojects/navtk-data` subfolder of the clone that was passed to
 *   `meson`
 * - A `navtk` subfolder of your user data folder:
 *   - On Linux, this defaults to `~/.local/share/`, but can be overridden by `$XDG_DATA_HOME`
 *   - On Mac, this is `~/Library/Application Support/`
 *   - On Windows, it's the location pointed to by your ``%%APPDATA% `` environment variable.
 * - A `navtk` subfolder of your system data folder(s):
 *   - On Linux, `/usr/local/share` and `/usr/share` (or colon-separated `$XDG_DATA_DIRS`)
 *   - On Windows, ``%%PROGRAMDATA% ``
 * - The `-Ddatadir=` subfolder of the `-Dprefix=` passed to meson. Defaults to `$prefix/share`
 * - The current working directory
 *
 * @param error_mode Whether to throw an exception, emit a log, or silently return `nullptr` when
 * the file could not be found. This is an optional parameter, which defaults to the current global
 * error mode.
 * @param label A human-readable description of the desired file. This will be incorporated in the
 * generated error message in the context `cannot open {} file`. It will also be used to generate
 * the file-specific environment variable.
 * @param basename The expected name of the file to check for in each search path.
 * @param mode **C++:** Mode used to open the file, defaulting to `std::ios_base::in`.
 * **Python:** A string file mode that will be passed into `io.open(...)`, defaulting to `"r"`
 * @return **C++:** an `ifstream` open to the data file if found, otherwise `nullptr`.
 * **Python:** An open Python file object (from `io.open(...)`) if found, otherwise `None`.
 */
std::shared_ptr<std::ifstream> open_data_file(ErrorMode error_mode,
                                              const std::string& label,
                                              const std::string& basename,
                                              std::ios_base::openmode mode = std::ios_base::in);

#ifndef NEED_DOXYGEN_EXHALE_WORKAROUND

/**
 * Search well-known locations for a data file and open it if possible.
 *
 * This is a convenience overload of navtk::utils::open_data_file(ErrorMode, const std::string&,
 * const std::string&, std::ios_base::openmode) that uses the current global error mode.
 *
 * @param label A human-readable description of the desired file. This will be incorporated in the
 * generated error message in the context `cannot open {} file`. It will also be used to generate
 * the file-specific environment variable.
 * @param basename The expected name of the file to check for in each search path.
 * @param mode **C++:** Mode used to open the file, defaulting to `std::ios_base::in`.
 * **Python:** A string file mode that will be passed into `io.open(...)`, defaulting to `"r"`
 * @return **C++:** an `ifstream` open to the data file if found, otherwise `nullptr`.
 * **Python:** An open Python file object (from `io.open(...)`) if found, otherwise `None`.
 */
std::shared_ptr<std::ifstream> open_data_file(const std::string& label,
                                              const std::string& basename,
                                              std::ios_base::openmode mode = std::ios_base::in);
#endif

}  // namespace utils
}  // namespace navtk
