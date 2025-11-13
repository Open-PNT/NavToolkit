#pragma once

#include <mutex>
#include <ostream>
#include <stdexcept>

#include <spdlog/spdlog.h>

// These includes must appear after #include <spdlog/spdlog.h> to prevent linker errors when
// formatting complex types such as std::vector.
#include <spdlog/fmt/bundled/ostream.h>
#include <spdlog/fmt/bundled/ranges.h>

namespace navtk {

/**
 * Defines whether error checking of inputs should be performed, and if so, how severe a response is
 * desired when an error occurs.
 *
 * For research & development purposes, as well as scientific code, ErrorMode::DIE is strongly
 * recommended to catch problems early rather than having to work backwards from nonsensical
 * outputs. ErrorMode::LOG can be useful for finding bugs in operational/embedded code.
 * ErrorMode::OFF should be used in production embedded systems to avoid error-checking overhead.
 *
 * The default global error mode is ErrorMode::OFF if navtk is compiled with `-DNO_MATRIX_VAL`.
 * Otherwise, the default is ErrorMode::DIE. This can be changed at runtime using
 * navtk::set_global_error_mode or by holding a navtk::ErrorModeLock. Many navtk functions will
 * read the global error mode to decide whether to validate their inputs.
 *
 * If you're extending navtk and wish to incorporate optional error checking, consider taking an
 * ErrorMode as an optional parameter that defaults to the value of navtk::get_global_error_mode, to
 * allow your users the option of avoiding the global error mode.
 *
 * navtk::utils::ValidationContext may be useful for checking bounds and conditions of matrices. If
 * you perform custom validation, make sure any expensive calculations involved only happen if the
 * error mode is not OFF, consider using navtk::log_or_throw to report your error messages.
 */
enum class ErrorMode {
	OFF,  //!< Skip as much error-checking as possible for maximum performance.
	LOG,  //!< Perform error checks and log warnings, but do not throw exceptions.
	DIE   //!< Throw exceptions when data are malformed.
};

/**
 * Print the name-qualified human-readable name of the ErrorMode, for example `"ErrorMode::OFF"`
 *
 * @param os Stream output
 * @param error_mode Value to print to the stream
 * @return The output stream \p os after writing.
 */
std::ostream& operator<<(std::ostream& os, ErrorMode error_mode);

/**
 * Exception type used by navtk::log_or_throw when no specific exception type is
 * specified by the caller.
 */
typedef std::runtime_error DefaultLogOrThrowException;

/**
 * Log level used by navtk::log_or_throw when no specific log level is specified by the caller.
 */
constexpr spdlog::level::level_enum DEFAULT_LOG_OR_THROW_LEVEL = spdlog::level::level_enum::err;

/**
 * Gets the current navtk::ErrorMode, which may have been set by a navtk::ErrorModeLock or
 * navtk::set_global_error_mode. Use this value to decide whether to inspect your function's inputs
 * for potential problems, or to decide whether to write errors to a log or throw an exception.
 * @return The current navtk::ErrorMode.
 */
ErrorMode get_global_error_mode();

/**
 * A recursive lock on the current global navtk::ErrorMode (as reported by
 * navtk::get_global_error_mode) that makes sure the value can only be changed by the current
 * thread. The lock is honored both by other navtk::ErrorModeLock instances and
 * navtk::set_global_error_mode.
 *
 * When an instance of ErrorModeLock is created, the current global ErrorMode is stored. If the
 * `enable_restore` parameter is `true` (the default), this saved state will be restored
 * (regardless of any same-thread calls to navtk::set_global_error_mode that may have run during its
 * lifetime).
 *
 * ErrorModeLock instances are backed by `std::unique_lock`, and as such can be moved but not
 * copied.
 *
 * Python users should use this object as a context manager (using the `with` statement) to ensure
 * the lock is correctly released at the appropriate time.
 *
 * ```
 * with navtk.ErrorModeLock(navtk.ErrorMode.OFF):
 *     # your code here
 * ```
 *
 * The ErrorModeLock python bindings are not threadsafe themselves. The `__enter__` and `__exit__`
 * methods (used by the `with` statement) should only be called by the thread that created the
 * ErrorModeLock.
 */
class ErrorModeLock {
public:
	/**
	 * Sets the value of the global ErrorMode (see navtk::get_global_error_mode) for the lifetime of
	 * this object.
	 *
	 * @param target_mode Mode to set.
	 * @param enable_restore Whether to restore the previous global error mode when this object
	 * goes out of scope.
	 */
	ErrorModeLock(ErrorMode target_mode, bool enable_restore = true);

	/**
	 * Moves ownership of the lock to a new instance, preventing the original instance from changing
	 * the global state when it is destroyed.
	 *
	 * @param src Source object being moved-from
	 */
	ErrorModeLock(ErrorModeLock&& src);

	ErrorModeLock(const ErrorModeLock&)            = delete;  //!< Deleted; use `std::move` instead.
	ErrorModeLock& operator=(ErrorModeLock&&)      = delete;  //!< Deleted; causes nonsense behavior
	ErrorModeLock& operator=(const ErrorModeLock&) = delete;  //!< Deleted; causes nonsense behavior

	virtual ~ErrorModeLock();

protected:
	/**
	 * Release the underlying lock and restore the previous navtk::ErrorMode. C++ users should allow
	 * the destructor to be called instead; this function exists to allow Python users to use
	 * ErrorModeLock as a context manager.
	 */
	void unlock();

	/**
	 * Re-acquire the underlying lock and set the global error mode to the `target_mode` passed into
	 * the constructor. C++ users should avoid this function; it exists to allow Python users to use
	 * ErrorModeLock as a context manager.
	 */
	void relock();

private:
	bool restore_enabled;
	bool restore_needed;
	std::unique_lock<std::recursive_mutex> lock;
	ErrorMode restore_mode;
	ErrorMode target_mode;
};


// TODO(PNTOS-387): optional timeout parameter to prevent deadlocks
/**
 * Sets the global ErrorMode (see navtk::get_global_error_mode). If another function holds an
 * ErrorModeLock, this function blocks until that lock exits. If the current thread holds an
 * ErrorModeLock, this function works as normal.
 *
 * See navtk::ErrorMode for a discussion of error modes.
 *
 * In a multi-threaded environment, consider using navtk::ErrorModeLock instead, as it will prevent
 * other threads from changing the global error state.
 *
 * @param mode Target error mode
 */
void set_global_error_mode(ErrorMode mode);

/**
 * Write a log message or throw an error, depending on the value of the given navtk::ErrorMode.
 *
 * @tparam Exc The exception type. When the mode is ErrorMode::DIE, an instance of this exception
 * will be thrown, initialized with a string error message as its constructor parameter.
 * @tparam Level The log level to use when logging the error message.
 * @tparam FormatArgs Type parameters for the message argument. The first of these should be
 * implicitly convertible to `std::string`. Others can be any type.
 * @param mode Which behavior(s) to enable. ErrorMode::LOG emits a log message only, ErrorMode::DIE
 * logs and throws an exception. This parameter can be omitted, in which case the current value of
 * get_global_error_mode() will be used.
 * @param message A `fmt`-style format string (`"foo: {}"`) and positional arguments. See
 * https://github.com/fmtlib/fmt for details.
 * @throw An instance of `Exc` if mode is ErrorMode::DIE.
 */
template <typename Exc                    = DefaultLogOrThrowException,
          spdlog::level::level_enum Level = DEFAULT_LOG_OR_THROW_LEVEL,
          typename... FormatArgs>
void log_or_throw(ErrorMode mode, FormatArgs&&... message) {
	if (mode != ErrorMode::OFF) spdlog::log(Level, message...);
	if (mode == ErrorMode::DIE) throw Exc(fmt::format(std::forward<FormatArgs>(message)...));
}

#ifndef NEED_DOXYGEN_EXHALE_WORKAROUND

template <typename Exc                    = DefaultLogOrThrowException,
          spdlog::level::level_enum Level = DEFAULT_LOG_OR_THROW_LEVEL,
          typename... FormatArgs>
void log_or_throw(FormatArgs&&... message) {
	log_or_throw<Exc, Level>(get_global_error_mode(), std::forward<FormatArgs>(message)...);
}

template <spdlog::level::level_enum Level, typename... LogOrThrowArgs>
void log_or_throw(LogOrThrowArgs&&... args) {
	log_or_throw<DefaultLogOrThrowException, Level>(std::forward<LogOrThrowArgs>(args)...);
}

// SEE ALSO: py_log_or_throw_ in bindings/python/navtk.cpp, a re-implementation that uses runtime
// values instead of template parameters.

#endif  // NEED_DOXYGEN_EXHALE_WORKAROUND

}  // namespace navtk
