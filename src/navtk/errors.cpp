#include <navtk/errors.hpp>

using navtk::ErrorMode;

namespace {

ErrorMode GLOBAL_ERROR_MODE =
#ifdef NO_MATRIX_VAL
    ErrorMode::OFF
#else
    ErrorMode::DIE
#endif
    ;

std::recursive_mutex GLOBAL_ERROR_MODE_MUTEX;

}  // namespace

namespace navtk {

std::ostream& operator<<(std::ostream& os, ErrorMode error_mode) {
	switch (error_mode) {
	case ErrorMode::OFF:
		return os << "ErrorMode::OFF";
	case ErrorMode::LOG:
		return os << "ErrorMode::LOG";
	case ErrorMode::DIE:
		return os << "ErrorMode::DIE";
	default:
		return os << "<INVALID ErrorMode(" << static_cast<int>(error_mode) << ")>";
	}
}

ErrorModeLock::ErrorModeLock(ErrorMode target_mode, bool enable_restore)
    : restore_enabled(enable_restore),
      restore_needed(enable_restore),
      lock(GLOBAL_ERROR_MODE_MUTEX),
      restore_mode(GLOBAL_ERROR_MODE),
      target_mode(target_mode) {
	GLOBAL_ERROR_MODE = target_mode;
}

ErrorModeLock::ErrorModeLock(ErrorModeLock&& src)
    : restore_enabled(src.restore_enabled),
      restore_needed(src.restore_needed),
      lock(std::move(src.lock)),
      restore_mode(src.restore_mode),
      target_mode(src.target_mode) {
	src.restore_needed = false;
}

ErrorModeLock::~ErrorModeLock() { unlock(); }

void ErrorModeLock::unlock() {
	if (restore_needed) GLOBAL_ERROR_MODE = restore_mode;
	restore_needed = false;
	if (lock) lock.unlock();
}

void ErrorModeLock::relock() {
	if (!lock) lock.lock();
	if (!restore_needed) {
		restore_needed = restore_enabled;
		restore_mode   = GLOBAL_ERROR_MODE;
	}
	GLOBAL_ERROR_MODE = target_mode;
}

ErrorMode get_global_error_mode() { return GLOBAL_ERROR_MODE; }

void set_global_error_mode(ErrorMode mode) { ErrorModeLock guard{mode, false}; }

}  // namespace navtk
