#pragma once

/** Convert argument to a string literal */
#define STRINGIFY(X) #X

#ifdef __clang__
#	define PRAGMA_DIAGNOSTIC(X) _Pragma(STRINGIFY(clang diagnostic X))
#elif defined(__GNUC__)
#	define PRAGMA_DIAGNOSTIC(X) _Pragma(STRINGIFY(GCC diagnostic X))
#else
/**
 * Expands to a compiler-specific `#pragma` on GCC and clang.
 *
 * On GCC it is equivalent to
 * ```
 * #pragma GCC diagnostic ARGS
 * ```
 *
 * On clang it is equivalent to
 * ```
 * #pragma clang diagnostic ARGS
 * ```
 *
 * On other compilers it does nothing.
 *
 * Example usage:
 * ```
 * PRAGMA_DIAGNOSTIC(ignored "-Wpedantic")
 * ```
 *
 * @param ARGS arguments for your compiler's diagnostic pragma, as raw tokens.
 */
#	define PRAGMA_DIAGNOSTIC(ARGS)
#endif

/**
 * Use PRAGMA_DIAGNOSTIC macro to disable a specific warning on GCC and clang.
 *
 * On GCC this is equivalent to
 * ```
 * #pragma GCC diagnostic push
 * #pragma GCC diagnostic ignored WARNING
 * ```
 *
 * On clang, it is equivalent to
 * ```
 * #pragma clang diagnostic push
 * #pragma clang diagnostic ignored WARNING
 * ```
 *
 * On other compilers it does nothing.
 *
 * @param WARNING The warning flag you'd like to suppress, as a string literal, e.g. `"-Wpedantic"`
 */
#define BEGIN_SUPPRESS_WARNING(WARNING) \
	PRAGMA_DIAGNOSTIC(push)             \
	PRAGMA_DIAGNOSTIC(ignored WARNING) struct __begin_suppress_warning_requires_trailing_semicolon

/**
 * Expands to an equivalent of `#pragma (your compiler) diagnostic pop`.
 */
#define END_SUPPRESS_WARNING \
	PRAGMA_DIAGNOSTIC(pop) struct __end_suppress_warning_requires_trailing_semicolon
