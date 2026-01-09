#pragma once

// This file allows for a filesystem to be robustly included across multiple environments.

#ifdef NAVTK_STD_HAS_FILESYSTEM_HEADER
#	include <filesystem>
#elif defined(NAVTK_STD_EXPERIMENTAL_FILESYSTEM_HEADER)
#	include <experimental/filesystem>
#elif defined(__has_include) && __has_include("filesystem")
#	include <filesystem>
#else
#	include <experimental/filesystem>
#endif

#ifdef NAVTK_STD_FILESYSTEM_NAMESPACE
// When NAVTK_STD_FILESYSTEM_NAMESPACE is defined, it should contain the namespace your compiler
// uses for the filesystem library. If you don't know, the #else below is a generic fallback that
// should work on most compilers.
namespace navtk {
namespace fs = NAVTK_STD_FILESYSTEM_NAMESPACE;
}
#else
// This works by creating a new `navtk::fs` namespace (rather than aliasing an existing namespace),
// and just populating that namespace with the contents of other namespaces that might exist
// depending on how filesystem is implemented in your libc++. Because the system libraries won't
// provide all of them, we declare the std namespaces we intend to try here ourselves before
// referencing them in the usings.
namespace std {
namespace filesystem {}
namespace experimental {
namespace filesystem {}
}  // namespace experimental
namespace __fs {
namespace filesystem {}
}  // namespace __fs
}  // namespace std

namespace navtk {
/**
 * This namespace is equivalent to the "filesystem" namespace provided by std, which, depending on
 * your compiler, may be prefixed by `experimental` or `__fs`.
 */
namespace fs {
using namespace ::std::experimental::filesystem;
using namespace ::std::__fs::filesystem;
using namespace ::std::filesystem;
}  // namespace fs
}  // namespace navtk
#endif
