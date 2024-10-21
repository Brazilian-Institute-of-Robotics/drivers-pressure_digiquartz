#ifndef PRESSURE_PKG__VISIBILITY_CONTROL_H_
#define PRESSURE_PKG__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PRESSURE_PKG_EXPORT __attribute__ ((dllexport))
    #define PRESSURE_PKG_IMPORT __attribute__ ((dllimport))
  #else
    #define PRESSURE_PKG_EXPORT __declspec(dllexport)
    #define PRESSURE_PKG_IMPORT __declspec(dllimport)
  #endif
  #ifdef PRESSURE_PKG_BUILDING_LIBRARY
    #define PRESSURE_PKG_PUBLIC PRESSURE_PKG_EXPORT
  #else
    #define PRESSURE_PKG_PUBLIC PRESSURE_PKG_IMPORT
  #endif
  #define PRESSURE_PKG_PUBLIC_TYPE PRESSURE_PKG_PUBLIC
  #define PRESSURE_PKG_LOCAL
#else
  #define PRESSURE_PKG_EXPORT __attribute__ ((visibility("default")))
  #define PRESSURE_PKG_IMPORT
  #if __GNUC__ >= 4
    #define PRESSURE_PKG_PUBLIC __attribute__ ((visibility("default")))
    #define PRESSURE_PKG_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PRESSURE_PKG_PUBLIC
    #define PRESSURE_PKG_LOCAL
  #endif
  #define PRESSURE_PKG_PUBLIC_TYPE
#endif

#endif  // PRESSURE_PKG__VISIBILITY_CONTROL_H_