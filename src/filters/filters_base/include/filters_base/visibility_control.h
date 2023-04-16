#ifndef FILTERS__VISIBILITY_CONTROL_H_
#define FILTERS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FILTERS_EXPORT __attribute__ ((dllexport))
    #define FILTERS_IMPORT __attribute__ ((dllimport))
  #else
    #define FILTERS_EXPORT __declspec(dllexport)
    #define FILTERS_IMPORT __declspec(dllimport)
  #endif
  #ifdef FILTERS_BUILDING_LIBRARY
    #define FILTERS_PUBLIC FILTERS_EXPORT
  #else
    #define FILTERS_PUBLIC FILTERS_IMPORT
  #endif
  #define FILTERS_PUBLIC_TYPE FILTERS_PUBLIC
  #define FILTERS_LOCAL
#else
  #define FILTERS_EXPORT __attribute__ ((visibility("default")))
  #define FILTERS_IMPORT
  #if __GNUC__ >= 4
    #define FILTERS_PUBLIC __attribute__ ((visibility("default")))
    #define FILTERS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FILTERS_PUBLIC
    #define FILTERS_LOCAL
  #endif
  #define FILTERS_PUBLIC_TYPE
#endif

#endif  // FILTERS__VISIBILITY_CONTROL_H_
