#ifndef DYNAMIC_SYSTEMS_CONTROL__VISIBILITY_CONTROL_H_
#define DYNAMIC_SYSTEMS_CONTROL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DYNAMIC_SYSTEMS_CONTROL_EXPORT __attribute__ ((dllexport))
    #define DYNAMIC_SYSTEMS_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define DYNAMIC_SYSTEMS_CONTROL_EXPORT __declspec(dllexport)
    #define DYNAMIC_SYSTEMS_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef DYNAMIC_SYSTEMS_CONTROL_BUILDING_LIBRARY
    #define DYNAMIC_SYSTEMS_CONTROL_PUBLIC DYNAMIC_SYSTEMS_CONTROL_EXPORT
  #else
    #define DYNAMIC_SYSTEMS_CONTROL_PUBLIC DYNAMIC_SYSTEMS_CONTROL_IMPORT
  #endif
  #define DYNAMIC_SYSTEMS_CONTROL_PUBLIC_TYPE DYNAMIC_SYSTEMS_CONTROL_PUBLIC
  #define DYNAMIC_SYSTEMS_CONTROL_LOCAL
#else
  #define DYNAMIC_SYSTEMS_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define DYNAMIC_SYSTEMS_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define DYNAMIC_SYSTEMS_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define DYNAMIC_SYSTEMS_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DYNAMIC_SYSTEMS_CONTROL_PUBLIC
    #define DYNAMIC_SYSTEMS_CONTROL_LOCAL
  #endif
  #define DYNAMIC_SYSTEMS_CONTROL_PUBLIC_TYPE
#endif

#endif  // DYNAMIC_SYSTEMS_CONTROL__VISIBILITY_CONTROL_H_
