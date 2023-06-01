#ifndef DYNAMIC_POSE__VISIBILITY_CONTROL_H_
#define DYNAMIC_POSE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DYNAMIC_POSE_EXPORT __attribute__ ((dllexport))
    #define DYNAMIC_POSE_IMPORT __attribute__ ((dllimport))
  #else
    #define DYNAMIC_POSE_EXPORT __declspec(dllexport)
    #define DYNAMIC_POSE_IMPORT __declspec(dllimport)
  #endif
  #ifdef DYNAMIC_POSE_BUILDING_LIBRARY
    #define DYNAMIC_POSE_PUBLIC POSE_EXPORT
  #else
    #define DYNAMIC_POSE_PUBLIC POSE_IMPORT
  #endif
  #define DYNAMIC_POSE_PUBLIC_TYPE POSE_PUBLIC
  #define DYNAMIC_POSE_LOCAL
#else
  #define DYNAMIC_POSE_EXPORT __attribute__ ((visibility("default")))
  #define DYNAMIC_POSE_IMPORT
  #if __GNUC__ >= 4
    #define DYNAMIC_POSE_PUBLIC __attribute__ ((visibility("default")))
    #define DYNAMIC_POSE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DYNAMIC_POSE_PUBLIC
    #define DYNAMIC_POSE_LOCAL
  #endif
  #define DYNAMIC_POSE_PUBLIC_TYPE
#endif

#endif  // DYNAMIC_POSE__VISIBILITY_CONTROL_H_
