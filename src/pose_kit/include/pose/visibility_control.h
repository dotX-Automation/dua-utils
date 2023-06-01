#ifndef POSE__VISIBILITY_CONTROL_H_
#define POSE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define POSE_EXPORT __attribute__ ((dllexport))
    #define POSE_IMPORT __attribute__ ((dllimport))
  #else
    #define POSE_EXPORT __declspec(dllexport)
    #define POSE_IMPORT __declspec(dllimport)
  #endif
  #ifdef POSE_BUILDING_LIBRARY
    #define POSE_PUBLIC POSE_EXPORT
  #else
    #define POSE_PUBLIC POSE_IMPORT
  #endif
  #define POSE_PUBLIC_TYPE POSE_PUBLIC
  #define POSE_LOCAL
#else
  #define POSE_EXPORT __attribute__ ((visibility("default")))
  #define POSE_IMPORT
  #if __GNUC__ >= 4
    #define POSE_PUBLIC __attribute__ ((visibility("default")))
    #define POSE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define POSE_PUBLIC
    #define POSE_LOCAL
  #endif
  #define POSE_PUBLIC_TYPE
#endif

#endif  // POSE__VISIBILITY_CONTROL_H_
