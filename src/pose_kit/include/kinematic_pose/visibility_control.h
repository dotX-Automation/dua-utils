#ifndef KINEMATIC_POSE__VISIBILITY_CONTROL_H_
#define KINEMATIC_POSE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KINEMATIC_POSE_EXPORT __attribute__ ((dllexport))
    #define KINEMATIC_POSE_IMPORT __attribute__ ((dllimport))
  #else
    #define KINEMATIC_POSE_EXPORT __declspec(dllexport)
    #define KINEMATIC_POSE_IMPORT __declspec(dllimport)
  #endif
  #ifdef KINEMATIC_POSE_BUILDING_LIBRARY
    #define KINEMATIC_POSE_PUBLIC POSE_EXPORT
  #else
    #define KINEMATIC_POSE_PUBLIC POSE_IMPORT
  #endif
  #define KINEMATIC_POSE_PUBLIC_TYPE POSE_PUBLIC
  #define KINEMATIC_POSE_LOCAL
#else
  #define KINEMATIC_POSE_EXPORT __attribute__ ((visibility("default")))
  #define KINEMATIC_POSE_IMPORT
  #if __GNUC__ >= 4
    #define KINEMATIC_POSE_PUBLIC __attribute__ ((visibility("default")))
    #define KINEMATIC_POSE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KINEMATIC_POSE_PUBLIC
    #define KINEMATIC_POSE_LOCAL
  #endif
  #define KINEMATIC_POSE_PUBLIC_TYPE
#endif

#endif  // KINEMATIC_POSE__VISIBILITY_CONTROL_H_
