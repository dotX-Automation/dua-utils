#ifndef DUA_QOS__VISIBILITY_CONTROL_H_
#define DUA_QOS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DUA_QOS_EXPORT __attribute__ ((dllexport))
    #define DUA_QOS_IMPORT __attribute__ ((dllimport))
  #else
    #define DUA_QOS_EXPORT __declspec(dllexport)
    #define DUA_QOS_IMPORT __declspec(dllimport)
  #endif
  #ifdef DUA_QOS_BUILDING_LIBRARY
    #define DUA_QOS_PUBLIC DUA_QOS_EXPORT
  #else
    #define DUA_QOS_PUBLIC DUA_QOS_IMPORT
  #endif
  #define DUA_QOS_PUBLIC_TYPE DUA_QOS_PUBLIC
  #define DUA_QOS_LOCAL
#else
  #define DUA_QOS_EXPORT __attribute__ ((visibility("default")))
  #define DUA_QOS_IMPORT
  #if __GNUC__ >= 4
    #define DUA_QOS_PUBLIC __attribute__ ((visibility("default")))
    #define DUA_QOS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DUA_QOS_PUBLIC
    #define DUA_QOS_LOCAL
  #endif
  #define DUA_QOS_PUBLIC_TYPE
#endif

#endif  // DUA_QOS__VISIBILITY_CONTROL_H_
