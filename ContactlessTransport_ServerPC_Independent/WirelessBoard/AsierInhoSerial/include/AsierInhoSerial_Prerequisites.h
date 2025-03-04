//#define _TIME_PROFILING
// Define ASIERINHO version
#define ASIERINHO_SERIAL_VERSION_MAJOR 2
#define ASIERINHO_SERIAL_VERSION_MINOR 0
#define ASIERINHO_SERIAL_VERSION_PATCH 0
#define ASIERINHO_SERIAL_VERSION_SUFFIX ""
#define ASIERINHO_SERIAL_VERSION_NAME "Zoe"

#define ASIERINHO_SERIAL_VERSION    ((ASIERINHO_VERSION_MAJOR << 16) | (ASIERINHO_VERSION_MINOR << 8) | ASIERINHO_VERSION_PATCH)

#if defined( ASIERINHO_SERIAL_NONCLIENT_BUILD )
#    define _AsierInhoSerial_Export __declspec( dllexport )
#else
#    if defined( __MINGW32__ )
#        define _AsierInhoSerial_Export 
#    else
#        define _AsierInhoSerial_Export  __declspec( dllimport )
#    endif
#endif

#ifndef ASIERINHO_SERIAL_VEC3
#define ASIERINHO_SERIAL_VEC3
namespace AsierInhoSerial {
	typedef struct _vec3 {
		union {
			struct { float x, y, z; };
			struct { float r, g, b; };
		};
		_vec3() : x(0), y(0), z(0) { ; }
		_vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) { ; }
		_vec3(const _vec3& pt) : x(pt.x), y(pt.y), z(pt.z) {}
		_vec3& operator=(const _vec3& v) {
			if (this != &v) {
				x = v.x; y = v.y; z = v.z;
			}
			return *this;
		}
	} vec3;
};
#define AsierInhoSerial_Handler long long
#endif