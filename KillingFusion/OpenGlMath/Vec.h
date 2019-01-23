#pragma once

#include <cmath>
#include <cstdlib>
#include <cstdint>
#include "Utils.h"

//------------------------------------------------------------------------------
// Vec2
//------------------------------------------------------------------------------


#define _DEFINE_VEC2_NO_FUNCTIONS(type, Vec2)
#define _DEFINE_VEC2_NO_MEMBERS(type, Vec2)


#define _DEFINE_VEC2_INT_MEMBERS(type, Vec2)                          \
	Vec2 &operator|=(const Vec2 &r) { x|=r.x; y|=r.y; return *this; } \
	Vec2 &operator&=(const Vec2 &r) { x&=r.x; y&=r.y; return *this; } \
	Vec2 &operator^=(const Vec2 &r) { x^=r.x; y^=r.y; return *this; } \
	Vec2 &operator%=(const Vec2 &r) { x%=r.x; y%=r.y; return *this; } \


#define _DEFINE_VEC2_INT_FUNCTIONS(type, Vec2)                                                                        \
static inline Vec2 floor_div(const Vec2 &a, const Vec2 &b) { return Vec2(floor_div(a.x, b.x), floor_div(a.y, b.y)); } \
static inline Vec2 operator&(const Vec2 &l, const Vec2 &r) { return Vec2(l.x & r.x, l.y & r.y); }                     \
static inline Vec2 operator|(const Vec2 &l, const Vec2 &r) { return Vec2(l.x | r.x, l.y | r.y); }                     \
static inline Vec2 operator^(const Vec2 &l, const Vec2 &r) { return Vec2(l.x ^ r.x, l.y ^ r.y); }                     \
static inline Vec2 operator%(const Vec2 &l, const Vec2 &r) { return Vec2(l.x % r.x, l.y % r.y); }                     \


#define _DEFINE_VEC2_FLOAT_FUNCTIONS(type, Vec2)                                      \
static inline type length(const Vec2 &v) { return std::sqrt(length2(v)); }            \
static inline Vec2 normalize(const Vec2 &v) { return v / Vec2(length(v)); }           \
static inline type distance(const Vec2 &v1, const Vec2 &v2) { return length(v2-v1); } \


#define _DEFINE_VEC2(type, Vec2, ADDITIONAL_MEMBERS, ADDITIONAL_FUNCTIONS)                            \
struct Vec2 {                                                                                         \
	union {                                                                                           \
		struct {                                                                                      \
			type x, y;                                                                                \
		};                                                                                            \
		type data[2];                                                                                 \
	};                                                                                                \
                                                                                                      \
	Vec2() = default;                                                                                 \
	Vec2(type ax, type ay): x(ax), y(ay) {}                                                           \
	explicit Vec2(type v): x(v), y(v) {}                                                              \
                                                                                                      \
	Vec2 &operator+=(const Vec2 &r) { x+=r.x; y+=r.y; return *this; }                                 \
	Vec2 &operator-=(const Vec2 &r) { x-=r.x; y-=r.y; return *this; }                                 \
	Vec2 &operator*=(const Vec2 &r) { x*=r.x; y*=r.y; return *this; }                                 \
	Vec2 &operator/=(const Vec2 &r) { x/=r.x; y/=r.y; return *this; }                                 \
	ADDITIONAL_MEMBERS(type, Vec2)                                                                    \
                                                                                                      \
	type &operator[](int i) { return data[i]; }                                                       \
	type operator[](int i) const { return data[i]; }                                                  \
};                                                                                                    \
                                                                                                      \
static inline Vec2 Vec2##_X(type v = 1) { return {v, 0}; }                                            \
static inline Vec2 Vec2##_Y(type v = 1) { return {0, v}; }                                            \
                                                                                                      \
static inline bool operator==(const Vec2 &l, const Vec2 &r) { return l.x == r.x && l.y == r.y; }      \
static inline bool operator!=(const Vec2 &l, const Vec2 &r) { return l.x != r.x || l.y != r.y; }      \
static inline bool operator<(const Vec2 &l, const Vec2 &r)  { return l.x < r.x && l.y < r.y; }        \
static inline bool operator>(const Vec2 &l, const Vec2 &r)  { return l.x > r.x && l.y > r.y; }        \
static inline bool operator<=(const Vec2 &l, const Vec2 &r) { return l.x <= r.x && l.y <= r.y; }      \
static inline bool operator>=(const Vec2 &l, const Vec2 &r) { return l.x >= r.x && l.y >= r.y; }      \
                                                                                                      \
static inline Vec2 operator-(const Vec2 &v) { return Vec2(-v.x, -v.y); }                              \
static inline Vec2 operator+(const Vec2 &l, const Vec2 &r) { return Vec2(l.x + r.x, l.y + r.y); }     \
static inline Vec2 operator-(const Vec2 &l, const Vec2 &r) { return Vec2(l.x - r.x, l.y - r.y); }     \
static inline Vec2 operator*(const Vec2 &l, const Vec2 &r) { return Vec2(l.x * r.x, l.y * r.y); }     \
static inline Vec2 operator/(const Vec2 &l, const Vec2 &r) { return Vec2(l.x / r.x, l.y / r.y); }     \
                                                                                                      \
static inline type area(const Vec2 &v) { return v.x * v.y; }                                          \
static inline type length2(const Vec2 &v) { return v.x*v.x + v.y*v.y; }                               \
static inline type dot(const Vec2 &v1, const Vec2 &v2) { return v1.x*v2.x + v1.y*v2.y; }              \
static inline type distance2(const Vec2 &v1, const Vec2 &v2) { return length2(v2-v1); }               \
static inline Vec2 min(const Vec2 &v1, const Vec2 &v2) { return {min(v1.x, v2.x), min(v1.y, v2.y)}; } \
static inline Vec2 max(const Vec2 &v1, const Vec2 &v2) { return {max(v1.x, v2.x), max(v1.y, v2.y)}; } \
ADDITIONAL_FUNCTIONS(type, Vec2)                                                                      \


_DEFINE_VEC2(float,    Vec2f,  _DEFINE_VEC2_NO_MEMBERS,  _DEFINE_VEC2_FLOAT_FUNCTIONS)
_DEFINE_VEC2(double,   Vec2d,  _DEFINE_VEC2_NO_MEMBERS,  _DEFINE_VEC2_FLOAT_FUNCTIONS)
_DEFINE_VEC2(int32_t,  Vec2i,  _DEFINE_VEC2_INT_MEMBERS, _DEFINE_VEC2_INT_FUNCTIONS)
_DEFINE_VEC2(int16_t,  Vec2s,  _DEFINE_VEC2_INT_MEMBERS, _DEFINE_VEC2_INT_FUNCTIONS)
_DEFINE_VEC2(int8_t,   Vec2b,  _DEFINE_VEC2_INT_MEMBERS, _DEFINE_VEC2_INT_FUNCTIONS)

_DEFINE_VEC2(uint16_t, Vec2us, _DEFINE_VEC2_INT_MEMBERS, _DEFINE_VEC2_INT_FUNCTIONS)

static inline Vec2f ToVec2f(const Vec2i &v) { return Vec2f(v.x, v.y); }
static inline Vec2i ToVec2i(const Vec2f &v) { return Vec2i(v.x, v.y); }

//------------------------------------------------------------------------------
// Vec3
//------------------------------------------------------------------------------

#define _DEFINE_VEC3_NO_FUNCTIONS(type, Vec3)
#define _DEFINE_VEC3_NO_MEMBERS(type, Vec3)


#define _DEFINE_VEC3_INT_MEMBERS(type, Vec3)                                  \
	Vec3 &operator&=(const Vec3 &r) { x&=r.x; y&=r.y; z&=r.z; return *this; } \
	Vec3 &operator|=(const Vec3 &r) { x|=r.x; y|=r.y; z|=r.z; return *this; } \
	Vec3 &operator^=(const Vec3 &r) { x^=r.x; y^=r.y; z^=r.z; return *this; } \
	Vec3 &operator%=(const Vec3 &r) { x%=r.x; y%=r.y; z%=r.z; return *this; } \


#define _DEFINE_VEC3_INT_FUNCTIONS(type, Vec3)                                                                                             \
static inline Vec3 floor_div(const Vec3 &a, const Vec3 &b) { return Vec3(floor_div(a.x, b.x), floor_div(a.y, b.y), floor_div(a.z, b.z)); } \
static inline Vec3 operator^(const Vec3 &l, const Vec3 &r) { return Vec3(l.x ^ r.x, l.y ^ r.y, l.z ^ r.z); }                               \
static inline Vec3 operator%(const Vec3 &l, const Vec3 &r) { return Vec3(l.x % r.x, l.y % r.y, l.z % r.z); }                               \
static inline Vec3 operator&(const Vec3 &l, const Vec3 &r) { return Vec3(l.x & r.x, l.y & r.y, l.z & r.z); }                               \
static inline Vec3 operator|(const Vec3 &l, const Vec3 &r) { return Vec3(l.x | r.x, l.y | r.y, l.z | r.z); }                               \
static inline Vec3 operator~(const Vec3 &v) { return Vec3(~v.x, ~v.y, ~v.z); }                                                             \


#define _DEFINE_VEC3_FLOAT_FUNCTIONS(type, Vec3)                                                                                          \
static inline Vec3 abs(const Vec3 &v) { return Vec3(std::abs(v.x), std::abs(v.y), std::abs(v.z)); }                                       \
static inline type length(const Vec3 &v) { return std::sqrt(length2(v)); }                                                                \
static inline Vec3 normalize(const Vec3 &v) { return v / Vec3(length(v)); }                                                               \
static inline bool is_nan(const Vec3 &v) { return std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z); }                                \
static inline type distance(const Vec3 &v1, const Vec3 &v2) { return length(v2-v1); }                                                     \
static inline Vec3 lerp(const Vec3 &a, const Vec3 &b, float v) { return a * Vec3(1 - v) + b * Vec3(v); }                                  \
static inline Vec3 mod(const Vec3 &a, const Vec3 &b) { return Vec3(std::fmod(a.x, b.x), std::fmod(a.y, b.y), std::fmod(a.z, b.z)); }      \
static inline Vec3 pow(const Vec3 &v1, const Vec3 &v2) { return Vec3(std::pow(v1.x, v2.x), std::pow(v1.y, v2.y), std::pow(v1.z, v2.z)); } \


#define _DEFINE_VEC3(type, Vec3, Vec2, ADDITIONAL_MEMBERS, ADDITIONAL_FUNCTIONS)                                                                           \
struct Vec3 {                                                                                                                                              \
	union {                                                                                                                                                \
		struct {                                                                                                                                           \
			type x, y, z;                                                                                                                                  \
		};                                                                                                                                                 \
		type data[3];                                                                                                                                      \
	};                                                                                                                                                     \
                                                                                                                                                           \
	Vec3() = default;                                                                                                                                      \
	Vec3(type ax, type ay, type az): x(ax), y(ay), z(az) {}                                                                                                \
	explicit Vec3(type v): x(v), y(v), z(v) {}                                                                                                             \
                                                                                                                                                           \
	Vec3 &operator+=(const Vec3 &r) { x+=r.x; y+=r.y; z+=r.z; return *this; }                                                                              \
	Vec3 &operator-=(const Vec3 &r) { x-=r.x; y-=r.y; z-=r.z; return *this; }                                                                              \
	Vec3 &operator*=(const Vec3 &r) { x*=r.x; y*=r.y; z*=r.z; return *this; }                                                                              \
	Vec3 &operator/=(const Vec3 &r) { x/=r.x; y/=r.y; z/=r.z; return *this; }                                                                              \
	ADDITIONAL_MEMBERS(type, Vec3)                                                                                                                         \
                                                                                                                                                           \
	type &operator[](int i) { return data[i]; }                                                                                                            \
	type operator[](int i) const { return data[i]; }                                                                                                       \
                                                                                                                                                           \
	Vec2 XY() const { return {x, y}; }                                                                                                                     \
	Vec2 XZ() const { return {x, z}; }                                                                                                                     \
	Vec2 YZ() const { return {y, z}; }                                                                                                                     \
};                                                                                                                                                         \
                                                                                                                                                           \
static inline Vec3 Vec3##_X(type v = 1) { return {v, 0, 0}; }                                                                                              \
static inline Vec3 Vec3##_Y(type v = 1) { return {0, v, 0}; }                                                                                              \
static inline Vec3 Vec3##_Z(type v = 1) { return {0, 0, v}; }                                                                                              \
static inline Vec3 Vec3##_XY(const Vec2 &v) { return {v.x, v.y, 0}; }                                                                                      \
static inline Vec3 Vec3##_XZ(const Vec2 &v) { return {v.x, 0, v.y}; }                                                                                      \
static inline Vec3 Vec3##_YZ(const Vec2 &v) { return {0, v.x, v.y}; }                                                                                      \
                                                                                                                                                           \
static inline bool operator==(const Vec3 &l, const Vec3 &r) { return l.x == r.x && l.y == r.y && l.z == r.z; }                                             \
static inline bool operator!=(const Vec3 &l, const Vec3 &r) { return l.x != r.x || l.y != r.y || l.z != r.z; }                                             \
static inline bool operator<(const Vec3 &l, const Vec3 &r)  { return l.x < r.x && l.y < r.y && l.z < r.z; }                                                \
static inline bool operator>(const Vec3 &l, const Vec3 &r)  { return l.x > r.x && l.y > r.y && l.z > r.z; }                                                \
static inline bool operator<=(const Vec3 &l, const Vec3 &r) { return l.x <= r.x && l.y <= r.y && l.z <= r.z; }                                             \
static inline bool operator>=(const Vec3 &l, const Vec3 &r) { return l.x >= r.x && l.y >= r.y && l.z >= r.z; }                                             \
static inline Vec3 operator+(const Vec3 &l, const Vec3 &r) { return Vec3(l.x + r.x, l.y + r.y, l.z + r.z); }                                               \
static inline Vec3 operator-(const Vec3 &l, const Vec3 &r) { return Vec3(l.x - r.x, l.y - r.y, l.z - r.z); }                                               \
static inline Vec3 operator*(const Vec3 &l, const Vec3 &r) { return Vec3(l.x * r.x, l.y * r.y, l.z * r.z); }                                               \
static inline Vec3 operator/(const Vec3 &l, const Vec3 &r) { return Vec3(l.x / r.x, l.y / r.y, l.z / r.z); }                                               \
static inline Vec3 operator-(const Vec3 &v) { return Vec3(-v.x, -v.y, -v.z); }                                                                             \
                                                                                                                                                           \
static inline type length2(const Vec3 &v) { return v.x*v.x + v.y*v.y + v.z*v.z; }                                                                          \
static inline type dot(const Vec3 &v1, const Vec3 &v2) { return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z; }                                                       \
static inline type volume(const Vec3 &v) { return v.x * v.y * v.z; }                                                                                       \
static inline Vec3 cross(const Vec3 &v1, const Vec3 &v2) { return Vec3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x); } \
static inline type distance2(const Vec3 &v1, const Vec3 &v2) { return length2(v2-v1); }                                                                    \
static inline Vec3 min(const Vec3 &v1, const Vec3 &v2) { return {min(v1.x, v2.x), min(v1.y, v2.y), min(v1.z, v2.z)}; }                                     \
static inline Vec3 max(const Vec3 &v1, const Vec3 &v2) { return {max(v1.x, v2.x), max(v1.y, v2.y), max(v1.z, v2.z)}; }                                     \
ADDITIONAL_FUNCTIONS(type, Vec3)                                                                                                                           \


_DEFINE_VEC3(float,   Vec3f, Vec2f, _DEFINE_VEC3_NO_MEMBERS,  _DEFINE_VEC3_FLOAT_FUNCTIONS)
_DEFINE_VEC3(double,  Vec3d, Vec2d, _DEFINE_VEC3_NO_MEMBERS,  _DEFINE_VEC3_FLOAT_FUNCTIONS)
_DEFINE_VEC3(int32_t, Vec3i, Vec2i, _DEFINE_VEC3_INT_MEMBERS, _DEFINE_VEC3_INT_FUNCTIONS)
_DEFINE_VEC3(int16_t, Vec3s, Vec2s, _DEFINE_VEC3_INT_MEMBERS, _DEFINE_VEC3_INT_FUNCTIONS)
_DEFINE_VEC3(int8_t,  Vec3b, Vec2b, _DEFINE_VEC3_INT_MEMBERS, _DEFINE_VEC3_INT_FUNCTIONS)

_DEFINE_VEC3(uint16_t, Vec3us, Vec2us, _DEFINE_VEC3_INT_MEMBERS, _DEFINE_VEC3_INT_FUNCTIONS)


static inline Vec3f ToVec3f(const Vec3i &v) { return Vec3f(v.x, v.y, v.z); }
static inline Vec3f ToVec3f(const Vec3d &v) { return Vec3f(v.x, v.y, v.z); }
static inline Vec3d ToVec3d(const Vec3i &v) { return Vec3d(v.x, v.y, v.z); }
static inline Vec3d ToVec3d(const Vec3f &v) { return Vec3d(v.x, v.y, v.z); }
static inline Vec3i ToVec3i(const Vec3f &v) { return Vec3i(v.x, v.y, v.z); }
static inline Vec3i ToVec3i(const Vec3d &v) { return Vec3i(v.x, v.y, v.z); }

static inline Vec3i floor(const Vec3f &v) { return Vec3i(std::floor(v.x), std::floor(v.y), std::floor(v.z)); }
static inline Vec3i floor(const Vec3d &v) { return Vec3i(std::floor(v.x), std::floor(v.y), std::floor(v.z)); }

static inline bool axes_equal(const Vec3i &a, const Vec3i &b, const Vec2i &axes)
{
	return a[axes[0]] == b[axes[0]] && a[axes[1]] == b[axes[1]];
}

static inline bool aabb_aabb_intersection(const Vec3i &amin, const Vec3i &amax,
	const Vec3i &bmin, const Vec3i &bmax)
{
	return !(
		amax.x < bmin.x ||
		amax.y < bmin.y ||
		amax.z < bmin.z ||
		amin.x > bmax.x ||
		amin.y > bmax.y ||
		amin.z > bmax.z
	);
}

//------------------------------------------------------------------------------
// Vec4
//------------------------------------------------------------------------------

struct Vec4f {
	union {
		struct {
			float x, y, z, w;
		};
		float data[4];
	};

	Vec4f() = default;
	Vec4f(float x, float y, float z, float w): x(x), y(y), z(z), w(w) {}
	explicit Vec4f(float v): x(v), y(v), z(v), w(v) {}

	Vec4f &operator+=(const Vec4f &r) { x+=r.x; y+=r.y; z+=r.z; w+=r.w; return *this; }
	Vec4f &operator-=(const Vec4f &r) { x-=r.x; y-=r.y; z-=r.z; w-=r.w; return *this; }
	Vec4f &operator*=(const Vec4f &r) { x*=r.x; y*=r.y; z*=r.z; w*=r.w; return *this; }
	Vec4f &operator/=(const Vec4f &r) { x/=r.x; y/=r.y; z/=r.z; w/=r.w; return *this; }

	float &operator[](int i) { return data[i]; }
	float operator[](int i) const { return data[i]; }
};

static inline Vec4f operator+(const Vec4f &l, const Vec4f &r) { return {l.x + r.x, l.y + r.y, l.z + r.z, l.w + r.w}; }
static inline Vec4f operator-(const Vec4f &l, const Vec4f &r) { return {l.x - r.x, l.y - r.y, l.z - r.z, l.w - r.w}; }
static inline Vec4f operator*(const Vec4f &l, const Vec4f &r) { return {l.x * r.x, l.y * r.y, l.z * r.z, l.w * r.w}; }
static inline Vec4f operator/(const Vec4f &l, const Vec4f &r) { return {l.x / r.x, l.y / r.y, l.z / r.z, l.w / r.w}; }

static inline float dot(const Vec4f &v1, const Vec4f &v2) { return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z + v1.w*v2.w; }

static inline Vec3f ToVec3(const Vec4f &v) { return Vec3f(v.x, v.y, v.z); }
static inline Vec4f ToVec4(const Vec3f &v) { return Vec4f(v.x, v.y, v.z, 1); }

//------------------------------------------------------------------------------
// Vec4i
//------------------------------------------------------------------------------

struct Vec4i {
	union {
		struct {
			int x, y, z, w;
		};
		int data[4];
	};

	Vec4i() = default;
	Vec4i(int x, int y, int z, int w): x(x), y(y), z(z), w(w) {}
	explicit Vec4i(int v): x(v), y(v), z(v), w(v) {}

	Vec4i &operator+=(const Vec4i &r) { x+=r.x; y+=r.y; z+=r.z; w+=r.w; return *this; }
	Vec4i &operator-=(const Vec4i &r) { x-=r.x; y-=r.y; z-=r.z; w-=r.w; return *this; }
	Vec4i &operator*=(const Vec4i &r) { x*=r.x; y*=r.y; z*=r.z; w*=r.w; return *this; }
	Vec4i &operator/=(const Vec4i &r) { x/=r.x; y/=r.y; z/=r.z; w/=r.w; return *this; }

	int &operator[](int i) { return data[i]; }
	int operator[](int i) const { return data[i]; }
};

static inline Vec4i operator+(const Vec4i &l, const Vec4i &r) { return {l.x + r.x, l.y + r.y, l.z + r.z, l.w + r.w}; }
static inline Vec4i operator-(const Vec4i &l, const Vec4i &r) { return {l.x - r.x, l.y - r.y, l.z - r.z, l.w - r.w}; }
static inline Vec4i operator*(const Vec4i &l, const Vec4i &r) { return {l.x * r.x, l.y * r.y, l.z * r.z, l.w * r.w}; }
static inline Vec4i operator/(const Vec4i &l, const Vec4i &r) { return {l.x / r.x, l.y / r.y, l.z / r.z, l.w / r.w}; }

static inline Vec4f ToVec4(const Vec4i &v) { return Vec4f(v.x, v.y, v.z, v.w); }
static inline Vec4i ToVec4i(const Vec4f &v) { return Vec4i(v.x, v.y, v.z, v.w); }

//------------------------------------------------------------------------------
// Macro Utils
//------------------------------------------------------------------------------

#define VEC2(v) (v).x, (v).y
#define VEC3(v) (v).x, (v).y, (v).z
#define VEC4(v) (v).x, (v).y, (v).z, (v).w
