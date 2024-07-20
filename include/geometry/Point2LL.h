// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_INT_POINT_H
#define UTILS_INT_POINT_H

#include <cmath>
#include <limits>
#include <numbers>
#include <polyclipping/clipper.hpp>

#include "geometry/Point3LL.h"
#include "utils/types/generic.h"

#ifdef __GNUC__
#define DEPRECATED(func) func __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(func) func
#endif

namespace cura
{

using Point2LL = ClipperLib::IntPoint;

#define POINT_MIN std::numeric_limits<ClipperLib::cInt>::min()
#define POINT_MAX std::numeric_limits<ClipperLib::cInt>::max()

static const Point2LL no_point(POINT_MIN, POINT_MIN);

/* Extra operators to make it easier to do math with the 64bit Point objects */
static inline Point2LL operator-(const Point2LL& p0)
{
    return { -p0.X, -p0.Y };
}

static inline Point2LL operator+(const Point2LL& p0, const Point2LL& p1)
{
    return { p0.X + p1.X, p0.Y + p1.Y };
}

static inline Point2LL operator-(const Point2LL& p0, const Point2LL& p1)
{
    return { p0.X - p1.X, p0.Y - p1.Y };
}

static inline Point2LL operator*(const Point2LL& p0, const coord_t i)
{
    return { p0.X * i, p0.Y * i };
}

template<utils::numeric T>
static inline Point2LL operator*(const Point2LL& p0, const T i)
{
    return { std::llrint(static_cast<T>(p0.X) * i), std::llrint(static_cast<T>(p0.Y) * i) };
}

template<utils::numeric T>
static inline Point2LL operator*(const T i, const Point2LL& p0)
{
    return p0 * i;
}

static inline Point2LL operator/(const Point2LL& p0, const coord_t i)
{
    return { p0.X / i, p0.Y / i };
}

static inline Point2LL operator/(const Point2LL& p0, const Point2LL& p1)
{
    return { p0.X / p1.X, p0.Y / p1.Y };
}

static inline Point2LL operator%(const Point2LL& p0, const coord_t i)
{
    return { p0.X % i, p0.Y % i };
}

static inline Point2LL& operator+=(Point2LL& p0, const Point2LL& p1)
{
    p0.X += p1.X;
    p0.Y += p1.Y;
    return p0;
}

static inline Point2LL& operator-=(Point2LL& p0, const Point2LL& p1)
{
    p0.X -= p1.X;
    p0.Y -= p1.Y;
    return p0;
}

static inline bool operator<(const Point2LL& p0, const Point2LL& p1)
{
    return (p0.X < p1.X) || (p0.X == p1.X && p0.Y < p1.Y);
}

static inline coord_t vSize2(const Point2LL& p0)
{
    return p0.X * p0.X + p0.Y * p0.Y;
}

static inline double vSize2f(const Point2LL& p0)
{
    return static_cast<double>(p0.X) * static_cast<double>(p0.X) + static_cast<double>(p0.Y) * static_cast<double>(p0.Y);
}

static inline bool shorterThan(const Point2LL& p0, const coord_t len)
{
    if (p0.X > len || p0.X < -len)
    {
        return false;
    }
    if (p0.Y > len || p0.Y < -len)
    {
        return false;
    }
    return vSize2(p0) <= len * len;
}

static inline coord_t vSize(const Point2LL& p0)
{
    return std::llrint(std::hypot(p0.X, p0.Y));
}

static inline double vSizeMM(const Point2LL& p0)
{
    double fx = INT2MM(p0.X);
    double fy = INT2MM(p0.Y);
    return std::hypot(fx, fy);
}

static inline Point2LL normal(const Point2LL& p0, coord_t length)
{
    const coord_t len = vSize(p0);
    if (len < 1)
    {
        return { length, 0 };
    }
    return p0 * length / len;
}

static inline Point2LL turn90CCW(const Point2LL& p0)
{
    return { -p0.Y, p0.X };
}

static inline Point2LL rotate(const Point2LL& p0, double angle)
{
    const double cos_angle = std::cos(angle);
    const double sin_angle = std::sin(angle);
    const double x = static_cast<double>(p0.X);
    const double y = static_cast<double>(p0.Y);
    return { std::llrint(cos_angle * x - sin_angle * y), std::llrint(sin_angle * x + cos_angle * y) };
}

static inline coord_t dot(const Point2LL& p0, const Point2LL& p1)
{
    return p0.X * p1.X + p0.Y * p1.Y;
}

static inline coord_t cross(const Point2LL& p0, const Point2LL& p1)
{
    return p0.X * p1.Y - p0.Y * p1.X;
}

static inline double angle(const Point2LL& p)
{
    double angle = std::atan2(p.Y, p.X) * 180.0 / std::numbers::pi;
    if (angle < 0.0)
    {
        angle += 360.0;
    }
    return angle;
}

// Identity function, used to make templated algorithms where the input is sometimes points, sometimes things that contain or can be converted to points.
static inline const Point2LL& make_point(const Point2LL& p)
{
    return p;
}

inline Point3LL operator+(const Point3LL& p3, const Point2LL& p2)
{
    return { p3.x_ + p2.X, p3.y_ + p2.Y, p3.z_ };
}

inline Point3LL& operator+=(Point3LL& p3, const Point2LL& p2)
{
    p3.x_ += p2.X;
    p3.y_ += p2.Y;
    return p3;
}

inline Point2LL operator+(const Point2LL& p2, const Point3LL& p3)
{
    return { p3.x_ + p2.X, p3.y_ + p2.Y };
}

inline Point3LL operator-(const Point3LL& p3, const Point2LL& p2)
{
    return { p3.x_ - p2.X, p3.y_ - p2.Y, p3.z_ };
}

inline Point3LL& operator-=(Point3LL& p3, const Point2LL& p2)
{
    p3.x_ -= p2.X;
    p3.y_ -= p2.Y;
    return p3;
}

inline Point2LL operator-(const Point2LL& p2, const Point3LL& p3)
{
    return { p2.X - p3.x_, p2.Y - p3.y_ };
}

} // namespace cura

namespace std
{
template<>
struct hash<cura::Point2LL>
{
    size_t operator()(const cura::Point2LL& pp) const noexcept
    {
        const int prime = 31;
        const int base = 89;
        int result = base;
        result = static_cast<int>(result * prime + pp.X);
        result = static_cast<int>(result * prime + pp.Y);
        return static_cast<size_t>(result);
    }
};
} // namespace std

#endif // UTILS_INT_POINT_H
