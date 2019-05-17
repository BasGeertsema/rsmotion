// Copyright (c) 2019 Bas Geertsema <mail@basgeertsema.com>

// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is furnished to do
// so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#ifndef RSMOTION_COORDINATESYSTEM_H_INCLUDED
#define RSMOTION_COORDINATESYSTEM_H_INCLUDED

#include <cstdint>
#include <limits>

#include "math.h"

namespace rsmotion
{

// the axes in the coordinate system
// to determine what is forward and right
// from the perspective of a reeds-shepp car.
//
//  ATTENTION:
//  PLEASE CHANGE THIS IF YOU USE
//  ANOTHER COORDINATE SYSTEM.
//
enum class Axis : uint8_t
{
    // axes on 3d vectors
    Fwd = 2,
    Right = 0,
    Up = 1,

    // axes on 2d vectors
    Fwd2 = 1,
    Right2 = 0,
    None = 4
};

// axes of the coordinate system
static const Axis FW{Axis::Fwd};
static const Axis RI{Axis::Right};
static const Axis UP{Axis::Up};

// returns the vector index for the specified axis
inline auto VecIndex(Axis ax)
{
    return static_cast<std::underlying_type<Axis>::type>(ax);
}

// returns the vector index for the specified axis
template <Axis ax>
inline auto VecIndex()
{
    return static_cast<std::underlying_type<Axis>::type>(ax);
}

template <class T>
inline const T &operator%(const math::Vec3<T> &v, const Axis &axis)
{
    return v[VecIndex(axis)];
}

template <class T>
inline T &operator%(math::Vec3<T> &v, const Axis &axis)
{
    return v[VecIndex(axis)];
}

template <Axis axis>
inline const math::Vec3f AxisUnitVector()
{
    math::Vec3f v{0, 0, 0};
    v % axis = 1.0f;
    return v;
}

// coordinate system used
class CoordinateSystem
{
public:
    CoordinateSystem();
    static const math::Vec3f Up() { return AxisUnitVector<Axis::Up>(); }
    static const math::Vec3f Right() { return AxisUnitVector<Axis::Right>(); }
    static const math::Vec3f Forward() { return AxisUnitVector<Axis::Fwd>(); }
};

// get the rotation in ground plane (fwd/right)
template <class T>
inline math::Angle<T> RotationInXY(const math::Quaternion<T> &q)
{
    if (q.At(VecIndex<Axis::Up>() + 1) >= static_cast<T>(0))
    {
        return math::Angle<T>::ArcCos(q.Real()) * 2;
    }
    return math::Angle<T>::ArcCos(q.Real()) * -2;
}

} // namespace rsmotion

#endif
