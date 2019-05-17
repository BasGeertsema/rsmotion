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


//
// Algorithm based on the paper by Reeds Shepp:
//
//  Reeds, J. A.; Shepp, L. A. Optimal paths for a car that goes both forwards and backwards.
//  Pacific J. Math. 145 (1990), no. 2, 367--393. https://projecteuclid.org/euclid.pjm/1102645450
//

#pragma once

#ifndef RSMOTION_RSMOTION_H
#define RSMOTION_RSMOTION_H

#include <limits>
#include <iterator>
#include <algorithm>
#include <numeric>
#include <array>
#include <string>
#include <vector>

#include "coordinatesystem.h"

namespace rsmotion
{

namespace algorithm
{

/**
 * The type of a path segment
 */
enum class SegmentType : uint8_t
{
    Straight = 0,
    Left = 1,
    Right = 2,
    None = 3
};

/**
 * The traversal direction of this segment. Fwd is forward. Bwd is in reverse.
 */
enum class SegmentDirection : uint8_t
{
    Fwd = 0,
    Bwd = 1
};

/**
 * A constraint used by the internal Reeds-Shepp algorithm.
 */
enum class SegmentLengthConstraint : uint8_t
{
    Unconstrained = 0,
    HalfPI = 1,
    Equal = 2
};

/**
 * A single segment of a path. A path can contain multiple segments.
 * Each segment is defined by a type, direction and distance.
 */
struct Segment
{
    explicit Segment() = default;
    Segment(
        SegmentType type,
        SegmentDirection dir = SegmentDirection::Fwd,
        SegmentLengthConstraint constraint = SegmentLengthConstraint::Unconstrained,
        float distance = std::numeric_limits<float>::max())
        : Type{type}, Direction{dir}, Constraint{constraint}, Distance{distance}
    {
    }

    SegmentType Type;
    SegmentDirection Direction;
    SegmentLengthConstraint Constraint;

    float Distance{std::numeric_limits<float>::max()};
    bool Infinite() const
    {
        return Distance == std::numeric_limits<float>::max();
    }
    float Length() const
    {
        return std::fabs(Distance);
    }
};

/**
 * Represent the optimal path to go from PointState A to PointState B.
 * A path is defined by 1 up to 5 path segments.
 */
class Path
{
public:
    Path() = default;
    Path(std::initializer_list<Segment> segments) noexcept;

    float Length() const;
    bool Valid() const;
    float &Distance(std::size_t n);

    explicit operator bool() const;
    bool operator<(const Path &rhs) const;

    std::vector<Segment> Segments;
};

struct State
{
    State() = default;
    State(float x, float y, float a, bool inReverse = false) noexcept
        : _val{x, y, a}, _inReverse{inReverse}
    {
    }
    const float &X() const
    {
        return _val[0];
    }
    const float &Y() const
    {
        return _val[1];
    }

    /// yaw (rotation around z-axis)
    const float &Phi() const
    {
        return _val[2];
    }

    bool InReverse() const
    {
        return _inReverse;
    }

private:
    float _val[3]{0, 0, 0};

    bool _inReverse{false};
};

/**
 * Calculates the optimal path towards the supplied PointState.
 * The start state is implicitly the  origin (0,0,0) and no rotation.
 */
Path SearchShortestPath(State toState);

/**
 * Returns the interpolated PointState (location and orientation) along the 
 * path by supplying a normalized value between and including 0.f and 1.f
 */
State InterpolateNormalized(const State &from, const Path &path, float t);

/**
 * Returns the interpolated PointState (location and orientation) along the 
 * path by supplying the traveled distance.
 */
State InterpolateDistance(const State &from, const Path &path, const float dist);

} // namespace algorithm

/**
 * Cntains the position and orientation of a point (car wheels / axis)
 */
struct PointState
{
    PointState() = default;
    PointState(math::Vec3f pos)
        : Pos{pos}
    {
    }

    PointState(math::Vec3f pos, math::Quatf orientation)
        : Pos{pos}, Orientation{orientation}
    {
    }

    math::Vec3f Pos{};
    math::Quatf Orientation{};
};

/**
 * The state (position, orientation and direction) of a car.
 * 
 * The position and orientations are defined for both axes.
 * The rear axis is the leading axis and used to determine the
 * path. The rear axis/wheels therefore will always be on the path.
 * The front wheels however are aligned based on the rear axis
 * and orientation. Therefore, the front wheel will not 
 * be following the path in turns.
 */
struct CarState
{
    CarState() = default;

    CarState(PointState rear, float wheelBase, bool inReverse = false)
        : Rear{rear}, Front{}, InReverse{inReverse}, WheelBase{wheelBase}
    {
        Align();
    }

    /**
     * Aligns the front wheels based on the
     * rear wheels, orientation and wheelbase
     */
    void Align();

    /**
     * State of rear axis / wheels
     */
    PointState Rear{};

    /**
     * State of front axis / wheels
     */
    PointState Front{};

    /**
     * Whether the car is travelling in reverse or forward
     */
    bool InReverse{false};

    /**
     * The distance between rear and front. Used for alignment.
     */
    float WheelBase{0.0f};
};

// returns the distance in R3
inline float EuclideanDistance(const PointState &a, const PointState &b)
{
    return math::Distance(a.Pos, b.Pos);
}

/**
 * Returns the total rotation required in radians between two states
 * takes into account two rotations when the distance 'from' state is far from the 'to' state.
 * when we are at a larger distance than we have to rotate first towards the point
 * to go there, before we can rotate back
 */
math::Angle<float> OrientationDistance(const PointState &from, const PointState &to, const float distance);

math::Angle<float> OrientationDistance(const PointState &from, const PointState &to);

/**
 * Returns the configuration distance between two point states as a single scalar.
 * Based on both euclidean distance and orientation distance.
 */
float ConfigurationDistance(const PointState &a, const PointState &b);

/** 
 * Returns the state distance between two robot states as a single scalar.
 * This is the sum of the state distance of only the rear wheels.
 */
float ConfigurationDistance(const CarState &a, const CarState &b);


/**
 * Returns the optimal path to go from 'from' towards 'to'. Uses Reeds-Shepp' algorithm.
 */
algorithm::Path SearchShortestPath(const PointState &from, const PointState &to);

/**
 * Returns the optimal path to go from the carstate 'from' towards pointstate 'to'. Uses Reeds-Shepp algorithm.
 */
algorithm::Path SearchShortestPath(const CarState &from, const PointState &to);


/**
 * Moves the car along the path given the normalized value [0.f-1.0f] with the given carstate as startpoint. Returns a new car state.
 */
CarState TraversePathNormalized(float normalizedProgress, const algorithm::Path &path, const CarState& start);

/**
 * Moves the car along the path given the distance travelled with the given carstate as startpoint. Returns a new car state.
 */
CarState TraversePathDistance(float distance, const algorithm::Path &path, const CarState &start);

} // namespace rsmotion

namespace std
{

std::string to_string(const rsmotion::algorithm::State &o);
std::string to_string(const rsmotion::algorithm::SegmentType o);
std::string to_string(const rsmotion::algorithm::SegmentDirection o);
std::string to_string(const rsmotion::algorithm::SegmentLengthConstraint o);
std::string to_string(const rsmotion::algorithm::Segment &o);
std::string to_string(const rsmotion::algorithm::Path &o);

} // namespace std

#endif
