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

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define _USE_MATH_DEFINES
#include <cmath>

#include <rsmotion/rsmotion.h>

using namespace rsmotion;
using namespace rsmotion::algorithm;

using std::acos;
using std::atan2;
using std::cos;
using std::fabs;
using std::fmod;
using std::sin;
using std::sqrt;

namespace
{

template <class T>
class pi
{
public:
    static constexpr T value() { return static_cast<T>(rsmotion::math::Pi()); }
    static constexpr T two() { return static_cast<T>(rsmotion::math::TwoPi()); }
    static constexpr T half() { return static_cast<T>(rsmotion::math::HalfPi()); }
};

#ifndef NDEBUG
const auto RS_EPS = 1e-5f;
#endif

const auto ZERO = 10 * std::numeric_limits<float>::epsilon();

// predefined segments
const auto LP = Segment{SegmentType::Left};
const auto LM = Segment{SegmentType::Left, SegmentDirection::Bwd};
const auto LUM = Segment{SegmentType::Left, SegmentDirection::Bwd, SegmentLengthConstraint::Equal};
const auto LUP = Segment{SegmentType::Left, SegmentDirection::Fwd, SegmentLengthConstraint::Equal};
const auto LHPIM = Segment{SegmentType::Left, SegmentDirection::Bwd, SegmentLengthConstraint::HalfPI};
const auto LHPIP = Segment{SegmentType::Left, SegmentDirection::Fwd, SegmentLengthConstraint::HalfPI};

const auto SP = Segment{SegmentType::Straight};
const auto SM = Segment{SegmentType::Straight, SegmentDirection::Bwd};

const auto RP = Segment{SegmentType::Right};
const auto RM = Segment{SegmentType::Right, SegmentDirection::Bwd};
const auto RUM = Segment{SegmentType::Right, SegmentDirection::Bwd, SegmentLengthConstraint::Equal};
const auto RUP = Segment{SegmentType::Right, SegmentDirection::Fwd, SegmentLengthConstraint::Equal};
const auto RHPIM = Segment{SegmentType::Right, SegmentDirection::Bwd, SegmentLengthConstraint::HalfPI};
const auto RHPIP = Segment{SegmentType::Right, SegmentDirection::Fwd, SegmentLengthConstraint::HalfPI};

State InvertState(const State &toState)
{
    return {
        toState.X() * cos(toState.Phi()) + toState.Y() * sin(toState.Phi()),
        toState.X() * sin(toState.Phi()) - toState.Y() * cos(toState.Phi()),
        toState.Phi()};
}

template <class R>
Path CalculateMinPath(Path basePath, State toState, R baseFormula)
{
    Path minPath;

    // for each equation, try each operation and return the shortest
    if (baseFormula(toState, basePath))
    {
        minPath = std::min(minPath, basePath);
    }
    if (baseFormula(Reflect(toState), basePath))
    {
        minPath = std::min(minPath, Reflect(basePath));
    }
    if (baseFormula(Timeflip(toState), basePath))
    {
        minPath = std::min(minPath, Timeflip(basePath));
    }
    if (baseFormula(Reflect(Timeflip(toState)), basePath))
    {
        minPath = std::min(minPath, Reflect(Timeflip(basePath)));
    }

    return minPath;
}

// some helper functions
template <class T>
inline T mod2pi(T x)
{
    T v = fmod(x, pi<T>::two());
    if (v < -pi<T>::value())
        v += pi<T>::two();
    else if (v > pi<T>::value())
        v -= pi<T>::two();
    return v;
}

template <class T>
struct Polar
{
    T r;
    T theta;
};

template <class T>
inline Polar<T> polar(T x, T y)
{
    return {sqrt(x * x + y * y), atan2(y, x)};
}

template <class T>
struct TauOmega
{
    T tau;
    T omega;
};

template <class T>
inline TauOmega<T> tauOmega(T u, T v, T xi, T eta, T phi)
{
    auto delta = mod2pi(u - v), A = sin(u) - sin(delta), B = cos(u) - cos(delta) - 1;
    auto t1 = atan2(eta * A - xi * B, xi * A + eta * B), t2 = 2 * (cos(delta) - cos(v) - cos(u)) + 3;
    auto tau = (t2 < 0) ? mod2pi(t1 + pi<T>::value()) : mod2pi(t1);
    return {tau, mod2pi(tau - u + v - phi)};
}

// formula 8.1 in Reeds-Shepp paper
template <class T>
inline bool LpSpLp(T x, T y, T phi, T &t, T &u, T &v)
{
    auto p = polar(x - sin(phi), y - 1 + cos(phi));
    t = p.theta;
    u = p.r;
    if (t >= -ZERO)
    {
        v = mod2pi(phi - t);
        if (v >= -ZERO)
        {
            assert(fabs(u * cos(t) + sin(phi) - x) < RS_EPS);
            assert(fabs(u * sin(t) - cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t + v - phi)) < RS_EPS);
            return true;
        }
    }
    return false;
}

// formula 8.2
template <class T>
inline bool LpSpRp(T x, T y, T phi, T &t, T &u, T &v)
{
    auto pol = polar(x + sin(phi), y - 1 - cos(phi));
    pol.r = pol.r * pol.r;
    if (pol.r >= ((T)4))
    {
        u = sqrt(pol.r - 4);
        auto theta = atan2((T)2, u);
        t = mod2pi(pol.theta + theta);
        v = mod2pi(t - phi);
        assert(fabs(2 * sin(t) + u * cos(t) - sin(phi) - x) < RS_EPS);
        assert(fabs(-2 * cos(t) + u * sin(t) + cos(phi) + 1 - y) < RS_EPS);
        assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
        return t >= -ZERO && v >= -ZERO;
    }
    return false;
}

// formula 8.3 / 8.4  *** TYPO IN PAPER ***
template <class T>
inline bool LpRmL(T x, T y, T phi, T &t, T &u, T &v)
{
    auto xi = x - sin(phi);
    auto eta = y - 1 + cos(phi);
    auto p = polar(xi, eta);
    if (p.r <= 4)
    {
        u = -2 * asin(((T).25) * p.r);
        t = mod2pi(p.theta + ((T).5) * u + (pi<T>::value()));
        v = mod2pi(phi - t + u);
        assert(fabs(2 * (sin(t) - sin(t - u)) + sin(phi) - x) < RS_EPS);
        assert(fabs(2 * (-cos(t) + cos(t - u)) - cos(phi) + 1 - y) < RS_EPS);
        assert(fabs(mod2pi(t - u + v - phi)) < RS_EPS);
        return t >= -ZERO && u <= ZERO;
    }
    return false;
}

Path CSC(State toState)
{
    return std::min(
        CalculateMinPath({LP, SP, LP}, toState, [](const State &s, Path &p) {
            return LpSpLp(s.X(), s.Y(), s.Phi(), p.Distance(0), p.Distance(1), p.Distance(2));
        }),
        CalculateMinPath({LP, SP, RP}, toState, [](const State &s, Path &p) { 
            return LpSpRp(s.X(), s.Y(), s.Phi(), p.Distance(0), p.Distance(1), p.Distance(2)); 
        }));
}

Path CCC(State toState)
{
    auto path = std::min(
        // forwards (8.3)
        CalculateMinPath({LP, RM, LP}, toState, [](const State &s, Path &p) {
            return LpRmL(s.X(), s.Y(), s.Phi(), p.Distance(0), p.Distance(1), p.Distance(2));
        }),
        // backwards (8.4).
        CalculateMinPath({LP, RM, LP}, InvertState(toState), [](const State &s, Path &p) {
			// flipped segments!
			return LpRmL(s.X(), s.Y(), s.Phi(), p.Distance(2), p.Distance(1), p.Distance(0)); 
        }));

    // the first and third segment can be either positive or negative
    if (!path.Segments.empty())
    {
        path.Segments[2].Direction = path.Segments[2].Distance < 0 ? SegmentDirection::Bwd : SegmentDirection::Fwd;
        path.Segments[0].Direction = path.Segments[0].Distance < 0 ? SegmentDirection::Bwd : SegmentDirection::Fwd;
    }

    return path;
}

// formula 8.7
template <class T>
inline bool LpRupLumRm(T x, T y, T phi, T &t, T &u, T &u2, T &v)
{
    auto xi = x + sin(phi);
    auto eta = y - 1 - cos(phi);
    auto rho = ((T).25) * (2 + sqrt(xi * xi + eta * eta));
    if (rho <= 1.)
    {
        u = acos(rho);
        u2 = -u;
        auto to = tauOmega(u, -u, xi, eta, phi);
        t = to.tau;
        v = to.omega;
        assert(fabs(2 * (sin(t) - sin(t - u) + sin(t - 2 * u)) - sin(phi) - x) < RS_EPS);
        assert(fabs(2 * (-cos(t) + cos(t - u) - cos(t - 2 * u)) + cos(phi) + 1 - y) < RS_EPS);
        assert(fabs(mod2pi(t - 2 * u - v - phi)) < RS_EPS);
        return t >= -ZERO && v <= ZERO;
    }
    return false;
}

// formula 8.8
template <class T>
inline bool LpRumLumRp(T x, T y, T phi, T &t, T &u, T &u2, T &v)
{
    auto xi = x + sin(phi);
    auto eta = y - 1 - cos(phi);
    auto rho = (20 - xi * xi - eta * eta) / 16;
    if (rho >= 0 && rho <= 1)
    {
        u = -acos(rho);
        if (u >= -pi<T>::half())
        {
            auto to = tauOmega(u, u, xi, eta, phi);
            t = to.tau;
            v = to.omega;
            u2 = u;
            // TODO: check out why db is not used?
            // auto db = fabs(4 * sin(t) - 2 * sin(t - u) - sin(phi) - x);
            assert(fabs(4 * sin(t) - 2 * sin(t - u) - sin(phi) - x) < RS_EPS);
            assert(fabs(-4 * cos(t) + 2 * cos(t - u) + cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
            return t >= -ZERO && v >= -ZERO;
        }
    }
    return false;
}

Path CCCC(State toState)
{
    return std::min(
        CalculateMinPath({LP, RUP, LUM, RM}, toState, [](const State &s, Path &p) {
            return LpRupLumRm(s.X(), s.Y(), s.Phi(), p.Distance(0), p.Distance(1), p.Distance(2), p.Distance(3));
        }),
        CalculateMinPath({LP, RUM, LUM, RP}, toState, [](const State &s, Path &p) { return LpRumLumRp(s.X(), s.Y(), s.Phi(), p.Distance(0), p.Distance(1), p.Distance(2), p.Distance(3)); }));
}

// formula 8.9
template <class T>
inline bool LpRmSmLm(T x, T y, T phi, T &t, T &constrained, T &u, T &v)
{
    auto xi = x - sin(phi);
    auto eta = y - 1 + cos(phi);
    auto p = polar(xi, eta);
    if (p.r >= 2)
    {
        auto r = sqrt(p.r * p.r - (T)(4));
        constrained = -pi<T>::half();
        u = (T)(2) - r;
        t = mod2pi(p.theta + atan2(r, (T)(-2)));
        v = mod2pi(phi - pi<T>::half() - t);
        // auto db1 = fabs(2 * (sin(t) - cos(t)) - u * sin(t) + sin(phi) - x);
        assert(fabs(2 * (sin(t) - cos(t)) - u * sin(t) + sin(phi) - x) < RS_EPS);
        // auto db = fabs(-2 * (sin(t) + cos(t)) + u * cos(t) - cos(phi) + 1 - y);
        assert(fabs(-2 * (sin(t) + cos(t)) + u * cos(t) - cos(phi) + 1 - y) < RS_EPS);
        assert(fabs(mod2pi(t + (pi<T>::value()) / 2 + v - phi)) < RS_EPS);
        return t >= -ZERO && u <= ZERO && v <= ZERO;
    }
    return false;
}

// formula 8.10
template <class T>
inline bool LpRmSmRm(T x, T y, T phi, T &t, T &constrained, T &u, T &v)
{
    auto xi = x + sin(phi);
    auto eta = y - 1 - cos(phi);
    auto p = polar(-eta, xi);
    if (p.r >= 2)
    {
        constrained = -pi<T>::half();
        t = p.theta;
        u = 2 - p.r;
        v = mod2pi(t + pi<T>::half() - phi);
        assert(fabs(2 * sin(t) - cos(t - v) - u * sin(t) - x) < RS_EPS);
        // auto db = fabs(-2 * cos(t) - sin(t - v) + u * cos(t) + 1 - y);
        assert(fabs(-2 * cos(t) - sin(t - v) + u * cos(t) + 1 - y) < RS_EPS);
        assert(fabs(mod2pi(t + (pi<T>::value()) / 2 - v - phi)) < RS_EPS);
        return t >= -ZERO && u <= ZERO && v <= ZERO;
    }
    return false;
}

Path CCSC(State toState)
{
    auto p1 = CalculateMinPath({LP, RM, SM, LM}, toState, [](const State &s, Path &p) {
        return LpRmSmLm(s.X(), s.Y(), s.Phi(), p.Distance(0), p.Distance(1), p.Distance(2), p.Distance(3));
    });
    auto p2 = CalculateMinPath({LP, RM, SM, RM}, toState, [](const State &s, Path &p) {
        return LpRmSmRm(s.X(), s.Y(), s.Phi(), p.Distance(0), p.Distance(1), p.Distance(2), p.Distance(3));
    });

    auto invertedState = InvertState(toState);

    // backwards
    auto p3 = CalculateMinPath({LM, SM, RM, LP}, invertedState, [](const State &s, Path &p) {
        return LpRmSmLm(s.X(), s.Y(), s.Phi(), p.Distance(3), p.Distance(2), p.Distance(1), p.Distance(0));
    });

    auto p4 = CalculateMinPath({RM, SM, RM, LP}, invertedState, [](const State &s, Path &p) {
        return LpRmSmRm(s.X(), s.Y(), s.Phi(), p.Distance(3), p.Distance(2), p.Distance(1), p.Distance(0));
    });

    return std::min({p1, p2, p3, p4});
}

// formula 8.11 *** TYPO IN PAPER ***
template <class T>
inline bool LpRmSLmRp(T x, T y, T phi, T &t, T &constrained1, T &u, T &constrained2, T &v)
{
    auto xi = x + sin(phi);
    auto eta = y - 1 - cos(phi);
    auto p = polar(xi, eta);
    if (p.r >= 2)
    {
        u = 4 - sqrt(p.r * p.r - 4);
        if (u <= ZERO)
        {
            t = mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
            v = mod2pi(t - phi);
            constrained1 = -pi<T>::half();
            constrained2 = -pi<T>::half();
            assert(fabs(4 * sin(t) - 2 * cos(t) - u * sin(t) - sin(phi) - x) < RS_EPS);
            assert(fabs(-4 * cos(t) - 2 * sin(t) + u * cos(t) + cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
            return t >= -ZERO && v >= -ZERO;
        }
    }
    return false;
}

Path CCSCC(State toState)
{
    return CalculateMinPath({LP, RHPIM, SM, LHPIM, RP}, toState, [](const State &s, Path &p) {
        return LpRmSLmRp(s.X(), s.Y(), s.Phi(), p.Distance(0), p.Distance(1), p.Distance(2), p.Distance(3), p.Distance(4));
    });
}

State StateAfterInterpolation(const State &s, const Segment &seg, float v)
{
    auto phi = s.Phi();
    switch (seg.Type)
    {
    case SegmentType::Left:
        return {s.X() + sin(phi + v) - sin(phi), s.Y() - cos(phi + v) + cos(phi), phi + v, v < 0};
    case SegmentType::Right:
        return {s.X() - sin(phi - v) + sin(phi), s.Y() + cos(phi - v) - cos(phi), phi - v, v < 0};
    case SegmentType::Straight:
        return {s.X() + v * cos(phi), s.Y() + v * sin(phi), phi, v < 0};
    default:
        return {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0, v < 0};
    }
}

template <class T>
inline T EnforceSO2Bounds(T val)
{
    T v = fmod(val, 2 * pi<T>::value());
    if (v <= -pi<T>::value())
        v += 2 * pi<T>::value();
    else if (v > pi<T>::value())
        v -= 2 * pi<T>::value();
    return v;
}

} // namespace

namespace rsmotion
{

namespace algorithm {

inline State Timeflip(const State &s)
{
    return State{-s.X(), s.Y(), -s.Phi()};
}

inline State Reflect(const State &s)
{
    return State{s.X(), -s.Y(), -s.Phi()};
}

SegmentType Reflect(const SegmentType &d)
{
    switch (d)
    {
    case SegmentType::Straight:
        return SegmentType::Straight;
    case SegmentType::Left:
        return SegmentType::Right;
    case SegmentType::Right:
        return SegmentType::Left;
    default:
        return SegmentType::None;    
    }
}

SegmentDirection Timeflip(const SegmentDirection &d)
{
    return d == SegmentDirection::Fwd ? SegmentDirection::Bwd : SegmentDirection::Fwd;
}

Segment Reflect(const Segment &x)
{
    return {Reflect(x.Type), x.Direction, x.Constraint, x.Distance};
}

Segment Timeflip(const Segment &x)
{
    return {x.Type, Timeflip(x.Direction), x.Constraint, -x.Distance};
}

Path::Path(std::initializer_list<Segment> segments) noexcept
{
    std::copy(std::begin(segments), std::end(segments), std::back_inserter(Segments));
}

float Path::Length() const
{
    if (Segments.empty())
    {
        return std::numeric_limits<float>::max();
    }

    float l = 0.f;
    for (auto &s : Segments)
    {
        if (s.Infinite())
            return std::numeric_limits<float>::max();
        l += s.Length();
    }
    return l;
}

bool Path::Valid() const
{
    return Length() < std::numeric_limits<float>::max();
}

float &Path::Distance(std::size_t n)
{
    return Segments.at(n).Distance;
}

Path::operator bool() const
{
    return Valid();
}

bool Path::operator<(const Path &rhs) const
{
    return Length() < rhs.Length();
}


Path Reflect(const Path &p)
{
    auto cpy = p;
    for (auto &s : cpy.Segments)
    {
        s = Reflect(s);
    }
    return cpy;
}

Path Timeflip(const Path &p)
{
    auto cpy = p;
    for (auto &s : cpy.Segments)
    {
        s = Timeflip(s);
    }
    return cpy;
}

float EnforceSO2Boundsf(float val)
{
    return EnforceSO2Bounds<float>(val);
}

Path SearchShortestPath(State toState)
{
    // the optimal path is always one of
    // these five path configurations
    return std::min({CSC(toState),
                     CCC(toState),
                     CCCC(toState),
                     CCSC(toState),
                     CCSCC(toState)});
}

State InterpolateDistance(const State &from, const Path &path, const float dist)
{
    auto remaining = dist;
    State to{0, 0, from.Phi()};
    for (auto i = 0; i < path.Segments.size() && remaining > 0; ++i)
    {
        auto &segment = path.Segments[i];
        auto v = segment.Distance < 0 ? std::max(-remaining, segment.Distance) : std::min(remaining, segment.Distance);
        remaining -= std::abs(v);
        to = StateAfterInterpolation(to, segment, v);
    }

    return {from.X() + to.X(), from.Y() + to.Y(), EnforceSO2Bounds(to.Phi()), to.InReverse()};
}

State InterpolateNormalized(const State &from, const Path &path, float t)
{
    auto pathLength = path.Length();
    if (pathLength == std::numeric_limits<float>::max())
    {
        return from;
    }

    return InterpolateDistance(from, path, t * pathLength);
}

}

}

namespace
{
using namespace rsmotion;
using namespace rsmotion::algorithm;

CarState CarFromRSState(const CarState& start, const State &rsState)
{
    CarState s = start;
    
    // convert back to car state
    s.Rear.Pos % FW = rsState.X();
    s.Rear.Pos % RI = rsState.Y();
    s.Rear.Orientation = rsmotion::math::Quatf(CoordinateSystem::Up(), rsmotion::math::Anglef::Radians(rsState.Phi()));    
    s.InReverse = rsState.InReverse();

    s.Align();

    return s;
}

} // namespace


namespace rsmotion
{

// rotate the toState in such a way that the current state is at 0,0,0 pointing forward
// the returning state is ready to be used as input for a search
algorithm::State ToRSSearchState(const PointState &fromState, const PointState &toState)
{
    // rotate the end state in such a way that the current state is at 0,0,0 pointing forward
    auto &rot = fromState.Orientation;
    auto fState = PointState{math::Conjugate(rot) * (toState.Pos - fromState.Pos),
                             math::Conjugate(rot) * toState.Orientation};
    return {
        fState.Pos % FW,
        fState.Pos % RI,
        algorithm::EnforceSO2Boundsf(RotationInXY(fState.Orientation).Value())};
}

// Gets the corresponding algorithm state based on the rear position and rear orientation
algorithm::State ToRSState(const PointState &ps)
{
    return {
        ps.Pos % FW,
        ps.Pos % RI,
        algorithm::EnforceSO2Boundsf(RotationInXY(ps.Orientation).Value())};
}

algorithm::Path SearchShortestPath(const PointState &from, const PointState &to)
{
    return algorithm::SearchShortestPath(ToRSSearchState(from, to));
}

algorithm::Path SearchShortestPath(const CarState &from, const PointState &to)
{
    return SearchShortestPath(from.Rear, to);
}

void CarState::Align()
{
    // use the rear axis to align
    Front.Pos = Rear.Pos + (Rear.Orientation * CoordinateSystem::Forward()) * WheelBase;    
}

CarState TraversePathNormalized(float normalizedProgress, const algorithm::Path &path, const CarState &start)
{
    auto rsState = algorithm::InterpolateNormalized(ToRSState(start.Rear), path, normalizedProgress);
    return CarFromRSState(start, rsState);
}

CarState TraversePathDistance(float distance, const algorithm::Path &path, const CarState &start)
{
    auto rsState = algorithm::InterpolateDistance(ToRSState(start.Rear), path, distance);
    return CarFromRSState(start, rsState);
}


math::Anglef OrientationDistance(const PointState &from, const PointState &to, const float distance)
{
    if (distance > 0.2f)
    {
        auto heading = AbsoluteAngle(ToNormalized(to.Pos - from.Pos));
        auto angleFromState = math::Anglef::ArcCos(Dot(from.Orientation, heading));
        auto angleToState = math::Anglef::ArcCos(Dot(heading, to.Orientation));
        return angleFromState + angleToState;
    }

    return math::Anglef::ArcCos(Dot(from.Orientation, to.Orientation)) * 0.2f;
}

math::Anglef OrientationDistance(const PointState &from, const PointState &to)
{
    return OrientationDistance(from, to, EuclideanDistance(from, to));
}

// returns the configuration distance between two point states as a single scalar
// Based on euclidean distance and orientation distance
float ConfigurationDistance(const PointState &a, const PointState &b)
{
    auto d = EuclideanDistance(a, b);
    return OrientationDistance(a, b, d).Value() + d;
}

// returns the state distance between two robot states as a single scalar
// is the sum of the state distance of only the rear wheels
float ConfigurationDistance(const CarState &a, const CarState &b)
{
    return ConfigurationDistance(a.Rear, b.Rear);
}


} // namespace rsmotion


///
/// to_string definitions
///

namespace std
{

std::string to_string(const rsmotion::algorithm::State &o)
{
    std::string str("[");
    str.append(std::to_string(o.X()))
        .append(",")
        .append(std::to_string(o.Y()))
        .append(",")
        .append(std::to_string(o.Phi()))
        .append("=")
        .append(std::to_string(static_cast<int32_t>((o.Phi() / 2. * rsmotion::math::Pi()) * 360)))
        .append("]");
    return str;
}

std::string to_string(const rsmotion::algorithm::SegmentType o)
{
    using namespace rsmotion::algorithm;
    switch (o)
    {
    case SegmentType::Straight:
        return "S";
    case SegmentType::Left:
        return "L";
    case SegmentType::Right:
        return "R";
    default:
        return "?";
    }
}

std::string to_string(const rsmotion::algorithm::SegmentDirection o)
{
    using namespace rsmotion::algorithm;
    switch (o)
    {
    case SegmentDirection::Fwd:
        return "+";
    case SegmentDirection::Bwd:
        return "-";
    default:
        return "?";
    }
}

std::string to_string(const rsmotion::algorithm::SegmentLengthConstraint o)
{
    using namespace rsmotion::algorithm;
    switch (o)
    {
    case SegmentLengthConstraint::Unconstrained:
        return "";
    case SegmentLengthConstraint::HalfPI:
        return "PI";
    case SegmentLengthConstraint::Equal:
        return "e";
    default:
        return "?";
    }
}

std::string to_string(const rsmotion::algorithm::Segment &o)
{
    return std::to_string(o.Type) + std::to_string(o.Direction) + std::to_string(o.Constraint) + "(" + (o.Infinite() ? std::string{"INF"} : std::to_string(o.Distance)) + ")";
}

std::string to_string(const rsmotion::algorithm::Path &o)
{
    std::string b = "P[";
    for (auto &s : o.Segments)
    {
        b += std::to_string(s);
        b += " ";
    }
    b += "LEN: " + std::to_string(o.Length());
    b += "]";
    return b;
}
} // namespace std