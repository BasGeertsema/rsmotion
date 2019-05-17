/**
 *  @author Matus Chochlik
 *
 *  Copyright 2010-2014 Matus Chochlik. Distributed under the Boost
 *  Software License, Version 1.0. (See accompanying file
 *  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
 */

/** 
 * @author Bas Geertsema
 * 
 * The oglplus math library has been trimmed down and moved to a different namespace
 * to simplify the rsmotion library. This makes it easier to embed the library
 * without a dependency on oglplus. 
 */

#include <type_traits>
#include <cstdint>
#include <cassert>
#include <limits>

namespace rsmotion
{
namespace math
{

template <typename T, std::size_t N>
class Vector;

typedef float AngleValueType;

// Nothing
struct Nothing
{
    typedef Nothing Type;
};

#ifdef M_PI
inline decltype(M_PI) Pi(void)
{
    return M_PI;
}

inline decltype(2 * M_PI) TwoPi(void)
{
    return 2 * M_PI;
}

inline decltype(0.5 * M_PI) HalfPi(void)
{
    return 0.5 * M_PI;
}
#else
inline decltype(std::atan(1.0) * 4.0) Pi(void)
{
    static auto _pi = std::atan(1.0) * 4.0;
    return _pi;
}

inline decltype(std::atan(1.0) * 8.0) TwoPi(void)
{
    static auto _pi = std::atan(1.0) * 8.0;
    return _pi;
}

inline decltype(std::atan(1.0) * 2.0) HalfPi(void)
{
    static auto _pi = std::atan(1.0) * 2.0;
    return _pi;
}
#endif

/// Class implementing planar angle-related functionality
/** @c Angle is a lightweight class allowing more natural construction and
 *  usage of planar angle values. The storage requirements are the same
 *  as for the template parameter type @c T, but the @c Angle template gives
 *  the @c T type special meaning and implements a set of angle-related member
 *  and friend functions. There are also several associated free functions
 *  for creating new instances of @c Angle.
 *
 *  @see oglplus::Radians
 *  @see oglplus::Degrees
 *  @see oglplus::FullCircles
 *  @see oglplus::RightAngles
 *  @see oglplus::ArcSin
 *  @see oglplus::ArcCos
 *  @see oglplus::ArcTan
 *
 *  @ingroup math_utils
 */
template <typename T>
class Angle
{
private:
    // the angle value in radians
    T _val_rad;

    struct Radians_
    {
    };
    Angle(T val_rad, Radians_)
        : _val_rad(val_rad)
    {
    }

    struct Degrees_
    {
    };
    Angle(T val_deg, Degrees_)
        : _val_rad(T(val_deg * (math::Pi() / T(180))))
    {
    }

public:
    /// Constructs a zero angle
    Angle(void)
        : _val_rad(T(0))
    {
    }

    /// Angle is copy constructible
    Angle(const Angle &) = default;

    Angle &operator=(const Angle &) = default;

    /// Angle is move constructible
    Angle(Angle &&) = default;

    /// Copy construction from angles using different underlying type
    template <typename U>
    Angle(const Angle<U> &other)
        : _val_rad(T(other.Value()))
    {
    }

    /// Constructs a new angle from value in radians
    static inline Angle Radians(T val_rad)
    {
        return Angle(val_rad, Radians_());
    }

    /// Constructs a new angle from value in degrees
    static inline Angle Degrees(T val_deg)
    {
        return Angle(val_deg, Degrees_());
    }

    /// Constructs a new angle using arc sine
    static inline Angle ArcSin(T x)
    {
        assert(-1.0f <= x && x <= 1.0f);
        return Angle(::std::asin(x), Radians_());
    }

    /// Constructs a new angle using arc cosine
    static inline Angle ArcCos(T x)
    {
        assert(-1.0f <= x && x <= 1.0f);
        return Angle(::std::acos(x), Radians_());
    }

    /// Returns the value of the angle in radians
    inline T Value(void) const
    {
        return _val_rad;
    }

    /// Returns the value of the angle in degrees
    inline T ValueInDegrees(void) const
    {
        return _val_rad * T(180 / math::Pi());
    }

    /// Returns the value of the angle in number of right angles
    inline T ValueInRightAngles(void) const
    {
        return _val_rad * T(2.0 / math::Pi());
    }

    /// Returns the value of the angle in number of full circles
    inline T ValueInFullCircles(void) const
    {
        return _val_rad * T(0.5 / math::Pi());
    }

    /// Equality comparison
    friend bool operator==(const Angle &a, const Angle &b)
    {
        return a._val_rad == b._val_rad;
    }

    /// Inequality comparison
    friend bool operator!=(const Angle &a, const Angle &b)
    {
        return a._val_rad != b._val_rad;
    }

    /// Less than comparison
    friend bool operator<(const Angle &a, const Angle &b)
    {
        return a._val_rad < b._val_rad;
    }

    /// Greater than comparison
    friend bool operator>(const Angle &a, const Angle &b)
    {
        return a._val_rad > b._val_rad;
    }

    /// Less than/equal comparison
    friend bool operator<=(const Angle &a, const Angle &b)
    {
        return a._val_rad <= b._val_rad;
    }

    /// Greater than/equal comparison
    friend bool operator>=(const Angle &a, const Angle &b)
    {
        return a._val_rad >= b._val_rad;
    }

    /// Negation
    Angle Negated(void) const
    {
        return Angle(-this->_val_rad, Radians_());
    }

    /// Negation operator
    friend Angle operator-(const Angle &angle)
    {
        return angle.Negated();
    }

    static Angle Added(const Angle &a, const Angle &b)
    {
        return Angle(a._val_rad + b._val_rad, Radians_());
    }

    /// Addition operator
    friend Angle operator+(const Angle &a, const Angle &b)
    {
        return Added(a, b);
    }

    /// Addition operator
    Angle &operator+=(const Angle &b)
    {
        *this = Add(*this, b);
        return *this;
    }

    static Angle Subtracted(const Angle &a, const Angle &b)
    {
        return Angle(a._val_rad - b._val_rad, Radians_());
    }

    /// Subtraction operator
    friend Angle operator-(const Angle &a, const Angle &b)
    {
        return Subtracted(a, b);
    }

    /// Subtraction operator
    Angle &operator-=(const Angle &b)
    {
        *this = Subtracted(*this, b);
        return *this;
    }

    static Angle Multiplied(const Angle &a, T mult)
    {
        return Angle(a._val_rad * mult, Radians_());
    }

    /// Multiplication by constant operator
    friend Angle operator*(const Angle &a, T mult)
    {
        return Multiplied(a, mult);
    }

    /// Multiplication by constant operator
    friend Angle operator*(T mult, const Angle &a)
    {
        return Multiplied(a, mult);
    }

    /// Multiplication by constant operator
    Angle &operator*=(T mult)
    {
        *this = Multiplied(*this, mult);
        return *this;
    }

    static Angle Divided(const Angle &a, T div)
    {
        assert(div != T(0));
        return Angle(a._val_rad / div, Radians_());
    }

    /// Division by constant operator
    friend Angle operator/(const Angle &a, T div)
    {
        return Divided(a, div);
    }

    /// Division by constant operator
    Angle &operator/=(T div)
    {
        *this = Divided(*this, div);
        return *this;
    }

    static T Ratio(const Angle &a, const Angle &b)
    {
        assert(b._val_rad != T(0));
        return a._val_rad / b._val_rad;
    }

    /// Ratio operator
    friend T operator/(const Angle &a, const Angle &b)
    {
        return Ratio(a, b);
    }

    /// Returns the sine of the angle
    T Sin(void) const
    {
        return ::std::sin(this->_val_rad);
    }

    /// Returns the cosine of the angle
    T Cos(void) const
    {
        return ::std::cos(this->_val_rad);
    }

    /// Returns the tangent of the angle
    T Tan(void) const
    {
        return ::std::tan(this->_val_rad);
    }
};

template <typename T>
inline Angle<T> Negate(const Angle<T> &a)
{
    return a.Negated();
}

template <typename T>
inline Angle<T> Add(const Angle<T> &a, const Angle<T> &b)
{
    return Angle<T>::Added(a, b);
}

template <typename T>
inline Angle<T> Subtract(const Angle<T> &a, const Angle<T> &b)
{
    return Angle<T>::Subtracted(a, b);
}

template <typename T>
inline Angle<T> Multiply(const Angle<T> &a, T v)
{
    return Angle<T>::Multiplied(a, v);
}

template <typename T>
inline Angle<T> Divide(const Angle<T> &a, T v)
{
    return Angle<T>::Divided(a, v);
}

template <typename T>
inline T Ratio(const Angle<T> &a, const Angle<T> &b)
{
    return Angle<T>::Ratio(a, b);
}

template <typename T>
inline T Radians(const Angle<T> &a)
{
    return a.Value();
}

template <typename T>
inline T Degrees(const Angle<T> &a)
{
    return a.ValueInDegrees();
}

template <typename T>
inline T Sin(const Angle<T> &a)
{
    return a.Sin();
}

template <typename T>
inline T Cos(const Angle<T> &a)
{
    return a.Cos();
}

template <typename T>
inline T Tan(const Angle<T> &a)
{
    return a.Tan();
}

/// Instantiation of Angle using GL floating-point as underlying type
typedef Angle<float> Anglef;

/// Creates a new angle from a value in radians
/** This function creates a new instance of @c Angle<T>
 *  from a floating-point value in radians.
 *
 *  @param val_rad a value in radians
 *
 *  @see Degrees
 *  @see FullCircles
 *  @see RightAngles
 *  @see ArcSin
 *  @see ArcCos
 *  @see ArcTan
 *
 *  @ingroup math_utils
 */
inline Angle<AngleValueType> Radians(AngleValueType val_rad)
{
    return Angle<AngleValueType>::Radians(val_rad);
}

/// Creates a new angle from a value in degrees
/** This function creates a new instance of @c Angle<AngleValueType>
 *  from a floating-point value in degrees.
 *  Examples:
 *  @code
 *  // create a 30 degree angle
 *  Degrees(30);
 *  // create a right angle
 *  Degrees(90);
 *  @endcode
 *
 *  @param val_deg a value in degrees
 *
 *  @see Radians
 *  @see FullCircles
 *  @see RightAngles
 *  @see ArcSin
 *  @see ArcCos
 *  @see ArcTan
 *
 *  @ingroup math_utils
 */
inline Angle<AngleValueType> Degrees(AngleValueType val_deg)
{
    return Angle<AngleValueType>::Degrees(val_deg);
}

/// Creates a new angle using the arc sine function
/**
 *  @param x the value must be between -1.0 and 1.0
 *
 *  @see Radians
 *  @see Degrees
 *  @see FullCircles
 *  @see RightAngles
 *  @see ArcCos
 *  @see ArcTan
 *
 *  @ingroup math_utils
 */
inline Angle<AngleValueType> ArcSin(AngleValueType x)
{
    return Angle<AngleValueType>::ArcSin(x);
}

/// Creates a new angle using the arc cosine function
/**
 *  @param x the value must be between -1.0 and 1.0
 *
 *  @see Radians
 *  @see Degrees
 *  @see FullCircles
 *  @see RightAngles
 *  @see ArcSin
 *  @see ArcTan
 *
 *  @ingroup math_utils
 */
inline Angle<AngleValueType> ArcCos(AngleValueType x)
{
    return Angle<AngleValueType>::ArcCos(x);
}

/// Creates a new angle using the arc tangent function
/**
 *
 *  @see Radians
 *  @see Degrees
 *  @see FullCircles
 *  @see RightAngles
 *  @see ArcSin
 *  @see ArcTan
 *
 *  @ingroup math_utils
 */
inline Angle<AngleValueType> ArcTan(AngleValueType x)
{
    return Angle<AngleValueType>::Radians(::std::atan(x));
}

/// Creates a new angle using the arc tangent function with 2 parameters
/**
 *
 *  @see Radians
 *  @see Degrees
 *  @see FullCircles
 *  @see RightAngles
 *  @see ArcSin
 *  @see ArcTan
 *
 *  @ingroup math_utils
 */
inline Angle<AngleValueType> ArcTan(AngleValueType y, AngleValueType x)
{
    return Angle<AngleValueType>::Radians(::std::atan2(y, x));
}

/// Common base class for vectors
template <typename T, std::size_t N>
class VectorBase
{
protected:
    T _elem[N];

    VectorBase(Nothing)
    {
    }

    VectorBase(void)
    {
        std::fill(_elem, _elem + N, T(0));
    }

    VectorBase(T v)
    {
        std::fill(_elem, _elem + N, v);
    }

    VectorBase(const T (&v)[N])
    {
        std::copy(v, v + N, _elem);
    }

    VectorBase(const T *v, std::size_t n)
    {
        assert(n >= N);

        std::copy(v, v + N, _elem);
    }

    VectorBase(const T *v, std::size_t n, T def)
    {
        if (n > N)
            n = N;
        std::copy(v, v + n, _elem);
        std::fill(_elem + n, _elem + N, def);
    }

public:
    struct Unit_
    {
    };

    VectorBase(const VectorBase &) = default;

    VectorBase(VectorBase &&) = default;

    VectorBase &operator=(const VectorBase &) = default;

    VectorBase &operator=(VectorBase &&) = default;

    /// The size (the number of components) of this vector
    static std::size_t Size(void)
    {
        return N;
    }

    /// Pointer to the components of this vector
    T *Data(void)
    {
        return this->_elem;
    }

    /// Pointer to the components of this vector
    const T *Data(void) const
    {
        return this->_elem;
    }

    /// Access to the i-th component of this vector
    /**
	 *  @pre (i < Size())
	 */
    T At(std::size_t i) const
    {
        assert(i < N);
        return _elem[i];
    }

    /// Access to the i-th component of this vector with a fallback
    /** Similar to At(i), but returns @c fallback if @c i is
	 *  greater than or equal to Size().
	 */
    T At(std::size_t i, T fallback) const
    {
        if (i < N)
            return _elem[i];
        else
            return fallback;
    }

    /// Access to the i-th component of this vector
    /**
	 *  @pre (i < Size())
	 */
    T &operator[](std::size_t i)
    {
        assert(i < N);
        return _elem[i];
    }

    /// Const access to the i-th component of this vector
    /**
	 *  @pre (i < Size())
	 */
    const T &operator[](std::size_t i) const
    {
        assert(i < N);
        return _elem[i];
    }

    /// Equality comparison
    friend bool Equal(const VectorBase &a, const VectorBase &b)
    {
        for (std::size_t i = 0; i != N; ++i)
            if (a._elem[i] != b._elem[i])
                return false;
        return true;
    }

    /// Adds @p v to this vector
    void Add(const VectorBase &v)
    {
        for (std::size_t i = 0; i != N; ++i)
            _elem[i] += v._elem[i];
    }

    /// Subtracts @p v from this vector
    void Subtract(const VectorBase &v)
    {
        for (std::size_t i = 0; i != N; ++i)
            _elem[i] -= v._elem[i];
    }

    /// Multiplies this vector by a scalar value
    void Multiply(T v)
    {
        for (std::size_t i = 0; i != N; ++i)
            _elem[i] *= v;
    }

    /// Multiplies the elements of this and that vector
    void Multiply(const VectorBase &that)
    {
        for (std::size_t i = 0; i != N; ++i)
            _elem[i] *= that._elem[i];
    }

    /// Divides this vector by a scalar value
    void Divide(T v)
    {
        for (std::size_t i = 0; i != N; ++i)
            _elem[i] /= v;
    }

    /// Divides the elements of this and that vector
    void Divide(const VectorBase &that)
    {
        for (std::size_t i = 0; i != N; ++i)
            _elem[i] /= that._elem[i];
    }

    /// Returns the lenght of this vector
    T Length(void) const
    {
        return std::sqrt(DotProduct(*this, *this));
    }

    /// Returns true if the vector is normal
    bool IsNormal(T eps = T(0)) const
    {
        return std::abs(DotProduct(*this, *this) - T(1)) <= eps;
    }

    /// Normalizes this vector
    void Normalize(void)
    {
        T l = Length();
        if (l != T(0) && l != T(1))
            Multiply(T(1) / l);
    }

    /// Computes the dot product of vectors @p a and @p b
    static T DotProduct(const VectorBase &a, const VectorBase &b)
    {
        T result = (a._elem[0] * b._elem[0]);
        for (std::size_t i = 1; i != N; ++i)
            result += (a._elem[i] * b._elem[i]);
        return result;
    }
};

template <typename T>
class Vector<T, 2>
    : public VectorBase<T, 2>
{
private:
    typedef VectorBase<T, 2> Base;
    typedef typename Base::Unit_ Unit_;

public:
    Vector(void)
    {
    }

    Vector(const T (&v)[2])
        : Base(v)
    {
    }

    Vector(const T *v, std::size_t n)
        : Base(v, n)
    {
    }

    explicit Vector(T v0)
        : Base(v0)
    {
    }

    Vector(T v0, T v1)
        : Base(Nothing())
    {
        this->_elem[0] = v0;
        this->_elem[1] = v1;
    }

    template <typename U>
    Vector(const Vector<U, 1> &v, T v1)
        : Base(Nothing())
    {
        this->_elem[0] = T(v[0]);
        this->_elem[1] = v1;
    }

    Vector(Unit_, std::size_t axis)
    {
        assert(axis < 2);
        this->_elem[axis] = T(1);
    }

    static Vector Unit(std::size_t axis)
    {
        return Vector(Unit_(), axis);
    }

    T x(void) const
    {
        return this->At(0);
    }

    T y(void) const
    {
        return this->At(1);
    }

    friend Vector Negated(const Vector &a)
    {
        return Vector(-a[0], -a[1]);
    }

    friend Vector Added(const Vector &a, const Vector &b)
    {
        return Vector(a[0] + b[0], a[1] + b[1]);
    }

    Vector &operator+=(const Vector &v)
    {
        this->Add(v);
        return *this;
    }

    friend Vector Subtracted(const Vector &a, const Vector &b)
    {
        return Vector(a[0] - b[0], a[1] - b[1]);
    }

    Vector &operator-=(const Vector &v)
    {
        this->Subtract(v);
        return *this;
    }

    friend Vector Multiplied(const Vector &a, T v)
    {
        return Vector(a[0] * v, a[1] * v);
    }

    Vector &operator*=(T v)
    {
        this->Multiply(v);
        return *this;
    }

    Vector &operator*=(const Vector &v)
    {
        this->Multiply(v);
        return *this;
    }

    friend Vector Divided(const Vector &a, T v)
    {
        return Vector(a[0] / v, a[1] / v);
    }

    Vector &operator/=(T v)
    {
        this->Divide(v);
        return *this;
    }
};

template <typename T>
class Vector<T, 3>
    : public VectorBase<T, 3>
{
private:
    typedef VectorBase<T, 3> Base;
    typedef typename Base::Unit_ Unit_;

public:
    Vector(void)
    {
    }

    Vector(const T (&v)[3])
        : Base(v)
    {
    }

    Vector(const T *v, std::size_t n)
        : Base(v, n)
    {
    }

    explicit Vector(T v0)
        : Base(v0)
    {
    }

    Vector(T v0, T v1, T v2)
        : Base(Nothing())
    {
        this->_elem[0] = v0;
        this->_elem[1] = v1;
        this->_elem[2] = v2;
    }

    template <typename U>
    Vector(const Vector<U, 1> &v, T v1, T v2)
        : Base(Nothing())
    {
        this->_elem[0] = T(v[0]);
        this->_elem[1] = v1;
        this->_elem[2] = v2;
    }

    template <typename U>
    Vector(const Vector<U, 2> &v, T v2)
        : Base(Nothing())
    {
        this->_elem[0] = T(v[0]);
        this->_elem[1] = T(v[1]);
        this->_elem[2] = v2;
    }

    Vector(Unit_, std::size_t axis)
    {
        assert(axis < 3);
        this->_elem[axis] = T(1);
    }

    static Vector Unit(std::size_t axis)
    {
        return Vector(Unit_(), axis);
    }

    T x(void) const
    {
        return this->At(0);
    }

    T y(void) const
    {
        return this->At(1);
    }

    T z(void) const
    {
        return this->At(2);
    }

    Vector<T, 2> xy(void) const
    {
        return Vector<T, 2>(this->At(0), this->At(1));
    }

    friend Vector Negated(const Vector &a)
    {
        return Vector(-a[0], -a[1], -a[2]);
    }

    friend Vector Added(const Vector &a, const Vector &b)
    {
        return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
    }

    Vector &operator+=(const Vector &v)
    {
        this->Add(v);
        return *this;
    }

    friend Vector Subtracted(const Vector &a, const Vector &b)
    {
        return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    }

    Vector &operator-=(const Vector &v)
    {
        this->Subtract(v);
        return *this;
    }

    friend Vector Multiplied(const Vector &a, T v)
    {
        return Vector(a[0] * v, a[1] * v, a[2] * v);
    }

    Vector &operator*=(T v)
    {
        this->Multiply(v);
        return *this;
    }

    Vector &operator*=(const Vector &v)
    {
        this->Multiply(v);
        return *this;
    }

    friend Vector Divided(const Vector &a, T v)
    {
        return Vector(a[0] / v, a[1] / v, a[2] / v);
    }

    Vector &operator/=(T v)
    {
        this->Divide(v);
        return *this;
    }
};

template <typename T, std::size_t N>
inline const T *Data(const Vector<T, N> &a)
{
    return a.Data();
}

template <typename T, std::size_t N>
inline std::size_t Size(const Vector<T, N> &)
{
    return N;
}

template <typename T, std::size_t N>
inline T At(const Vector<T, N> &a, std::size_t i)
{
    return a.At(i);
}

template <typename T, std::size_t N>
inline T At(const Vector<T, N> &a, std::size_t i, T fallback)
{
    return a.At(i, fallback);
}

template <typename T, std::size_t N>
inline Vector<T, 1> Extract(
    const Vector<T, N> &a,
    std::size_t d0)
{
    return Vector<T, 1>(a[d0]);
}

template <typename T, std::size_t N>
inline Vector<T, 2> Extract(
    const Vector<T, N> &a,
    std::size_t d0,
    std::size_t d1)
{
    return Vector<T, 2>(a[d0], a[d1]);
}

template <typename T, std::size_t N>
inline Vector<T, 3> Extract(
    const Vector<T, N> &a,
    std::size_t d0,
    std::size_t d1,
    std::size_t d2)
{
    return Vector<T, 3>(a[d0], a[d1], a[d2]);
}

template <typename T, std::size_t N>
inline Vector<T, 4> Extract(
    const Vector<T, N> &a,
    std::size_t d0,
    std::size_t d1,
    std::size_t d2,
    std::size_t d3)
{
    return Vector<T, 4>(a[d0], a[d1], a[d2], a[d3]);
}

template <typename T, std::size_t N>
inline T Dot(const Vector<T, N> &a, const Vector<T, N> &b)
{
    return Vector<T, N>::DotProduct(a, b);
}

template <typename T, std::size_t N>
inline T Length(const Vector<T, N> &a)
{
    return std::sqrt(Dot(a, a));
}

template <typename T, std::size_t N>
inline T Distance(const Vector<T, N> &a, const Vector<T, N> &b)
{
    return Length(Subtracted(a, b));
}

template <typename T, std::size_t N>
inline Vector<T, N> Normalized(Vector<T, N> a)
{
    T l = Length(a);
    if (l != T(0) && l != T(1))
        a = Multiplied(a, T(1) / l);
    return a;
}

template <typename T>
inline Vector<T, 2> Perpendicular(const Vector<T, 2> &a)
{
    return Vector<T, 2>(-a[1], a[0]);
}

template <typename T>
inline Vector<T, 3> Cross(const Vector<T, 3> &a, const Vector<T, 3> &b)
{
    return Vector<T, 3>(
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]);
}

template <typename T, std::size_t N>
inline bool operator==(const Vector<T, N> &a, const Vector<T, N> &b)
{
    return Equal(a, b);
}

template <typename T, std::size_t N>
inline bool operator!=(const Vector<T, N> &a, const Vector<T, N> &b)
{
    return !Equal(a, b);
}

template <typename T, std::size_t N>
inline Vector<T, N> operator+(const Vector<T, N> &v)
{
    return v;
}

template <typename T, std::size_t N>
inline Vector<T, N> operator-(const Vector<T, N> &v)
{
    return Negated(v);
}

template <typename T, std::size_t N>
inline Vector<T, N> operator+(const Vector<T, N> &a, const Vector<T, N> &b)
{
    return Added(a, b);
}

template <typename T, std::size_t N>
inline Vector<T, N> operator-(const Vector<T, N> &a, const Vector<T, N> &b)
{
    return Subtracted(a, b);
}

template <typename T, typename V, std::size_t N>
inline typename std::enable_if<
    std::is_convertible<V, T>::value,
    Vector<T, N>>::type
operator*(const Vector<T, N> &a, V v)
{
    return Multiplied(a, T(v));
}

template <typename T, typename V, std::size_t N>
inline typename std::enable_if<
    std::is_convertible<V, T>::value,
    Vector<T, N>>::type
operator*(V v, const Vector<T, N> &a)
{
    return Multiplied(a, T(v));
}

template <typename T, typename V, std::size_t N>
inline typename std::enable_if<
    std::is_convertible<V, T>::value,
    Vector<T, N>>::type
operator/(const Vector<T, N> &a, V v)
{
    return Divided(a, v);
}

template <typename T, std::size_t N>
Vector<T, N> ToNormalized(math::Vector<T, N> v)
{
    v.Normalize();
    return v;
}

/// 2D float vector
/**
 *  @ingroup math_utils
 */
typedef Vector<float, 2> Vec2f;

/// 3D float vector
/**
 *  @ingroup math_utils
 */
typedef Vector<float, 3> Vec3f;

// /// 4D float vector
// /**
//  *  @ingroup math_utils
//  */
// typedef Vector<float, 4> Vec4f;

/// Template class for quaternions
/**
 *  @ingroup math_utils
 */
template <typename T>
class Quaternion
{
private:
    T _a, _x, _y, _z;

public:
    Quaternion()
        : _a(1), _x(0), _y(0), _z(0)
    {
    }

    /// Constructs a quaternion
    Quaternion(T a, T x, T y, T z)
        : _a(a), _x(x), _y(y), _z(z)
    {
    }

    /// Construct a Quaternion from an @p axis and an @p angle
    Quaternion(Vector<T, 3> axis, Angle<T> angle)
    {
        axis.Normalize();
        angle /= T(2);
        T sx = Sin(angle);
        _a = Cos(angle);
        _x = sx * axis.x();
        _y = sx * axis.y();
        _z = sx * axis.z();
    }

    /// Construct q Quaternion from the @p real and @p imag parts
    Quaternion(T real, const Vector<T, 3> &imag)
        : _a(real), _x(imag.x()), _y(imag.y()), _z(imag.z())
    {
    }

    /// Returns the real scalar part of the quaternion
    T Real(void) const
    {
        return _a;
    }

    /// Returns the imaginary vector part of the quaternion
    Vector<T, 3> Imag(void) const
    {
        return Vector<T, 3>(_x, _y, _z);
    }

    T At(std::size_t index) const
    {
        assert(index < 4);
        if (index == 0)
            return _a;
        if (index == 1)
            return _x;
        if (index == 2)
            return _y;
        if (index == 3)
            return _z;
        return T(0);
    }

    static T DotProduct(const Quaternion &q1, const Quaternion &q2)
    {
        return q1._a * q2._a + q1._x * q2._x + q1._y * q2._y + q1._z * q2._z;
    }

    /// Returns the magnitude of the quaternion
    T Magnitude(void) const
    {
        return std::sqrt(DotProduct(*this, *this));
    }

    /// Returns true if the quaternion has unit Magnitude
    /**
	 *  @see IsNormal
	 *  @see Normalize
	 */
    bool IsUnit(T eps = T(0)) const
    {
        return std::abs(DotProduct(*this, *this) - T(1)) <= eps;
    }

    /// Synonym for IsUnit
    /**
	 *  @see IsUnit
	 *  @see Normalize
	 */
    bool IsNormal(T eps = T(0)) const
    {
        return IsUnit(eps);
    }

    /// Returns true if the quaternion has zero Magnitude
    bool IsDegenerate(T eps = T(0)) const
    {
        return DotProduct(*this, *this) <= eps;
    }

    /// Normalizes this quaternion
    /**
	 *  @pre !this->IsDegenerate();
	 *  @post this->IsUnit()
	 */
    Quaternion &Normalize(void)
    {
        T im = Magnitude();
        assert(im != T(0));
        im = T(1) / im;
        _a *= im;
        _x *= im;
        _y *= im;
        _z *= im;
        return *this;
    }

    /// Equality comparison
    friend bool Equal(const Quaternion &q1, const Quaternion &q2)
    {
        return (q1._a == q2._a) &&
               (q1._x == q2._x) &&
               (q1._y == q2._y) &&
               (q1._z == q2._z);
    }

    /// Near-equality comparison
    friend bool Close(const Quaternion &q1, const Quaternion &q2, T eps)
    {
        for (std::size_t i = 0; i != 4; ++i)
        {
            T u = q1.At(i);
            T v = q2.At(i);
            T d = std::abs(u - v);
            bool ca = d <= std::abs(u) * eps;
            bool cb = d <= std::abs(v) * eps;
            if (!ca && !cb)
                return false;
        }
        return true;
    }

    /// Equality comparison
    friend bool operator==(const Quaternion &q1, const Quaternion &q2)
    {
        return Equal(q1, q2);
    }

    /// Non-equality comparison
    friend bool operator!=(const Quaternion &q1, const Quaternion &q2)
    {
        return !Equal(q1, q2);
    }

    static Quaternion Conjugate(const Quaternion &q1)
    {
        return Quaternion(q1._a, -q1._x, -q1._y, -q1._z);
    }

    /// Conjugation
    friend Quaternion operator~(const Quaternion &q1)
    {
        return Conjugate(q1);
    }

    static Quaternion Inverse(const Quaternion &q1)
    {
        T id = DotProduct(q1, q1);
        assert(id != T(0));
        id *= T(1) / id;
        return Quaternion(
            +q1._a * id,
            -q1._x * id,
            -q1._y * id,
            -q1._z * id);
    }

    static Quaternion Added(const Quaternion &q1, const Quaternion &q2)
    {
        return Quaternion(
            q1._a + q2._a,
            q1._x + q2._x,
            q1._y + q2._y,
            q1._z + q2._z);
    }

    /// Addition operator
    friend Quaternion operator+(const Quaternion &q1, const Quaternion &q2)
    {
        return Added(q1, q2);
    }

    static Quaternion Multiplied(const Quaternion &q1, const Quaternion &q2)
    {
        return Quaternion(
            q1._a * q2._a - q1._x * q2._x - q1._y * q2._y - q1._z * q2._z,
            q1._a * q2._x + q1._x * q2._a + q1._y * q2._z - q1._z * q2._y,
            q1._a * q2._y - q1._x * q2._z + q1._y * q2._a + q1._z * q2._x,
            q1._a * q2._z + q1._x * q2._y - q1._y * q2._x + q1._z * q2._a);
    }

    /// Multiplication operator
    friend Quaternion operator*(const Quaternion &q1, const Quaternion &q2)
    {
        return Multiplied(q1, q2);
    }

    static Quaternion Multiplied(const Quaternion &q1, T t)
    {
        return Quaternion(q1._a * t, q1._x * t, q1._y * t, q1._z * t);
    }

    /// Multiplication by scalar operator
    friend Quaternion operator*(const Quaternion &q1, T t)
    {
        return Multiplied(q1, t);
    }

    /// Multiplication by scalar operator
    friend Quaternion operator*(T t, const Quaternion &q1)
    {
        return Multiplied(q1, t);
    }

    static Vector<T, 3> RotateVector(
        const Quaternion &q,
        const Vector<T, 3> &v)
    {
        return (q * Quaternion(T(0), v) * Conjugate(q)).Imag();
    }
};

template <typename T>
inline T Dot(const Quaternion<T> &q1, const Quaternion<T> &q2)
{
    return Quaternion<T>::DotProduct(q1, q2);
}

template <typename T>
inline T Magnitude(const Quaternion<T> &q1)
{
    return q1.Magnitude();
}

template <typename T>
inline Quaternion<T> Conjugate(const Quaternion<T> &q1)
{
    return Quaternion<T>::Conjugate(q1);
}

template <typename T>
inline Quaternion<T> Inverse(const Quaternion<T> &q1)
{
    return Quaternion<T>::Inverse(q1);
}

template <typename T>
inline Quaternion<T> Add(const Quaternion<T> &q1, const Quaternion<T> &q2)
{
    return Quaternion<T>::Added(q1, q2);
}

template <typename T>
inline Quaternion<T> Multiply(const Quaternion<T> &q1, const Quaternion<T> &q2)
{
    return Quaternion<T>::Multiplied(q1, q2);
}

template <typename T>
inline Vector<T, 3> Rotate(const Quaternion<T> &q, const Vector<T, 3> &v)
{
    return Quaternion<T>::RotateVector(q, v);
}

// dual quaternion
template <class T>
class DQuaternion
{
public:
    using Quaternion = Quaternion<T>;
    using Vector = Vector<T, 3>;

    DQuaternion()
    {
    }

    explicit DQuaternion(const Quaternion real, const Quaternion dual)
        : _real{real}, _dual{dual}
    {
        _real.Normalize();
    }

    DQuaternion(const Vector translation)
        : _real{1, 0, 0, 0}, _dual{0, translation[0] * 0.5f, translation[1] * 0.5f, translation[2] * 0.5f}
    {
    }

    DQuaternion(const Quaternion rotation, const Vector translation)
        : _real{rotation}
    {
        _real.Normalize();
        _dual = Quaternion{0, translation[0], translation[1], translation[2]} * _real * 0.5f;
    }

    explicit DQuaternion(const T ra, const T rx, const T ry, const T rz,
                         const T da, const T dx, const T dy, const T dz)
        : _real{ra, rx, ry, rz}, _dual{da, dx, dy, dz}
    {
    }

    void Rotation(Quaternion rotation)
    {
        auto trans = Translation();
        _real = rotation;
        _real.Normalize();
        Translation(trans);
    }

    void Translation(const Vector translation)
    {
        _dual = math::Conjugate(_real) * Quaternion{0, translation[0] * 0.5f, translation[1] * 0.5f, translation[2] * 0.5f};
    }

    void TranslateLocal(const Vector translation)
    {
        _dual = _dual + (_real * Quaternion{0, translation[0] * 0.5f, translation[1] * 0.5f, translation[2] * 0.5f});
    }

    DQuaternion operator+(const DQuaternion &rhs) const
    {
        return {_real + rhs._real, _dual + rhs._dual};
    }

    DQuaternion operator*(const T scale) const
    {
        return {_real * scale, _dual * scale};
    }

    // multiplication order - left to right
    DQuaternion operator*(const DQuaternion &rhs) const
    {
        return DQuaternion{
            rhs._real * _real,
            (rhs._real * _dual) + (rhs._dual * _real)};
    }

    DQuaternion Conjugate() const
    {
        // type 3 conjugation
        return DQuaternion{
            _real.At(0), -_real.At(1), -_real.At(2), -_real.At(3),
            -_dual.At(0), _dual.At(1), _dual.At(2), _dual.At(3)};
    }

    Quaternion Rotation() const
    {
        return _real;
    }

    Vector Translation() const
    {
        auto t = _dual * math::Conjugate(_real) * 2.0f;
        return t.Imag();
    }

    DQuaternion &Normalize()
    {
        const T mag = math::Dot(_real, _real);
        _real *= 1.0 / mag;
        _dual *= 1.0 / mag;
        return *this;
    }

    const Quaternion &Real() const
    {
        return _real;
    }
    const Quaternion &Dual() const
    {
        return _dual;
    }

private:
    Quaternion _real{1, 0, 0, 0};
    Quaternion _dual{0, 0, 0, 0};
};

template <class T>
inline math::Vector<T, 3> operator*(const DQuaternion<T> &dquat, const math::Vector<T, 3> &vec)
{
    return math::Rotate(dquat.Real(), vec) + dquat.Translation();
}

template <class T>
inline math::Quaternion<T> AbsoluteAngle(const math::Vector<T, 3> &normalizedPath, const math::Vector<T, 3> reference = {0, 0, 1})
{
    auto &u = reference;
    auto &v = normalizedPath;

    // taken from: http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    auto w = Cross(u, v);
    auto dot = Dot(u, v);

    if (dot < -0.999999f)
    {
        // opposite vectors. For now only rotate around up axis (y).
        return math::Quaternion<T>{static_cast<T>(0), static_cast<T>(0), static_cast<T>(1), static_cast<T>(0)};
        /*
		w = rsmotion::Vec3f{ 0.f, 0.f, 1.f }; / std::sqrt(u[1] * u[1] + u[0] * u[0]);
		w = std::abs(u[0]) > std::abs(u[2])
			? rsmotion::Vec3f{ -u[1], u[0], 0.f } / std::sqrt(u[1] * u[1] + u[0] * u[0])
			: rsmotion::Vec3f{ 0, -u[2], u[1] } / sqrt(u[1] * u[1] + u[2] * u[2]);
		return math::Quaternion<T>{ static_cast<T>(0), w[0], w[1], w[2] };*/
    }

    auto q = math::Quaternion<T>{static_cast<T>(1) + dot, w[0], w[1], w[2]};
    q.Normalize();
    return q;
}

/// Base template for spherical-linear interpolation functors
/** BaseSLERP implements the basic sperical-linear implementation
 *  for Quaternion or Vector pairs.
 *
 *  @ingroup math_utils
 */
template <typename Value, typename T>
class BaseSLERP
{
private:
    Value _v1, _v2;
    Angle<T> _omega;
    T _inv_sin_omega;

    Value _first(T) const
    {
        return _v1;
    }

    Value _lerp(T t) const
    {
        return _v1 * (T(1) - t) + _v2 * t;
    }

    Value _slerp(T t) const
    {
        return _v1 * Sin((T(1) - t) * _omega) * _inv_sin_omega +
               _v2 * Sin(t * _omega) * _inv_sin_omega;
    }

    Value (BaseSLERP::*_func)(T) const;

public:
    /// Constructs a SLERP functor from two values
    /**
	 *  @pre v1.IsNormal() && v2.IsNormal()
	 */
    BaseSLERP(
        const Value &v1,
        const Value &v2,
        T eps) : _v1(v1), _v2(v2)
                 //, _omega(Angle<T>::ArcCos(Dot(_v1, _v2)))
                 // 2016-mar-21 Added by Bas G because the dot product sometimes returns >1.f for equal normalized quaternions
                 ,
                 _omega(Angle<T>::ArcCos(std::min(1.f, std::max(-1.f, Dot(_v1, _v2))))), _inv_sin_omega(Sin(_omega)), _func(nullptr)
    {
        if (_inv_sin_omega == T(0))
        {
            _func = &BaseSLERP::_first;
        }
        else if (std::abs(_inv_sin_omega) < eps)
        {
            _func = &BaseSLERP::_lerp;
        }
        else
        {
            _func = &BaseSLERP::_slerp;
            _inv_sin_omega = T(1) / _inv_sin_omega;
        }
        assert(_func != nullptr);
    }

    /// Interpolates between the values passed to constructor
    /**
	 *  @pre (param >= 0) && (param <= 1)
	 */
    Value operator()(T param) const
    {
        assert(_func != nullptr);
        return (this->*_func)(param);
    }
};

/// Functor template for quaternion spherical-linear interpolation
/** QuaternionSLERP can be used to smoothly interpolate between two
 *  unit quaternions.
 *
 *  @ingroup math_utils
 */
template <typename T>
class QuaternionSLERP
    : public BaseSLERP<Quaternion<T>, T>
{
public:
    /// Constructs a SLERP functor from two unit quaternions
    /**
	 *  @pre q1.IsNormal() && q2.IsNormal()
	 */
    QuaternionSLERP(
        const Quaternion<T> &q1,
        const Quaternion<T> &q2,
        T eps = 0.001) : BaseSLERP<Quaternion<T>, T>(q1, q2, eps)
    {
    }
};

template <class T>
inline Vector<T, 3> operator*(const Quaternion<T> &quat, const Vector<T, 3> &vec)
{
    return Rotate(quat, vec);
}

/// Shorthand aliases
template <class T>
using Vec3 = math::Vector<T, 3>;

using Anglef = math::Angle<float>;

using Quatf = math::Quaternion<float>;

using DQuat = math::DQuaternion<float>;

} // namespace math

} // namespace rsmotion

namespace std
{

template <class T>
inline std::string to_string(const rsmotion::math::Vector<T, 2> &vector)
{
    return {"(" + std::to_string(vector.x()) + "," + std::to_string(vector.y()) + ")"};
}

template <class T>
inline std::string to_string(const rsmotion::math::Vector<T, 3> &vector)
{
    return {"(" + std::to_string(vector.x()) + "," + std::to_string(vector.y()) + "," + std::to_string(vector.z()) + ")"};
}

template <class T, int N>
struct hash<rsmotion::math::Vector<T, N>>
{
    std::size_t operator()(const rsmotion::math::Vector<T, N> &s) const
    {
        std::size_t seed = 0;
        for (int i = 0; i < N; ++i)
            combine_hash(seed, s[0]);
        return seed;
    }
};

template <class T>
inline std::string to_string(const rsmotion::math::Quaternion<T> &x)
{
    return "(" + std::to_string(x.Real()) + "|" + std::to_string(x.Imag().x()) + "," + std::to_string(x.Imag().y()) + "," + std::to_string(x.Imag().z()) + ")";
}

template <class T>
inline std::ostream &operator<<(std::ostream &os, const rsmotion::math::Quaternion<T> &x)
{
    return os << std::to_string(x);
}

template <class T>
inline std::string to_string(const rsmotion::math::DQuaternion<T> &x)
{
    return "[" + std::to_string(x.Real()) + "," + std::to_string(x.Dual()) + "]";
}

template <class T>
inline std::ostream &operator<<(std::ostream &os, const rsmotion::math::DQuaternion<T> &x)
{
    return os << std::to_string(x);
}

} // namespace std