// Header Guard
#pragma once

#include <cmath>

// A namespace for my lib
namespace vanvitelli
{

    /*
        THe Vector3 class represents a generic vector in R^3
        The template typename Scalar holds the scalar type (int, float, double, etc...)
    */
    template <typename Scalar>
    class Vector3
    {
    private:
        // The 3 components
        Scalar x_, y_, z_;

    public:
        // A constructor
        Vector3(const Scalar &x, const Scalar &y, const Scalar &z)
            : x_(x), y_(y), z_(z)
        {
        }

        // A default copy constructor
        Vector3(const Vector3<Scalar> &v) = default;

        // A default destructor
        ~Vector3() = default;

        // A getter, note that we return a const reference
        const Scalar &x() const
        {
            return x_;
        }

        const Scalar &y() const
        {
            return y_;
        }

        const Scalar &z() const
        {
            return z_;
        }

        /*
            This is both a getter and a setter
            the return type is a non-const reference
            v.x() can be used as lvalue
            v.x() = 3;
        */
        Scalar &x()
        {
            return x_;
        }

        Scalar &y()
        {
            return y_;
        }

        Scalar &z()
        {
            return z_;
        }

        Scalar squaredNorm() const
        {
            return this->scalarProd(*this);
        }

        Scalar norm() const
        {
            return std::sqrt(squaredNorm());
        }

        Scalar scalarProd(const Vector3<Scalar> &v2) const
        {
            return x_ * v2.x_ + y_ * v2.y_ + z_ * v2.z_;
        }

        Vector3<Scalar> cross(const Vector3<Scalar> &v2) const
        {
            return Vector3<Scalar>(y_ * v2.z_ - z_ * v2.y_, z_ * v2.x_ - x_ * v2.z_, x_ * v2.y_ - y_ * v2.x_);
        }

        // operators:

        // to enable
        // v2 = -v1;
        Vector3<Scalar> operator-() const
        {
            return Vector3<Scalar>(-x_, -y_, -z_);
        }

        // v3 = v1 + v2;
        Vector3<Scalar> operator+(const Vector3<Scalar> &v2) const
        {
            return Vector3<Scalar>(x_ + v2.x_, y_ + v2.y_, z_ + v2.z_);
        }

        // v2 = v1 / a; where a is a scalar
        Vector3<Scalar> operator/(const Scalar &a) const
        {
            return Vector3<Scalar>(x_ / a, y_ / a, z_ / a);
        }

        // v2 = v1 * a; where a is a scalar
        Vector3<Scalar> operator*(const Scalar &a) const
        {
            return Vector3<Scalar>(x_ * a, y_ * a, z_ * a);
        }

        // v2 = a * v; where a is a scalar
        friend Vector3<Scalar> operator*(const Scalar &a, const Vector3<Scalar> &v)
        {
            return v * a;
        }
    };

}