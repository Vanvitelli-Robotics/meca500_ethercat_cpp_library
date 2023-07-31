#pragma once

#include "Vector3.h"
#include <iostream>

namespace vanvitelli
{

    // Forward declaration, it is needed in the unit() method
    template <typename Scalar>
    class UnitQuaternion;

    // The Quaternion class
    template <typename Scalar>
    class Quaternion
    {
    protected:
        // vector part
        Vector3<Scalar> v_;
        // scalar part
        Scalar w_;

    public:
        // Full constructor
        // note that we use the initializer list to construct v_
        Quaternion(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &w) : v_(x, y, z),
                                                                                         w_(w)
        {
        }

        // Constructors...

        Quaternion(const Vector3<Scalar> &v, const Scalar &w)
            : v_(v),
              w_(w)
        {
        }

        Quaternion(const Scalar &w, const Vector3<Scalar> &v)
            : Quaternion(v, w)
        {
        }

        // getters
        const Vector3<Scalar> &v() const
        {
            return v_;
        }


        const Scalar &w() const
        {
            return w_;
        }

        // setters. note that they are virtual
        // we need to modify the setters in the UnitQuaternion class
        virtual void setV(const Vector3<Scalar> &v)
        {
            v_ = v;
        }

        virtual void setW(const Scalar &w)
        {
            w_ = w;
        }

        // a destructor, note "virtual"
        virtual ~Quaternion() = default;

        virtual Scalar squaredNorm() const
        {
            return v_.squaredNorm() + w_ * w_;
        }

        virtual Scalar norm() const
        {
            return std::sqrt(squaredNorm());
        }

        // here we need the forward declaration
        // this method normalizes the quaternion and returns it as a UnitQuaternion
        virtual UnitQuaternion<Scalar> unit() const
        {
            Scalar norm_ = norm();
            return UnitQuaternion<Scalar>(v_ / norm_, w_ / norm_);
        }

        // this method normalizes *this* quaternion
        // the quaternion will not be an UnitQuaternion!!
        virtual void normalize()
        {
            Scalar norm_ = norm();
            v_ = v_ / norm_;
            w_ = w_ / norm_;
        }

        virtual Quaternion<Scalar> prod(const Quaternion<Scalar> &q2) const
        {
            Scalar w_prod = w_ * q2.w_ - v_.scalarProd(q2.v_);
            Vector3<Scalar> v_prod = w_ * q2.v_ + q2.w_ * v_ + v_.cross(q2.v_);
            return Quaternion<Scalar>(v_prod, w_prod);
        }

        virtual void print()
        {
            std::cout << "Quaternion: < " << v_.x() << " , " << v_.y() << " , " << v_.z() << " > , " << w_ << "\n";
        }
    };

    // conj e inv are separate functions (this is not mandatory)

    template <typename Scalar>
    Quaternion<Scalar> conj(const Quaternion<Scalar> &q)
    {
        return Quaternion<Scalar>(-q.v(), q.w());
    }

    template <typename Scalar>
    Quaternion<Scalar>
    inv(const Quaternion<Scalar> &q)
    {
        Scalar norm2_ = q.squaredNorm();
        return Quaternion<Scalar>(-q.v() / norm2_, q.w() / norm2_);
    }

}