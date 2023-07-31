#pragma once

#include "Quaternion.h"
#include <limits>
#include <stdexcept>

namespace vanvitelli
{

    // The UnitQuaternion class
    template <typename Scalar>
    class UnitQuaternion : public Quaternion<Scalar>
    {
    protected:
    public:

        // The default constructor builds an Identity Quaternion
        UnitQuaternion()
            : Quaternion<Scalar>(Scalar(0), Scalar(0), Scalar(0), Scalar(1))
        {
        }

        UnitQuaternion(const Scalar &x, const Scalar &y, const Scalar &z, const Scalar &w)
            : Quaternion<Scalar>(x, y, z, w)
        {
            /* 
                we have to be sure that the UnitQuaternion is unit!
                One approach is to normalize the quaternion in the constructor
                Here we choose to check if the quaternion is unit, if not we raise an exception.
            */
            checkNormalized();
        }

        UnitQuaternion(const Vector3<Scalar> &v, const Scalar &w)
            : Quaternion<Scalar>(v, w)
        {
            checkNormalized();
        }

        UnitQuaternion(const Scalar &w, const Vector3<Scalar> &v)
            : UnitQuaternion(v, w)
        {
        }

        void checkNormalized() const
        {
            /*
                std::numeric_limits<T>::epsilon() returns the machine eps for the type T
                it is similar to the MATLAB eps. Please see the official documentation
            */

            /* 
                note that we use Quaternion<Scalar>::squaredNorm() to call the base class method
                instead of the squaredNorm defined in the UnitQuaternion class 
                (it is similar to super.squaredNorm() in java)
            */
            Scalar tmp = Quaternion<Scalar>::squaredNorm() - Scalar(1.0);
            tmp = tmp * tmp;
            if (tmp > std::numeric_limits<Scalar>::epsilon())
            {
                throw std::runtime_error("UnitQuaternion not unit!, the squared norm is: " + std::to_string(Quaternion<Scalar>::squaredNorm()));
            }
        }

        virtual ~UnitQuaternion() = default;

        // The setters have to check the quaternion norm!!!
        virtual void setV(const Vector3<Scalar> &v) override
        {
            // NOTE: to access the base class protected attribute v_ we have to use this->v_
            // (v_ without this will cause a compilation error)
            this->v_ = v;
            checkNormalized(); // raise an exception if the quaternion is not unit after the set
        }

        virtual void setW(const Scalar &w) override
        {
            this->w_ = w;
            checkNormalized();
        }

        /*
            This is to force the normalization of the UnitQuaternion
            in the current class implementation, the UnitQuaternion could not be unit
            if we use the setters in a try/catch block (see the testQuaternion.cpp file)
        */
        virtual void forceNormalize()
        {
            // question: why we can't use Quaternion<Scalar>::norm() here?
            Scalar norm_ = std::sqrt(Quaternion<Scalar>::squaredNorm());
            std::cout << "forceNormalize - the norm is:" << norm_ << "\n";
            this->v_ = this->v_ / norm_;
            this->w_ = this->w_ / norm_;
        }

        // we don't have to compute the norm for this class
        virtual Scalar squaredNorm() const override
        {
            return Scalar(1);
        }

        virtual Scalar norm() const override
        {
            return Scalar(1);
        }

        // For this class, unit() can just call the copy constructor
        virtual UnitQuaternion<Scalar> unit() const override
        {
            return UnitQuaternion<Scalar>(*this);
        }

        // this is not an override! this is an overload!
        virtual UnitQuaternion<Scalar> prod(const UnitQuaternion<Scalar> &q2) const
        {
            // call the base class prod(Quaternion) then convert the resulting Quaternion
            // into a UnitQuaternion
            return Quaternion<Scalar>::prod(q2).unit();
        }

        template<typename T>
        void euler_xyz(const T* euler) {
            double c1 = cos(euler[0] * 0.5);
            double c2 = cos(euler[1] * 0.5);
            double c3 = cos(euler[2] * 0.5);
            double s1 = sin(euler[0] * 0.5);
            double s2 = sin(euler[1] * 0.5);
            double s3 = sin(euler[2] * 0.5);

            this->v_.x() = s1*c2*c3 + c1*s2*s3;
            this->v_.y() = -s1*c2*s3 + c1*s2*c3;
            this->v_.z() = c1*c2*s3 + s1*s2*c3;
            this->w_ = c1*c2*c3 - s1*s2*s3;
        }

        virtual void print() override
        {
            std::cout << "UnitQuaternion: << " << Quaternion<Scalar>::v_.x() << " , " << Quaternion<Scalar>::v_.y() << " , " << Quaternion<Scalar>::v_.z() << " >> , " << Quaternion<Scalar>::w_ << "\n";
        }
    };

    // also these functions (external to the class) are overloads

    template <typename Scalar>
    UnitQuaternion<Scalar> conj(const UnitQuaternion<Scalar> &q)
    {
        return UnitQuaternion<Scalar>(-q.v(), q.w());
    }

    template <typename Scalar>
    UnitQuaternion<Scalar>
    inv(const UnitQuaternion<Scalar> &q)
    {
        return conj(q);
    }

}