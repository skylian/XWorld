#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include "bullet_engine/common.h"

namespace xrobot {

// TODO: Right now xScalar cannot be float
#ifdef USE_DOUBLE_PRECISION
typedef double xScalar;
#else
typedef float xScalar;
#endif

class xMatrix3x3;
class xTransform;

class xVector3 {
public:
    xVector3() : v_(btVector3(0, 0, 0)) {}

    xVector3(const xScalar x, const xScalar y, const xScalkar z) :
            v_(btVector3(x,y,z,)) {}

    xVector3(const xVector3& rhs) : v_(rhs.v_) {}

    xVector3(const btVector3& rhs) : v_(rhs) {}

    xVector3(const glm::vec3& rhs) : v_(btVector3(rhs[0], rhs[1], rhs[2])) {}

    xVector3& operator=(const xVector3 rhs) {
        v_ = rhs.v_;
        return *this;
    }

    xScalar operator[](const size_t i) const {
        assert(i >= 0 && i <= 2);
        return v_[i]; 
    }

    glm::vec3 glm_vec() const {
        return glm::vec3(v_[0], v_[1], v_[2]);
    }

    friend xVector3 operator+(const xVector3& a, const xVector3& b);

    friend glm::vec3 operator+(const xVector3& a, const glm::vec3& b);

    friend xVector3 operator*(const xMatrix3x3& m, const xVector3& v);

private:
    btVector3 v_;

    friend class xTransform;
};

xVector3 operator+(const xVector3& a, const xVector3& b) {
    return xVector3(a.v_ + b.v_);
}

glm::vec3 operator+(const xVector3& a, const glm::vec3& b) {
    return a.glm() + b;
}

class xMatrix3x3 {
public:
    xMatrix3x3() : m_() {}
    
    xMatrix3x3(const xMatrix3x3& rhs) : m_(rhs.m_) {}

    xMatrix3x3(const btMatrix3x3& m) : m_(m) {}

    void setEulerYPR(const xScalar y, const xScalar p, const xScalar r) {
        m_.setEulerYPR(y, p, r);
    }

    friend xMatrix3x3 operator+(const xMatrix3x3& a, const xMatrix3x3& b);

    friend xMatrix3x3 operator*(const xMatrix3x3& a, const xMatrix3x3& b);

    friend xVector3 operator*(const xMatrix3x3& m, const xVector3& v);

    friend glm::vec3 operator*(const xMatrix3x3& m, const glm::vec3& v);

private:
    btMatrix3x3 m_;

    friend class xTransform;
};

xMatrix3x3 operator*(const xMatrix3x3& a, const xMatrix3x3& b) {
    return xMatrix3x3(a.m_ * b.m_);
}

xMatrix3x3 operator+(const xMatrix3x3& a, const xMatrix3x3& b) {
    return xMatrix3x3(a.m_ + b.m_);
}

xVector3 operator*(const xMatrix3x3& m, const xVector3& v) {
    return xVector3(m.m_ * v.v_);
}

glm::vec3 operator*(const xMatrix3x3& m, const glm::vec3& v) {
    auto r = m * xVector3(v);
    return glm::vec3(r[0], r[1], r[2]);
}

class xQuaternion {
public:
    xQuaternion(const xScalar r, const xScalar p, const xScalar y) :
            q_(r, p, y) {}
    
    xQuaternion() : xQuaternion(0, 0, 0) {}

    xQuaternion(const xVector3& axis_size, const xScalar angle) {
        q_ = btQuaternion(btVector3(axis[0], axis[1], axis[2]), angle);
    }

    xQuaternion(const xQuaternion& rhs) : q_(rhs.q_) {}

    xQuaternion(const btQuaternion& rhs) : q_(rhs) {}

    xQuaternion& operator=(const xQuaternion& rhs) {
        q_ = rhs.q_;
        return *this;
    }

    xScalar operator[](const size_t i) const {
        assert(i >= 0 && i <= 3);
        return q_[i]; 
    }

    xQuaternion& operator*=(const xQuaternion& o) {
        q_ = q_ * o.q_;
        return *this;
    }

    friend xQuaternion operator*(const xQuaternion& a, const xQuaternion& b);

    friend xQuaternion operator*(const xTransform& t, const xQuaternion& q);

private:
    btQuaternion q_;

    friend class xTransform;
};

friend xQuaternion operator*(const xQuaternion& a, const xQuaternion& b) {
    return xQuaternion(a.q_ * b.q_);
}

class xTransform {
public:
    xTransform() : t_() {}

    xTransform(const xTransform& rhs) {
        t_ = rhs.t_;
    }

    xTransform(const btTransform& rhs) {
        t_ = rhs;
    }

    xTransform& operator=(const xTransform& rhs) {
        t_ = rhs.t_;
    } 

    void setIdentity() { t_.setIdentity(); }

    xVector3& getOrigin() { return xVector3(t_.getOrigin()); }

    const xVector3& getOrigin() const { return xVector3(t_.getOrigin()); }

    void setOrigin(const xVector3& o) {
        t_.setOrigin(o.v_);
    }

    xMatrix3x3& getBasis() { return t_.getBasis(); }

    const xMatrix3x3& getBasis() const { return t_.getBasis(); }

    xQuaternion& getRotation() { return xQuaternion(t_.getRotation()); }

    const xQuaternion& getRotation() const {
        return xQuaternion(t_.getRotation());
    }

    void setRotation(const xQuaternion& q) {
        t_.setRotation(q.q_);
    }

    btTransform inverse() const {
        return xTransform(t_.inverse());
    }

    glm::mat4 glmMat4() const {
        btScalar M[16];
        t_.getOpenGLMatrix(M);
        return glm::make_mat4(M);
    }

    friend xVector3 operator*(const xTransform& t, const xVector3& v);

private:
    btTransform t_;
};

xVector3 operator*(const xTransform& t, const xVector3& v) {
    return xVector3(t.t_ * v.v_);
};

} // namespace xrobot
#endif
