#include "world.h"
#include "utils.h"

namespace xrobot {

using namespace render_engine;
using namespace bullet_engine;

Object::Object() : RenderPart(),
                   BulletObject(),
                   bullet_world_(),
                   body_uid_(-1) {}

void Object::Sleep() {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::sleep(world->client_, id());
}

void Object::EnableSleeping() {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::enable_sleeping(world->client_, id());
}

void Object::DisableSleeping() {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::disable_sleeping(world->client_, id());
}

void Object::Wake() {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::wake(world->client_, id());
}

void Object::GetMass(xScalar& mass) {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::get_mass(world->client_, id(), mass);
}

void Object::SetMass(const xScalar mass) {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_mass(world->client_, id(), mass);
}

void Object::SetStatic() {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::get_mass(world->client_, id(), object_mass_original_);
    BulletObject::change_mass(world->client_, id(), 0.0f);
}

void Object::RecoverFromStatic() {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_mass(
            world->client_, id(), object_mass_original_);
}

void Object::GetAABB(glm::vec3& aabb_min, glm::vec3& aabb_max) {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::get_AABB(world->client_, id(), aabb_min, aabb_max);
}

void Object::ChangeLinearDamping(const xScalar damping) {
    assert(damping >= 0);
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_linear_damping(world->client_, id(), damping);
}

void Object::ChangeAngularDamping(const xScalar damping) {
    assert(damping >= 0);
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_angular_damping(world->client_, id(), damping);
}

void Object::ChangeLateralFriction(const xScalar friction) {
    assert(friction >= 0);
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_lateral_friction(world->client_, id(), friction);
}

void Object::ChangeSpinningFriction(const xScalar friction) {
    assert(friction >= 0);
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_spinning_friction(world->client_, id(), friction);
}

void Object::ChangeRollingFriction(xScalar friction) {
    assert(friction >= 0);
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::change_rolling_friction(world->client_, id(), friction);
}

void Object::ApplyForce(
        const xScalar x, const xScalar y, const xScalar z, const int flags) {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::apply_force(world->client_, id(), x, y, z, flags);
}

void Object::ApplyTorque(
        const xScalar x, const xScalar y, const xScalar z, const int flags) {
    auto world = wptr_to_sptr(bullet_world_);
    BulletObject::apply_torque(world->client_, id(), x, y, z, flags);
}

glm::mat4 Object::translation_matrix() const {
    return BulletObject::translation_matrix();
}

glm::mat4 Object::local_inertial_frame() const {
    return BulletObject::local_inertial_frame();
}

}
