#include "world.h"
#include "utils.h"

namespace xrobot {

using namespace render_engine;
using namespace bullet_engine;

Joint::Joint() : BulletJoint(),
                 bullet_world_(),
                 bullet_robot_() {}

Joint::~Joint() {}

void Joint::EnableJointSensor(const bool enable) {
    auto robot = wptr_to_sptr(bullet_robot_);
    auto world = wptr_to_sptr(bullet_world_);
    enable_sensor(world->client_, robot->id(), enable);
}

void Joint::GetJointMotorState(glm::vec3& force, glm::vec3& torque) {
    auto robot = wptr_to_sptr(bullet_robot_);
    auto world = wptr_to_sptr(bullet_world_);
    get_motor_state(world->client_, robot->id(), force, torque);
}

void Joint::ResetJointState(const xScalar pos, const xScalar vel) {
    auto robot = wptr_to_sptr(bullet_robot_);
    auto world = wptr_to_sptr(bullet_world_);
    reset_state(world->client_, robot->id(), pos, vel);
}

void Joint::SetJointMotorControlTorque(const xScalar torque) {
    auto robot = wptr_to_sptr(bullet_robot_);
    auto world = wptr_to_sptr(bullet_world_);
    set_motor_control_torque(world->client_, robot->id(), torque);
}

void Joint::SetJointMotorControlVelocity(const xScalar speed,
                                         const xScalar k_d,
                                         const xScalar max_force) {
    
    auto robot = wptr_to_sptr(bullet_robot_);
    auto world = wptr_to_sptr(bullet_world_);
    set_motor_control_velocity(
            world->client_, robot->id(), speed, k_d, max_force);
}

void Joint::SetJointMotorControlPosition(
        const xScalar target,
        const xScalar k_p,
        const xScalar k_d,
        const xScalar max_force) {
    auto robot = wptr_to_sptr(bullet_robot_);
    auto world = wptr_to_sptr(bullet_world_);
    set_motor_control_position(
            world->client_, robot->id(), target, k_p, k_d, max_force);
}

}
