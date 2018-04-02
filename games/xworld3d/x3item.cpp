// Copyright (c) 2017 Baidu Inc. All Rights Reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "x3item.h"

namespace simulator {
namespace xworld3d {

using simulator::util::path_join;

const x3real X3Item::UNIT = FLAGS_x3_unit;
const x3real X3Item::UNIT_INV = 1.0 / FLAGS_x3_unit;

const x3real REACH_HEIGHT_THRESHOLD = X3Item::UNIT;
const x3real CAMERA_BIRD_VIEW_HEIGHT = 10.0 * X3Item::UNIT;

X3ItemPtr X3Item::create_item(const Entity& e, World& world) {
    float scale = 1.0;
    if (e.type == "agent") {
        scale = 2;
    }
    if (e.model_type == "mjcf") {
        return std::make_shared<MuJoCoItem>(e, scale, world, true);
    } else if (e.model_type == "sdf") {
        return std::make_shared<MuJoCoItem>(e, scale, world, false);
    } else {
        return std::make_shared<URDFItem>(e, scale, world);
    }
}

X3Item::X3Item(const Entity& e, World& world) :
        e_(e),
        move_speed_norm_(FLAGS_x3_move_speed * UNIT),
        jump_speed_norm_(FLAGS_x3_jump_speed * UNIT),
        reaching_dist_(FLAGS_x3_reaching_distance * UNIT) {
}

void X3Item::destroy() {
    for (auto& p : parts_) {
        p.destroy();
    }
}

X3Item::Pose X3Item::pose() const {
    return root_part_ptr_->pose();
}

void X3Item::get_direction(x3real &dir_x, x3real &dir_y) const {
    x3real yaw = std::get<2>(root_part_ptr_->pose().rpy());
    dir_x = cos(yaw);
    dir_y = sin(yaw);
}

void X3Item::sync_entity_info() {
    Pose pose = root_part_ptr_->pose();
    std::tie(e_.loc.x, e_.loc.y, e_.loc.z) = pose.xyz();
    e_.loc.scale(UNIT_INV);
    e_.yaw = std::get<2>(pose.rpy());
}

void X3Item::move_underground() {
   Pose pose(root_part_ptr_->pose());
   pose.set_xyz(pose.x(), pose.y(), -2);
   root_part_ptr_->set_pose_and_speed(pose, 0.0f, 0.0f, 0.0f);
}

void X3Item::set_speed(x3real vx, x3real vy, x3real vz) {
    root_part_ptr_->set_speed(vx, vy, vz);
}

void X3Item::set_pose(const Pose& pose) {
    root_part_ptr_->set_pose(pose);
}

void X3Item::set_pose_and_speed(const Pose& pose,
                                x3real vx, x3real vy, x3real vz) {
    root_part_ptr_->set_pose_and_speed(pose, vx, vy, vz);
}

URDFItem::URDFItem(const Entity& e, x3real scale, World& world) :
        X3Item(e, world) {
    Pose pose(e_.loc.x * UNIT, e_.loc.y * UNIT, e_.loc.z * UNIT);
    pose.rotate_z(e_.yaw);
    parts_.push_back(world.load_urdf(e.asset_path, pose, scale, false, false));
    root_part_ptr_ = &(parts_[0]);
}

void URDFItem::set_entity(const Entity& e) {
   e_ = e;
   Pose pose(e_.loc.x * UNIT, e_.loc.y * UNIT, e_.loc.z * UNIT);
   pose.rotate_z(e_.yaw);
   root_part_ptr_->set_pose_and_speed(pose, 0.0f, 0.0f, 0.0f);
}

X3ItemPtr URDFItem::collect_item(const std::map<std::string, X3ItemPtr>& items,
                                 const std::string& type) {
    X3ItemPtr item = nullptr;
    root_part_ptr_->set_speed(0.0f, 0.0f, 0.0f);
    // the angle between the agent's facing direction and
    // the direction from the agent to the item should be less than 45 degrees
    x3real best_score = 0.707; // 45 degrees is the minimum
    x3real score;
    for (auto& kv : items) {
        if (kv.second->type() == type) {
            score = reach_test(kv.second->pose());
            if (score > best_score) {
                item = kv.second;
                best_score = score;
            }
        }
    }
    return item;
}

void URDFItem::move_forward() {
    Pose pose(root_part_ptr_->pose());
    x3real yaw = std::get<2>(pose.rpy());
    x3real vx = move_speed_norm_ * cos(yaw);
    x3real vy = move_speed_norm_ * sin(yaw);
    x3real vz = root_part_ptr_->speed_z();
    root_part_ptr_->set_speed(vx, vy, 0.0f);
}

void URDFItem::move_backward() {
    Pose pose(root_part_ptr_->pose());
    x3real yaw = std::get<2>(pose.rpy());
    pose.set_xyz(pose.x(), pose.y(), pose.z());
    x3real vx = -move_speed_norm_ * cos(yaw);
    x3real vy = -move_speed_norm_ * sin(yaw);
    x3real vz = root_part_ptr_->speed_z();
    root_part_ptr_->set_speed(vx, vy, 0.0f);
}

void URDFItem::move_left() {
    Pose pose(root_part_ptr_->pose());
    x3real yaw = std::get<2>(pose.rpy());
    pose.set_xyz(pose.x(), pose.y(), pose.z());
    x3real vx = -move_speed_norm_ * sin(yaw);
    x3real vy = move_speed_norm_ * cos(yaw);
    x3real vz = root_part_ptr_->speed_z();
    root_part_ptr_->set_speed(vx, vy, 0.0f);
}

void URDFItem::move_right() {
    Pose pose(root_part_ptr_->pose());
    x3real yaw = std::get<2>(pose.rpy());
    pose.set_xyz(pose.x(), pose.y(), pose.z());
    x3real vx = move_speed_norm_ * sin(yaw);
    x3real vy = -move_speed_norm_ * cos(yaw);
    x3real vz = root_part_ptr_->speed_z();
    root_part_ptr_->set_speed(vx, vy, 0.0f);
}

void URDFItem::turn_left() {
    LOG(INFO) << "turn left";
    Pose pose(root_part_ptr_->pose());
    pose.set_xyz(pose.x(), pose.y(), pose.z());
    pose.rotate_z(FLAGS_x3_turning_rad);
    x3real vz = root_part_ptr_->speed_z();
    root_part_ptr_->set_pose(pose);
    root_part_ptr_->set_pose_and_speed(pose, 0.0f, 0.0f, 0.0f);
}

void URDFItem::turn_right() {
    Pose pose(root_part_ptr_->pose());
    x3real yaw = std::get<2>(pose.rpy());
    pose.set_xyz(pose.x(), pose.y(), pose.z());
    pose.rotate_z(-FLAGS_x3_turning_rad);
    LOG(INFO) << "turn right from " << yaw << " to " << std::get<2>(pose.rpy());
    x3real vz = root_part_ptr_->speed_z();
    root_part_ptr_->set_pose_and_speed(pose, 0.0f, 0.0f, 0.0f);
}

void URDFItem::clear_move() {
    Pose pose(root_part_ptr_->pose());
    root_part_ptr_->set_pose_and_speed(pose, 0.0f, 0.0f, 0.0f);
}

x3real URDFItem::reach_test(const Pose& pose) {
    const Pose self = root_part_ptr_->pose();
    x3real yaw = std::get<2>(pose.rpy());
    x3real dir_x = cos(yaw);
    x3real dir_y = sin(yaw);
    x3real dx = pose.x() - self.x();
    x3real dy = pose.y() - self.y();
    x3real dz = pose.z() - self.z();
    x3real d = sqrt(dx * dx + dy * dy);
    x3real reaching_score = -1; // lower end of cos range
    if (d < reaching_dist_ && dz < REACH_HEIGHT_THRESHOLD) {
        dx /= d;
        dy /= d;
        reaching_score = dx * dir_x + dy * dir_y;
    }
    return reaching_score;
}

/********************************* MuJoCoItem *********************************/
MuJoCoItem::MuJoCoItem(const Entity& e, x3real scale, World& world, bool mjcf) :
        X3Item(e, world) {
    parts_ = world.load_sdf_mjcf(e.asset_path, mjcf);
    root_part_ptr_ = &(parts_[0]);

    set_entity(e);
}

void MuJoCoItem::move_forward() {
    x3real dir_x, dir_y;
    get_direction(dir_x, dir_y);
    LOG(INFO) << "move_forward direction: " << dir_x << " " << dir_y;
    x3real vx = move_speed_norm_ * dir_x;
    x3real vy = move_speed_norm_ * dir_y;
    float pos, vel;
    root_part_ptr_->joint_current_position(0, pos, vel);
    root_part_ptr_->joint_reset_current_position(0, pos, vx);
    root_part_ptr_->joint_current_position(1, pos, vel);
    root_part_ptr_->joint_reset_current_position(1, pos, vy);
}

void MuJoCoItem::move_backward() {
    x3real dir_x, dir_y;
    get_direction(dir_x, dir_y);
    x3real vx = -move_speed_norm_ * dir_x;
    x3real vy = -move_speed_norm_ * dir_y;
    float pos, vel;
    root_part_ptr_->joint_current_position(0, pos, vel);
    root_part_ptr_->joint_reset_current_position(0, pos, vx);
    root_part_ptr_->joint_current_position(1, pos, vel);
    root_part_ptr_->joint_reset_current_position(1, pos, vy);
}

void MuJoCoItem::move_left() {
    x3real dir_x, dir_y;
    get_direction(dir_x, dir_y);
    x3real vx = -move_speed_norm_ * dir_y;
    x3real vy = move_speed_norm_ * dir_x;
    float pos, vel;
    root_part_ptr_->joint_current_position(0, pos, vel);
    root_part_ptr_->joint_reset_current_position(0, pos, vx);
    root_part_ptr_->joint_current_position(1, pos, vel);
    root_part_ptr_->joint_reset_current_position(1, pos, vy);
}

void MuJoCoItem::move_right() {
    x3real dir_x, dir_y;
    get_direction(dir_x, dir_y);
    x3real vx = move_speed_norm_ * dir_y;
    x3real vy = -move_speed_norm_ * dir_x;
    float pos, vel;
    root_part_ptr_->joint_current_position(0, pos, vel);
    root_part_ptr_->joint_reset_current_position(0, pos, vx);
    root_part_ptr_->joint_current_position(1, pos, vel);
    root_part_ptr_->joint_reset_current_position(1, pos, vy);
}

void MuJoCoItem::turn_left() {
    float pos, vel;
    root_part_ptr_->joint_current_position(0, pos, vel);
    root_part_ptr_->joint_reset_current_position(0, pos, 0);
    root_part_ptr_->joint_current_position(1, pos, vel);
    root_part_ptr_->joint_reset_current_position(1, pos, 0);
    root_part_ptr_->joint_current_position(2, pos, vel);
    pos += FLAGS_x3_turning_rad;
    if (pos < 0) {
        pos += M_PI * 2;
    } else if (pos > M_PI * 2) {
        pos -= M_PI * 2;
    }
    root_part_ptr_->joint_reset_current_position(2, pos, 0);
}

void MuJoCoItem::turn_right() {
    float pos, vel;
    root_part_ptr_->joint_current_position(0, pos, vel);
    root_part_ptr_->joint_reset_current_position(0, pos, 0);
    root_part_ptr_->joint_current_position(1, pos, vel);
    root_part_ptr_->joint_reset_current_position(1, pos, 0);
    root_part_ptr_->joint_current_position(2, pos, vel);
    pos -= FLAGS_x3_turning_rad;
    if (pos < 0) {
        pos += M_PI * 2;
    } else if (pos > M_PI * 2) {
        pos -= M_PI * 2;
    }
    root_part_ptr_->joint_reset_current_position(2, pos, 0);
}

void MuJoCoItem::clear_move() {
    float pos, vel;
    root_part_ptr_->joint_current_position(0, pos, vel);
    root_part_ptr_->joint_reset_current_position(0, pos, 0);
    root_part_ptr_->joint_current_position(1, pos, vel);
    root_part_ptr_->joint_reset_current_position(1, pos, 0);
    root_part_ptr_->joint_current_position(2, pos, vel);
    root_part_ptr_->joint_reset_current_position(2, pos, 0);
}

void MuJoCoItem::joint_control(const size_t joint_id, const x3real delta) {
    float pos, vel;
    root_part_ptr_->joint_current_relative_position(joint_id, pos, vel); 
    pos = std::max(-1.0, std::min(pos + delta, 1.0));
    root_part_ptr_->joint_set_relative_servo_target(joint_id, pos);
}

void MuJoCoItem::set_entity(const Entity& e) {
    e_ = e;
    Pose pose(e.loc.x * UNIT, e.loc.y * UNIT, e.loc.z * UNIT);
    root_part_ptr_->set_pose_and_speed(pose, 0.0f, 0.0f, 0.0f);
    root_part_ptr_->joint_reset_current_position(0, 0, 0);
    root_part_ptr_->joint_reset_current_position(1, 0, 0);
    root_part_ptr_->joint_reset_current_position(2, e.yaw, 0);
}

X3ItemPtr MuJoCoItem::collect_item(
        const std::map<std::string, X3ItemPtr>& items,
        const std::string& type) {
    // TODO
    return nullptr;
}

x3real MuJoCoItem::reach_test(const Pose& pose) {
    // TODO
    return 0;
}

/********************************** X3Camera **********************************/
X3Camera::X3Camera(World& world, int img_height, int img_width) :
        camera_(world.new_camera_free_float(img_width, img_height, "camera")),
        item_(NULL),
        view_angle_(0) {}

void X3Camera::attach_item(X3Item* item) {
    if (item && item_ != item) {
        item_ = item;
    }
}

void X3Camera::update() {
    // TODO: to support rendering using detached camera
    CHECK(item_) << "camera is detached";
    Pose p = item_->pose();
    x3real dir_x, dir_y;
    item_->get_direction(dir_x, dir_y);
    LOG(INFO) << "camera update " << dir_x << " " << dir_y;
    switch (view_angle_) {
        case 0:
            // first person
            item_->get_direction(dir_x, dir_y);
            camera_.move_and_look_at(p.x(), p.y(), p.z()+0.9*X3Item::UNIT,
                                     p.x()+dir_x, p.y()+dir_y, p.z()+0.5);
            break;
        case 1:
            // bird view
            camera_.move_and_look_at(p.x(), p.y(), p.z()+2, p.x(), p.y(), p.z());
            break;
        case 2:
            // left
            camera_.move_and_look_at(p.x()-dir_y, p.y()+dir_x, p.z()+0.5, p.x(), p.y(), p.z()+0.5);
            break;
        case 3:
            // right
            camera_.move_and_look_at(p.x()+dir_y, p.y()-dir_x, p.z()+0.5, p.x(), p.y(), p.z()+0.5);
            break;
        case 4:
            // front
            camera_.move_and_look_at(p.x()+dir_x, p.y()+dir_y, p.z()+0.5, p.x(), p.y(), p.z()+0.5);
            break;
    }
}

roboschool::RenderResult X3Camera::render(X3Item* item) {
    attach_item(item);
    update();
    return camera_.render(false, false, false);
}

}} // simulator::xworld3d
