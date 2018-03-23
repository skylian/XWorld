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

#pragma once

#include <map>
#include <tuple>

#include "roboschool_API.h"
#include "simulator_entity.h"
#include "simulator_util.h"
#include "xworld3d_flags.h"

namespace simulator {
namespace xworld3d {

#define EPSILON 1e-6

class X3Item;

typedef double x3real;
typedef std::shared_ptr<roboschool::Object> ObjectPtr;
typedef std::shared_ptr<X3Item> X3ItemPtr;

class X3Item {
public:
    using World = roboschool::World;
    using Part = roboschool::Object;
    using Thing = roboschool::Thingy;
protected:
    using Pose = roboschool::Pose;
    using RenderResult = roboschool::RenderResult;
    using Camera = roboschool::Camera;
public:
    static X3ItemPtr create_item(const Entity& e, World& world);

    X3Item(const Entity& e, World& world);

    X3Item(const X3Item&) = delete;

    X3Item& operator=(const X3Item&) = delete;

    // destructor
    virtual ~X3Item() { this->destroy(); }

    void destroy();

    std::vector<Part>& parts() { return parts_; }

    // get the type of the item
    std::string type() const { return e_.type; }

    // get the id of the item
    std::string id() const { return e_.id; }

    // get the name of the item
    std::string name() const { return e_.name; }

    // get the asset path of the item
    std::string asset_path() const { return e_.asset_path; }

    // get the color of the item
    std::string color() const { return e_.color; }

    Pose pose() const;

    void get_direction(x3real &dir_x, x3real &dir_y) const;

    Entity entity() const { return e_; }

    void set_entity(const Entity& e);

    void sync_entity_info();

    virtual void move_forward() = 0;

    virtual void move_backward() = 0;

    virtual void move_left() = 0;

    virtual void move_right() = 0;

    virtual void turn_left() = 0;

    virtual void turn_right() = 0;

    virtual void clear_move() = 0;

    void move_underground();

    virtual void joint_control(const size_t joint_id, const x3real delta) {
        LOG(FATAL) << "actions not defined!";
    }

    virtual X3ItemPtr collect_item(const std::map<std::string, X3ItemPtr>& items,
                                   const std::string& type) = 0;
    virtual int get_num_actions() const { return 0; }

    bool equal(const X3Item& i) const {
        return (this->e_.id == i.e_.id);
    }

    static const x3real UNIT;
    static const x3real UNIT_INV;

protected:
    void set_pose(const Pose& pose);

    void set_speed(const x3real vx, const x3real vy, const x3real vz);

    void set_pose_and_speed(const Pose& pose,
                            const x3real vx,
                            const x3real vy,
                            const x3real vz);

   virtual x3real reach_test(const Pose& pose) {
        LOG(FATAL) << "function not defined!;";
    }

    const x3real move_speed_norm_;
    const x3real jump_speed_norm_;
    const x3real reaching_dist_; // An agent can collect a goal if the goal is
                                 // within this reaching distance
    Entity e_;
    std::vector<Part> parts_;
    Part* root_part_ptr_;
};


class URDFItem : public X3Item {
public:
    URDFItem(const Entity& e, x3real scale, World& world);

    URDFItem(const URDFItem&) = delete;

    URDFItem& operator=(const URDFItem&) = delete;

    X3ItemPtr collect_item(const std::map<std::string, X3ItemPtr>& items,
                           const std::string& type) override;

    virtual void move_forward() override;
    
    virtual void move_backward() override;

    virtual void move_left() override;

    virtual void move_right() override;

    virtual void turn_left() override;

    virtual void turn_right() override;

    virtual void clear_move() override;

protected:
    x3real reach_test(const Pose& pose) override;

    int yaw_id_;
};

class MuJoCoItem : public X3Item {
public:
    MuJoCoItem(const Entity& e, x3real scale, World& world);

    MuJoCoItem(const MuJoCoItem&) = delete;

    MuJoCoItem& operator=(const MuJoCoItem&) = delete;

    void move_forward() override;

    void move_backward() override;

    void move_left() override;

    void move_right() override;

    void turn_left() override;

    void turn_right() override;

    void clear_move() override;

    X3ItemPtr collect_item(const std::map<std::string, X3ItemPtr>& items,
                           const std::string& type) override;

    void joint_control(const size_t joint_id, const x3real delta) override;

private:
    x3real reach_test(const Pose& pose) override;
};

class X3Camera {
private:
    using Pose = roboschool::Pose;
    using World = roboschool::World;
    using Camera = roboschool::Camera;
public:
    X3Camera(World& world, int img_height, int img_width);

    X3Camera(const X3Camera&) = delete;

    X3Camera& operator=(const X3Camera&) = delete;

    // TODO: make this function const
    Pose pose() { return camera_.pose(); }

    // Return the image seen by agent.
    // If bird_view is true, a bird view image is also returned.
    roboschool::RenderResult render(X3Item* item, bool bird_view = false);

    // Camera can be attached to an agent so that the rendered image is centered
    // at the agent
    void attach_item(X3Item* item);

    void detach() { item_ = NULL; }

private:
    // Update the pose of the camera.
    void update(bool bird_view);

    Camera camera_;
    X3Item* item_;
};

}} // simulator::xworld3d

//namespace std {
//template <>
//struct hash<simulator::Vec3> {
//    size_t operator()(const simulator::Vec3& l) const {
//        return hash<simulator::xworld3d::x3real>()(l.x) ^
//               hash<simulator::xworld3d::x3real>()(l.y) ^
//               hash<simulator::xworld3d::x3real>()(l.z);
//    }
//};
//}
