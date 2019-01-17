#ifndef BULLET_BODY_H_
#define BULLET_BODY_H_

#include "bullet_joint.h"
#include "bullet_object.h"
#include "common.h"

namespace xrobot {
namespace bullet_engine {

class BulletBody {
public:
    BulletBody() : root_obj_(NULL) {}

    virtual ~BulletBody() {}

// protected:
    bool load_urdf(
            const ClientHandle client,
            const std::string& filename,
            const glm::vec3& pos,
            const glm::vec4& quat,
            const double scale,
            const bool fixed_base,
            const bool self_collision,
            const bool use_multibody,
            const bool concave);

    void load_root_part(const ClientHandle client, BulletObject* root);

    bool load_part(
            const ClientHandle client,
            const int part_id,
            BulletJoint* joint,
            BulletObject* part);

    int id() const { return body_uid_; }

    int get_visual_shape_info(const ClientHandle client);

    int get_visual_shape(
            int i,
            glm::vec4& color,
            glm::vec3& scale,
            glm::vec3& pos,
            glm::vec4& quat,
            std::string& filename,
            int& geometry_type);

    bool load_obj(
            const ClientHandle client,
            const std::string& filename,
            const glm::vec3& pos,
            const glm::vec4& quat,
            const glm::vec3& scale,
            const double mass,
            const bool concave);

    void remove_from_bullet(const ClientHandle client, const int id);

    void update_joints(const ClientHandle client);

    void query_pose(const ClientHandle client,
                    const double** room_iner_frame,
                    const double** q,
                    const double** q_dot);

    void query_link(const ClientHandle client,
                    const int id,
                    b3LinkState& state);

    void inverse_kinematics(
            const ClientHandle client,
            const int id,
            const int end_index,
            const glm::vec3& target_pos,
            const glm::vec4& target_quat,
            const double* joint_damping,
            double* output_joint_pos,
            int& num_poses);

    void get_closest_points(const ClientHandle client, 
            std::vector<ContactPoint>& points);

    void get_contact_points(const ClientHandle client, 
            std::vector<ContactPoint>& points,
            const int link_id  = -1);

public:
    BulletObject* root_obj_ = NULL;
    int body_uid_;
    b3VisualShapeInformation visual_shape_info_;
};


}} // namespace xrobot::bullet_engine
#endif
