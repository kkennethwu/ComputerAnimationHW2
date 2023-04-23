#pragma once

#include <memory>

#include "Eigen/Core"
#include "graphics/sphere.h"

namespace acclaim {
struct Bone;
}
namespace kinematics {
class Ball {
 public:
    explicit Ball(const acclaim::Bone* bone) noexcept;
    // no copy constructor
    Ball(const Ball&) = delete;
    Ball(Ball&&) noexcept;
    // You need this for alignment otherwise it may crash
    // Ref: https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Ball& operator=(const Ball&) = delete;
    Ball& operator=(Ball&&) noexcept;
    // Get the underlying graphics object
    std::unique_ptr<graphics::Sphere>& getGraphics();
    // Calculate and set position at specific time stamp
    void set_model_matrix(int time);
    // Reset the ball
    void release();
    void set_thrown_frame(int total_frame_new);

 private:
    Eigen::Vector4d pre_pos;
    Eigen::Vector4d thrown_pos;
    Eigen::Vector4d gravity = Eigen::Vector4d(0.0, -0.098, 0.0, 0.0);
    Eigen::Vector4d throw_velocity;
    bool catched = true;
    bool thrown = false;
    const acclaim::Bone* catcher;
    int thrown_frame_base = 276 - 150;
    float thrown_frame =  276 - 150; 
    std::unique_ptr<graphics::Sphere> graphics;
};
}  // namespace kinematics
