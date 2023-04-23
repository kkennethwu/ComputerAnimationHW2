#include "simulation/ball.h"
#include <utility>
#include "acclaim/bone.h"
#include <iostream>
namespace kinematics {
Ball::Ball(const acclaim::Bone* bone) noexcept : catcher(bone), graphics(std::make_unique<graphics::Sphere>()) {}
Ball::Ball(Ball&& other) noexcept
    : pre_pos(std::move(other.pre_pos)),
      thrown_pos(std::move(other.pre_pos)),
      gravity(std::move(other.gravity)),
      throw_velocity(std::move(other.throw_velocity)),
      catched(other.catched),
      thrown(other.thrown),
      catcher(other.catcher),
      thrown_frame_base(other.thrown_frame_base),
      thrown_frame(other.thrown_frame),
      graphics(std::move(other.graphics)) {}

Ball& Ball::operator=(Ball&& other) noexcept {
    if (this != &other) {
        pre_pos = std::move(other.pre_pos);
        thrown_pos = std::move(other.thrown_pos);
        gravity = std::move(other.gravity);
        throw_velocity = std::move(other.throw_velocity);
        catched = other.catched;
        thrown = other.thrown;
        catcher = other.catcher;
        thrown_frame_base = other.thrown_frame_base;
        thrown_frame = other.thrown_frame;
        graphics = std::move(other.graphics);
    }
    return *this;
}
void Ball::release() { catched = false; }

std::unique_ptr<graphics::Sphere>& Ball::getGraphics() { return graphics; }

void Ball::set_model_matrix(int time) {
    if (time == 0) {
        catched = true;
        thrown = false;
    }
    Eigen::Affine3d trans = Eigen::Affine3d::Identity();
    Eigen::Vector4d center = (catcher->start_position + catcher->end_position) * 0.5;
    
    if (catched) {
        
        if (time >= thrown_frame && !thrown) {
            thrown = true;
            thrown_pos = center;
            throw_velocity = thrown_pos - pre_pos;
        }

        if (!thrown) {

            trans.translate(center.head<3>());
            pre_pos = center;

        }

    } 

    if (thrown) {

        Eigen::Vector4d new_position = thrown_pos + throw_velocity * (time - thrown_frame) +
                                       0.005 * (time - thrown_frame) * (time - thrown_frame) * (gravity);
        trans.translate(new_position.head<3>());
    }

    trans.scale(Eigen::Vector3d(0.125, 0.125, 0.125));
    graphics->setModelMatrix(trans.cast<float>());
}

void Ball::set_thrown_frame(int total_frame_new) { 
    thrown_frame = thrown_frame_base * total_frame_new / 328;
}
}  // namespace kinematics
