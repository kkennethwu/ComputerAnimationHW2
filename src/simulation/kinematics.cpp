#include "simulation/kinematics.h"

#include "Eigen/Dense"
#include <iostream>
#include "acclaim/bone.h"
#include "util/helper.h"

namespace kinematics {
void forwardSolver(const acclaim::Posture& posture, acclaim::Bone* bone) {
    // TODO#1 (FK)
    // You should set these variables:
    //     bone->start_position = Eigen::Vector4d::Zero();
    //     bone->end_position = Eigen::Vector4d::Zero();
    //     bone->rotation = Eigen::Matrix4d::Zero();
    // The sample above just set everything to zero
    // Hint:
    //   1. posture.bone_translations, posture.bone_rotations
    // Note:
    //   1. This function will be called with bone == root bone of the skeleton
    //   2. we use 4D vector to represent 3D vector, so keep the last dimension as "0"
    //   3. util::rotate{Degree | Radian} {XYZ | ZYX}
    //      e.g. rotateDegreeXYZ(x, y, z) means:
    //      x, y and z are presented in degree rotate z degrees along z - axis first, then y degrees along y - axis, and then x degrees along x - axis 
    

    if (bone->parent == nullptr) {
        bone->rotation = bone->rot_parent_current * util::rotateDegreeZYX(posture.bone_rotations[bone->idx]);
        bone->start_position = posture.bone_translations[bone->idx];
        bone->end_position = bone->rotation * bone->dir * bone->length + bone->start_position;
    } 
    else {
        bone->rotation = bone->parent->rotation * bone->rot_parent_current * util::rotateDegreeZYX(posture.bone_rotations[bone->idx]); //矩陣相乘不能換位置
        bone->start_position = bone->parent->end_position;
        bone->end_position = bone->rotation * bone->dir * bone->length + bone->start_position;
    }
    /*DFS*/
    if (bone->child != nullptr) forwardSolver(posture, bone->child);
    if (bone->sibling != nullptr) forwardSolver(posture, bone->sibling);


}

std::vector<acclaim::Posture> timeWarper(const std::vector<acclaim::Posture>& postures, int allframe_old, int allframe_new) {

    int total_frames = static_cast<int>(postures.size());
    int total_bones = static_cast<int>(postures[0].bone_rotations.size());
    std::vector<acclaim::Posture> new_postures;
    for (int i = 0; i < allframe_new; ++i) {
        acclaim::Posture new_poseture(total_bones);
        for (int j = 0; j < total_bones; ++j) {

            // TODO#2 (Time warping)
            // original: |--------------|
            // new     : |----------------------|
            // OR
            // original: |--------------|
            // new     : |-------|
            // You should set these variables:
            //     new_postures[i].bone_translations[j] = Eigen::Vector4d::Zero();
            //     new_postures[i].bone_rotations[j] = Eigen::Vector4d::Zero();
            // The sample above just set everything to zero
            // Hint:
            //   1. Scale the frames.
            //   2. You can use linear interpolation with translations.
            //   3. You should use spherical linear interpolation for rotations.


            float ratio = allframe_old / allframe_new;
            int frame1 = floor(ratio * i);
            int frame2 = frame1 + 1;
            float interpolation = ratio * i - frame1;
            /*translation*/
            new_poseture.bone_translations[j] = 
                (postures[frame2].bone_translations[j] - postures[frame1].bone_translations[j]) * interpolation + postures[frame1].bone_translations[j];
            /*rotation*/
            Eigen::Quaterniond Q1(postures[frame1].bone_rotations[j]);
            Eigen::Quaterniond Q2(postures[frame2].bone_rotations[j]);
            new_poseture.bone_rotations[j] = Q1.slerp(interpolation, Q2).coeffs();              
        }


        new_postures.push_back(new_poseture);
    }
    return new_postures;
}
}  // namespace kinematics
