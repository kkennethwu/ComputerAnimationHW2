#pragma once
#include <vector>
#include <string>
#include "acclaim/posture.h"

namespace acclaim {
struct Bone;
}
namespace kinematics {
// Apply forward kinematics to skeleton
void forwardSolver(const acclaim::Posture& posture, acclaim::Bone* bone);
// Apply time warping to motion
std::vector<acclaim::Posture> timeWarper(const std::vector<acclaim::Posture>& postures, int allframe_old, int allframe_new);
}  // namespace kinematics
