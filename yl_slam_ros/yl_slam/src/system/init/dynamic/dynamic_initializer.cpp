#include "system/init/dynamic/dynamic_initializer.h"

namespace YL_SLAM {

DynamicInitializer::DynamicInitializer([[maybe_unused]] const VecXf &parameters, Vec3f g_w) : g_w_(std::move(g_w)) {
    YL_FATAL("Not implement yet!");
}

bool DynamicInitializer::initialize([[maybe_unused]] const FrameBundle::sPtr &bundle,
                                    [[maybe_unused]] const Imus &imus) {
    return false;
}

void DynamicInitializer::reset() {}

void DynamicInitializer::print(std::ostream &out) const {
    out << "Dynamic initializer: Not implement yet!" << std::endl;
}

} // namespace YL_SLAM
