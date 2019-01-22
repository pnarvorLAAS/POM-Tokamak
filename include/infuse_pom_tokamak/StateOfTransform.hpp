#ifndef __STATE_OF_TRANSFORM_HPP__
#define __STATE_OF_TRANSFORM_HPP__

#include <infuse_pom_base/PositionManagerBase.hpp>
#include <gtsam/inference/Key.h>

namespace tokamak
{
    struct StateOfTransform
    {
        bool fused;
        int flagged;
        bool isDeltaPose;
        PositionManager::TimeUs timeOfAddition;
        PositionManager::TimeUs timeOfParent;
        PositionManager::Pose pose_fixedFrame_robotFrame;
        
        int countOptimizer;
        bool isComplete;
        gtsam::Key graphKey;
    };
}


#endif
