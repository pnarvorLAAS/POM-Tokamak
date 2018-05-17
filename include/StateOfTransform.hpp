#ifndef __STATE_OF_TRANSFORM_HPP__
#define __STATE_OF_TRANSFORM_HPP__

#include <PositionManagerBase.hpp>

namespace tokamak
{
    struct StateOfTransform
    {
        bool fused;
        int flagged;
        PositionManager::TimeUs timeofAddition;
        PositionManager::Pose pose_fixedFrame_robotFrame;
    };
}


#endif
