#ifndef __STATE_OF_TRANSFORM_HPP__
#define __STATE_OF_TRANSFORM_HPP__

#include <PositionManagerBase.hpp>

namespace tokamak
{
    struct StateOfTransform
    {
        //TODO add transform
        bool fused;
        int flagged;
        PositionManager::FrameId frameId;
        PositionManager::TimeUs timeofAddition;
    };
}


#endif
