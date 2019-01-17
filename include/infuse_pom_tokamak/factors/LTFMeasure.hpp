#ifndef __LTFMEASURE_HPP__
#define __LTFMEASURE_HPP__

#include <gtsam/slam/PriorFactor.h> 

namespace tokamak
{
    namespace LTFMeasure
    {
        typedef PriorFactor<Pose2> Prior;
    }
}

#endif
