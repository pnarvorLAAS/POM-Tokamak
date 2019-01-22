#ifndef __RELATIVEMEASURE_HPP__
#define __RELATIVEMEASURE_HPP__

#include <gtsam/geometry/Pose3.h> // Pose3
#include <gtsam/inference/Key.h> // Key to associate with graph nodes
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;

namespace tokamak
{
    namespace RelativeMeasure
    {
        class TranslationFactor: public NoiseModelFactor2<Pose3,Pose3>
        {
            Point3 measure_;
        
            public:
        
            virtual ~TranslationFactor(){}
        
            TranslationFactor(Key j1,Key j2, Point3 m, const SharedNoiseModel& model): NoiseModelFactor2<Pose3,Pose3>(model,j1,j2), measure_(m) {}
        
            Vector evaluateError(const Pose3 &site_robot1, const Pose3 &site_robot2,  boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const;
            
            virtual gtsam::NonlinearFactor::shared_ptr clone() const;
        };
    
        typedef BetweenFactor<Pose3> FullPoseFactor;
    }
}

#endif
