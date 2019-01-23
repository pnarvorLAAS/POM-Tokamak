#ifndef __UNARYFACTOR_HPP__
#define __UNARYFACTOR_HPP__

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace std;
using namespace gtsam;

//#define SLOW_BUT_CORRECT_BINARYROTATIONFACTOR
namespace tokamak
{
    namespace Unary
    {
        class GPSFactor : public NoiseModelFactor1<Pose3>
        {
            Point3 measure_;
            Point3 robot_sensor_;
            Pose3 global_site_;
        
            public:
        
            virtual ~GPSFactor() {}
        
            typedef boost::shared_ptr<GPSFactor> shared_ptr;
            
            GPSFactor(Key j1, Point3 m, Point3 p,Pose3 sf2gtf, const SharedNoiseModel& model): NoiseModelFactor1<Pose3>(model,j1), measure_(m), robot_sensor_(p), global_site_(sf2gtf) {}
        
            GPSFactor(Key j1,  Point3 m, Pose3 sf2gtf, double altitude, const SharedNoiseModel& model): NoiseModelFactor1<Pose3>(model,j1), measure_(m), global_site_(sf2gtf)
            {
                    robot_sensor_ = Point3(Point3::identity());
            }
        
            Vector evaluateError(const Pose3 &site_robot, boost::optional<Matrix&> Hi = boost::none) const;
        };
    }
}


#endif
