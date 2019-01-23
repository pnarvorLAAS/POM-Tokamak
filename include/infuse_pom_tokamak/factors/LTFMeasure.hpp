#ifndef __LTFMEASURE_HPP__
#define __LTFMEASURE_HPP__

#include <gtsam/slam/PriorFactor.h> // Necessary for first Local Terrain Frame measure
#include <gtsam/geometry/Pose3.h> // Pose3
#include <gtsam/geometry/Pose2.h> // Pose 2
#include <gtsam/inference/Key.h> // Key to associate with graph nodes


namespace tokamak
{
    namespace LTFMeasure
    {
        using namespace gtsam;
        typedef PriorFactor<Pose2> Prior;
    
        class RotationFactor : public NoiseModelFactor2<Pose2,Pose3>
        {
            Rot3 measure_;

            public:
            RotationFactor(Key j1, Key j2, Rot3 m, const SharedNoiseModel& model):
                NoiseModelFactor2<Pose2,Pose3>(model,j1,j2), measure_(m) {}

            virtual ~RotationFactor() {}

            Vector evaluateError(const Pose2& site_local, const Pose3& site_robot, boost::optional<Matrix&> Hi = boost::none, boost::optional<Matrix&> Hj = boost::none) const;

        };
        
        class TranslationFactor: public NoiseModelFactor2<Pose2,Pose3>
        {

            Point3 measure_;
            double altitude_;
        
            public:
        
            virtual ~TranslationFactor(){}
        
            TranslationFactor(Key j1,Key j2, Point3 m, double altitude, const SharedNoiseModel& model): NoiseModelFactor2<Pose2,Pose3>(model,j1,j2), measure_(m), altitude_(altitude) {}
        
            Vector evaluateError(const Pose2 &site_local, const Pose3 &site_robot,  boost::optional<Matrix&> H_sl = boost::none, boost::optional<Matrix&> H_sr = boost::none) const;

            virtual gtsam::NonlinearFactor::shared_ptr clone() const;
        };
        
        class FullPoseFactor : public NoiseModelFactor2<Pose2,Pose3>
        {
            Pose3 measure_;
            double altitude_;

            public: 

            virtual ~FullPoseFactor() {}

            typedef boost::shared_ptr<FullPoseFactor> shared_ptr;

            FullPoseFactor(Key j1, Key j2, Pose3 measure, double altitude, const SharedNoiseModel& model) : NoiseModelFactor2<Pose2,Pose3>(model, j1, j2),measure_(measure), altitude_(altitude) {}

            Vector evaluateError(const Pose2 &Ti, const Pose3 &Tj, boost::optional<Matrix&> Hi = boost::none, boost::optional<Matrix&> Hj = boost::none) const;
        };
    }
}

#endif
