#ifndef __TOKAMAK_HPP__
#define __TOKAMAK_HPP__

#include <PositionManagerBase.hpp>
#include <StateOfTransform.hpp>
#include <vector>

#define DEFAULT_FREQUENCY   30
#define DEFAULT_TIME_KEPT   300
#define DEFAULT_FIXED_FRAME "localTerrainFrame"

namespace tokamak
{
    class Tokamak
    {
        protected:
            std::map<TimeUs,StateOfTransform> timeLine; // Ideal would be a circular map
            PositionManager::Graph poseGraph; // Actual Graph
            PositionManager::FrameId fixedFrame; // Frame in which the robot pose will be released by PoM
            int32_t runningFrequency; //Frequency of the base job of PoM (release latest pose)
            int32_t secondsKept;
            int32_t bufferSize;


        public:
            Tokamak();
            Tokamak(int32_t freq);
            ~Tokamak();
            insertNewTransforms(std::vector<PositionManager::Pose>& listOfTransforms /*, timeLine*/);
            getLatestRobotPose(/*timeLine,poseGraph,fixedFrame*/); // Latest Transform from RBF (Robot Body Frame) to LTF (Local Terrain Frame)

            getTransform(PositionManager::Pose pose/*, timeLine*/);

            getTransform(TimeUs parentTime, 
                         TimeUs childTime, 
                         PositionManager::FrameId parentFrame, 
                         PositionManager::FrameId childFrame /*,
                         timeLine
                         */
                         );

            getTransform(TimeUs time,
                         PositionManager::FrameId parentFrame,
                         PositionManager::FrameId childFrame /*,
                         timeLine
                         */
                         );

            getTransform(TimeUs parentTime,
                         TimeUs childTime,
                         PositionManager::FrameId frame /*,
                         timeLine
                         */
                         );

            flagTransform(TimeUs time);
            unFlagTransform(TimeUs time);



    };
}

#endif
