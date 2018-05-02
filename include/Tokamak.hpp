#ifndef __TOKAMAK_HPP__
#define __TOKAMAK_HPP__

#include <PositionManagerBase.hpp>
#include <StateOfTransform.hpp>
#include <vector>
#include <mutex>

#define DEFAULT_FREQUENCY   30 // Frequency at which poses are outputed
#define DEFAULT_TIME_KEPT   300 // Maximum size of the timeline in seconds
#define DEFAULT_CONSERVATION_TIME 300 //Time during wich we conserve unflagged transforms in seconds
#define DEFAULT_FIXED_FRAME "localTerrainFrame" // Fixed reference frame in which robot poses will be outputed
#define DEFAULT_ROVER_FRAME "robotBodyFrame" // Frame of the robot

namespace tokamak
{
    class Tokamak
    {
        protected:
            std::map<PositionManager::TimeUs,StateOfTransform> timeLine; // Ideal would be a circular map
            PositionManager::Graph poseGraph; // Actual Graph
            PositionManager::FrameId fixedFrame; // Frame in which the robot pose will be released by PoM
            int32_t runningFrequency; //Frequency of the base job of PoM (release latest pose)
            int32_t secondsKept; // Number of seconds contained into memory
            int32_t bufferSize; // Size of the buffer to hold all information in memory
            std::mutex transformAccess;
            void lockTimeLine();
            void unlockTimeline();


        public:
            Tokamak();
            Tokamak(int32_t freq);
            ~Tokamak();
            //TODO Add a way for PoM to know which frames exist --> URDF/XML => Format à définir 
            bool insertNewTransform(PositionManager::Pose transform /*,timeLine*/);
            bool insertNewTransforms(std::vector<PositionManager::Pose>& listOfTransforms /*, timeLine*/);
            PositionManager::Pose getLatestRobotPose(/*timeLine,poseGraph,fixedFrame*/); // Latest Transform from RBF (Robot Body Frame) to LTF (Local Terrain Frame)

            PositionManager::Pose getTransform(PositionManager::Pose pose/*, timeLine*/);

            PositionManager::Pose getTransform(
                                        PositionManager::TimeUs parentTime, 
                                        PositionManager::TimeUs childTime, 
                                        PositionManager::FrameId parentFrame, 
                                        PositionManager::FrameId childFrame /*,
                                        timeLine
                                        */
                                        );

            PositionManager::Pose getTransform(
                                        PositionManager::TimeUs time,
                                        PositionManager::FrameId parentFrame,
                                        PositionManager::FrameId childFrame /*,
                                        timeLine
                                        */
                                        );

            PositionManager::Pose getTransform(
                                        PositionManager::TimeUs parentTime,
                                        PositionManager::TimeUs childTime,
                                        PositionManager::FrameId frame /*,
                                        timeLine
                                        */
                                        );

            int garbageCollector(/*timeLine*/); // Remove transforms that are flagged as useless for more than a giev time

            //TODO: add some kind of serialization of both timeline and graph... Probably need some kind of work on Envire..


            bool flagTransform(PositionManager::TimeUs time); //Times might not be exactly the ones the DFPCs ask for. In this case, interpolation will be necessary.
            bool unFlagTransform(PositionManager::TimeUs time);


    };
}

#endif
