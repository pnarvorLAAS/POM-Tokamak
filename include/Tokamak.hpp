#ifndef __TOKAMAK_HPP__
#define __TOKAMAK_HPP__

#include <PositionManagerBase.hpp>
#include <StateOfTransform.hpp>
#include <exception.hpp>
#include <circularMap.hpp>
#include <vector>
#include <mutex>

#define DEFAULT_FREQUENCY   30 // Frequency at which poses are outputed
#define DEFAULT_TIME_KEPT   300 // Maximum size of the timeline in seconds
#define DEFAULT_CONSERVATION_TIME 300 //Time during wich we conserve unflagged transforms in seconds
#define DEFAULT_FIXED_FRAME "localTerrainFrame" // Fixed reference frame in which robot poses will be outputed
#define DEFAULT_ROVER_FRAME "robotBodyFrame" // Frame of the robot

#define e_wrong_childframe              Error("Child frame is not robot")
#define e_wrong_delta                   Error("Wrong delta pose format")
#define e_future_transform              Error("Asking for a transform in the future")
#define e_inverse_time_transform        Error("The transform asked is reversed in time")
#define e_interpolation                 Error("The transform requires interpolation. It has not been implemented yet")
#define e_static_transform              Error("The transform has same parent and child. It no transform at all")
#define e_wrong_frames                  Error("The request is in the wrong frame");


namespace tokamak
{
    typedef std::map<PositionManager::TimeUs,StateOfTransform>::iterator iterator;

    class Tokamak
    {
        protected:
            circularMap<PositionManager::TimeUs,StateOfTransform>* timeLine; 
            int32_t runningFrequency; //Frequency of the base job of PoM (release latest pose)
            int32_t secondsKept; // Number of seconds contained into memory
            PositionManager::FrameId fixedFrame; // Frame in which the robot pose will be released by PoM
            PositionManager::FrameId robotBodyFrame; // Frame describing the robot pose
            int32_t bufferSize; // Size of the buffer to hold all information in memory

            std::mutex transformAccess;
            void lockTimeLine();
            void unlockTimeLine();


        public:
            Tokamak();
            Tokamak(int32_t freq); // Constructor with tokamak running frequency
            Tokamak(int32_t freq, int32_t sec); // Constructor with tokamak running frequency and number of seconds kept
            Tokamak(int32_t freq, int32_t sec, std::string worldFrame);
            Tokamak(int32_t freq, int32_t sec, std::string worldFrame, std::string robotFrame);
            ~Tokamak();
            void clean_up(); // Release memory allocated by instanciating tokamak

            //TODO Add a way for PoM to know which frames exist --> URDF/XML => Format à définir 
            void validityCheckInsertion(const PositionManager::Pose transform);
            bool insertNewTransform(PositionManager::Pose transform /*,timeLine*/);
            bool insertNewTransforms(std::vector<PositionManager::Pose>& listOfTransforms /*, timeLine*/);
            PositionManager::Pose getLatestRobotPose(/*timeLine,poseGraph,fixedFrame*/); // Latest Transform from RBF (Robot Body Frame) to LTF (Local Terrain Frame)

            void validityCheckGetTransform(
                                        const PositionManager::TimeUs parentTime, 
                                        const PositionManager::TimeUs childTime, 
                                        const PositionManager::FrameId parentFrame, 
                                        const PositionManager::FrameId childFrame /*,
                                        timeLine
                                        */
                                        );

            PositionManager::Pose getTransform(PositionManager::Pose pose);

            PositionManager::Pose getTransform(
                                        const PositionManager::TimeUs parentTime, 
                                        const PositionManager::TimeUs childTime, 
                                        const PositionManager::FrameId parentFrame, 
                                        const PositionManager::FrameId childFrame /*,
                                        timeLine
                                        */
                                        );

            PositionManager::Pose getTransform(
                                        const PositionManager::TimeUs time,
                                        const PositionManager::FrameId parentFrame,
                                        const PositionManager::FrameId childFrame /*,
                                        timeLine
                                        */
                                        );

            PositionManager::Pose getTransform(
                                        const PositionManager::TimeUs parentTime,
                                        const PositionManager::TimeUs childTime,
                                        const PositionManager::FrameId frame /*,
                                        timeLine
                                        */
                                        );

            int garbageCollector(/*timeLine*/); // Remove transforms that are flagged as useless for more than a given time

            //TODO: add some kind of serialization of both timeline and graph... Probably need some kind of work on Envire..


            bool flagTransform(PositionManager::TimeUs time); //Times might not be exactly the ones the DFPCs ask for. In this case, interpolation will be necessary.
            bool unFlagTransform(PositionManager::TimeUs time);


    };
}

#endif
