#ifndef __TOKAMAK_HPP__
#define __TOKAMAK_HPP__

#include <infuse_pom_base/PositionManagerBase.hpp>
#include <infuse_pom_base/UrdfParser.hpp>

#include <infuse_pom_tokamak/StateOfTransform.hpp>
#include <infuse_pom_tokamak/exception.hpp>
#include <infuse_pom_tokamak/circularMap.hpp>
#include <vector>
#include <mutex>

//GTSAM
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h> // For fixed frames
#include <gtsam/slam/BetweenFactor.h> // For odometry
#include <gtsam/nonlinear/NonlinearFactor.h> // For absolute poses. Factors need to be implemented
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#define DEFAULT_CONSERVATION_TIME 300 //Time during wich we conserve unflagged transforms in seconds
#define DEFAULT_FIXED_FRAME "localTerrainFrame" // Fixed reference frame in which robot poses will be outputed
#define DEFAULT_ROVER_FRAME "robotBodyFrame" // Frame of the robot

#define e_wrong_childframe              Error("Child frame is not robot")
#define e_wrong_delta                   Error("Wrong delta pose format")
#define e_future_transform              Error("Asking for a transform in the future")
#define e_inverse_time_transform        Error("The transform asked is reversed in time")
#define e_interpolation                 Error("The transform requires interpolation. It has not been implemented yet")
#define e_static_transform              Error("The transform has same parent and child. It no transform at all")
#define e_wrong_frames                  Error("The request is in the wrong frame")
#define e_no_publish                    Error("Publishing of poses has not yet started")
#define e_urdf                          Error("Cannot process URDF file")
#define e_unknown_pose_type             Error("Only 3 pose types can be included at the moment: Translation, Rotation, and Full pose")
#define e_pose_not_recognized           Error("Unable to process such pose")
#define e_pose_insertion_future         Error("Can't insert pose in graph that are in the future")

using namespace gtsam;

namespace tokamak
{
    typedef std::map<PositionManager::TimeUs,StateOfTransform>::iterator iterator;
    enum POSE_TYPE {FULL_POSE,TRANSLATION,ORIENTATION};


    class Tokamak
    {
        private:
            typedef typename circularMap<PositionManager::TimeUs,StateOfTransform>::iterator timeIterator;
            std::shared_ptr<circularMap<PositionManager::TimeUs,StateOfTransform> > timeLine; 
            PositionManager::FrameId fixedFrame; // Frame in which the robot pose will be released by PoM
            PositionManager::FrameId robotBodyFrame; // Frame describing the robot pose
            int bufferSize; // Size of the buffer to hold all information in memory
            PositionManager::Graph fixedFramesGraph;
            PositionManager::Graph internalFramesGraph;
            std::string poseBaseline;

            std::mutex transformAccess;
            void lockTimeLine();
            void unlockTimeLine();
            void init();

            PositionManager::Pose fixedTransform;

            //GTSAM
            std::shared_ptr<gtsam::NonlinearFactorGraph> poseGraph;



        protected:
            PositionManager::Pose posePublish;
            PositionManager::Pose poseRespond;

        private:
            // Those are the factor that we are going to use in the graph
            std::string graphFrame;
            double LTFaltitude;
            int fixedFramesCounter;
            std::map<std::string,gtsam::Symbol> fixedFramesSymbol;

        public:
            POSE_TYPE getPoseType(PositionManager::Pose& transform);
            bool insertBaseline(PositionManager::Pose& transform, POSE_TYPE pt);
            bool checkBaselineTranslation(PositionManager::Pose& transform);
            bool addGraphFactor(PositionManager::Pose& transform, POSE_TYPE pt);
            bool optimize();

        public:
            Tokamak();
            Tokamak(std::string worldFrame, std::string robotFrame, std::string graphFrameName, std::string poseBaseLine);
            ~Tokamak();
            void clean_up(); // Release memory allocated by instanciating tokamak
            void print_buffer(){timeLine->print();}

            bool setAbsolutePose(double& x, double& y, double& z, double& phi, std::string& absoluteFrameId); // Set the heading from GPS data.

            bool readFixedFrames(std::string pathToUrdf);
            bool readInternalFrames(std::string pathToUrdf);
            bool addFixedFrame(const PositionManager::Pose& transform);
            bool getFixedTransform(const PositionManager::FrameId& parent); // Get Transform from world fixed frame to robot body frame
            bool convertTransform(PositionManager::Pose& pose); // Convert given robot pose to robot pose in fixed frame

            void validityCheckInsertion(const PositionManager::Pose transform);
            bool insertNewTransform(PositionManager::Pose transform /*,timeLine*/);
            bool insertNewTransforms(std::vector<PositionManager::Pose>& listOfTransforms /*, timeLine*/);
            PositionManager::Pose getLatestRobotPose(/*timeLine,posePublish,fixedFrame*/); // Latest Transform from RBF (Robot Body Frame) to LTF (Local Terrain Frame)

            void validityCheckGetTransform(
                                        const PositionManager::TimeUs parentTime, 
                                        const PositionManager::TimeUs childTime, 
                                        const PositionManager::FrameId parentFrame, 
                                        const PositionManager::FrameId childFrame /*,
                                        timeLine
                                        */
                                        );

            PositionManager::Pose getTransform(PositionManager::Pose pose/*,timeLine,poseRequest*/);

            PositionManager::Pose getTransform(
                                        PositionManager::TimeUs parentTime, 
                                        PositionManager::TimeUs childTime, 
                                        const PositionManager::FrameId parentFrame, 
                                        const PositionManager::FrameId childFrame /*,
                                        timeLine,
                                        poseRequest
                                        */
                                        );

            PositionManager::Pose getTransform(
                                        PositionManager::TimeUs time,
                                        const PositionManager::FrameId parentFrame,
                                        const PositionManager::FrameId childFrame /*,
                                        timeLine,
                                        poseRequest
                                        */
                                        );

            PositionManager::Pose getTransform(
                                        PositionManager::TimeUs parentTime,
                                        PositionManager::TimeUs childTime,
                                        const PositionManager::FrameId frame /*,
                                        timeLine,
                                        poseRequest
                                        */
                                        );

            int garbageCollector(/*timeLine*/); // Remove transforms that are flagged as useless for more than a given time

            //TODO: add some kind of serialization of timeline 


            bool flagTransform(PositionManager::TimeUs time); //Times might not be exactly the ones the DFPCs ask for. In this case, interpolation will be necessary.
            bool unFlagTransform(PositionManager::TimeUs time);
            StateOfTransform timeLine_find(PositionManager::TimeUs key);

            void printNumberOfElements();


    };
}

#endif
