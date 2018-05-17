#include <Tokamak.hpp>

namespace tokamak
{
    Tokamak::Tokamak( )
    {
        runningFrequency = DEFAULT_FREQUENCY;
        secondsKept = DEFAULT_TIME_KEPT;
        bufferSize = DEFAULT_FREQUENCY * DEFAULT_TIME_KEPT;
        fixedFrame = DEFAULT_FIXED_FRAME;
        robotBodyFrame = DEFAULT_ROVER_FRAME;
        timeLine = new circularMap<PositionManager::TimeUs,StateOfTransform>(bufferSize);
    }

    Tokamak::Tokamak(int32_t freq): runningFrequency(freq)
    {
        secondsKept = DEFAULT_TIME_KEPT;
        fixedFrame = DEFAULT_FIXED_FRAME;
        robotBodyFrame = DEFAULT_ROVER_FRAME;
        bufferSize = runningFrequency * DEFAULT_TIME_KEPT;
        timeLine = new circularMap<PositionManager::TimeUs,StateOfTransform>(bufferSize);
    }

    Tokamak::Tokamak(int32_t freq, int32_t sec): runningFrequency(freq), secondsKept(sec)
    {
        fixedFrame = DEFAULT_FIXED_FRAME;
        robotBodyFrame = DEFAULT_ROVER_FRAME;
        bufferSize = runningFrequency * secondsKept;
        timeLine = new circularMap<PositionManager::TimeUs,StateOfTransform>(bufferSize);
    }

    Tokamak::Tokamak(int32_t freq, int32_t sec, std::string worldFrame): runningFrequency(freq), secondsKept(sec), fixedFrame(worldFrame)
    {
        robotBodyFrame = DEFAULT_ROVER_FRAME;
        bufferSize = runningFrequency * secondsKept;
        timeLine = new circularMap<PositionManager::TimeUs,StateOfTransform>(bufferSize);
    }

    Tokamak::Tokamak(int32_t freq, int32_t sec, std::string worldFrame, std::string robotFrame): runningFrequency(freq), secondsKept(sec), fixedFrame(worldFrame), robotBodyFrame(robotFrame)
    {
        bufferSize = runningFrequency * secondsKept;
        timeLine = new circularMap<PositionManager::TimeUs,StateOfTransform>(bufferSize);
    }

    Tokamak::~Tokamak()
    {
        clean_up();
    }

    void Tokamak::clean_up()
    {
        delete timeLine;
    }

    void Tokamak::validityCheckInsertion(const PositionManager::Pose transform)
    {

        PositionManager::TimeUs parentTime = transform._parentTime;
        PositionManager::TimeUs childTime = transform._childTime;
        PositionManager::FrameId parentFrame = transform._parent;
        PositionManager::FrameId childFrame = transform._child;

        try
        {
            // Case where transform goes from future to past
            if (parentTime < childTime)
            {
                throw e_inverse_time_transform;
            }
            
            // Case where the transform we want to insert is not from the robot
            if (transform._child != robotBodyFrame)
            {
                throw e_wrong_childframe;
            }
            
            // Case of a "non-transform" : same times and same frames
            if (parentTime == childTime && parentFrame == childFrame)
            {
                throw e_static_transform;
            }

            // TODO add a case to check if parent frame is known (be it robot body frame but also any other world fixed frame)

        }
        catch (std::exception const& e)
        {
            std::cerr << "[VALIDITY CHECK FOR INSERTION FAILED] : " << e.what() << std::endl;
        }
    }

    bool Tokamak::insertNewTransform(PositionManager::Pose transform)
    {
        // Build the state of transform
        StateOfTransform tfState;
        tfState.fused = false;
        tfState.flagged = 0;

        // Time of addition inside the timeLine
        tfState.timeofAddition = PositionManager::TimeManager::now();

        try
        {
            validityCheckInsertion(transform);

            // Case delta pose
            if (transform._parent == transform._child)
            {

                lockTimeLine();
                PositionManager::Transform fixedFrame_robotFrame_parentTime = timeLine->find(transform._parentTime)->second.pose_fixedFrame_robotFrame._tr;
                unlockTimeLine();

                PositionManager::Transform fixedFrame_robotFrame_childTime = fixedFrame_robotFrame_parentTime * transform._tr;
                tfState.pose_fixedFrame_robotFrame._tr = fixedFrame_robotFrame_childTime;
            }
            
            // Case Normal pose
            if (transform._parent == fixedFrame) // The incoming transform "fixedFrame" is the right one
            {
                tfState.pose_fixedFrame_robotFrame = transform;
            }
            else
            {
                //TODO: set the transform in the right frame of reference (if possible). But this
                //will depend on how we add a way for PoM to know which frames exist
            }

            // Add to timeLine

            lockTimeLine();
            timeLine->put(transform._childTime,tfState);
            unlockTimeLine();

        }
        catch (std::exception const& e)
        {
            std::cerr << "[INSERTION OF THE NEW TRANSFORM IN THE TIMELINE FAILED]: " << e.what() << std::endl;
        }

        return true;
    }

    bool Tokamak::insertNewTransforms(std::vector<PositionManager::Pose>& listOfTransforms)
    {
        for (std::vector<PositionManager::Pose>::iterator it = listOfTransforms.begin(); it != listOfTransforms.end(); it++)
        {
            if (!insertNewTransform(*it))
            {
                return false;
            }
        }
        return true;
    }

    PositionManager::Pose Tokamak::getLatestRobotPose()
    {
        lockTimeLine();
        return timeLine->end()->second.pose_fixedFrame_robotFrame;
        unlockTimeLine();
    }
    
    void Tokamak::validityCheckGetTransform(
            const PositionManager::TimeUs parentTime,
            const PositionManager::TimeUs childTime,
            const PositionManager::FrameId parentFrame,
            const PositionManager::FrameId childFrame /*,
                                                  timeLine
                                                 */)
    {
        try
        {
            // Case where transform goes from future to past
            if (parentTime < childTime)
            {
                //TODO Treat this directly with inversion of transform
                throw e_inverse_time_transform;
            }

            if (childFrame != robotBodyFrame)
            {
                throw e_wrong_frames;
            }

            // Come back when parsing of other fixed frame is done
            if (parentFrame != robotBodyFrame && parentFrame != fixedFrame)
            {
                //TODO
                throw e_wrong_frames;
            }

            // Case of a "non-transform" : same times and same frames
            if (parentTime == childTime && parentFrame == childFrame)
            {
                throw e_static_transform;
            }
        }
        catch (std::exception const &e)
        {
            std::cerr << "[VALIDITY CHECK FOR TRANSFORM REQUEST FAILED: ] " << std::endl;
        }
    }

    PositionManager::Pose Tokamak::getTransform(
            const PositionManager::TimeUs parentTime,
            const PositionManager::TimeUs childTime,
            const PositionManager::FrameId parentFrame,
            const PositionManager::FrameId childFrame /*,
                                                  timeLine
                                                 */)
    {
        PositionManager::Pose transformRobot;
        /* Validity checks */
        try
        {
            
            validityCheckGetTransform(parentTime,childTime,parentFrame,childFrame);
            lockTimeLine();
            
            iterator child = timeLine->find_lower(childTime);
            iterator parent = timeLine->find_lower(parentTime);

            // Case where parent is in the future
            if (parent == timeLine->end())
            {
                throw e_future_transform;
            }

            // Case where child is in the future
            if (child == timeLine->end() && childTime !=  child->first)
            {
                throw e_future_transform;
            }

            // Case where timestamps asked are not exactly in memory at the moment: this case requires interpolation
            if (childTime != child->first || parentTime != parent->first)
            {
                //TODO
                throw e_interpolation;
            }

            unlockTimeLine();
        }
        catch (std::exception const& e)
        {
            std::cerr << "[VALIDITY CHECK FAILED]: " << e.what() << std::endl;
        }

        /* Case delta pose */ 
        try
        {

            if (parentFrame == childFrame) // Delta pose
            {
                lockTimeLine();
				PositionManager::Transform robotFrame_fixedFrame_parent = timeLine->find(parentTime)->second.pose_fixedFrame_robotFrame._tr;
				PositionManager::Transform robotFrame_fixedFrame_child = timeLine->find(childTime)->second.pose_fixedFrame_robotFrame._tr;
				PositionManager::Transform robotFrame_parent_child = robotFrame_fixedFrame_parent.inverse() * robotFrame_fixedFrame_child;
				transformRobot._tr = robotFrame_parent_child;
                unlockTimeLine();

            }
        }
        catch (std::exception const& e)
        {
            std::cerr  << "[DELTA POSE CHECK FAILED]: " << e.what() << std::endl;
        }
        return transformRobot;
    }

    PositionManager::Pose Tokamak::getTransform(PositionManager::Pose pose/*, timeLine*/)
    {
        PositionManager::Pose robotPose;
        return robotPose;
    }

    PositionManager::Pose Tokamak::getTransform(
            const PositionManager::TimeUs time,
            const PositionManager::FrameId parentFrame,
            const PositionManager::FrameId childFrame /*,
                                                  timeLine
                                                 */)
    {
        PositionManager::Pose robotPose;
        return robotPose;
    }

    PositionManager::Pose Tokamak::getTransform(
            const PositionManager::TimeUs parentTime,
            const PositionManager::TimeUs childTime,
            const PositionManager::FrameId frame /*,
                                             timeLine
                                            */)
    {
        PositionManager::Pose robotPose;
        return robotPose;
    }

    void Tokamak::lockTimeLine()
    {
        transformAccess.lock();
    }

    void Tokamak::unlockTimeLine()
    {
        transformAccess.unlock();
    }


}
