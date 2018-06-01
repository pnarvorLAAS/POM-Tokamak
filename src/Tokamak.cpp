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

    bool readFixedFrame(std::string pathToUrdf)
    {
        UrdfParser parser;
        if (!parser.parseURDF(pathToUrdf)
        {
            std::cout << "Can't parse File" << std::endl;
        }


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

                PositionManager::Transform fixedFrame_robotFrame_parentTime = timeLine_find(transform._parentTime).pose_fixedFrame_robotFrame._tr;

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
                std::cout << "Parent Frame is not right" << std::endl;
                throw e_wrong_frames;
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
        try
        {
            lockTimeLine();
            if (timeLine->empty())
            {
                throw e_no_publish;
            }
            posePublish = timeLine->end()->second.pose_fixedFrame_robotFrame;
            unlockTimeLine();
        }
        catch (std::exception const &e)
        {
            cerr << "[CANNOT PUBLISH POSE: ]" << e.what() << std::endl;
        }
        return posePublish;
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
            if (parentTime > childTime)
            {
                //TODO Treat this directly with inversion of transform
                std::cout << "Child time is: " << childTime << std::endl;
                std::cout << "Parent time is: " << parentTime << std::endl;
                throw e_inverse_time_transform;
            }

            if (childFrame != robotBodyFrame)
            {
                std::cout << "Child frame is :" << childFrame;
                std::cout << " while robot frame is : " << robotBodyFrame << std::endl;
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
            std::cerr << "[VALIDITY CHECK FOR TRANSFORM REQUEST FAILED: ] " << e.what() <<  std::endl;
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
        poseRespond._parent = parentFrame;
        poseRespond._child = childFrame;
        poseRespond._parentTime = parentTime;
        poseRespond._childTime = childTime;
        validityCheckGetTransform(parentTime,childTime,parentFrame,childFrame);
        /* Validity checks */
        try
        {
            
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
				PositionManager::Transform robotFrame_fixedFrame_parent = timeLine_find(parentTime).pose_fixedFrame_robotFrame._tr;
				PositionManager::Transform robotFrame_fixedFrame_child = timeLine_find(childTime).pose_fixedFrame_robotFrame._tr;
				PositionManager::Transform robotFrame_parent_child = robotFrame_fixedFrame_parent.inverse() * robotFrame_fixedFrame_child;
				poseRespond._tr = robotFrame_parent_child;
            }
        }
        catch (std::exception const& e)
        {
            std::cerr  << "[DELTA POSE CHECK FAILED]: " << e.what() << std::endl;
        }

        /* Case normal pose */
        try
        {
            if (parentFrame == fixedFrame)
            {
                poseRespond._tr = timeLine_find(childTime).pose_fixedFrame_robotFrame._tr;
            }
        }
        catch (std::exception const &e)
        {
            std::cerr << "[POSE REQUEST FAILED]: " << e.what() << std::endl;
        }
        return poseRespond;
    }

    PositionManager::Pose Tokamak::getTransform(PositionManager::Pose pose/*, timeLine*/)
    {
        return getTransform(pose._parentTime,pose._childTime,pose._parent,pose._child);
    }

    PositionManager::Pose Tokamak::getTransform(
            const PositionManager::TimeUs time,
            const PositionManager::FrameId parentFrame,
            const PositionManager::FrameId childFrame /*,
                                                  timeLine
                                                 */)
    {
        return getTransform(time,time,parentFrame,childFrame);
    }

    PositionManager::Pose Tokamak::getTransform(
            const PositionManager::TimeUs parentTime,
            const PositionManager::TimeUs childTime,
            const PositionManager::FrameId frame /*,
                                             timeLine
                                            */)
    {
        return getTransform(parentTime,childTime,frame,frame);
    }

    void Tokamak::lockTimeLine()
    {
        transformAccess.lock();
    }

    void Tokamak::unlockTimeLine()
    {
        transformAccess.unlock();
    }

    StateOfTransform Tokamak::timeLine_find(PositionManager::TimeUs key)
    {
        lockTimeLine();
        StateOfTransform it = timeLine->find(key)->second;
        unlockTimeLine();
        return it;
    }
}
