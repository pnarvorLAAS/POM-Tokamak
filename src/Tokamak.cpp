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

    bool Tokamak::readFixedFrames(std::string pathToUrdf)
    {
        PositionManager::UrdfParser parser;
        try
        {
            if (!parser.parseURDF(pathToUrdf))
            {
                throw e_urdf;
            }
        }

        catch (std::exception const &e)
        {
            std::cout << "[URDF parsing failed]: " << e.what() << std::endl;
            return false;
        }

        for(list<PositionManager::FrameId>::iterator it = parser._frameIds.begin(); it != parser._frameIds.end(); ++it)
        {
            fixedFramesGraph.addFrame(*it);
        }

        for(list<PositionManager::Pose>::iterator it = parser._poses.begin(); it != parser._poses.end(); ++it)
        {
            fixedFramesGraph.addTransform(it->_parent, it->_child, it->_tr);
        }

        return true;
    }

    bool Tokamak::getFixedTransform(const PositionManager::FrameId& parent)
    {
        try
        {
            fixedTransform._tr = fixedFramesGraph.getTransform(parent,robotBodyFrame);
        }
        catch (std::exception const & e)
        {
            std::cout << "[GetFixedFrame failed]: " << e.what() << std::endl;
            return false;
        }
        return true;
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

            if (parentFrame != fixedFrame && parentFrame != robotBodyFrame)
            {
                if (!fixedFramesGraph.containsFrame(parentFrame))
                {
                    throw e_wrong_frames;
                }
            }
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

        // All transforms inserted in the timeline are fixedFrame -> robotBodyFrame
        tfState.pose_fixedFrame_robotFrame._parent = fixedFrame;
        tfState.pose_fixedFrame_robotFrame._child = robotBodyFrame;
        tfState.pose_fixedFrame_robotFrame._childTime = transform._childTime;
        tfState.pose_fixedFrame_robotFrame._parentTime = transform._childTime;

        // Time of addition inside the timeLine
        tfState.timeOfAddition = PositionManager::TimeManager::now();

        std::cout << "Entering insertion function" << std::endl;

        try
        {
            validityCheckInsertion(transform);

            // Case delta pose
            if (transform._parent == transform._child)
            {
                StateOfTransform tfParent = timeLine_find(transform._parentTime);
                PositionManager::Transform fixedFrame_robotFrame_parentTime = tfParent.pose_fixedFrame_robotFrame._tr;

                PositionManager::Transform fixedFrame_robotFrame_childTime = fixedFrame_robotFrame_parentTime * transform._tr;
                tfState.pose_fixedFrame_robotFrame._tr = fixedFrame_robotFrame_childTime;
                tfState.isDeltaPose = true;
                tfState.timeOfParent = transform._parentTime;
            }

            // Case Normal pose
            if (transform._parent == fixedFrame) // The incoming transform "fixedFrame" is the right one
            {
                tfState.pose_fixedFrame_robotFrame = transform;
            }
            else
            {
                PositionManager::Transform fixedFrame_worldFrame = fixedFramesGraph.getTransform(fixedFrame,transform._parent);
                PositionManager::Transform fixedFrame_robotFrame = fixedFrame_worldFrame * transform._tr;
                tfState.pose_fixedFrame_robotFrame._tr = fixedFrame_robotFrame;
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

             // Case where the parentFrame is unknown
             if (parentFrame != robotBodyFrame && parentFrame != fixedFrame)
             {
                 if (!fixedFramesGraph.containsFrame(parentFrame))
                 {
                     throw e_wrong_frames;
                 }
             }

             // Case of a "non-transform" : same times and same frames
             if (parentTime == childTime && parentFrame == childFrame)
             {
                 throw e_static_transform;
             }
             // Case where parent is unknown
             if (parentFrame != fixedFrame)
             {
                 if (!fixedFramesGraph.containsFrame(parentFrame))
                 {
                     throw e_wrong_frames;
                 }
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

            lockTimeLine();

            iterator child = timeLine->find_lower(childTime);
            iterator parent = timeLine->find_lower(parentTime);

            // Case where parent is in the future
            if (parent == timeLine->end())
            {
                unlockTimeLine();
                throw e_future_transform;
            }

            // Case where child is in the future
            if (child == timeLine->end() && childTime !=  child->first)
            {
                unlockTimeLine();
                throw e_future_transform;
            }

            // Case where timestamps asked are not exactly in memory at the moment: this case requires interpolation
            if (childTime != child->first || parentTime != parent->first)
            {
                //TODO
                unlockTimeLine();
                throw e_interpolation;
            }

            unlockTimeLine();

            /* Case delta pose */ 

            if (parentFrame == childFrame) // Delta pose
            {
                PositionManager::Transform robotFrame_fixedFrame_parent = timeLine_find(parentTime).pose_fixedFrame_robotFrame._tr;
                PositionManager::Transform robotFrame_fixedFrame_child = timeLine_find(childTime).pose_fixedFrame_robotFrame._tr;
                PositionManager::Transform robotFrame_parent_child = robotFrame_fixedFrame_parent.inverse() * robotFrame_fixedFrame_child;
                poseRespond._tr = robotFrame_parent_child;
            }

            /* Case normal pose */
            if (parentFrame == fixedFrame)
            {
                poseRespond._tr = timeLine_find(childTime).pose_fixedFrame_robotFrame._tr;
            }
            else 
            {
                PositionManager::Transform parentFrame_robotFrame = fixedFramesGraph.getTransform(parentFrame,robotBodyFrame);
                poseRespond._tr = parentFrame_robotFrame * timeLine_find(childTime).pose_fixedFrame_robotFrame._tr;
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
