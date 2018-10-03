#include <infuse_pom_tokamak/Tokamak.hpp>

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


    bool Tokamak::setAbsolutePose(double& x, double& y, double& z, double& phi,std::string& absoluteFrameId)
    {
        //Creation of the Pose to insert

        PositionManager::Pose absPose;
        absPose._tr.transform.cov = 1e-6*base::Matrix6d::Identity();
        absPose._tr.transform.orientation =  base::Quaterniond(base::AngleAxisd(phi,base::Vector3d::UnitZ()));
        absPose._tr.transform.translation(0) = x;
        absPose._tr.transform.translation(1) = y;
        absPose._tr.transform.translation(2) = z;

        absPose._parent = absoluteFrameId;
        absPose._child = robotBodyFrame;

        //absPose._parentTime = PositionManager::TimeManager::now();
        //absPose._childTime = PositionManager::TimeManager::now();
        absPose._parentTime= 0;
        absPose._childTime =  0;

        //Insertion of the Pose inside the timeLine

        return insertNewTransform(absPose);
	
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
    
    bool Tokamak::readInternalFrames(std::string pathToUrdf)
    {
        std::cout << "Reading internal frames" << std::endl;
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
            internalFramesGraph.addFrame(*it);
        }

        for(list<PositionManager::Pose>::iterator it = parser._poses.begin(); it != parser._poses.end(); ++it)
        {
            internalFramesGraph.addTransform(it->_parent, it->_child, it->_tr);
        }

        return true;
    }

    bool Tokamak::addFixedFrame(const PositionManager::Pose& transform)
    {
        fixedFramesGraph.addFrame(transform._child);
        fixedFramesGraph.addTransform(transform._parent,transform._child,transform._tr);
        return true;
    }

    bool Tokamak::getFixedTransform(const PositionManager::FrameId& parent)
    {
        try
        {
            fixedTransform._tr = fixedFramesGraph.getTransform(fixedFrame,parent);
        }
        catch (std::exception const & e)
        {
            std::cout << "[GetFixedFrame failed]: " << e.what() << std::endl;
            return false;
        }
        return true;
    }

    bool Tokamak::convertTransform(PositionManager::Pose & pose)
    {
        //PositionManager::Transform childFrame_robotFrame = internalFramesGraph.getTransform(pose._child,robotBodyFrame);
        PositionManager::Transform childFrame_robotFrame = internalFramesGraph.getTransform(pose._child, robotBodyFrame);
        PositionManager::Transform parentFrame_robotFrame = pose._tr * childFrame_robotFrame;
        pose._tr = parentFrame_robotFrame;
        //pose._parent = fixedFrame;
        //pose._child = robotBodyFrame;
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
            if (childFrame.compare(robotBodyFrame) != 0)
            {
                if (!internalFramesGraph.containsFrame(childFrame))
                {
                    throw e_wrong_childframe;
                }
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
        tfState.pose_fixedFrame_robotFrame._parentTime = transform._parentTime;

        // Time of addition inside the timeLine
        tfState.timeOfAddition = PositionManager::TimeManager::now();

        //std::cout << "Entering insertion function" << std::endl;
        validityCheckInsertion(transform);

        try
        {

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
                tfState.pose_fixedFrame_robotFrame._tr = transform._tr;
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
        lockTimeLine();
        if (timeLine->empty())
        {
            unlockTimeLine();
            throw e_no_publish;
        }
        posePublish = timeLine->last_element()->second.pose_fixedFrame_robotFrame;
        unlockTimeLine();
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
            PositionManager::TimeUs parentTime,
            PositionManager::TimeUs childTime,
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
            if (parent == timeLine->last_element())
            {
                unlockTimeLine();
                throw e_future_transform;
            }

            // Case where child is in the future
            if (child == timeLine->last_element() && childTime !=  child->first)
            {
                unlockTimeLine();
                throw e_future_transform;
            }

            // Case where timestamps asked are not exactly in memory at the moment: this case requires interpolation
            if (childTime != child->first )
            {
                std::cout << "The requested child time necessitates interpolation. It has not been implemented yet. Closest transform in time will be provided" << std::endl;
                if (std::abs(child->first- childTime) > std::abs(std::prev(child)->first - childTime))
                {
                    if (parentTime == childTime)
                    {
                        parentTime = std::prev(child)->first;
                    }
                    childTime = std::prev(child)->first;
                    
                }
                else
                {
                    if (parentTime == childTime)
                    {
                        parentTime = child->first;
                    }
                    childTime = child->first;
                }

            }

            if (parentTime != parent->first && parentTime != childTime )
            {
                std::cout << "The requested parent time necessitates interpolation. It has not been implemented yet. Closest transform in time will be provided" << std::endl;
                if (std::abs(parent->first- parentTime) > std::abs(std::prev(parent)->first - parentTime))
                {
                    if (parentTime == childTime)
                    {
                        parentTime = std::prev(child)->first;
                    }
                    childTime = std::prev(child)->first;
                    
                }
                else
                {
                    if (parentTime == childTime)
                    {
                        parentTime = child->first;
                    }
                    childTime = child->first;
                }
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
                PositionManager::Transform parentFrame_fixedFrame= fixedFramesGraph.getTransform(parentFrame,fixedFrame);
                poseRespond._tr = parentFrame_fixedFrame * timeLine_find(childTime).pose_fixedFrame_robotFrame._tr;
            }

            return poseRespond;
        }

    PositionManager::Pose Tokamak::getTransform(PositionManager::Pose pose/*, timeLine*/)
    {
        return getTransform(pose._parentTime,pose._childTime,pose._parent,pose._child);
    }

    PositionManager::Pose Tokamak::getTransform(
            PositionManager::TimeUs time,
            const PositionManager::FrameId parentFrame,
            const PositionManager::FrameId childFrame /*,
                                                        timeLine
                                                       */)
        {
            return getTransform(time,time,parentFrame,childFrame);
        }

    PositionManager::Pose Tokamak::getTransform(
            PositionManager::TimeUs parentTime,
            PositionManager::TimeUs childTime,
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

    void Tokamak::printNumberOfElements()
    {
        lockTimeLine();
        timeLine->printSize();
        unlockTimeLine();
    }
}
