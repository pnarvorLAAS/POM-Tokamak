#include <infuse_pom_tokamak/Tokamak.hpp>
#include <infuse_pom_tokamak/factors/LTFMeasure.hpp>
#include <infuse_pom_tokamak/factors/RelativeMeasure.hpp>
#include <infuse_pom_tokamak/factors/Unary.hpp>

namespace tokamak
{
    Tokamak::Tokamak( ) : fixedFrame(DEFAULT_FIXED_FRAME), robotBodyFrame(DEFAULT_ROVER_FRAME)
    {
        init();
    }

    Tokamak::Tokamak(std::string worldFrame, std::string robotFrame): fixedFrame(worldFrame), robotBodyFrame(robotFrame)
    {
        init();
    }

    void Tokamak::init()
    {
        int buffersize = 10000;
        timeLine = std::make_shared<circularMap<PositionManager::TimeUs,StateOfTransform>>(buffersize);
        poseGraph = std::make_shared<gtsam::NonlinearFactorGraph>();
    }

    Tokamak::~Tokamak(){}



    bool Tokamak::setAbsolutePose(double& x, double& y, double& z, double& phi,std::string& absoluteFrameId)
    {
        // Create new symbol for fixed frame

        fixedFramesSymbol.insert(std::pair<std::string,gtsam::Symbol>("LocalTerrainFrame",gtsam::Symbol('x',fixedFramesCounter)));
        fixedFramesCounter++;

        // Get location of LTF in World frame

        PositionManager::Transform absolute_world = fixedFramesGraph.getTransform(absoluteFrameId, fixedFrame);

        // Create Pose2 

        Pose2 LTFMeasure(x - absolute_world.transform.translation(0), y - absolute_world.transform.translation(1), phi);

        // Add to graph
        poseGraph->add(LTFMeasure::Prior(fixedFramesSymbol["LocalTerrainFrame"].key(), LTFMeasure, noiseModel::Diagonal::Variances((Vector(3) << 1e-6,1e-6,0.1).finished())));

        // Keep position of LTF in memory
    
        PositionManager::Transform absPose;
        absPose.transform.cov = 1e-6*base::Matrix6d::Identity();
        absPose.transform.orientation =  base::Quaterniond(base::AngleAxisd(phi,base::Vector3d::UnitZ()));
        absPose.transform.translation(0) = x - absolute_world.transform.translation(0);
        absPose.transform.translation(1) = y - absolute_world.transform.translation(1);
        absPose.transform.translation(2) = z - absolute_world.transform.translation(2);
        
        LTFaltitude  = z - absolute_world.transform.translation(2);

        fixedFramesGraph.addTransform(fixedFrame, "LocalTerrainFrame", absPose );

        //absPose._parent = absoluteFrameId;
        //absPose._child = robotBodyFrame;

        ////absPose._parentTime = PositionManager::TimeManager::now();
        ////absPose._childTime = PositionManager::TimeManager::now();
        //absPose._parentTime= 0;
        //absPose._childTime =  0;

        ////Insertion of the Pose inside the timeLine

        //return insertNewTransform(absPose);
	
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
        std::string internalString = childFrame_robotFrame.toString();
        std::cout << "Getting Robot->GPS : " << std::endl;
        std::cout << internalString << std::endl;

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
            //Check Pose type

            POSE_TYPE poseType = getPoseType(transform);
            if (transform._producerId.compare(poseBaseline))
            {
                insertBaseline(transform, poseType);
            }
            addGraphFactor(transform,poseType);

        }
        catch (std::exception const& e)
        {
            std::cerr << "[INSERTION OF THE NEW TRANSFORM IN THE TIMELINE FAILED]: " << e.what() << std::endl;
        }

        return true;
    }

    POSE_TYPE Tokamak::getPoseType(PositionManager::Pose& transform)
    {
        int sum = 0;
        for (int i = 0; i < 7; i++)
        {
            sum += transform._dataEstimated[i];
        }

        if (sum == 3) // 3 values estimated
        {
            return TRANSLATION;
        }
        else if (sum == 4) // 4 values estimated
        {
            return ORIENTATION;
        }
        else if (sum == 7) // All values estimated
        {
            return FULL_POSE;
        }
        else
        {
            throw e_unknown_pose_type;
        }

    }

    bool Tokamak::insertBaseline(PositionManager::Pose& transform, POSE_TYPE pt)
    {

        /*TODO: 
            - check locking of timeline for all of this.
            - check variance computation 
            - probably refactor the whole stuff
        */

        lockTimeLine();

        timeIterator it = timeLine->find(transform._childTime);

        if (it != timeLine->end())
        {
            if (pt == TRANSLATION)
            {
                PositionManager::Transform poseOfChild;

                if (it->second.timeOfParent != 0) // Not the first input to timeLine
                {
                    //find parent
                    timeIterator parent = timeLine->find(transform._parentTime);

                    if (checkBaselineTranslation(transform))
                    {
                        //Compose with new pose
                        poseOfChild = parent->second.pose_fixedFrame_robotFrame._tr * transform._tr;
                    }
                    else
                    {
                        // We did not move, dont change translation
                        poseOfChild = parent->second.pose_fixedFrame_robotFrame._tr;
                    }
                }
                else
                {
                    poseOfChild = transform._tr;
                }

                // Update translation
                it->second.pose_fixedFrame_robotFrame._tr.transform.translation = poseOfChild.transform.translation;

                //Set parent time
                if (it->second.timeOfParent != 0)
                {
                    it->second.timeOfParent = transform._parentTime;
                }
                
                //Set variance
                it->second.pose_fixedFrame_robotFrame._tr.transform.cov.block<3,3>(0,0) = poseOfChild.transform.cov.block<3,3>(0,0);

            }

            else
            {
                it->second.pose_fixedFrame_robotFrame._tr.transform.orientation = transform._tr.transform.orientation;

                //Set variance
                it->second.pose_fixedFrame_robotFrame._tr.transform.cov.block<3,3>(3,3) = transform._tr.transform.cov.block<3,3>(3,3);
            }

            it->second.isComplete = true;
        }
        else
        {

            // Create new addition to the timeline
            StateOfTransform tf;

            tf.timeOfAddition = PositionManager::TimeManager::now();

            bool firstInput = timeLine->empty();
            if (firstInput)
            {
                tf.timeOfParent = 0;
            }
            tf.isComplete = false;
            tf.pose_fixedFrame_robotFrame = transform; //Easy when it's the rotation
            tf.graphKey = transform._childTime;

            if (pt == TRANSLATION)
            {
                PositionManager::Transform poseOfChild;
                if (!firstInput)
                {
                    //find parent
                    tf.timeOfParent=  transform._parentTime;
                    timeIterator parent = timeLine->find(transform._parentTime);
                    //Compose with new pose
                    if (checkBaselineTranslation(transform))
                    {
                        poseOfChild = parent->second.pose_fixedFrame_robotFrame._tr * transform._tr;
                    }
                    else
                    {
                        poseOfChild = parent->second.pose_fixedFrame_robotFrame._tr;
                    }
                }
                else
                {
                    // First input is the direct transform
                    poseOfChild = transform._tr;
                }

                tf.pose_fixedFrame_robotFrame._tr = poseOfChild;
                tf.pose_fixedFrame_robotFrame._parent = fixedFrame;
            }

            timeLine->put(transform._childTime,tf);
        }
        unlockTimeLine();
        return true;
    }

    bool Tokamak::checkBaselineTranslation(PositionManager::Pose &transform)
    {
        // Under a speed of 0.01m/s we consider it a zero translation
        double dx = transform._tr.transform.translation(0);
        double dy = transform._tr.transform.translation(1);
        double dz = transform._tr.transform.translation(2);

        double distance = sqrt(dx*dx+dy*dy+dz*dz);

        if (distance < 0.01 / (transform._childTime - transform._parentTime))
        {
            return false;
        }
        return true;

    }

    bool Tokamak::addGraphFactor(PositionManager::Pose& transform, POSE_TYPE pt)
    {
        // Find time closest to child in timeline
        timeIterator titChild = timeLine->find_closest(transform._childTime);
        if  (titChild == timeLine->end())
        {
            throw e_pose_insertion_future;
        }
            
        gtsam::Key childKey = titChild->second.graphKey;

        // Case where we have LTF-> RBF transform
        if (transform._parent == fixedFrame || titChild->second.pose_fixedFrame_robotFrame._parentTime == 0)
        {
            // Get parent key (LTF)
            gtsam::Key parentKey = fixedFramesSymbol[fixedFrame].key();

            // Case Translation
            if (pt == TRANSLATION)
            {
                //Recreate measure
                gtsam::Point3 measure(transform._tr.transform.translation);
                LTFMeasure::TranslationFactor transToAdd(parentKey,childKey,measure,LTFaltitude,gtsam::noiseModel::Diagonal::Variances((Vector(3) << transform._tr.transform.cov(0,0), transform._tr.transform.cov(0,0), transform._tr.transform.cov(0,0)).finished()));
                
                //Add to graph
                poseGraph->add(transToAdd);
            }

            //Case rotation
            else if (pt == ORIENTATION)
            {
                gtsam::Rot3 measure(transform._tr.transform.orientation);
                LTFMeasure::RotationFactor rotToAdd(parentKey,childKey,measure,noiseModel::Diagonal::Variances(transform._tr.transform.cov.diagonal().tail(3)));
                poseGraph->add(rotToAdd);
            }

            // Case full pose
            else if (pt == FULL_POSE)
            {
                gtsam::Rot3 rotation(transform._tr.transform.orientation);
                gtsam::Point3 translation(transform._tr.transform.translation);
                gtsam::Pose3 measure(rotation,translation);
                LTFMeasure::FullPoseFactor transToAdd(parentKey,childKey,measure,LTFaltitude,noiseModel::Diagonal::Variances(transform._tr.transform.cov.diagonal()));
                poseGraph->add(transToAdd);
            }

        }

        //Case transform is a delta pose
        /* TODO:
            - Care when timeline is locked
        */
    
        if (transform._parent == transform._child)
        {
            timeIterator titParent = timeLine->find_closest(transform._parentTime);

            if (titParent == timeLine->end() || titChild == timeLine->end())
            {
                throw e_wrong_delta;
            }
            
            gtsam::Key parentKey = titParent->second.graphKey;

            // Case where pose is a translation
            if (pt == TRANSLATION)
            {
                if (checkBaselineTranslation(transform))
                {
                    //Recreate measure
                    gtsam::Point3 measure(transform._tr.transform.translation);
                    RelativeMeasure::TranslationFactor transToAdd(parentKey,childKey,measure,gtsam::noiseModel::Diagonal::Variances((Vector(3) << transform._tr.transform.cov(0,0), transform._tr.transform.cov(0,0), transform._tr.transform.cov(0,0)).finished()));
                    
                    //Add to graph
                    poseGraph->add(transToAdd);
                }
                else 
                {
                    // No translation
                    gtsam::Point3 measure(0,0,0);
                    RelativeMeasure::TranslationFactor transToAdd(parentKey,childKey,measure,noiseModel::Diagonal::Variances((Vector(3) << 1e-6, 1e-6, 1e-6).finished()));
                    // Add to graph
                    poseGraph->add(transToAdd);
                }
            }
            else if (pt == FULL_POSE)
            {
                //Create pose
                gtsam::Point3 translation(transform._tr.transform.translation);
                gtsam::Rot3 rotation(transform._tr.transform.orientation);
                gtsam::Pose3 measure(rotation,translation);
                RelativeMeasure::FullPoseFactor poseToAdd(parentKey,childKey,measure,noiseModel::Diagonal::Variances(transform._tr.transform.cov.diagonal()));
                poseGraph->add(poseToAdd);

            }
            else
            {
                throw e_pose_not_recognized;
            }

            return true;
        }
        else
        {
            PositionManager::Transform parent_graphFrame = fixedFramesGraph.getTransform(transform._parent,graphFrame);
            if (transform._parent != robotBodyFrame)
            {
                PositionManager::Transform robot_sensor = fixedFramesGraph.getTransform(robotBodyFrame,transform._child);
                Point3 t_robot_sensor(robot_sensor.transform.translation);
                Rot3 rot_parent_graph(parent_graphFrame.transform.orientation);
                Point3 t_parent_graph(parent_graphFrame.transform.translation);
                Pose3 parent_graph(rot_parent_graph,t_parent_graph);

                Point3 measure(transform._tr.transform.translation);
                Unary::GPSFactor transToAdd(childKey, measure, t_robot_sensor, parent_graph, noiseModel::Diagonal::Variances(transform._tr.transform.cov.diagonal().head(3)));
                poseGraph->add(transToAdd);

            }
            else
            {
                throw e_pose_not_recognized;
            }
            return true;
        }

        

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
