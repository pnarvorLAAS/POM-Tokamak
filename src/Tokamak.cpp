#include <Tokamak.hpp>

namespace tokamak
{
    Tokamak::Tokamak( )
    {
       runningFrequency = DEFAULT_FREQUENCY;
       secondsKept = DEFAULT_TIME_KEPT;
       bufferSize = DEFAULT_FREQUENCY * DEFAULT_TIME_KEPT;

    }

    Tokamak::Tokamak(int32_t freq): runningFrequency(freq)
    {
        secondsKept = DEFAULT_TIME_KEPT;
        bufferSize = DEFAULT_FREQUENCY * DEFAULT_TIME_KEPT;
    }

    bool Tokamak::insertNewTransform(PositionManager::Pose transform)
    {
        // Build the state of transform
        StateOfTransform tfState;
        tfState.fused = false;
        tfState.flagged = 0;

        // Time of addition inside the timeLine
        tfState.timeofAddition = PositionManager::TimeManager::now();

        // Add time to the frame to make it easier
        std::ostringstream oss;
        oss << transform._childTime;
        tfState.frameId = transform._child + oss.str();

        // Add to timeLine

        lockTimeLine();
        timeLine.insert(std::pair<PositionManager::TimeUs,StateOfTransform>(transform._childTime,tfState));
        unlockTimeline();

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
        PositionManager::Pose robotPose;

        return robotPose;
    }

    PositionManager::Pose Tokamak::getTransform(PositionManager::Pose pose/*, timeLine*/)
    {
        PositionManager::Pose robotPose;
        return robotPose;
    }

    PositionManager::Pose Tokamak::getTransform(
                                PositionManager::TimeUs parentTime, 
                                PositionManager::TimeUs childTime, 
                                PositionManager::FrameId parentFrame, 
                                PositionManager::FrameId childFrame /*,
                                timeLine
                                */)
    {
        PositionManager::Pose robotPose;
        return robotPose;
    }

    PositionManager::Pose Tokamak::getTransform(
                                PositionManager::TimeUs time,
                                PositionManager::FrameId parentFrame,
                                PositionManager::FrameId childFrame /*,
                                timeLine
                                */)
    {
        PositionManager::Pose robotPose;
        return robotPose;
    }

    PositionManager::Pose Tokamak::getTransform(
                                        PositionManager::TimeUs parentTime,
                                        PositionManager::TimeUs childTime,
                                        PositionManager::FrameId frame /*,
                                        timeLine
                                        */)
    {
        PositionManager::Pose robotPose;
        return robotPose;
    }

    void Tokamak::lockTimeline()
    {
        transformAccess.lock();
    }
    
    void Tokamak::unlockTimeline()
    {
        transformAccess.unlock();
    }


}
