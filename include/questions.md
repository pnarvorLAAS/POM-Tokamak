## Tokamak has to provide the most recent pose

* **In which frame?** This is why I added the fixedFrame attribute. This will be configurable from the start, and most likely Local Terrain Frame (LTF).
* **How should this be triggered?** Depending on how data is given to tokamak, it might receive several non-fused pose at once. In this case it shall release the latest one. This triggers the need for several things:
    * A circular buffer, as described in redmine. This enables us to allocate memory at init time, but also to not worry about insertion. I don't think there exists out of the box implementations of circular std::map but this should be doable.
    * A pointer to the latest inserted pose
    This way when getLatestRobotPose is called it is only a lookup in the graph with the frameId the "latestpose" pointer is pointing to.

## Tokamak has to gather all localization information, pose or delta-poses

* **Is the difference between delta poses and poses made?** 
    * If not: Each time where going to have to fuse poses, we will have to do the composition.
    * **If yes**: Composition can be done only once, and the graph will be "star-shaped" w.r.t. to the fixedFrame, making it easy to look for a transform in the graph. Makes it also easier to remove "useless transforms".

## Tokamak has to provide a service where a client may request a transform between any set of pair time/frame

* This is where the flagging should happen: if a time is requested it means that it is used and should be saved

## Tokamak has to provide two services to tell him which times to forget and which times to keep
