// -*- coding: utf-8 -*-
// Copyright (C) 2012-2015 MUJIN Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/** \file visionmanagerimpl.h
    \brief Public headers of MujinVisionManager.
 */
#ifndef MUJIN_VISION_MANAGER_H
#define MUJIN_VISION_MANAGER_H

#include <mujincontrollerclient/binpickingtask.h>
#include <mujincontrollerclient/mujinzmq.hpp>
#include "mujinvision/visionparameters.h"
#include "mujinvision/imagesubscribermanager.h"
#include "mujinvision/detectormanager.h"

#include <queue>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

namespace mujinvision {

using namespace mujinclient;
using namespace mujinzmq;

class MUJINVISION_API MujinVisionManager
{
public:
    /** \brief sets up vision manager
     */    
    MujinVisionManager(ImageSubscriberManagerPtr imagesubscribermanager, DetectorManagerPtr detectormanager, const unsigned int statusport, const unsigned int commandport, const unsigned int configport, const std::string& configdir);
    virtual ~MujinVisionManager();

    virtual void Destroy();

    /// \brief vision server parameters
    struct MUJINVISION_API VisionServerParameters : public ParametersBase
    {
        VisionServerParameters(const ptree& pt)
        {
            _pt = pt;
            maxPositionError = pt.get<double>("max_position_error");
            clearRadius = pt.get<double>("clear_radius");
            timeToRemember = pt.get<unsigned int>("time_to_remember");
            timeToIgnore = pt.get<unsigned int>("time_to_ignore");
            numDetectionsToKeep = pt.get<unsigned int>("num_detections_to_keep");
        }

        virtual ~VisionServerParameters() {
        }


        double maxPositionError; ///< in meter, max position error to consider detections the same
        double clearRadius; ///< in meter, clear detection results within the radius of the last picked locations
        unsigned int timeToRemember; ///< in millisecond, time to keep detection result before forgetting it
        unsigned int timeToIgnore; ///< in millisecond, time to ignore detection result after picking in the region
        unsigned int numDetectionsToKeep; ///< number of detection history to keep

        std::string GetJsonString()
        {
            std::stringstream ss;
            ss << "{";
            ss << ParametersBase::GetJsonString("max_position_error") << ": " << maxPositionError << ",";
            ss << ParametersBase::GetJsonString("clear_radius") << ": " << clearRadius << ",";
            ss << ParametersBase::GetJsonString("time_to_remember") << ": " << timeToRemember << ",";
            ss << ParametersBase::GetJsonString("time_to_ignore") << ": " << timeToIgnore << ",";
            ss << ParametersBase::GetJsonString("num_detections_to_keep") << ": " << numDetectionsToKeep;
            ss << "}";
            return ss.str();
        }

        ptree GetPropertyTree()
        {
            if (_pt.empty()) {
                _pt.put<double>("max_position_error", maxPositionError);
                _pt.put<double>("clear_radius", clearRadius);
                _pt.put<unsigned int>("time_to_remember", timeToRemember);
                _pt.put<unsigned int>("time_to_ignore", timeToIgnore);
                _pt.put<unsigned int>("num_detections_to_keep", numDetectionsToKeep);
            }
            return _pt;
        }
    };
    typedef boost::shared_ptr<VisionServerParameters> VisionServerParametersPtr;
    typedef boost::shared_ptr<VisionServerParameters const> VisionServerParametersConstPtr;
    typedef boost::weak_ptr<VisionServerParameters> VisionServerParametersWeakPtr;

    class MUJINVISION_API StatusPublisher : public ZmqPublisher {
public:
        StatusPublisher(boost::shared_ptr<zmq::context_t> context, const unsigned int port) : ZmqPublisher(port)
        {
            _InitializeSocket(context);
        }

        virtual ~StatusPublisher()
        {
            _DestroySocket();
        }
    };
    typedef boost::shared_ptr<StatusPublisher> StatusPublisherPtr;
    typedef boost::shared_ptr<StatusPublisher const> StatusPublisherConstPtr;
    typedef boost::weak_ptr<StatusPublisher> StatusPublisherWeakPtr;

    class MUJINVISION_API CommandServer : public ZmqServer {
public:
        CommandServer(boost::shared_ptr<zmq::context_t> context, const unsigned int port) : ZmqServer(port)
        {
            _InitializeSocket(context);
        }

        virtual ~CommandServer()
        {
            _DestroySocket();
        }
    };
    typedef boost::shared_ptr<CommandServer> CommandServerPtr;
    typedef boost::shared_ptr<CommandServer const> CommandServerConstPtr;
    typedef boost::weak_ptr<CommandServer> CommandServerWeakPtr;


    /** \brief preparation for the detection process
        - process configurations
        - subscribe to image streams
        - connect to the mujin controller
        - initialize detection
     */
    virtual void Initialize(const std::string& visionmanagerconfigname,
                            const std::string& detectorconfigname,
                            const std::string& imagesubscriberconfigname,
                            const std::string& controllerIp,
                            const unsigned int controllerPort,
                            const std::string& controllerUsernamePass,
                            const std::string& robotControllerUri,
                            const unsigned int binpickingTaskZmqPort,
                            const unsigned int binpickingTaskHeartbeatPort,
                            const double binpickingTaskHeartbeatTimeout,
                            const std::string& binpickingTaskScenePk,
                            const std::string& robotname,
                            const std::string& targetname,
                            const std::vector<std::string>& streamerUris,
                            const std::string& tasktype="binpicking"
                            );

    /** \brief Detects objects in specified region with specified cameras
        \param regionname name of the region
        \param cameranames names of the cameras
        \param detectedobjects detection results in meters in world frame
        \param ignoreocclusion whether to skip occlusion check
        \param maxage max time difference in ms allowed between the current time and the timestamp of image used for detection, 0 means infinity
     */
    virtual void DetectObjects(const std::string& regionname,
                               const std::vector<std::string>& cameranames,
                               std::vector<DetectedObjectPtr>& detectedobjectsworld,
                               bool& iscontainerempty,
                               const bool ignoreocclusion=false,
                               const unsigned int maxage=0);

    /** \brief starts detection thread to continuously detect objects and sends detection results to mujin controller
     */
    virtual void StartDetectionLoop(const std::string& regionname,
                                    const std::vector<std::string>& cameranames,
                                    const double voxelsize=0.01,
                                    const double pointsize=0.005,
                                    const bool ignoreocclusion=false,
                                    const unsigned int maxage=0,
                                    const std::string& obstaclename="__dynamicobstacle__");

    virtual void StopDetectionLoop();

    /** \brief Updates the point cloud obstacle and sends it to mujin controller
        \param regionname name of the region where the detection happens
        \param cameranames names of the cameras used for detection
        \param detectedobjects detection result in meters in  world frame
        \param voxelsize size of the voxel grid in meters used for simplifying the cloud
        \param pointsize size of the point in meters to be sent to the mujin controller
     */
    virtual void SendPointCloudObstacleToController(const std::string& regionname,
                                                    const std::vector<std::string>& cameranames,
                                                    const std::vector<DetectedObjectPtr>& detectedobjectsworld,
                                                    const double voxelsize=0.01,
                                                    const double pointsize=0.005,
                                                    const std::string obstaclename="__dynamicobstacle__");

    /** \brief Visualizes the raw camera point clouds on mujin controller
     */
    virtual void VisualizePointCloudOnController(const std::string& regionname,
                                                 const std::vector<std::string>& cameranames,
                                                 const double pointsize=0.005,
                                                 const bool ignoreocclusion=false,
                                                 const unsigned int maxage=0);

    /** \brief Clears visualization made by VisualizePointCloudOnController on mujin controller.
     */
    virtual void ClearVisualizationOnController();

    /** \brief Detects the transform of region
        \param regionname name of the region where the detection happens
        \param cameranames names of the cameras used for detection
        \param regiontransform detected new transform of the region
        \param ignoreocclusion whether to skip occlusion check
        \param maxage max time difference in ms allowed between the current time and the timestamp of image used for detection, 0 means infinity
     */
    virtual void DetectRegionTransform(const std::string& regionname, const std::vector<std::string>& cameranames, mujinvision::Transform& regiontransform, const bool ignoreocclusion, const unsigned int maxage=0);

    /** \brief Saves a snapshot for each sensor mapped to the region. If detection was called before, snapshots of the images used for the last detection will be saved. Images are saved to the visionmanager application directory.
     */
    virtual void SaveSnapshot(const std::string& regionname, const bool ignoreocclusion=true, const unsigned int maxage=0);

    /** \brief Updates the locally maintained list of the detected objects
        \param detectedobjectsworld detection result in world frame
        \param sendtocontroller whether to send the list to mujin controller
     */
    virtual void UpdateDetectedObjects(const std::vector<DetectedObjectPtr>& detectobjectsworld, const bool iscontainerempty, const bool sendtocontroller=false);

    /** \brief Updates the region info from the mujin controller
        - updates position of the region
        - updates globalroi3d of the region
     */
    virtual void SyncRegion(const std::string& regionname);

    /** \brief Updates info about the cameras associated with the region from the mujin controller. If no cameraname is provided, then update all cameras mapped to the region.
        - updates positions of the cameras
     */
    virtual void SyncCameras(const std::string& regionname,
                              const std::vector<std::string>& cameranames);

    /** \brief Gets id of the camera from name
     */
    virtual void GetCameraId(const std::string& cameraname, std::string& cameraid);

    /** \brief Shuts down visionmanager
     */
    void Shutdown();

    /** \brief Whether visionmanager is shut down
     */
    bool IsShutdown();

    virtual void GetLatestDetectedObjects(std::vector<DetectedObjectPtr>& detectobjectsworld);

    virtual void GetConfig(const std::string& type, std::string& config);
    virtual void SaveConfig(const std::string& type, const std::string& configname, const std::string& config="");

    void GetStatusPort(unsigned int &port) {
        port = _statusport;
    }

    void GetConfigPort(unsigned int &port) {
        port = _configport;
    }

    bool IsDetectionRunning();

    /** \brief Registers a command.
     */
    typedef boost::function<bool (MujinVisionManager*, const ptree&, std::ostream&)> CustomCommandFn;

    class MUJINVISION_API CustomCommand
    {
public:
        CustomCommand() {
        }
        CustomCommand(CustomCommandFn newfn) : fn(newfn) {
        }
        CustomCommandFn fn; ///< command function to run
    };

    /** \brief Registers a command and its help string. <b>[multi-thread safe]</b>
        \param cmdname - command name, converted to lower case
        \param fncmd function to execute for the command
    */
    virtual void RegisterCustomCommand(const std::string& cmdname, CustomCommandFn fncmd);

    /** \brief Unregisters the command. <b>[multi-thread safe]</b>
     */
    virtual void UnregisterCommand(const std::string& cmdname);

private:

    enum ManagerStatus {
        MS_Lost=0,
        MS_Pending=1,
        MS_Active=2,
        MS_Preempting=3,
        MS_Preempted=4,
        MS_Succeeded=5,
        MS_Paused=6,
        MS_Aborted=7,
    };

    inline const char* _GetManagerStatusString(ManagerStatus status) {
        switch (status) {
        case MS_Lost: return "Lost";
        case MS_Pending: return "Pending";
        case MS_Active: return "Active";
        case MS_Preempting: return "Preempting";
        case MS_Preempted: return "Preempted";
        case MS_Succeeded: return "Succeeded";
        case MS_Paused: return "Paused";
        case MS_Aborted: return "Aborted";
        }
        return "";
    };

    struct DetectedInfo {
        unsigned long long timestamp; ///< timestamp of part's last detection.
        unsigned int count; ///< count is the number of detections of part.
        Vector meanPosition; ///< meanPosition is the mean position of part's history of detected positions.
        Vector meanRotation; ///< meanRotation is the mean rotation of part's history of detected rotations.
        std::vector<Vector> positions; ///< positions is part's history of detected positions. positions[i] is the i'th history of part's XYZ position.
        std::vector<Vector> rotations; ///< rotations is part's history of detected rotations. rotation[i] is the i'th history of part's rotation.
        std::vector<std::string> confidences; ///< confidences is part's history of detection confidence. confidences[i] is the i'th history of part's confidence.
    };

    void _DeInitialize();
    
    void _SetStatusMessage(const std::string& msg);
    void _SetStatus(ManagerStatus status, const std::string& msg="", const bool allowInterrupt=true);

    /** \brief Executes command in json string. Returns result in json string.
     */
    void _ExecuteConfigurationCommand(const ptree& command_pt, std::stringstream& result_ss);
    void _ExecuteUserCommand(const ptree& command_pt, std::stringstream& result_ss);

    /** \brief Receives and executes commands from the user and sends results back.
     */
    void _CommandThread(const unsigned int port);
    void _StartCommandServer(const unsigned int port);
    void _StopCommandServer(const unsigned int port);
    void _StartCommandThread(const unsigned int port);
    void _StopCommandThread(const unsigned int port);

    /** \brief Publishes status periodically. When there are more status messages on the status queue, then publish all of them at once, otherwise, publish the last status message.
     */
    void _StatusThread(const unsigned int port, const unsigned int ms);
    void _StartStatusThread(const unsigned int port, const unsigned int ms=100);
    void _StopStatusThread();
    void _StartStatusPublisher(const unsigned int port);
    void _PublishStopStatus();

    void _DetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const std::string& obstaclename);
    void _StartDetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const std::string& obstaclename);
    void _StopDetectionThread();

    /** \brief Updates the environment state on mujin controller with the pointcloud obstacle and detected objects.
        \param regionname name of the region of which the pointcloud obstacle represents
        \param cameranames names of the cameras to be used to capture the pointcloud obstacle
        \param detectobjectsworld detected objects in world frame
        \param voxelsize size of the voxel grid in meters used for simplifying the cloud
        \param pointsize size of the point in meters to be sent to the mujin controller
     */
    void _UpdateEnvironmentThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const std::string& obstaclename="__dynamicobstacle__", const unsigned int waitinterval=50);
    void _StartUpdateEnvironmentThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const std::string& obstaclename, const unsigned int waitinterval=50);
    void _StopUpdateEnvironmentThread();

    /** \brief Gets transform of the instobject in meters.
     */
    mujinvision::Transform _GetTransform(const std::string& instobjname);

    /** \brief Updates the world transform of region from the mujin controller.
     */
    void _SyncRegion(const std::string& regionname);

    /** \brief Updates the world transform of camera from the mujin controller.
     */
    void _SyncCamera(const std::string& regionname, const std::string& cameraname);

    /** \brief Gets color images (uncropped) from image subscriber manager.
        \param maxage in milliseconds, if non-0, only images that are less than maxage ms will be returned
        \param waitinterval in milliseconds, if failed to get image, time to wait before the next try
        \return number of images fetched
     */
    unsigned int _GetColorImages(const std::string& regionname, const std::vector<std::string>& cameranames, std::vector<ColorImagePtr>& images, const bool ignoreocclusion=false, const unsigned int maxage=0/*ms*/, const unsigned int waitinterval=50);

    /** \brief Gets depth images (uncropped) from image subscriber manager.
        \param maxage in milliseconds, if non-0, only images that are less than maxage ms will be returned
        \param waitinterval in milliseconds, if failed to get image, time to wait before the next try
        \return number of images fetched
     */
    unsigned int _GetDepthImages(const std::string& regionname, const std::vector<std::string>& cameranames, std::vector<DepthImagePtr>& images, const bool ignoreocclusion=false, const unsigned int maxage=0/*ms*/, const unsigned int waitinterval=50);

    /** \brief Converts a vector detectedobjects to "objects": [detectedobject->GetJsonString()]
     */
    std::string _GetJsonString(const std::vector<DetectedObjectPtr>& detectedobjects);

    /** \brief Filters cameranames with region, so that only cameras mapped to the region are returned. If no cameranames is specified, then return all cameras mapped to the region.
     */
    std::vector<std::string> _GetCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames);

    /** \brief This function wraps _GetCameraNames so that it returns only color cameras.
     */
    std::vector<std::string> _GetColorCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames);

    /** \brief This function wraps _GetCameraNames so that it returns only depth cameras.
     */
    std::vector<std::string> _GetDepthCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames);

    /** \brief Sends detected object list to mujin controller.
        \param detectobjectsworld detected objects in world frame
     */
    void _SendDetectedObjectsToController(const std::vector<DetectedObjectPtr>& detectedobjectsworld, const bool iscontainerempty);

    /** \brief Converts mujinclient::Transform to Transform.
     */
    Transform _GetTransform(const mujinclient::Transform& t);

    /** \brief Get string representation of transform.
     */
    std::string _GetString(const Transform& transform);

    /** \brief Gets status json string.
     */
    std::string _GetStatusJsonString(const unsigned long long timestamp, const std::string& status, const std::string& message);

    /** \brief Breaks the camera name string into camerabodyname and sensorname.
        \param cameraname string such as 'camerabodyname/sensorname'
        \param camerabodyname name of the camera kinbody
        \param sensorname name of the attached sensor
     */
    void _ParseCameraName(const std::string& cameraname, std::string& camerabodyname, std::string& sensorname);

    std::string _GetConfigFileName(const std::string& type, const std::string& configname);
    void _LoadConfig(const std::string& filename, std::string& content);

    unsigned int _statusport, _commandport, _configport;
    std::string _configdir;
    std::string _detectorconfig, _imagesubscriberconfig;
    
    unsigned int _binpickingTaskZmqPort;
    unsigned int _binpickingTaskHeartbeatPort;
    double _binpickingTaskHeartbeatTimeout;
    std::string _binpickingTaskScenePk;
    std::string  _robotControllerUri;
    std::string _tasktype;

    boost::shared_ptr<zmq::context_t> _zmqcontext;

    ControllerClientPtr _pControllerClient;
    SceneResourcePtr _pSceneResource;
    VisionServerParametersPtr _pVisionServerParameters;
    BinPickingTaskResourcePtr _pBinpickingTask;

    std::queue<ManagerStatus> _statusQueue;
    std::queue<std::string> _messageQueue;
    std::queue<unsigned long long> _timestampQueue;
    boost::mutex _mutexStatusQueue; ///< protects _statusQueue, _messageQueue, and _timestampQueue
    //boost::condition _condStatus; ///< notification when _statusqueue has data

    std::map<unsigned int, boost::shared_ptr<boost::thread> > _mPortCommandThread; ///< port -> thread
    boost::shared_ptr<boost::thread> _pStatusThread;
    boost::shared_ptr<boost::thread> _pDetectionThread;
    boost::shared_ptr<boost::thread> _pUpdateEnvironmentThread;

    boost::mutex _mutexCancelCommand;
    std::map<unsigned int, CommandServerPtr> _mPortCommandServer; ///< port -> server
    boost::mutex _mutexCommandServerMap;
    StatusPublisherPtr _pStatusPublisher;

    std::string _targetname; ///< name of the target object
    std::map<std::string, RegionPtr > _mNameRegion; ///< name->region
    std::map<std::string, CameraParametersPtr> _mNameCameraParameters; ///< name->camera param
    std::map<std::string, CameraPtr > _mNameCamera; ///< name->camera
    std::map<std::string, std::map<std::string, CameraPtr > > _mRegionColorCameraMap; ///< regionname -> name->camera
    std::map<std::string, std::map<std::string, CameraPtr > > _mRegionDepthCameraMap; ///< regionname -> name->camera
    std::map<std::string, boost::shared_ptr<CustomCommand> > _mNameCommand; ///< all registered commands, command name -> custom command

    std::vector<ImageSubscriberPtr> _vSubscribers;
    unsigned int _numDepthImagesToAverage;
    ImageSubscriberManagerPtr _pImagesubscriberManager;

    ObjectDetectorPtr _pDetector;
    boost::mutex _mutexDetector;
    DetectorManagerPtr _pDetectorManager;

    unsigned long long _tsStartDetection; ///< timestamp when start detection loop was first called
    std::set<unsigned long long> _sTimestamp; ///< set of saved timestamp in millisecond
    boost::mutex _mutexDetectedInfo; ///< lock for detection result
    std::vector<DetectedInfo> _vDetectedInfo; ///< latest detection result
    std::vector<DetectedObjectPtr> _vDetectedObject; ///< latest detection result
    bool _resultIsContainerEmpty; ///< container status of the latest result
    unsigned long long _resultTImestamp; ///< timestamp of latest detection result
    std::map<std::string, std::vector<Real> > _mResultPoints; ///< result pointcloud obstacle, cameraname -> points
    
    bool _bInitialized; ///< whether visionmanager is initialized
    bool _bShutdown; ///< whether the visionmanager is shut down
    bool _bStopStatusThread; ///< whether to stop status thread
    bool _bStopDetectionThread; ///< whether to stop detection thread
    bool _bStopUpdateEnvironmentThread; ///< whether to stop update environment thread
    bool _bCancelCommand; ///< whether to cancel the current user command
    bool _bExecutingUserCommand; ///< whether currently executing a user command
    std::map<unsigned int, bool > _mPortStopCommandThread; ///< port -> bool, whether to stop the command thread of specified port

};
typedef boost::shared_ptr<MujinVisionManager> MujinVisionManagerPtr;
typedef boost::shared_ptr<MujinVisionManager const> MujinVisionManagerConstPtr;
typedef boost::weak_ptr<MujinVisionManager> MujinVisionManagerWeakPtr;
} // namespace mujinvision
#endif
