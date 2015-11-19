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

#ifndef USE_LOG4CPP // logging

#define VISIONMANAGER_LOG_DEBUG(msg) std::cout << msg << std::endl;
#define VISIONMANAGER_LOG_INFO(msg) std::cout << msg << std::endl;
#define VISIONMANAGER_LOG_WARN(msg) std::cout << msg << std::endl;
#define VISIONMANAGER_LOG_ERROR(msg) std::cerr << msg << std::endl;

#else

#include <log4cpp/Category.hh>
#include <log4cpp/PropertyConfigurator.hh>

LOG4CPP_LOGGER_N(mujinvisionmanagerlogger, "mujinvisionmanager");

#define VISIONMANAGER_LOG_DEBUG(msg) LOG4CPP_DEBUG_S(mujinvisionmanagerlogger) << msg;
#define VISIONMANAGER_LOG_INFO(msg) LOG4CPP_INFO_S(mujinvisionmanagerlogger) << msg;
#define VISIONMANAGER_LOG_WARN(msg) LOG4CPP_WARN_S(mujinvisionmanagerlogger) << msg;
#define VISIONMANAGER_LOG_ERROR(msg) LOG4CPP_ERROR_S(mujinvisionmanagerlogger) << msg;

#endif // logging

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
                            const std::string& robotDeviceIOUri,
                            const unsigned int binpickingTaskZmqPort,
                            const unsigned int binpickingTaskHeartbeatPort,
                            const double binpickingTaskHeartbeatTimeout,
                            const std::string& binpickingTaskScenePk,
                            const std::string& robotname,
                            const std::string& targetname,
                            const std::string& streamerIp,
                            const unsigned int streamerPort,
                            const std::string& tasktype="binpicking",
                            const double controllertimeout=10.0, /*seconds*/
                            const std::string& locale="en_US",
                            const std::string& targeturi="",
                            const std::string& slaverequestid=""
                            );

    /** \brief Detects objects in specified region with specified cameras
        \param regionname name of the region
        \param cameranames names of the cameras
        \param detectedobjects detection results in meters in world frame
        \param resultstate additional information about the detection result
        \param ignoreocclusion whether to skip occlusion check
        \param maxage max time difference in ms allowed between the current time and the timestamp of image used for detection, 0 means infinity
        \param fetchimagetimeout max time in ms to wait for getting images for detection
        \param fastdetection whether to prioritize speed
        \param bindetection whether to detect bin
        \param request whether to request new images instead of getting them off the buffer
        \param useold whether to use previously captured images
     */
    virtual void DetectObjects(const std::string& regionname,
                               const std::vector<std::string>& cameranames,
                               std::vector<DetectedObjectPtr>& detectedobjectsworld,
                               std::string& resultstate,
                               const bool ignoreocclusion=false,
                               const unsigned int maxage=0,
                               const unsigned int fetchimagetimeout=0,
                               const bool fastdetection=false,
                               const bool bindetection=false,
                               const bool request=false,
                               const bool useold=false);

    /** \brief starts detection thread to continuously detect objects and sends detection results to mujin controller
     */
    virtual void StartDetectionLoop(const std::string& regionname,
                                    const std::vector<std::string>& cameranames,
                                    const double voxelsize=0.01,
                                    const double pointsize=0.005,
                                    const bool ignoreocclusion=false,
                                    const unsigned int maxage=0,
                                    const unsigned int fetchimagetimeout=0,
                                    const std::string& obstaclename="__dynamicobstacle__",
                                    const unsigned long long& starttime=0 /*ms*/,
                                    const std::string& locale="en_US");

    virtual void StopDetectionLoop();

    /** \brief Updates the point cloud obstacle and sends it to mujin controller
        \param regionname name of the region where the detection happens
        \param cameranames names of the cameras used for detection
        \param detectedobjects detection result in meters in  world frame
        \param maxage max time difference in ms allowed between the current time and the timestamp of image used for detection, 0 means infinity
        \param fetchimagetimeout max time in ms to wait for getting images for detection
        \param voxelsize size of the voxel grid in meters used for simplifying the cloud
        \param pointsize size of the point in meters to be sent to the mujin controller
        \param fast whether to prioritize speed
        \param request whether to request new images instead of getting them off the buffer
        \param async whether to return immediately
     */
    virtual void SendPointCloudObstacleToController(const std::string& regionname,
                                                    const std::vector<std::string>& cameranames,
                                                    const std::vector<DetectedObjectPtr>& detectedobjectsworld,
                                                    const unsigned int maxage=0,
                                                    const unsigned int fetchimagetimeout=0,
                                                    const double voxelsize=0.01,
                                                    const double pointsize=0.005,
                                                    const std::string& obstaclename="__dynamicobstacle__",
                                                    const bool fast=false,
                                                    const bool request=true,
                                                    const bool async=false,
                                                    const std::string& locale="en_US");

    /** \brief Visualizes the raw camera point clouds on mujin controller
     */
    virtual void VisualizePointCloudOnController(const std::string& regionname,
                                                 const std::vector<std::string>& cameranames,
                                                 const double pointsize=0.005,
                                                 const bool ignoreocclusion=false,
                                                 const unsigned int maxage=0,
                                                 const unsigned int fetchimagetimeout=0,
                                                 const bool request=true);

    /** \brief Clears visualization made by VisualizePointCloudOnController on mujin controller.
     */
    virtual void ClearVisualizationOnController();

    /** \brief Detects the transform of region
        \param regionname name of the region where the detection happens
        \param cameranames names of the cameras used for detection
        \param regiontransform detected new transform of the region
        \param ignoreocclusion whether to skip occlusion check
        \param maxage max time difference in ms allowed between the current time and the timestamp of image used for detection, 0 means infinity
        \param fetchimagetimeout max time in ms to wait for getting images for detection
        \param request whether to request new images instead of getting them off the buffer
     */
    virtual void DetectRegionTransform(const std::string& regionname, const std::vector<std::string>& cameranames, mujinvision::Transform& regiontransform, const bool ignoreocclusion, const unsigned int maxage=0, const unsigned int fetchimagetimeout=0, const bool request=false);

    /** \brief Saves a snapshot for each sensor mapped to the region. If detection was called before, snapshots of the images used for the last detection will be saved. Images are saved to the visionmanager application directory.
     */
    virtual void SaveSnapshot(const std::string& regionname, const bool ignoreocclusion=true, const unsigned int maxage=0, const unsigned int fetchimagetimeout=0, const bool request=true);

    /** \brief Updates the locally maintained list of the detected objects
        \param detectedobjectsworld detection result in world frame
        \param sendtocontroller whether to send the list to mujin controller
     */
    virtual void UpdateDetectedObjects(const std::vector<DetectedObjectPtr>& detectobjectsworld, const std::string& resultstate, const bool sendtocontroller=false);

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

    virtual void GetLatestDetectedObjects(std::vector<DetectedObjectPtr>& detectobjectsworld, std::string& resultstate, std::vector<Real>& points, const bool returnpoints=false);

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

    enum ThreadType {
        TT_Command=0,
        TT_Config=1,
        TT_Detector=2,
        TT_UpdateEnvironment=3,
        TT_ControllerMonitor=4,
        TT_SendPointcloudObstacle=5,
    };

    void _SetDetectorStatusMessage(const std::string& msg, const std::string& err="");
    void _SetStatusMessage(ThreadType tt, const std::string& msg, const std::string& err="");
    void _SetStatus(ThreadType tt, ManagerStatus status, const std::string& msg="", const std::string& err="", const bool allowInterrupt=true);

    /** \brief Executes command in json string. Returns result in json string.
     */
    void _ExecuteConfigurationCommand(const ptree& command_pt, std::stringstream& result_ss);
    void _ExecuteUserCommand(const ptree& command_pt, std::stringstream& result_ss);

    /** \brief Receives and executes commands from the user and sends results back.
     */
    void _CommandThread(const unsigned int port);
    void _StartCommandServer(const unsigned int port);
    void _StartCommandThread(const unsigned int port);
    void _StopCommandThread(const unsigned int port);

    /** \brief Publishes status periodically. When there are more status messages on the status queue, then publish all of them at once, otherwise, publish the last status message.
     */
    void _StatusThread(const unsigned int port, const unsigned int ms);
    void _StartStatusThread(const unsigned int port, const unsigned int ms=100);
    void _StopStatusThread();
    void _StartStatusPublisher(const unsigned int port);

    void _DetectObjects(ThreadType tt,
                        BinPickingTaskResourcePtr pBinpickingTask,
                        const std::string& regionname,
                        const std::vector<std::string>& cameranames,
                        std::vector<DetectedObjectPtr>& detectedobjectsworld,
                        std::string& resultstate,
                        const bool ignoreocclusion=false,
                        const unsigned int maxage=0,
                        const unsigned int fetchimagetimeout=0,
                        const bool fastdetection=false,
                        const bool bindetection=false,
                        const bool request=false,
                        const bool useold=false);
    void _DetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const std::string& obstaclename);
    void _StartDetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const std::string& obstaclename, const unsigned long long& starttime);
    void _StopDetectionThread();

    /** \brief Updates the environment state on mujin controller with the pointcloud obstacle and detected objects.
        \param regionname name of the region of which the pointcloud obstacle represents
        \param cameranames names of the cameras to be used to capture the pointcloud obstacle
        \param detectobjectsworld detected objects in world frame
        \param voxelsize size of the voxel grid in meters used for simplifying the cloud
        \param pointsize size of the point in meters to be sent to the mujin controller
     */
    void _UpdateEnvironmentThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const std::string& obstaclename="__dynamicobstacle__", const unsigned int waitinterval=50, const std::string& locale="en_US");
    void _StartUpdateEnvironmentThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const std::string& obstaclename, const unsigned int waitinterval=50, const std::string& locale="en_US");
    void _StopUpdateEnvironmentThread();

    void _ControllerMonitorThread(const unsigned int waitinterval=100, const std::string& locale="en_US");
    void _StartControllerMonitorThread(const unsigned int waitinterval=100, const std::string& locale="en_US");
    void _StopControllerMonitorThread();

    void _SendPointCloudObstacleToControllerThread(const std::string& regionname,
                                                   const std::vector<std::string>& cameranames,
                                                   const std::vector<DetectedObjectPtr>& detectedobjectsworld,
                                                   const unsigned int maxage=0,
                                                   const unsigned int fetchimagetimeout=0,
                                                   const double voxelsize=0.01,
                                                   const double pointsize=0.005,
                                                   const std::string& obstaclename="__dynamicobstacle__");

    /** \brief Gets transform of the instobject in meters.
     */
    mujinvision::Transform _GetTransform(const std::string& instobjname);

    /** \brief Updates the world transform of region from the mujin controller.
     */
    void _SyncRegion(const std::string& regionname);
    void _SyncRegion(const std::string& regionname, const mujinclient::Transform& t);
    void _SyncRegion(const std::string& regionname, const mujinvision::Transform& t);

    /** \brief Updates the world transform of camera from the mujin controller.
     */
    void _SyncCamera(const std::string& regionname, const std::string& cameraname);
    void _SyncCamera(const std::string& regionname, const std::string& cameraname, const mujinclient::Transform& t);

    void _GetImages(ThreadType tt, BinPickingTaskResourcePtr pBinpickingTask, const std::string& regionname, const std::vector<std::string>& colorcameranames, const std::vector<std::string>& depthcameranames, std::vector<ImagePtr>& colorimages, std::vector<ImagePtr>& depthimages, std::vector<ImagePtr>& resultimages, const bool ignoreocclusion, const unsigned int maxage=0 /*ms*/, const unsigned int fetchimagetimeout=0 /*ms*/, const bool request=false, const bool useold=false, const unsigned int waitinterval=50 /*ms*/);

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

    /** \brief Converts mujinclient::Transform to Transform.
     */
    Transform _GetTransform(const mujinclient::Transform& t);

    /** \brief Get string representation of transform.
     */
    std::string _GetString(const Transform& transform);

    /** \brief Gets status json string.
     */
    std::string _GetStatusJsonString(const unsigned long long timestamp, const std::string& status, const std::string& cmdmsg="", const std::string& cmderr="", const std::string& cfgmsg="", const std::string& cfgerr="", const std::string& detectormsg="", const std::string& detectorerr="", const std::string& updateenvmsg="", const std::string& updateenverr="", const std::string& controllermonmsg="", const std::string& controllermonerr="", const std::string& sendpclmsg="", const std::string& sendpclerr="");

    /** \brief Breaks the camera name string into camerabodyname and sensorname.
        \param cameraname string such as 'camerabodyname/sensorname'
        \param camerabodyname name of the camera kinbody
        \param sensorname name of the attached sensor
     */
    void _ParseCameraName(const std::string& cameraname, std::string& camerabodyname, std::string& sensorname);

    std::string _GetConfigFileName(const std::string& type, const std::string& configname);
    void _LoadConfig(const std::string& filename, std::string& content);

    bool _PreemptSubscriber();

    unsigned int _statusport, _commandport, _configport;
    std::string _configdir;
    std::string _detectorconfig, _imagesubscriberconfig;

    unsigned int _binpickingTaskZmqPort;
    unsigned int _binpickingTaskHeartbeatPort;
    double _binpickingTaskHeartbeatTimeout;
    std::string _binpickingTaskScenePk;
    std::string _robotControllerUri;
    std::string _robotDeviceIOUri;
    std::string _tasktype;

    boost::shared_ptr<zmq::context_t> _zmqcontext;

    ControllerClientPtr _pControllerClient;
    SceneResourcePtr _pSceneResource;
    BinPickingTaskResourcePtr _pBinpickingTask;

    std::queue<ManagerStatus> _statusQueue;
    std::queue<std::string> _commandMessageQueue, _configMessageQueue, _detectorMessageQueue, _updateenvironmentMessageQueue, _controllermonitorMessageQueue, _sendpointcloudMessageQueue;
    std::queue<std::string> _commandErrorQueue, _configErrorQueue, _detectorErrorQueue, _updateenvironmentErrorQueue, _controllermonitorErrorQueue, _sendpointcloudErrorQueue;
    std::queue<unsigned long long> _timestampQueue;
    boost::mutex _mutexStatusQueue; ///< protects _statusQueue, _messageQueue, and _timestampQueue
    //boost::condition _condStatus; ///< notification when _statusqueue has data

    std::map<unsigned int, boost::shared_ptr<boost::thread> > _mPortCommandThread; ///< port -> thread
    boost::shared_ptr<boost::thread> _pStatusThread;
    boost::shared_ptr<boost::thread> _pDetectionThread;
    boost::shared_ptr<boost::thread> _pUpdateEnvironmentThread;
    boost::shared_ptr<boost::thread> _pControllerMonitorThread;
    boost::shared_ptr<boost::thread> _pSendPointCloudObstacleThread;

    boost::mutex _mutexCancelCommand;
    std::map<unsigned int, CommandServerPtr> _mPortCommandServer; ///< port -> server
    boost::mutex _mutexCommandServerMap;
    StatusPublisherPtr _pStatusPublisher;

    std::string _targetname; ///< name of the target object
    std::string _targeturi; ///< uri of the target
    std::map<std::string, RegionPtr > _mNameRegion; ///< name->region
    std::map<std::string, CameraParametersPtr> _mNameCameraParameters; ///< name->camera param
    std::map<std::string, CameraPtr > _mNameCamera; ///< name->camera
    std::map<std::string, std::map<std::string, CameraPtr > > _mRegionColorCameraMap; ///< regionname -> name->camera
    std::map<std::string, std::map<std::string, CameraPtr > > _mRegionDepthCameraMap; ///< regionname -> name->camera
    std::map<std::string, boost::shared_ptr<CustomCommand> > _mNameCommand; ///< all registered commands, command name -> custom command

    ImageSubscriberManagerPtr _pImagesubscriberManager;

    boost::property_tree::ptree _ptDetectorConfig; ///< pt of detector config
    ObjectDetectorPtr _pDetector;
    DetectorManagerPtr _pDetectorManager;

    unsigned long long _tsStartDetection; ///< timestamp when start detection loop was first called
    std::set<unsigned long long> _sTimestamp; ///< set of saved timestamp in millisecond
    boost::mutex _mutexDetectedInfo; ///< lock for detection result
    std::vector<DetectedInfo> _vDetectedInfo; ///< latest detection result
    std::vector<DetectedObjectPtr> _vDetectedObject; ///< latest detection result
    unsigned long long _resultTimestamp; ///< timestamp of latest detection result
    std::map<std::string, std::vector<Real> > _mResultPoints; ///< result pointcloud obstacle, cameraname -> points
    boost::mutex _mutexControllerBinpickingState; ///< lock for controller binpicking state
    int _numPickAttempt; ///< num of picking attempts
    unsigned long long _binpickingstateTimestamp; ///< timestamp of latest binpicking state
    unsigned long long _lastocclusionTimestamp;
    std::vector<ImagePtr> _lastcolorimages; ///< last color images used for detection
    std::vector<ImagePtr> _lastdepthimages; ///< last depth images used for detection
    std::vector<ImagePtr> _lastresultimages; ///< last result image used for detection
    boost::mutex _mutexImagesubscriber; ///< lock for image subscriber
    boost::mutex _mutexDetector; ///< lock for detector

    std::string _locale; ///< controller locale
    std::string _resultState; ///< additional information about the detection result
    double _controllerCommandTimeout; ///< controller command timeout in seconds
    std::string _userinfo_json; ///< userinfo json
    std::string _slaverequestid; ///< slaverequestid to ensure that binpicking task uses the same slave

    bool _bIsControllerPickPlaceRunning; ///< whether pick and place thread is running on the controller
    bool _bIsRobotOccludingSourceContainer; ///< whether robot is occluding the source container
    bool _bForceRequestDetectionResults; ///< whether to run detection ignoring _numPickAttempt

    bool _bInitialized; ///< whether visionmanager is initialized
    bool _bShutdown; ///< whether the visionmanager is shut down
    bool _bStopStatusThread; ///< whether to stop status thread
    bool _bStopDetectionThread; ///< whether to stop detection thread
    bool _bStopUpdateEnvironmentThread; ///< whether to stop update environment thread
    bool _bStopControllerMonitorThread; ///< whether to stop controller monitor
    bool _bCancelCommand; ///< whether to cancel the current user command
    bool _bExecutingUserCommand; ///< whether currently executing a user command
    std::map<unsigned int, bool > _mPortStopCommandThread; ///< port -> bool, whether to stop the command thread of specified port

};
typedef boost::shared_ptr<MujinVisionManager> MujinVisionManagerPtr;
typedef boost::shared_ptr<MujinVisionManager const> MujinVisionManagerConstPtr;
typedef boost::weak_ptr<MujinVisionManager> MujinVisionManagerWeakPtr;
} // namespace mujinvision
#endif
