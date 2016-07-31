// -*- coding: utf-8 -*-
// Copyright (C) 2012-2016 MUJIN Inc.
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
#include <mujincontrollerclient/mujinzmq.h>
#include "mujinvision/visionparameters.h"
#include "mujinvision/imagesubscribermanager.h"
#include "mujinvision/detectormanager.h"

#include <queue>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

namespace mujinvision {

using namespace mujinclient;
using namespace mujinzmq;

class MUJINVISION_API MujinVisionManager
{
public:
    /** \brief sets up vision manager
     */
    MujinVisionManager(ImageSubscriberManagerPtr imagesubscribermanager, DetectorManagerPtr detectormanager, const unsigned int statusport, const unsigned int commandport, const unsigned int configport, const std::string& configdir, const std::string& detectiondir);
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

        /param defaultTaskParameters a JSON string with the default task parameters that should be included in every command call to the controller. If empty, then insert nothing
     */
    virtual void Initialize(const std::string& visionmanagerconfig,
                            const std::string& detectorconfigname,
                            const std::string& imagesubscriberconfig,
                            const std::string& controllerIp,
                            const unsigned int controllerPort,
                            const std::string& controllerUsernamePass,
                            const std::string& defaultTaskParameters,
                            const std::string& containerParameters,
                            const unsigned int binpickingTaskZmqPort,
                            const unsigned int binpickingTaskHeartbeatPort,
                            const double binpickingTaskHeartbeatTimeout,
                            const std::string& binpickingTaskScenePk,
                            const std::string& targetname,
                            const std::string& targeturi,
                            const std::string& targetupdatename,
                            const std::string& streamerIp,
                            const unsigned int streamerPort,
                            const std::string& tasktype="binpicking",
                            const double controllertimeout=10.0, /*seconds*/
                            const std::string& locale="en_US",
                            const std::string& slaverequestid="",
                            const std::string& targetdetectionarchiveurl="");

    /** \brief Detects objects in specified region with specified cameras
        \param regionname name of the region
        \param cameranames names of the cameras
        \param detectedobjects detection results in millimeters in world frame
        \param resultstate additional information about the detection result
        \param imageStartTimestamp for all captured images, the starttime in ms of the image capture
        \param imageEndTimestamp for all captured images, the endtime in ms of the image capture
        \param isContainerPresent whether container specified by regionname is present
        \param ignoreocclusion whether to skip occlusion check
        \param maxage max time difference in ms allowed between the current time and the timestamp of image used for detection, 0 means infinity
        \param newerthantimestamp if non 0, requries the starttimestamp of the image used for detection to be newer than the specified timestamp in milliseconds
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
                               unsigned long long& imageStartTimestamp,
                               unsigned long long& imageEndTimestamp,
                               int& isContainerPresent,
                               const bool ignoreocclusion=false,
                               const unsigned int maxage=0,
                               const unsigned long long newerthantimestamp=0,
                               const unsigned int fetchimagetimeout=0,
                               const bool fastdetection=false,
                               const bool bindetection=false,
                               const bool request=false,
                               const bool useold=false);

    /** \brief starts detection thread to continuously detect objects and sends detection results to mujin controller
        \param detectionstarttimestamp timestamp in ms indicating the start of the detection, if not specified, used the current time when detection thread is started. only images taken after detectionstarttime will be used for detection
        \param sendVerificationPointCloud whether send verification point cloud or not from current detection loop
        \param worldresultoffsettransform in millimeter
        \param voxelsize in millimeter
        \param targetupdatename name of the detected objects, if not specified, uses the name from the Initialize
     */
    virtual void StartDetectionLoop(const std::string& regionname,
                                    const std::vector<std::string>& cameranames,
                                    const std::vector<std::string>& executionverificationcameranames,
                                    const Transform& worldresultoffsettransform,
                                    const double voxelsize=0.01 * 1000,
                                    const double pointsize=5 * 1000,
                                    const bool ignoreocclusion=false,
                                    const unsigned int maxage=0,
                                    const unsigned int fetchimagetimeout=0,
                                    const std::string& obstaclename="__dynamicobstacle__",
                                    const unsigned long long detectionstarttimestamp=0 /*ms*/,
                                    const std::string& locale="en_US",
                                    const unsigned int maxnumfastdetection=1,
                                    const unsigned int maxnumdetection=0,
                                    const bool sendVerificationPointCloud=true,
                                    const bool stopOnLeftInOrder=false,
                                    const std::string& targetupdatename="");


    virtual void StopDetectionLoop();

    /** \brief Updates the point cloud obstacle and sends it to mujin controller
        \param regionname name of the region where the detection happens
        \param cameranames names of the cameras used for detection
        \param detectedobjects detection result in millimeters in  world frame
        \param maxage max time difference in ms allowed between the current time and the timestamp of image used for detection, 0 means infinity
        \param newerthantimestamp if non 0, requries the starttimestamp of the image used for getting pointcloud obstacle to be newer than the specified timestamp in milliseconds
        \param fetchimagetimeout max time in ms to wait for getting images for detection
        \param voxelsize size of the voxel grid in millimeters used for simplifying the cloud
        \param pointsize size of the point in millimeters to be sent to the mujin controller
        \param fast whether to prioritize speed
        \param request whether to request new images instead of getting them off the buffer
        \param async whether to return immediately
     */
    virtual void SendPointCloudObstacleToController(const std::string& regionname,
                                                    const std::vector<std::string>& cameranames,
                                                    const std::vector<DetectedObjectPtr>& detectedobjectsworld,
                                                    const unsigned int maxage=0,
                                                    const unsigned long long newerthantimestamp=0,
                                                    const unsigned int fetchimagetimeout=0,
                                                    const double voxelsize=0.01 * 1000,
                                                    const double pointsize=5 * 1000,
                                                    const std::string& obstaclename="__dynamicobstacle__",
                                                    const bool fast=false,
                                                    const bool request=true,
                                                    const bool async=false,
                                                    const std::string& locale="en_US");

    /** \brief Visualizes the raw camera point clouds on mujin controller
        \param maxage max time difference in ms allowed between the current time and the timestamp of image used, 0 means infinity
        \param newerthantimestamp if non 0, requries the starttimestamp of the image used for visualization to be newer than the specified timestamp in milliseconds
        \param voxelsize in millimeters
        \param pointsize in millimeters
     */
    virtual void VisualizePointCloudOnController(const std::string& regionname,
                                                 const std::vector<std::string>& cameranames,
                                                 const double pointsize=5 * 1000,
                                                 const bool ignoreocclusion=false,
                                                 const unsigned int maxage=0,
                                                 const unsigned long long newerthantimestamp=0,
                                                 const unsigned int fetchimagetimeout=0,
                                                 const bool request=true,
                                                 const double voxelsize=0.005 * 1000);

    /** \brief Clears visualization made by VisualizePointCloudOnController on mujin controller.
     */
    virtual void ClearVisualizationOnController();

    /** \brief Continuously synchronizing cameras and visualizing point clouds on mujin controller
        \param maxage max time difference in ms allowed between the current time and the timestamp of image used, 0 means infinity
        \param newerthantimestamp if non 0, requries the starttimestamp of the image used for visualization to be newer than the specified timestamp in milliseconds
        \param voxelsize in millimeters
        \param pointsize in millimeters
     */
    virtual void StartVisualizePointCloudThread(const std::string& regionname,
                                                const std::vector<std::string>& cameranames,
                                                const double pointsize=5 * 1000,
                                                const bool ignoreocclusion=false,
                                                const unsigned int maxage=0,
                                                const unsigned long long newerthantimestamp=0,
                                                const unsigned int fetchimagetimeout=0,
                                                const bool request=true,
                                                const double voxelsize=0.005 * 1000);

    virtual void StopVisualizePointCloudThread();

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

    virtual void GetLatestDetectedObjects(std::vector<DetectedObjectPtr>& detectobjectsworld, std::string& resultstate, std::vector<Real>& points, unsigned long long& imageStartTimestamp, unsigned long long& imageEndTimestamp, const bool returnpoints=false);

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

    struct DetectionThreadParams {
        double voxelsize;
        double pointsize;
        bool ignoreocclusion;
        bool stoponleftinorder;
        unsigned int maxage;
        unsigned long long newerthantimestamp;
        unsigned int fetchimagetimeout;
        unsigned int maxnumfastdetection;
        unsigned int maxnumdetection;
    };

    struct UpdateEnvironmentThreadParams {
        std::string regionname;
        std::vector<std::string> cameranames;
        double voxelsize;
        double pointsize;
        std::string obstaclename;
        unsigned int waitinterval;
        std::string locale;
    };

    struct SendExecutionVerificationPointCloudParams {
        std::vector<std::string> cameranames;
        std::vector<std::string> executionverificationcameranames;
        double voxelsize;
        double pointsize;
        std::string obstaclename;
        unsigned int waitinterval;
        std::string locale;
    };

    struct SendPointCloudObstacleToControllerThreadParams {
        std::string regionname;
        std::vector<std::string> cameranames;
        std::vector<DetectedObjectPtr> detectedobjectsworld;
        unsigned int maxage;
        unsigned long long newerthantimestamp;
        unsigned int fetchimagetimeout;
        double voxelsize;
        double pointsize;
        std::string obstaclename;
    };

    struct VisualizePointcloudThreadParams {
        std::string regionname;
        std::vector<std::string> cameranames;
        double pointsize;
        bool ignoreocclusion;
        unsigned int maxage;
        unsigned long long newerthantimestamp;
        unsigned int fetchimagetimeout;
        bool request;
        double voxelsize;
    };

    // for managing camera capturing life cycle
    class CameraCaptureHandle
    {
public:
        CameraCaptureHandle(ImageSubscriberManagerPtr pImagesubscriberManager, const std::string& cameraid, const std::string& cameraname);
        ~CameraCaptureHandle();
private:
        ImageSubscriberManagerPtr _pImagesubscriberManager;
        std::string _cameraid;
        std::string _cameraname;
    };
    typedef boost::shared_ptr<CameraCaptureHandle> CameraCaptureHandlePtr;
    typedef boost::weak_ptr<CameraCaptureHandle> CameraCaptureHandleWeakPtr;

    void _DeInitialize();

    enum ThreadType {
        TT_Command=0,
        TT_Config=1,
        TT_Detector=2,
        TT_UpdateEnvironment=3,
        TT_ControllerMonitor=4,
        TT_SendPointcloudObstacle=5,
        TT_VisualizePointCloud=6,
        TT_SendExecutionVerificationPointCloud=7
    };

    void _SetDetectorStatusMessage(const std::string& msg, const std::string& err="");
    void _SetStatusMessage(ThreadType tt, const std::string& msg, const std::string& err="");
    void _SetStatus(ThreadType tt, ManagerStatus status, const std::string& msg="", const std::string& err="", const bool allowInterrupt=true);

    /** \brief Executes command in json string. Returns result in json string.
     */
    void _ExecuteConfigurationCommand(const ptree& command_pt, std::stringstream& result_ss);
    void _ExecuteUserCommand(const ptree& command_pt, std::stringstream& result_ss);

    /** \brief Receives and executes commands from the user and sends results back.

        \brief commandindex is the index into _mPortStopCommandThread for seeing if the thread should be stopped.
     */
    void _RunCommandThread(const unsigned int port, int commandindex);

    /// \brief commandindex is the index into _mPortStopCommandThread for seeing if the thread should be stopped.
    void _StartCommandThread(const unsigned int port, int commandindex);

    /// \brief commandindex is the index into _mPortStopCommandThread for seeing if the thread should be stopped.
    void _StopCommandThread(int commandindex);

    /** \brief Publishes status periodically. When there are more status messages on the status queue, then publish all of them at once, otherwise, publish the last status message.
     */
    void _RunStatusThread(const unsigned int port, const unsigned int ms);
    void _StartStatusThread(const unsigned int port, const unsigned int ms=100);
    void _StopStatusThread();

    /**
       \returns number of detected objects
     */
    int _DetectObjects(ThreadType tt,
                       BinPickingTaskResourcePtr pBinpickingTask,
                       const std::string& regionname,
                       const std::vector<std::string>& cameranames,
                       std::vector<DetectedObjectPtr>& detectedobjectsworld,
                       std::string& resultstate,
                       unsigned long long& imageStartTimestamp,
                       unsigned long long& imageEndTimestamp,
                       int& isContainerPresent,
                       const bool ignoreocclusion=false,
                       const unsigned int maxage=0,
                       const unsigned long long newerthantimestamp=0,
                       const unsigned int fetchimagetimeout=0,
                       const bool fastdetection=false,
                       const bool bindetection=false,
                       const bool request=false,
                       const bool useold=false,
                       const bool checkcontaineremptyonly=false);

    /** \brief runs detection in a loop
     */
    void _DetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, DetectionThreadParams params);
    void _StartDetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const unsigned long long detectionstarttimestamp, const unsigned int maxnumfastdetection, const unsigned int maxnumdetection, const bool stoponleftinorder, const std::string& targetupdatename="");
    void _StopDetectionThread();

    void _VisualizePointCloudThread(VisualizePointcloudThreadParams params);
    void _StartVisualizePointCloudThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double pointsize=0.005 * 1000, const bool ignoreocclusion=false, const unsigned int maxage=0, const unsigned long long newerthantimestamp=0, const unsigned int fetchimagetimeout=0, const bool request=true, const double voxelsize=0.005 * 1000);
    void _StopVisualizePointCloudThread();

    /** \brief Updates the environment state on mujin controller with the pointcloud obstacle and detected objects.
        \param regionname name of the region of which the pointcloud obstacle represents
        \param cameranames names of the cameras to be used to capture the pointcloud obstacle
        \param detectobjectsworld detected objects in world frame
        \param voxelsize size of the voxel grid in millimeters used for simplifying the cloud
        \param pointsize size of the point in millimeters to be sent to the mujin controller
     */
    void _UpdateEnvironmentThread(UpdateEnvironmentThreadParams params);
    void _StartUpdateEnvironmentThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const std::string& obstaclename, const unsigned int waitinterval=50, const std::string& locale="en_US");
    void _StopUpdateEnvironmentThread();

    /** \brief thread that sends the execution verification point cloud
     */
    void _SendExecutionVerificationPointCloudThread(SendExecutionVerificationPointCloudParams params);
    void _StartExecutionVerificationPointCloudThread(const std::vector<std::string>& cameranames, const std::vector<std::string>& evcamnames, const double voxelsize, const double pointsize, const std::string& obstaclename, const unsigned int waitinterval=50, const std::string& locale="en_US");
    void _StopExecutionVerificationPointCloudThread();

    void _ControllerMonitorThread(const unsigned int waitinterval=100, const std::string& locale="en_US");
    void _StartControllerMonitorThread(const unsigned int waitinterval=100, const std::string& locale="en_US");
    void _StopControllerMonitorThread();

    void _SendPointCloudObstacleToController(const std::string& regionname, const std::vector<std::string>& cameranames, const std::vector<DetectedObjectPtr>& detectedobjectsworld, const unsigned int maxage=0, const unsigned long long newerthantimestamp=0, const unsigned int fetchimagetimeout=0, const double voxelsize=0.01 * 1000, const double pointsize=0.005 * 1000, const std::string& obstaclename="__dynamicobstacle__", const bool fast=false, const bool request=true, const bool async=false, const std::string& locale="en_US");
    void _SendPointCloudObstacleToControllerThread(SendPointCloudObstacleToControllerThreadParams params);
    void _StopSendPointCloudObstacleToControllerThread();

    void _VisualizePointCloudOnController(const std::string& regionname, const std::vector<std::string>& cameranames, const double pointsize=0.005 * 1000, const bool ignoreocclusion=false, const unsigned int maxage=0, const unsigned long long newerthantimestamp=0, const unsigned int fetchimagetimeout=0, const bool request=true, const double voxelsize=0.005 * 1000);

    /** \brief Gets transform of the instobject in meters.
     */
    mujinvision::Transform _GetTransform(const std::string& instobjname);

    /** \brief Updates the world transform of region from the mujin controller.
     */
    void _SyncRegion(const std::string& regionname);

    /** \brief Updates the world transform of region from the mujin controller.

        \param regionname the container name that can be queried via the controller api
        \param regiontransform the transform of the container origin that can be queried via the controller api
        \param baselinkobb the obb of just the link whose name is "base" of the container defined by regionname. obb is in world frame.
        \param innerobb the inner extents of the empty region where parts can be placed. obb is in world frame.
     */
    void _SyncRegion(const std::string& regionname, const Transform& regiontransform, const BinPickingTaskResource::ResultOBB& baselinkobb, const BinPickingTaskResource::ResultOBB& innerobb);

    /** \brief Updates the world transform of camera from the mujin controller.
     */
    void _SyncCamera(const std::string& cameraname);
    void _SyncCamera(const std::string& cameraname, const mujinclient::Transform& t);

    /** \brief gets images from specified cameras for specified region
        \param output imageStartTimestamp for all captured images, the starttime in ms of the image captured
        \param output imageEndTimestamp for all captured images, the endtime in ms of the image captured
        \param maxage max age in milliseconds of image to use, the call blocks until all images satisfy requirements or passed fetchimagetimeout
        \param newerthantimestamp images must be newer than the specified timestamp, the call blocks until all images satisfy requirements or passed fetchimagetimeout
     */
    void _GetImages(ThreadType tt, BinPickingTaskResourcePtr pBinpickingTask, const std::string& regionname, const std::vector<std::string>& colorcameranames, const std::vector<std::string>& depthcameranames, std::vector<ImagePtr>& colorimages, std::vector<ImagePtr>& depthimages, std::vector<ImagePtr>& resultimages, unsigned long long& imageStartTimestamp, unsigned long long& imageEndTimestamp, bool ignoreocclusion, const unsigned int maxage=0 /*ms*/, const unsigned long long newerthantimestamp=0 /*ms*/, const unsigned int fetchimagetimeout=0 /*ms*/, const bool request=false, const bool useold=false, const unsigned int waitinterval=50 /*ms*/);

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

    std::vector<std::string> _GetHardwareIds(const std::vector<std::string>& cameranames);

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

    bool _CheckPreemptSubscriber();
    bool _CheckPreemptDetector(const unsigned int checkpreemptbits);

    /** \brief checks if region camera mapping has changed for specified region and cameras, if so, reset cached data, detector, and streamer accordingly
     */
    void _CheckAndUpdateRegionCameraMapping(const std::string& regionname, const std::vector<std::string>& cameranames);

    void _StartAndGetCaptureHandle(const std::vector<std::string>& camreaids, const std::vector<std::string>& cameranamestocheckocclusion, std::vector<CameraCaptureHandlePtr>& capturehandles, bool force=false);

    boost::mutex _mutexCaptureHandles; ///< lock for _mCameranameCaptureHandles
    std::map<std::string, CameraCaptureHandleWeakPtr> _mCameranameCaptureHandles; ///< list of handles that maintain the runtime capture state of cameras, protected by _mutexCaptureHandles

    unsigned int _statusport, _commandport, _configport;
    std::string _configdir;
    std::string _detectiondir;
    std::string _detectorconfig, _imagesubscriberconfig;

    unsigned int _binpickingTaskZmqPort;
    unsigned int _binpickingTaskHeartbeatPort;
    double _binpickingTaskHeartbeatTimeout;
    std::string _binpickingTaskScenePk;
    std::string _defaultTaskParameters; ///< a JSON string with the default task parameters that should be included in every command call to the controller. If empty, then insert nothing
    std::string _containerParameters; ///< a json string with container info
    std::string _tasktype;

    boost::shared_ptr<zmq::context_t> _zmqcontext;

    ControllerClientPtr _pControllerClient;
    SceneResourcePtr _pSceneResource;
    BinPickingTaskResourcePtr _pBinpickingTask;

    std::queue<ManagerStatus> _statusQueue;
    std::queue<std::string> _commandMessageQueue, _configMessageQueue, _detectorMessageQueue, _updateenvironmentMessageQueue, _controllermonitorMessageQueue, _sendpointcloudMessageQueue, _visualizepointcloudMessageQueue, _sendexecverificationMessageQueue;
    std::queue<std::string> _commandErrorQueue, _configErrorQueue, _detectorErrorQueue, _updateenvironmentErrorQueue, _controllermonitorErrorQueue, _sendpointcloudErrorQueue, _visualizepointcloudErrorQueue, _sendexecverificationErrorQueue;
    std::queue<unsigned long long> _timestampQueue;
    boost::mutex _mutexStatusQueue; ///< protects _statusQueue, _messageQueue, and _timestampQueue

    boost::array< boost::shared_ptr<boost::thread>, 2 > _mPortCommandThread; ///< command index -> thread
    boost::shared_ptr<boost::thread> _pStatusThread;
    boost::shared_ptr<boost::thread> _pDetectionThread;
    boost::shared_ptr<boost::thread> _pUpdateEnvironmentThread;
    boost::shared_ptr<boost::thread> _pExecutionVerificationPointCloudThread;
    boost::shared_ptr<boost::thread> _pControllerMonitorThread;
    boost::shared_ptr<boost::thread> _pSendPointCloudObstacleThread;
    boost::shared_ptr<boost::thread> _pVisualizePointCloudThread;

    boost::mutex _mutexCancelCommand;

    std::string _targetname; ///< name of the target object
    std::string _targeturi; ///< uri of the target
    std::string _targetupdatename; ///< prefix of the detected object name used to update environment

    //@{ info related to region. protected by _mutexDetectedInfo
    boost::mutex _mutexRegion;  ///< lock protecting region info
    std::map<std::string, RegionPtr > _mNameRegion; ///< name->region
    std::map<std::string, std::string> _mCameranameActiveRegionname; ///< cameraname -> name of region that is being detected
    //@}

    std::map<std::string, CameraParametersPtr> _mNameCameraParameters; ///< name->camera param
    std::map<std::string, CameraPtr > _mNameCamera; ///< name->camera
    std::map<std::string, boost::shared_ptr<CustomCommand> > _mNameCommand; ///< all registered commands, command name -> custom command
    std::map<std::string, std::string> _mCameraNameHardwareId; ///< camera name -> camera hardware id
    std::map<std::string, std::string> _mHardwareIdCameraName; ///< camera hardware id -> name
    std::map<std::string, std::string> _mDetectorExtraInitializationOptions; ///< extra init options for detector
    ImageSubscriberManagerPtr _pImagesubscriberManager;

    ObjectDetectorPtr _pDetector;
    DetectorManagerPtr _pDetectorManager;

    unsigned long long _tsStartDetection; ///< timestamp when start detection loop was first called
    unsigned long long _tsLastEnvUpdate; ///< timestamp when binpickingtask->UpdateEnvironmentState was called
    std::set<unsigned long long> _sTimestamp; ///< set of saved timestamp in millisecond
    unsigned long long _resultTimestamp; ///< ms timestamp of latest detection result. protected by _mutexDetectedInfo
    unsigned long long _resultImageStartTimestamp, _resultImageEndTimestamp; ///< ms timestamps for the latest detection result, the image start and end timestamps.
    unsigned long long _lastSendPointCloudObstacleTimestamp; ///< ms timestamp when pointcloud obstacle is sent
    boost::mutex _mutexControllerBinpickingState; ///< lock for controller binpicking state
    unsigned long long _lastocclusionTimestamp;
    std::vector<ImagePtr> _lastcolorimages; ///< last color images used for detection
    std::vector<ImagePtr> _lastdepthimages; ///< last depth images used for detection
    std::vector<ImagePtr> _lastresultimages; ///< last result image used for detection
    boost::mutex _mutexImagesubscriber; ///< lock for image subscriber
    boost::mutex _mutexDetector; ///< lock for detector

    ptree _visionserverpt; ///< ptree storing visionserver params
    std::vector<std::string> _vExecutionVerificationCameraNames; ///< names of cameras for exec verification
    double _filteringvoxelsize;  ///< point cloud filting param for exec verification
    double _filteringstddev;  ///< point cloud filting param for exec verification
    int _filteringnumnn;  ///< point cloud filting param for exec verification
    Transform _tWorldResultOffset; ///< transform to be applied to detection result in world frame

    std::string _locale; ///< controller locale

    //@{ detection state. protected by _mutexDetectedInfo
    boost::mutex _mutexDetectedInfo; ///< lock for detection result
    std::vector<DetectedObjectPtr> _vDetectedObject; ///< latest detection result. protected by _mutexDetectedInfo
    std::string _resultState; ///< additional information about the detection result. protected by _mutexDetectedInfo
    std::map<std::string, std::vector<Real> > _mResultPoints; ///< result pointcloud obstacle, cameraname -> points. protected by _mutexDetectedInfo
    //@}

    double _controllerCommandTimeout; ///< controller command timeout in seconds
    std::string _userinfo_json; ///< userinfo json
    std::string _slaverequestid; ///< slaverequestid to ensure that binpicking task uses the same slave
    std::string _controllerIp; ///< controller ip

    // mujin controller binpicking state
    unsigned long long _binpickingstateTimestamp; ///< timestamp of latest binpicking state
    unsigned long long _lastGrabbedTargetTimestamp; ///< timestamp when last grabbed target
    unsigned long long _lastDetectStartTimestamp; ///< starting timestamp of the images sent to the detector in the last detection call
    int _numPickAttempt; ///< num of picking attempts
    int _orderNumber;  ///< num of ordered items
    int _numLeftInOrder; ///< num of order to go
    int _numLeftInSupply; ///< num of items left in supply
    int _placedInDest; ///< num placed in destination
    bool _bIsControllerPickPlaceRunning; ///< whether pick and place thread is running on the controller
    bool _bIsRobotOccludingSourceContainer; ///< whether robot is occluding the source container
    bool _bForceRequestDetectionResults; ///< whether to run detection ignoring _numPickAttempt
    bool _bIsGrabbingTarget; ///< whether the robot is grabbing target
    bool _bIsGrabbingLastTarget; ///< whether the robot is grabbing the last target

    // vision manager flags
    bool _bInitialized; ///< whether visionmanager is initialized
    bool _bShutdown; ///< whether the visionmanager is shut down
    bool _bStopStatusThread; ///< whether to stop status thread
    bool _bStopDetectionThread; ///< whether to stop detection thread
    bool _bStopUpdateEnvironmentThread; ///< whether to stop update environment thread
    bool _bStopExecutionVerificationPointCloudThread; ///< whether to stop the verification point cloud from sending
    bool _bStopControllerMonitorThread; ///< whether to stop controller monitor
    bool _bStopSendPointCloudObstacleToControllerThread; ///< whether to stop async send pointcloud obstacle to controller call
    bool _bStopVisualizePointCloudThread; ///< whether to stop pointcloud visualization thread
    bool _bCancelCommand; ///< whether to cancel the current user command
    bool _bExecutingUserCommand; ///< whether currently executing a user command
    bool _bIsDetectionRunning; ///< true if detection thread is running
    bool _bIsVisualizePointcloudRunning; ///< whether the point cloud visualization thread is running
    bool _bIsSendPointcloudRunning; ///< whether send point cloud obstacle thread is running
    bool _bIsEnvironmentUpdateRunning; ///< whether env update thread is running
    bool _bSendVerificationPointCloud; ///< whether send verification point cloud or not
    bool _bDetectedObjectsValid; ///< if true, then _vDetectedObject is valid and should be updated to the scene. if false, then not valid and UpdateEnvironmentState should not update the objects. protected by _mutexDetectedInfo
    bool _bUseGrabbedTargeInfoInDetectionPreempt; ///< if true, then use _lastGrabbedTargetTimestamp and numLeftInOrder to determine whether the detector should be preempted or not. by default it is false.
    bool _bDetectBin; ///< whether to detect bin
    boost::array<bool, 2> _mPortStopCommandThread; ///< command index -> bool, whether to stop the command thread of specified port

    std::string _detectionRegionName; ///< name of the region on which the last detection was started or currently running
};
typedef boost::shared_ptr<MujinVisionManager> MujinVisionManagerPtr;
typedef boost::shared_ptr<MujinVisionManager const> MujinVisionManagerConstPtr;
typedef boost::weak_ptr<MujinVisionManager> MujinVisionManagerWeakPtr;
} // namespace mujinvision
#endif
