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
#include "mujinvision/mujinvisionmanager.h"
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <utime.h>
#include <sys/stat.h>

#include "logging.h"

MUJIN_LOGGER("mujin.mujinvision.manager");

#define CREATE_SAFE_DELETER_CAMERAHANDLES(capturehandles) boost::shared_ptr<void> capturehandlessafedeleter((void*)0, boost::bind(&MujinVisionManager::_DeleteCameraHandlesSafely, this, boost::ref(capturehandles)));

#ifndef MUJIN_TIME
#define MUJIN_TIME
#include <time.h>

#ifndef _WIN32
#include <unistd.h>
#if !(defined(CLOCK_GETTIME_FOUND) && (POSIX_TIMERS > 0 || _POSIX_TIMERS > 0))
#include <sys/time.h>
#endif
#else
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <sys/timeb.h>    // ftime(), struct timeb
inline void usleep(unsigned long microseconds) {
    Sleep((microseconds+999)/1000);
}
#include <tchar.h>
#endif

namespace mujinvision {

#ifdef _WIN32
inline uint64_t GetMilliTime()
{
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (uint64_t)((count.QuadPart * 1000) / freq.QuadPart);
}

inline uint64_t GetMicroTime()
{
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000) / freq.QuadPart;
}

inline uint64_t GetNanoTime()
{
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000000) / freq.QuadPart;
}

inline static uint64_t GetNanoPerformanceTime() {
    return GetNanoTime();
}

#else

inline void GetWallTime(uint32_t& sec, uint32_t& nsec)
{
#if defined(CLOCK_GETTIME_FOUND) && (POSIX_TIMERS > 0 || _POSIX_TIMERS > 0)
    struct timespec start;
    clock_gettime(CLOCK_REALTIME, &start);
    sec  = start.tv_sec;
    nsec = start.tv_nsec;
#else
    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    sec  = timeofday.tv_sec;
    nsec = timeofday.tv_usec * 1000;
#endif
}

inline uint64_t GetMilliTimeOfDay()
{
    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    return (uint64_t)timeofday.tv_sec*1000+(uint64_t)timeofday.tv_usec/1000;
}

inline uint64_t GetNanoTime()
{
    uint32_t sec,nsec;
    GetWallTime(sec,nsec);
    return (uint64_t)sec*1000000000 + (uint64_t)nsec;
}

inline uint64_t GetMicroTime()
{
    uint32_t sec,nsec;
    GetWallTime(sec,nsec);
    return (uint64_t)sec*1000000 + (uint64_t)nsec/1000;
}

inline uint64_t GetMilliTime()
{
    uint32_t sec,nsec;
    GetWallTime(sec,nsec);
    return (uint64_t)sec*1000 + (uint64_t)nsec/1000000;
}

inline static uint64_t GetNanoPerformanceTime()
{
#if defined(CLOCK_GETTIME_FOUND) && (POSIX_TIMERS > 0 || _POSIX_TIMERS > 0) && defined(_POSIX_MONOTONIC_CLOCK)
    struct timespec start;
    uint32_t sec, nsec;
    clock_gettime(CLOCK_MONOTONIC, &start);
    sec  = start.tv_sec;
    nsec = start.tv_nsec;
    return (uint64_t)sec*1000000000 + (uint64_t)nsec;
#else
    return GetNanoTime();
#endif
}
#endif
#endif

// http://stackoverflow.com/questions/504810/how-do-i-find-the-current-machines-full-hostname-in-c-hostname-and-domain-info
void __GetMachineName(char* machineName)
{
    char Name[150];

    #ifdef WIN32
    TCHAR infoBuf[150];
    DWORD bufCharCount = 150;
    memset(Name, 0, 150);
    if( GetComputerName( infoBuf, &bufCharCount ) ) {
        for(int i=0; i<150; i++) {
            Name[i] = infoBuf[i];
        }
    } else {
        strcpy(Name, "Unknown_Host_Name");
    }
    #else
    memset(Name, 0, 150);
    gethostname(Name, 150);
    #endif
    strncpy(machineName, Name, 150);
}

std::string __GetString(const std::vector<std::string>& strings)
{
    std::stringstream ss;
    for (size_t i=0; i<strings.size(); ++i) {
        ss << strings[i] << " ";
    }
    return ss.str();
}

enum CommandThreadIndex
{
    CDI_Command=0,
    CDI_Configure=1,
};

/// \brief the components of the system used to signify which one is being preempted
enum PreemptComponent
{
    PC_DetectionThread=1,
    PC_SendPointcloudThread=2,
    PC_ExecutionVerificationThread=4,
    PC_VisualizePointCloudThread=8,
};

std::string _GetExtraCaptureOptions(const std::vector<std::string>& cameraids, const std::vector<std::string>& cameraidstocheckocclusion, const rapidjson::Document& visionserverconfig, const std::string& controllerip, int binpickingTaskZmqPort, const std::string& slaverequestid, const std::map<std::string, std::string>& mCameraNameHardwareId, const std::map<std::string, std::string>& mCameranameActiveRegionname, const std::string& subscriberid, const bool ignoreocclusion=false, const std::string& bindetectionMode="never")
{
    std::string controllerclientconnectionstring = str(boost::format("tcp://%s:%d") % controllerip % binpickingTaskZmqPort);
    std::string occlusioncheckcommandtemplate = GetJsonValueByKey<std::string>(visionserverconfig, "occlusioncheckcommandtemplate");
    std::string customparameters;
    if (visionserverconfig.HasMember("streamercustomparameters")) {
        customparameters = DumpJson(visionserverconfig["streamercustomparameters"]);
    }
    boost::replace_all(occlusioncheckcommandtemplate, "dummyslaverequestid", slaverequestid);

    rapidjson::Document cameraidfullnamemapjson(rapidjson::kObjectType),
                        cameraidregionnamejson(rapidjson::kObjectType),
                        cameraidcheckocclusionjson(rapidjson::kObjectType);
    FOREACH(v, mCameraNameHardwareId) {
        SetJsonValueByKey(cameraidfullnamemapjson, v->second, v->first);
        std::map<std::string, std::string>::const_iterator cit = mCameranameActiveRegionname.find(v->first);
        if (cit != mCameranameActiveRegionname.end()) {
            SetJsonValueByKey(cameraidregionnamejson, v->second, cit->second);
        } else {
            MUJIN_LOG_VERBOSE("failed to find regionname for camera " << v->first);
        }
    }
    for (size_t i=0; i<cameraids.size(); ++i) {
        SetJsonValueByKey(cameraidcheckocclusionjson, cameraids[i],
                          std::find(cameraidstocheckocclusion.begin(), cameraidstocheckocclusion.end(), cameraids[i]) != cameraidstocheckocclusion.end());
    }
    rapidjson::Document extraoptionsjson(rapidjson::kObjectType);
    SetJsonValueByKey(extraoptionsjson, "controllerclientconnectionstring", controllerclientconnectionstring);
    SetJsonValueByKey(extraoptionsjson, "occlusioncheckcommandtemplate", occlusioncheckcommandtemplate);
    SetJsonValueByKey(extraoptionsjson, "cameraidfullnamemap", cameraidfullnamemapjson);
    SetJsonValueByKey(extraoptionsjson, "cameraidregionnamemap", cameraidregionnamejson);
    SetJsonValueByKey(extraoptionsjson, "cameraidcheckocclusionmap", cameraidcheckocclusionjson);
    SetJsonValueByKey(extraoptionsjson, "subscriberid", subscriberid);
    SetJsonValueByKey(extraoptionsjson, "ignoreocclusion", (int)ignoreocclusion);
    SetJsonValueByKey(extraoptionsjson, "bindetectionmode", bindetectionMode);
    if (customparameters.size() > 0) {
        SetJsonValueByKey(extraoptionsjson, "customparameters", customparameters);
    }
    try {
        return DumpJson(extraoptionsjson);
    } catch (const std::exception& ex) {
        MUJIN_LOG_WARN("failed to get extraoptions string. controllerclientconnectionstring=" << controllerclientconnectionstring << " occlusioncheckcommandtemplate=" << occlusioncheckcommandtemplate);
        return "";
    }
}

void MujinVisionManager::_CheckPreemptSubscriber(const unsigned int checkpreemptbits)
{
    bool bCheckDetectionThreadPreempt = !!(checkpreemptbits&PC_DetectionThread);
    bool bCheckSendPointcloudThreadPreempt = !!(checkpreemptbits&PC_SendPointcloudThread);
    bool bCheckExecutionVerificationThreadPreempt = !!(checkpreemptbits&PC_ExecutionVerificationThread);
    bool bCheckVisualizePointCloudThreadPreempt = !!(checkpreemptbits&PC_VisualizePointCloudThread);

    if (bCheckDetectionThreadPreempt) { // when called from subscriber used by the detection thread
        bool bPreemptSubscriberFromDetectionThread = _bShutdown || _bCancelCommand || (_bStopDetectionThread && !_bIsSendPointcloudRunning && !_bIsVisualizePointcloudRunning); // do not preempt when send pointcloud or visualize pointcloud thread is running
        if (bPreemptSubscriberFromDetectionThread) {
            MUJIN_LOG_DEBUG(str(boost::format("preempt from detection thread. _bShutdown=%d _bCancelCommand=%d _bStopDetectionThread=%d _bIsSendPointcloudRunning=%d _bIsVisualizePointcloudRunning=%d") % _bShutdown % _bCancelCommand % _bStopDetectionThread % _bIsSendPointcloudRunning % _bIsVisualizePointcloudRunning));
            throw mujinclient::UserInterruptException("preempt from detection thread");
        }
    }
    if (bCheckExecutionVerificationThreadPreempt) { // when called from subscriber used by the execution verification thread
        bool bPreemptSubscriberFromExecutionVerificationThread = _bShutdown || _bCancelCommand || (_bStopExecutionVerificationPointCloudThread && !_bIsSendPointcloudRunning && !_bIsVisualizePointcloudRunning); // do not preempt when send pointcloud or visualize pointcloud thread is running
        if (bPreemptSubscriberFromExecutionVerificationThread) {
            MUJIN_LOG_DEBUG(str(boost::format("preempt subscriber from execution verification thread. _bShutdown=%d _bCancelCommand=%d _bStopExecutionVerificationPointCloudThread=%d _bIsSendPointcloudRunning=%d _bIsVisualizePointcloudRunning=%d") % _bShutdown % _bCancelCommand % _bStopExecutionVerificationPointCloudThread % _bIsSendPointcloudRunning % _bIsVisualizePointcloudRunning));
            throw mujinclient::UserInterruptException("preempt from execution verification thread");
        }
    }
    if (bCheckSendPointcloudThreadPreempt) { // when called from subscriber used by the send pointcloud thread
        bool bPreemptSubscriberFromSendPointcloudThread = _bShutdown || _bCancelCommand || (_bStopSendPointCloudObstacleToControllerThread && !_bIsDetectionRunning && !_bIsExecutionVerificationPointCloudRunning && !_bIsVisualizePointcloudRunning); // do not preempt when detection or execution verification or visualize pointcloud thread is running
        if (bPreemptSubscriberFromSendPointcloudThread) {
            MUJIN_LOG_DEBUG(str(boost::format("preempt subscriber from send pointcloud thread. _bShutdown=%d _bCancelCommand=%d _bStopSendPointCloudObstacleToControllerThread=%d _bIsDetectionRunning=%d _bIsExecutionVerificationPointCloudRunning=%d _bIsVisualizePointcloudRunning=%d") % _bShutdown % _bCancelCommand % _bStopSendPointCloudObstacleToControllerThread % _bIsDetectionRunning % _bIsExecutionVerificationPointCloudRunning % _bIsVisualizePointcloudRunning));
            throw mujinclient::UserInterruptException("preempt from send pointcloud thread");
        }
    }
    if (bCheckVisualizePointCloudThreadPreempt) { // when called from subscriber used by visualize pointcloud thread
        bool bPreemptSubscriberFromVisualizePointcloudThread = _bShutdown || _bCancelCommand || (_bStopVisualizePointCloudThread && !_bIsSendPointcloudRunning && !_bIsDetectionRunning && !_bIsExecutionVerificationPointCloudRunning); // do not preempt when detection or execution verification or send pointcloud thread is running
        if (bPreemptSubscriberFromVisualizePointcloudThread) {
            MUJIN_LOG_DEBUG(str(boost::format("preempt subscriber from visualize pointcloud thread. _bShutdown=%d _bCancelCommand=%d _bStopVisualizePointCloudThread=%d _bIsDetectionRunning=%d _bIsExecutionVerificationPointCloudRunning=%d _bIsSendPointcloudRunning=%d") % _bShutdown % _bCancelCommand % _bStopVisualizePointCloudThread % _bIsDetectionRunning % _bIsExecutionVerificationPointCloudRunning % _bIsSendPointcloudRunning));
            throw mujinclient::UserInterruptException("preempt from visualize point cloud thread");
        }
    }

    /**
       // if SendingPointCloud is running, don't stop subscriber even we are stoping threads, so threads might get blocked when getting images
       bool bpreempt = _bShutdown || _bCancelCommand; // || ((_bStopDetectionThread || _bStopUpdateEnvironmentThread) && !_bIsSendPointcloudRunning); // || _bStopExecutionVerificationPointCloudThread;
       if (bpreempt ) {
        MUJIN_LOG_DEBUG("preempt subscriber! _bShutdown=" << int(_bShutdown) << " _bCancelCommand=" << int(_bCancelCommand) << " _bStopDetectionThread=" << _bStopDetectionThread << " _bStopUpdateEnvironmentThread=" << _bStopUpdateEnvironmentThread << " _bStopExecutionVerificationPointCloudThread=" << _bStopExecutionVerificationPointCloudThread << " _bIsSendingPointCloudRunning=" << _bIsSendPointcloudRunning);
       }
     */
}

void MujinVisionManager::_CheckPreemptDetector(const unsigned int checkpreemptbits)
{
    bool bIsGrabbingLastTarget;
    bool bForceRequestDetectionResults;
    unsigned long long lastGrabbedTargetTimestamp;
    int numLeftInOrder;
    {
        boost::mutex::scoped_lock lock(_mutexControllerBinpickingState);
        bIsGrabbingLastTarget = _bIsGrabbingLastTarget;
        bForceRequestDetectionResults = _bForceRequestDetectionResults;
        lastGrabbedTargetTimestamp = _lastGrabbedTargetTimestamp;
        numLeftInOrder = _numLeftInOrder;
    }

    bool bCheckDetectionThreadPreempt = !!(checkpreemptbits&PC_DetectionThread);
    bool bCheckSendPointcloudThreadPreempt = !!(checkpreemptbits&PC_SendPointcloudThread);
    
    if (bCheckDetectionThreadPreempt) {
        bool bPreemptDetectionThread = _bShutdown || _bCancelCommand || _bStopDetectionThread;
        if( _bUseGrabbedTargeInfoInDetectionPreempt ) {
            if( _tsStartDetection > 0 ) { // detection is running
                if( lastGrabbedTargetTimestamp > _tsStartDetection ) { // already on the last item since lastGrabbedTargetTimestamp is set
                    if( (bIsGrabbingLastTarget || numLeftInOrder == 0) ) { // currently grabbing the last item or the last item has already been placed so numLeftInOrder is 0
                        MUJIN_LOG_DEBUG("lastGrabbedTargetTimestamp=" << lastGrabbedTargetTimestamp << " _lastDetectStartTimestamp=" << _lastDetectStartTimestamp);
                        if( lastGrabbedTargetTimestamp > _lastDetectStartTimestamp ) { // if image time is earlier than the grabbed timestamp, have to preempt
                            bPreemptDetectionThread = true;
                        }
                    }
                }
            }
        }
        if (bPreemptDetectionThread) {
            MUJIN_LOG_INFO(str(boost::format("Preempting detector call by DetectionThread! bPreemptDetectionThread=%d _bShutdown=%d _bCancelCommand=%d _bStopDetectionThread=%d lastGrabbedTargetTimestamp=%u _lastDetectStartTimestamp=%u _tsStartDetection=%u bForceRequestDetectionResults=%d bIsGrabbingLastTarget=%d numLeftInOrder=%d")%bPreemptDetectionThread%_bShutdown%_bCancelCommand%_bStopDetectionThread%lastGrabbedTargetTimestamp%_lastDetectStartTimestamp%_tsStartDetection%bForceRequestDetectionResults%bIsGrabbingLastTarget%numLeftInOrder));
            throw mujinclient::UserInterruptException("preempt detection thread from detector.");
        }
    }
    if (bCheckSendPointcloudThreadPreempt) {
        bool bPreemptSendPointcloudObstacleThread = _bShutdown || _bCancelCommand || _bStopSendPointCloudObstacleToControllerThread;
        if (bPreemptSendPointcloudObstacleThread) {
            MUJIN_LOG_INFO(str(boost::format("Preempting detector call by SendPointCloudObstacleThread! bPreemptSendPointcloudObstacleThread=%d _bShutdown=%d _bCancelCommand=%d")%bPreemptSendPointcloudObstacleThread%_bShutdown%_bCancelCommand));
            throw mujinclient::UserInterruptException("preempt send point cloud from detector.");
        }
    }
}

MujinVisionManager::CameraCaptureHandle::CameraCaptureHandle(ImageSubscriberManagerPtr pImagesubscriberManager, const std::string& cameraid, const std::string& cameraname) : _pImagesubscriberManager(pImagesubscriberManager), _cameraid(cameraid), _cameraname(cameraname)
{
}

MujinVisionManager::CameraCaptureHandle::~CameraCaptureHandle() {
    try {
        std::vector<std::string> tostop;
        tostop.push_back(_cameraid);
        MUJIN_LOG_INFO("stop capturing for camera " << _cameraname << "(" << _cameraid << ")");
        _pImagesubscriberManager->StopCaptureThread(tostop);
    }
    catch(const std::exception& ex) {
        MUJIN_LOG_ERROR(str(boost::format("failed to stop capturing for camera %s (%s) due to %s")%_cameraname%_cameraid%ex.what()));
    }
    catch (...) {
        MUJIN_LOG_ERROR("failed to stop capturing for camera " << _cameraname << "(" << _cameraid << ") due to unknown error.");
    }
}

void MujinVisionManager::_StartAndGetCaptureHandle(const std::vector<std::string>& cameranames, const std::vector<std::string>& cameranamestocheckocclusion, std::vector<CameraCaptureHandlePtr>& capturehandles, const bool force, const bool ignoreocclusion)
{
    bool runpublisher = GetJsonValueByKey<bool>(_visionserverconfig, "runpublisher", true);
    if (!force && !runpublisher) {
        capturehandles.resize(0);
        return;
    }

    std::vector<std::string> tostart;
    std::vector<CameraCaptureHandlePtr> tempcapturehandles(cameranames.size()); CREATE_SAFE_DELETER_CAMERAHANDLES(tempcapturehandles);
    {
        boost::mutex::scoped_lock lock(_mutexCaptureHandles);
        for(size_t i = 0; i < cameranames.size(); ++i) {
            std::map<std::string, CameraCaptureHandleWeakPtr>::iterator itcapture = _mCameranameCaptureHandles.find(cameranames[i]);
            if( itcapture != _mCameranameCaptureHandles.end() ) {
                CameraCaptureHandlePtr phandle = itcapture->second.lock();
                if( !!phandle ) {
                    if (!force) {
                        MUJIN_LOG_DEBUG("do not start capturing for " << cameranames[i] << " as it is already running");
                        MUJIN_LOG_DEBUG("do not start capturing for " << cameranames[i]  << " (" << _GetHardwareId(cameranames[i]) << ") as it is already running");
                    } else {
                        tostart.push_back(cameranames[i]);
                    }
                    tempcapturehandles[i] = phandle; // should always copy the handle
                    continue;
                }
                // handle is invalid, all the threads released it
            }

            tostart.push_back(cameranames[i]);
            tempcapturehandles[i].reset(new CameraCaptureHandle(_pImagesubscriberManager, _mCameraNameHardwareId[cameranames[i]], cameranames[i]));
            _mCameranameCaptureHandles[cameranames[i]] = tempcapturehandles[i];
        }
    }
    if( tostart.size() > 0 ) {
        double timeout = 5.0;
        int numimages = -1;
        std::vector<std::string> ids = _GetHardwareIds(tostart);
        MUJIN_LOG_INFO("force=" << force << " Start capturing of cameras " << __GetString(tostart) << "(" << __GetString(ids) << ")");
        std::string extracaptureoptions;
        {
            boost::mutex::scoped_lock lock(_mutexRegion);
            extracaptureoptions = _GetExtraCaptureOptions(ids, _GetHardwareIds(cameranamestocheckocclusion), _visionserverconfig, _controllerIp, _binpickingTaskZmqPort, _slaverequestid, _mCameraNameHardwareId, _mCameranameActiveRegionname, _subscriberid, ignoreocclusion, _bindetectionMode);
        }
        try {
            _pImagesubscriberManager->StartCaptureThread(ids, timeout, numimages, extracaptureoptions);
        } catch(const std::exception& ex) {
            MUJIN_LOG_ERROR("caught exception " << ex.what());
            std::stringstream errss;
            errss << "failed to start capturing for cameras " << __GetString(tostart);
            MUJIN_LOG_ERROR(errss.str());
            MUJIN_LOG_WARN("need to clear out old images");

            if( GetMilliTime() - _pImagesubscriberManager->GetSubscribeStartedTimeStamp() > 10000 ) { // if 10s passed, then try to recreated the socket
                MUJIN_LOG_WARN("reset imagesubscriber");
                _pImagesubscriberManager->Reset();
            }
            _lastresultimages.resize(0);
            _lastcolorimages.resize(0);
            _lastdepthimages.resize(0);
            throw MujinVisionException(errss.str(), MVE_ImageAcquisitionError);
        }
    } else {
        MUJIN_LOG_INFO("capturing of cameras " << __GetString(cameranames) << " have already been started");
    }
    {
        boost::mutex::scoped_lock lock(_mutexCaptureHandles);
        capturehandles = tempcapturehandles;
        tempcapturehandles.clear();
    }
}

void MujinVisionManager::_DeleteCameraHandlesSafely(std::vector<CameraCaptureHandlePtr>& capturehandles)
{
    if( capturehandles.size() > 0 ) {
        boost::mutex::scoped_lock lock(_mutexCaptureHandles);
        capturehandles.resize(0);
    }
}

MujinVisionManager::MujinVisionManager(ImageSubscriberManagerPtr imagesubscribermanager, DetectorManagerPtr detectormanager, const unsigned int statusport, const unsigned int commandport, const unsigned configport, const std::string& configdir, const std::string& detectiondir)
{
    _bInitialized = false;
    _bShutdown = false;
    _bStopStatusThread = false;
    _bStopDetectionThread = false;
    _bStopUpdateEnvironmentThread = false;
    _bStopExecutionVerificationPointCloudThread = false;
    _bStopControllerMonitorThread = false;
    _bStopSendPointCloudObstacleToControllerThread = false;
    _bStopVisualizePointCloudThread = false;
    _bCancelCommand = false;
    _bExecutingUserCommand = false;
    _bIsControllerPickPlaceRunning = false;
    _bIsRobotOccludingSourceContainer = false;
    _bIsDetectionRunning = false;
    _bIsExecutionVerificationPointCloudRunning = false;
    _bIsVisualizePointcloudRunning = false;
    _bIsSendPointcloudRunning = false;
    _bIsEnvironmentUpdateRunning = false;
    _bIsControllerPickPlaceRunning = false;
    _bIsRobotOccludingSourceContainer = false;
    _bForceRequestDetectionResults = false;
    _bIsGrabbingTarget = false;
    _bIsGrabbingLastTarget = false;
    _bindetectionMode = "never";
    _numPickAttempt = 0;
    _binpickingstateTimestamp = 0;
    _lastGrabbedTargetTimestamp = 0;
    _orderNumber = 0;
    _numLeftInOrder = 0;
    _numLeftInSupply = 0;
    _placedInDest = 0;
    _tsStartDetection = 0;
    _tsLastEnvUpdate = 0;
    _resultTimestamp = 0;
    _resultImageStartTimestamp = _resultImageEndTimestamp = 0;
    _resultState = "{}";
    _pImagesubscriberManager = imagesubscribermanager;
    _pImagesubscriberManager->SetPreemptFn(boost::bind(&MujinVisionManager::_CheckPreemptSubscriber, this, _1));
    _bUseGrabbedTargeInfoInDetectionPreempt = false;
    _pDetectorManager = detectormanager;
    _zmqcontext.reset(new zmq::context_t(8));
    _statusport = statusport;
    _commandport = commandport;
    _configport = configport;
    _configdir = configdir;
    _detectiondir = detectiondir;
    _binpickingTaskZmqPort = 0;
    _binpickingTaskHeartbeatPort = 0;
    _binpickingTaskHeartbeatTimeout = 10;
    _lastocclusionTimestamp = 0;
    _lastSendPointCloudObstacleTimestamp = 0;
    _controllerCommandTimeout = 10.0;
    _locale = "en_US";
    _detectorconfig = "";
    _imagesubscriberconfig = "";
    _tasktype = "";
    _targetname = "";
    _targeturi = "";
    _targetupdatename = "detected_";
    _userinfo_json = "";
    _slaverequestid = "";
    _controllerIp = "";
    _defaultTaskParameters = "";
    _filteringsubsample = 1;
    _filteringvoxelsize = 0.001 * 1000;
    _filteringstddev = 0.01;
    _filteringnumnn = 1;
    _detectionRegionName = "";
    char hostname[150];
    __GetMachineName(hostname);
    _commandMessageQueue.push("");
    _commandErrorQueue.push("");
    _configMessageQueue.push("");
    _configErrorQueue.push("");
    _detectorMessageQueue.push("");
    _detectorErrorQueue.push("");
    _updateenvironmentMessageQueue.push("");
    _updateenvironmentErrorQueue.push("");
    _controllermonitorMessageQueue.push("");
    _controllermonitorErrorQueue.push("");
    _sendpointcloudMessageQueue.push("");
    _sendpointcloudErrorQueue.push("");
    _StartStatusThread(statusport);
    _StartCommandThread(commandport, CDI_Command);
    _StartCommandThread(configport, CDI_Configure);
}

MujinVisionManager::~MujinVisionManager()
{
    Destroy();
}

void MujinVisionManager::Destroy()
{
    MUJIN_LOG_DEBUG("Destroying MujinVisionManager");
    Shutdown();
    for(size_t i = 0; i < _mPortCommandThread.size(); ++i) {
        _StopCommandThread(i);
    }
}

void MujinVisionManager::Shutdown()
{
    _bShutdown=true;
    _StopStatusThread();
    _StopCommandThread(CDI_Command); // command
    // do not stop config command thread here as this method can be called from there
    // Destroy() will stop config command thread
    // do not stop threads started by the command thread
}

bool MujinVisionManager::IsShutdown()
{
    return _bShutdown;
}

void MujinVisionManager::GetConfig(const std::string& type, std::string& config)
{
    if (type == "detector") {
        config = _detectorconfig;
    } else if (type == "imagesubscriber") {
        config = _imagesubscriberconfig;
    }
}

std::string MujinVisionManager::_GetConfigFileName(const std::string& type, const std::string& configname)
{
    std::string filename = _configdir + "/" + type;
    if (configname.size() > 0) {
        filename += "-" + configname;
    }
    filename += ".json";
    return filename;
}

void MujinVisionManager::_LoadConfig(const std::string& filename, std::string& content)
{
    if (!boost::filesystem::exists(filename)) {
        std::stringstream errss;
        errss << "file " << filename << " does not exist!";
        throw MujinVisionException("file " + filename + " does not exist!", MVE_InvalidArgument);
    } else {
        std::stringstream ss;
        ss << "Loading file from " << filename;
        MUJIN_LOG_DEBUG(ss.str());
        std::ifstream t(filename.c_str());
        t.seekg(0, std::ios::end);
        content.reserve(t.tellg());
        t.seekg(0, std::ios::beg);
        content.assign((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    }
}

void MujinVisionManager::SaveConfig(const std::string& type, const std::string& visionmanagerconfigname, const std::string& config)
{
    std::string filename = _GetConfigFileName(type, visionmanagerconfigname);
    if (!boost::filesystem::exists(filename)) {
        throw MujinVisionException(filename+" does not exist.", MVE_InvalidArgument);
    }
    std::string content;
    if (config == "") {
        GetConfig(type, content);
    } else {
        ValidateJsonString(config);
        content = config;
    }
    std::ofstream out(filename.c_str());
    out << content;
    out.close();
}

void MujinVisionManager::_SetStatus(ThreadType tt, ManagerStatus status, const std::string& msg, const std::string& err, const bool allowInterrupt)
{
    if (status == MS_Preempted) {
        {
            boost::mutex::scoped_lock lock(_mutexCancelCommand);
            _bCancelCommand = false;
        }
    }
    if(_bCancelCommand && allowInterrupt) {
        throw mujinclient::UserInterruptException("Cancelling command.");
    }
    std::stringstream ss;
    ss << GetMilliTime() << " " << _GetManagerStatusString(status) << ": " << msg;
    if (msg == "") {
        MUJIN_LOG_DEBUG(ss.str());
    } else {
        MUJIN_LOG_INFO(ss.str());
    }
    boost::mutex::scoped_lock lock(_mutexStatusQueue);
    std::string cmdmsg;
    std::string cmderr;
    std::string cfgmsg;
    std::string cfgerr;
    std::string detectormsg;
    std::string detectorerr;
    std::string updateenvmsg;
    std::string updateenverr;
    std::string controllermonmsg;
    std::string controllermonerr;
    std::string sendpclmsg;
    std::string sendpclerr;
    std::string visualizepcmsg;
    std::string visualizepcerr;
    std::string sendexecvpcmsg;
    std::string sendexecvpcerr;
    std::string statusmsg;
    std::string statuserr;
    switch (tt) {
    case TT_Command:
        cmdmsg = msg;
        cmderr = err;
        break;
    case TT_Config:
        cfgmsg = msg;
        cfgerr = err;
        break;
    case TT_Detector:
        detectormsg = msg;
        detectorerr = err;
        break;
    case TT_UpdateEnvironment:
        updateenvmsg = msg;
        updateenverr = err;
        break;
    case TT_ControllerMonitor:
        controllermonmsg = msg;
        controllermonerr = err;
        break;
    case TT_SendPointcloudObstacle:
        sendpclmsg = msg;
        sendpclerr = err;
        break;
    case TT_VisualizePointCloud:
        visualizepcmsg = msg;
        visualizepcerr = err;
        break;
    case TT_SendExecutionVerificationPointCloud:
        sendexecvpcmsg = msg;
        sendexecvpcerr = err;
        break;
    default:
        std::stringstream ss;
        ss << "unknown thread type " << tt << " cannot update status";
        throw MujinVisionException(ss.str(), MVE_Failed);
    }
    _statusQueue.push(status);
    _commandMessageQueue.push(cmdmsg);
    _commandErrorQueue.push(cmderr);
    _configMessageQueue.push(cfgmsg);
    _configErrorQueue.push(cfgerr);
    _detectorMessageQueue.push(detectormsg);
    _detectorErrorQueue.push(detectorerr);
    _updateenvironmentMessageQueue.push(updateenvmsg);
    _updateenvironmentErrorQueue.push(updateenverr);
    _controllermonitorMessageQueue.push(controllermonmsg);
    _controllermonitorErrorQueue.push(controllermonerr);
    _sendpointcloudMessageQueue.push(sendpclmsg);
    _sendpointcloudErrorQueue.push(sendpclerr);
    _visualizepointcloudMessageQueue.push(visualizepcmsg);
    _visualizepointcloudErrorQueue.push(visualizepcerr);
    _sendexecverificationMessageQueue.push(sendexecvpcmsg);
    _sendexecverificationErrorQueue.push(sendexecvpcerr);
    _timestampQueue.push(GetMilliTime());
    ss.clear();
    ss << "updated status queue: " << status << " " << cmdmsg << " " << cmderr << " " << cfgmsg << " " << cfgerr << " " << detectormsg << " " << detectorerr << " " << updateenvmsg << " " << updateenverr << " " << controllermonmsg << " " << controllermonerr << " " << sendpclmsg << " " << sendpclerr << " " << GetMilliTime();
    MUJIN_LOG_DEBUG(ss.str());

}

void MujinVisionManager::_SetDetectorStatusMessage(const std::string& msg, const std::string& err)
{
    if (_statusQueue.size()>0) {
        _SetStatus(TT_Detector, _statusQueue.front(), msg, err);
    } else {
        throw MujinVisionException("VisionManager is in invalid state.", MVE_Failed);
    }
}

void MujinVisionManager::_SetStatusMessage(ThreadType tt, const std::string& msg, const std::string& err)
{
    if (_statusQueue.size()>0) {
        _SetStatus(tt, _statusQueue.front(), msg, err);
    } else {
        throw MujinVisionException("VisionManager is in invalid state.", MVE_Failed);
    }
}

void MujinVisionManager::_StartStatusThread(const unsigned int port, const unsigned int ms)
{
    _bStopStatusThread = false;
    _pStatusThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_RunStatusThread, this, port, ms)));
}

void MujinVisionManager::_StopStatusThread()
{
    if (!!_pStatusThread) {
        {
            boost::mutex::scoped_lock lock(_mutexStatusQueue);
            _bStopStatusThread = true;
        }
        _pStatusThread->join();
        _pStatusThread.reset();
        MUJIN_LOG_DEBUG("Stopped status thread");
    }
}

void MujinVisionManager::_StartCommandThread(const unsigned int port, int commandindex)
{
    _mPortStopCommandThread[commandindex] = false;
    _mPortCommandThread[commandindex].reset(new boost::thread(boost::bind(&MujinVisionManager::_RunCommandThread, this, port, commandindex)));
}

void MujinVisionManager::_StopCommandThread(int commandindex)
{
    if (_mPortStopCommandThread[commandindex] == false) {
        std::stringstream ss;
        ss << "stopping command thread index " << commandindex;
        MUJIN_LOG_DEBUG(ss.str());
        _mPortStopCommandThread[commandindex] = true;
        _mPortCommandThread[commandindex]->join();
        _mPortCommandThread[commandindex].reset();
    }
    std::stringstream ss;
    ss << "Stopped command thread index " << commandindex;
    MUJIN_LOG_DEBUG(ss.str());
}

void MujinVisionManager::_ExecuteConfigurationCommand(const rapidjson::Document& commandjson, rapidjson::Document& resultjson)
{
    std::string command = GetJsonValueByKey<std::string>(commandjson, "command");
    if (command == "Ping") {
        SetJsonValueByKey(resultjson, "timestamp", GetMilliTime());
    } else if (command == "Cancel") {
        boost::mutex::scoped_lock lock(_mutexCancelCommand);
        if (_bExecutingUserCommand) { // only cancel when user command is being executed
            _bCancelCommand = true;
            _SetStatus(TT_Config, MS_Preempting, "", "", false);
        } else {
            _SetStatusMessage(TT_Config, "No command is being excuted, do nothing.");
        }
        SetJsonValueByKey(resultjson, "status", _GetManagerStatusString(MS_Preempting));
    } else if (command == "Quit") {
        // throw exception, shutdown gracefully
        Shutdown();
        throw mujinclient::UserInterruptException("User requested exit.");
    } else {
        std::string errstr = "received unknown config command " + command;
        MUJIN_LOG_ERROR(errstr);
        throw MujinVisionException(errstr, MVE_CommandNotSupported);
    }
}

void MujinVisionManager::_ExecuteUserCommand(const rapidjson::Document& commandjson, rapidjson::Document& resultjson) {
    uint64_t starttime = GetMilliTime();
    _SetStatus(TT_Command, MS_Active);
    {
        // only one command thread is running, so _bExecutingUserCommand must be false at this point, and _bCancelCommand must not be true, therefore no race condition of setting _bCancelCommand from true to false
        boost::mutex::scoped_lock lock(_mutexCancelCommand);
        _bCancelCommand = false;
        _bExecutingUserCommand = true;
    }
    std::string command =  GetJsonValueByKey<std::string>(commandjson, "command");
    if (command == "StartDetectionLoop" ||
        command == "StopDetectionLoop" ||
        command == "IsDetectionRunning" ||
        command == "SendPointCloudObstacleToController" ||
        (command.size() >= 3 && command.substr(0,3) == "Get")) {
        // these commands can be called at any time

        if (command == "StartDetectionLoop") {
            if (!commandjson.HasMember("regionname")) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = GetJsonValueByKey<std::string>(commandjson, "regionname");
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() before calling " + command, MVE_NotInitialized);
            }

            std::vector<std::string> cameranames = GetJsonValueByKey<std::vector<std::string> >(commandjson, "cameranames");
            std::vector<std::string> evcamnames = GetJsonValueByKey<std::vector<std::string> >(commandjson, "executionverificationcameranames");
            bool ignoreocclusion = GetJsonValueByKey<bool>(commandjson, "ignoreocclusion", false);
            bool stoponleftinorder = GetJsonValueByKey<bool>(commandjson, "stoponleftinorder", false);
            unsigned int fetchimagetimeout = GetJsonValueByKey<unsigned int>(commandjson, "fetchimagetimeout", 0);
            std::string obstaclename = GetJsonValueByKey<std::string>(commandjson, "obstaclename", "__dynamicobstacle__");
            unsigned long long detectionstarttimestamp = GetJsonValueByKey<unsigned long long>(commandjson, "detectionstarttimestamp", 0);
            std::string locale = GetJsonValueByKey<std::string>(commandjson, "locale", "en_US");
            unsigned int maxnumfastdetection = GetJsonValueByKey<unsigned int>(commandjson, "maxnumfastdetection", GetJsonValueByKey<unsigned int>(_visionserverconfig, "maxnumfastdetection", 1));
            unsigned int maxnumdetection = GetJsonValueByKey<unsigned int>(commandjson, "maxnumdetection", 0);
            std::string targetupdatename = GetJsonValueByKey<std::string>(commandjson, "targetupdatename", "");
            _locale = locale;
            Transform tworldresultoffset;
            if (commandjson.HasMember("worldresultoffsettransform")) {
                tworldresultoffset = GetTransform(commandjson["worldresultoffsettransform"]);
            } else {
                tworldresultoffset.trans[0] = 0;
                tworldresultoffset.trans[1] = 0;
                tworldresultoffset.trans[2] = 0;
                tworldresultoffset.rot[0] = 1;
                tworldresultoffset.rot[1] = 0;
                tworldresultoffset.rot[2] = 0;
                tworldresultoffset.rot[3] = 0;
            }
            bool sendVerificationPointCloud = GetJsonValueByKey<bool>(commandjson, "sendVerificationPointCloud", true);
            unsigned int numthreads = GetJsonValueByKey<unsigned int>(commandjson, "numthreads", 0);
            std::string cycleindex = GetJsonValueByKey<std::string>(commandjson, "cycleindex", "0");
            if (IsDetectionRunning()) {
                MUJIN_LOG_WARN("detection is already running, do nothing.");
            } else {
                StartDetectionLoop(regionname, cameranames, evcamnames, tworldresultoffset, ignoreocclusion, fetchimagetimeout, obstaclename, detectionstarttimestamp, locale, maxnumfastdetection, maxnumdetection, sendVerificationPointCloud, stoponleftinorder, targetupdatename, numthreads, cycleindex);
            }
            SetJsonValueByKey(resultjson, "computationtime", GetMicroTime() - starttime);
        } else if (command == "StopDetectionLoop") {
            StopDetectionLoop();
            SetJsonValueByKey(resultjson, "computationtime", GetMilliTime() - starttime);
        } else if (command == "IsDetectionRunning") {
            SetJsonValueByKey(resultjson, "isdetectionrunning", IsDetectionRunning());
        } else if (command == "GetLatestDetectedObjects") {
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() before calling " + command, MVE_NotInitialized);
            }
            std::vector<DetectedObjectPtr> detectedobjectsworld;
            std::string resultstate;
            std::vector<Real> points;
            bool returnpoints = GetJsonValueByKey<bool>(commandjson, "returnpoints", false);
            unsigned long long imageStartTimestamp, imageEndTimestamp;
            GetLatestDetectedObjects(detectedobjectsworld, resultstate, points, imageStartTimestamp, imageEndTimestamp, returnpoints);
            SetJsonValueByKey(resultjson, "detectedobjects", detectedobjectsworld);
            if( resultstate.size() > 0 ) {
                rapidjson::Document resultstatejson;
                ParseJson(resultstatejson, resultstate);
                SetJsonValueByKey(resultjson, "state", resultstatejson);
            }
            else {
                rapidjson::Document resultstatejson(rapidjson::kObjectType);
                SetJsonValueByKey(resultjson, "state", resultstatejson);
            }
            if (returnpoints) {
                SetJsonValueByKey(resultjson, "points", points);
            }
            SetJsonValueByKey(resultjson, "computationtime", GetMicroTime()-starttime);
            SetJsonValueByKey(resultjson, "imageStartTimestamp", imageStartTimestamp);
            SetJsonValueByKey(resultjson, "imageEndTimestamp", imageEndTimestamp);
        } else if (command == "GetCameraId") {
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            std::string cameraname = GetJsonValueByKey<std::string>(commandjson, "cameraname");
            std::string cameraid;
            GetCameraId(cameraname, cameraid);
            SetJsonValueByKey(resultjson, "cameraid", cameraid);
            SetJsonValueByKey(resultjson, "computationtime", GetMicroTime()-starttime);
        } else if (command == "GetVisionmanagerConfig") {
            std::string config;
            GetConfig("visionmanager", config); // is this never used?
            rapidjson::Document configjson;
            ParseJson(configjson, config);
            SetJsonValueByKey(resultjson, "visionmanagerconfig", configjson);
            SetJsonValueByKey(resultjson, "computationtime", GetMicroTime()-starttime);
        } else if (command == "GetDetectorConfig") {
            std::string config;
            rapidjson::Document configjson;
            GetConfig("detector", config);
            ParseJson(configjson, config);
            SetJsonValueByKey(resultjson, "detectorconfigname", configjson);
            SetJsonValueByKey(resultjson, "computationtime", GetMicroTime()-starttime);
        } else if (command == "GetImagesubscriberConfig") {
            std::string config;
            rapidjson::Document configjson;
            GetConfig("imagesubscriber", config);
            ParseJson(configjson, config);
            SetJsonValueByKey(resultjson, "imagesubscriberconfigname", configjson);
            SetJsonValueByKey(resultjson, "computationtime", GetMicroTime()-starttime);
        } else if (command == "GetConfigPort") {
            unsigned int port;
            GetConfigPort(port);
            SetJsonValueByKey(resultjson, "configport", (int)(port));
            SetJsonValueByKey(resultjson, "computationtime", GetMicroTime()-starttime);
        } else if (command == "GetStatusPort") {
            unsigned int port;
            GetStatusPort(port);
            SetJsonValueByKey(resultjson, "configport", (int)(port));
            SetJsonValueByKey(resultjson, "computationtime", GetMicroTime()-starttime);
        } else if (command == "SendPointCloudObstacleToController") {
            if (!commandjson.HasMember("regionname")) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = GetJsonValueByKey<std::string>(commandjson, "regionname");
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }

            std::vector<std::string> cameranames = GetJsonValueByKey<std::vector<std::string> >(commandjson, "cameranames");
            std::vector<DetectedObjectPtr> detectedobjects = GetJsonValueByKey<std::vector<DetectedObjectPtr> >(commandjson, "detectedobjects");
            unsigned long long newerthantimestamp = GetJsonValueByKey<unsigned long long>(commandjson, "newerthantimestamp", 0);
            unsigned int fetchimagetimeout = GetJsonValueByKey<unsigned int>(commandjson, "fetchimagetimeout");
            std::string obstaclename = GetJsonValueByKey<std::string>(commandjson, "obstaclename", "__dynamicobstacle__");
            bool fast = GetJsonValueByKey<bool>(commandjson, "fast", false);
            bool request = GetJsonValueByKey<bool>(commandjson, "request", true);
            bool async = GetJsonValueByKey<bool>(commandjson, "async", false);
            std::string locale = GetJsonValueByKey<std::string>(commandjson, "locale", "en_US");
            _locale = locale;
            SendPointCloudObstacleToController(regionname, cameranames, detectedobjects, newerthantimestamp, fetchimagetimeout, obstaclename, fast, request, async, locale);
            SetJsonValueByKey(resultjson, "computationtime", GetMicroTime()-starttime);
        }
    } else if (!!_pDetectionThread && _bIsDetectionRunning) {
        throw MujinVisionException("Cannot execute " + command + " while detection thread is running, please stop it first.", MVE_Busy);
    } else {
        // these commands can only be called when detection is not running

        if (command == "Initialize") {
            if (_bInitialized) {
                _SetStatusMessage(TT_Command, "Vision manager was initialized, de-initialize it first.");
                _DeInitialize();
            } else {
                _bInitialized = true;
            }
            std::string locale = GetJsonValueByKey<std::string>(commandjson, "locale", "en_US");
            std::vector<std::string> targetdetectionarchiveurls;
            if (commandjson.HasMember("targetdetectionarchiveurl")) {
                try {
                    const rapidjson::Value& targetdetectionarchiveurljson = commandjson["targetdetectionarchiveurl"];
                    if (targetdetectionarchiveurljson.IsString()) {
                        MUJIN_LOG_DEBUG("parsing targetdetectionarchiveurl as a string");
                        targetdetectionarchiveurls.push_back(targetdetectionarchiveurljson.GetString());
                    } else {
                        MUJIN_LOG_DEBUG("parsing targetdetectionarchiveurl as a list");
                        targetdetectionarchiveurls = GetJsonValueByKey<std::vector<std::string> >(commandjson, "targetdetectionarchiveurl");
                    }
                } catch (std::exception& e) {
                    MUJIN_LOG_ERROR("failed to parse targetdetectionarchiveurl");
                }
            }

            Initialize(GetJsonValueByKey<std::string>(commandjson, "visionmanagerconfig"),
                       GetJsonValueByKey<std::string>(commandjson, "detectorconfigname"),
                       GetJsonValueByKey<std::string>(commandjson, "imagesubscriberconfig"),
                       GetJsonValueByKey<std::string>(commandjson, "mujinControllerIp", ""),
                       GetJsonValueByKey<unsigned int>(commandjson, "mujinControllerPort", 0),
                       GetJsonValueByKey<std::string>(commandjson, "mujinControllerUsernamePass"),
                       GetJsonValueByKey<std::string>(commandjson, "defaultTaskParameters"),
                       GetJsonValueByKey<std::string>(commandjson, "containerParameters"),
                       GetJsonValueByKey<unsigned int>(commandjson, "binpickingTaskZmqPort"),
                       GetJsonValueByKey<unsigned int>(commandjson, "binpickingTaskHeartbeatPort"),
                       GetJsonValueByKey<double>(commandjson, "binpickingTaskHeartbeatTimeout"),
                       GetJsonValueByKey<std::string>(commandjson, "binpickingTaskScenePk"),
                       GetJsonValueByKey<std::string>(commandjson, "targetname"),
                       GetJsonValueByKey<std::string>(commandjson, "targeturi"),
                       GetJsonValueByKey<std::string>(commandjson, "targetupdatename"),
                       GetJsonValueByKey<std::string>(commandjson, "streamerIp"),
                       GetJsonValueByKey<unsigned int>(commandjson, "streamerPort"),
                       GetJsonValueByKey<std::string>(commandjson, "tasktype","binpicking"),
                       GetJsonValueByKey<unsigned int>(commandjson, "controllertimeout", 10),
                       GetJsonValueByKey<std::string>(commandjson, "locale", "en_US"),
                       GetJsonValueByKey<std::string>(commandjson, "slaverequestid", ""),
                       targetdetectionarchiveurls);
            SetJsonValueByKey(resultjson, "computationtime", GetMicroTime()-starttime);
        } else if (command == "DetectObjects") {
            if (!commandjson.HasMember("regionname") == 0) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = GetJsonValueByKey<std::string>(commandjson, "regionname");
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            std::vector<std::string> cameranames = GetJsonValueByKey<std::vector<std::string> >(commandjson, "cameranames");
            bool ignoreocclusion = GetJsonValueByKey<bool>(commandjson, "ignoreocclusion", false);
            unsigned long long newerthantimestamp = GetJsonValueByKey<unsigned long long>(commandjson, "newerthantimestamp", 0);
            unsigned int fetchimagetimeout = GetJsonValueByKey<unsigned int>(commandjson, "fetchimagetimeout", 0);
            bool fastdetection = GetJsonValueByKey<bool>(commandjson, "fastdetection", false);
            std::string bindetectionMode = GetJsonValueByKey<std::string>(commandjson, "bindetectionmode", "never");
            bool bindetection = false;
            if (bindetectionMode == "always" || bindetectionMode == "once") {
                bindetection = true;
            }
            std::vector<DetectedObjectPtr> detectedobjects;
            std::string resultstate;
            unsigned long long imageStartTimestamp=0, imageEndTimestamp=0;
            int isContainerPresent=-1;
            _bUseGrabbedTargeInfoInDetectionPreempt = false;
            DetectObjects(regionname, cameranames, detectedobjects, resultstate, imageStartTimestamp, imageEndTimestamp, isContainerPresent, ignoreocclusion, newerthantimestamp, fetchimagetimeout, fastdetection, bindetection);
            SetJsonValueByKey(resultjson, "objects", detectedobjects);
            SetJsonValueByKey(resultjson, "imagestarttime", imageStartTimestamp);
            SetJsonValueByKey(resultjson, "imageendtime", imageEndTimestamp);
            SetJsonValueByKey(resultjson, "objects", detectedobjects);
            SetJsonValueByKey(resultjson, "iscontainerpresent", isContainerPresent);
            rapidjson::Document resultstatejson;
            ParseJson(resultstatejson, resultstate);
            SetJsonValueByKey(resultjson, "state", resultstatejson);
            SetJsonValueByKey(resultjson, "imagestarttime", imageStartTimestamp);
            SetJsonValueByKey(resultjson, "imageendtime", imageEndTimestamp);
            SetJsonValueByKey(resultjson, "computationtime", GetMilliTime() - starttime);
        } else if (command == "VisualizePointCloudOnController") {
            if (!_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            std::string regionname = GetJsonValueByKey<std::string>(commandjson, "regionname");
            std::vector<std::string> cameranames = GetJsonValueByKey<std::vector<std::string> >(commandjson, "cameranames");
            bool ignoreocclusion = GetJsonValueByKey<bool>(commandjson, "ignoreocclusion", false);
            unsigned long long newerthantimestamp = GetJsonValueByKey<unsigned long long>(commandjson, "newerthantimestamp", 0);
            unsigned int fetchimagetimeout = GetJsonValueByKey<unsigned int>(commandjson, "fetchimagetimeout", 0);
            bool request = GetJsonValueByKey<bool>(commandjson, "request", true);
            VisualizePointCloudOnController(regionname, cameranames, ignoreocclusion, newerthantimestamp, fetchimagetimeout, request);
            SetJsonValueByKey(resultjson, "computationtime", GetMilliTime() - starttime);
        } else if (command == "ClearVisualizationOnController") {
            if (!_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            ClearVisualizationOnController();
            SetJsonValueByKey(resultjson, "computationtime", GetMilliTime() - starttime);
        } else if (command == "StartVisualizePointCloudThread") {
            if (!_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            std::string regionname = GetJsonValueByKey<std::string>(commandjson, "regionname");
            std::vector<std::string> cameranames = GetJsonValueByKey<std::vector<std::string> >(commandjson, "cameranames");
            double pointsize = GetJsonValueByKey<double>(commandjson, "pointsize", 5);
            bool ignoreocclusion = GetJsonValueByKey<bool>(commandjson, "ignoreocclusion", false);
            unsigned long long newerthantimestamp = GetJsonValueByKey<unsigned long long>(commandjson, "newerthantimestamp", 0);
            unsigned int fetchimagetimeout = GetJsonValueByKey<unsigned int>(commandjson, "fetchimagetimeout", 0);
            bool request = GetJsonValueByKey<bool>(commandjson, "request", true);
            StartVisualizePointCloudThread(regionname, cameranames, pointsize, ignoreocclusion, newerthantimestamp, fetchimagetimeout, request);
            SetJsonValueByKey(resultjson, "computationtime", GetMilliTime() - starttime);
        } else if (command == "StopVisualizePointCloudThread") {
            // do not need controllerclient
            // if (!_pBinpickingTask) {
            //     throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            // }
            StopVisualizePointCloudThread();
            SetJsonValueByKey(resultjson, "computationtime", GetMilliTime() - starttime);
        } else if (command == "UpdateDetectedObjects") {
            if (!commandjson.HasMember("regionname")) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = GetJsonValueByKey<std::string>(commandjson, "regionname");
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }

            std::vector<DetectedObjectPtr> detectedobjects = GetJsonValueByKey<std::vector<DetectedObjectPtr> >(commandjson, "detectedobjects");
            std::string resultstate = GetJsonValueByKey<std::string>(commandjson, "state");
            bool sendtocontroller = GetJsonValueByKey<bool>(commandjson, "sendtocontroller", true);
            UpdateDetectedObjects(detectedobjects, resultstate, sendtocontroller);
            SetJsonValueByKey(resultjson, "computationtime", GetMilliTime() - starttime);
        } else if (command == "SyncRegion") {
            if (!commandjson.HasMember("regionname")) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = GetJsonValueByKey<std::string>(commandjson, "regionname");
            if (!_pDetector || !_pBinpickingTask || !_pImagesubscriberManager) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            SyncRegion(regionname);
            SetJsonValueByKey(resultjson, "computationtime", GetMilliTime() - starttime);
        } else if (command == "SyncCameras") {
            std::string regionname = GetJsonValueByKey<std::string>(commandjson, "regionname");
            if (!_pBinpickingTask || !_pImagesubscriberManager) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            std::vector<std::string> cameranames = GetJsonValueByKey<std::vector<std::string> >(commandjson, "cameranames");
            SyncCameras(regionname, cameranames);
            SetJsonValueByKey(resultjson, "computationtime", GetMilliTime() - starttime);
        } else if (command == "SaveVisionmanagerConfig") {
            if (!commandjson.HasMember("visionmanagerconfigname")) {
                throw MujinVisionException("visionmanagerconfigname is not specified.", MVE_InvalidArgument);
            }
            if (!commandjson.HasMember("config")) {
                throw MujinVisionException("config is not specified.", MVE_InvalidArgument);
            }
            SaveConfig("visionmanager", GetJsonValueByKey<std::string>(commandjson, "visionmanagerconfigname"), GetJsonValueByKey<std::string>(commandjson, "config"));
            SetJsonValueByKey(resultjson, "computationtime", GetMilliTime() - starttime);
        } else if (command == "SaveDetectorConfig") {
            if (!commandjson.HasMember("detectorconfigname")) {
                throw MujinVisionException("detectorconfigname is not specified.", MVE_InvalidArgument);
            }
            if (!commandjson.HasMember("config")) {
                throw MujinVisionException("config is not specified.", MVE_InvalidArgument);
            }
            SaveConfig("detector", GetJsonValueByKey<std::string>(commandjson, "detectorconfigname"), GetJsonValueByKey<std::string>(commandjson, "config"));
            SetJsonValueByKey(resultjson, "computationtime", GetMilliTime() - starttime);
        } else if (command == "SaveImagesubscriberConfig") {
            if (!commandjson.HasMember("imagesubscriberconfigname")) {
                throw MujinVisionException("imagesubsriberconfigname is not specified.", MVE_InvalidArgument);
            }
            if (!commandjson.HasMember("config")) {
                throw MujinVisionException("config is not specified.", MVE_InvalidArgument);
            }
            SaveConfig("imagesubscriber", GetJsonValueByKey<std::string>(commandjson, "imagesubscriberconfigname"), GetJsonValueByKey<std::string>(commandjson, "config"));
            SetJsonValueByKey(resultjson, "computationtime", GetMilliTime() - starttime);
        } else {
            if(_mNameCommand.find(command) == _mNameCommand.end()) {
                std::stringstream ss;
                ss << "Received unknown command " << command << ".";
                throw MujinVisionException(ss.str(), MVE_CommandNotSupported);
            } else {
                boost::shared_ptr<CustomCommand> customcommand = _mNameCommand[command];
                std::stringstream customresultss;
                rapidjson::Document customresultjson(rapidjson::kObjectType);
                customcommand->fn(this, commandjson, customresultjson);
                SetJsonValueByKey(resultjson, "customresult", customresultjson);
                SetJsonValueByKey(resultjson, "computationtime", GetMilliTime() - starttime);
            }
        }
    }
    _SetStatus(TT_Command, MS_Pending);

}

bool MujinVisionManager::IsDetectionRunning()
{
    return !!_pDetectionThread && _bIsDetectionRunning;
}

void MujinVisionManager::_RunStatusThread(const unsigned int port, const unsigned int ms)
{
    StatusPublisherPtr pStatusPublisher;
    _SetStatus(TT_Command, MS_Pending); // to initialize the status messages

    std::vector<ManagerStatus> vstatus;
    std::vector<std::string> vcfgmsg, vcmdmsg, vdetectormsg, vupdateenvmsg, vcontrollermonmsg, vsendpclmsg;
    std::vector<std::string> vcfgerr, vcmderr, vdetectorerr, vupdateenverr, vcontrollermonerr, vsendpclerr;
    std::vector<unsigned long long> vtimestamp;
    while (!_bStopStatusThread) {

        if( !pStatusPublisher ) {
            try {
                pStatusPublisher.reset(new StatusPublisher(_zmqcontext, port));
                std::stringstream ss;
                ss << "Started status thread (port: " << port << ").";
                MUJIN_LOG_DEBUG(ss.str());
            }
            catch(const std::exception& ex) {
                std::stringstream ss;
                ss << "could not create publisher on port=" << port << ", perhaps another visionmanager instance is running? " << ex.what();
                MUJIN_LOG_ERROR(ss.str());
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000)); // sleep a little to stop flooding the messages
                continue;
            }
        }

        {
            boost::mutex::scoped_lock lock(_mutexStatusQueue);
            vstatus.resize(0);
            vtimestamp.resize(0);
            vcmdmsg.resize(0);
            vcfgmsg.resize(0);
            vdetectormsg.resize(0);
            vupdateenvmsg.resize(0);
            vcontrollermonmsg.resize(0);
            vsendpclmsg.resize(0);
            vcmderr.resize(0);
            vcfgerr.resize(0);
            vdetectorerr.resize(0);
            vupdateenverr.resize(0);
            vcontrollermonerr.resize(0);
            vsendpclerr.resize(0);
            while (_statusQueue.size() > 1) {
                vstatus.push_back(_statusQueue.front());
                _statusQueue.pop();
                vtimestamp.push_back(_timestampQueue.front());
                _timestampQueue.pop();
            }
            if (vstatus.size() == 0) {
                vstatus.push_back(_statusQueue.front());
                vtimestamp.push_back(_timestampQueue.front());
            }
            while (_commandMessageQueue.size() > 1) {
                vcmdmsg.push_back(_commandMessageQueue.front());
                _commandMessageQueue.pop();
                vcmderr.push_back(_commandErrorQueue.front());
                _commandErrorQueue.pop();
            }
            if (vcmdmsg.size() == 0) {
                vcmdmsg.push_back(_commandMessageQueue.front());
                vcmderr.push_back(_commandErrorQueue.front());
            }
            while (_configMessageQueue.size() > 1) {
                vcfgmsg.push_back(_configMessageQueue.front());
                _configMessageQueue.pop();
                vcfgerr.push_back(_configErrorQueue.front());
                _configErrorQueue.pop();
            }
            if (vcfgmsg.size() == 0) {
                vcfgmsg.push_back(_configMessageQueue.front());
                vcfgerr.push_back(_configErrorQueue.front());
            }
            while (_detectorMessageQueue.size() > 1) {
                vdetectormsg.push_back(_detectorMessageQueue.front());
                _detectorMessageQueue.pop();
                vdetectorerr.push_back(_detectorErrorQueue.front());
                _detectorErrorQueue.pop();
            }
            if (vdetectormsg.size() == 0) {
                vdetectormsg.push_back(_detectorMessageQueue.front());
                vdetectorerr.push_back(_detectorErrorQueue.front());
            }
            while (_updateenvironmentMessageQueue.size() > 1) {
                vupdateenvmsg.push_back(_updateenvironmentMessageQueue.front());
                _updateenvironmentMessageQueue.pop();
                vupdateenverr.push_back(_updateenvironmentErrorQueue.front());
                _updateenvironmentErrorQueue.pop();
            }
            if (vupdateenvmsg.size() == 0) {
                vupdateenvmsg.push_back(_updateenvironmentMessageQueue.front());
                vupdateenverr.push_back(_updateenvironmentErrorQueue.front());
            }
            while (_controllermonitorMessageQueue.size() > 1) {
                vcontrollermonmsg.push_back(_controllermonitorMessageQueue.front());
                _controllermonitorMessageQueue.pop();
                vcontrollermonerr.push_back(_controllermonitorErrorQueue.front());
                _controllermonitorErrorQueue.pop();
            }
            if (vcontrollermonmsg.size() == 0) {
                vcontrollermonmsg.push_back(_controllermonitorMessageQueue.front());
                vcontrollermonerr.push_back(_controllermonitorErrorQueue.front());
            }
            while (_sendpointcloudMessageQueue.size() > 1) {
                vsendpclmsg.push_back(_sendpointcloudMessageQueue.front());
                _sendpointcloudMessageQueue.pop();
                vsendpclerr.push_back(_sendpointcloudErrorQueue.front());
                _sendpointcloudErrorQueue.pop();
            }
            if (vsendpclmsg.size() == 0) {
                vsendpclmsg.push_back(_sendpointcloudMessageQueue.front());
                vsendpclerr.push_back(_sendpointcloudErrorQueue.front());
            }
            // TODO add VisualizePointCloud and SendExecutionVerificationPointCloud messages
        }
        for (unsigned int i=0; i<vstatus.size(); i++) {
            //MUJIN_LOG_ERROR(_GetStatusJsonString(vtimestamp.at(i), _GetManagerStatusString(vstatus.at(i)), vcmdmsg.at(i), vcmderr.at(i), vcfgmsg.at(i), vcfgerr.at(i), vdetectormsg.at(i), vdetectorerr.at(i), vupdateenvmsg.at(i), vupdateenverr.at(i), vcontrollermonmsg.at(i), vcontrollermonerr.at(i), vsendpclmsg.at(i), vsendpclerr.at(i)));
            pStatusPublisher->Publish(_GetStatusJsonString(vtimestamp.at(i), _GetManagerStatusString(vstatus.at(i)), vcmdmsg.at(i), vcmderr.at(i), vcfgmsg.at(i), vcfgerr.at(i), vdetectormsg.at(i), vdetectorerr.at(i), vupdateenvmsg.at(i), vupdateenverr.at(i), vcontrollermonmsg.at(i), vcontrollermonerr.at(i), vsendpclmsg.at(i), vsendpclerr.at(i)));
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
    }
    if( !!pStatusPublisher ) {
        pStatusPublisher->Publish(_GetStatusJsonString(GetMilliTime(), _GetManagerStatusString(MS_Lost), ""));
    }
    MUJIN_LOG_DEBUG("Stopped status publisher");
}

std::string MujinVisionManager::_GetStatusJsonString(const unsigned long long timestamp, const std::string& status, const std::string& cmdmsg, const std::string& cmderr, const std::string& cfgmsg, const std::string& cfgerr, const std::string& detectormsg, const std::string& detectorerr, const std::string& updateenvmsg, const std::string& updateenverr, const std::string& controllermonmsg, const std::string& controllermonerr, const std::string& sendpclmsg, const std::string& sendpclerr)
{
    rapidjson::Document d(rapidjson::kObjectType);
    SetJsonValueByKey(d, "timestamp", timestamp);
    SetJsonValueByKey(d, "status", status);
    SetJsonValueByKey(d, "commandmessage", cmdmsg);
    if (cmderr != "") {
        SetJsonValueByKey(d, "commanderrorcode", cmderr);
    }
    SetJsonValueByKey(d, "detectormessage", detectormsg);
    if (detectorerr != "") {
        SetJsonValueByKey(d, "detectorerrorcode", detectorerr);
    }
    SetJsonValueByKey(d, "updateenvironmentmessage", updateenvmsg);
    if (updateenverr != "") {
        SetJsonValueByKey(d, "updateenvironmenterrorcode", updateenverr);
    }
    SetJsonValueByKey(d, "controllermonitormessage", controllermonmsg);
    if (controllermonerr != "") {
        SetJsonValueByKey(d, "controllermonitorerrorcode", controllermonerr);
    }
    SetJsonValueByKey(d, "sendpointcloudmessage", sendpclmsg);
    if (sendpclerr != "") {
        SetJsonValueByKey(d, "sendpointclouderrorcode", sendpclerr);
    }
    SetJsonValueByKey(d, "isdetectionrunning", IsDetectionRunning());
    if (_detectionRegionName != "") {
        SetJsonValueByKey(d, "detectionRegionName", _detectionRegionName);
    }
    SetJsonValueByKey(d, "isvisualizepointcloudrunning", _bIsVisualizePointcloudRunning);
    SetJsonValueByKey(d, "isexecutionverificationpointcloudrunnin", _bIsExecutionVerificationPointCloudRunning);
    SetJsonValueByKey(d, "issendpointcloudrunning", _bIsSendPointcloudRunning);
    SetJsonValueByKey(d, "isenvironmentupdaterunning", _bIsEnvironmentUpdateRunning);
    SetJsonValueByKey(d, "lastupdateenvironmenttimestamp", _tsLastEnvUpdate);
    return  DumpJson(d);
}

void MujinVisionManager::_RunCommandThread(const unsigned int port, int commandindex)
{
    CommandServerPtr pCommandServer;
    std::string incomingmessage;
    rapidjson::Document commandjson, resultjson(rapidjson::kObjectType);
    while (!_mPortStopCommandThread[commandindex]) {
        try {
            if( !pCommandServer ) {
                try {
                    pCommandServer.reset(new CommandServer(_zmqcontext, port));
                    std::stringstream ss;
                    ss << "Started command thread (port: " << port << ").";
                    MUJIN_LOG_DEBUG(ss.str());
                }
                catch(const std::exception& ex) {
                    std::stringstream ss;
                    ss << "could not create command on port=" << port << ", perhaps another visionmanager instance is running? " << ex.what();
                    MUJIN_LOG_ERROR(ss.str());
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1000)); // sleep a little to stop flooding the messages
                    continue;
                }
            }

            // receive message
            if (pCommandServer->Recv(incomingmessage, 100) > 0) {
                MUJIN_LOG_DEBUG("Received command message: " + incomingmessage + ".");
                // execute command
                ParseJson(commandjson, incomingmessage);
                resultjson.SetObject();
                try {
                    if (port == _configport) {
                        _ExecuteConfigurationCommand(commandjson, resultjson);
                    } else if (port == _commandport) {
                        _ExecuteUserCommand(commandjson, resultjson);
                    }
                }
                catch (const mujinclient::UserInterruptException& ex) { // need to catch it here, otherwise zmq will be in bad state
                    if (commandindex == CDI_Configure) {
                        MUJIN_LOG_WARN("User requested program exit.");
                        _mPortStopCommandThread[commandindex] = true;
                    } else {
                        _SetStatus(TT_Command, MS_Preempted, "", "", false);
                        MUJIN_LOG_WARN("User interruped command execution.");
                        SetJsonValueByKey(resultjson, "status", _GetManagerStatusString(MS_Preempted));
                    }
                }
                catch (const MujinVisionException& e) {
                    MUJIN_LOG_ERROR("MujinVisionException " + e.message());
                    switch (e.GetCode()) {
                    case MVE_Failed: break;
                    case MVE_InvalidArgument: break;
                    case MVE_CommandNotSupported: break;
                    case MVE_ConnectionError: break;
                    case MVE_ImageAcquisitionError: break;
                    case MVE_RecognitionError: break;
                    case MVE_ConfigurationFileError: break;
                    case MVE_NotImplemented: break;
                    case MVE_Busy: break;
                    case MVE_ControllerError: break;
                    default: break;
                    }
                    SetJsonValueByKey(resultjson, "error", e);
                    _SetStatus(TT_Command, MS_Aborted, "", e.message(), false);
                }
                catch (const zmq::error_t& e) {
                    std::stringstream ss;
                    ss << "caught zmq exception errornum=" << e.num();
                    rapidjson::Document errorjson(rapidjson::kObjectType);
                    SetJsonValueByKey(errorjson, "type", GetErrorCodeString(MVE_Failed));
                    SetJsonValueByKey(errorjson, "desc", ss.str());
                    SetJsonValueByKey(resultjson, "error", errorjson);
                    std::string errstr = DumpJson(resultjson);
                    MUJIN_LOG_ERROR(ss.str());
                    _SetStatus(TT_Command, MS_Aborted, "", errstr.substr(1, errstr.size() - 2), false); // remove '{' and '}'
                }
                catch (std::exception& e) {
                    rapidjson::Document errorjson(rapidjson::kObjectType);
                    SetJsonValueByKey(errorjson, "type", GetErrorCodeString(MVE_Failed));
                    SetJsonValueByKey(errorjson, "desc", e.what());
                    SetJsonValueByKey(resultjson, "error", errorjson);
                    std::string errstr = DumpJson(resultjson);
                    MUJIN_LOG_ERROR("unhandled std exception, " + errstr);
                    _SetStatus(TT_Command, MS_Aborted, "", errstr.substr(1, errstr.size() - 2), false); // remove '{' and '}'
                }
                catch (...) {
                    std::string whatstr = "unhandled exception!";
                    MUJIN_LOG_ERROR(whatstr);
                    rapidjson::Document errorjson(rapidjson::kObjectType);
                    SetJsonValueByKey(errorjson, "type", GetErrorCodeString(MVE_Failed));
                    SetJsonValueByKey(errorjson, "desc", whatstr);
                    SetJsonValueByKey(resultjson, "error", errorjson);
                    std::string errstr = DumpJson(resultjson);
                    MUJIN_LOG_ERROR("unhandled std exception, " + errstr);
                    _SetStatus(TT_Command, MS_Aborted, "", errstr.substr(1, errstr.size() - 2), false); // remove '{' and '}'
                }

                // send output
                pCommandServer->Send(DumpJson(resultjson));
            }
        }
        catch (const MujinVisionException& e) {
            std::string errstr = "Caught unhandled MujinVisionException " + e.message();
            _SetStatus(TT_Command, MS_Aborted, errstr, "", false);
            MUJIN_LOG_WARN(errstr);
        }
        catch (const zmq::error_t& e) {
            if (!_mPortStopCommandThread[commandindex]) {
                std::string errstr = "Failed to receive command";
                _SetStatus(TT_Command, MS_Aborted, errstr, "", false);
                MUJIN_LOG_WARN(errstr);
            }
        }
        catch (const mujinclient::UserInterruptException& ex) {
            std::string errstr = "User requested program exit";
            _SetStatus(TT_Command, MS_Aborted, errstr, "", false);
            MUJIN_LOG_WARN(errstr);
        }
        catch (...) {
            if (!_bShutdown) {
                std::stringstream errss;
                errss << "caught unhandled exception in command thread port=" << port;
                _SetStatus(TT_Command, MS_Aborted, errss.str(), "", false);
                MUJIN_LOG_WARN(errss.str());
            }
        }
    }
    if (port == _commandport) {
        MUJIN_LOG_INFO("stopping threads started by command thread");
        _StopDetectionThread();
        _StopUpdateEnvironmentThread();
        _StopExecutionVerificationPointCloudThread();
        _StopControllerMonitorThread();
        _StopVisualizePointCloudThread();
        _StopControllerMonitorThread();
        _pDetectorManager.reset();
        _pDetector.reset();
        _pImagesubscriberManager.reset();
    }
}

void MujinVisionManager::_StartDetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const bool ignoreocclusion, const unsigned int fetchimagetimeout, const unsigned long long detectionstarttimestamp, const unsigned int maxnumfastdetection, const unsigned int maxnumdetection, const bool stoponleftinorder, const std::string& targetupdatename, const unsigned int numthreads, const std::string& cycleindex)
{
    if (targetupdatename.size() > 0) {
        _targetupdatename = targetupdatename;
    }
    if (detectionstarttimestamp > 0) {
        _tsStartDetection = detectionstarttimestamp;
    } else {
        _tsStartDetection = GetMilliTime();
    }
    if (!!_pDetectionThread && !_bStopDetectionThread) {
        _SetStatusMessage(TT_Command, "Detection thread is already running, do nothing.");
    } else {
        _bStopDetectionThread = false;
        DetectionThreadParams params;
        params.ignoreocclusion = ignoreocclusion;
        params.newerthantimestamp = detectionstarttimestamp;
        params.fetchimagetimeout = fetchimagetimeout;
        params.maxnumfastdetection = maxnumfastdetection;
        params.maxnumdetection = maxnumdetection;
        params.stoponleftinorder = stoponleftinorder;
        params.numthreads = numthreads;
        params.cycleindex = cycleindex;

        _bIsDetectionRunning = true;
        _detectionRegionName = regionname;
        // reset cached binpicking state to ensure clean state, e.g. lastGrabbedTargetTimestamp
        {
            boost::mutex::scoped_lock lock(_mutexControllerBinpickingState);
            _bIsControllerPickPlaceRunning = false;
            _bIsRobotOccludingSourceContainer = false;
            _bForceRequestDetectionResults = false;
            _numPickAttempt = 0;
            _binpickingstateTimestamp = 0;
            _lastGrabbedTargetTimestamp = 0;
            _bIsGrabbingTarget = false;
            _bIsGrabbingLastTarget = false;
            _orderNumber = 0;
            _numLeftInOrder = 0;
            _numLeftInSupply = 0;
            _placedInDest = 0;
        }
        _pDetectionThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_DetectionThread, this, regionname, cameranames, params)));
    }
}

void MujinVisionManager::_StartUpdateEnvironmentThread(const std::string& regionname, const std::vector<std::string>& cameranames, const std::string& obstaclename, const unsigned int waitinterval, const std::string& locale)
{
    if (!!_pUpdateEnvironmentThread && !_bStopUpdateEnvironmentThread) {
        _SetStatusMessage(TT_Command, "UpdateEnvironment thread is already running, do nothing.");
    } else {
        _bStopUpdateEnvironmentThread = false;
        _bIsEnvironmentUpdateRunning = true;
        UpdateEnvironmentThreadParams params;
        params.regionname = regionname;
        params.cameranames = cameranames;
        params.obstaclename = obstaclename;
        params.waitinterval = waitinterval;
        params.locale = locale;
        _pUpdateEnvironmentThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_UpdateEnvironmentThread, this, params)));
    }
}

void MujinVisionManager::_StartExecutionVerificationPointCloudThread(const std::vector<std::string>& cameranames, const std::vector<std::string>& evcamnames, const bool ignoreocclusion, const std::string& obstaclename, const unsigned int waitinterval, const std::string& locale)
{
    if (!!_pExecutionVerificationPointCloudThread && !_bStopExecutionVerificationPointCloudThread) {
        _SetStatusMessage(TT_Command, "ExecutionVerificationPointCloud thread is already running, do nothing.");
    } else {
        _bStopExecutionVerificationPointCloudThread = false;
        _bIsExecutionVerificationPointCloudRunning = true;
        SendExecutionVerificationPointCloudParams params;
        params.cameranames = cameranames;
        params.executionverificationcameranames = evcamnames;
        params.ignoreocclusion = ignoreocclusion;
        //params.obstaclename = obstaclename;
        params.waitinterval = waitinterval;
        params.locale = locale;
        _pExecutionVerificationPointCloudThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_SendExecutionVerificationPointCloudThread, this, params)));
    }
}

void MujinVisionManager::_StartControllerMonitorThread(const unsigned int waitinterval, const std::string& locale)
{
    if (!!_pControllerMonitorThread && !_bStopControllerMonitorThread) {
        _SetStatusMessage(TT_Command, "ControllerMonitor thread is already running, do nothing.");
    } else {
        _bStopControllerMonitorThread = false;
        _pControllerMonitorThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_ControllerMonitorThread, this, waitinterval, locale)));
    }
}

void MujinVisionManager::_StartVisualizePointCloudThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double pointsize, const bool ignoreocclusion, const unsigned long long newerthantimestamp, const unsigned int fetchimagetimeout, const bool request)
{
    if (!!_pVisualizePointCloudThread && !_bStopVisualizePointCloudThread) {
        _SetStatusMessage(TT_Command, "VisualizePointCloud thread is already running, do nothing.");
    } else {
        _bStopVisualizePointCloudThread = false;
        _bIsVisualizePointcloudRunning = true;
        VisualizePointcloudThreadParams params;
        params.regionname = regionname;
        params.cameranames = cameranames;
        params.pointsize = pointsize;
        params.ignoreocclusion = ignoreocclusion;
        params.newerthantimestamp = newerthantimestamp;
        params.fetchimagetimeout = fetchimagetimeout;
        params.request = request;
        _pVisualizePointCloudThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_VisualizePointCloudThread, this, params)));
    }
}

void MujinVisionManager::_StopSendPointCloudObstacleToControllerThread()
{
    _SetStatusMessage(TT_Command, "Stopping sendpointcloudobstacle thread.");
    _bStopSendPointCloudObstacleToControllerThread = true;
    if (!!_pSendPointCloudObstacleThread) {
        _pSendPointCloudObstacleThread->join();
        _pSendPointCloudObstacleThread.reset();
        _SetStatusMessage(TT_Command, "Stopped sendpointcloudobstacle thread.");
    }
}

void MujinVisionManager::_StopDetectionThread()
{
    std::stringstream ss;
    _SetStatusMessage(TT_Command, "Stopping detection thread.");
    if (!_bStopDetectionThread) {
        _bStopDetectionThread = true;
        if (!!_pDetectionThread) {
            _pDetectionThread->join();
            _pDetectionThread.reset();
            ss << "Stopped detection thread. " << (GetMilliTime() - _tsStartDetection ) / 1000.0f << " secs since start";
            _SetStatusMessage(TT_Command, ss.str());
        }
        _bStopDetectionThread = false; // reset so that _GetImage works properly afterwards
    } else {
        ss << "Stopped detection thread.";
    }
    MUJIN_LOG_INFO(ss.str());
    _tsStartDetection = 0;
}

void MujinVisionManager::_StopUpdateEnvironmentThread()
{
    _SetStatusMessage(TT_Command, "Stopping update environment thread.");
    if (!!_pUpdateEnvironmentThread) {
        _bStopUpdateEnvironmentThread = true;
        _pUpdateEnvironmentThread->join();
        _pUpdateEnvironmentThread.reset();
        _SetStatusMessage(TT_Command, "Stopped update environment thread.");
    }
    _bStopUpdateEnvironmentThread = false; // reset so that _GetImage works properly afterwards
    MUJIN_LOG_DEBUG("stopped updateenvironment thread");
}

void MujinVisionManager::_StopExecutionVerificationPointCloudThread()
{
    _SetStatusMessage(TT_Command, "Stopping execution verification thread.");
    if (!!_pExecutionVerificationPointCloudThread) {
        _bStopExecutionVerificationPointCloudThread = true;
        _pExecutionVerificationPointCloudThread->join();
        _pExecutionVerificationPointCloudThread.reset();
        _SetStatusMessage(TT_Command, "Stopped execution verification thread.");
    }
    _bStopExecutionVerificationPointCloudThread = false; // reset so that _GetImage works properly afterwards
    MUJIN_LOG_DEBUG("stopped execution verification thread");
}

void MujinVisionManager::_StopControllerMonitorThread()
{
    _SetStatusMessage(TT_Command, "Stopping controller monitor thread.");
    if (!_bStopControllerMonitorThread) {
        _bStopControllerMonitorThread = true;
        if (!!_pControllerMonitorThread) {
            _pControllerMonitorThread->join();
            _pControllerMonitorThread.reset();
            _SetStatusMessage(TT_Command, "Stopped controller monitor thread.");
        }
        _bStopControllerMonitorThread = false;
    }
    MUJIN_LOG_DEBUG("stopped controllermonitor thread");
}

void MujinVisionManager::_StopVisualizePointCloudThread()
{
    _SetStatusMessage(TT_Command, "Stopping pointcloud visualization thread.");
    if (!_bStopVisualizePointCloudThread) {
        _bStopVisualizePointCloudThread = true;
        if (!!_pVisualizePointCloudThread) {
            _pVisualizePointCloudThread->join();
            _pVisualizePointCloudThread.reset();
            _SetStatusMessage(TT_Command, "Stopped pointcloud visualization thread.");
        }
        _bStopVisualizePointCloudThread = false;
    }
    MUJIN_LOG_DEBUG("stopped pointcloud visualization thread");
}

class FalseSetter
{
public:
    FalseSetter(bool& value) : _value(value) {
    }
    ~FalseSetter() {
        _value = false;
    }
    bool& _value;
};

class TrueSetter
{
public:
    TrueSetter(bool& value) : _value(value) {
    }
    ~TrueSetter() {
        _value = true;
    }
    bool& _value;
};

void MujinVisionManager::_DetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, DetectionThreadParams params)
{
    FalseSetter turnOffDetection(_bIsDetectionRunning);
    uint64_t time0, starttime;
    bool ignoreocclusion = params.ignoreocclusion;
    bool stoponleftinorder = params.stoponleftinorder;
    unsigned long long newerthantimestamp = params.newerthantimestamp; // used to control the next image to use for detection. In the beginning it is initialized from user, but after images are used, this will be udpated
    unsigned int fetchimagetimeout = params.fetchimagetimeout;
    unsigned int maxnumfastdetection = params.maxnumfastdetection;
    unsigned int maxnumdetection = params.maxnumdetection;
    std::string cycleindex = params.cycleindex;
    BinPickingTaskResourcePtr pBinpickingTask = _pSceneResource->GetOrCreateBinPickingTaskFromName_UTF8(_tasktype+std::string("task1"), _tasktype, TRO_EnableZMQ);
    std::string userinfo = _GetUserInfoJsonString();
    MUJIN_LOG_DEBUG("initialzing binpickingtask in DetectionThread with userinfo " + userinfo);
    time0 = GetMilliTime();
    MUJIN_LOG_DEBUG("ValidateJsonString took " << (GetMilliTime()-time0)/1000.0f << " secs");
    time0 = GetMilliTime();
    pBinpickingTask->Initialize(_defaultTaskParameters, _binpickingTaskZmqPort, _binpickingTaskHeartbeatPort, _zmqcontext, false, 0/*do not start monitor thread*/, _controllerCommandTimeout, userinfo, _slaverequestid);
    MUJIN_LOG_DEBUG("pBinpickingTask->Initialize() took " << (GetMilliTime()-time0)/1000.0f << " secs");
    int numfastdetection = maxnumfastdetection; // max num of times to run fast detection
    bool bindetectiononly = false;
    if (numfastdetection == 0 && (_bindetectionMode == "once" || _bindetectionMode == "always")) {
        MUJIN_LOG_INFO("maxnumfastdetection is set to 0, need to do bin detection for at least once");
        bindetectiononly = true;
    }
    int lastDetectedId = 0;
    int lastPickedId = -1;
    uint64_t oldbinpickingstatets = 0;
    uint64_t lastbinpickingstatewarningts = 0;
    uint64_t lastwaitforocclusionwarningts = 0;
    uint64_t lastattemptts = 0;
    int numPickAttempt = 0;
    bool isControllerPickPlaceRunning = false;
    bool isRobotOccludingSourceContainer = false;
    bool forceRequestDetectionResults = false;
    //bool isGrabbingTarget = false;
    //bool isGrabbingLastTarget = false;
    bool detectcontaineronly = false;
    int numLeftInOrder = -1;
    int orderNumber = -1;
    unsigned long long binpickingstateTimestamp = 0;
    unsigned long long lastGrabbedTargetTimestamp = 0;
    unsigned int numdetection = 0;
    std::vector<DetectedObjectPtr> detectedobjects, detectedparts;
    _bUseGrabbedTargeInfoInDetectionPreempt = stoponleftinorder; // use the grabbed target info only if stoponleftinorder is set

    uint64_t lastCaptureResetTimestamp = 0; // ms timestamp when the capture handles were last reset. Used to prevent too many force resets in one time.
    uint64_t lastCaptureResetTimeout = 4000; // how long to wait until force reset is called again

    bool bDetectorHasRunAtLeastOnce = false;
    size_t filteringsubsample = 0;
    double filteringstddev = 0;
    int filteringnumnn = 0;
    double pointsize = 0;
    {
        boost::mutex::scoped_lock lock(_mutexRegion);

        RegionParametersPtr pregion = _mNameRegion[regionname]->pRegionParameters;
        if( !!pregion ) {
            MUJIN_LOG_INFO("pointsize=0, using pointsize= " << pointsize << " in regionparam");
            pointsize = pregion->pointsize;

            filteringsubsample = pregion->filteringsubsample;
            filteringstddev = pregion->filteringstddev;
            filteringnumnn = pregion->filteringnumnn;
        }
    }
    if( filteringnumnn == 0 ) {
        filteringnumnn = _filteringnumnn;
    }
    if( filteringsubsample == 0 ) {
        filteringsubsample = _filteringsubsample;
    }
    if( filteringstddev == 0 ) {
        filteringstddev = _filteringstddev;
    }

//    std::vector<CameraCaptureHandlePtr> capturehandles; CREATE_SAFE_DELETER_CAMERAHANDLES(capturehandles);
//    MUJIN_LOG_DEBUG("_StartAndGetCaptureHandle with cameranames " << __GetString(cameranames));
//    try {
//        _StartAndGetCaptureHandle(cameranames, cameranames, capturehandles, /*force=*/ false, ignoreocclusion); // force the first time
//    }
//    catch( const mujinclient::UserInterruptException& ex) {
//        MUJIN_LOG_INFO("User interrupted DetectObject!");
//        _bStopDetectionThread = true;
//    }
//    catch (const MujinVisionException& e) {
//        MUJIN_LOG_ERROR(e.message());
//        _SetDetectorStatusMessage(e.message(), e.GetCodeString());
//        MUJIN_LOG_INFO("caught exception, stopping detection thread");
//        _bStopDetectionThread = true;
//    }
//    catch(const std::exception& ex) {
//        std::stringstream ss;
//        ss << "Caught exception: " << ex.what();
//        MUJIN_LOG_ERROR(ss.str());
//        _SetDetectorStatusMessage(ss.str(), GetErrorCodeString(MVE_Failed));
//        MUJIN_LOG_INFO("caught exception, stopping detection thread");
//        _bStopDetectionThread = true;
//    }
    _pDetector->SetNumThreads(params.numthreads);
    while (!_bStopDetectionThread && (maxnumdetection <= 0 || numdetection < maxnumdetection) && !(stoponleftinorder && numLeftInOrder == 0 && lastGrabbedTargetTimestamp > _tsStartDetection && _tsLastEnvUpdate > 0 && _resultImageEndTimestamp > 0 && lastGrabbedTargetTimestamp < _resultImageEndTimestamp && !_bIsGrabbingLastTarget)) {
        detectcontaineronly = false;
        starttime = GetMilliTime();
        std::string resultstate;
        detectedobjects.resize(0);
        detectedparts.resize(0);
        int numresults = 0;
        unsigned long long imageStartTimestamp=0, imageEndTimestamp=0;
        int isContainerPresent=-1;
        bool bDetectorHasRunThisCycle = false;

        try {
            time0=GetMilliTime();
            oldbinpickingstatets = binpickingstateTimestamp;
            {
                boost::mutex::scoped_lock lock(_mutexControllerBinpickingState);
                if (binpickingstateTimestamp != _binpickingstateTimestamp) {
                    MUJIN_LOG_DEBUG("DetectionThread binpickingstate: ts=" << _binpickingstateTimestamp << " numPickAttempt=" << _numPickAttempt << " isControllerPickPlaceRunning=" << _bIsControllerPickPlaceRunning << " isRobotOccludingContainer=" << _bIsRobotOccludingSourceContainer << " forceRequestDetectionResults=" << forceRequestDetectionResults << " _numLeftInOrder=" << _numLeftInOrder << " lastGrabbedTargetTimestamp=" << _lastGrabbedTargetTimestamp << " _tsLastEnvUpdate=" << _tsLastEnvUpdate << " _resultImageEndTimestamp" << _resultImageEndTimestamp << " _bIsGrabbingLastTarget=" << _bIsGrabbingLastTarget);
                }
                binpickingstateTimestamp = _binpickingstateTimestamp;
                lastGrabbedTargetTimestamp = _lastGrabbedTargetTimestamp;
                if (_numPickAttempt > numPickAttempt) {
                    lastattemptts = binpickingstateTimestamp;
                }
                numPickAttempt = _numPickAttempt;
                isControllerPickPlaceRunning = _bIsControllerPickPlaceRunning;
                isRobotOccludingSourceContainer = _bIsRobotOccludingSourceContainer;
                //isGrabbingTarget = _bIsGrabbingTarget;
                //isGrabbingLastTarget = _bIsGrabbingLastTarget;
                forceRequestDetectionResults = _bForceRequestDetectionResults;
                numLeftInOrder = _numLeftInOrder;
                orderNumber = _orderNumber;
            }
            if (binpickingstateTimestamp > oldbinpickingstatets) {
                MUJIN_LOG_DEBUG("get binpicking state took " << (GetMilliTime()-time0)/1000.0f << " secs");
            }
            if (_bStopDetectionThread) {
                break;
            }
            time0 = GetMilliTime();
            if (stoponleftinorder && orderNumber > 0 && (numLeftInOrder == 0 || (numLeftInOrder == 1 && _bIsGrabbingLastTarget)) ) { // make sure to test when numLeftInOrder is 0 (or 1 and robot is grabbing last target).
                // last part is already picked up and robot is most likely moving to its home position. turn off preempting, but make sure all images are after lastGrabbedTargetTimestamp
                _bUseGrabbedTargeInfoInDetectionPreempt = false; // starting to do container empty detection only, so do not preempt detector anymore
                if( lastGrabbedTargetTimestamp == 0 ) {
                    MUJIN_LOG_WARN("last part is already placed, but lastGrabbedTargetTimestamp is 0!");
                }
                MUJIN_LOG_INFO("numLeftInOrder=" << numLeftInOrder << " orderNumber=" << orderNumber << " stoponleftinorder=" << stoponleftinorder << ", check container empty only.");
                // have to guarantee that capture is started
//                MUJIN_LOG_DEBUG("_StartAndGetCaptureHandle with cameranames " << __GetString(cameranames));
//                _StartAndGetCaptureHandle(cameranames, cameranames, capturehandles, /*force=*/ false, ignoreocclusion);
                detectcontaineronly = true;
            } else if (!isControllerPickPlaceRunning || forceRequestDetectionResults) { // detect if forced or not during pick and place
//                MUJIN_LOG_INFO("force detection, start capturing..." << (int)isControllerPickPlaceRunning << " " << (int)forceRequestDetectionResults);
//                MUJIN_LOG_DEBUG("_StartAndGetCaptureHandle with cameranames " << __GetString(cameranames));
//                _StartAndGetCaptureHandle(cameranames, cameranames, capturehandles, /*force=*/ false, ignoreocclusion);
            } else {  // do the following only if pick and place thread is running and detection is not forced
                if (numPickAttempt <= lastPickedId) { // if robot has picked
                    if (GetMilliTime() - binpickingstateTimestamp < 10000) { // only do the following if the binpicking state message is up-to-date
                        if (isRobotOccludingSourceContainer) { // skip detection if robot occludes camera
                            if (binpickingstateTimestamp > _lastocclusionTimestamp) {
                                _lastocclusionTimestamp = binpickingstateTimestamp;
                            }
                            // stop capturing by removing capture handles
//                            capturehandles.resize(0);
                            continue;
                        } else {
                            // detect when robot is not occluding camera anymore
                            // even when it is placing the last part, because
                            // 1, the placing could fail, and we can use the new result
                            // 2, or, the result of this detection call could arrive after placing is done but before the next detection call, therefore saving another round of detection
//                            MUJIN_LOG_INFO("need to detect for this picking attempt, starting image capturing... numPickAttempt=" << numPickAttempt << " lastPickedId=" << lastPickedId << " forceRequestDetectionResults=" << int(forceRequestDetectionResults) << " lastDetectedId=" << lastDetectedId << " numLeftInOrder=" << numLeftInOrder << " stoponleftinorder=" << stoponleftinorder << " lastGrabbedTargetTimestamp=" << lastGrabbedTargetTimestamp);
//                            MUJIN_LOG_DEBUG("_StartAndGetCaptureHandle with cameranames " << __GetString(cameranames));
//                            _StartAndGetCaptureHandle(cameranames, cameranames, capturehandles, /*force=*/ false, ignoreocclusion);
                        }
                    } else { // do not detect if binpicking status message is old (controller in bad state)
                        if (GetMilliTime() - lastbinpickingstatewarningts > 1000.0) {
                            MUJIN_LOG_WARN("binpickingstateTimestamp (" << binpickingstateTimestamp << ") is > " << 10000 << "ms older than current time (" << GetMilliTime() << ") 1");
                            lastbinpickingstatewarningts = GetMilliTime();
                        }
                        continue;
                    }
                } else { // if robot has not picked
                    if (GetMilliTime() - binpickingstateTimestamp < 10000) { // only do the following if the binpicking state message is up-to-date
                        if (!isRobotOccludingSourceContainer && GetMilliTime() - lastattemptts <= 10000.0) { // skip detection if robot does not occlude camera up to 10.0 seconds from last picking attempt
                            if (GetMilliTime() - lastwaitforocclusionwarningts > 1000.0) {
                                MUJIN_LOG_INFO("wait until robot picks (occludes camera)...");
                                lastwaitforocclusionwarningts = GetMilliTime();
                            }
                            boost::this_thread::sleep(boost::posix_time::milliseconds(50));
                            continue;
                        } else {
                            lastPickedId = numPickAttempt;
                            MUJIN_LOG_INFO(str(boost::format("robot has picked %d")%lastPickedId));
                            continue;
                        }
                    } else { // do not detect if binpicking status message is old (controller in bad state)
                        if (GetMilliTime() - lastbinpickingstatewarningts > 1000.0) {
                            MUJIN_LOG_WARN("binpickingstateTimestamp (" << binpickingstateTimestamp << ") is > " << 10000 << "ms older than current time (" << GetMilliTime() << ")");
                            lastbinpickingstatewarningts = GetMilliTime();
                        }
                        continue;
                    }
                }
            }
            MUJIN_LOG_DEBUG("streamer management took " << (GetMilliTime()-time0)/1000.0f << " secs");
            if( !_bIsEnvironmentUpdateRunning ) {
                MUJIN_LOG_WARN("environment update thread stopped! so stopping detector");
                break;
            }

            if (_bStopDetectionThread) {
                break;
            }
            if (detectcontaineronly) {
                MUJIN_LOG_DEBUG("detect to check if container is empty");
                bool fastdetection=false;
                bool bindetection = false;
                bool request=false;
                bool useold=false;
                bool checkcontaineremptyonly=true;
                // images processed here should be after lastGrabbedTargetTimestamp.
                if (lastGrabbedTargetTimestamp == 0) {
                    MUJIN_LOG_ERROR("lastGrabbedTargetTimestamp should not be 0!");
                }
                numresults = _DetectObjects(TT_Detector, pBinpickingTask, regionname, cameranames, detectedobjects, resultstate, imageStartTimestamp, imageEndTimestamp, isContainerPresent, ignoreocclusion, std::max(newerthantimestamp, lastGrabbedTargetTimestamp), fetchimagetimeout, fastdetection, bindetection, request, useold, checkcontaineremptyonly, cycleindex);
                if (numresults == -1) {
                    if( lastCaptureResetTimestamp == 0 || GetMilliTime() - lastCaptureResetTimestamp > lastCaptureResetTimeout ) {
                        lastCaptureResetTimestamp = GetMilliTime();
                        MUJIN_LOG_ERROR("detection failed");
//                        MUJIN_LOG_INFO("force capturing, in case streamer crashed");
//                        MUJIN_LOG_DEBUG("try to start capturing with cameranames " << __GetString(cameranames));
//                        _StartAndGetCaptureHandle(cameranames, cameranames, capturehandles, true, ignoreocclusion);
                    }
//                    else {
                        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
//                    }
                }
                else if( numresults >= 0 ) {
                    // have to update newerthantimestamp or else detector will continue processing old images
                    if( newerthantimestamp < imageEndTimestamp ) {
                        newerthantimestamp = imageEndTimestamp;
                    }
                    bDetectorHasRunThisCycle = bDetectorHasRunAtLeastOnce = true;
                }
            }
            else if (numfastdetection > 0 || bindetectiononly) {
                bool bFailedBecauseOfNoImage = false;
                while (!_bStopDetectionThread && numresults == 0 && (numfastdetection > 0 || bindetectiononly)) {
                    bool fastdetection=true;
                    bool bindetection = false;
                    if (_bindetectionMode == "once" || _bindetectionMode == "always") {
                        bindetection = true;
                    }
                    bool request=true;
                    bool useold=false;
                    bool checkcontaineremptyonly=false;
                    if (bindetectiononly) {
                        fastdetection = false;
                        bindetectiononly = false;
                        MUJIN_LOG_DEBUG("DetectObjects() in normal mode for bindetection only");
                    } else {
                        MUJIN_LOG_DEBUG("DetectObjects() in fast mode");
                    }
                    MUJIN_LOG_DEBUG(str(boost::format("call DetectObjects() with bindetection=%d _bindetectionMode=%s") % bindetection % _bindetectionMode));
                    bFailedBecauseOfNoImage = false;
                    numresults = _DetectObjects(TT_Detector, pBinpickingTask, regionname, cameranames, detectedobjects, resultstate, imageStartTimestamp, imageEndTimestamp, isContainerPresent, ignoreocclusion, newerthantimestamp, fetchimagetimeout, fastdetection, bindetection, request, useold, checkcontaineremptyonly, cycleindex);
                    if (numresults == -1) {
                        bFailedBecauseOfNoImage = true;
                        if( lastCaptureResetTimestamp == 0 || GetMilliTime() - lastCaptureResetTimestamp > lastCaptureResetTimeout ) {
                            lastCaptureResetTimestamp = GetMilliTime();
                            MUJIN_LOG_ERROR("detection failed");
//                            MUJIN_LOG_INFO("force capturing, in case streamer crashed");
//                            MUJIN_LOG_DEBUG("try to start capturing with cameranames " << __GetString(cameranames));
//                            _StartAndGetCaptureHandle(cameranames, cameranames, capturehandles, true, ignoreocclusion);
                        }
//                        else {
                            boost::this_thread::sleep(boost::posix_time::milliseconds(50));
//                        }
                        numresults = 0;
                    }
                    else if( numresults >= 0 ) {
                        // do not update newerthantimestamp since might have to rerun on the same old images, which will require using the original newerthantimestamp
                        bDetectorHasRunThisCycle = bDetectorHasRunAtLeastOnce = true;
                    }

                    if (isContainerPresent == 0) {
                        numresults = 0;
                        MUJIN_LOG_WARN("container is not present, detect again");
                        numfastdetection = maxnumfastdetection;
                    }
                    if (numresults == 0) {
                        // have to do again, so publish the current result. even if no detected objects, resultstate can have info about the container (like empty)
                        if (resultstate != "null") { // only publish result when resultstate is valid
                            boost::mutex::scoped_lock lock(_mutexDetectedInfo);
                            if( !detectcontaineronly ) {
                                // only update the objects if detector actually returned them, otherwise will be erasing previously sent good results
                                _vDetectedObject.swap(detectedobjects);
                                _bDetectedObjectsValid = true;
                            }
                            else {
                                _bDetectedObjectsValid = false;
                            }
                            _resultState = resultstate;
                            _resultTimestamp = GetMilliTime();
                            _resultImageStartTimestamp = imageStartTimestamp;
                            _resultImageEndTimestamp = imageEndTimestamp;
                            MUJIN_LOG_INFO(str(boost::format("send %d (%d) detected objects with _resultTimestamp=%u, imageStartTimestamp=%u imageEndTimestamp=%u detectcontaineronly=%d resultstate=%s")%_vDetectedObject.size()%(int)_bDetectedObjectsValid%_resultTimestamp%imageStartTimestamp%_resultImageEndTimestamp%detectcontaineronly%resultstate));
                        }
                        if (!bFailedBecauseOfNoImage) { // do not skip fast detection due to no image
                            numfastdetection -= 1;
                        }
                    } else {
                        numfastdetection = 0;
                    }
                }
                if (!_bStopDetectionThread && numresults == 0 && numfastdetection == 0 ) {
                    MUJIN_LOG_DEBUG("DetectObjects() in fast mode found no object, detect in normal mode");
                    bool fastdetection=false;
                    bool bindetection = false;
                    if ((bDetectorHasRunAtLeastOnce && _bindetectionMode == "always") || (!bDetectorHasRunAtLeastOnce && (_bindetectionMode == "once" || _bindetectionMode == "always"))) {
                        bindetection = true;
                    }
                    bool request=true;
                    bool useold=!bFailedBecauseOfNoImage; //true;
                    bool checkcontaineremptyonly=false;
                    numresults = _DetectObjects(TT_Detector, pBinpickingTask, regionname, cameranames, detectedobjects, resultstate, imageStartTimestamp, imageEndTimestamp, isContainerPresent, ignoreocclusion, newerthantimestamp, fetchimagetimeout, fastdetection, bindetection, request, useold, checkcontaineremptyonly, cycleindex);
                    if (numresults == -1) {
                        if( lastCaptureResetTimestamp == 0 || GetMilliTime() - lastCaptureResetTimestamp > lastCaptureResetTimeout ) {
                            lastCaptureResetTimestamp = GetMilliTime();
                            MUJIN_LOG_ERROR("detection failed");
//                            MUJIN_LOG_INFO("force capturing, in case streamer crashed");
//                            MUJIN_LOG_DEBUG("try to start capturing with cameranames " << __GetString(cameranames));
//                            _StartAndGetCaptureHandle(cameranames, cameranames, capturehandles, true, ignoreocclusion);
                        }
//                        else {
                            boost::this_thread::sleep(boost::posix_time::milliseconds(50));
//                        }
                        numresults = 0;
                    }
                    else if( numresults >= 0 ) {
                        // have to update newerthantimestamp or else detector will continue processing old images
                        if( newerthantimestamp < imageEndTimestamp ) {
                            newerthantimestamp = imageEndTimestamp;
                        }
                        bDetectorHasRunThisCycle = bDetectorHasRunAtLeastOnce = true;
                    }
                }
                else {
                    if( numresults >= 0 ) {
                        // have to update newerthantimestamp or else detector will continue processing old images
                        if( newerthantimestamp < imageEndTimestamp ) {
                            newerthantimestamp = imageEndTimestamp;
                        }
                    }
                }
                        
            } else {
                MUJIN_LOG_DEBUG("detect normally");
                bool fastdetection=false;
                bool bindetection = false;
                if ((bDetectorHasRunAtLeastOnce && _bindetectionMode == "always") || (!bDetectorHasRunAtLeastOnce && (_bindetectionMode == "once" || _bindetectionMode == "always"))) {
                    bindetection = true;
                }
                bool request=true;
                bool useold=false;
                bool checkcontaineremptyonly=false;
                numresults = _DetectObjects(TT_Detector, pBinpickingTask, regionname, cameranames, detectedobjects, resultstate, imageStartTimestamp, imageEndTimestamp, isContainerPresent, ignoreocclusion, newerthantimestamp, fetchimagetimeout, fastdetection, bindetection, request, useold, checkcontaineremptyonly, cycleindex);
                if (numresults == -1) {
//                    if( lastCaptureResetTimestamp == 0 || GetMilliTime() - lastCaptureResetTimestamp > lastCaptureResetTimeout ) {
//                        lastCaptureResetTimestamp = GetMilliTime();
//                        MUJIN_LOG_INFO("force capturing, in case streamer crashed");
//                        MUJIN_LOG_DEBUG("try to start capturing with cameranames " << __GetString(cameranames));
//                        _StartAndGetCaptureHandle(cameranames, cameranames, capturehandles, true, ignoreocclusion);
//                    }
//                    else {
                        MUJIN_LOG_ERROR("detection failed");
                        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
//                    }
                    numresults = 0;
                }
                else if( numresults >= 0 ) {
                    // have to update newerthantimestamp or else detector will continue processing old images
                    if( newerthantimestamp < imageEndTimestamp ) {
                        newerthantimestamp = imageEndTimestamp;
                    }
                    bDetectorHasRunThisCycle = bDetectorHasRunAtLeastOnce = true;
                }

            }
            if (_bStopDetectionThread) {
                break;
            }
            if (bDetectorHasRunThisCycle && !detectcontaineronly) {  // skip getting pointcloud obstacle if detecting container only
                // call GetPointCloudObstacle on parts only
                {
                    boost::mutex::scoped_lock lock(_mutexRegion);
                    for (size_t i=0; i<detectedobjects.size(); ++i) {
                        if (_mNameRegion.find(detectedobjects[i]->name) == _mNameRegion.end()) {
                            detectedparts.push_back(detectedobjects[i]);
                        }
                    }
                }
                std::vector<std::string> cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);
                for (unsigned int i=0; i<cameranamestobeused.size(); i++) {
                    std::string cameraname = cameranamestobeused[i];
                    std::vector<Real> points;
                    std::stringstream ss;
                    uint64_t starttime = GetMilliTime();
                    {
                        boost::mutex::scoped_lock lock(_mutexDetector);
                        _pDetector->GetPointCloudObstacle(regionname, cameraname, detectedparts, points, pointsize, false, true, filteringstddev, filteringnumnn, 1, cycleindex);
                    }
                    ss << "GetPointCloudObstacle() took " << (GetMilliTime() - starttime) / 1000.0f << " secs";
                    MUJIN_LOG_INFO(ss.str());
                    if (points.size() / 3 == 0) {
                        _SetDetectorStatusMessage("got 0 point from GetPointCloudObstacle() in detection loop");
                    }
                    {
                        boost::mutex::scoped_lock lock(_mutexDetectedInfo);
                        _mResultPoints[cameraname] = points;
                    }
                }
            }
            lastDetectedId = numPickAttempt;
//            if (isControllerPickPlaceRunning && !forceRequestDetectionResults && numresults > 0) {
//                MUJIN_LOG_INFO("detected at least 1 object, stop image capturing...");
//                // stop capturing by removing capture handles
//                capturehandles.resize(0);
//                MUJIN_LOG_INFO("capturing stopped");
//            }
//            else {
//                //MUJIN_LOG_INFO("detected no object, do not stop image capturing...");
//            }
        }
        catch(const mujinclient::UserInterruptException& ex) {
            MUJIN_LOG_INFO("User interrupted DetectObject!");
            numdetection += 1;
            continue; // continue and let while loop logic determine if should detect again
        }
        catch (const MujinVisionException& e) {
            MUJIN_LOG_ERROR(e.message());
            _SetDetectorStatusMessage(e.message(), e.GetCodeString());
            numdetection += 1;
            continue;
        }
        catch(const std::exception& ex) {
            std::stringstream ss;
            ss << "Caught exception in the detection loop: " << ex.what();
            MUJIN_LOG_ERROR(ss.str());
            _SetDetectorStatusMessage(ss.str(), GetErrorCodeString(MVE_RecognitionError));
            numdetection += 1;
            continue;
        }

        // process results
        if (_bStopDetectionThread) {
            break;
        }

        if( !_bIsEnvironmentUpdateRunning ) {
            MUJIN_LOG_WARN("environment update thread stopped! so stopping detector");
            break;
        }

        if (resultstate != "null" && resultstate.size() > 0) {
            boost::mutex::scoped_lock lock(_mutexDetectedInfo);
            if( !detectcontaineronly ) {
                // only update the objects if detector actually returned them, otherwise will be erasing previously sent good results
                _vDetectedObject.swap(detectedobjects);
                _bDetectedObjectsValid = true;
            }
            else {
                _bDetectedObjectsValid = false;
            }
            _resultState = resultstate;
            _resultTimestamp = GetMilliTime();
            _resultImageStartTimestamp = imageStartTimestamp;
            _resultImageEndTimestamp = imageEndTimestamp;
            if (_bDetectedObjectsValid) {
                MUJIN_LOG_INFO(str(boost::format("send %d detected objects with _resultTimestamp=%u, imageStartTimestamp=%u imageEndTimestamp=%u resultstate=%s")%_vDetectedObject.size()%_resultTimestamp%imageStartTimestamp%_resultImageEndTimestamp%resultstate));
            }
            else {
                MUJIN_LOG_INFO(str(boost::format("send resultstate with _resultTimestamp=%u, imageStartTimestamp=%u imageEndTimestamp=%u resultstate=%s")%_resultTimestamp%imageStartTimestamp%_resultImageEndTimestamp%resultstate));
            }
        }
        else {
            MUJIN_LOG_INFO("resultstate is null, do not update result");
        }
        if (_bStopDetectionThread) {
            break;
        }

        MUJIN_LOG_INFO("Cycle time: " + boost::lexical_cast<std::string>((GetMilliTime() - starttime)/1000.0f) + " secs");
        if( bDetectorHasRunThisCycle ) {
            numdetection += 1;
        }
    }
    if (stoponleftinorder && numLeftInOrder == 0) {
        MUJIN_LOG_INFO("got out of detection loop because numLeftInOrder is 0, wait for environment to update");
        while (_resultTimestamp > _tsLastEnvUpdate && !_bStopDetectionThread) { // have to check that detection was not stopped, or else can spin here forever since update thread would have been stopped.
            boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        }
        MUJIN_LOG_INFO("environment is updated with latest result, stop environment updating and capturing");
        MUJIN_LOG_INFO("stopped environment update thread");
        // stop capturing by removing capture handles
//        capturehandles.resize(0);
//        MUJIN_LOG_INFO("capturing stopped");
        _StopUpdateEnvironmentThread();
        _StopExecutionVerificationPointCloudThread();
    }
    if (bDetectorHasRunAtLeastOnce && numdetection >= maxnumdetection && maxnumdetection!=0) {
        MUJIN_LOG_INFO("reached max num detection, wait for environment to update");
        while (_resultTimestamp > _tsLastEnvUpdate && !_bStopDetectionThread) { // have to check that detection was not stopped, or else can spin here forever since update thread would have been stopped.
            boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        }
        MUJIN_LOG_INFO("environment is updated with latest result, stop environment updating and capturing");
        // since threads might be blocking on waiting for captures, so stop capturing to enable the preempt function to exit
        // stop capturing by removing capture handles
//        capturehandles.resize(0);
//        MUJIN_LOG_INFO("capturing stopped");
        _StopUpdateEnvironmentThread();
        _StopExecutionVerificationPointCloudThread();
        MUJIN_LOG_INFO("stopped environment update thread");
    }

    if (lastGrabbedTargetTimestamp > _tsLastEnvUpdate) {
        MUJIN_LOG_WARN("_tsLastEnvUpdate=" << _tsLastEnvUpdate << " is older than lastGrabbedTargetTimestamp=" << lastGrabbedTargetTimestamp << "!");
    }

    MUJIN_LOG_INFO("ending detection thread. numdetection=" << numdetection << " numLeftInOrder=" << numLeftInOrder << " _bStopDetectionThread=" << _bStopDetectionThread << " lastGrabbedTargetTimestamp=" << lastGrabbedTargetTimestamp << " _tsLastEnvUpdate=" << _tsLastEnvUpdate << " _tsStartDetection=" << _tsStartDetection << " _resultImageEndTimestamp" << _resultImageEndTimestamp << " _resultTimestamp=" << _resultTimestamp);
}

void MujinVisionManager::_UpdateEnvironmentThread(UpdateEnvironmentThreadParams params)
{
    try {
        FalseSetter turnoffstatusvar(_bIsEnvironmentUpdateRunning);
        std::string regionname = params.regionname;
        std::vector<std::string> cameranames = params.cameranames;
        double pointsize = 0;
        std::string locationIOName;
        {
            boost::mutex::scoped_lock lock(_mutexRegion);
            pointsize = _mNameRegion[regionname]->pRegionParameters->pointsize;
            MUJIN_LOG_INFO("pointsize=0, using pointsize= " << pointsize << " in regionparam");
            locationIOName = _mNameRegion[regionname]->pRegionParameters->locationIOName;
        }
        std::string obstaclename = params.obstaclename;
        unsigned int waitinterval = params.waitinterval;
        std::string locale = params.locale;
        std::vector<DetectedObjectPtr> vDetectedObject; ///< latest detection result
        std::string resultstate;


        uint64_t lastUpdateTimestamp = GetMilliTime();
        std::vector<std::string> cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);

        BinPickingTaskResourcePtr pBinpickingTask = _pSceneResource->GetOrCreateBinPickingTaskFromName_UTF8(_tasktype+std::string("task1"), _tasktype, TRO_EnableZMQ);
        std::string userinfo = _GetUserInfoJsonString();
        MUJIN_LOG_DEBUG("initialzing binpickingtask in UpdateEnvironmentThread with userinfo " + userinfo);
        pBinpickingTask->Initialize(_defaultTaskParameters, _binpickingTaskZmqPort, _binpickingTaskHeartbeatPort, _zmqcontext, false, 0/*do not start monitor thread*/, _controllerCommandTimeout, userinfo, _slaverequestid);
        uint64_t starttime;
        uint64_t lastwarnedtimestamp1 = 0;

        std::vector<BinPickingTaskResource::DetectedObject> detectedobjects;
        std::vector<Real> totalpoints;
        std::map<std::string, std::vector<Real> > mResultPoints;
        std::vector<Real> newpoints;
        std::vector<CameraCaptureHandlePtr> capturehandles; CREATE_SAFE_DELETER_CAMERAHANDLES(capturehandles);

        // do not ensure capturing here, it should be done inside threads that actually process the new images
        //MUJIN_LOG_DEBUG("_StartAndGetCaptureHandle with cameranames " << __GetString(cameranames));
        //_StartAndGetCaptureHandle(cameranames, cameranames, capturehandles);
        while (!_bStopUpdateEnvironmentThread) {
            bool update = false;
            bool bDetectedObjectsValid = false;
            {
                boost::mutex::scoped_lock lock(_mutexDetectedInfo);
                update = _resultTimestamp != 0 && _resultTimestamp > lastUpdateTimestamp;
                vDetectedObject = _vDetectedObject;
                resultstate = _resultState;
                bDetectedObjectsValid = _bDetectedObjectsValid;
                mResultPoints = _mResultPoints;
            }

            if (!update) {
                //MUJIN_LOG_DEBUG(str(boost::format("have no detector results to update _resultTimestamp (%u) <= lastUpdateTimestamp (%u), waitfor %dms")%_resultTimestamp%lastUpdateTimestamp%waitinterval));
                boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                continue;
            }
            else {
                lastUpdateTimestamp = _resultTimestamp;
                MUJIN_LOG_INFO(str(boost::format("updating environment with %d detected objects, valid=%d")%vDetectedObject.size()%bDetectedObjectsValid));
                detectedobjects.resize(0);
                totalpoints.resize(0);
                if( bDetectedObjectsValid ) {
                    // use the already acquired detection results without locking
                    unsigned int nameind = 0;
                    DetectedObjectPtr detectedobject;
                    RegionPtr region;
                    TransformMatrix O_T_region, O_T_baselinkcenter, baselinkcenter_T_region;
                    unsigned int numUpdatedRegions = 0;
                    for (unsigned int i=0; i<vDetectedObject.size(); i++) {
                        Transform newtransform;
                        BinPickingTaskResource::DetectedObject detectedobject;
//                        if (vDetectedObject[i]->type == "container") {
//                            if (numUpdatedRegions > 0) {
//                                MUJIN_LOG_WARN("more than 1 container is returned in detectedobjects, do not update this object!");
//                                continue;
//                            }
                        {
                            boost::mutex::scoped_lock lock(_mutexRegion);
                            if (_mNameRegion.find(vDetectedObject[i]->name) != _mNameRegion.end()) {
                                newtransform = vDetectedObject[i]->transform; // use result directly for container
                                // a region is being updated
                                //MUJIN_LOG_WARN("container " << regionname << " is unknown, do not update this object");
                                //continue;
                                // use detected object name for regionname
                                detectedobject.name = vDetectedObject[i]->name;
                                // convert O_T_baselinkcenter to O_T_region, because detector assumes O_T_baselinkcenter, controller asumes O_T_region
                                region = _mNameRegion[detectedobject.name];
                                baselinkcenter_T_region = region->pRegionParameters->baselinkcenter_T_region;
                                O_T_baselinkcenter = newtransform;
                                O_T_region = O_T_baselinkcenter * baselinkcenter_T_region;
                                newtransform = O_T_region;
                                numUpdatedRegions++;
                            } else {
                                newtransform = _tWorldResultOffset * vDetectedObject[i]->transform; // apply offset to result transform
                                // overwrite name because we need to add id to the end
                                std::stringstream name_ss;
                                name_ss << _targetupdatename << nameind;
                                detectedobject.name = name_ss.str();
                                nameind++;
                            }
                        }

                        // convert to mujinclient format
                        mujinclient::Transform transform;
                        transform.quaternion[0] = newtransform.rot[0];
                        transform.quaternion[1] = newtransform.rot[1];
                        transform.quaternion[2] = newtransform.rot[2];
                        transform.quaternion[3] = newtransform.rot[3];
                        transform.translate[0] = newtransform.trans[0];
                        transform.translate[1] = newtransform.trans[1];
                        transform.translate[2] = newtransform.trans[2];

                        detectedobject.object_uri = vDetectedObject[i]->objecturi;
                        detectedobject.transform = transform;
                        detectedobject.confidence = vDetectedObject[i]->confidence;
                        detectedobject.timestamp = vDetectedObject[i]->timestamp;
                        detectedobject.extra = vDetectedObject[i]->extra;
                        detectedobjects.push_back(detectedobject);
                    }
                    for(unsigned int i=0; i<cameranamestobeused.size(); i++) {
                        std::string cameraname = cameranamestobeused[i];
                        std::map<std::string, std::vector<Real> >::const_iterator itpoints = mResultPoints.find(cameraname);
                        if( itpoints != mResultPoints.end() ) {
                            // get point cloud obstacle
                            newpoints.resize(itpoints->second.size());
                            for (size_t j=0; j<itpoints->second.size(); j+=3) {
                                Vector newpoint = _tWorldResultOffset * Vector(itpoints->second.at(j), itpoints->second.at(j+1), itpoints->second.at(j+2));
                                newpoints[j] = newpoint.x;
                                newpoints[j+1] = newpoint.y;
                                newpoints[j+2] = newpoint.z;
                            }
                            totalpoints.insert(totalpoints.end(), newpoints.begin(), newpoints.end());
                        }
                    }
                    try {
                        MUJIN_LOG_DEBUG(str(boost::format("detectedobjects.size()=%d, _targetupdatename=%s")%detectedobjects.size()%_targetupdatename));
                        starttime = GetMilliTime();
                        pBinpickingTask->UpdateEnvironmentState(_targetupdatename, detectedobjects, totalpoints, resultstate, pointsize, obstaclename, "mm", 10, regionname, locationIOName);
                        std::stringstream ss;
                        ss << "UpdateEnvironmentState with " << detectedobjects.size() << " objects " << (totalpoints.size()/3.) << " points, took " << (GetMilliTime() - starttime) / 1000.0f << " secs";
                        _SetStatusMessage(TT_UpdateEnvironment, ss.str());
                    }
                    catch(const std::exception& ex) {
                        if ( lastwarnedtimestamp1 == 0 || GetMilliTime() - lastwarnedtimestamp1 > 1000) {
                            lastwarnedtimestamp1 = GetMilliTime();
                            std::stringstream ss;
                            ss << "Failed to update environment state: " << ex.what() << ".";
                            //std::string errstr = ParametersBase::GetExceptionJsonString(GetErrorCodeString(MVE_ControllerError), ss.str());
                            _SetStatusMessage(TT_UpdateEnvironment, ss.str(), GetErrorCodeString(MVE_ControllerError));
                            MUJIN_LOG_WARN(ss.str());
                        }
                        boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                    }
                    _tsLastEnvUpdate = _resultTimestamp;
                }
                else {
                    // only resultstate is valid and shouldn't update the detected objects or pointcloud... (hack for now)
                    try {
                        starttime = GetMilliTime();
                        pBinpickingTask->UpdateObjects("", std::vector<mujinclient::Transform>(), std::vector<std::string>(), resultstate, "mm", 10.0);
                    }
                    catch(const std::exception& ex) {
                        if (GetMilliTime() - lastwarnedtimestamp1 > 1000.0) {
                            lastwarnedtimestamp1 = GetMilliTime();
                            std::stringstream ss;
                            ss << "Failed to update objects state: " << ex.what() << ".";
                            _SetStatusMessage(TT_UpdateEnvironment, ss.str(), GetErrorCodeString(MVE_ControllerError));
                            MUJIN_LOG_WARN(ss.str());
                        }
                        boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                    }
                    _tsLastEnvUpdate = _resultTimestamp;
                }
            }
        }
    }
    catch (const zmq::error_t& e) {
        std::stringstream errss;
        errss << "Caught zmq exception errornum=" << e.num() << " in _UpdateEnvironmentThread!";
        _SetStatus(TT_UpdateEnvironment, MS_Aborted, errss.str(), "", false);
        MUJIN_LOG_ERROR(errss.str());
    }
    catch (const MujinVisionException& e) {
        std::stringstream errss;
        errss << "Caught MujinVisionException " << e.message() << " in _UpdateEnvironmentThread!";
        _SetStatus(TT_UpdateEnvironment, MS_Aborted, errss.str(), "", false);
        MUJIN_LOG_ERROR(errss.str());
    }
    catch (const mujinclient::UserInterruptException& ex) {
        std::stringstream errss;
        errss << "User interrupted _UpdateEnvironmentThread!";
        _SetStatus(TT_UpdateEnvironment, MS_Preempted, errss.str(), "", false);
        MUJIN_LOG_WARN(errss.str());
    }
    catch(const std::exception& ex) {
        std::stringstream errss;
        errss << "Caught exception in _UpdateEnvironmentThread " << ex.what();
        _SetStatus(TT_UpdateEnvironment, MS_Preempted, errss.str(), "", false);
        MUJIN_LOG_WARN(errss.str());
    }
    catch (...) {
        std::stringstream errss;
        errss << "Caught unknown exception in _UpdateEnvironmentThread!";
        _SetStatus(TT_UpdateEnvironment, MS_Aborted, errss.str(), "", false);
        MUJIN_LOG_ERROR(errss.str());
    }
}

std::string MujinVisionManager::_GetUserInfoJsonString() {
    rapidjson::Document userinfojson(rapidjson::kObjectType);
    SetJsonValueByKey(userinfojson, "username", _pControllerClient->GetUserName());
    SetJsonValueByKey(userinfojson, "locale", _locale);
    return DumpJson(userinfojson);
}

void MujinVisionManager::_SendExecutionVerificationPointCloudThread(SendExecutionVerificationPointCloudParams params)
{
    try {
        bool hasRobotExecutionStarted = false;
        do { // wait until robot starts
            {
                boost::mutex::scoped_lock(_mutexControllerBinpickingState);
                hasRobotExecutionStarted = _bHasRobotExecutionStarted;
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(20));
        } while (!hasRobotExecutionStarted && !_bStopControllerMonitorThread);

        FalseSetter turnoffstatusvar(_bIsExecutionVerificationPointCloudRunning);
        std::vector<std::string> cameranames = params.cameranames;
        std::vector<std::string> evcamnames = params.executionverificationcameranames;
        MUJIN_LOG_INFO("starting SendExecutionVerificationPointCloudThread " + GetJsonString(evcamnames));
        bool ignoreocclusion = params.ignoreocclusion;
        //std::string obstaclename = params.obstaclename;
        unsigned int waitinterval = params.waitinterval;
        std::string locale = params.locale;
        std::vector<DetectedObjectPtr> vDetectedObject; ///< latest detection result
        std::string resultstate;

        std::vector<CameraCaptureHandlePtr> capturehandles; CREATE_SAFE_DELETER_CAMERAHANDLES(capturehandles);

        //uint64_t lastUpdateTimestamp = GetMilliTime();
        //std::vector<std::string> cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);

        BinPickingTaskResourcePtr pBinpickingTask = _pSceneResource->GetOrCreateBinPickingTaskFromName_UTF8(_tasktype+std::string("task1"), _tasktype, TRO_EnableZMQ);

        std::string userinfo = _GetUserInfoJsonString();

        MUJIN_LOG_DEBUG("initialzing binpickingtask in _SendExecutionVerificationPointCloudThread with userinfo " + userinfo);

        pBinpickingTask->Initialize(_defaultTaskParameters, _binpickingTaskZmqPort, _binpickingTaskHeartbeatPort, _zmqcontext, false, 0/*do not start monitor thread*/, _controllerCommandTimeout, userinfo, _slaverequestid);
        //uint64_t starttime;
        uint64_t lastwarnedtimestamp0 = 0;
        //uint64_t lastwarnedtimestamp1 = 0;
        std::map<std::string, uint64_t> mCameranameLastsentcloudtime;
        std::string regionname;

        uint64_t lastCaptureResetTimestamp = 0; // ms timestamp when the capture handles were last reset. Used to prevent too many force resets in one time.
        uint64_t lastCaptureResetTimeout = 4000; // how long to wait until force reset is called again
        //uint64_t lastUpdateTimestamp = 0;
        while (!_bStopExecutionVerificationPointCloudThread && evcamnames.size() > 0) {
            // send latest pointcloud for execution verification
            for (unsigned int i=0; i<evcamnames.size(); ++i) {
                std::vector<double> points;
                std::string cameraname = evcamnames.at(i);
                unsigned long long cloudstarttime, cloudendtime;
                // ensure publishing
                MUJIN_LOG_DEBUG("_StartAndGetCaptureHandle with cameranames " << __GetString(evcamnames));
                _StartAndGetCaptureHandle(evcamnames, evcamnames, capturehandles, false, ignoreocclusion);
                double newpointsize = 0;
                size_t filteringsubsample = 0;
                double filteringstddev = 0;
                int filteringnumnn = 0;

                {
                    boost::mutex::scoped_lock lock(_mutexRegion);
                    if (_mCameranameActiveRegionname.find(cameraname) == _mCameranameActiveRegionname.end()) {
                        MUJIN_LOG_WARN("cannot check occlusion for camera " << cameraname << " (" << _GetHardwareId(cameraname) << "), because no region is mapped to it");
                        regionname = "";
                    } else {
                        regionname = _mCameranameActiveRegionname[cameraname];
                    }
                    if (regionname.size() > 0 && _mNameRegion.find(regionname) != _mNameRegion.end()) {
                        RegionParametersPtr pregion = _mNameRegion[regionname]->pRegionParameters;
                        filteringsubsample = pregion->filteringsubsample;
                        filteringstddev = pregion->filteringstddev;
                        filteringnumnn = pregion->filteringnumnn;
                        newpointsize = pregion->pointsize;
                        MUJIN_LOG_INFO("using pointsize= " << newpointsize << " in regionparam for camera " << cameraname  << " (" << _GetHardwareId(cameraname) << ") of region " << regionname);
                    } else {
                        newpointsize = 10;
                        MUJIN_LOG_INFO("pointsize=0 but no active region is found for this camera " << cameraname << " (" << _GetHardwareId(cameraname) << "), use default value=" << newpointsize);
                    }
                }

                if( filteringnumnn == 0 ) {
                    filteringnumnn = _filteringnumnn;
                }
                if( filteringsubsample == 0 ) {
                    filteringsubsample = _filteringsubsample;
                }
                if( filteringstddev == 0 ) {
                    filteringstddev = _filteringstddev;
                }
                MUJIN_LOG_DEBUG("pointsize=" << newpointsize << ", and filteringstddev=" << filteringstddev << ", and filteringnumnn=" << filteringnumnn);
                double timeout = 3.0; // secs
                int isoccluded = -1;
                isoccluded = _pImagesubscriberManager->GetCollisionPointCloud(cameraname, points, cloudstarttime, cloudendtime, newpointsize, filteringstddev, filteringnumnn, regionname, timeout, 0, filteringsubsample, PC_ExecutionVerificationThread);
                if (isoccluded == -2 ) {
                    MUJIN_LOG_DEBUG("did not get depth from " << cameraname << " (" << _GetHardwareId(cameraname) << "), so do not send to controller");
                    if( lastCaptureResetTimestamp == 0 || GetMilliTime() - lastCaptureResetTimestamp > lastCaptureResetTimeout ) {
                        lastCaptureResetTimestamp = GetMilliTime();

                        //MUJIN_LOG_WARN("reset image subscriber");
                        //_pImagesubscriberManager->Reset();
                        MUJIN_LOG_DEBUG("try to force capturing");
                        MUJIN_LOG_DEBUG("_StartAndGetCaptureHandle with cameranames " << __GetString(evcamnames));
                        _StartAndGetCaptureHandle(evcamnames, evcamnames, capturehandles, true, ignoreocclusion);
                    }
                } else if (mCameranameLastsentcloudtime.find(cameraname) == mCameranameLastsentcloudtime.end() || cloudstarttime > mCameranameLastsentcloudtime[cameraname]) {
                    if( points.size() == 0 ) {
                        MUJIN_LOG_WARN("sending 0 points from camera " << cameraname << " (" << _GetHardwareId(cameraname) << ")");
                    }
                    try {
                        uint64_t starttime = GetMilliTime();
                        pBinpickingTask->AddPointCloudObstacle(points, newpointsize, "latestobstacle_"+cameraname, cloudstarttime, cloudendtime, true, "mm", isoccluded, regionname);
                        mCameranameLastsentcloudtime[cameraname] = cloudstarttime;
                        std::stringstream ss;
                        ss << "Sent latest pointcloud of " << cameraname << " (" << _GetHardwareId(cameraname) << ") with " << (points.size()/3.) << " points, region=" << regionname << ", isoccluded=" << isoccluded << ", took " << (GetMilliTime() - starttime) / 1000.0f << " secs";
                        MUJIN_LOG_DEBUG(ss.str());
                    } catch(const std::exception& ex) {
                        if (GetMilliTime() - lastwarnedtimestamp0 > 1000.0) {
                            lastwarnedtimestamp0 = GetMilliTime();
                            std::stringstream ss;
                            ss << "Failed to send latest pointcloud of " << cameraname << " (" << _GetHardwareId(cameraname) << ") ex.what()=" << ex.what() << ".";
                            _SetStatusMessage(TT_UpdateEnvironment, ss.str(), GetErrorCodeString(MVE_ControllerError));
                            MUJIN_LOG_WARN(ss.str());
                        }
                    }
                } else {
                    MUJIN_LOG_INFO("got old point cloud from camera " << cameraname << " (" << _GetHardwareId(cameraname) << "), do not send to controller. cloudstarttime=" << cloudstarttime << " oldtime=" << mCameranameLastsentcloudtime[cameraname]);

                    if (GetMilliTime() - mCameranameLastsentcloudtime[cameraname] > 10 * 1000.0) {
                        MUJIN_LOG_INFO("it has been " << (GetMilliTime() - mCameranameLastsentcloudtime[cameraname]) / 1000.0f << " secs since we got the last pointcloud, cameras might be stopped, try to force capturing");
                        MUJIN_LOG_DEBUG("_StartAndGetCaptureHandle with cameranames " << __GetString(evcamnames));
                        _StartAndGetCaptureHandle(evcamnames, evcamnames, capturehandles, true, ignoreocclusion);
                    }
                }
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
        }
    }
    catch (const zmq::error_t& e) {
        std::stringstream errss;
        errss << "Caught zmq exception errornum=" << e.num() << " in _SendExecutionVerificationPointCloudThread!";
        _SetStatus(TT_SendExecutionVerificationPointCloud, MS_Aborted, errss.str(), "", false);
        MUJIN_LOG_ERROR(errss.str());
    }
    catch (const MujinVisionException& e) {
        std::stringstream errss;
        errss << "Caught MujinVisionException " << e.message() << " in _SendExecutionVerificationPointCloudThread!";
        _SetStatus(TT_SendExecutionVerificationPointCloud, MS_Aborted, errss.str(), "", false);
        MUJIN_LOG_ERROR(errss.str());
    }
    catch (const mujinclient::UserInterruptException& ex) {
        std::stringstream errss;
        errss << "User interrupted _SendExecutionVerificationPointCloudThread!";
        _SetStatus(TT_SendExecutionVerificationPointCloud, MS_Preempted, errss.str(), "", false);
        MUJIN_LOG_WARN(errss.str());
    }
    catch(const std::exception& ex) {
        std::stringstream errss;
        errss << "Caught exception in _SendExecutionVerificationPointCloudThread " << ex.what();
        _SetStatus(TT_SendExecutionVerificationPointCloud, MS_Preempted, errss.str(), "", false);
        MUJIN_LOG_WARN(errss.str());
    }
    catch (...) {
        std::stringstream errss;
        errss << "Caught unknown exception in VisualizePointcloudThread!";
        _SetStatus(TT_SendExecutionVerificationPointCloud, MS_Aborted, errss.str(), "", false);
        MUJIN_LOG_ERROR(errss.str());
    }
}

void MujinVisionManager::_ControllerMonitorThread(const unsigned int waitinterval, const std::string& locale)
{
    try {
        BinPickingTaskResourcePtr pBinpickingTask = _pSceneResource->GetOrCreateBinPickingTaskFromName_UTF8(_tasktype+std::string("task1"), _tasktype, TRO_EnableZMQ);
        std::string userinfo = _GetUserInfoJsonString();
        pBinpickingTask->Initialize(_defaultTaskParameters, _binpickingTaskZmqPort, _binpickingTaskHeartbeatPort, _zmqcontext, false, _binpickingTaskHeartbeatTimeout, _controllerCommandTimeout, userinfo, _slaverequestid);
        BinPickingTaskResource::ResultGetBinpickingState binpickingstate;
        uint64_t lastwarnedtimestamp = 0;
        while (!_bStopControllerMonitorThread) {
            uint64_t lastUpdateTimestamp;
            {
                boost::mutex::scoped_lock lock(_mutexControllerBinpickingState);
                try {
                    pBinpickingTask->GetPublishedTaskState(binpickingstate, "", "mm", 1.0);
                }
                catch(const std::exception& ex) {
                    if (GetMilliTime() - lastwarnedtimestamp > 1000.0) {
                        lastwarnedtimestamp = GetMilliTime();
                        std::stringstream ss;
                        ss << "Failed to get published task state from mujin controller: " << ex.what() << ".";
                        _SetStatusMessage(TT_ControllerMonitor, ss.str(), GetErrorCodeString(MVE_ControllerError));
                        MUJIN_LOG_WARN(ss.str());
                    }
                    boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                    continue;
                }
                _bIsControllerPickPlaceRunning = (binpickingstate.statusPickPlace == "Running");
                _bIsRobotOccludingSourceContainer = binpickingstate.isRobotOccludingSourceContainer;
                _bForceRequestDetectionResults = binpickingstate.forceRequestDetectionResults;
                _numPickAttempt = binpickingstate.pickAttemptFromSourceId;
                _binpickingstateTimestamp = binpickingstate.timestamp;
                _lastGrabbedTargetTimestamp = binpickingstate.lastGrabbedTargetTimeStamp;
                _bIsGrabbingTarget = binpickingstate.isGrabbingTarget;
                _bIsGrabbingLastTarget = binpickingstate.isGrabbingLastTarget;
                _bHasRobotExecutionStarted = binpickingstate.hasRobotExecutionStarted;
                _orderNumber = binpickingstate.orderNumber;
                _numLeftInOrder = binpickingstate.numLeftInOrder;
                _numLeftInSupply = binpickingstate.numLeftInSupply;
                _placedInDest = binpickingstate.placedInDest;
                lastUpdateTimestamp = GetMilliTime();
            }

            uint64_t dt = GetMilliTime() - lastUpdateTimestamp;

            if (dt < waitinterval) {
                boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval- dt));
            }
        }
    }
    catch (const zmq::error_t& e) {
        std::stringstream errss;
        errss << "Caught zmq exception errornum=" << e.num() << " in _ControllerMonitorThread!";
        _SetStatus(TT_ControllerMonitor, MS_Aborted, errss.str(), "", false);
        MUJIN_LOG_ERROR(errss.str());
    }
    catch (const MujinVisionException& e) {
        std::stringstream errss;
        errss << "Caught MujinVisionException " << e.message() << " in _ControllerMonitorThread!";
        _SetStatus(TT_ControllerMonitor, MS_Aborted, errss.str(), "", false);
        MUJIN_LOG_ERROR(errss.str());
    }
    catch (const mujinclient::UserInterruptException& ex) {
        std::stringstream errss;
        errss << "User interrupted _ControllerMonitorThread!";
        _SetStatus(TT_ControllerMonitor, MS_Preempted, errss.str(), "", false);
        MUJIN_LOG_WARN(errss.str());
    }
    catch(const std::exception& ex) {
        std::stringstream errss;
        errss << "Caught exception in _ControllerMonitorThread " << ex.what();
        _SetStatus(TT_ControllerMonitor, MS_Preempted, errss.str(), "", false);
        MUJIN_LOG_WARN(errss.str());
    }
    catch (...) {
        std::stringstream errss;
        errss << "Caught unknown exception in VisualizePointcloudThread!";
        _SetStatus(TT_ControllerMonitor, MS_Aborted, errss.str(), "", false);
        MUJIN_LOG_ERROR(errss.str());
    }
}

void MujinVisionManager::_VisualizePointCloudThread(VisualizePointcloudThreadParams params)
{
    try {
        FalseSetter turnOffVisualize(_bIsVisualizePointcloudRunning);
        std::string regionname = params.regionname;
        std::vector<std::string> cameranames = params.cameranames;
        std::vector<std::string> emptycameranames;
        double pointsize = params.pointsize;
        if (pointsize == 0) {
            boost::mutex::scoped_lock lock(_mutexRegion);
            pointsize = _mNameRegion[regionname]->pRegionParameters->pointsize;
            MUJIN_LOG_INFO("pointsize=0, using pointsize= " << pointsize << " in regionparam");
        }
        bool ignoreocclusion = params.ignoreocclusion;
        unsigned long long newerthantimestamp = params.newerthantimestamp;
        unsigned int fetchimagetimeout = params.fetchimagetimeout;
        bool request = params.request;
        std::vector<CameraCaptureHandlePtr> capturehandles; CREATE_SAFE_DELETER_CAMERAHANDLES(capturehandles);
        while (!_bStopVisualizePointCloudThread) {
            MUJIN_LOG_DEBUG("_StartAndGetCaptureHandle with cameranames " << __GetString(cameranames));
            _StartAndGetCaptureHandle(cameranames, emptycameranames, capturehandles, true, ignoreocclusion);
            SyncCameras(regionname, cameranames);
            if (_bStopVisualizePointCloudThread) {
                break;
            }
            VisualizePointCloudOnController(regionname, cameranames, pointsize, ignoreocclusion, newerthantimestamp, fetchimagetimeout, request);
        }
    }
    catch (const zmq::error_t& e) {
        std::stringstream errss;
        errss << "Caught zmq exception errornum=" << e.num() << " in _VisualizePointCloudThread!";
        _SetStatus(TT_VisualizePointCloud, MS_Aborted, errss.str(), "", false);
        MUJIN_LOG_ERROR(errss.str());
    }
    catch (const MujinVisionException& e) {
        std::stringstream errss;
        errss << "Caught MujinVisionException " << e.message() << " in _VisualizePointCloudThread!";
        _SetStatus(TT_VisualizePointCloud, MS_Aborted, errss.str(), "", false);
        MUJIN_LOG_ERROR(errss.str());
    }
    catch (const mujinclient::UserInterruptException& ex) {
        std::stringstream errss;
        errss << "User interrupted _VisualizePointCloudThread!";
        _SetStatus(TT_VisualizePointCloud, MS_Preempted, errss.str(), "", false);
        MUJIN_LOG_WARN(errss.str());
    }
    catch(const std::exception& ex) {
        std::stringstream errss;
        errss << "Caught exception in _VisualizePointCloudThread " << ex.what();
        _SetStatus(TT_VisualizePointCloud, MS_Preempted, errss.str(), "", false);
        MUJIN_LOG_WARN(errss.str());
    }
    catch (...) {
        std::stringstream errss;
        errss << "Caught unknown exception in VisualizePointcloudThread!";
        _SetStatus(TT_VisualizePointCloud, MS_Aborted, errss.str(), "", false);
        MUJIN_LOG_ERROR(errss.str());
    }
}

mujinvision::Transform MujinVisionManager::_GetTransform(const std::string& instobjname)
{
    mujinclient::Transform t;
    _pBinpickingTask->GetTransform(instobjname,t,"mm");
    return _GetTransform(t);
}

void MujinVisionManager::_SyncCamera(const std::string& cameraname)
{
    if (_mNameCamera.find(cameraname) == _mNameCamera.end()) {
        throw MujinVisionException("Camera "+cameraname+ " is unknown!", MVE_InvalidArgument);
    }

    RobotResource::AttachedSensorResource::SensorData sensordata;
    std::string camerabodyname, sensorname;
    _ParseCameraName(cameraname, camerabodyname, sensorname);
    utils::GetSensorData(_pControllerClient, _pSceneResource, camerabodyname, sensorname, sensordata);

    mujinclient::Transform O_T_C0;
    utils::GetSensorTransform(_pControllerClient, _pSceneResource, camerabodyname, sensorname, O_T_C0, "mm");
    Transform O_T_C = _GetTransform(O_T_C0); // sensor transform in world frame
    _mNameCamera[cameraname]->SetWorldTransform(O_T_C);
    MUJIN_LOG_DEBUG("setting camera transform for " << cameraname  << " (" << _GetHardwareId(cameraname) << ") to:\n" << _GetString(_mNameCamera[cameraname]->GetWorldTransform()));
}

void MujinVisionManager::_SyncCamera(const std::string& cameraname, const mujinclient::Transform& t)
{
    if (_mNameCamera.find(cameraname) == _mNameCamera.end()) {
        throw MujinVisionException("Camera "+cameraname+ " is unknown!", MVE_InvalidArgument);
    }
    Transform O_T_C = _GetTransform(t); // sensor transform in world frame
    _mNameCamera[cameraname]->SetWorldTransform(O_T_C);
    MUJIN_LOG_DEBUG("setting camera transform for " << cameraname  << " (" << _GetHardwareId(cameraname) << ") to:\n" + _GetString(_mNameCamera[cameraname]->GetWorldTransform()));
}

void MujinVisionManager::_SyncRegion(const std::string& regionname)
{
    mujinvision::Transform regiontransform = _GetTransform(regionname);
    MUJIN_LOG_DEBUG("Computing globalroi3d from mujin controller.");
    // get axis aligned bounding box for region
    BinPickingTaskResource::ResultOBB baselinkobb;
    _pBinpickingTask->GetOBB(baselinkobb, regionname, "base", "mm");
    // get inner obb from mujin controller
    MUJIN_LOG_DEBUG("getting obb from mujin controller.");
    BinPickingTaskResource::ResultOBB innerobb;
    _pBinpickingTask->GetInnerEmptyRegionOBB(innerobb, regionname, "base", "mm");
    _SyncRegion(regionname, regiontransform, baselinkobb, innerobb);
}

void MujinVisionManager::_SyncRegion(const std::string& regionname, const mujinvision::Transform& O_T_region, const BinPickingTaskResource::ResultOBB& baselinkobb, const BinPickingTaskResource::ResultOBB& innerobb)
{
    boost::mutex::scoped_lock lock(_mutexRegion);
    //_mNameRegion[regionname]->SetWorldTransform(regiontransform);
    MUJIN_LOG_DEBUG(std::string("setting region ") + regionname + std::string(" transform to:\n") + _GetString(O_T_region)); //_mNameRegion[regionname]->GetWorldTransform()));
    // update globalroi3d from mujin controller
    _mNameRegion[regionname]->pRegionParameters->outerTranslation = baselinkobb.translation;
    _mNameRegion[regionname]->pRegionParameters->outerExtents = baselinkobb.extents;
    _mNameRegion[regionname]->pRegionParameters->outerRotationmat = baselinkobb.rotationmat;
    _mNameRegion[regionname]->pRegionParameters->innerTranslation = innerobb.translation;
    _mNameRegion[regionname]->pRegionParameters->innerExtents = innerobb.extents;
    _mNameRegion[regionname]->pRegionParameters->innerRotationmat = innerobb.rotationmat;
    Transform O_T_baselinkcenter;
    // define the center of where vision results return the container
    O_T_baselinkcenter.trans[0] = baselinkobb.translation[0];
    O_T_baselinkcenter.trans[1] = baselinkobb.translation[1];
    O_T_baselinkcenter.trans[2] = baselinkobb.translation[2];
    if( 0 ) { // TODO
        // top center face (on +Z)
        O_T_baselinkcenter.trans[0] += baselinkobb.rotationmat[2]*baselinkobb.extents[2];
        O_T_baselinkcenter.trans[1] += baselinkobb.rotationmat[5]*baselinkobb.extents[2];
        O_T_baselinkcenter.trans[2] += baselinkobb.rotationmat[8]*baselinkobb.extents[2];
    }
    TransformMatrix matrix;
    matrix.m[0] = baselinkobb.rotationmat[0]; matrix.m[1] = baselinkobb.rotationmat[1]; matrix.m[2] = baselinkobb.rotationmat[2];
    matrix.m[4] = baselinkobb.rotationmat[3]; matrix.m[5] = baselinkobb.rotationmat[4]; matrix.m[6] = baselinkobb.rotationmat[5];
    matrix.m[8] = baselinkobb.rotationmat[6]; matrix.m[9] = baselinkobb.rotationmat[7]; matrix.m[10] = baselinkobb.rotationmat[8];

    Vector quat = quatFromMatrix(matrix);
    O_T_baselinkcenter.rot[0] = quat[0];
    O_T_baselinkcenter.rot[1] = quat[1];
    O_T_baselinkcenter.rot[2] = quat[2];
    O_T_baselinkcenter.rot[3] = quat[3];
    _mNameRegion[regionname]->pRegionParameters->baselinkcenter_T_region = O_T_baselinkcenter.inverse() * O_T_region;
}

void MujinVisionManager::RegisterCustomCommand(const std::string& cmdname, CustomCommandFn fncmd)
{
    if((cmdname.size() == 0) || (cmdname == "commands")) {
        throw MujinVisionException(boost::str(boost::format("command '%s' invalid")%cmdname), MVE_Failed);
    }
    if(_mNameCommand.find(cmdname) != _mNameCommand.end()) {
        throw MujinVisionException(str(boost::format("command '%s' already registered")%cmdname),MVE_Failed);
    }
    _mNameCommand[cmdname] = boost::shared_ptr<CustomCommand>(new CustomCommand(fncmd));
}

void MujinVisionManager::UnregisterCommand(const std::string& cmdname)
{
    if(_mNameCommand.find(cmdname) != _mNameCommand.end()) {
        _mNameCommand.erase(_mNameCommand.find(cmdname));
    }
}

bool MujinVisionManager::_GetImages(GetImagesParams params, std::vector<ImagePtr>& resultcolorimages, std::vector<ImagePtr>& resultdepthimages, std::vector<ImagePtr>& resultresultimages, unsigned long long& imageStartTimestamp, unsigned long long& imageEndTimestamp)
{
    ThreadType tt = params.threadtype;
    BinPickingTaskResourcePtr pBinpickingTask = params.pBinpickingTask;
    const std::string& regionname = params.regionname;
    const std::vector<std::string>& colorcameranames = params.colorcameranames;
    const std::vector<std::string>& depthcameranames = params.depthcameranames;
    bool ignoreocclusion = params.ignoreocclusion;
    const unsigned long long newerthantimestamp = params.newerthantimestamp;
    const unsigned int fetchimagetimeout = params.fetchimagetimeout;
    const bool request = params.request;
    const bool useold = params.useold;
    const unsigned int waitinterval = params.waitinterval;
    const unsigned int checkpreemptbits = params.checkpreemptbits;
    if (useold && _lastcolorimages.size() == colorcameranames.size() && _lastdepthimages.size() == depthcameranames.size()) {
        // check if all old images are newer than newerthantimestamp
        bool needToGetNewImages = false;
        for (size_t i=0; !needToGetNewImages && i < _lastdepthimages.size(); ++i) {
            if (_lastdepthimages.at(i)->GetStartTimestamp() <= newerthantimestamp) {
                needToGetNewImages = true;
            }
        }
        for (size_t i=0; !needToGetNewImages && i < _lastcolorimages.size(); ++i) {
            if (_lastcolorimages.at(i)->GetStartTimestamp() <= newerthantimestamp) {
                needToGetNewImages = true;
            }
        }
        for (size_t i=0; !needToGetNewImages && i < _lastresultimages.size(); ++i) {
            if (_lastresultimages.at(i)->GetStartTimestamp() <= newerthantimestamp) {
                needToGetNewImages = true;
            }
        }
        if (!needToGetNewImages) {
            MUJIN_LOG_INFO("using last images");
            resultcolorimages = _lastcolorimages;
            resultdepthimages = _lastdepthimages;
            resultresultimages = _lastresultimages;
            // returning old images, have to set imageStartTimestamp and imageEndTimestamp correctly
            std::vector<ImagePtr> allimages; allimages.reserve(resultcolorimages.size() + resultdepthimages.size() + resultresultimages.size());
            allimages.insert(allimages.end(), resultcolorimages.begin(), resultcolorimages.end());
            allimages.insert(allimages.end(), resultdepthimages.begin(), resultdepthimages.end());
            allimages.insert(allimages.end(), resultresultimages.begin(), resultresultimages.end());
            if( allimages.size() > 0 ) {
                imageStartTimestamp = 0;
                imageEndTimestamp = 0;
                for(size_t iimage = 0; iimage < allimages.size(); ++iimage) {
                    if( allimages[iimage]->GetStartTimestamp() != 0 && allimages[iimage]->GetEndTimestamp() != 0 ) {
                        if( imageStartTimestamp == 0 || imageEndTimestamp == 0 ) {
                            imageStartTimestamp = allimages[iimage]->GetStartTimestamp();
                            imageEndTimestamp = allimages[iimage]->GetEndTimestamp();
                        }
                        else {
                            if( imageStartTimestamp > allimages[iimage]->GetStartTimestamp() ) {
                                imageStartTimestamp = allimages[iimage]->GetStartTimestamp();
                            }
                            if( imageEndTimestamp < allimages[iimage]->GetEndTimestamp() ) {
                                imageEndTimestamp = allimages[iimage]->GetEndTimestamp();
                            }
                        }
                    }
                }

                return true;
            }
            else {
                return false;
            }
        }
        else {
            MUJIN_LOG_INFO(str(boost::format("do not use last images because images are older than newerthantimestamp=%d")%newerthantimestamp));
        }
    }
    std::vector<std::string> cameranames;
    for (size_t i=0; i<colorcameranames.size(); ++i) {
        cameranames.push_back(colorcameranames.at(i));
    }
    for (size_t i=0; i<depthcameranames.size(); ++i) {
        std::string cameraname = depthcameranames.at(i);
        if (std::find(cameranames.begin(), cameranames.end(), cameraname) == cameranames.end()) {
            cameranames.push_back(cameraname);
        }
    }
    // use local images so that we don't accidentally return images that are not verified
    std::vector<ImagePtr> colorimages, depthimages, resultimages;

    uint64_t start0 = GetMilliTime();
    uint64_t starttime0;
    imageStartTimestamp = 0; imageEndTimestamp = 0;
    unsigned long long imagepacktimestamp = 0, oldimagepacktimestamp = 0;
    std::string cameraname;
    //uint64_t lastimageagecheckfailurets = 0;
    uint64_t lastimagetscheckfailurets = 0;
    uint64_t lastfirstimagecheckfailurewarnts = 0;
    uint64_t lastocclusioncheckfailurewarnts = 0;
    uint64_t lastocclusionwarnts = 0;
    uint64_t lastresultimagecheckfailurets = 0;
    //uint64_t lastcouldnotcapturewarnts = 0;
//    bool usecache = !((request || !GetJsonValueByKey<bool>(_visionserverconfig, "runpublisher", true)) && (colorcameranames.size() == 1 && depthcameranames.size() == 1));
    bool bGotAllImages = false; // true if received all the images

    while (!_bCancelCommand && // command is not being canceled
           !_bShutdown &&  // visionmanager is not being shutdown
           ((fetchimagetimeout == 0) || (fetchimagetimeout > 0 && GetMilliTime() - start0 < fetchimagetimeout)) && // not timed out yet
           ((tt != TT_Detector) || (tt == TT_Detector && !_bStopDetectionThread)) // detection thread is not being stopped if called from it
           ) {

        // get images from subscriber
        if (!request) {
            std::map<std::string, std::string> mCameraIdRegionName;
            {
                boost::mutex::scoped_lock lock(_mutexRegion);
                FOREACH(v, _mCameraNameHardwareId) {
                    std::map<std::string, std::string>::const_iterator cit = _mCameranameActiveRegionname.find(v->first);
                    if (cit != _mCameranameActiveRegionname.end()) {
                        mCameraIdRegionName[v->second] = cit->second;
                    } else {
                        MUJIN_LOG_VERBOSE("failed to find regionname for camera " << v->first);
                    }
                }
            }
            _pImagesubscriberManager->GetImagePackFromBuffer(colorcameranames, depthcameranames, colorimages, depthimages, resultimages, imageStartTimestamp, imageEndTimestamp, imagepacktimestamp, fetchimagetimeout / 1000.0, oldimagepacktimestamp, mCameraIdRegionName, checkpreemptbits);
        } else { // detection thread always request
            colorimages.resize(0);
            depthimages.resize(0);
            resultimages.resize(0);
            // assuming that the depth camera will return color image as well (with color camera name)
            std::string extracaptureoptions;
            {
                boost::mutex::scoped_lock lock(_mutexRegion);
                extracaptureoptions = _GetExtraCaptureOptions(_GetHardwareIds(cameranames), _GetHardwareIds(cameranames), _visionserverconfig, _controllerIp, _binpickingTaskZmqPort, _slaverequestid, _mCameraNameHardwareId, _mCameranameActiveRegionname, _subscriberid, ignoreocclusion);
            }
            _pImagesubscriberManager->SnapImages(cameranames, imageStartTimestamp, imageEndTimestamp, colorimages, depthimages, resultimages, fetchimagetimeout / 1000.0, /*numimages=*/ -1, extracaptureoptions, checkpreemptbits);
        }
        // if called by detection thread, break if it is being stopped
        if (tt == TT_Detector && _bStopDetectionThread) {
            break;
        }

        // ensure streamer and try to get images again if got fewer than expected images
        if (resultimages.size() == 0 && (colorimages.size() < colorcameranames.size() || depthimages.size() < depthcameranames.size())) {
            MUJIN_LOG_WARN("Could not get all images, will try again" << ": # color " << colorimages.size() << "," << colorcameranames.size() << ", # depth " << depthimages.size() << "," << depthcameranames.size() << ", # result images = " << resultimages.size() << ", request= " << request);
            //lastcouldnotcapturewarnts = GetMilliTime();

            colorimages.resize(0);
            depthimages.resize(0);
            break;
        }
        if (resultimages.size() > 0) { // if the streamer provides the result, call detector only if the images are updated, i.e. new result
            if (imageStartTimestamp == _resultImageStartTimestamp) {
                if (GetMilliTime() - lastresultimagecheckfailurets > 1000.0) {
                    lastresultimagecheckfailurets = GetMilliTime();
                    MUJIN_LOG_DEBUG(str(boost::format("skip image pack as it was used already, imageStartTimestamp=%d imageEndTimestamp=%d")%imageStartTimestamp%imageEndTimestamp));
                }
                colorimages.resize(0);
                depthimages.resize(0);
                resultimages.resize(0);
                oldimagepacktimestamp = imagepacktimestamp; // need to update oldimagepacktimestamp so that we don't process the same image pack
                continue;
            }
        }

        // throw exception if acquired images are from the future
        if (GetMilliTime()  < imageStartTimestamp || GetMilliTime() < imageEndTimestamp) {
            std::stringstream msg_ss;
            msg_ss << "Image is acquired from the future, please ensure that clocks are synchronized, imageStartTimestamp=" << imageStartTimestamp << " imageEndTimestamp=" << imageEndTimestamp << ", request= " << request;
            MUJIN_LOG_ERROR(msg_ss.str());
            colorimages.resize(0);
            depthimages.resize(0);
            resultimages.resize(0);
            throw MujinVisionException(msg_ss.str(), MVE_ImageAcquisitionError);
        }

        // try to get images again if images are older than the specified timestamp
        if (newerthantimestamp > 0 && imageStartTimestamp <= newerthantimestamp) {
            oldimagepacktimestamp = imagepacktimestamp;
            if (GetMilliTime() - lastimagetscheckfailurets > 1000.0) {
                MUJIN_LOG_INFO("image is older than newerthantimestamp=" << newerthantimestamp << " imageStartTimestamp=" << imageStartTimestamp);
                lastimagetscheckfailurets = GetMilliTime();
            }
            colorimages.resize(0);
            depthimages.resize(0);
            resultimages.resize(0);
            continue;
        }

        // skip images and try to get images again if images are taken before detection loop started
        if (!request && _tsStartDetection > 0 && imageStartTimestamp < _tsStartDetection) {
            if (lastfirstimagecheckfailurewarnts != imageStartTimestamp) {
                lastfirstimagecheckfailurewarnts = imageStartTimestamp;
                std::stringstream msg_ss;
                msg_ss << "Image was taken " << (_tsStartDetection - imageStartTimestamp) << " ms before _tsStartDetection " << _tsStartDetection << ", will try to get again" << ", request= " << request;
                MUJIN_LOG_WARN(msg_ss.str());
            }
            colorimages.resize(0);
            depthimages.resize(0);
            resultimages.resize(0);
            continue;
        }

        // check if all images have starttimestamps within delta time
        unsigned long long maxstarttimedelta = 1000; // ms
        if (imageEndTimestamp < imageStartTimestamp && imageEndTimestamp - imageStartTimestamp < maxstarttimedelta) {
            MUJIN_LOG_WARN("imageEndTimestamp=" << imageEndTimestamp << " imageStartTimestamp=" << imageStartTimestamp << " of imagepack " << imagepacktimestamp << " is too big >" << maxstarttimedelta << ", will try to get again");
            colorimages.resize(0);
            depthimages.resize(0);
            resultimages.resize(0);
            continue;
        }

        // skip images and try to get them again if failed to check for occlusion
        bool isoccluding = false;
        if (!ignoreocclusion && regionname.size() > 0) {
            try {
                std::vector<std::string> checkedcameranames;
                rapidjson::Document tmpjson;
                for (size_t i=0; i<colorimages.size() && !isoccluding; ++i) {
                    ImagePtr image = colorimages.at(i);
                    std::string cameraname = _mHardwareIdCameraName[image->GetCameraId()];
                    if (std::find(checkedcameranames.begin(), checkedcameranames.end(), cameraname) == checkedcameranames.end()) {
                        int isoccluded = -1;
                        if (image->GetMetadata().size() > 0) {
                            ParseJson(tmpjson, image->GetMetadata());
                            isoccluded = GetJsonValueByKey<int>(tmpjson, "isoccluded", -1);
                        }
                        if (isoccluded == -1) {
                            //MUJIN_LOG_ERROR(image->GetMetadata());
                            starttime0 = GetMilliTime();
                            pBinpickingTask->IsRobotOccludingBody(regionname, cameraname, image->GetStartTimestamp(), image->GetEndTimestamp(), isoccluding);
                            MUJIN_LOG_DEBUG("IsRobotOccludingBody for " << cameraname  << " (" << _GetHardwareId(cameraname) << ") took " << (GetMilliTime() - starttime0)/1000.0f << " secs");
                            checkedcameranames.push_back(cameraname);
                        } else {
                            isoccluding = isoccluded == 1;
                            //MUJIN_LOG_ERROR("occlusioncheck was done in streamer");
                        }
                    } else {
                        //MUJIN_LOG_ERROR("done occlusioncheck for " << cameraname);
                    }
                }
                for (size_t i=0; i<depthimages.size() && !isoccluding; ++i) {
                    ImagePtr image = depthimages.at(i);
                    std::string cameraname = _mHardwareIdCameraName[image->GetCameraId()];
                    if (std::find(checkedcameranames.begin(), checkedcameranames.end(), cameraname) == checkedcameranames.end()) {
                        int isoccluded = -1;
                        if (image->GetMetadata().size() > 0) {
                            ParseJson(tmpjson, image->GetMetadata());
                            isoccluded = GetJsonValueByKey<int>(tmpjson, "isoccluded", -1);
                        }
                        if (isoccluded == -1) {
                            pBinpickingTask->IsRobotOccludingBody(regionname, cameraname, image->GetStartTimestamp(), image->GetEndTimestamp(), isoccluding);
                            checkedcameranames.push_back(cameraname);
                        } else {
                            isoccluding = isoccluded == 1;
                            //MUJIN_LOG_ERROR("occlusioncheck was done in streamer");
                        }
                    } else {
                        //MUJIN_LOG_ERROR("done occlusioncheck for " << cameraname);
                    }
                }
            } catch(const std::exception& ex) {
                if (GetMilliTime() - lastocclusioncheckfailurewarnts > 1000.0) {
                    lastocclusioncheckfailurewarnts = GetMilliTime();
                    std::stringstream ss;
                    ss << "Failed to check for occluded, will try again (" << ex.what() << ")";
                    MUJIN_LOG_WARN(ss.str());
                    _SetStatusMessage(tt, "", ss.str());
                }
                boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                colorimages.resize(0);
                depthimages.resize(0);
                resultimages.resize(0);
                continue;
            }
        } else {
            //MUJIN_LOG_WARN("skip occlusion check ignoreocclusion=" << ignoreocclusion << " regionname=" << regionname);
        }

        // skip images if there was occlusion
        if (isoccluding) {
            if (GetMilliTime() - lastocclusionwarnts > 1000.0) {
                lastocclusionwarnts = GetMilliTime();
                std::stringstream msg_ss;
                msg_ss << "Region is occluded in the view of camera, will try again"
                       << " imageStartTimestamp " << imageStartTimestamp
                       << " imageEndTimestamp " << imageEndTimestamp
                       << ", request= " << request;
                MUJIN_LOG_WARN(msg_ss.str());
            }
            colorimages.resize(0);
            depthimages.resize(0);
            resultimages.resize(0);
            continue;
        } else {
            std::stringstream ss;
            ss << "got good imagepack. imageStartTimestamp=" << imageStartTimestamp << " imageEndTimestamp=" << imageEndTimestamp << " total=" << (imageEndTimestamp-imageStartTimestamp)/1000.0f << " " << (GetMilliTime()-imageStartTimestamp) / 1000.0f << " secs old, request=" << request << " newerthantimestamp=" << newerthantimestamp;
            MUJIN_LOG_DEBUG(ss.str());
            bGotAllImages = true;
            break;
        }
    }
    if ( _bCancelCommand || // canceled?
         _bShutdown ||  // shutdown?
         (!bGotAllImages && fetchimagetimeout > 0 && GetMilliTime() - start0 > fetchimagetimeout) || // timeed out?
         (tt == TT_Detector && _bStopDetectionThread) // canceled detection loop?
         ) {
        std::stringstream ss;
        ss << "do not use images because we got out of while loop unexpectedly: " << " _bCancelCommand=" << _bCancelCommand << " _bShutdown=" << _bShutdown << " " << GetMilliTime() - start0 << ">" << fetchimagetimeout;
        if (tt == TT_Detector) {
            ss << " _bStopDetectionThread=" << _bStopDetectionThread;
        }
        MUJIN_LOG_DEBUG(ss.str());
    }
    else if( bGotAllImages ) {
        resultcolorimages = colorimages;
        resultdepthimages = depthimages;
        resultresultimages = resultimages;
        _lastcolorimages = colorimages;
        _lastdepthimages = depthimages;
        _lastresultimages = resultimages;
    }
    return bGotAllImages;
}

void MujinVisionManager::Initialize(
    const std::string& visionmanagerconfig,
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
    const std::string& tasktype,
    const double controllertimeout,
    const std::string& locale,
    const std::string& slaverequestid,
    const std::vector<std::string>& targetdetectionarchiveurls
    )
{
    _mNameCommand.clear();
    _locale = locale;
    uint64_t time0 = GetMilliTime();
    uint64_t starttime = GetMilliTime();
    _tsStartDetection = 0;
    _binpickingTaskZmqPort = binpickingTaskZmqPort;
    _binpickingTaskHeartbeatPort = binpickingTaskHeartbeatPort;
    _binpickingTaskHeartbeatTimeout = binpickingTaskHeartbeatTimeout;
    _binpickingTaskScenePk = binpickingTaskScenePk;
    _controllerCommandTimeout = controllertimeout;
    _defaultTaskParameters = defaultTaskParameters;
    _tasktype = tasktype;
    _slaverequestid = slaverequestid;
    _bSendVerificationPointCloud = true;
    _containerParameters = containerParameters;
    _controllerIp = controllerIp;
    std::stringstream url_ss;
    url_ss << "http://" << controllerIp << ":" << controllerPort;
    ControllerClientPtr controller = CreateControllerClient(controllerUsernamePass, url_ss.str());
    _pControllerClient = controller;

    // fetch or update modelfile
    std::string modelfilename;
    if (targeturi != "" && boost::starts_with(targeturi, "mujin:/")) {
        modelfilename = targeturi.substr(sizeof("mujin:/")-1, std::string::npos);

        starttime = GetMilliTime();

        // get local modified time
        std::string modelfilepath = _detectiondir + "/" + modelfilename;
        struct stat filestat;
        long localtimeval = 0;
        if (stat(modelfilepath.c_str(), &filestat) == 0) {
            localtimeval = mktime(localtime(&filestat.st_mtime));
        }

        // do the fetch
        long remotetimeval = 0;
        std::vector<unsigned char> data;
        _pControllerClient->DownloadFileFromControllerIfModifiedSince_UTF8(targeturi, localtimeval, remotetimeval, data);
        if (remotetimeval > 0) {
            // write file
            {
                std::ofstream outfile(modelfilepath.c_str(), std::ios::out | std::ios::binary);
                outfile.write(reinterpret_cast<char *>(&data[0]), data.size());
                outfile.close();
            }

            // set file modification time
            {
                struct utimbuf newtimes;
                newtimes.actime = filestat.st_atime;
                newtimes.modtime = remotetimeval;
                utime(modelfilepath.c_str(), &newtimes);
            }
        }

        MUJIN_LOG_DEBUG("fetching model " << targeturi << " took " << ((GetMilliTime() - starttime)/1000.0f) << " secs, downloaded " << data.size() << " bytes");
    }

    // update target archive if needed
    std::string detectionpath = _detectiondir + "/" + targetname;
    std::string detectorconfigfilename = detectionpath + "/detector.json";
    std::string targetdetectionarchiveurl;
    for (size_t iurl=0; iurl<targetdetectionarchiveurls.size(); ++iurl) {
        targetdetectionarchiveurl = targetdetectionarchiveurls[iurl];
        if (targetdetectionarchiveurl != "" && boost::starts_with(targetdetectionarchiveurl, "mujin:/")) {
            starttime = GetMilliTime();

            // get local modified time
            std::string archivefilename = detectionpath + "/" + targetname + ".tar.gz";
            struct stat filestat;
            long localtimeval = 0;
            if (stat(archivefilename.c_str(), &filestat) == 0) {
                localtimeval = mktime(localtime(&filestat.st_mtime));
            }

            long remotetimeval = 0;
            std::vector<unsigned char> data;

            try {
                _pControllerClient->DownloadFileFromControllerIfModifiedSince_UTF8(targetdetectionarchiveurl, localtimeval, remotetimeval, data);
            }
            catch(const std::exception& ex) {
                MUJIN_LOG_DEBUG(str(boost::format("failed to download file %s, so giving up %s")%targetdetectionarchiveurl%ex.what()));
            }

            if (remotetimeval > 0) {
                // write file
                {
                    boost::filesystem::create_directory(detectionpath);
                    std::ofstream outfile(archivefilename.c_str(), std::ios::out | std::ios::binary);
                    outfile.write(reinterpret_cast<char *>(&data[0]), data.size());
                    outfile.close();
                }

                // set file modification time
                {
                    struct utimbuf newtimes;
                    newtimes.actime = filestat.st_atime;
                    newtimes.modtime = remotetimeval;
                    utime(archivefilename.c_str(), &newtimes);
                }
            }

            MUJIN_LOG_DEBUG("fetching archive " << targetdetectionarchiveurl << " took " << ((GetMilliTime() - starttime)/1000.0f) << " secs, downloaded " << data.size() << " bytes");

            if (remotetimeval > 0) {
                // extract files if downloaded
                boost::this_thread::sleep(boost::posix_time::milliseconds(100)); // sometimes got incomplete tar file and the following would fail, possibly because of disk delays? for now sleeping a little helped resolve the problem.
                starttime = GetMilliTime();
                try {
                    std::stringstream commandss;
                    commandss << "tar xzf " << archivefilename << " -C " << detectionpath;
                    system(commandss.str().c_str()); // TODO: check process exit code here
                } catch (...) {
                    std::stringstream errss;
                    errss << "Failed to prepare config files because " << archivefilename << " could not be decompressed.";
                    MUJIN_LOG_ERROR(errss.str());
                    throw MujinVisionException(errss.str(), MVE_Failed);
                }
                MUJIN_LOG_DEBUG("extracting archive " << archivefilename << " took " << ((GetMilliTime() - starttime)/1000.0f) << " secs");
            }

            // when target archive uri is supplied, the detector.json should always exists
            if (!boost::filesystem::exists(detectorconfigfilename)) {
                throw MujinVisionException("Failed to sync target detection archive. Detector conf " + detectorconfigfilename + " does not exist!", MVE_InvalidArgument);
            }
        }
    }
    // prepare config files
    _mDetectorExtraInitializationOptions.clear();
    if (boost::filesystem::exists(detectorconfigfilename)) {
        MUJIN_LOG_INFO("using detectionpath " << detectionpath << " as path to detectorconfig, ignoring detectorconfigname");
        _mDetectorExtraInitializationOptions["templateDir"] = detectionpath;
    } else {
        MUJIN_LOG_INFO("could not find detector conf at " << detectorconfigfilename << ", will attempt default config location");
        detectorconfigfilename = "";
    }

    // load visionserver configuration

    ParseJson(_visionserverconfig, visionmanagerconfig);

    _filteringsubsample = GetJsonValueByKey<double>(_visionserverconfig, "filteringsubsample", 1.0);
    if (_filteringsubsample == 1) {
        MUJIN_LOG_WARN("filteringsubsample=" << _filteringsubsample << ". Set it to a higher value for speedup.");
    }
    _filteringvoxelsize = GetJsonValueByKey<double>(_visionserverconfig, "filteringvoxelsize", 0.01 * 1000);
    _filteringstddev = GetJsonValueByKey<double>(_visionserverconfig, "filteringstddev", 0.01);
    if (_filteringvoxelsize < 1) {
        MUJIN_LOG_WARN("it seems that filteringvoxelsize=" << _filteringvoxelsize << " are in meters, converting them to mm, please update conf");
        _filteringvoxelsize = _filteringvoxelsize * 1000;
    }
    _filteringnumnn = GetJsonValueByKey<int>(_visionserverconfig, "filteringnumnn", 80);
    std::string bindetectionMode = "never";
    if (_visionserverconfig.HasMember("bindetectionMode")) {
        bindetectionMode = GetJsonValueByKey<std::string>(_visionserverconfig, "bindetectionMode", "once");
    } else if (_visionserverconfig.HasMember("bindetection")) {
        MUJIN_LOG_WARN("bindetection is deprecated, please use bindetectionMode instead");
        bindetectionMode = GetJsonValueByKey<std::string>(_visionserverconfig, "bindetection", "once");
    }
    if (bindetectionMode == "once" || bindetectionMode == "1") {
        _bindetectionMode = "once";
    } else if (bindetectionMode == "always" || bindetectionMode == "2") {
        _bindetectionMode = "always";
    } else if (bindetectionMode == "never" || bindetectionMode == "0") {
        _bindetectionMode = "never";
    } else {
        MUJIN_LOG_WARN(str(boost::format("bindetectionMode=\"%s\" is not support, use default value \"never\"") % bindetectionMode));
        _bindetectionMode = "never";
    }

    std::string detectorconfigstr;
    if (detectorconfigfilename.size() == 0) {
        detectorconfigfilename = _GetConfigFileName("detector", detectorconfigname);
        MUJIN_LOG_INFO("using default detector conf at " << detectorconfigfilename);
    }
    _LoadConfig(detectorconfigfilename, detectorconfigstr);

    // append additional params to detectorconf string
    bool debug = GetJsonValueByKey<bool>(_visionserverconfig, "debug", false);
    rapidjson::Document detectorconfigjson;
    ParseJson(detectorconfigjson, detectorconfigstr);
    SetJsonValueByKey(detectorconfigjson, "debug", debug);
    if (_visionserverconfig.HasMember("cleanParameters")) {
        SetJsonValueByKey(detectorconfigjson, "cleanParameters", _visionserverconfig["cleanParameters"]);
        SetJsonValueByKey(detectorconfigjson, "visionManagerConfiguration", _visionserverconfig);
    }
    SetJsonValueByKey(detectorconfigjson, "modelFilename", modelfilename);
    _detectorconfig = DumpJson(detectorconfigjson);
    

    std::string detectormodulename = GetJsonValueByPath<std::string>(detectorconfigjson, "/detection/modulename", "");
    std::string detectorclassname = GetJsonValueByPath<std::string>(detectorconfigjson, "/detection/classname", "");
    MUJIN_LOG_DEBUG(str(boost::format("from detector.json detectormodulename=%s detectorclassname=%s")%detectormodulename%detectorclassname));
    if (detectormodulename.size() == 0) {
        detectormodulename = GetJsonValueByKey<std::string>(_visionserverconfig, "modulename", "");
    }
    if (detectorclassname.size() == 0) {
        detectorclassname = GetJsonValueByKey<std::string>(_visionserverconfig, "classname", "");
    }
    MUJIN_LOG_DEBUG(str(boost::format("final detectormodulename=%s detectorclassname=%s")%detectormodulename%detectorclassname));
    if (detectormodulename.size() > 0 && detectorclassname.size() > 0) {
        _mDetectorExtraInitializationOptions["modulename"] = detectormodulename;
        _mDetectorExtraInitializationOptions["classname"] = detectorclassname;
    }

    // load unit info
    _mDetectorExtraInitializationOptions["inputUnit"] = GetJsonValueByKey<std::string>(_visionserverconfig, "inputunit", "mm");
    _mDetectorExtraInitializationOptions["outputUnit"] = GetJsonValueByKey<std::string>(_visionserverconfig, "outputunit", "mm");

    // set up regions
    std::vector<std::string> regionnames;
    std::vector<RegionParametersPtr > vRegionParameters;
    {
        boost::mutex::scoped_lock lock(_mutexRegion);
        _mNameRegion.clear();
        RegionParametersPtr pRegionParameters;
        rapidjson::Document regionparametersjson;
        ParseJson(regionparametersjson, containerParameters);
        vRegionParameters = GetJsonValueByKey<std::vector<RegionParametersPtr> >(regionparametersjson, "regions");
        FOREACH(v, vRegionParameters) {
            _mNameRegion[(*v)->instobjectname] = RegionPtr(new Region(*v));
            regionnames.push_back((*v)->instobjectname);
        }
    }

    // connect to mujin controller
    _userinfo_json = _GetUserInfoJsonString();

    _SetStatusMessage(TT_Command, "Connected to mujin controller at " + url_ss.str());
    SceneResourcePtr scene(new SceneResource(controller,binpickingTaskScenePk));
    _pSceneResource = scene;
    _pBinpickingTask = scene->GetOrCreateBinPickingTaskFromName_UTF8(tasktype+std::string("task1"), tasktype, TRO_EnableZMQ);

    _SetStatusMessage(TT_Command, "Syncing cameras");
    std::vector<std::string> cameranames;
    std::map<std::string, std::string> mAllCameraNameHardwareId; // mapping of all the cameras, even ones that are not used
    _mCameraNameHardwareId.clear();
    scene->GetSensorMapping(mAllCameraNameHardwareId);

    // need to initialize cameras that are only in the regions _mHardwareIdCameraName! Otherwise can get unused cameras and they can conflict with the ID -> Camera mapping
    _mCameraNameHardwareId.clear();
    FOREACH(it, mAllCameraNameHardwareId) {
        bool bInRegion = false;
        FOREACH(itregion, vRegionParameters) {
            if( find((*itregion)->cameranames.begin(), (*itregion)->cameranames.end(), it->first) != (*itregion)->cameranames.end() ) {
                MUJIN_LOG_DEBUG(str(boost::format("adding camera %s since matching to region %s")%it->first%(*itregion)->instobjectname));
                bInRegion = true;
                break;
            }
        }
        if( bInRegion ) {
            _mCameraNameHardwareId[it->first] = it->second;
        }
        else {
            MUJIN_LOG_DEBUG(str(boost::format("could not find region for camera %s since matching to region %s")%it->first%it->second));
        }
    }

    _mNameCameraParameters.clear();
    FOREACH(v, _mCameraNameHardwareId) {
        MUJIN_LOG_DEBUG("got camera hardware id " << v->first << " with id " << v->second);
        _mHardwareIdCameraName[v->second] = v->first;
        if (_mNameCameraParameters.find(v->first) != _mNameCameraParameters.end()) {
            _mNameCameraParameters[v->first]->id = v->second;
        } else {
            _mNameCameraParameters[v->first].reset(new CameraParameters(v->second));
        }
        _mNameCameraParameters[v->first]->isDepthCamera = v->first.find("_l_rectified") != std::string::npos || v->first.find("Projector") != std::string::npos  || (v->first.find("projector") != std::string::npos && v->first.find("RVprojector") == std::string::npos && v->first.find("TVSprojector") == std::string::npos) || v->first.find("RVcamera") != std::string::npos || v->first.find("TVScamera0") != std::string::npos || v->first.find("TVCS-F30A-0") != std::string::npos;  // FIXME: hack
        _mNameCameraParameters[v->first]->isColorCamera = v->first.find("projector") == std::string::npos && v->first.find("RVprojector") == std::string::npos && v->first.find("TVSprojector") == std::string::npos;  // FIXME: hack
        if (_mNameCameraParameters[v->first]->isDepthCamera) {
            MUJIN_LOG_DEBUG("camera " << v->first << " is a depth camera");
        }
        cameranames.push_back(v->first);
    }

    MUJIN_LOG_DEBUG("initialzing binpickingtask in Initialize() with userinfo " + _userinfo_json);

    _pBinpickingTask->Initialize(defaultTaskParameters, binpickingTaskZmqPort, binpickingTaskHeartbeatPort, _zmqcontext, false, 0/*do not start monitor thread*/, _controllerCommandTimeout, _userinfo_json, slaverequestid);

    _SetStatusMessage(TT_Command, "Syncing regions");
    BinPickingTaskResource::ResultGetInstObjectAndSensorInfo resultgetinstobjectandsensorinfo;
    starttime = GetMilliTime();
    _pBinpickingTask->GetInstObjectAndSensorInfo(regionnames, cameranames, resultgetinstobjectandsensorinfo, "mm", controllertimeout);
    MUJIN_LOG_DEBUG("GetInstObjectAndSensorInfo() took: " + boost::lexical_cast<std::string>((GetMilliTime() - starttime)/1000.0f) + " secs");

    // sync regions
    starttime = GetMilliTime();
    mujinvision::Transform regiontransform;
    for (size_t i=0; i<regionnames.size(); ++i) {
        regiontransform = _GetTransform(resultgetinstobjectandsensorinfo.minstobjecttransform[regionnames[i]]);
        _SyncRegion(regionnames[i], regiontransform, resultgetinstobjectandsensorinfo.minstobjectobb[regionnames[i]], resultgetinstobjectandsensorinfo.minstobjectinnerobb[regionnames[i]]);
    }
    MUJIN_LOG_DEBUG("sync regions took: " + boost::lexical_cast<std::string>((GetMilliTime() - starttime)/1000.0f) + " secs");

    // set up cameras
    starttime = GetMilliTime();
    _SetStatusMessage(TT_Command, "Setting up cameras.");
    _mNameCamera.clear();
    MUJIN_LOG_INFO("reset capture handles");
    {
        boost::mutex::scoped_lock lock(_mutexCaptureHandles);
        _mCameranameCaptureHandles.clear();
    }
    FOREACH(it, _mNameCameraParameters) {
        std::string cameraname = it->first;
        CameraParametersPtr pcameraparameters = it->second;
        RobotResource::AttachedSensorResource::SensorData sensordata = resultgetinstobjectandsensorinfo.msensordata[cameraname];
        /*
           // do not use webapi to get sensordata for speed reason
           RobotResource::AttachedSensorResource::SensorData sensordata;
           std::string camerabodyname,sensorname;
           _ParseCameraName(cameraname, camerabodyname, sensorname);

           utils::GetSensorData(_pControllerClient, _pSceneResource, camerabodyname, sensorname, sensordata);
         */
        CalibrationDataPtr calibrationdata(new CalibrationData());
        calibrationdata->fx           = sensordata.intrinsic[0];
        calibrationdata->fy           = sensordata.intrinsic[4];
        calibrationdata->pu           = sensordata.intrinsic[2];
        calibrationdata->pv           = sensordata.intrinsic[5];
        calibrationdata->s            = sensordata.intrinsic[1];
        calibrationdata->focal_length = sensordata.focal_length;
        calibrationdata->distortion_model = sensordata.distortion_model;
        for (size_t idceff = 0; idceff < 5; idceff++) {
            calibrationdata->distortion_coeffs[idceff] = sensordata.distortion_coeffs[idceff];
        }
        if (sensordata.extra_parameters.size()==4 && sensordata.extra_parameters[0]==1) { // TODO: reorganize
            calibrationdata->kappa = sensordata.extra_parameters[1];
        } else {
            calibrationdata->kappa = 0;
        }
        calibrationdata->image_width  = sensordata.image_dimensions[0];
        calibrationdata->image_height = sensordata.image_dimensions[1];

        if (sensordata.extra_parameters.size()==4 && sensordata.extra_parameters[0]==1) { // TODO: reorganize
            calibrationdata->pixel_width = sensordata.extra_parameters[2];
            calibrationdata->pixel_height = sensordata.extra_parameters[3];
        } else {
            calibrationdata->pixel_width  = sensordata.focal_length / sensordata.intrinsic[0];
            calibrationdata->pixel_height = sensordata.focal_length / sensordata.intrinsic[4];
        }
        _mNameCamera[cameraname] = CameraPtr(new Camera(cameraname, pcameraparameters, calibrationdata));
        _SyncCamera(cameraname, resultgetinstobjectandsensorinfo.msensortransform[cameraname]);
    }
    std::map<std::string, std::map<std::string, CameraPtr> > mRegionnameCameramap;
    {
        boost::mutex::scoped_lock lock(_mutexRegion);
        _mCameranameActiveRegionname.clear(); // will be set up later in SendPointCloudObstacleToController or StartDetectionLoop
        FOREACH(itr, _mNameRegion) {
            std::string regionname = itr->first;
            MUJIN_LOG_DEBUG("checking cameras for region " << regionname);
            RegionPtr region = _mNameRegion[regionname];
            std::string cameraname;
            std::map<std::string, CameraPtr> mCameranameCamera;
            for (unsigned int i=0; i<region->pRegionParameters->cameranames.size(); ++i) {
                cameraname = region->pRegionParameters->cameranames.at(i);
                if( _mNameCamera.find(cameraname) == _mNameCamera.end() ) {
                    std::stringstream ssvalidcameranames;
                    FOREACH(itc, _mNameCamera) {
                        ssvalidcameranames << itc->first << ", ";
                    }
                    throw MujinVisionException(str(boost::format("scene sensor mapping does not have camera %s coming from region %s. valid camera names are [%s], mAllCameraNameHardwareId=%d, vRegionParameters=%d")%cameraname%regionname%ssvalidcameranames.str()%mAllCameraNameHardwareId.size()%vRegionParameters.size()), MVE_InvalidArgument);
                }
                MUJIN_LOG_DEBUG("adding camera " << cameraname  << " (" << _GetHardwareId(cameraname) << ") to region " << regionname);
                mCameranameCamera[cameraname] = _mNameCamera[cameraname];
                _mCameranameActiveRegionname[cameraname] = regionname;
            }
            mRegionnameCameramap[regionname] = mCameranameCamera;
        }
    }
    MUJIN_LOG_DEBUG("sync cameras took: " + boost::lexical_cast<std::string>((GetMilliTime() - starttime)/1000.0f) + " secs");

    // set up subscribers
    _SetStatusMessage(TT_Command, "Loading subscriber configuration.");
    // load subscriber configuration
    _imagesubscriberconfig = imagesubscriberconfig;
    rapidjson::Document imagesubscriberjson;
    ParseJson(imagesubscriberjson, imagesubscriberconfig);

    // set up image subscriber manager
    _subscriberid = GetJsonValueByKey<std::string>(imagesubscriberjson, "subscriberid", "");
    _SetStatusMessage(TT_Command, str(boost::format("Setting up image manager %s.")%_subscriberid));
    _pImagesubscriberManager->Initialize(_mNameCamera, streamerIp, streamerPort, imagesubscriberjson, _zmqcontext);

    // set up detectors
    starttime = GetMilliTime();
    _SetStatusMessage(TT_Command, "Setting up detector.");
    _targetname = targetname;
    _targeturi = targeturi;
    _targetupdatename = targetupdatename;
    {
        boost::mutex::scoped_lock lock(_mutexRegion);
        _pDetector = _pDetectorManager->CreateObjectDetector(_detectorconfig, _targetname, _mNameRegion, mRegionnameCameramap, boost::bind(&MujinVisionManager::_SetDetectorStatusMessage, this, _1, _2), _mDetectorExtraInitializationOptions, boost::bind(&MujinVisionManager::_CheckPreemptDetector, this, _1));
    }
    MUJIN_LOG_DEBUG("detector initialization took: " + boost::lexical_cast<std::string>((GetMilliTime() - starttime)/1000.0f) + " secs");
    MUJIN_LOG_DEBUG("Initialize() took: " + boost::lexical_cast<std::string>((GetMilliTime() - time0)/1000.0f) + " secs");

    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::_DeInitialize()
{
    _StopSendPointCloudObstacleToControllerThread();
    _StopDetectionThread();
    _StopUpdateEnvironmentThread();
    _StopExecutionVerificationPointCloudThread();
    _StopControllerMonitorThread();
    _StopVisualizePointCloudThread();
    std::string regionname;
    {
        boost::mutex::scoped_lock lock(_mutexDetector);
        if (!!_pDetector) {
            _pDetector.reset();
            MUJIN_LOG_DEBUG("reset detector");
        }
    }
    if (!!_pImagesubscriberManager) {
        _pImagesubscriberManager->DeInitialize(); // do not reset because it is created and passed in from outside
        MUJIN_LOG_DEBUG("reset imagesubscribermanager");
    }
    _SetStatusMessage(TT_Command, "DeInitialized vision manager.");
}

void MujinVisionManager::DetectObjects(const std::string& regionname, const std::vector<std::string>&cameranames, std::vector<DetectedObjectPtr>& detectedobjects, std::string& resultstate, unsigned long long& imageStartTimestamp, unsigned long long& imageEndTimestamp, int& isContainerPresent, const bool ignoreocclusion, const unsigned long long newerthantimestamp, const unsigned int fetchimagetimeout, const bool fastdetection, const bool bindetection, const bool request, const bool useold, const std::string& cycleindex)
{
    if (!_pImagesubscriberManager) {
        throw MujinVisionException("image subscriber manager is not initialzied", MVE_Failed);
    }
    std::vector<CameraCaptureHandlePtr> capturehandles; CREATE_SAFE_DELETER_CAMERAHANDLES(capturehandles);
    MUJIN_LOG_DEBUG("_StartAndGetCaptureHandle with cameranames " << __GetString(cameranames));
    _StartAndGetCaptureHandle(cameranames, cameranames, capturehandles, /*force=*/ false, ignoreocclusion);
    bool checkcontaineremptyonly = 0;
    int numresults = _DetectObjects(TT_Command, _pBinpickingTask, regionname, cameranames, detectedobjects, resultstate, imageStartTimestamp, imageEndTimestamp, isContainerPresent, ignoreocclusion, newerthantimestamp, fetchimagetimeout, fastdetection, bindetection, request, useold, checkcontaineremptyonly, cycleindex);
}

int MujinVisionManager::_DetectObjects(ThreadType tt, BinPickingTaskResourcePtr pBinpickingTask, const std::string& regionname, const std::vector<std::string>&cameranames, std::vector<DetectedObjectPtr>& detectedobjects, std::string& resultstate, unsigned long long& imageStartTimestamp, unsigned long long& imageEndTimestamp, int& isContainerPresent, const bool ignoreocclusion, const unsigned long long newerthantimestamp, const unsigned int fetchimagetimeout, const bool fastdetection, const bool bindetection, const bool request, const bool useold, const bool checkcontaineremptyonly, const std::string& cycleindex)
{
    boost::mutex::scoped_lock lock(_mutexDetector);
    uint64_t starttime = GetMilliTime();

    std::vector<std::string> colorcameranames = _GetColorCameraNames(regionname, cameranames);
    std::vector<std::string> depthcameranames = _GetDepthCameraNames(regionname, cameranames);

    // set up images
    std::vector<ImagePtr> colorimages, depthimages, resultimages;
    unsigned int waitinterval = 50;
    GetImagesParams params;
    params.threadtype = tt;
    params.pBinpickingTask = pBinpickingTask;
    params.regionname = regionname;
    params.colorcameranames = colorcameranames;
    params.depthcameranames = depthcameranames;
    params.ignoreocclusion = ignoreocclusion;
    params.newerthantimestamp = newerthantimestamp;
    params.fetchimagetimeout = fetchimagetimeout;
    params.request = request;
    params.useold = useold;
    params.waitinterval = waitinterval;
    params.checkpreemptbits = PC_DetectionThread;
    bool bGotAllImages = _GetImages(params, colorimages, depthimages, resultimages, imageStartTimestamp, imageEndTimestamp);

    MUJIN_LOG_INFO("Getting images took " << ((GetMilliTime() - starttime) / 1000.0f) << " for " << __GetString(colorcameranames) << " " << __GetString(depthcameranames) << " newerthantimestamp=" << newerthantimestamp);
    starttime = GetMilliTime();
    if (bGotAllImages && (resultimages.size() > 0 || (colorimages.size() == colorcameranames.size() && depthimages.size() == depthcameranames.size()))) {
        for (size_t i=0; i<colorimages.size(); ++i) {
            _pDetector->SetColorImage(colorimages.at(i));
        }
        for (size_t i=0; i<depthimages.size(); ++i) {
            _pDetector->SetDepthImage(depthimages.at(i));
        }
        // detect objects
        _lastDetectStartTimestamp = imageStartTimestamp;
        unsigned int checkpreemptbits = 1;
        if (resultimages.size() > 0) {
            _pDetector->DetectObjects(regionname, colorcameranames, depthcameranames, resultimages, detectedobjects, resultstate, fastdetection, bindetection, checkcontaineremptyonly, checkpreemptbits, cycleindex);
        } else {
            _pDetector->DetectObjects(regionname, colorcameranames, depthcameranames, detectedobjects, resultstate, fastdetection, bindetection, checkcontaineremptyonly, checkpreemptbits, cycleindex);
        }
    }
    else {
        if (!_bStopDetectionThread) {
            MUJIN_LOG_ERROR("Not enough images, cannot detect! colorimages=" << colorimages.size() << " depthimages=" << depthimages.size() << " resultimages=" << resultimages.size());
        }
        // do not ensure capturehandles here, do it at the caller because the handles here would be removed too quickly before all desired images are captured
        resultstate = "null";
        return -1;
    }
    int numresults = 0;
    if (resultstate == "") {
        resultstate = "null";
    } else {
        rapidjson::Document d;
        ParseJson(d, resultstate);
        if (!d.HasMember("numDetectedParts")) {
            MUJIN_LOG_WARN("numDetectedObjects is not in resultstate");
        }
        numresults = GetJsonValueByKey<int>(d, "numDetectedParts", detectedobjects.size());
        if (!d.HasMember("isContainerPresent")) {
            MUJIN_LOG_WARN("isContainerPresent is not in resultstate");
        }
        isContainerPresent = GetJsonValueByKey<int>(d, "isContainerPresent", -1);
    }
    //std::stringstream msgss;
    //msgss << "Detected " << detectedobjects.size() << " objects, state: " << resultstate <<". Took " << (GetMilliTime()-starttime)/1000.0f << " seconds.";
    //_SetStatusMessage(tt, msgss.str());
    _SetStatus(tt, MS_Succeeded);
    return numresults;
}

void MujinVisionManager::StartDetectionLoop(const std::string& regionname, const std::vector<std::string>& cameranames, const std::vector<std::string>& evcamnames, const Transform& worldresultoffsettransform, const bool ignoreocclusion, const unsigned int fetchimagetimeout, const std::string& obstaclename, const unsigned long long detectionstarttimestamp, const std::string& locale, const unsigned int maxnumfastdetection, const unsigned int maxnumdetection, const bool sendVerificationPointCloud, const bool stopOnLeftInOrder, const std::string& targetupdatename, const unsigned int numthreads, const std::string& cycleindex)
{
    if (!_pImagesubscriberManager) {
        throw MujinVisionException("image subscriber manager is not initialzied", MVE_Failed);
    }
    // reset region info when necessary
    _CheckAndUpdateRegionCameraMapping(regionname, cameranames);
    {
        boost::mutex::scoped_lock lock(_mutexDetectedInfo);
        // clear cached detected objects
        _bDetectedObjectsValid = false;
        _vDetectedObject.resize(0);
        _resultState = "{}";
    }
    _bSendVerificationPointCloud = sendVerificationPointCloud;
    _tWorldResultOffset = worldresultoffsettransform;
    _StartDetectionThread(regionname, cameranames, ignoreocclusion, fetchimagetimeout, detectionstarttimestamp, maxnumfastdetection, maxnumdetection, stopOnLeftInOrder, targetupdatename, numthreads, cycleindex);
    _StartUpdateEnvironmentThread(regionname, cameranames, obstaclename, 50, locale);
    if( _bSendVerificationPointCloud ) {
        _StartExecutionVerificationPointCloudThread(evcamnames, evcamnames, ignoreocclusion, obstaclename, 50, locale);
    }
    _StartControllerMonitorThread(50, locale);
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::StopDetectionLoop()
{
    _StopDetectionThread();
    _StopUpdateEnvironmentThread();
    _StopExecutionVerificationPointCloudThread();
    _StopControllerMonitorThread();
    //_StopSendPointCloudObstacleToControllerThread(); // do not stop SendPointCloudObstacleToControllerThread because it is not started by detection loop
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::StartVisualizePointCloudThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double pointsize, const bool ignoreocclusion, const unsigned long long newerthantimestamp, const unsigned int fetchimagetimeout, const bool request)
{
    if (!_pImagesubscriberManager) {
        throw MujinVisionException("image subscriber manager is not initialzied", MVE_Failed);
    }
    _StartVisualizePointCloudThread(regionname, cameranames, pointsize, ignoreocclusion, newerthantimestamp, fetchimagetimeout, request);
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::StopVisualizePointCloudThread()
{
    _StopVisualizePointCloudThread();
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::SendPointCloudObstacleToController(const std::string& regionname, const std::vector<std::string>&cameranames, const std::vector<DetectedObjectPtr>& detectedobjectsworld, const unsigned long long newerthantimestamp, const unsigned int fetchimagetimeout, const std::string& obstaclename, const bool fast, const bool request, const bool async, const std::string& locale)
{
    _SendPointCloudObstacleToController(regionname, cameranames, detectedobjectsworld, newerthantimestamp, fetchimagetimeout, obstaclename, fast, request, async, locale);
}

void MujinVisionManager::_SendPointCloudObstacleToController(const std::string& regionname, const std::vector<std::string>&cameranames, const std::vector<DetectedObjectPtr>& detectedobjectsworld, const unsigned long long newerthantimestamp, const unsigned int fetchimagetimeout, const std::string& obstaclename, const bool fast, const bool request, const bool async, const std::string& locale)
{
    if (!_pImagesubscriberManager) {
        throw MujinVisionException("image subscriber manager is not initialzied", MVE_Failed);
    }
    try {
        // reset region info when necessary
        _CheckAndUpdateRegionCameraMapping(regionname, cameranames);
        uint64_t starttime = GetMilliTime();
        double pointsize = 0;
        size_t filteringsubsample = 0;
        double filteringstddev = 0;
        int filteringnumnn = 0;

        {
            boost::mutex::scoped_lock lock(_mutexRegion);

            RegionParametersPtr pregion = _mNameRegion[regionname]->pRegionParameters;
            if( !!pregion ) {
                MUJIN_LOG_INFO("pointsize=0, using pointsize= " << pointsize << " in regionparam");
                pointsize = pregion->pointsize;

                filteringsubsample = pregion->filteringsubsample;
                filteringstddev = pregion->filteringstddev;
                filteringnumnn = pregion->filteringnumnn;
            }
        }
        if( filteringnumnn == 0 ) {
            filteringnumnn = _filteringnumnn;
        }
        if( filteringsubsample == 0 ) {
            filteringsubsample = _filteringsubsample;
        }
        if( filteringstddev == 0 ) {
            filteringstddev = _filteringstddev;
        }

        std::vector<ImagePtr> dummyimages;
        std::vector<std::string> dummycameranames;
        if (!async) {
            std::vector<CameraCaptureHandlePtr> capturehandles; CREATE_SAFE_DELETER_CAMERAHANDLES(capturehandles);
            std::vector<std::string> depthcameranames = _GetDepthCameraNames(regionname, cameranames);
            MUJIN_LOG_DEBUG("_StartAndGetCaptureHandle with cameranames " << __GetString(depthcameranames));
            _StartAndGetCaptureHandle(depthcameranames, depthcameranames, capturehandles); // always force it for SendPointCloudObstacleToController
            // set up images
            std::vector<ImagePtr> depthimages;
            bool ignoreocclusion = true;
            unsigned long long imageStartTimestamp = 0, imageEndTimestamp = 0;
            GetImagesParams params;
            params.threadtype = TT_Command;
            params.pBinpickingTask = _pBinpickingTask;
            params.regionname = regionname;
            params.colorcameranames = dummycameranames;
            params.depthcameranames = depthcameranames;
            params.ignoreocclusion = ignoreocclusion;
            params.newerthantimestamp = newerthantimestamp;
            params.fetchimagetimeout = fetchimagetimeout;
            params.request = request;
            params.useold = false;
            params.waitinterval = 50;
            params.checkpreemptbits = PC_SendPointcloudThread;
            bool bGotAllImages = _GetImages(params, dummyimages, depthimages, dummyimages, imageStartTimestamp, imageEndTimestamp);
            if (bGotAllImages && depthimages.size() == depthcameranames.size()) {
                std::vector<Real> totalpoints;
                int isregionoccluded = -2;
                int isoccluded = -1;
                unsigned long long cloudstarttime, cloudendtime;
                double timeout = 3.0; // secs
                for (size_t i=0; i<depthimages.size(); ++i) {
                    std::string cameraname = depthcameranames.at(i);
                    CameraPtr camera= _mNameCamera[cameraname];
                    // get point cloud obstacle
                    std::vector<Real> points;
                    ImagePtr depthimage = depthimages.at(i);

                    isoccluded = _pImagesubscriberManager->GetCollisionPointCloud(cameraname, points, cloudstarttime, cloudendtime, pointsize, filteringstddev, filteringnumnn, regionname, timeout, newerthantimestamp, filteringsubsample, PC_SendPointcloudThread);
                    if (points.size() / 3 == 0) {
                        _SetStatusMessage(TT_Command, "got 0 point from GetPointCloudObstacle()");
                        int numretries = 3;
                        std::vector<std::string> depthcameranames1;
                        std::vector<ImagePtr> depthimages1;
                        depthcameranames1.push_back(cameraname);
                        while (numretries > 0 && points.size() / 3 == 0) {
                            points.clear();
                            _SetStatusMessage(TT_Command, "re-try getting depthimage and pointcloudobstacle");
                            GetImagesParams params;
                            params.threadtype = TT_Command;
                            params.pBinpickingTask = _pBinpickingTask;
                            params.regionname = regionname;
                            params.colorcameranames = dummycameranames;
                            params.depthcameranames = depthcameranames1;
                            params.ignoreocclusion = ignoreocclusion;
                            params.newerthantimestamp = newerthantimestamp;
                            params.fetchimagetimeout = fetchimagetimeout;
                            params.request = request;
                            params.useold = false;
                            params.waitinterval = 50;
                            params.checkpreemptbits = PC_SendPointcloudThread;
                            bool bGotAllImages2 = _GetImages(params, dummyimages, depthimages1, dummyimages, imageStartTimestamp, imageEndTimestamp);
                            if( bGotAllImages2 ) {
                                isoccluded = _pImagesubscriberManager->GetCollisionPointCloud(cameraname, points, cloudstarttime, cloudendtime, pointsize, filteringstddev, filteringnumnn, regionname, timeout, newerthantimestamp, filteringsubsample, PC_SendPointcloudThread);
                            }
                            numretries--;
                        }
                        if (points.size() / 3 == 0) {
                            throw MujinVisionException("got 0 point from GetPointCloudObstacle() after retries", MVE_Failed);
                        }
                    }
                    std::stringstream ss;

                    // check only if we are not sure region is occluded
                    if (isregionoccluded < 1 && !!depthimage && depthimage->GetMetadata().size() > 0) {
                        rapidjson::Document tmpjson;
                        ParseJson(tmpjson, depthimage->GetMetadata());
                        int iscameraoccluded = GetJsonValueByKey<int>(tmpjson, "isoccluded", -1);
                        if (isregionoccluded == -2 || isregionoccluded == 0) { // use camera occlusion status if region occlusion status is never set or not occluded
                            isregionoccluded = iscameraoccluded;
                        } else if (isregionoccluded == -1) { // if region occlusion status is unknown
                            if (iscameraoccluded == 0 || iscameraoccluded == -1) { // stay unknown if we don't know the camera is occluded
                                isregionoccluded = -1;
                            } else if (iscameraoccluded == 1) { // set region to be occluded if the camera is occluded
                                isregionoccluded = 1;
                            }
                        } else if (isregionoccluded == 1) { // should not get here
                            MUJIN_LOG_WARN("should not get here!");
                        }
                    }
                    MUJIN_LOG_DEBUG("adding " << (points.size()/3) << " points from " << cameraname  << " (" << _GetHardwareId(cameraname) << ")");
                    totalpoints.insert(totalpoints.end(), points.begin(), points.end());
                }
                std::stringstream ss;
                ss <<"Sending over " << (totalpoints.size()/3) << " points";
                _SetStatusMessage(TT_Command, ss.str());
                _pBinpickingTask->AddPointCloudObstacle(totalpoints, pointsize, obstaclename, imageStartTimestamp, imageEndTimestamp, false, "mm", isregionoccluded, regionname);
                _lastSendPointCloudObstacleTimestamp = imageStartTimestamp;
            } else {
                MUJIN_LOG_WARN("failed to get image for getting pointcloud obstacle");
            }
        } else {
            _bStopSendPointCloudObstacleToControllerThread = false;
            _bIsSendPointcloudRunning = true;
            SendPointCloudObstacleToControllerThreadParams params;
            params.regionname = regionname;
            params.cameranames = cameranames;
            params.detectedobjectsworld = detectedobjectsworld;
            params.newerthantimestamp = newerthantimestamp;
            params.fetchimagetimeout = fetchimagetimeout;
            params.pointsize = pointsize;
            params.obstaclename = obstaclename;
            _pSendPointCloudObstacleThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_SendPointCloudObstacleToControllerThread, this, params)));
        }
        std::stringstream ss;
        ss << "SendPointCloudObstacleToController async " << int(async) << " took " << (GetMilliTime() - starttime) / 1000.0f << " secs";
        MUJIN_LOG_INFO(ss.str());
        _SetStatus(TT_Command, MS_Succeeded);
    }
    catch(const mujinclient::UserInterruptException& ex) {
        MUJIN_LOG_DEBUG(str(boost::format("User interrupted from %s")%ex.what()));
    }
    catch (const MujinVisionException& e) {
        MUJIN_LOG_ERROR(e.message());
        _SetStatusMessage(TT_Command, e.message(), e.GetCodeString());
    }
    catch(const std::exception& ex) {
        std::stringstream ss;
        ss << "Caught exception: " << ex.what();
        //std::string errstr = ParametersBase::GetExceptionJsonString(GetErrorCodeString(MVE_RecognitionError), ss.str());
        MUJIN_LOG_ERROR(ss.str());
        _SetStatusMessage(TT_Command, ss.str(), GetErrorCodeString(MVE_Failed));
    }

}

void MujinVisionManager::_SendPointCloudObstacleToControllerThread(SendPointCloudObstacleToControllerThreadParams params)
{
    FalseSetter turnoffstatusvar(_bIsSendPointcloudRunning);
    std::string regionname = params.regionname;
    std::vector<std::string> cameranames = params.cameranames;
    std::vector<DetectedObjectPtr> detectedobjectsworld = params.detectedobjectsworld;
    unsigned long long newerthantimestamp = params.newerthantimestamp;
    unsigned int fetchimagetimeout = params.fetchimagetimeout;
    double pointsize = params.pointsize;
    std::string obstaclename = params.obstaclename;
    std::vector<CameraCaptureHandlePtr> capturehandles; CREATE_SAFE_DELETER_CAMERAHANDLES(capturehandles);

    if (!_pImagesubscriberManager) {
        throw MujinVisionException("image subscriber manager is not initialzied", MVE_Failed);
    }

    try {
        BinPickingTaskResourcePtr pBinpickingTask = _pSceneResource->GetOrCreateBinPickingTaskFromName_UTF8(_tasktype+std::string("task1"), _tasktype, TRO_EnableZMQ);
        std::string userinfo = _GetUserInfoJsonString();
        MUJIN_LOG_DEBUG("initialzing binpickingtask in _SendPointCloudObstacleToControllerThread with userinfo " + userinfo);

        pBinpickingTask->Initialize(_defaultTaskParameters, _binpickingTaskZmqPort, _binpickingTaskHeartbeatPort, _zmqcontext, false, 0/*do not start monitor thread*/, _controllerCommandTimeout, userinfo, _slaverequestid);

        std::vector<std::string> depthcameranames = _GetDepthCameraNames(regionname, cameranames);
        // set up images
        std::vector<ImagePtr> depthimages;
        bool ignoreocclusion = true;
        std::vector<ImagePtr> dummyimages;
        std::vector<std::string> dummycameranames;
        unsigned long long imageStartTimestamp = 0, imageEndTimestamp = 0;
        //uint64_t lastwarnedtimestamp = 0;
        std::vector<Real> points;
        size_t filteringsubsample = 0;
        double filteringstddev = 0;
        int filteringnumnn = 0;

        {
            boost::mutex::scoped_lock lock(_mutexRegion);

            RegionParametersPtr pregion = _mNameRegion[regionname]->pRegionParameters;
            if( !!pregion ) {
                MUJIN_LOG_INFO("pointsize=0, using pointsize= " << pointsize << " in regionparam");
                pointsize = pregion->pointsize;

                filteringsubsample = pregion->filteringsubsample;
                filteringstddev = pregion->filteringstddev;
                filteringnumnn = pregion->filteringnumnn;
            }
        }
        if( filteringnumnn == 0 ) {
            filteringnumnn = _filteringnumnn;
        }
        if( filteringsubsample == 0 ) {
            filteringsubsample = _filteringsubsample;
        }
        if( filteringstddev == 0 ) {
            filteringstddev = _filteringstddev;
        }

        while (!_bStopSendPointCloudObstacleToControllerThread) {
            MUJIN_LOG_DEBUG("_StartAndGetCaptureHandle with cameranames " << __GetString(depthcameranames));
            _StartAndGetCaptureHandle(depthcameranames, depthcameranames, capturehandles);  // always force for SendPointCloudObstacleToController
            GetImagesParams params;
            params.threadtype = TT_SendPointcloudObstacle;
            params.pBinpickingTask = pBinpickingTask;
            params.regionname = regionname;
            params.colorcameranames = dummycameranames;
            params.depthcameranames = depthcameranames;
            params.ignoreocclusion = ignoreocclusion;
            params.newerthantimestamp = newerthantimestamp;
            params.fetchimagetimeout = fetchimagetimeout;
            params.request = true;
            params.useold = false;
            params.waitinterval = 50;
            params.checkpreemptbits = PC_SendPointcloudThread;
            bool bGotAllImages = _GetImages(params, dummyimages, depthimages, dummyimages, imageStartTimestamp, imageEndTimestamp); // TODO update GetCollisionPointCloud() to remove this
            MUJIN_LOG_DEBUG(str(boost::format("got %d images in SendPointCloudObstacleToControllerThread imageStartTimestamp=%u newerthantimestamp=%u")%depthimages.size()%imageStartTimestamp%newerthantimestamp));
            if (bGotAllImages && depthimages.size() == depthcameranames.size()) {
                std::vector<Real> totalpoints;
                int isregionoccluded = -2;
                for (size_t i=0; i<depthimages.size(); ++i) {
                    std::string cameraname = depthcameranames.at(i);
                    CameraPtr camera= _mNameCamera[cameraname];
                    // get point cloud obstacle
                    ImagePtr depthimage = depthimages.at(i);

                    unsigned long long cloudstarttime, cloudendtime;
                    double timeout = 3.0; // secs

                    points.resize(0);
                    int isoccluded = _pImagesubscriberManager->GetCollisionPointCloud(cameraname, points, cloudstarttime, cloudendtime, pointsize, filteringstddev, filteringnumnn, regionname, timeout, 0, filteringsubsample, PC_SendPointcloudThread);

                    if (points.size() / 3 == 0) {
                        _SetStatusMessage(TT_SendPointcloudObstacle, "got 0 point from GetPointCloudObstacle()");
                        int numretries = 3;
                        std::vector<std::string> depthcameranames1;
                        std::vector<ImagePtr> depthimages1;
                        depthcameranames1.push_back(cameraname);
                        while (numretries > 0 && points.size() / 3 == 0) {
                            MUJIN_LOG_DEBUG("force ensure capturing _StartAndGetCaptureHandle with cameranames " << __GetString(depthcameranames));
                            _StartAndGetCaptureHandle(depthcameranames1, depthcameranames1, capturehandles, true);  // always force for SendPointCloudObstacleToController
                            points.clear();
                            _SetStatusMessage(TT_SendPointcloudObstacle, "re-try getting depthimage and pointcloudobstacle");
                            GetImagesParams params;
                            params.threadtype = TT_SendPointcloudObstacle;
                            params.pBinpickingTask = pBinpickingTask;
                            params.regionname = regionname;
                            params.colorcameranames = dummycameranames;
                            params.depthcameranames = depthcameranames1;
                            params.ignoreocclusion = ignoreocclusion;
                            params.newerthantimestamp = newerthantimestamp;
                            params.fetchimagetimeout = fetchimagetimeout;
                            params.request = true;
                            params.useold = false;
                            params.waitinterval = 50;
                            params.checkpreemptbits = PC_SendPointcloudThread;
                            bool bGotAllImages2 = _GetImages(params, dummyimages, depthimages1, dummyimages, imageStartTimestamp, imageEndTimestamp); // TODO update GetCollisionPointCloud() to remove this
                            if( bGotAllImages2 ) {
                                isoccluded = _pImagesubscriberManager->GetCollisionPointCloud(cameraname, points, cloudstarttime, cloudendtime, pointsize, filteringstddev, filteringnumnn, regionname, timeout, 0, filteringsubsample, PC_SendPointcloudThread);
                            }
                            numretries--;
                        }
                        if (points.size() / 3 == 0) {
                            throw MujinVisionException("got 0 point from GetPointCloudObstacle() after retries", MVE_Failed);
                        }
                    }

                    // check only if we are not sure region is occluded
                    if (isregionoccluded < 1 && !!depthimage && depthimage->GetMetadata().size() > 0) {
                        rapidjson::Document tmpjson;
                        ParseJson(tmpjson, depthimage->GetMetadata());
                        int iscameraoccluded = GetJsonValueByKey<int>(tmpjson, "isoccluded", -1);
                        if (isregionoccluded == -2 || isregionoccluded == 0) { // use camera occlusion status if region occlusion status is never set or not occluded
                            isregionoccluded = iscameraoccluded;
                        } else if (isregionoccluded == -1) { // if region occlusion status is unknown
                            if (iscameraoccluded == 0 || iscameraoccluded == -1) { // stay unknown if we don't know the camera is occluded
                                isregionoccluded = -1;
                            } else if (iscameraoccluded == 1) { // set region to be occluded if the camera is occluded
                                isregionoccluded = 1;
                            }
                        } else if (isregionoccluded == 1) { // should not get here
                            MUJIN_LOG_WARN("should not get here!");
                        }
                    }
                    MUJIN_LOG_DEBUG("adding " << (points.size()/3) << " points from " << cameraname << " (" << _GetHardwareId(cameraname) << ")");
                    totalpoints.insert(totalpoints.end(), points.begin(), points.end());
                }
                std::stringstream ss;
                ss <<"Sending over " << (totalpoints.size()/3) << " points";
                _SetStatusMessage(TT_SendPointcloudObstacle, ss.str());
                pBinpickingTask->AddPointCloudObstacle(totalpoints, pointsize, obstaclename, imageStartTimestamp, imageEndTimestamp, false, "mm", isregionoccluded, regionname);
                _lastSendPointCloudObstacleTimestamp = imageStartTimestamp;
                _SetStatus(TT_SendPointcloudObstacle, MS_Succeeded);
                break;
            } else {
                MUJIN_LOG_WARN("failed to get images in SendPointCloudObstacleToControllerThread, depthimages.size()=" << depthimages.size() << " depthcameranames.size()=" << depthcameranames.size());
                MUJIN_LOG_DEBUG("force ensure capturing _StartAndGetCaptureHandle with cameranames " << __GetString(depthcameranames) << " current handles are " << capturehandles.size());
                _StartAndGetCaptureHandle(depthcameranames, depthcameranames, capturehandles, true);  // always force for SendPointCloudObstacleToController
                boost::this_thread::sleep(boost::posix_time::milliseconds(500)); // sleep a little to stop flooding the messages
            }
        }
    }
    catch(const mujinclient::UserInterruptException& ex) {
        MUJIN_LOG_DEBUG(str(boost::format("User interrupted %s")%ex.what()));
    }
    catch (const MujinVisionException& e) {
        MUJIN_LOG_ERROR(e.message());
        _SetStatusMessage(TT_SendPointcloudObstacle, e.message(), e.GetCodeString());
    }
    catch (const std::exception& ex) {
        std::stringstream ss;
        ss << "Caught exception: " << ex.what();
        //std::string errstr = ParametersBase::GetExceptionJsonString(GetErrorCodeString(MVE_RecognitionError), ss.str());
        MUJIN_LOG_ERROR(ss.str());
        _SetStatusMessage(TT_SendPointcloudObstacle, ss.str(), GetErrorCodeString(MVE_Failed));
    }
    MUJIN_LOG_DEBUG("end of SendPointCloudObstacleToControllerThread");
}

void MujinVisionManager::VisualizePointCloudOnController(const std::string& regionname, const std::vector<std::string>& cameranames, const double pointsize, const bool ignoreocclusion, const unsigned long long newerthantimestamp, const unsigned int fetchimagetimeout, const bool request)
{
    _VisualizePointCloudOnController(regionname, cameranames, pointsize, ignoreocclusion, newerthantimestamp, fetchimagetimeout, request);
}

void MujinVisionManager::_VisualizePointCloudOnController(const std::string& regionname, const std::vector<std::string>&cameranames, const double pointsize, const bool ignoreocclusion, const unsigned long long newerthantimestamp, const unsigned int fetchimagetimeout, const bool request)
{
    std::vector<std::string> cameranamestobeused;
    if (regionname != "") {
        cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);
    } else if (cameranames.size() > 0) {
        cameranamestobeused = cameranames;
    } else {
        throw MujinVisionException("neither region name nor camera names is specified, cannot visualize pointcloud", MVE_InvalidArgument);
    }
    if (cameranamestobeused.size() == 0) {
        MUJIN_LOG_INFO("no camera to be used for visualization, do nothing");
        _SetStatus(TT_Command, MS_Succeeded);
        return;
    }

    std::vector<std::string> dummycameranames;

    std::vector<std::vector<Real> > pointslist;
    std::vector<std::string> names;
    std::vector<double> points;
    std::vector<ImagePtr> dummyimages;
    unsigned long long cloudstarttime, cloudendtime;
    for (unsigned int i=0; i<cameranamestobeused.size(); i++) {
        points.resize(0);
        std::string cameraname = cameranamestobeused.at(i);
        CameraPtr camera = _mNameCamera[cameraname];
        if (!camera->pCameraParameters->isDepthCamera) {
            continue;
        }
        std::vector<ImagePtr> depthimages;
        std::vector<std::string> dcamnames;
        dcamnames.push_back(cameraname);
        MUJIN_LOG_DEBUG("Visualize for depth camera " << cameraname << " (" << _GetHardwareId(cameraname) << ")");
        unsigned long long imageStartTimestamp = 0, imageEndTimestamp = 0;
        GetImagesParams params;
        params.threadtype = TT_Command;
        params.pBinpickingTask = _pBinpickingTask;
        params.regionname = "";
        params.colorcameranames = dummycameranames;
        params.depthcameranames = dcamnames;
        params.ignoreocclusion = ignoreocclusion;
        params.newerthantimestamp = newerthantimestamp;
        params.fetchimagetimeout = fetchimagetimeout;
        params.request = request;
        params.useold = false;
        params.waitinterval = 50;
        params.checkpreemptbits = PC_VisualizePointCloudThread;
        bool bGotAllImages = _GetImages(params, dummyimages, depthimages, dummyimages, imageStartTimestamp, imageEndTimestamp);
        if (!bGotAllImages || depthimages.size() == 0) {
            throw MujinVisionException("failed to get depth image for " + cameraname + ", cannot visualize point cloud", MVE_Failed);
        }
        _pImagesubscriberManager->GetCollisionPointCloud(cameraname, points, cloudstarttime, cloudendtime, _filteringvoxelsize, _filteringstddev, _filteringnumnn, "", 20.0, 0, 1, PC_VisualizePointCloudThread);
        if (points.size()>0) {
            pointslist.push_back(points);
            std::stringstream name_ss;
            name_ss << "__pointcloud_" << i;
            names.push_back(name_ss.str());
        }
    }
    _pBinpickingTask->VisualizePointCloud(pointslist, pointsize, names, "mm");
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::ClearVisualizationOnController()
{
    _pBinpickingTask->ClearVisualization();
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::UpdateDetectedObjects(const std::vector<DetectedObjectPtr>& detectedobjectsworld, const std::string& resultstate, const bool sendtocontroller)
{
    if (detectedobjectsworld.size()==0) {
        _SetStatus(TT_Command, MS_Succeeded);
    }
    if (sendtocontroller) {
        std::vector<mujinclient::Transform> transformsworld;
        std::vector<std::string> confidences;
        for (unsigned int i=0; i<detectedobjectsworld.size(); i++) {
            mujinclient::Transform transform;
            transform.quaternion[0] = detectedobjectsworld[i]->transform.rot[0];
            transform.quaternion[1] = detectedobjectsworld[i]->transform.rot[1];
            transform.quaternion[2] = detectedobjectsworld[i]->transform.rot[2];
            transform.quaternion[3] = detectedobjectsworld[i]->transform.rot[3];
            transform.translate[0] = detectedobjectsworld[i]->transform.trans[0];
            transform.translate[1] = detectedobjectsworld[i]->transform.trans[1];
            transform.translate[2] = detectedobjectsworld[i]->transform.trans[2];
            transformsworld.push_back(transform);
            confidences.push_back(detectedobjectsworld[i]->confidence);
        }
        _pBinpickingTask->UpdateObjects(detectedobjectsworld[0]->name, transformsworld, confidences, resultstate, "mm");
    }
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::SyncRegion(const std::string& regionname)
{
    _SyncRegion(regionname);
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::SyncCameras(const std::string& regionname, const std::vector<std::string>&cameranames)
{
    std::vector<std::string> cameranamestobeused;
    if (regionname != "") {
        cameranamestobeused = _GetCameraNames(regionname, cameranames);
    } else if (cameranames.size() > 0) {
        cameranamestobeused = cameranames;
    } else {
        throw MujinVisionException("neither region name nor camera names is specified, cannot sync cameras", MVE_InvalidArgument);
    }
    for (unsigned int i=0; i<cameranamestobeused.size(); i++) {
        MUJIN_LOG_DEBUG("updating " + boost::lexical_cast<std::string>(cameranamestobeused[i]));
        _SyncCamera(cameranamestobeused[i]);
    }
    // update cameras in detector
    if (!!_pDetector) {
        boost::mutex::scoped_lock lock(_mutexRegion);
        if (_mNameRegion.find(regionname) == _mNameRegion.end()) {
            MUJIN_LOG_WARN("region " + regionname + " is unknown!");
        } else {
            _pDetector->UpdateRegion(regionname, _mNameRegion[regionname], _mNameCamera);
        }
    }
    // update cameras in subscriber
    _pImagesubscriberManager->UpdateCameras(_mNameCamera);
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::GetCameraId(const std::string& cameraname, std::string& cameraid)
{
    if (_mNameCameraParameters.find(cameraname) == _mNameCameraParameters.end()) {
        throw MujinVisionException(cameraname + " is not defined in visionmanager config file.", MVE_ConfigurationFileError);
    }
    cameraid = _mNameCameraParameters[cameraname]->id;
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::GetLatestDetectedObjects(std::vector<DetectedObjectPtr>& detectedobjectsworld, std::string& resultstate, std::vector<Real>& points, unsigned long long& imageStartTimestamp, unsigned long long& imageEndTimestamp, const bool returnpoints)
{
    {
        boost::mutex::scoped_lock lock(_mutexDetectedInfo);
        detectedobjectsworld = _vDetectedObject;
        resultstate = _resultState;
        imageStartTimestamp = _resultImageStartTimestamp;
        imageEndTimestamp = _resultImageEndTimestamp;
        if (returnpoints) {
            points.clear();
            FOREACH(it, _mResultPoints) {
                points.insert(points.end(), it->second.begin(), it->second.end());
            }
        }
    }
    _SetStatus(TT_Command, MS_Succeeded);
}

std::vector<std::string> MujinVisionManager::_GetCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames)
{
    boost::mutex::scoped_lock lock(_mutexRegion);
    if (_mNameRegion.find(regionname) == _mNameRegion.end()) {
        throw MujinVisionException("Region "+regionname+ " is unknown for _GetCameraNames!", MVE_InvalidArgument);
    }
    std::vector<std::string> cameranamestobeused;
    std::vector<std::string> mappedcameranames = _mNameRegion[regionname]->pRegionParameters->cameranames;
    if (cameranames.size()==0) {
        cameranamestobeused = mappedcameranames;
    } else {
        for (unsigned int i=0; i<cameranames.size(); i++) {
            if (std::find(mappedcameranames.begin(), mappedcameranames.end(), cameranames[i]) != mappedcameranames.end()) {
                cameranamestobeused.push_back(cameranames[i]);
            }
        }
    }
    return cameranamestobeused;
}

std::vector<std::string> MujinVisionManager::_GetColorCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames)
{
    std::vector<std::string> cameranamescandidates = _GetCameraNames(regionname, cameranames);
    std::vector<std::string> colorcameranames;
    for(unsigned int i = 0; i < cameranamescandidates.size(); ++i) {
        std::string cameraname = cameranamescandidates.at(i);
        if (_mNameCameraParameters.find(cameraname) == _mNameCameraParameters.end()) {
            throw MujinVisionException(cameraname + " is not in camera parameters map.", MVE_ConfigurationFileError);
        }
        if (std::find(colorcameranames.begin(), colorcameranames.end(), cameraname)==colorcameranames.end() && _mNameCameraParameters[cameraname]->isColorCamera) {
            colorcameranames.push_back(cameraname);
        }
    }
    return colorcameranames;
}

std::vector<std::string> MujinVisionManager::_GetDepthCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames)
{
    std::vector<std::string> cameranamescandidates= _GetCameraNames(regionname, cameranames);
    std::vector<std::string> depthcameranames;
    for(unsigned int i = 0; i < cameranamescandidates.size(); ++i) {
        std::string cameraname = cameranamescandidates.at(i);
        if (_mNameCameraParameters.find(cameraname) == _mNameCameraParameters.end()) {
            throw MujinVisionException(cameraname + " is not in camera parameters map.", MVE_ConfigurationFileError);
        }
        if (std::find(depthcameranames.begin(), depthcameranames.end(), cameraname)==depthcameranames.end() && _mNameCameraParameters[cameraname]->isDepthCamera) {
            depthcameranames.push_back(cameraname);
        }
    }
    return depthcameranames;
}

std::vector<std::string> MujinVisionManager::_GetHardwareIds(const std::vector<std::string>& cameranames)
{
    std::vector<std::string> hardwareids;
    for (size_t i=0; i<cameranames.size(); ++i) {
        std::string cameraname = cameranames.at(i);
        if (_mCameraNameHardwareId.find(cameraname) != _mCameraNameHardwareId.end()) {
            hardwareids.push_back(_mCameraNameHardwareId[cameraname]);
        } else {
            MUJIN_LOG_WARN("cameraname " << cameraname << " is not in _mCameraNameHardwareId");
        }
    }
    return hardwareids;
}

std::string MujinVisionManager::_GetHardwareId(const std::string& cameraname)
{
    if (_mCameraNameHardwareId.find(cameraname) != _mCameraNameHardwareId.end()) {
        return _mCameraNameHardwareId[cameraname];
    } else {
        MUJIN_LOG_WARN("cameraname " << cameraname << " is not in _mCameraNameHardwareId");
        return "";
    }
}

Transform MujinVisionManager::_GetTransform(const mujinclient::Transform& t)
{
    Transform transform;
    for (unsigned int i=0; i<3; i++) {
        transform.trans[i] = t.translate[i];
    }
    for (unsigned int i=0; i<4; i++) {
        transform.rot[i] = t.quaternion[i];
    }
    return transform;
}

void Utils::TransformDetectedObjects(const std::vector<DetectedObjectPtr>& detectedobjectsfrom, std::vector<DetectedObjectPtr>& detectedobjectsto, const Transform& O_T_S, const Transform& O_T_G)
{
    detectedobjectsto.clear();
    if (detectedobjectsfrom.size()==0) {
        return;
    }
    Transform G_T_S = O_T_G.inverse()*O_T_S;
    const std::string name = detectedobjectsfrom.at(0)->name;
    for (size_t i=0; i<detectedobjectsfrom.size(); i++) {
        Transform G_T_A = G_T_S * detectedobjectsfrom.at(i)->transform;
        DetectedObjectPtr detectedobj(new DetectedObject(name, detectedobjectsfrom.at(i)->objecturi, G_T_A, detectedobjectsfrom.at(i)->confidence, detectedobjectsfrom.at(i)->timestamp, detectedobjectsfrom.at(i)->extra));
        detectedobjectsto.push_back(detectedobj);
    }
    BOOST_ASSERT(detectedobjectsfrom.size() == detectedobjectsto.size());
}

std::string MujinVisionManager::_GetString(const Transform& transform)
{
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<Real>::digits10+1);
    TransformMatrix t(transform);
    for (unsigned int r = 0; r < 3; ++r) {
        for (unsigned int c = 0; c < 3; ++c) {
            ss << t.m[r*4+c] << " ";
        }
        ss << t.trans[r] << std::endl;
    }
    return ss.str();
}

void MujinVisionManager::_ParseCameraName(const std::string& cameraname, std::string& camerabodyname, std::string& sensorname)
{
    size_t pos = cameraname.find("/");
    if (pos == std::string::npos) {
        throw MujinVisionException("cameraname (" + cameraname + ") does not have /", MVE_InvalidArgument);
    }
    camerabodyname = cameraname.substr(0,pos);
    sensorname = cameraname.substr(pos+1);
}

void MujinVisionManager::_CheckAndUpdateRegionCameraMapping(const std::string& regionname, const std::vector<std::string>& cameranames)
{
    bool regioninfochanged = false;
    boost::mutex::scoped_lock lock(_mutexRegion);
    if (_mNameRegion.find(regionname) == _mNameRegion.end()) {
        throw MujinVisionException("regionname " + regionname + " is unknown, cannot start detection");
    }
    std::string cameraname;
    for (size_t i = 0; i < cameranames.size(); ++i) {
        cameraname = cameranames[i];
        if (_mNameCamera.find(cameraname) == _mNameCamera.end()) {
            throw MujinVisionException("cameraname " + cameraname + " is unknown, cannot start detection");
        }
        if (_mCameranameActiveRegionname.find(cameraname) == _mCameranameActiveRegionname.end() || _mCameranameActiveRegionname[cameraname] != regionname) {
            if (_mCameranameActiveRegionname.find(cameraname) != _mCameranameActiveRegionname.end()) {
                MUJIN_LOG_DEBUG("cameraname active regionname changed from " << _mCameranameActiveRegionname[cameraname] << " to " << regionname);
            }
            regioninfochanged = true;
            break;
        }
    }

    if (regioninfochanged) {
        MUJIN_LOG_INFO("region info changed, need to reset region/camera mapping");
        RegionPtr region = _mNameRegion[regionname];
        region->pRegionParameters->cameranames = cameranames;
        for (size_t i = 0; i < cameranames.size(); ++i) {
            cameraname = cameranames[i];
            _mCameranameActiveRegionname[cameraname] = regionname;
            if (_mNameCamera.find(cameraname) == _mNameCamera.end()) {
                throw MujinVisionException("cameraname " + cameraname + " is unknown, cannot start detection");
            }
        }
        MUJIN_LOG_INFO("updated region/camera mapping for region " << regionname);


        if (_mNameRegion.find(regionname) == _mNameRegion.end()) {
            throw MujinVisionException("region " + regionname + " is unknown!");
        }
        std::map<std::string, CameraPtr> mNameCamera;
        std::string cameraname;
        CameraPtr camera;
        for (unsigned int i=0; i<region->pRegionParameters->cameranames.size(); ++i) {
            cameraname = region->pRegionParameters->cameranames.at(i);
            camera = _mNameCamera[cameraname];
            mNameCamera[cameraname] = camera;
        }
        _pDetector->UpdateRegion(regionname, region, mNameCamera);

        // do nothing for now, assuming binpickingui re-initializes visionmanager when mapping changes
        // TODO: there is a race condition to be debugged
        //MUJIN_LOG_WARN("region info changed, need to force start capture for all sensors again to reload new region camera mappings");
        // std::vector<std::string> ids = _GetHardwareIds(cameranames);
        // std::string extracaptureoptions;
        // extracaptureoptions = _GetExtraCaptureOptions(ids, ids, _visionserverpt, _controllerIp, _binpickingTaskZmqPort, _slaverequestid, _mCameraNameHardwareId, _mCameranameActiveRegionname);
        // _pImagesubscriberManager->StartCaptureThread(ids, 2.0, 1, extracaptureoptions);
    }
}

} // namespace mujinvision
