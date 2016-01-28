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
#include "mujinvision/mujinvisionmanager.h"
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
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

namespace mujinvision {

MujinVisionManager::ImagesubscriberHandler::ImagesubscriberHandler(ImageSubscriberManagerPtr pImagesubscriberManager, const std::vector<std::string>& ids)
{
    _ts = GetMilliTime();
    VISIONMANAGER_LOG_DEBUG("in ImagesubscriberHandler constructor " + boost::lexical_cast<std::string>(_ts));
    _pManager = pImagesubscriberManager;
    _vIds = ids;
    _pManager->StartCaptureThread(ids);
}

MujinVisionManager::ImagesubscriberHandler::~ImagesubscriberHandler() {
    VISIONMANAGER_LOG_DEBUG("in ImagesubscriberHandler destructor " +  boost::lexical_cast<std::string>(_ts));
    _pManager->StopCaptureThread(_vIds);
}

void ParametersBase::Print()
{
    VISIONMANAGER_LOG_INFO(GetJsonString());
}

bool MujinVisionManager::_PreemptSubscriber()
{
    bool bpreempt = _bShutdown || _bCancelCommand || _bStopDetectionThread || _bStopUpdateEnvironmentThread || _bStopExecutionVerificationPointCloudThread;
    if (bpreempt ) {
        std::stringstream ss;
        ss << "preempt subscriber! _bShutdown=" << int(_bShutdown) << " _bCancelCommand=" << int(_bCancelCommand) << " _bStopDetectionThread=" << _bStopDetectionThread;
        VISIONMANAGER_LOG_DEBUG(ss.str())
    }
    return bpreempt;
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
    _bIsVisualizePointcloudRunning = false;
    _bIsSendPointcloudRunning = false;
    _bIsEnvironmentUpdateRunning = false;
    _bIsControllerPickPlaceRunning = false;
    _bIsRobotOccludingSourceContainer = false;
    _bForceRequestDetectionResults = false;
    _bIsGrabbingTarget = false;
    _bIsGrabbingLastTarget = false;
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
    _resultState = "{}";
    _pImagesubscriberManager = imagesubscribermanager;
    _pImagesubscriberManager->SetPreemptFn(boost::bind(&MujinVisionManager::_PreemptSubscriber, this));
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
    _controllerCommandTimeout = 10.0;
    _locale = "en_US";
    _detectorconfig = "";
    _imagesubscriberconfig = "";
    _tasktype = "";
    _targetname = "";
    _targeturi = "";
    _targetupdatename = "detected_";
    _resultState = "";
    _userinfo_json = "";
    _slaverequestid = "";
    _defaultTaskParameters = "";
    _filteringvoxelsize = 0.001;
    _filteringstddev = 0.01;
    _filteringnumnn = 1;
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
    _StartCommandThread(commandport);
    _StartCommandThread(configport);
}

MujinVisionManager::~MujinVisionManager()
{
    Destroy();
}

void MujinVisionManager::Destroy()
{
    VISIONMANAGER_LOG_DEBUG("Destroying MujinVisionManager");
    Shutdown();
}

void MujinVisionManager::Shutdown()
{
    _bShutdown=true;
    _StopStatusThread();
    _StopDetectionThread();
    _StopUpdateEnvironmentThread();
    _StopExecutionVerificationPointCloudThread();
    _StopControllerMonitorThread();
    _StopCommandThread(_commandport);
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
        VISIONMANAGER_LOG_DEBUG(ss.str());
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
        ParametersBase::ValidateJsonString(config);
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
        throw UserInterruptException("Cancelling command.");
    }
    std::stringstream ss;
    ss << GetMilliTime() << " " << _GetManagerStatusString(status) << ": " << msg;
    if (msg == "") {
        VISIONMANAGER_LOG_DEBUG(ss.str());
    } else {
        VISIONMANAGER_LOG_INFO(ss.str());
    }
    boost::mutex::scoped_lock lock(_mutexStatusQueue);
    std::string cmdmsg = "";
    std::string cmderr = "";
    std::string cfgmsg = "";
    std::string cfgerr = "";
    std::string detectormsg = "";
    std::string detectorerr = "";
    std::string updateenvmsg = "";
    std::string updateenverr = "";
    std::string controllermonmsg = "";
    std::string controllermonerr = "";
    std::string sendpclmsg = "";
    std::string sendpclerr = "";
    std::string visualizepcmsg = "";
    std::string visualizepcerr = "";
    std::string sendexecvpcmsg = "";
    std::string sendexecvpcerr = "";
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
    VISIONMANAGER_LOG_DEBUG(ss.str());

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

void MujinVisionManager::_StartStatusPublisher(const unsigned int port)
{
    _pStatusPublisher.reset(new StatusPublisher(_zmqcontext, port));
    if (!_pStatusPublisher) {
        throw MujinVisionException("Failed to start status publisher!", MVE_Failed);
    }
    _SetStatus(TT_Command, MS_Pending);
}

void MujinVisionManager::_StartStatusThread(const unsigned int port, const unsigned int ms)
{
    _bStopStatusThread = false;
    _pStatusThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_StatusThread, this, port, ms)));
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
        VISIONMANAGER_LOG_DEBUG("Stopped status thread");
    }
}

void MujinVisionManager::_StartCommandThread(const unsigned int port)
{
    _mPortStopCommandThread[port] = false;
    _mPortCommandThread[port].reset(new boost::thread(boost::bind(&MujinVisionManager::_CommandThread, this, port)));
}

void MujinVisionManager::_StopCommandThread(const unsigned int port)
{
    if (_mPortStopCommandThread[port] == false) {
        std::stringstream ss;
        ss << "stopping command thread (port: " << port << ").";
        VISIONMANAGER_LOG_DEBUG(ss.str());
        _mPortStopCommandThread[port] = true;
        _mPortCommandThread[port]->join();
        _mPortCommandThread[port].reset();
    }
    std::stringstream ss;
    ss << "Stopped command thread (port: " << port << ").";
    VISIONMANAGER_LOG_DEBUG(ss.str());
}

void MujinVisionManager::_StartCommandServer(const unsigned int port)
{
    {
        boost::mutex::scoped_lock lock(_mutexCommandServerMap);
        _mPortCommandServer[port].reset(new CommandServer(_zmqcontext, port));
    }
    if (!_mPortCommandServer[port]) {
        std::stringstream ss;
        ss << "Failed to start command server at port " << port << "!";
        throw MujinVisionException(ss.str(), MVE_Failed);
    }
}

void MujinVisionManager::_ExecuteConfigurationCommand(const ptree& command_pt, std::stringstream& result_ss)
{
    std::string command = command_pt.get<std::string>("command");
    if (command == "Ping") {
        result_ss << "{";
        result_ss << "\"timestamp\": " << GetMilliTime();
        result_ss << "}";
    } else if (command == "Cancel") {
        boost::mutex::scoped_lock lock(_mutexCancelCommand);
        if (_bExecutingUserCommand) { // only cancel when user command is being executed
            _bCancelCommand = true;
            _SetStatus(TT_Config, MS_Preempting, "", "", false);
        } else {
            _SetStatusMessage(TT_Config, "No command is being excuted, do nothing.");
        }
        result_ss << "{";
        result_ss << ParametersBase::GetJsonString("status", _GetManagerStatusString(MS_Preempting));
        result_ss << "}";
    } else if (command == "Quit") {
        // throw exception, shutdown gracefully
        Shutdown();
        throw UserInterruptException("User requested exit.");
    } else {
        std::string errstr = "received unknown config command " + command;
        VISIONMANAGER_LOG_ERROR(errstr);
        throw MujinVisionException(errstr, MVE_CommandNotSupported);
    }
}

void MujinVisionManager::_ExecuteUserCommand(const ptree& command_pt, std::stringstream& result_ss)
{
    uint64_t starttime = GetMilliTime();
    _SetStatus(TT_Command, MS_Active);
    {
        // only one command thread is running, so _bExecutingUserCommand must be false at this point, and _bCancelCommand must not be true, therefore no race condition of setting _bCancelCommand from true to false
        boost::mutex::scoped_lock lock(_mutexCancelCommand);
        _bCancelCommand = false;
        _bExecutingUserCommand = true;
    }
    std::string command = command_pt.get<std::string>("command");
    if (command == "StartDetectionLoop" ||
        command == "StopDetectionLoop" ||
        command == "IsDetectionRunning" ||
        (command.size() >= 3 && command.substr(0,3) == "Get")) {
        // these commands can be called at any time

        if (command == "StartDetectionLoop") {
            if (command_pt.count("regionname") == 0) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = command_pt.get<std::string>("regionname");
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() before calling " + command, MVE_NotInitialized);
            }

            std::vector<std::string> cameranames;
            boost::optional<const ptree&> cameranames_pt(command_pt.get_child_optional("cameranames"));
            if (!!cameranames_pt) {
                FOREACH(v, *cameranames_pt) {
                    cameranames.push_back(v->second.get<std::string>(""));
                }
            }
            std::vector<std::string> evcamnames;
            boost::optional<const ptree&> evcamnames_pt(command_pt.get_child_optional("executionverificationcameranames"));
            if (!!evcamnames_pt) {
                FOREACH(v, *evcamnames_pt) {
                    evcamnames.push_back(v->second.get<std::string>(""));
                }
            }
            double voxelsize = command_pt.get<double>("voxelsize", 0.01);
            double pointsize = command_pt.get<double>("pointsize", 0.005);
            bool ignoreocclusion = command_pt.get<bool>("ignoreocclusion", false);
            bool stoponleftinorder = command_pt.get<bool>("stoponleftinorder", false);
            unsigned int maxage = command_pt.get<unsigned int>("maxage", 0);
            unsigned int fetchimagetimeout = command_pt.get("fetchimagetimeout", 0);
            std::string obstaclename = command_pt.get<std::string>("obstaclename", "__dynamicobstacle__");
            unsigned long long detectionstarttime = command_pt.get<unsigned long long>("starttime", 0);
            std::string locale = command_pt.get<std::string>("locale", "en_US");
            unsigned int maxnumfastdetection = command_pt.get<unsigned int>("maxnumfastdetection", 1);
            unsigned int maxnumdetection = command_pt.get<unsigned int>("maxnumdetection", 0);
            _locale = locale;
            boost::optional< const boost::property_tree::ptree& > optchild;
            optchild = command_pt.get_child_optional("worldresultoffsettransform");
            Transform tworldresultoffset;
            if (!!optchild) {
                ptree worldresultoffsetpt = command_pt.get_child("worldresultoffsettransform");
                tworldresultoffset = GetTransform(worldresultoffsetpt);
            } else {
                tworldresultoffset.trans[0] = 0;
                tworldresultoffset.trans[1] = 0;
                tworldresultoffset.trans[2] = 0;
                tworldresultoffset.rot[0] = 1;
                tworldresultoffset.rot[1] = 0;
                tworldresultoffset.rot[2] = 0;
                tworldresultoffset.rot[3] = 0;
            }
            bool sendVerificationPointCloud = command_pt.get<bool>("sendVerificationPointCloud", true);
            if (IsDetectionRunning()) {
                VISIONMANAGER_LOG_WARN("detection is already running, do nothing.");
            } else {
                StartDetectionLoop(regionname, cameranames, evcamnames, tworldresultoffset, voxelsize, pointsize, ignoreocclusion, maxage, fetchimagetimeout, obstaclename, detectionstarttime, locale, maxnumfastdetection, maxnumdetection, sendVerificationPointCloud, stoponleftinorder);
            }
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "StopDetectionLoop") {
            StopDetectionLoop();
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "IsDetectionRunning") {
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("isdetectionrunning") << ": " << IsDetectionRunning();
            result_ss << "}";
        } else if (command == "GetLatestDetectedObjects") {
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() before calling " + command, MVE_NotInitialized);
            }
            std::vector<DetectedObjectPtr> detectedobjectsworld;
            std::string resultstate;
            std::vector<Real> points;
            bool returnpoints = command_pt.get<bool>("returnpoints", false);
            GetLatestDetectedObjects(detectedobjectsworld, resultstate, points, returnpoints);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("detectedobjects") << ": [";
            for (unsigned int i=0; i<detectedobjectsworld.size(); ++i) {
                result_ss << detectedobjectsworld.at(i)->GetJsonString();
                if (i+1 < detectedobjectsworld.size()) {
                    result_ss << ", ";
                }
            }
            result_ss << "], ";
            if( resultstate.size() > 0 ) {
                result_ss << ParametersBase::GetJsonString("state") << ": " << resultstate << ", ";
            }
            else {
                result_ss << ParametersBase::GetJsonString("state") << ": {},";
            }
            if (returnpoints) {
                result_ss << ParametersBase::GetJsonString("points") << ": " << ParametersBase::GetJsonString(points) << ", ";
            }
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "GetCameraId") {
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            std::string cameraname = command_pt.get<std::string>("cameraname");
            std::string cameraid;
            GetCameraId(cameraname, cameraid);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("cameraid", cameraid) << ", ";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "GetVisionmanagerConfig") {
            std::string config;
            GetConfig("visionmanager", config);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("visionmanagerconfig") << ": " << config << ",";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "GetDetectorConfig") {
            std::string config;
            GetConfig("detector", config);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("detectorconfigname") << ": " << config << ",";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "GetImagesubscriberConfig") {
            std::string config;
            GetConfig("imagesubscriber", config);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("imagesubscriberconfigname") << ": " << config << ",";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "GetConfigPort") {
            unsigned int port;
            GetConfigPort(port);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("configport") << ": " << port << ",";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "GetStatusPort") {
            unsigned int port;
            GetStatusPort(port);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("statusport") << ": " << port << ",";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        }

    } else if (!!_pDetectionThread && !_bStopDetectionThread) {
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
            std::string locale = command_pt.get<std::string>("locale", "en_US");
            Initialize(command_pt.get<std::string>("visionmanagerconfig"),
                       command_pt.get<std::string>("detectorconfigname"),
                       command_pt.get<std::string>("imagesubscriberconfig"),
                       command_pt.get<std::string>("mujinControllerIp", ""),
                       command_pt.get<unsigned int>("mujinControllerPort", 0),
                       command_pt.get<std::string>("mujinControllerUsernamePass"),
                       command_pt.get<std::string>("defaultTaskParameters"),
                       command_pt.get<std::string>("containerParameters"),
                       command_pt.get<unsigned int>("binpickingTaskZmqPort"),
                       command_pt.get<unsigned int>("binpickingTaskHeartbeatPort"),
                       command_pt.get<double>("binpickingTaskHeartbeatTimeout"),
                       command_pt.get<std::string>("binpickingTaskScenePk"),
                       command_pt.get<std::string>("targetname"),
                       command_pt.get<std::string>("targeturi"),
                       command_pt.get<std::string>("targetupdatename"),
                       command_pt.get<std::string>("streamerIp"),
                       command_pt.get<unsigned int>("streamerPort"),
                       command_pt.get<std::string>("tasktype","binpicking"),
                       command_pt.get<unsigned int>("controllertimeout", 10),
                       command_pt.get<std::string>("locale", "en_US"),
                       command_pt.get<std::string>("slaverequestid", ""),
                       command_pt.get<std::string>("targetdetectionarchiveurl", "")
                       );
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "DetectObjects") {
            if (command_pt.count("regionname") == 0) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = command_pt.get<std::string>("regionname");
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            std::vector<std::string> cameranames;
            boost::optional<const ptree&> cameranames_pt(command_pt.get_child_optional("cameranames"));
            if (!!cameranames_pt) {
                FOREACH(v, *cameranames_pt) {
                    cameranames.push_back(v->second.get<std::string>(""));
                }
            }
            bool ignoreocclusion = command_pt.get("ignoreocclusion", false);
            unsigned int maxage = command_pt.get("maxage", 0);
            unsigned int fetchimagetimeout = command_pt.get("fetchimagetimeout", 0);
            bool fastdetection = command_pt.get("fastdetection", false);
            bool bindetection = command_pt.get("bindetection", false);
            std::vector<DetectedObjectPtr> detectedobjects;
            std::string resultstate;
            _DetectObjects(TT_Command, _pBinpickingTask, regionname, cameranames, detectedobjects, resultstate, ignoreocclusion, maxage, fetchimagetimeout, fastdetection, bindetection);
            result_ss << "{";
            result_ss << _GetJsonString(detectedobjects) << ", ";
            result_ss << ParametersBase::GetJsonString("state") << ": " << resultstate << ", ";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "SendPointCloudObstacleToController") {
            if (command_pt.count("regionname") == 0) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = command_pt.get<std::string>("regionname");
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }

            std::vector<std::string> cameranames;
            boost::optional<const ptree&> cameranames_pt(command_pt.get_child_optional("cameranames"));
            if (!!cameranames_pt) {
                FOREACH(v, *cameranames_pt) {
                    cameranames.push_back(v->second.get<std::string>(""));
                }
            }
            std::vector<DetectedObjectPtr> detectedobjects;
            boost::optional<const ptree&> detectedobjects_pt(command_pt.get_child_optional("detectedobjects"));
            if (!!detectedobjects_pt) {
                FOREACH(v, *detectedobjects_pt) {
                    detectedobjects.push_back(DetectedObjectPtr(new DetectedObject(v->second.get_child(""))));
                }
            }
            unsigned int maxage = command_pt.get("maxage", 0);
            unsigned int fetchimagetimeout = command_pt.get("fetchimagetimeout", 0);
            double voxelsize = command_pt.get("voxelsize", 0.01);
            double pointsize = command_pt.get("pointsize", 0.005);
            std::string obstaclename = command_pt.get("obstaclename", "__dynamicobstacle__");
            bool fast = command_pt.get("fast", false);
            bool request = command_pt.get("request", true);
            bool async = command_pt.get("async", false);
            std::string locale = command_pt.get("locale", "en_US");
            _locale = locale;
            SendPointCloudObstacleToController(regionname, cameranames, detectedobjects, maxage, fetchimagetimeout, voxelsize, pointsize, obstaclename, fast, request, async, locale);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "VisualizePointCloudOnController") {
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            std::string regionname = command_pt.get<std::string>("regionname", "");
            std::vector<std::string> cameranames;
            boost::optional<const ptree&> cameranames_pt(command_pt.get_child_optional("cameranames"));
            if (!!cameranames_pt) {
                FOREACH(v, *cameranames_pt) {
                    cameranames.push_back(v->second.get<std::string>(""));
                }
            }
            double pointsize = command_pt.get("pointsize",0.005);
            double voxelsize = command_pt.get("voxelsize",0.005);
            bool ignoreocclusion = command_pt.get("ignoreocclusion",false);
            unsigned int maxage = command_pt.get("maxage",0);
            unsigned int fetchimagetimeout = command_pt.get("fetchimagetimeout", 0);
            bool request = command_pt.get("request", true);
            VisualizePointCloudOnController(regionname, cameranames, pointsize, ignoreocclusion, maxage, fetchimagetimeout, request, voxelsize);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "ClearVisualizationOnController") {
            if (!_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            ClearVisualizationOnController();
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "StartVisualizePointCloudThread") {
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            std::string regionname = command_pt.get<std::string>("regionname", "");
            std::vector<std::string> cameranames;
            boost::optional<const ptree&> cameranames_pt(command_pt.get_child_optional("cameranames"));
            if (!!cameranames_pt) {
                FOREACH(v, *cameranames_pt) {
                    cameranames.push_back(v->second.get<std::string>(""));
                }
            }
            double pointsize = command_pt.get("pointsize",0.005);
            double voxelsize = command_pt.get("voxelsize",0.005);
            bool ignoreocclusion = command_pt.get("ignoreocclusion",false);
            unsigned int maxage = command_pt.get("maxage",0);
            unsigned int fetchimagetimeout = command_pt.get("fetchimagetimeout", 0);
            bool request = command_pt.get("request", true);
            StartVisualizePointCloudThread(regionname, cameranames, pointsize, ignoreocclusion, maxage, fetchimagetimeout, request, voxelsize);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "StopVisualizePointCloudThread") {
            if (!_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            StopVisualizePointCloudThread();
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "DetectRegionTransform") {
            if (command_pt.count("regionname") == 0) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = command_pt.get<std::string>("regionname");
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            std::vector<std::string> cameranames;
            boost::optional<const ptree&> cameranames_pt(command_pt.get_child_optional("cameranames"));
            if (!!cameranames_pt) {
                FOREACH(v, *cameranames_pt) {
                    cameranames.push_back(v->second.get<std::string>(""));
                }
            }
            bool ignoreocclusion = command_pt.get("ignoreocclusion",false);
            unsigned int maxage = command_pt.get("maxage",0);
            unsigned int fetchimagetimeout = command_pt.get("fetchimagetimeout", 0);
            mujinvision::Transform regiontransform;
            DetectRegionTransform(regionname, cameranames, regiontransform, ignoreocclusion, maxage, fetchimagetimeout);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString(regiontransform) << ", ";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "SaveSnapshot") {
            if (!_pBinpickingTask || !_pImagesubscriberManager) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }

            bool ignoreocclusion = command_pt.get("ignoreocclusion",false);
            unsigned int maxage = command_pt.get("maxage",0);
            unsigned int fetchimagetimeout = command_pt.get("fetchimagetimeout", 0);
            SaveSnapshot(command_pt.get<std::string>("regionname"), ignoreocclusion, maxage, fetchimagetimeout);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "UpdateDetectedObjects") {
            if (command_pt.count("regionname") == 0) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = command_pt.get<std::string>("regionname");
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }

            std::vector<DetectedObjectPtr> detectedobjects;
            boost::optional<const ptree&> detectedobjects_pt(command_pt.get_child_optional("detectedobjects"));
            if (!!detectedobjects_pt) {
                FOREACH(v, *detectedobjects_pt) {
                    detectedobjects.push_back(DetectedObjectPtr(new DetectedObject(v->second.get_child(""))));
                }
            }
            std::string resultstate = command_pt.get<std::string>("state", "");
            bool sendtocontroller = command_pt.get<bool>("sendtocontroller", true);
            UpdateDetectedObjects(detectedobjects, resultstate, sendtocontroller);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "SyncRegion") {
            if (command_pt.count("regionname") == 0) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = command_pt.get<std::string>("regionname");
            if (!_pDetector || !_pBinpickingTask || !_pImagesubscriberManager) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }
            SyncRegion(regionname);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "SyncCameras") {
            std::string regionname = command_pt.get<std::string>("regionname", "");
            if (!_pBinpickingTask || !_pImagesubscriberManager) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first before calling " + command, MVE_NotInitialized);
            }

            std::vector<std::string> cameranames;
            boost::optional<const ptree&> cameranames_pt(command_pt.get_child_optional("cameranames"));
            if (!!cameranames_pt) {
                FOREACH(v, *cameranames_pt) {
                    cameranames.push_back(v->second.get<std::string>(""));
                }
            }
            SyncCameras(regionname, cameranames);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "SaveVisionmanagerConfig") {
            if (command_pt.count("visionmanagerconfigname") == 0) {
                throw MujinVisionException("visionmanagerconfigname is not specified.", MVE_InvalidArgument);
            }
            if (command_pt.count("config") == 0) {
                throw MujinVisionException("config is not specified.", MVE_InvalidArgument);
            }
            SaveConfig("visionmanager", command_pt.get<std::string>("visionmanagerconfigname"), command_pt.get<std::string>("config", ""));
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "SaveDetectorConfig") {
            if (command_pt.count("detectorconfigname") == 0) {
                throw MujinVisionException("detectorconfigname is not specified.", MVE_InvalidArgument);
            }
            if (command_pt.count("config") == 0) {
                throw MujinVisionException("config is not specified.", MVE_InvalidArgument);
            }
            SaveConfig("detector", command_pt.get<std::string>("detectorconfigname"), command_pt.get<std::string>("config", ""));
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "SaveImagesubscriberConfig") {
            if (command_pt.count("imagesubscriberconfigname") == 0) {
                throw MujinVisionException("imagesubscriberconfigname is not specified.", MVE_InvalidArgument);
            }
            if (command_pt.count("config") == 0) {
                throw MujinVisionException("config is not specified.", MVE_InvalidArgument);
            }
            SaveConfig("imagesubscriber", command_pt.get<std::string>("imagesubscriberconfigname"), command_pt.get<std::string>("config", ""));
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else {
            if(_mNameCommand.find(command) == _mNameCommand.end()) {
                std::stringstream ss;
                ss << "Received unknown command " << command << ".";
                throw MujinVisionException(ss.str(), MVE_CommandNotSupported);
            } else {
                boost::shared_ptr<CustomCommand> customcommand = _mNameCommand[command];
                std::stringstream customresultss;
                customcommand->fn(this, command_pt, customresultss);
                result_ss << "{";
                result_ss << ParametersBase::GetJsonString("customresult") << ": " << customresultss.str();
                result_ss << ", " << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
                result_ss << "}";
            }
        }
    }
    _SetStatus(TT_Command, MS_Pending);
}

bool MujinVisionManager::IsDetectionRunning()
{
    return !!_pDetectionThread && _bIsDetectionRunning;
}

void MujinVisionManager::_StatusThread(const unsigned int port, const unsigned int ms)
{
    _StartStatusPublisher(port);
    std::stringstream ss;
    ss << "Started status thread (port: " << port << ").";
    VISIONMANAGER_LOG_DEBUG(ss.str());
    std::vector<ManagerStatus> vstatus;
    std::vector<std::string> vcfgmsg, vcmdmsg, vdetectormsg, vupdateenvmsg, vcontrollermonmsg, vsendpclmsg;
    std::vector<std::string> vcfgerr, vcmderr, vdetectorerr, vupdateenverr, vcontrollermonerr, vsendpclerr;
    std::vector<unsigned long long> vtimestamp;
    while (!_bStopStatusThread) {
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
            //VISIONMANAGER_LOG_ERROR(_GetStatusJsonString(vtimestamp.at(i), _GetManagerStatusString(vstatus.at(i)), vcmdmsg.at(i), vcmderr.at(i), vcfgmsg.at(i), vcfgerr.at(i), vdetectormsg.at(i), vdetectorerr.at(i), vupdateenvmsg.at(i), vupdateenverr.at(i), vcontrollermonmsg.at(i), vcontrollermonerr.at(i), vsendpclmsg.at(i), vsendpclerr.at(i)));
            _pStatusPublisher->Publish(_GetStatusJsonString(vtimestamp.at(i), _GetManagerStatusString(vstatus.at(i)), vcmdmsg.at(i), vcmderr.at(i), vcfgmsg.at(i), vcfgerr.at(i), vdetectormsg.at(i), vdetectorerr.at(i), vupdateenvmsg.at(i), vupdateenverr.at(i), vcontrollermonmsg.at(i), vcontrollermonerr.at(i), vsendpclmsg.at(i), vsendpclerr.at(i)));
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
    }
    _pStatusPublisher->Publish(_GetStatusJsonString(GetMilliTime(), _GetManagerStatusString(MS_Lost), ""));
    VISIONMANAGER_LOG_DEBUG("Stopped status publisher");
}

std::string MujinVisionManager::_GetStatusJsonString(const unsigned long long timestamp, const std::string& status, const std::string& cmdmsg, const std::string& cmderr, const std::string& cfgmsg, const std::string& cfgerr, const std::string& detectormsg, const std::string& detectorerr, const std::string& updateenvmsg, const std::string& updateenverr, const std::string& controllermonmsg, const std::string& controllermonerr, const std::string& sendpclmsg, const std::string& sendpclerr)
{
    std::stringstream ss;
    ss << "{";
    ss << ParametersBase::GetJsonString("timestamp") << ": " << timestamp << ", ";
    ss << ParametersBase::GetJsonString("status", status) << ", ";
    ss << ParametersBase::GetJsonString("commandmessage", cmdmsg) << ", ";
    if (cmderr != "") {
        ss << ParametersBase::GetJsonString("commanderrorcode", cmderr) << ", ";
    }
    ss << ParametersBase::GetJsonString("detectormessage", detectormsg) << ", ";
    if (detectorerr != "") {
        ss << ParametersBase::GetJsonString("detectorerrorcode", detectorerr) << ", ";
    }
    ss << ParametersBase::GetJsonString("updateenvironmentmessage", updateenvmsg) << ", ";
    if (updateenverr != "") {
        ss << ParametersBase::GetJsonString("updateenvironmenterrorcode", cmderr) << ", ";
    }
    ss << ParametersBase::GetJsonString("controllermonitormessage", controllermonmsg) << ", ";
    if (controllermonerr != "") {
        ss << ParametersBase::GetJsonString("controllermonitorerrorcode", controllermonerr) << ", ";
    }
    ss << ParametersBase::GetJsonString("sendpointcloudmessage", sendpclmsg) << ", ";
    if (sendpclerr != "") {
        ss << ParametersBase::GetJsonString("sendpointclouderrorcode", sendpclerr) << ", ";
    }
    ss << ParametersBase::GetJsonString("isdetectionrunning", IsDetectionRunning());
    ss << ", " << ParametersBase::GetJsonString("isvisualizepointcloudrunning", _bIsVisualizePointcloudRunning);
    ss << ", " << ParametersBase::GetJsonString("issendpointcloudrunning", _bIsSendPointcloudRunning);
    ss << ", " << ParametersBase::GetJsonString("isenvironmentupdaterunning", _bIsEnvironmentUpdateRunning);
    ss << ", " << ParametersBase::GetJsonString("lastupdateenvironmenttimestamp", _tsLastEnvUpdate);
    ss << "}";
    ParametersBase::ValidateJsonString(ss.str());
    return ss.str();
}

void MujinVisionManager::_CommandThread(const unsigned int port)
{
    _StartCommandServer(port);
    std::stringstream ss;
    ss << "Started command thread (port: " << port << ").";
    VISIONMANAGER_LOG_DEBUG(ss.str());
    std::string incomingmessage;
    ptree command_pt;
    std::stringstream command_ss, result_ss;
    std::string resultstr;
    while (!_mPortStopCommandThread[port]) {
        try {
            // receive message
            if (_mPortCommandServer[port]->Recv(incomingmessage, 100) > 0) {
                VISIONMANAGER_LOG_DEBUG("Received command message: " + incomingmessage + ".");
                // execute command
                command_ss.str("");
                command_ss.clear();
                command_ss.str(incomingmessage);
                read_json(command_ss, command_pt);
                result_ss.str("");
                result_ss.clear();
                try {
                    if (port == _configport) {
                        _ExecuteConfigurationCommand(command_pt, result_ss);
                    } else if (port == _commandport) {
                        _ExecuteUserCommand(command_pt, result_ss);
                    }
                }
                catch (const UserInterruptException& ex) { // need to catch it here, otherwise zmq will be in bad state
                    if (port == _configport) {
                        VISIONMANAGER_LOG_WARN("User requested program exit.");
                        result_ss << "{}";
                        _mPortStopCommandThread[_configport] = true;
                    } else {
                        _SetStatus(TT_Command, MS_Preempted, "", "", false);
                        VISIONMANAGER_LOG_WARN("User interruped command execution.");
                        result_ss << "{" << ParametersBase::GetJsonString("status", _GetManagerStatusString(MS_Preempted)) << "}";
                    }
                }
                catch (const MujinVisionException& e) {
                    VISIONMANAGER_LOG_ERROR("MujinVisionException " + e.message());
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
                    result_ss << "{" << ParametersBase::GetJsonString(e) << "}";
                    _SetStatus(TT_Command, MS_Aborted, "", e.message(), false);
                }
                catch (const zmq::error_t& e) {
                    std::stringstream ss;
                    ss << "caught zmq exception errornum=" << e.num();
                    std::string errstr = ParametersBase::GetExceptionJsonString(GetErrorCodeString(MVE_Failed), ss.str());
                    result_ss << "{" << errstr << "}";
                    VISIONMANAGER_LOG_ERROR(ss.str());
                    _SetStatus(TT_Command, MS_Aborted, "", errstr, false);
                }
                catch (std::exception& e) {
                    std::string whatstr = e.what();
                    boost::replace_all(whatstr, "\"", ""); // need to remove " in the message so that json parser works
                    boost::replace_all(whatstr, "\\", ""); // need to remove \ in the message so that json parser works
                    std::string errstr = ParametersBase::GetExceptionJsonString(GetErrorCodeString(MVE_Failed), whatstr);
                    result_ss << "{" << errstr << "}";
                    VISIONMANAGER_LOG_ERROR("unhandled std exception, " + whatstr);
                    _SetStatus(TT_Command, MS_Aborted, "", errstr, false);
                }
                catch (...) {
                    std::string whatstr = "unhandled exception!";
                    VISIONMANAGER_LOG_ERROR(whatstr);
                    std::string errstr = ParametersBase::GetExceptionJsonString(GetErrorCodeString(MVE_Failed), whatstr);
                    result_ss << "{" << errstr << "}";
                    _SetStatus(TT_Command, MS_Aborted, "", errstr, false);
                }

                // send output
                ParametersBase::ValidateJsonString(result_ss.str());
                _mPortCommandServer[port]->Send(result_ss.str());
            }
        }
        catch (const zmq::error_t& e) {
            if (!_mPortStopCommandThread[port]) {
                std::string errstr = "Failed to receive command";
                _SetStatus(TT_Command, MS_Aborted, errstr, "", false);
                VISIONMANAGER_LOG_WARN(errstr);
            }
        }
        catch (const UserInterruptException& ex) {
            std::string errstr = "User requested program exit";
            _SetStatus(TT_Command, MS_Aborted, errstr, "", false);
            VISIONMANAGER_LOG_WARN(errstr);
        }
        catch (...) {
            if (!_bShutdown) {
                std::stringstream errss;
                errss << "caught unhandled exception in command thread port=" << port;
                _SetStatus(TT_Command, MS_Aborted, errss.str(), "", false);
                VISIONMANAGER_LOG_WARN(errss.str());
            }
        }
    }
}

void MujinVisionManager::_StartDetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const unsigned long long& starttime, const unsigned int maxnumfastdetection, const unsigned int maxnumdetection, const bool stoponleftinorder, ImagesubscriberHandlerPtr ih)
{
    if (starttime > 0) {
        _tsStartDetection = starttime;
    } else {
        _tsStartDetection = GetMilliTime();
    }
    if (!!_pDetectionThread && !_bStopDetectionThread) {
        _SetStatusMessage(TT_Command, "Detection thread is already running, do nothing.");
    } else {
        _bStopDetectionThread = false;
        DetectionThreadParams params;
        params.voxelsize = voxelsize;
        params.pointsize = pointsize;
        params.ignoreocclusion = ignoreocclusion;
        params.maxage = maxage;
        params.fetchimagetimeout = fetchimagetimeout;
        params.maxnumfastdetection = maxnumfastdetection;
        params.maxnumdetection = maxnumdetection;
        params.stoponleftinorder = stoponleftinorder;

        _bIsDetectionRunning = true;
        // reset cached binpicking state to ensure clean state, e.g. lastGrabbedTargetTimeStamp
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
        // need to pass in a reference of ih since we don't want the thread object to hold a reference to it, only the thread. However, this means we have to wait until the thread starts running before we resume. In order to achieve that, wait on a condition that will be signaled by the running thread.
        boost::condition condrunningthread;
        {
            boost::mutex::scoped_lock lock(_mutexThreadResourceSync);
            _pDetectionThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_DetectionThread, this, regionname, cameranames, params, boost::ref(ih), boost::ref(condrunningthread))));
            condrunningthread.wait(lock);
        }
    }
}

void MujinVisionManager::_StartUpdateEnvironmentThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const std::string& obstaclename, ImagesubscriberHandlerPtr ih, const unsigned int waitinterval, const std::string& locale)
{
    if (!!_pUpdateEnvironmentThread && !_bStopUpdateEnvironmentThread) {
        _SetStatusMessage(TT_Command, "UpdateEnvironment thread is already running, do nothing.");
    } else {
        _bStopUpdateEnvironmentThread = false;
        _bIsEnvironmentUpdateRunning = true;
        UpdateEnvironmentThreadParams params;
        params.regionname = regionname;
        params.cameranames = cameranames;
        params.voxelsize = voxelsize;
        params.pointsize = pointsize;
        params.obstaclename = obstaclename;
        params.waitinterval = waitinterval;
        params.locale = locale;
        // need to pass in a reference of ih since we don't want the thread object to hold a reference to it, only the thread. However, this means we have to wait until the thread starts running before we resume. In order to achieve that, wait on a condition that will be signaled by the running thread.
        boost::condition condrunningthread;
        {
            boost::mutex::scoped_lock lock(_mutexThreadResourceSync);
            _pUpdateEnvironmentThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_UpdateEnvironmentThread, this, params, boost::ref(ih), boost::ref(condrunningthread))));
            condrunningthread.wait(lock);
        }
    }
}

void MujinVisionManager::_StartExecutionVerificationPointCloudThread(const std::string& regionname, const std::vector<std::string>& cameranames, const std::vector<std::string>& evcamnames, const double voxelsize, const double pointsize, const std::string& obstaclename, ImagesubscriberHandlerPtr ih, const unsigned int waitinterval, const std::string& locale)
{
    if (!!_pExecutionVerificationPointCloudThread && !_bStopExecutionVerificationPointCloudThread) {
        _SetStatusMessage(TT_Command, "ExecutionVerificationPointCloud thread is already running, do nothing.");
    } else {
        _bStopExecutionVerificationPointCloudThread = false;
        _bIsEnvironmentUpdateRunning = true;
        SendExecutionVerificationPointCloudParams params;
        params.regionname = regionname;
        params.cameranames = cameranames;
        params.executionverificationcameranames = evcamnames;
        params.voxelsize = voxelsize;
        params.pointsize = pointsize;
        params.obstaclename = obstaclename;
        params.waitinterval = waitinterval;
        params.locale = locale;
        // need to pass in a reference of ih since we don't want the thread object to hold a reference to it, only the thread. However, this means we have to wait until the thread starts running before we resume. In order to achieve that, wait on a condition that will be signaled by the running thread.
        boost::condition condrunningthread;
        {
            boost::mutex::scoped_lock lock(_mutexThreadResourceSync);
            _pExecutionVerificationPointCloudThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_SendExecutionVerificationPointCloudThread, this, params, boost::ref(ih), boost::ref(condrunningthread))));
            condrunningthread.wait(lock);
        }
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

void MujinVisionManager::_StartVisualizePointCloudThread(const std::string& regionname, const std::vector<std::string>& cameranames, ImagesubscriberHandlerPtr ih, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request, const double voxelsize)
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
        params.maxage = maxage;
        params.fetchimagetimeout = fetchimagetimeout;
        params.request = request;
        params.voxelsize = voxelsize;
        // need to pass in a reference of ih since we don't want the thread object to hold a reference to it, only the thread. However, this means we have to wait until the thread starts running before we resume. In order to achieve that, wait on a condition that will be signaled by the running thread.
        boost::condition condrunningthread;
        {
            boost::mutex::scoped_lock lock(_mutexThreadResourceSync);
            _pVisualizePointCloudThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_VisualizePointCloudThread, this, params, boost::ref(ih), boost::ref(condrunningthread))));
            condrunningthread.wait(lock);
        }
    }
}

void MujinVisionManager::_StopDetectionThread()
{
    std::stringstream ss;
    _SetStatusMessage(TT_Command, "Stopping detectoin thread.");
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
    VISIONMANAGER_LOG_DEBUG(ss.str());
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
    VISIONMANAGER_LOG_DEBUG("stopped updateenvironment thread");
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
    VISIONMANAGER_LOG_DEBUG("stopped execution verification thread");
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
    VISIONMANAGER_LOG_DEBUG("stopped controllermonitor thread");
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
    VISIONMANAGER_LOG_DEBUG("stopped pointcloud visualization thread");
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

void MujinVisionManager::_DetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, DetectionThreadParams params, ImagesubscriberHandlerPtr& ihraw, boost::condition& condrunningthread)
{
    ImagesubscriberHandlerPtr ih = ihraw;
    {
        // notify creator that the handle was copied
        boost::mutex::scoped_lock lock(_mutexThreadResourceSync);
        condrunningthread.notify_all();
    }
    FalseSetter turnOffDetection(_bIsDetectionRunning);
    double voxelsize = params.voxelsize;
    //double pointsize = params.pointsize;
    bool ignoreocclusion = params.ignoreocclusion;
    bool stoponleftinorder = params.stoponleftinorder;
    unsigned int maxage = params.maxage;
    unsigned int fetchimagetimeout = params.fetchimagetimeout;
    unsigned int maxnumfastdetection = params.maxnumfastdetection;
    unsigned int maxnumdetection = params.maxnumdetection;
    BinPickingTaskResourcePtr pBinpickingTask = _pSceneResource->GetOrCreateBinPickingTaskFromName_UTF8(_tasktype+std::string("task1"), _tasktype, TRO_EnableZMQ);
    std::string userinfo_json = "{\"username\": " + ParametersBase::GetJsonString(_pControllerClient->GetUserName()) + ", \"locale\": " + ParametersBase::GetJsonString(_locale) + "}";
    VISIONMANAGER_LOG_DEBUG("initialzing binpickingtask in DetectionThread with userinfo " + userinfo_json);

    ParametersBase::ValidateJsonString(userinfo_json);

    pBinpickingTask->Initialize(_defaultTaskParameters, _binpickingTaskZmqPort, _binpickingTaskHeartbeatPort, _zmqcontext, false, _binpickingTaskHeartbeatTimeout, _controllerCommandTimeout, userinfo_json, _slaverequestid);

    uint64_t time0;
    int numfastdetection = maxnumfastdetection; // max num of times to run fast detection
    int lastDetectedId = 0;
    int lastPickedId = -1;
    uint64_t lastocclusionwarningts = 0;
    uint64_t lastbinpickingstatewarningts = 0;
    uint64_t lastwaitforocclusionwarningts = 0;
    uint64_t lastattemptts = 0;
    int numPickAttempt = 0;
    bool isControllerPickPlaceRunning = false;
    bool isRobotOccludingSourceContainer = false;
    bool forceRequestDetectionResults = false;
    bool isGrabbingTarget = false;
    bool isGrabbingLastTarget = false;
    bool detectcontaineronly = false;
    int numLeftInOrder = -1;
    int orderNumber = -1;
    unsigned long long binpickingstateTimestamp = 0;
    unsigned long long lastGrabbedTargetTimeStamp = 0;
    unsigned int numdetection = 0;
    std::vector<DetectedObjectPtr> detectedobjects;
    while (!_bStopDetectionThread && (maxnumdetection <= 0 || numdetection < maxnumdetection) && !(stoponleftinorder && numLeftInOrder == 0 && lastGrabbedTargetTimeStamp > _tsStartDetection && _tsLastEnvUpdate > 0 && lastGrabbedTargetTimeStamp < _tsLastEnvUpdate)) {
        detectcontaineronly = false;
        time0 = GetMilliTime();
        std::string resultstate;
        detectedobjects.resize(0);
        try {
            {
                boost::mutex::scoped_lock lock(_mutexControllerBinpickingState);
                if (binpickingstateTimestamp != _binpickingstateTimestamp) {
                    std::stringstream ss;
                    ss << "DetectionThread binpickingstate: ts=" << _binpickingstateTimestamp << " numPickAttempt=" << _numPickAttempt << " isControllerPickPlaceRunning=" << _bIsControllerPickPlaceRunning << " isRobotOccludingContainer=" << _bIsRobotOccludingSourceContainer << " forceRequestDetectionResults=" << forceRequestDetectionResults << " numLeftInOrder=" << numLeftInOrder << " lastGrabbedTargetTimeStamp=" << _lastGrabbedTargetTimestamp << " _tsLastEnvUpdate=" << _tsLastEnvUpdate;
                    VISIONMANAGER_LOG_DEBUG(ss.str());
                }
                binpickingstateTimestamp = _binpickingstateTimestamp;
                lastGrabbedTargetTimeStamp = _lastGrabbedTargetTimestamp;
                if (_numPickAttempt > numPickAttempt) {
                    lastattemptts = binpickingstateTimestamp;
                }
                numPickAttempt = _numPickAttempt;
                isControllerPickPlaceRunning = _bIsControllerPickPlaceRunning;
                isRobotOccludingSourceContainer = _bIsRobotOccludingSourceContainer;
                isGrabbingTarget = _bIsGrabbingTarget;
                isGrabbingLastTarget = _bIsGrabbingLastTarget;
                forceRequestDetectionResults = _bForceRequestDetectionResults;
                numLeftInOrder = _numLeftInOrder;
                orderNumber = _orderNumber;
            }
            if (_bStopDetectionThread) {
                break;
            }
            if (stoponleftinorder && orderNumber > 0 && numLeftInOrder == 0) {
                std::stringstream debugss;
                debugss <<"numLeftInOrder=" << numLeftInOrder << " orderNumber=" << orderNumber << " stoponleftinorder=" << stoponleftinorder << ", check container empty only.";
                VISIONMANAGER_LOG_INFO(debugss.str());
                _pImagesubscriberManager->StartCaptureThread(_GetHardwareIds(cameranames));
                detectcontaineronly = true;
            } else if (!isControllerPickPlaceRunning || forceRequestDetectionResults || _vDetectedObject.size() == 0) { // detect if forced or no result
                std::stringstream ss;
                ss << "force detection, start capturing..." << (int)isControllerPickPlaceRunning << " " << (int)forceRequestDetectionResults << " " << _vDetectedObject.size();
                _pImagesubscriberManager->StartCaptureThread(_GetHardwareIds(cameranames));
            } else {  // do the following only if pick and place thread is running and detection is not forced
                if (numPickAttempt <= lastPickedId) { // if robot has picked
                    if (GetMilliTime() - binpickingstateTimestamp < maxage) { // only do the following if the binpicking state message is up-to-date
                        if (isRobotOccludingSourceContainer) { // skip detection if robot occludes camera
                            if (GetMilliTime() - lastocclusionwarningts > 1000.0) {
                                VISIONMANAGER_LOG_INFO("robot is picking now (occluding camera), stop capturing " + ParametersBase::GetJsonString(_GetHardwareIds(cameranames)));
                                lastocclusionwarningts = GetMilliTime();
                            }
                            if (binpickingstateTimestamp > _lastocclusionTimestamp) {
                                _lastocclusionTimestamp = binpickingstateTimestamp;
                            }
                            _pImagesubscriberManager->StopCaptureThread(_GetHardwareIds(cameranames));
                            continue;
                        } else { // detect when robot is not occluding camera
                            std::stringstream ss;
                            ss << "need to detect for this picking attempt, starting image capturing... " << numPickAttempt << " " << lastPickedId << " " << int(forceRequestDetectionResults) << " " << lastDetectedId << std::endl;
                            VISIONMANAGER_LOG_INFO(ss.str());
                            _pImagesubscriberManager->StartCaptureThread(_GetHardwareIds(cameranames));
                        }
                    } else { // do not detect if binpicking status message is old (controller in bad state)
                        if (GetMilliTime() - lastbinpickingstatewarningts > 1000.0) {
                            std::stringstream ss;
                            ss << "binpickingstateTimestamp (" << binpickingstateTimestamp << ") is > " << maxage << "ms older than current time (" << GetMilliTime() << ") 1" << std::endl;
                            VISIONMANAGER_LOG_WARN(ss.str());
                            lastbinpickingstatewarningts = GetMilliTime();
                        }
                        continue;
                    }
                } else { // if robot has not picked
                    std::stringstream ss;
                    if (GetMilliTime() - binpickingstateTimestamp < maxage) { // only do the following if the binpicking state message is up-to-date
                        if (!isRobotOccludingSourceContainer && GetMilliTime() - lastattemptts <= 10000.0) { // skip detection if robot does not occlude camera up to 10.0 seconds from last picking attempt
                            if (GetMilliTime() - lastwaitforocclusionwarningts > 1000.0) {
                                VISIONMANAGER_LOG_INFO("wait until robot picks (occludes camera)...");
                                lastwaitforocclusionwarningts = GetMilliTime();
                            }
                            boost::this_thread::sleep(boost::posix_time::milliseconds(50));
                            continue;
                        } else {
                            lastPickedId = numPickAttempt;
                            VISIONMANAGER_LOG_INFO("robot has picked");
                            continue;
                        }
                    } else { // do not detect if binpicking status message is old (controller in bad state)
                        if (GetMilliTime() - lastbinpickingstatewarningts > 1000.0) {
                            ss << "binpickingstateTimestamp (" << binpickingstateTimestamp << ") is > " << maxage << "ms older than current time (" << GetMilliTime() << ")" << std::endl;
                            VISIONMANAGER_LOG_WARN(ss.str());
                            lastbinpickingstatewarningts = GetMilliTime();
                        }
                        continue;
                    }
                }
            }

            if( !_bIsEnvironmentUpdateRunning ) {
                VISIONMANAGER_LOG_WARN("environment update thread stopped! so stopping detector");
                break;
            }

            if (_bStopDetectionThread) {
                break;
            }
            if (detectcontaineronly) {
                VISIONMANAGER_LOG_DEBUG("detect to check if container is empty");
                _DetectObjects(TT_Detector, pBinpickingTask, regionname, cameranames, detectedobjects, resultstate, ignoreocclusion, maxage, fetchimagetimeout, false, false, false, false, true);
            } else if (numfastdetection > 0) {
                while (detectedobjects.size() == 0 && numfastdetection > 0) {
                    VISIONMANAGER_LOG_DEBUG("DetectObjects() in fast mode");
                    _DetectObjects(TT_Detector, pBinpickingTask, regionname, cameranames, detectedobjects, resultstate, ignoreocclusion, maxage, fetchimagetimeout, true, true);
                    if (detectedobjects.size() == 0) {
                        numfastdetection -= 1;
                    } else {
                        numfastdetection = 0;
                    }
                }
                if (detectedobjects.size() == 0 && numfastdetection == 0) {
                    VISIONMANAGER_LOG_DEBUG("DetectObjects() in fast mode found no object, detect in normal mode");
                    _DetectObjects(TT_Detector, pBinpickingTask, regionname, cameranames, detectedobjects, resultstate, ignoreocclusion, maxage, fetchimagetimeout, false, false, false, true);
                }
                numfastdetection -= 1;
            } else {
                VISIONMANAGER_LOG_DEBUG("detect normally");
                _DetectObjects(TT_Detector, pBinpickingTask, regionname, cameranames, detectedobjects, resultstate, ignoreocclusion, maxage, fetchimagetimeout, false, false);
            }
            if (_bStopDetectionThread) {
                break;
            }
            std::vector<std::string> cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);
            for (unsigned int i=0; i<cameranamestobeused.size(); i++) {
                std::string cameraname = cameranamestobeused[i];
                std::vector<Real> points;
                std::stringstream ss;
                uint64_t starttime = GetMilliTime();
                {
                    boost::mutex::scoped_lock lock(_mutexDetector);
                    _pDetector->GetPointCloudObstacle(regionname, cameraname, detectedobjects, points, voxelsize, false, true, _filteringstddev, _filteringnumnn);
                }
                ss << "GetPointCloudObstacle() took " << (GetMilliTime() - starttime) / 1000.0f << " secs";
                VISIONMANAGER_LOG_INFO(ss.str());
                if (points.size() / 3 == 0) {
                    _SetDetectorStatusMessage("got 0 point from GetPointCloudObstacle() in detection loop");
                }
                {
                    boost::mutex::scoped_lock lock(_mutexDetectedInfo);
                    _mResultPoints[cameraname] = points;
                }
            }

            lastDetectedId = numPickAttempt;
            if (isControllerPickPlaceRunning && !forceRequestDetectionResults && detectedobjects.size() > 0) {
                VISIONMANAGER_LOG_INFO("detected at least 1 object, stop image capturing...");
                _pImagesubscriberManager->StopCaptureThread(_GetHardwareIds(cameranames));
                VISIONMANAGER_LOG_INFO("capturing stopped");
            } else {
                //VISIONMANAGER_LOG_INFO("detected no object, do not stop image capturing...");
            }
        }
        catch(const std::exception& ex) {
            std::stringstream ss;
            ss << "Caught exception in the detection loop: " << ex.what();
            //std::string errstr = ParametersBase::GetExceptionJsonString(GetErrorCodeString(MVE_RecognitionError), ss.str());
            std::string errstr = ss.str();
            boost::replace_all(errstr, "\"", ""); // need to remove " in the message so that json parser works
            boost::replace_all(errstr, "\\", ""); // need to remove \ in the message so that json parser works
            VISIONMANAGER_LOG_ERROR(errstr);
            _SetDetectorStatusMessage(errstr, GetErrorCodeString(MVE_RecognitionError));
            continue;
        }

        // process results
        if (_bStopDetectionThread) {
            break;
        }

        if( !_bIsEnvironmentUpdateRunning ) {
            VISIONMANAGER_LOG_WARN("environment update thread stopped! so stopping detector");
            break;
        }

        if (resultstate != "null") {
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
            VISIONMANAGER_LOG_INFO(str(boost::format("send %d detected objects with _resultTimestamp=%u")%_vDetectedObject.size()%_resultTimestamp));
        } else {
            VISIONMANAGER_LOG_INFO("resultstate is null, do not update result");
        }
        if (_bStopDetectionThread) {
            break;
        }

        VISIONMANAGER_LOG_INFO("Cycle time: " + boost::lexical_cast<std::string>((GetMilliTime() - time0)/1000.0f) + " secs");
        VISIONMANAGER_LOG_INFO(" ------------------------");
        numdetection += 1;
    }
    if (stoponleftinorder && numLeftInOrder == 0) {
        VISIONMANAGER_LOG_INFO("got out of detection loop because numLeftInOrder is 0, wait for environment to update");
        while (_resultTimestamp > _tsLastEnvUpdate) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        }
        VISIONMANAGER_LOG_INFO("environment is updated with latest result, stop environment updating and capturing");
        VISIONMANAGER_LOG_INFO("stopped environment update thread");
        _pImagesubscriberManager->StopCaptureThread(_GetHardwareIds(cameranames));
        VISIONMANAGER_LOG_INFO("capturing stopped");
        _StopUpdateEnvironmentThread();
        _StopExecutionVerificationPointCloudThread();
    }
    if (numdetection >= maxnumdetection && maxnumdetection!=0) {
        VISIONMANAGER_LOG_INFO("reached max num detection, wait for environment to update");
        while (_resultTimestamp > _tsLastEnvUpdate) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        }
        VISIONMANAGER_LOG_INFO("environment is updated with latest result, stop environment updating and capturing");
        // since threads might be blocking on waiting for captures, so stop capturing to enable the preempt function to exit
        _pImagesubscriberManager->StopCaptureThread(_GetHardwareIds(cameranames));
        VISIONMANAGER_LOG_INFO("capturing stopped");
        _StopUpdateEnvironmentThread();
        _StopExecutionVerificationPointCloudThread();
        VISIONMANAGER_LOG_INFO("stopped environment update thread");
    }
    std::stringstream ss;
    ss << "ending detection thread. numdetection=" << numdetection << " numLeftInOrder=" << numLeftInOrder << " _bStopDetectionThread=" << _bStopDetectionThread << " lastGrabbedTargetTimeStamp=" << lastGrabbedTargetTimeStamp << " _tsLastEnvUpdate=" << _tsLastEnvUpdate << " _tsStartDetection=" << _tsStartDetection;
    VISIONMANAGER_LOG_INFO(ss.str());
}

void MujinVisionManager::_UpdateEnvironmentThread(UpdateEnvironmentThreadParams params, ImagesubscriberHandlerPtr& ihraw, boost::condition& condrunningthread)
{
    ImagesubscriberHandlerPtr ih = ihraw;
    {
        // notify creator that the handle was copied
        boost::mutex::scoped_lock lock(_mutexThreadResourceSync);
        condrunningthread.notify_all();
    }

    try {
        FalseSetter turnoffstatusvar(_bIsEnvironmentUpdateRunning);
        std::string regionname = params.regionname;
        std::vector<std::string> cameranames = params.cameranames;
        //double voxelsize = params.voxelsize;
        double pointsize = params.pointsize;
        std::string obstaclename = params.obstaclename;
        unsigned int waitinterval = params.waitinterval;
        std::string locale = params.locale;
        std::vector<DetectedObjectPtr> vDetectedObject; ///< latest detection result
        std::string resultstate;


        uint64_t lastUpdateTimestamp = GetMilliTime();
        std::vector<std::string> cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);

        BinPickingTaskResourcePtr pBinpickingTask = _pSceneResource->GetOrCreateBinPickingTaskFromName_UTF8(_tasktype+std::string("task1"), _tasktype, TRO_EnableZMQ);
        std::string userinfo_json = "{\"username\": " + ParametersBase::GetJsonString(_pControllerClient->GetUserName()) + ", \"locale\": " + ParametersBase::GetJsonString(locale) + "}";
        VISIONMANAGER_LOG_DEBUG("initialzing binpickingtask in UpdateEnvironmentThread with userinfo " + userinfo_json);

        ParametersBase::ValidateJsonString(userinfo_json);

        pBinpickingTask->Initialize(_defaultTaskParameters, _binpickingTaskZmqPort, _binpickingTaskHeartbeatPort, _zmqcontext, false, _binpickingTaskHeartbeatTimeout, _controllerCommandTimeout, userinfo_json, _slaverequestid);
        uint64_t starttime;
        uint64_t lastwarnedtimestamp1 = 0;

        std::vector<BinPickingTaskResource::DetectedObject> detectedobjects;
        std::vector<Real> totalpoints;
        std::map<std::string, std::vector<Real> > mResultPoints;
        std::vector<Real> newpoints;

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
                //VISIONMANAGER_LOG_DEBUG(str(boost::format("have no detector results to update _resultTimestamp (%u) <= lastUpdateTimestamp (%u), waitfor %dms")%_resultTimestamp%lastUpdateTimestamp%waitinterval));
                boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                continue;
            }
            else {
                lastUpdateTimestamp = _resultTimestamp;
                VISIONMANAGER_LOG_INFO(str(boost::format("updating environment with %d detected objects")%vDetectedObject.size()));
                detectedobjects.resize(0);
                totalpoints.resize(0);
                if( bDetectedObjectsValid ) {
                    // use the already acquired detection results without locking
                    for (unsigned int i=0; i<vDetectedObject.size(); i++) {
                        mujinclient::Transform transform;
                        Transform newtransform = _tWorldResultOffset * vDetectedObject[i]->transform; // apply offset to result transform
                        transform.quaternion[0] = newtransform.rot[0];
                        transform.quaternion[1] = newtransform.rot[1];
                        transform.quaternion[2] = newtransform.rot[2];
                        transform.quaternion[3] = newtransform.rot[3];
                        transform.translate[0] = newtransform.trans[0];
                        transform.translate[1] = newtransform.trans[1];
                        transform.translate[2] = newtransform.trans[2];

                        BinPickingTaskResource::DetectedObject detectedobject;
                        std::stringstream name_ss;
                        name_ss << _targetupdatename << i;
                        detectedobject.name = name_ss.str();
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
                        starttime = GetMilliTime();
                        pBinpickingTask->UpdateEnvironmentState(_targetupdatename, detectedobjects, totalpoints, resultstate, pointsize, obstaclename, "m", 10);
                        std::stringstream ss;
                        ss << "UpdateEnvironmentState with " << detectedobjects.size() << " objects " << (totalpoints.size()/3.) << " points, took " << (GetMilliTime() - starttime) / 1000.0f << " secs";
                        _SetStatusMessage(TT_UpdateEnvironment, ss.str());
                    }
                    catch(const std::exception& ex) {
                        if (GetMilliTime() - lastwarnedtimestamp1 > 1000.0) {
                            lastwarnedtimestamp1 = GetMilliTime();
                            std::stringstream ss;
                            ss << "Failed to update environment state: " << ex.what() << ".";
                            //std::string errstr = ParametersBase::GetExceptionJsonString(GetErrorCodeString(MVE_ControllerError), ss.str());
                            _SetStatusMessage(TT_UpdateEnvironment, ss.str(), GetErrorCodeString(MVE_ControllerError));
                            VISIONMANAGER_LOG_WARN(ss.str());
                        }
                        boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                    }
                    _tsLastEnvUpdate = _resultTimestamp;
                }
                else {
                    // only resultstate is valid and shouldn't update the detected objects or pointcloud... (hack for now)
                    try {
                        starttime = GetMilliTime();
                        pBinpickingTask->UpdateObjects("", std::vector<mujinclient::Transform>(), std::vector<std::string>(), resultstate, "m", 10.0);
                    }
                    catch(const std::exception& ex) {
                        if (GetMilliTime() - lastwarnedtimestamp1 > 1000.0) {
                            lastwarnedtimestamp1 = GetMilliTime();
                            std::stringstream ss;
                            ss << "Failed to update objects state: " << ex.what() << ".";
                            _SetStatusMessage(TT_UpdateEnvironment, ss.str(), GetErrorCodeString(MVE_ControllerError));
                            VISIONMANAGER_LOG_WARN(ss.str());
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
        VISIONMANAGER_LOG_ERROR(errss.str());
    }
    catch (const MujinVisionException& e) {
        std::stringstream errss;
        errss << "Caught MujinVisionException " << e.message() << " in _UpdateEnvironmentThread!";
        _SetStatus(TT_UpdateEnvironment, MS_Aborted, errss.str(), "", false);
        VISIONMANAGER_LOG_ERROR(errss.str());
    }
    catch (const UserInterruptException& ex) {
        std::stringstream errss;
        errss << "User interrupted _UpdateEnvironmentThread!";
        _SetStatus(TT_UpdateEnvironment, MS_Preempted, errss.str(), "", false);
        VISIONMANAGER_LOG_WARN(errss.str());
    }
    catch(const std::exception& ex) {
        std::stringstream errss;
        errss << "Caught exception in _UpdateEnvironmentThread " << ex.what();
        _SetStatus(TT_UpdateEnvironment, MS_Preempted, errss.str(), "", false);
        VISIONMANAGER_LOG_WARN(errss.str());
    }
    catch (...) {
        std::stringstream errss;
        errss << "Caught unknown exception in VisualizePointcloudThread!";
        _SetStatus(TT_UpdateEnvironment, MS_Aborted, errss.str(), "", false);
        VISIONMANAGER_LOG_ERROR(errss.str());
    }
}

void MujinVisionManager::_SendExecutionVerificationPointCloudThread(SendExecutionVerificationPointCloudParams params, ImagesubscriberHandlerPtr& ihraw, boost::condition& condrunningthread)
{
    ImagesubscriberHandlerPtr ih = ihraw;
    {
        // notify creator that the handle was copied
        boost::mutex::scoped_lock lock(_mutexThreadResourceSync);
        condrunningthread.notify_all();
    }

    try {
        //FalseSetter turnoffstatusvar(_bIsExecutionVerificationPointCloudRunning);
        std::string regionname = params.regionname;
        std::vector<std::string> cameranames = params.cameranames;
        std::vector<std::string> evcamnames = params.executionverificationcameranames;
        VISIONMANAGER_LOG_INFO("starting SendExecutionVerificationPointCloudThread " + ParametersBase::GetJsonString(evcamnames));
        //double voxelsize = params.voxelsize;
        double pointsize = params.pointsize;
        std::string obstaclename = params.obstaclename;
        unsigned int waitinterval = params.waitinterval;
        std::string locale = params.locale;
        std::vector<DetectedObjectPtr> vDetectedObject; ///< latest detection result
        std::string resultstate;


        //uint64_t lastUpdateTimestamp = GetMilliTime();
        std::vector<std::string> cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);

        BinPickingTaskResourcePtr pBinpickingTask = _pSceneResource->GetOrCreateBinPickingTaskFromName_UTF8(_tasktype+std::string("task1"), _tasktype, TRO_EnableZMQ);
        std::string userinfo_json = "{\"username\": " + ParametersBase::GetJsonString(_pControllerClient->GetUserName()) + ", \"locale\": " + ParametersBase::GetJsonString(locale) + "}";
        VISIONMANAGER_LOG_DEBUG("initialzing binpickingtask in _SendExecutionVerificationPointCloudThread with userinfo " + userinfo_json);

        ParametersBase::ValidateJsonString(userinfo_json);

        pBinpickingTask->Initialize(_defaultTaskParameters, _binpickingTaskZmqPort, _binpickingTaskHeartbeatPort, _zmqcontext, false, _binpickingTaskHeartbeatTimeout, _controllerCommandTimeout, userinfo_json, _slaverequestid);
        //uint64_t starttime;
        uint64_t lastwarnedtimestamp0 = 0;
        //uint64_t lastwarnedtimestamp1 = 0;
        uint64_t lastsentcloudtime = 0;
        while (!_bStopExecutionVerificationPointCloudThread && evcamnames.size() > 0) {
            // ensure publishing
            _pImagesubscriberManager->StartCaptureThread(_GetHardwareIds(evcamnames));

            // send latest pointcloud for execution verification
            for (unsigned int i=0; i<evcamnames.size(); ++i) {
                std::vector<double> points;
                std::string cameraname = evcamnames.at(i);
                unsigned long long cloudstarttime, cloudendtime;

                _pImagesubscriberManager->GetCollisionPointCloud(cameraname, points, cloudstarttime, cloudendtime, _filteringvoxelsize, _filteringstddev, _filteringnumnn);
                if (cloudstarttime > lastsentcloudtime) {
                    lastsentcloudtime = cloudstarttime;
                    try {
                        uint64_t starttime = GetMilliTime();
                        pBinpickingTask->AddPointCloudObstacle(points, pointsize, "latestobstacle_"+cameraname, cloudstarttime, cloudendtime, true);
                        std::stringstream ss;
                        ss << "Sent latest pointcloud of " << cameraname << " with " << (points.size()/3.) << " points, took " << (GetMilliTime() - starttime) / 1000.0f << " secs";
                        VISIONMANAGER_LOG_DEBUG(ss.str());
                    } catch(const std::exception& ex) {
                        if (GetMilliTime() - lastwarnedtimestamp0 > 1000.0) {
                            lastwarnedtimestamp0 = GetMilliTime();
                            std::stringstream ss;
                            ss << "Failed to send latest pointcloud of " << cameraname << " ex.what()=" << ex.what() << ".";
                            std::string whatstr = ss.str();
                            boost::replace_all(whatstr, "\"", ""); // need to remove " in the message so that json parser works
                            boost::replace_all(whatstr, "\\", ""); // need to remove \ in the message so that json parser works
                            _SetStatusMessage(TT_UpdateEnvironment, whatstr, GetErrorCodeString(MVE_ControllerError));
                            VISIONMANAGER_LOG_WARN(ss.str());
                        }
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
        VISIONMANAGER_LOG_ERROR(errss.str());
    }
    catch (const MujinVisionException& e) {
        std::stringstream errss;
        errss << "Caught MujinVisionException " << e.message() << " in _SendExecutionVerificationPointCloudThread!";
        _SetStatus(TT_SendExecutionVerificationPointCloud, MS_Aborted, errss.str(), "", false);
        VISIONMANAGER_LOG_ERROR(errss.str());
    }
    catch (const UserInterruptException& ex) {
        std::stringstream errss;
        errss << "User interrupted _SendExecutionVerificationPointCloudThread!";
        _SetStatus(TT_SendExecutionVerificationPointCloud, MS_Preempted, errss.str(), "", false);
        VISIONMANAGER_LOG_WARN(errss.str());
    }
    catch(const std::exception& ex) {
        std::stringstream errss;
        errss << "Caught exception in _SendExecutionVerificationPointCloudThread " << ex.what();
        _SetStatus(TT_SendExecutionVerificationPointCloud, MS_Preempted, errss.str(), "", false);
        VISIONMANAGER_LOG_WARN(errss.str());
    }
    catch (...) {
        std::stringstream errss;
        errss << "Caught unknown exception in VisualizePointcloudThread!";
        _SetStatus(TT_SendExecutionVerificationPointCloud, MS_Aborted, errss.str(), "", false);
        VISIONMANAGER_LOG_ERROR(errss.str());
    }
}

void MujinVisionManager::_ControllerMonitorThread(const unsigned int waitinterval, const std::string& locale)
{
    try {
        BinPickingTaskResourcePtr pBinpickingTask = _pSceneResource->GetOrCreateBinPickingTaskFromName_UTF8(_tasktype+std::string("task1"), _tasktype, TRO_EnableZMQ);
        std::string userinfo_json = "{\"username\": " + ParametersBase::GetJsonString(_pControllerClient->GetUserName()) + ", \"locale\": " + ParametersBase::GetJsonString(_locale) + "}";

        ParametersBase::ValidateJsonString(userinfo_json);

        pBinpickingTask->Initialize(_defaultTaskParameters, _binpickingTaskZmqPort, _binpickingTaskHeartbeatPort, _zmqcontext, false, _binpickingTaskHeartbeatTimeout, _controllerCommandTimeout, userinfo_json, _slaverequestid);

        BinPickingTaskResource::ResultGetBinpickingState binpickingstate;
        uint64_t lastwarnedtimestamp = 0;
        while (!_bStopControllerMonitorThread) {
            uint64_t lastUpdateTimestamp;
            {
                boost::mutex::scoped_lock lock(_mutexControllerBinpickingState);
                try {
                    pBinpickingTask->GetPublishedTaskState(binpickingstate, "m", 1.0);
                }
                catch(const std::exception& ex) {
                    if (GetMilliTime() - lastwarnedtimestamp > 1000.0) {
                        lastwarnedtimestamp = GetMilliTime();
                        std::stringstream ss;
                        ss << "Failed to get published task state from mujin controller: " << ex.what() << ".";
                        std::string errstr = ss.str();
                        boost::replace_all(errstr, "\"", ""); // need to remove " in the message so that json parser works
                        boost::replace_all(errstr, "\\", ""); // need to remove \ in the message so that json parser works
                        _SetStatusMessage(TT_ControllerMonitor, errstr, GetErrorCodeString(MVE_ControllerError));
                        VISIONMANAGER_LOG_WARN(errstr);
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
        VISIONMANAGER_LOG_ERROR(errss.str());
    }
    catch (const MujinVisionException& e) {
        std::stringstream errss;
        errss << "Caught MujinVisionException " << e.message() << " in _ControllerMonitorThread!";
        _SetStatus(TT_ControllerMonitor, MS_Aborted, errss.str(), "", false);
        VISIONMANAGER_LOG_ERROR(errss.str());
    }
    catch (const UserInterruptException& ex) {
        std::stringstream errss;
        errss << "User interrupted _ControllerMonitorThread!";
        _SetStatus(TT_ControllerMonitor, MS_Preempted, errss.str(), "", false);
        VISIONMANAGER_LOG_WARN(errss.str());
    }
    catch(const std::exception& ex) {
        std::stringstream errss;
        errss << "Caught exception in _ControllerMonitorThread " << ex.what();
        _SetStatus(TT_ControllerMonitor, MS_Preempted, errss.str(), "", false);
        VISIONMANAGER_LOG_WARN(errss.str());
    }
    catch (...) {
        std::stringstream errss;
        errss << "Caught unknown exception in VisualizePointcloudThread!";
        _SetStatus(TT_ControllerMonitor, MS_Aborted, errss.str(), "", false);
        VISIONMANAGER_LOG_ERROR(errss.str());
    }
}

void MujinVisionManager::_VisualizePointCloudThread(VisualizePointcloudThreadParams params, ImagesubscriberHandlerPtr& ihraw, boost::condition& condrunningthread)
{
    ImagesubscriberHandlerPtr ih = ihraw;
    {
        // notify creator that the handle was copied
        boost::mutex::scoped_lock lock(_mutexThreadResourceSync);
        condrunningthread.notify_all();
    }

    try {
        FalseSetter turnOffVisualize(_bIsVisualizePointcloudRunning);
        std::string regionname = params.regionname;
        std::vector<std::string> cameranames = params.cameranames;
        double pointsize = params.pointsize;
        bool ignoreocclusion = params.ignoreocclusion;
        unsigned int maxage = params.maxage;
        unsigned int fetchimagetimeout = params.fetchimagetimeout;
        bool request = params.request;
        double voxelsize = params.voxelsize;

        while (!_bStopVisualizePointCloudThread) {
            SyncCameras(regionname, cameranames);
            if (_bStopVisualizePointCloudThread) {
                break;
            }
            VisualizePointCloudOnController(regionname, cameranames, pointsize, ignoreocclusion, maxage, fetchimagetimeout, request, voxelsize);
        }
    }
    catch (const zmq::error_t& e) {
        std::stringstream errss;
        errss << "Caught zmq exception errornum=" << e.num() << " in _VisualizePointCloudThread!";
        _SetStatus(TT_VisualizePointCloud, MS_Aborted, errss.str(), "", false);
        VISIONMANAGER_LOG_ERROR(errss.str());
    }
    catch (const MujinVisionException& e) {
        std::stringstream errss;
        errss << "Caught MujinVisionException " << e.message() << " in _VisualizePointCloudThread!";
        _SetStatus(TT_VisualizePointCloud, MS_Aborted, errss.str(), "", false);
        VISIONMANAGER_LOG_ERROR(errss.str());
    }
    catch (const UserInterruptException& ex) {
        std::stringstream errss;
        errss << "User interrupted _VisualizePointCloudThread!";
        _SetStatus(TT_VisualizePointCloud, MS_Preempted, errss.str(), "", false);
        VISIONMANAGER_LOG_WARN(errss.str());
    }
    catch(const std::exception& ex) {
        std::stringstream errss;
        errss << "Caught exception in _VisualizePointCloudThread " << ex.what();
        _SetStatus(TT_VisualizePointCloud, MS_Preempted, errss.str(), "", false);
        VISIONMANAGER_LOG_WARN(errss.str());
    }
    catch (...) {
        std::stringstream errss;
        errss << "Caught unknown exception in VisualizePointcloudThread!";
        _SetStatus(TT_VisualizePointCloud, MS_Aborted, errss.str(), "", false);
        VISIONMANAGER_LOG_ERROR(errss.str());
    }
}

mujinvision::Transform MujinVisionManager::_GetTransform(const std::string& instobjname)
{
    mujinclient::Transform t;
    _pBinpickingTask->GetTransform(instobjname,t,"m");
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
    utils::GetSensorTransform(_pControllerClient, _pSceneResource, camerabodyname, sensorname, O_T_C0, "m");
    Transform O_T_C = _GetTransform(O_T_C0); // sensor transform in world frame
    _mNameCamera[cameraname]->SetWorldTransform(O_T_C);
    VISIONMANAGER_LOG_DEBUG("setting camera transform to:\n" + _GetString(_mNameCamera[cameraname]->GetWorldTransform()));
}

void MujinVisionManager::_SyncCamera(const std::string& cameraname, const mujinclient::Transform& t)
{
    if (_mNameCamera.find(cameraname) == _mNameCamera.end()) {
        throw MujinVisionException("Camera "+cameraname+ " is unknown!", MVE_InvalidArgument);
    }
    Transform O_T_C = _GetTransform(t); // sensor transform in world frame
    _mNameCamera[cameraname]->SetWorldTransform(O_T_C);
    VISIONMANAGER_LOG_DEBUG("setting camera transform to:\n" + _GetString(_mNameCamera[cameraname]->GetWorldTransform()));
}

void MujinVisionManager::_SyncRegion(const std::string& regionname)
{
    if (_mNameRegion.find(regionname) == _mNameRegion.end()) {
        throw MujinVisionException("Region "+regionname+ " is unknown!", MVE_InvalidArgument);
    }
    mujinvision::Transform regiontransform = _GetTransform(regionname);
    _SyncRegion(regionname, regiontransform);
}


void MujinVisionManager::_SyncRegion(const std::string& regionname, const mujinclient::Transform& t)
{
    if (_mNameRegion.find(regionname) == _mNameRegion.end()) {
        throw MujinVisionException("Region "+regionname+ " is unknown!", MVE_InvalidArgument);
    }
    mujinvision::Transform regiontransform = _GetTransform(t);
    _SyncRegion(regionname, regiontransform);
}

void MujinVisionManager::_SyncRegion(const std::string& regionname, const mujinvision::Transform& regiontransform)
{
    _mNameRegion[regionname]->SetWorldTransform(regiontransform);
    VISIONMANAGER_LOG_DEBUG("setting region transform to:\n" + _GetString(_mNameRegion[regionname]->GetWorldTransform()));
    // update globalroi3d from mujin controller
    VISIONMANAGER_LOG_DEBUG("Computing globalroi3d from mujin controller.");
    // get axis aligned bounding box for region
    BinPickingTaskResource::ResultOBB robbe;
    _pBinpickingTask->GetOBB(robbe, regionname, "base", "m");
    _mNameRegion[regionname]->pRegionParameters->outerTranslation = robbe.translation;
    _mNameRegion[regionname]->pRegionParameters->outerExtents = robbe.extents;
    _mNameRegion[regionname]->pRegionParameters->outerRotationmat = robbe.rotationmat;
    // get inner obb from mujin controller
    VISIONMANAGER_LOG_DEBUG("getting obb from mujin controller.");
    BinPickingTaskResource::ResultOBB robb;
    _pBinpickingTask->GetInnerEmptyRegionOBB(robb, regionname, "base", "m");
    _mNameRegion[regionname]->pRegionParameters->innerTranslation = robb.translation;
    _mNameRegion[regionname]->pRegionParameters->innerExtents = robb.extents;
    _mNameRegion[regionname]->pRegionParameters->innerRotationmat = robb.rotationmat;
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

void MujinVisionManager::_GetImages(ThreadType tt, BinPickingTaskResourcePtr pBinpickingTask, const std::string& regionname, const std::vector<std::string>& colorcameranames, const std::vector<std::string>& depthcameranames, std::vector<ImagePtr>& resultcolorimages, std::vector<ImagePtr>& resultdepthimages, std::vector<ImagePtr>& resultresultimages, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request, const bool useold, const unsigned int waitinterval)
{
    if (useold && _lastcolorimages.size() == colorcameranames.size() && _lastdepthimages.size() == depthcameranames.size()) {
        VISIONMANAGER_LOG_INFO("using last images");
        resultcolorimages = _lastcolorimages;
        resultdepthimages = _lastdepthimages;
        if (_lastresultimages.size() > 0) {
            resultresultimages = _lastresultimages;
        }
        return;
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
    unsigned long long starttime = 0, endtime = 0;
    std::string cameraname;
    uint64_t lastimageagecheckfailurets = 0;
    uint64_t lastfirstimagecheckfailurewarnts = 0;
    uint64_t lastocclusioncheckfailurewarnts = 0;
    uint64_t lastocclusionwarnts = 0;
    uint64_t lastcouldnotcapturewarnts = 0;
    bool usecache = !request;

    while (!_bCancelCommand && // command is not being canceled
           !_bShutdown &&  // visionmanager is not being shutdown
           ((fetchimagetimeout == 0) || (fetchimagetimeout > 0 && GetMilliTime() - start0 < fetchimagetimeout)) && // not timed out yet
           ((tt != TT_Detector) || (tt == TT_Detector && !_bStopDetectionThread)) // detection thread is not being stopped if called from it
           ) {

        // get images from subscriber
        if (usecache) {
            _pImagesubscriberManager->GetImagePackFromBuffer(colorcameranames, depthcameranames, colorimages, depthimages, resultimages, starttime, endtime, fetchimagetimeout / 1000.0);
        } else {
            VISIONMANAGER_LOG_WARN("snapping is not supported, using cache");
            _pImagesubscriberManager->GetImagePackFromBuffer(colorcameranames, depthcameranames, colorimages, depthimages, resultimages, starttime, endtime, fetchimagetimeout / 1000.0);
        }

        // if called by detection thread, break if it is being stopped
        if (tt == TT_Detector && _bStopDetectionThread) {
            break;
        }

        // ensure streamer and try to get images again if got fewer than expected images
        if (colorimages.size() < colorcameranames.size() || depthimages.size() < depthcameranames.size()) {
            if (GetMilliTime() - lastcouldnotcapturewarnts > 1000.0) {
                std::stringstream msg_ss;
                msg_ss << "Could not get all images, ensure capturing thread, will try again"
                       << ": # color " << colorimages.size() << "," << colorcameranames.size()
                       << ", # depth " << depthimages.size() << "," << depthcameranames.size()
                       << ", # result images = " << resultimages.size()
                       << ", use_cache = " << usecache;
                VISIONMANAGER_LOG_WARN(msg_ss.str());
                lastcouldnotcapturewarnts = GetMilliTime();
            }
            //_pImagesubscriberManager->StartCaptureThread(_GetHardwareIds(cameranames));

            boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
            colorimages.clear();
            depthimages.clear();
            continue;
        }

        // throw exception if acquired images are from the future
        if (GetMilliTime()  < starttime || GetMilliTime() < endtime) {
            std::stringstream msg_ss;
            msg_ss << "Image is acquired from the future, please ensure that clocks are synchronized, starttime=" << starttime << " endtime=" << endtime << ", use_cache = " << usecache;
            VISIONMANAGER_LOG_ERROR(msg_ss.str());
            colorimages.clear();
            depthimages.clear();
            resultimages.clear();
            throw MujinVisionException(msg_ss.str(), MVE_ImageAcquisitionError);
        }

        // ensure streamer and try to get images again if images are too old
        if (maxage>0 && GetMilliTime()-starttime>maxage) {
            if (GetMilliTime() - lastimageagecheckfailurets > 1000.0) {
                std::stringstream msg_ss;
                msg_ss << "Image is more than " << maxage << " ms old (" << (GetMilliTime() - starttime) << "), will try to get again"
                       << ", use_cache = " << usecache;
                VISIONMANAGER_LOG_WARN(msg_ss.str());
                lastimageagecheckfailurets = GetMilliTime();
                VISIONMANAGER_LOG_DEBUG("start image capturing, in case streamer was reset");
            }
            //_pImagesubscriberManager->StartCaptureThread(_GetHardwareIds(cameranames));

            colorimages.clear();
            depthimages.clear();
            resultimages.clear();
            continue;
        }

        // skip images and try to get images again if images are taken before detection loop started
        if (!request && _tsStartDetection > 0 && starttime < _tsStartDetection) {
            if (lastfirstimagecheckfailurewarnts != starttime) {
                lastfirstimagecheckfailurewarnts = starttime;
                std::stringstream msg_ss;
                msg_ss << "Image was taken " << (_tsStartDetection - starttime) << " ms before _tsStartDetection " << _tsStartDetection << ", will try to get again"
                       << ", use_cache = " << usecache;
                VISIONMANAGER_LOG_WARN(msg_ss.str());
            }
            colorimages.clear();
            depthimages.clear();
            resultimages.clear();
            continue;
        }

        // skip images and try to get them again if failed to check for occlusion
        bool isoccluding = false;
        if (!ignoreocclusion && regionname != "") {
            try {
                std::vector<std::string> checkedcameranames;
                for (size_t i=0; i<colorcameranames.size() && !isoccluding; ++i) {
                    std::string cameraname = colorcameranames.at(i);
                    //uint64_t time0 = GetMilliTime();
                    //std::stringstream ss;
                    pBinpickingTask->IsRobotOccludingBody(regionname, cameraname, starttime, endtime, isoccluding);
                    checkedcameranames.push_back(cameraname);
                    //ss << "_pBinpickingTask->IsRobotOccludingBody() took " << (GetMilliTime() - time0) / 1000.0f << " secs";
                    //VISIONMANAGER_LOG_DEBUG(ss.str());
                }
                for (size_t i=0; i < depthcameranames.size() && !isoccluding; ++i) {
                    std::string cameraname = depthcameranames.at(i);
                    if (std::find(checkedcameranames.begin(), checkedcameranames.end(), cameraname) == checkedcameranames.end()) {
                        pBinpickingTask->IsRobotOccludingBody(regionname, cameraname, starttime, endtime, isoccluding);
                        checkedcameranames.push_back(cameraname);
                    }
                }
            } catch (...) {
                if (GetMilliTime() - lastocclusioncheckfailurewarnts > 1000.0) {
                    lastocclusioncheckfailurewarnts = GetMilliTime();
                    std::stringstream ss;
                    ss << "Failed to check for occluded, will try again";
                    VISIONMANAGER_LOG_WARN(ss.str());
                    _SetStatusMessage(tt, "", ss.str());
                }
                boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                colorimages.clear();
                depthimages.clear();
                resultimages.clear();
                continue;
            }
        }
        // skip images if there was occlusion
        if (isoccluding) {
            if (GetMilliTime() - lastocclusionwarnts > 1000.0) {
                lastocclusionwarnts = GetMilliTime();
                std::stringstream msg_ss;
                msg_ss << "Region is occluded in the view of camera, will try again"
                       << " starttime " << starttime
                       << " endtime " << endtime
                       << ", use_cache = " << usecache;
                VISIONMANAGER_LOG_WARN(msg_ss.str());
            }
            colorimages.clear();
            depthimages.clear();
            resultimages.clear();
            continue;
        } else {
            std::stringstream ss;
            ss << "got good imagepack. starttime=" << starttime << " endtime=" << endtime << " total=" << (endtime-starttime)/1000.0f << " " << (GetMilliTime()-starttime) / 1000.0f << " secs old";
            VISIONMANAGER_LOG_DEBUG(ss.str());
            break;
        }
    }
    if (!(!_bCancelCommand && // canceled?
          !_bShutdown &&  // shutdown?
          ((fetchimagetimeout == 0) || (fetchimagetimeout > 0 && GetMilliTime() - start0 < fetchimagetimeout)) && // timeed out?
          (tt != TT_Detector || (tt == TT_Detector && !_bStopDetectionThread)) // canceled detection loop?
          )) {
        std::stringstream ss;
        ss << "do not use images because we got out of while loop unexpectedly: " << " _bCancelCommand=" << _bCancelCommand << " _bShutdown=" << _bShutdown << " " << GetMilliTime() - start0 << ">" << fetchimagetimeout;
        if (tt == TT_Detector) {
            ss << " _bStopDetectionThread=" << _bStopDetectionThread;
        }
        VISIONMANAGER_LOG_DEBUG(ss.str());
    } else {
        resultcolorimages = colorimages;
        resultdepthimages = depthimages;
        resultresultimages = resultimages;
        _lastcolorimages = colorimages;
        _lastdepthimages = depthimages;
        _lastresultimages = resultimages;
    }
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
    const std::string& targetdetectionarchiveurl
    )
{
    _locale = locale;
    uint64_t time0 = GetMilliTime();
    uint64_t starttime = GetMilliTime();
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

    std::stringstream url_ss;
    url_ss << "http://" << controllerIp << ":" << controllerPort;
    ControllerClientPtr controller = CreateControllerClient(controllerUsernamePass, url_ss.str());
    _pControllerClient = controller;

    std::string detectorconfigfilename;
    std::string detectionpath;
    std::string modelfilename;
    std::string modelurl;

    detectionpath = _detectiondir + "/" + targetname;

    // fetch or update modelfile
    if (targeturi != "") {
        modelfilename = targeturi.substr(sizeof("mujin:/")-1, std::string::npos);
        modelurl = "http://" + controllerUsernamePass + "@" + controllerIp + "/u/" + _pControllerClient->GetUserName() + "/" + modelfilename;
        VISIONMANAGER_LOG_DEBUG("updating " + modelfilename + " from " + modelurl);
        try {
            std::string cmdstr = "wget --quiet --timestamping --timeout=0.5 --tries=1 " + modelurl + " -P " + _detectiondir;
            system(cmdstr.c_str()); // TODO: check process exit code
        } catch (...) {
            std::stringstream errss;
            errss << "Failed to fetch model file from controller.";
            VISIONMANAGER_LOG_ERROR(errss.str());
            throw MujinVisionException(errss.str(), MVE_Failed);
        }
    }

    // update target archive if needed
    if (targetdetectionarchiveurl != "") {
        // fetch archive
        try {
            std::string cmdstr = "wget --quiet --timestamping --timeout=0.5 --tries=1 " + targetdetectionarchiveurl + " -P " + detectionpath;
            system(cmdstr.c_str()); // TODO: check process exit code here
        } catch (...) {
            std::stringstream errss;
            errss << "Failed to prepare config files because " << targetdetectionarchiveurl << " could not be fetched.";
            VISIONMANAGER_LOG_ERROR(errss.str());
            throw MujinVisionException(errss.str(), MVE_Failed);
        }

        // extract files
        std::string archivefilename = detectionpath + "/" + targetname + ".tar.gz";
        try {
            std::stringstream commandss;
            commandss << "tar xzf " << archivefilename << " -C " << detectionpath;
            system(commandss.str().c_str()); // TODO: check process exit code here
        } catch (...) {
            std::stringstream errss;
            errss << "Failed to prepare config files because " << archivefilename << " could not be decompressed.";
            VISIONMANAGER_LOG_ERROR(errss.str());
            throw MujinVisionException(errss.str(), MVE_Failed);
        }
    }

    // prepare config files
    detectorconfigfilename = detectionpath + "/detector.json";
    if (boost::filesystem::exists(detectorconfigfilename)) {
        VISIONMANAGER_LOG_INFO("getting detector config file");
        VISIONMANAGER_LOG_DEBUG("using detectionpath " + detectionpath + " as path to detectorconfig, ignoring detectorconfigname");
        _mDetectorExtraInitializationOptions["templateDir"] = detectionpath;
    } else {
        detectorconfigfilename = "";
    }

    // load visionserver configuration
    std::stringstream visionmanagerconfigss;
    visionmanagerconfigss << visionmanagerconfig;

    // read execution verification configuration
    ptree visionserverpt;
    read_json(visionmanagerconfigss, visionserverpt);
    _filteringvoxelsize = visionserverpt.get<double>("filteringvoxelsize");
    _filteringstddev = visionserverpt.get<double>("filteringstddev");
    _filteringnumnn = visionserverpt.get<int>("filteringnumnn");

    std::string detectormodulename = visionserverpt.get<std::string>("modulename", "");
    std::string detectorclassname = visionserverpt.get<std::string>("classname", "");
    if (detectormodulename.size() > 0 && detectorclassname.size() > 0) {
        _mDetectorExtraInitializationOptions["modulename"] = detectormodulename;
        _mDetectorExtraInitializationOptions["classname"] = detectorclassname;
    }

    // set up regions
    std::vector<std::string> regionnames;
    std::vector<RegionParametersPtr > vRegionParameters;
    RegionParametersPtr pRegionParameters;
    ptree containerpt;
    std::stringstream containerss;
    containerss << containerParameters;
    read_json(containerss, containerpt);
    FOREACH(v, containerpt.get_child("regions")) {
        RegionParametersPtr pregionparameters(new RegionParameters(v->second));
        vRegionParameters.push_back(pregionparameters);
        _mNameRegion[pregionparameters->instobjectname] = RegionPtr(new Region(pregionparameters));
        regionnames.push_back(pregionparameters->instobjectname);
    }

    // connect to mujin controller
    _userinfo_json = "{\"username\": " + ParametersBase::GetJsonString(_pControllerClient->GetUserName()) + ", \"locale\": " + ParametersBase::GetJsonString(locale) + "}";

    _SetStatusMessage(TT_Command, "Connected to mujin controller at " + url_ss.str());
    SceneResourcePtr scene(new SceneResource(controller,binpickingTaskScenePk));
    _pSceneResource = scene;
    _pBinpickingTask = scene->GetOrCreateBinPickingTaskFromName_UTF8(tasktype+std::string("task1"), tasktype, TRO_EnableZMQ);

    _SetStatusMessage(TT_Command, "Syncing cameras");
    std::vector<std::string> cameranames;
    scene->GetSensorMapping(_mCameraNameHardwareId);
    FOREACH(v, _mCameraNameHardwareId) {
        if (_mNameCameraParameters.find(v->first) != _mNameCameraParameters.end()) {
            _mNameCameraParameters[v->first]->id = v->second;
        } else {
            _mNameCameraParameters[v->first].reset(new CameraParameters(v->second));
        }
        _mNameCameraParameters[v->first]->isDepthCamera = v->first.find("_l_rectified") != std::string::npos;
        cameranames.push_back(v->first);
    }

    VISIONMANAGER_LOG_DEBUG("initialzing binpickingtask in Initialize() with userinfo " + _userinfo_json);

    ParametersBase::ValidateJsonString(_userinfo_json);

    _pBinpickingTask->Initialize(defaultTaskParameters, binpickingTaskZmqPort, binpickingTaskHeartbeatPort, _zmqcontext, false, _binpickingTaskHeartbeatTimeout, _controllerCommandTimeout, _userinfo_json, slaverequestid);

    _SetStatusMessage(TT_Command, "Syncing regions");
    BinPickingTaskResource::ResultGetInstObjectAndSensorInfo resultgetinstobjectandsensorinfo;
    starttime = GetMilliTime();
    _pBinpickingTask->GetInstObjectAndSensorInfo(regionnames, cameranames, resultgetinstobjectandsensorinfo, "m", controllertimeout);
    VISIONMANAGER_LOG_DEBUG("GetInstObjectAndSensorInfo() took: " + boost::lexical_cast<std::string>((GetMilliTime() - starttime)/1000.0f) + " secs");

    // sync regions
    starttime = GetMilliTime();
    _SetStatusMessage(TT_Command, "Syncing regions.");
    FOREACH(it, resultgetinstobjectandsensorinfo.minstobjecttransform) {
        _SyncRegion(it->first, it->second);
    }
    VISIONMANAGER_LOG_DEBUG("sync regions took: " + boost::lexical_cast<std::string>((GetMilliTime() - starttime)/1000.0f) + " secs");

    // set up cameras
    starttime = GetMilliTime();
    _SetStatusMessage(TT_Command, "Setting up cameras.");
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
    }
    std::vector<std::string> syncedcamera;
    FOREACH(itr, _mNameRegion) {
        std::string regionname = itr->first;
        std::map<std::string, CameraPtr> mNameColorCamera, mNameDepthCamera;
        RegionPtr region = _mNameRegion[regionname];
        std::string cameraname;
        CameraParametersPtr pcameraparameters;
        for (unsigned int i=0; i<region->pRegionParameters->cameranames.size(); ++i) {
            cameraname = region->pRegionParameters->cameranames.at(i);
            pcameraparameters = _mNameCamera[cameraname]->pCameraParameters;
            if (std::find(syncedcamera.begin(), syncedcamera.end(), cameraname) == syncedcamera.end()) {
                _SyncCamera(cameraname, resultgetinstobjectandsensorinfo.msensortransform[cameraname]);
                syncedcamera.push_back(cameraname);
            } else {
                throw MujinVisionException("does not support same camera mapped to more than one region.", MVE_CommandNotSupported);
            }
            if (pcameraparameters->isColorCamera) {
                _SetStatusMessage(TT_Command, "Loading parameters for color camera " + cameraname +" for region " + regionname +".");
                mNameColorCamera[cameraname] = _mNameCamera[cameraname];
            }
            if (pcameraparameters->isDepthCamera) {
                _SetStatusMessage(TT_Command, "Loading parameters for depth camera " + cameraname +" for region " + regionname +".");
                mNameDepthCamera[cameraname] = _mNameCamera[cameraname];
            }
            _mCameranameRegionname[cameraname] = regionname;
        }
        _mRegionColorCameraMap[regionname] = mNameColorCamera;
        _mRegionDepthCameraMap[regionname] = mNameDepthCamera;
    }
    VISIONMANAGER_LOG_DEBUG("sync cameras took: " + boost::lexical_cast<std::string>((GetMilliTime() - starttime)/1000.0f) + " secs");

    // set up subscribers
    _SetStatusMessage(TT_Command, "Loading subscriber configuration.");
    // load subscriber configuration
    _imagesubscriberconfig = imagesubscriberconfig;
    std::stringstream imagesubscriberconfigss;
    imagesubscriberconfigss << imagesubscriberconfig;
    ptree imagesubscriberpt;
    read_json(imagesubscriberconfigss, imagesubscriberpt);

    // set up image subscriber manager
    _SetStatusMessage(TT_Command, "Setting up image manager.");
    _pImagesubscriberManager->Initialize(_mNameCamera, streamerIp, streamerPort, imagesubscriberpt.get_child("zmq_subscriber"), _zmqcontext);

    // set up detectors
    starttime = GetMilliTime();
    _SetStatusMessage(TT_Command, "Setting up detector.");
    std::string detectorconfig;
    if (detectorconfigfilename.size() == 0) {
        detectorconfigfilename = _GetConfigFileName("detector", detectorconfigname);
    }
    _LoadConfig(detectorconfigfilename, detectorconfig);

    // append additional params to detectorconf string
    bool debug = visionserverpt.get<bool>("debug", false);
    double cleanSize = visionserverpt.get<double>("cleanSize", 0.007);
    size_t index = detectorconfig.find_last_of("}");
    if (index == std::string::npos) {
        throw MujinVisionException("invalid detectorconfig: " + detectorconfig, MVE_InvalidArgument);
    }
    _detectorconfig = detectorconfig.substr(0, index) + ", " + ParametersBase::GetJsonString("debug", debug) + ", " + ParametersBase::GetJsonString("cleanSize", cleanSize) + ", " + ParametersBase::GetJsonString("modelFilename", modelfilename) + "}";
    ParametersBase::ValidateJsonString(_detectorconfig);
    _targetname = targetname;
    _targeturi = targeturi;
    _targetupdatename = targetupdatename;
    _pDetector = _pDetectorManager->CreateObjectDetector(_detectorconfig, _targetname, _mNameRegion, _mRegionColorCameraMap, _mRegionDepthCameraMap, boost::bind(&MujinVisionManager::_SetDetectorStatusMessage, this, _1, _2), _mDetectorExtraInitializationOptions);
    VISIONMANAGER_LOG_DEBUG("detector initialization took: " + boost::lexical_cast<std::string>((GetMilliTime() - starttime)/1000.0f) + " secs");
    VISIONMANAGER_LOG_DEBUG("Initialize() took: " + boost::lexical_cast<std::string>((GetMilliTime() - time0)/1000.0f) + " secs");
    VISIONMANAGER_LOG_DEBUG(" ------------------------");

    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::_DeInitialize()
{
    _bStopSendPointCloudObstacleToControllerThread = true;
    if (!!_pSendPointCloudObstacleThread) {
        _pSendPointCloudObstacleThread->join();
        _pSendPointCloudObstacleThread.reset();
        _SetStatusMessage(TT_Command, "Stopped sendpointcloudobstacle thread.");
    }
    _StopDetectionThread();

    std::string regionname;
    if (!!_pDetector) {
        _pDetector.reset();
        VISIONMANAGER_LOG_DEBUG("reset detector");
    }
    if (!!_pImagesubscriberManager) {
        _pImagesubscriberManager->DeInitialize(); // do not reset because it is created and passed in from outside
        VISIONMANAGER_LOG_DEBUG("reset imagesubscribermanager");
    }
    _SetStatusMessage(TT_Command, "DeInitialized vision manager.");
}

void MujinVisionManager::DetectObjects(const std::string& regionname, const std::vector<std::string>&cameranames, std::vector<DetectedObjectPtr>& detectedobjects, std::string& resultstate, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool fastdetection, const bool bindetection, const bool request, const bool useold)
{
    if (!_pImagesubscriberManager) {
        throw MujinVisionException("image subscriber manager is not initialzied", MVE_Failed);
    }
    _pImagesubscriberManager->StartCaptureThread(_GetHardwareIds(cameranames));
    _DetectObjects(TT_Command, _pBinpickingTask, regionname, cameranames, detectedobjects, resultstate, ignoreocclusion, maxage, fetchimagetimeout, fastdetection, bindetection, request, useold);
}

void MujinVisionManager::_DetectObjects(ThreadType tt, BinPickingTaskResourcePtr pBinpickingTask, const std::string& regionname, const std::vector<std::string>&cameranames, std::vector<DetectedObjectPtr>& detectedobjects, std::string& resultstate, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool fastdetection, const bool bindetection, const bool request, const bool useold, const bool checkcontaineremptyonly)
{
    boost::mutex::scoped_lock lock(_mutexDetector);
    uint64_t starttime = GetMilliTime();

    std::vector<std::string> colorcameranames = _GetColorCameraNames(regionname, cameranames);
    std::vector<std::string> depthcameranames = _GetDepthCameraNames(regionname, cameranames);

    // set up images
    std::vector<ImagePtr> colorimages, depthimages, resultimages;
    _GetImages(tt, pBinpickingTask, regionname, colorcameranames, depthcameranames, colorimages, depthimages, resultimages, ignoreocclusion, maxage, fetchimagetimeout, request, useold);
    VISIONMANAGER_LOG_INFO("Getting images took " + boost::lexical_cast<std::string>((GetMilliTime() - starttime) / 1000.0f));
    starttime = GetMilliTime();
    if (colorimages.size() == colorcameranames.size() && depthimages.size() == depthcameranames.size()) {
        for (size_t i=0; i<colorimages.size(); ++i) {
            _pDetector->SetColorImage(colorimages.at(i));
        }
        for (size_t i=0; i<depthimages.size(); ++i) {
            _pDetector->SetDepthImage(depthimages.at(i));
        }
        // detect objects
        if (resultimages.size() > 0) {
            _pDetector->DetectObjects(regionname, colorcameranames, depthcameranames, resultimages, detectedobjects, resultstate, fastdetection, bindetection, checkcontaineremptyonly);
        } else {
            _pDetector->DetectObjects(regionname, colorcameranames, depthcameranames, detectedobjects, resultstate, fastdetection, bindetection, checkcontaineremptyonly);
        }
    }
    if (resultstate == "") {
        resultstate = "null";
    }
    std::stringstream msgss;
    msgss << "Detected " << detectedobjects.size() << " objects, state: " << resultstate <<". Took " << (GetMilliTime()-starttime)/1000.0f << " seconds.";
    _SetStatusMessage(tt, msgss.str());
    _SetStatus(tt, MS_Succeeded);
}

void MujinVisionManager::StartDetectionLoop(const std::string& regionname, const std::vector<std::string>& cameranames, const std::vector<std::string>& evcamnames, const Transform& worldresultoffsettransform, const double voxelsize, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const std::string& obstaclename, const unsigned long long& starttime, const std::string& locale, const unsigned int maxnumfastdetection, const unsigned int maxnumdetection, const bool sendVerificationPointCloud, const bool stopOnLeftInOrder)
{
    if (!_pImagesubscriberManager) {
        throw MujinVisionException("image subscriber manager is not initialzied", MVE_Failed);
    }
    {
        boost::mutex::scoped_lock lock(_mutexDetectedInfo);
        // clear cached detected objects
        _bDetectedObjectsValid = false;
        _vDetectedObject.resize(0);
        _resultState = "{}";
    }
    // do not start capturing here, let the detection thread and env update thread handle this
    _bSendVerificationPointCloud = sendVerificationPointCloud;
    _tWorldResultOffset = worldresultoffsettransform;
    _vCameranames = cameranames;
    std::vector<std::string> ids = _GetHardwareIds(cameranames);
    if (_bSendVerificationPointCloud) {
        std::vector<std::string> ids1 = _GetHardwareIds(evcamnames);
        for (size_t i=0; i<ids1.size(); ++i) {
            if (std::find(ids.begin(), ids.end(), ids1[i]) == ids.end()) {
                ids.push_back(ids1[i]);
            }
        }
    }
    ImagesubscriberHandlerPtr ih(new ImagesubscriberHandler(_pImagesubscriberManager, ids));
    _StartDetectionThread(regionname, cameranames, voxelsize, pointsize, ignoreocclusion, maxage, fetchimagetimeout, starttime, maxnumfastdetection, maxnumdetection, stopOnLeftInOrder, ih);
    _StartUpdateEnvironmentThread(regionname, cameranames, voxelsize, pointsize, obstaclename, ih, 50, locale);
    if( _bSendVerificationPointCloud ) {
        _StartExecutionVerificationPointCloudThread(regionname, cameranames, evcamnames, voxelsize, pointsize, obstaclename, ih, 50, locale);
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
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::StartVisualizePointCloudThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request, const double voxelsize)
{
    if (!_pImagesubscriberManager) {
        throw MujinVisionException("image subscriber manager is not initialzied", MVE_Failed);
    }
    _vCameranames = cameranames;
    ImagesubscriberHandlerPtr ih(new ImagesubscriberHandler(_pImagesubscriberManager, _GetHardwareIds(cameranames)));
    _StartVisualizePointCloudThread(regionname, cameranames, ih, pointsize, ignoreocclusion, maxage, fetchimagetimeout, request, voxelsize);
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::StopVisualizePointCloudThread()
{
    _StopVisualizePointCloudThread();
    if (!!_pImagesubscriberManager) {
        _pImagesubscriberManager->StopCaptureThread(_GetHardwareIds(_vCameranames));
    }
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::SendPointCloudObstacleToController(const std::string& regionname, const std::vector<std::string>&cameranames, const std::vector<DetectedObjectPtr>& detectedobjectsworld, const unsigned int maxage, const unsigned int fetchimagetimeout, const double voxelsize, const double pointsize, const std::string& obstaclename, const bool fast, const bool request, const bool async, const std::string& locale)
{
    /*
       if (async) {
        if (!!_pSendPointCloudObstacleThread) {
            _pSendPointCloudObstacleThread->join();
            _pSendPointCloudObstacleThread.reset();
        }
       }
     */
    ImagesubscriberHandlerPtr ih(new ImagesubscriberHandler(_pImagesubscriberManager, _GetHardwareIds(cameranames)));
    _SendPointCloudObstacleToController(regionname, cameranames, detectedobjectsworld, ih, maxage, fetchimagetimeout, voxelsize, pointsize, obstaclename, fast, request, async, locale);
}

void MujinVisionManager::_SendPointCloudObstacleToController(const std::string& regionname, const std::vector<std::string>&cameranames, const std::vector<DetectedObjectPtr>& detectedobjectsworld, ImagesubscriberHandlerPtr ih, const unsigned int maxage, const unsigned int fetchimagetimeout, const double voxelsize, const double pointsize, const std::string& obstaclename, const bool fast, const bool request, const bool async, const std::string& locale)
{
    if (!_pImagesubscriberManager) {
        throw MujinVisionException("image subscriber manager is not initialzied", MVE_Failed);
    }
    uint64_t starttime = GetMilliTime();
    std::vector<ImagePtr> dummyimages;
    std::vector<std::string> dummycameranames;
    if (!async) {
        std::vector<std::string> depthcameranames = _GetDepthCameraNames(regionname, cameranames);
        // set up images
        std::vector<ImagePtr> depthimages;
        bool ignoreocclusion = true;
        _GetImages(TT_Command, _pBinpickingTask, regionname, dummycameranames, depthcameranames, dummyimages, depthimages, dummyimages, ignoreocclusion, maxage, fetchimagetimeout, request, false);
        if (depthimages.size() == depthcameranames.size()) {
            for (size_t i=0; i<depthimages.size(); ++i) {
                std::string cameraname = depthcameranames.at(i);
                CameraPtr camera= _mNameCamera[cameraname];
                // get point cloud obstacle
                std::vector<Real> points;
                {
                    boost::mutex::scoped_lock lock(_mutexDetector);
                    _pDetector->SetDepthImage(depthimages.at(i));
                    _pDetector->GetPointCloudObstacle(regionname, cameraname, detectedobjectsworld, points, voxelsize, fast, true, _filteringstddev, _filteringnumnn);
                }
                if (points.size() / 3 == 0) {
                    _SetStatusMessage(TT_Command, "got 0 point from GetPointCloudObstacle()");
                    int numretries = 3;
                    std::vector<std::string> depthcameranames1;
                    std::vector<ImagePtr> depthimages1;
                    depthcameranames1.push_back(cameraname);
                    while (numretries > 0 && points.size() / 3 == 0) {
                        points.clear();
                        _SetStatusMessage(TT_Command, "re-try getting depthimage and pointcloudobstacle");
                        _GetImages(TT_Command, _pBinpickingTask, regionname, dummycameranames, depthcameranames, dummyimages, depthimages, dummyimages, ignoreocclusion, maxage, fetchimagetimeout, request, false);
                        {
                            boost::mutex::scoped_lock lock(_mutexDetector);
                            _pDetector->SetDepthImage(depthimages1.at(0));
                            _pDetector->GetPointCloudObstacle(regionname, cameraname, detectedobjectsworld, points, voxelsize, fast, true, _filteringstddev, _filteringnumnn);
                        }
                        numretries--;
                    }
                    if (points.size() / 3 == 0) {
                        throw MujinVisionException("got 0 point from GetPointCloudObstacle() after retries", MVE_Failed);
                    }
                }
                std::stringstream ss;
                ss <<"Sending over " << (points.size()/3) << " points from " << cameraname << ".";
                _SetStatusMessage(TT_Command, ss.str());
                _pBinpickingTask->AddPointCloudObstacle(points, pointsize, obstaclename);
            }
        }
    }  else {
        _bStopSendPointCloudObstacleToControllerThread = false;
        _bIsSendPointcloudRunning = true;
        SendPointCloudObstacleToControllerThreadParams params;
        params.regionname = regionname;
        params.cameranames = cameranames;
        params.detectedobjectsworld = detectedobjectsworld;
        params.maxage = maxage;
        params.fetchimagetimeout = fetchimagetimeout;
        params.voxelsize = voxelsize;
        params.pointsize = pointsize;
        params.obstaclename = obstaclename;

        // need to pass in a reference of ih since we don't want _pSendPointCloudObstacleThread object to hold a reference to it, only the thread. However, this means we have to wait until the thread starts running before we resume. In order to achieve that, wait on a condition that will be signaled by the running thread.
        boost::condition condrunningthread;
        {
            boost::mutex::scoped_lock lock(_mutexThreadResourceSync);
            _pSendPointCloudObstacleThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_SendPointCloudObstacleToControllerThread, this, params, boost::ref(ih), boost::ref(condrunningthread))));
            condrunningthread.wait(lock);
        }
    }
    std::stringstream ss;
    ss << "SendPointCloudObstacleToController async " << int(async) << " took " << (GetMilliTime() - starttime) / 1000.0f << " secs";
    VISIONMANAGER_LOG_INFO(ss.str());
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::_SendPointCloudObstacleToControllerThread(SendPointCloudObstacleToControllerThreadParams params, ImagesubscriberHandlerPtr& ihraw, boost::condition& condrunningthread)
{
    ImagesubscriberHandlerPtr ih = ihraw;
    {
        // notify creator that the handle was copied
        boost::mutex::scoped_lock lock(_mutexThreadResourceSync);
        condrunningthread.notify_all();
    }
    FalseSetter turnoffstatusvar(_bIsSendPointcloudRunning);
    std::string regionname = params.regionname;
    std::vector<std::string> cameranames = params.cameranames;
    std::vector<DetectedObjectPtr> detectedobjectsworld = params.detectedobjectsworld;
    unsigned int maxage = params.maxage;
    unsigned int fetchimagetimeout = params.fetchimagetimeout;
    double voxelsize = params.voxelsize;
    double pointsize = params.pointsize;
    std::string obstaclename = params.obstaclename;

    if (!_pImagesubscriberManager) {
        throw MujinVisionException("image subscriber manager is not initialzied", MVE_Failed);
    }

    try {
        BinPickingTaskResourcePtr pBinpickingTask = _pSceneResource->GetOrCreateBinPickingTaskFromName_UTF8(_tasktype+std::string("task1"), _tasktype, TRO_EnableZMQ);
        std::string userinfo_json = "{\"username\": " + ParametersBase::GetJsonString(_pControllerClient->GetUserName()) + ", \"locale\": " + ParametersBase::GetJsonString(_locale) + "}";
        VISIONMANAGER_LOG_DEBUG("initialzing binpickingtask in _SendPointCloudObstacleToControllerThread with userinfo " + userinfo_json);

        ParametersBase::ValidateJsonString(userinfo_json);

        pBinpickingTask->Initialize(_defaultTaskParameters, _binpickingTaskZmqPort, _binpickingTaskHeartbeatPort, _zmqcontext, false, _binpickingTaskHeartbeatTimeout, _controllerCommandTimeout, userinfo_json, _slaverequestid);

        std::vector<std::string> depthcameranames = _GetDepthCameraNames(regionname, cameranames);
        // set up images
        std::vector<ImagePtr> depthimages;
        bool ignoreocclusion = true;
        std::vector<ImagePtr> dummyimages;
        std::vector<std::string> dummycameranames;
        //_GetDepthImages(TT_SendPointcloudObstacle, regionname, depthcameranames, depthimages, ignoreocclusion, maxage, fetchimagetimeout, true);
        _GetImages(TT_SendPointcloudObstacle, pBinpickingTask, regionname, dummycameranames, depthcameranames, dummyimages, depthimages, dummyimages, ignoreocclusion, maxage, fetchimagetimeout, true, false);
        VISIONMANAGER_LOG_DEBUG(str(boost::format("got images %d in SendPointCloudObstacleToControllerThread")%depthimages.size()));
        if (depthimages.size() == depthcameranames.size()) {
            for (size_t i=0; i<depthimages.size(); ++i) {
                std::string cameraname = depthcameranames.at(i);
                CameraPtr camera= _mNameCamera[cameraname];
                // get point cloud obstacle
                std::vector<Real> points;
                if (!!_pDetector) {
                    boost::mutex::scoped_lock lock(_mutexDetector);
                    _pDetector->SetDepthImage(depthimages.at(i));
                    _pDetector->GetPointCloudObstacle(regionname, cameraname, detectedobjectsworld, points, voxelsize, false, true, _filteringstddev, _filteringnumnn);
                } else {
                    VISIONMANAGER_LOG_WARN("detector is reset, stop async SendPointCloudObstacleToController call");
                    return;
                }
                if (points.size() / 3 == 0) {
                    _SetStatusMessage(TT_SendPointcloudObstacle, "got 0 point from GetPointCloudObstacle()");
                    int numretries = 3;
                    std::vector<std::string> depthcameranames1;
                    std::vector<ImagePtr> depthimages1;
                    depthcameranames1.push_back(cameraname);
                    while (numretries > 0 && points.size() / 3 == 0) {
                        points.clear();
                        _SetStatusMessage(TT_SendPointcloudObstacle, "re-try getting depthimage and pointcloudobstacle");
                        //_GetDepthImages(TT_SendPointcloudObstacle, regionname, depthcameranames, depthimages, ignoreocclusion, maxage, fetchimagetimeout, true);
                        _GetImages(TT_SendPointcloudObstacle, pBinpickingTask, regionname, dummycameranames, depthcameranames, dummyimages, depthimages, dummyimages, ignoreocclusion, maxage, fetchimagetimeout, true, false);
                        if (!!_pDetector) {
                            boost::mutex::scoped_lock lock(_mutexDetector);
                            _pDetector->SetDepthImage(depthimages1.at(0));
                            _pDetector->GetPointCloudObstacle(regionname, cameraname, detectedobjectsworld, points, voxelsize, false, true, _filteringstddev, _filteringnumnn);
                        } else {
                            VISIONMANAGER_LOG_WARN("detector is reset, stop async SendPointCloudObstacleToController call");
                            return;
                        }
                        numretries--;
                    }
                    if (points.size() / 3 == 0) {
                        throw MujinVisionException("got 0 point from GetPointCloudObstacle() after retries", MVE_Failed);
                    }
                }
                std::stringstream ss;
                ss <<"Sending over " << (points.size()/3) << " points from " << cameraname << ".";
                _SetStatusMessage(TT_SendPointcloudObstacle, ss.str());
                pBinpickingTask->AddPointCloudObstacle(points, pointsize, obstaclename);
            }
        } else {
            VISIONMANAGER_LOG_WARN("failed to get images in SendPointCloudObstacleToControllerThread");
        }
        _SetStatus(TT_SendPointcloudObstacle, MS_Succeeded);
    }
    catch(const std::exception& ex) {
        VISIONMANAGER_LOG_WARN(str(boost::format("threw exception while sending pointcloud: %s")%ex.what()));
    }
    VISIONMANAGER_LOG_DEBUG("end of SendPointCloudObstacleToControllerThread");
}

void MujinVisionManager::DetectRegionTransform(const std::string& regionname, const std::vector<std::string>& cameranames, mujinvision::Transform& regiontransform, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request)
{

    ImagesubscriberHandlerPtr ih(new ImagesubscriberHandler(_pImagesubscriberManager, _GetHardwareIds(cameranames)));
    _DetectRegionTransform(regionname, cameranames, regiontransform, ignoreocclusion, ih, maxage, fetchimagetimeout, request);
}

void MujinVisionManager::_DetectRegionTransform(const std::string& regionname, const std::vector<std::string>& cameranames, mujinvision::Transform& regiontransform, const bool ignoreocclusion, ImagesubscriberHandlerPtr ih, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request)
{
    if (!_pImagesubscriberManager) {
        throw MujinVisionException("image subscriber manager is not initialzied", MVE_Failed);
    }
    // TODO: use actual cameras
    std::vector<ImagePtr> colorimages;
    std::vector<std::string> ccamnames;
    ccamnames.push_back(_GetColorCameraNames(regionname, cameranames).at(0));
    //_GetColorImages(TT_Command, regionname, ccamnames, colorimages, ignoreocclusion, maxage, fetchimagetimeout, request);

    std::vector<ImagePtr> depthimages;
    std::vector<std::string> dcamnames;
    dcamnames.push_back(_GetDepthCameraNames(regionname, cameranames).at(0));
    //_GetDepthImages(TT_Command, regionname, dcamnames, depthimages, ignoreocclusion, maxage, fetchimagetimeout, request);
    std::vector<ImagePtr> dummyimages;
    std::vector<std::string> dummycameranames;
    _GetImages(TT_Command, _pBinpickingTask, regionname, ccamnames, dcamnames, colorimages, depthimages, dummyimages, ignoreocclusion, maxage, fetchimagetimeout, request, false);
    mujinvision::Transform regiontransform0 = regiontransform;
    {
        boost::mutex::scoped_lock lock(_mutexDetector);
        _pDetector->SetColorImage(colorimages.at(0));
        _pDetector->SetDepthImage(depthimages.at(0));
        _pDetector->DetectRegionTransform(regionname, ccamnames.at(0), dcamnames.at(0), regiontransform);
    }
    if (regiontransform.rot.x == regiontransform0.rot.x &&
        regiontransform.rot.y == regiontransform0.rot.y &&
        regiontransform.rot.z == regiontransform0.rot.z &&
        regiontransform.rot.w == regiontransform0.rot.w &&
        regiontransform.trans.x == regiontransform0.trans.x &&
        regiontransform.trans.y == regiontransform0.trans.y &&
        regiontransform.trans.z == regiontransform0.trans.z &&
        regiontransform.trans.w == regiontransform0.trans.w) {
        regiontransform = _GetTransform(regionname);
    }
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::VisualizePointCloudOnController(const std::string& regionname, const std::vector<std::string>& cameranames, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request, const double voxelsize)
{
    ImagesubscriberHandlerPtr ih(new ImagesubscriberHandler(_pImagesubscriberManager, _GetHardwareIds(cameranames)));
    _VisualizePointCloudOnController(regionname, cameranames, ih, pointsize, ignoreocclusion, maxage, fetchimagetimeout, request, voxelsize);
}

void MujinVisionManager::_VisualizePointCloudOnController(const std::string& regionname, const std::vector<std::string>&cameranames, ImagesubscriberHandlerPtr ih, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request, const double voxelsize)
{
    std::vector<std::string> cameranamestobeused;
    if (regionname != "") {
        cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);
    } else if (cameranames.size() > 0) {
        cameranamestobeused = cameranames;
    } else {
        throw MujinVisionException("neither region name nor camera names is specified, cannot sync cameras", MVE_InvalidArgument);
    }

    std::vector<std::vector<Real> > pointslist;
    std::vector<std::string> names;
    std::vector<double> points;
    std::vector<ImagePtr> dummyimages;
    std::vector<std::string> dummycameranames;
    for (unsigned int i=0; i<cameranamestobeused.size(); i++) {
        points.resize(0);
        std::string cameraname = cameranamestobeused.at(i);
        CameraPtr camera = _mNameCamera[cameraname];
        std::vector<ImagePtr> depthimages;
        std::vector<std::string> dcamnames;
        dcamnames.push_back(cameraname);

        _GetImages(TT_Command, _pBinpickingTask, "", dummycameranames, dcamnames, dummyimages, depthimages, dummyimages, ignoreocclusion, maxage, fetchimagetimeout, request, false);
        if (depthimages.size() == 0) {
            throw MujinVisionException("failed to get depth image for " + cameraname + ", cannot visualize point cloud", MVE_Failed);
        }
        {
            boost::mutex::scoped_lock lock(_mutexDetector);
            if (regionname == "") {
                _pDetector->GetCameraPointCloud(_mCameranameRegionname[cameranamestobeused[i]], cameranamestobeused[i], depthimages.at(0), points, voxelsize);
            } else {
                _pDetector->GetCameraPointCloud(regionname, cameranamestobeused[i], depthimages.at(0), points, voxelsize);
            }
        }
        if (points.size()>0) {
            pointslist.push_back(points);
            std::stringstream name_ss;
            name_ss << "__pointcloud_" << i;
            names.push_back(name_ss.str());
        }
    }
    _pBinpickingTask->VisualizePointCloud(pointslist, pointsize*1000.0f, names); // need to convert pointsize to millimeter
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::ClearVisualizationOnController()
{
    _pBinpickingTask->ClearVisualization();
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::SaveSnapshot(const std::string& regionname, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request)
{
    std::vector<std::string> cameranames;
    std::vector<std::string> cameranamestobeused = _GetCameraNames(regionname, cameranames);
    FOREACH(iter,_mRegionColorCameraMap[regionname]) {
        std::string colorcameraname = iter->first;
        std::string camerabodyname, sensorname;
        _ParseCameraName(colorcameraname, camerabodyname, sensorname);
        if (std::find(cameranamestobeused.begin(), cameranamestobeused.end(), colorcameraname) != cameranamestobeused.end()) {
            std::stringstream filename_ss;
            filename_ss << camerabodyname << "-" << sensorname << "-2d-" << GetMilliTime() << ".png";
            std::vector<std::string> ccamnames;
            ccamnames.push_back(colorcameraname);
            _pImagesubscriberManager->WriteColorImage(colorcameraname, filename_ss.str());
        }
    }
    FOREACH(iter,_mRegionDepthCameraMap[regionname]) {
        std::string depthcameraname = iter->first;
        std::string camerabodyname, sensorname;
        _ParseCameraName(depthcameraname, camerabodyname, sensorname);
        if (std::find(cameranamestobeused.begin(), cameranamestobeused.end(), depthcameraname) != cameranamestobeused.end()) {
            std::stringstream filename_ss;
            filename_ss << camerabodyname << "-" << sensorname << "-3d-" << GetMilliTime() << ".pcd";
            std::vector<ImagePtr> depthimages;
            std::vector<std::string> dcamnames;
            dcamnames.push_back(depthcameraname);
            _pImagesubscriberManager->WriteDepthImage(depthcameraname, filename_ss.str());
        }
    }
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
        _pBinpickingTask->UpdateObjects(detectedobjectsworld[0]->name, transformsworld, confidences, resultstate, "m");
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
        VISIONMANAGER_LOG_DEBUG("updating " + boost::lexical_cast<std::string>(cameranamestobeused[i]));
        _SyncCamera(cameranamestobeused[i]);
    }
    // update cameras in detector
    if (!!_pDetector) {
        _pDetector.reset();
        VISIONMANAGER_LOG_DEBUG("reset detector");
        _pDetector = _pDetectorManager->CreateObjectDetector(_detectorconfig, _targetname, _mNameRegion, _mRegionColorCameraMap, _mRegionDepthCameraMap, boost::bind(&MujinVisionManager::_SetDetectorStatusMessage, this, _1, _2), _mDetectorExtraInitializationOptions);
    }
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

void MujinVisionManager::GetLatestDetectedObjects(std::vector<DetectedObjectPtr>& detectedobjectsworld, std::string& resultstate, std::vector<Real>& points, const bool returnpoints)
{
    {
        boost::mutex::scoped_lock lock(_mutexDetectedInfo);
        detectedobjectsworld = _vDetectedObject;
        resultstate = _resultState;
        if (returnpoints) {
            points.clear();
            FOREACH(it, _mResultPoints) {
                points.insert(points.end(), it->second.begin(), it->second.end());
            }
        }
    }
    _SetStatus(TT_Command, MS_Succeeded);
}

std::string MujinVisionManager::_GetJsonString(const std::vector<DetectedObjectPtr>&detectedobjects)
{
    std::stringstream ss;
    ss << ParametersBase::GetJsonString("objects") << ": [";
    for (unsigned int i=0; i<detectedobjects.size(); i++) {
        ss << detectedobjects[i]->GetJsonString();
        if (i<detectedobjects.size()-1) {
            ss << ",";
        }
    }
    ss << "]";
    return ss.str();
}

std::vector<std::string> MujinVisionManager::_GetCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames)
{
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
        if(_mNameCameraParameters[cameraname]->isColorCamera) {
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
        if (_mNameCameraParameters[cameraname]->isDepthCamera) {
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
        }
    }
    return hardwareids;
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



} // namespace mujinvision
