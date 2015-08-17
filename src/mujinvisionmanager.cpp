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
#ifndef MUJIN_TIME
#define MUJIN_TIME
#include <time.h>

#ifndef _WIN32
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

namespace mujinvision {

void ParametersBase::Print()
{
    VISIONMANAGER_LOG_INFO(GetJsonString());
}

MujinVisionManager::MujinVisionManager(ImageSubscriberManagerPtr imagesubscribermanager, DetectorManagerPtr detectormanager, const unsigned int statusport, const unsigned int commandport, const unsigned configport, const std::string& configdir)
{
    _bInitialized = false;
    _bShutdown = false;
    _bStopStatusThread = false;
    _bStopDetectionThread = false;
    _bStopUpdateEnvironmentThread = false;
    _bStopControllerMonitorThread = false;
    _bCancelCommand = false;
    _bExecutingUserCommand = false;
    _bIsControllerPickPlaceRunning = false;
    _bIsRobotOccludingSourceContainer = false;
    _bForceRequestDetectionResults = false;
    _numPickAttempt = 0;
    _tsStartDetection = 0;
    _resultTimestamp = 0;
    _pImagesubscriberManager = imagesubscribermanager;
    _pDetectorManager = detectormanager;
    _zmqcontext.reset(new zmq::context_t(8));
    _statusport = statusport;
    _commandport = commandport;
    _configport = configport;
    _configdir = configdir;
    _binpickingTaskZmqPort = 0;
    _binpickingTaskHeartbeatPort = 0;
    _binpickingTaskHeartbeatTimeout = 10;
    _binpickingstateTimestamp = 0;
    _lastocclusionTimestamp = 0;
    _controllerCommandTimeout = 10.0;
    _locale = "en_US";
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
    _StopStatusThread();
    _StopCommandThread(_commandport);
    _StopCommandThread(_configport);
}

void MujinVisionManager::Shutdown()
{
    _bShutdown=true;
}

bool MujinVisionManager::IsShutdown()
{
    return _bShutdown;
}

void MujinVisionManager::GetConfig(const std::string& type, std::string& config)
{
    if (type == "visionmanager") {
        config = _pVisionServerParameters->GetJsonString();
    } else if (type == "detector") {
        config = _detectorconfig;
    } else if (type == "imagesubscriber") {
        config = _imagesubscriberconfig;
    }
}

std::string MujinVisionManager::_GetConfigFileName(const std::string& type, const std::string& configname)
{
    return _configdir + "/" + type + "-" + configname + ".json";
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
        // TODO: validate
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

void MujinVisionManager::_PublishStopStatus()
{
    StatusPublisherPtr pStatusPublisher = _pStatusPublisher;
    if( !!pStatusPublisher ) {
        pStatusPublisher->Publish(_GetStatusJsonString(GetMilliTime(), _GetManagerStatusString(MS_Lost), ""));
        VISIONMANAGER_LOG_DEBUG("Stopped status publisher");
    }
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
    if (!!_mPortStopCommandThread[port]) {
        _mPortStopCommandThread[port] = true;
        _mPortCommandThread[port]->join();
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

void MujinVisionManager::_StopCommandServer(const unsigned int port)
{
    std::stringstream ss;
    ss << "Stopped command server (port: " << port << ").";
    VISIONMANAGER_LOG_DEBUG(ss.str());
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
        _bShutdown=true;
        _StopStatusThread();
        _StopCommandThread(_commandport);
        throw UserInterruptException("User requested exit.");
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
            double voxelsize = command_pt.get<double>("voxelsize", 0.01);
            double pointsize = command_pt.get<double>("pointsize", 0.005);
            bool ignoreocclusion = command_pt.get<bool>("ignoreocclusion", false);
            unsigned int maxage = command_pt.get<unsigned int>("maxage", 0);
            std::string obstaclename = command_pt.get<std::string>("obstaclename", "__dynamicobstacle__");
            unsigned long long starttime = command_pt.get<unsigned long long>("starttime", 0);
            std::string locale = command_pt.get<std::string>("locale", "en_US");
            _locale = locale;
            StartDetectionLoop(regionname, cameranames, voxelsize, pointsize, ignoreocclusion, maxage, obstaclename, starttime);
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
            if (resultstate == "") {
                resultstate = "null";
            }
            result_ss << ParametersBase::GetJsonString("state") << ": " << resultstate << ", ";
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
            _locale = locale;
            Initialize(command_pt.get<std::string>("visionmanagerconfigname"),
                       command_pt.get<std::string>("detectorconfigname"),
                       command_pt.get<std::string>("imagesubscriberconfigname"),
                       command_pt.get<std::string>("mujinControllerIp", ""),
                       command_pt.get<unsigned int>("mujinControllerPort", 0),
                       command_pt.get<std::string>("mujinControllerUsernamePass"),
                       command_pt.get<std::string>("robotControllerUri"),
                       command_pt.get<std::string>("robotDeviceIOUri"),
                       command_pt.get<unsigned int>("binpickingTaskZmqPort"),
                       command_pt.get<unsigned int>("binpickingTaskHeartbeatPort"),
                       command_pt.get<double>("binpickingTaskHeartbeatTimeout"),
                       command_pt.get<std::string>("binpickingTaskScenePk"),
                       command_pt.get<std::string>("robotname", ""),
                       command_pt.get<std::string>("targetname"),
                       command_pt.get<std::string>("streamerIp"),
                       command_pt.get<unsigned int>("streamerPort"),
                       command_pt.get<std::string>("tasktype","binpicking"),
                       command_pt.get<unsigned int>("controllertimeout", 10),
                       command_pt.get<std::string>("locale", "en_US")
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
            _DetectObjects(TT_Command, regionname, cameranames, detectedobjects, resultstate, ignoreocclusion, maxage, fetchimagetimeout, fastdetection, bindetection);
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
            double pointsize = command_pt.get("pointsize",0.005);
            bool ignoreocclusion = command_pt.get("ignoreocclusion",false);
            unsigned int maxage = command_pt.get("maxage",0);
            unsigned int fetchimagetimeout = command_pt.get("fetchimagetimeout", 0);
            VisualizePointCloudOnController(regionname, cameranames, pointsize, ignoreocclusion, maxage, fetchimagetimeout);
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
            if (command_pt.count("regionname") == 0) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = command_pt.get<std::string>("regionname");
            if (!_pDetector || !_pBinpickingTask || !_pImagesubscriberManager) {
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
    return !!_pDetectionThread;
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
    boost::shared_ptr<void> onexit = boost::shared_ptr<void>((void*)0, boost::bind(&MujinVisionManager::_PublishStopStatus, this));
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
        }
        for (unsigned int i=0; i<vstatus.size(); i++) {
            //VISIONMANAGER_LOG_ERROR(_GetStatusJsonString(vtimestamp.at(i), _GetManagerStatusString(vstatus.at(i)), vcmdmsg.at(i), vcmderr.at(i), vcfgmsg.at(i), vcfgerr.at(i), vdetectormsg.at(i), vdetectorerr.at(i), vupdateenvmsg.at(i), vupdateenverr.at(i), vcontrollermonmsg.at(i), vcontrollermonerr.at(i), vsendpclmsg.at(i), vsendpclerr.at(i)));
            _pStatusPublisher->Publish(_GetStatusJsonString(vtimestamp.at(i), _GetManagerStatusString(vstatus.at(i)), vcmdmsg.at(i), vcmderr.at(i), vcfgmsg.at(i), vcfgerr.at(i), vdetectormsg.at(i), vdetectorerr.at(i), vupdateenvmsg.at(i), vupdateenverr.at(i), vcontrollermonmsg.at(i), vcontrollermonerr.at(i), vsendpclmsg.at(i), vsendpclerr.at(i)));
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
    }
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
    ss << "}";
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
            if( _mPortCommandServer[port]->Recv(incomingmessage) > 0 ) {
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
                catch (std::exception& e) {
                    std::string whatstr = e.what();
                    std::string errstr = ParametersBase::GetExceptionJsonString(GetErrorCodeString(MVE_Failed), whatstr);
                    result_ss << "{" << errstr << "}";
                    VISIONMANAGER_LOG_ERROR("unhandled exception, " + whatstr);
                    _SetStatus(TT_Command, MS_Aborted, "", errstr, false);
                }

                // send output
                // TODO: verify validity
                _mPortCommandServer[port]->Send(result_ss.str());

            } else {
                // wait for command
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            }
        }
        catch (const UserInterruptException& ex) {
            _SetStatus(TT_Command, MS_Aborted, "User requested program exit", "", false);
            VISIONMANAGER_LOG_WARN("User requested program exit.");
            throw;
        }
    }
    _StopCommandServer(port);
}

void MujinVisionManager::_StartDetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const std::string& obstaclename, const unsigned long long& starttime)
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
        _pDetectionThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_DetectionThread, this, regionname, cameranames, voxelsize, pointsize, ignoreocclusion, maxage, obstaclename)));
    }
}

void MujinVisionManager::_StartUpdateEnvironmentThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const std::string& obstaclename, const unsigned int waitinterval, const std::string& locale)
{
    if (!!_pUpdateEnvironmentThread && !_bStopUpdateEnvironmentThread) {
        _SetStatusMessage(TT_Command, "UpdateEnvironment thread is already running, do nothing.");
    } else {
        _bStopUpdateEnvironmentThread = false;
        _pUpdateEnvironmentThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_UpdateEnvironmentThread, this, regionname, cameranames, voxelsize, pointsize, obstaclename, waitinterval, locale)));
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

void MujinVisionManager::_StopDetectionThread()
{
    _SetStatusMessage(TT_Command, "Stopping detectoin thread.");
    if (!_bStopDetectionThread) {
        _bStopDetectionThread = true;
        if (!!_pDetectionThread) {
            _pDetectionThread->join();
            _pDetectionThread.reset();
            _SetStatusMessage(TT_Command, "Stopped detection thread.");
        }
        _bStopDetectionThread = false; // reset so that _GetImage works properly afterwards
    }
}

void MujinVisionManager::_StopUpdateEnvironmentThread()
{
    _SetStatusMessage(TT_Command, "Stopping update environment thread.");
    if (!_bStopUpdateEnvironmentThread) {
        _bStopUpdateEnvironmentThread = true;
        if (!!_pUpdateEnvironmentThread) {
            _pUpdateEnvironmentThread->join();
            _pUpdateEnvironmentThread.reset();
            _SetStatusMessage(TT_Command, "Stopped update environment thread.");
        }
        _bStopUpdateEnvironmentThread = false; // reset so that _GetImage works properly afterwards
    }
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
}

void MujinVisionManager::_DetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const std::string& obstaclename)
{
    uint64_t time0;
    int numfastdetection = 1; // max num of times to run fast detection
    int lastDetectedId = 0;
    int lastPickedId = -1;
    uint64_t lastocclusionwarningts = 0;
    uint64_t lastdetectionresultwarningts = 0;
    uint64_t lastbinpickingstatewarningts = 0;
    uint64_t lastwaitforocclusionwarningts = 0;
    uint64_t lastattemptts = 0;
    int numPickAttempt = 0;
    bool isControllerPickPlaceRunning = false;
    bool isRobotOccludingSourceContainer = false;
    bool forceRequestDetectionResults = false;
    unsigned long long binpickingstateTimestamp = 0;

    while (!_bStopDetectionThread) {
        time0 = GetMilliTime();
        // update picked positions
        Vector weights(2,2,1); // prioritize XY over Z
        if (_pVisionServerParameters->clearRadius > 0) {
            BinPickingTaskResource::ResultGetPickedPositions pickedpositions;
            try {
                _pBinpickingTask->GetPickedPositions(pickedpositions,"m");
            }
            catch(const std::exception& ex) {
                std::stringstream ss;
                ss << "Failed to get picked positions from mujin controller: " << ex.what() << ".";
                //std::string errstr = ParametersBase::GetExceptionJsonString(GetErrorCodeString(MVE_ControllerError), ss.str());
                _SetDetectorStatusMessage(ss.str(), GetErrorCodeString(MVE_ControllerError));
                VISIONMANAGER_LOG_WARN(ss.str());
                continue;
            }
            const unsigned int numPickedPositions = pickedpositions.transforms.size();
            VISIONMANAGER_LOG_DEBUG("Got " + boost::lexical_cast<std::string>(numPickedPositions) + " picked positions");

            // remove saved detection results near picked positions
            //bool pickedRecently = false;
            for (unsigned int i=0; i<numPickedPositions; i++) {
                unsigned long long timestamp = pickedpositions.timestamps[i];
                // if timestamp is known
                if (_sTimestamp.find(timestamp)!=_sTimestamp.end()) {
                    if (GetMilliTime() - timestamp < _pVisionServerParameters->timeToIgnore) {
                        VISIONMANAGER_LOG_DEBUG("Just picked up an object, keep ignoring detection in this region (" + boost::lexical_cast<std::string>(_pVisionServerParameters->timeToIgnore - (GetMilliTime()-timestamp)) + " ms left).");
                    } else {
                        //std::cout << "Already cleared picked position at timestamp " << (GetMilliTime() - timestamp) << " ms ago." << std::endl;
                        continue;
                    }
                } else { // for new timestamp
                    _sTimestamp.insert(timestamp);
                    VISIONMANAGER_LOG_DEBUG("Added timestamp " + boost::lexical_cast<std::string>(timestamp) + " to cleared set.");
                    //pickedRecently = true;
                }
                VISIONMANAGER_LOG_DEBUG("An object was picked " + boost::lexical_cast<std::string>(GetMilliTime()-timestamp) + " ms ago, clear known detection results that are nearby.");
                Transform transform = _GetTransform(pickedpositions.transforms[i]);
                Vector position = transform.trans;

                if (_vDetectedInfo.size()>0) {
                    for (int j = _vDetectedInfo.size() - 1; j >= 0; j--) { // have to iterate from the end to remove items from the vectors
                        double dist = std::sqrt(((position-_vDetectedInfo.at(j).meanPosition)*weights).lengthsqr3());
                        VISIONMANAGER_LOG_DEBUG("Part " + boost::lexical_cast<std::string>(j) + " distance to object " + boost::lexical_cast<std::string>(dist));
                        if (dist < _pVisionServerParameters->clearRadius) {
                            VISIONMANAGER_LOG_DEBUG("Part " + boost::lexical_cast<std::string>(j) + " is within the clear radius of picked position, clear its records.")
                            _vDetectedInfo.erase(_vDetectedInfo.begin()+j);
                        }
                    }
                }
            }

            // detect objects
            if (_bStopDetectionThread) {
                break;
            }
        }

        {
            boost::mutex::scoped_lock lock(_mutexControllerBinpickingState);
            if (binpickingstateTimestamp != _binpickingstateTimestamp) {
                std::stringstream ss;
                ss << _binpickingstateTimestamp << " " << _numPickAttempt << " " << _bIsControllerPickPlaceRunning << " " << _bIsRobotOccludingSourceContainer << " " << forceRequestDetectionResults;
                VISIONMANAGER_LOG_DEBUG(ss.str());
            }
            binpickingstateTimestamp = _binpickingstateTimestamp;
            if (_numPickAttempt > numPickAttempt) {
                lastattemptts = binpickingstateTimestamp;
            }
            numPickAttempt = _numPickAttempt;
            isControllerPickPlaceRunning = _bIsControllerPickPlaceRunning;
            isRobotOccludingSourceContainer = _bIsRobotOccludingSourceContainer;
            forceRequestDetectionResults = _bForceRequestDetectionResults;
        }
        if (_bStopDetectionThread) {
            break;
        }

        if (!isControllerPickPlaceRunning || forceRequestDetectionResults || _vDetectedObject.size() == 0) { // detect if forced or no result
            std::stringstream ss;
            ss << "force detection, start capturing..." << (int)isControllerPickPlaceRunning << " " << (int)forceRequestDetectionResults << " " << _vDetectedObject.size();
            VISIONMANAGER_LOG_INFO(ss.str());
            _pImagesubscriberManager->StartCaptureThread();
        } else {  // do the following only if pick and place thread is running and detection is not forced
            if (numPickAttempt <= lastPickedId) { // if robot has picked
                if (GetMilliTime() - binpickingstateTimestamp < maxage) { // only do the following if the binpicking state message is up-to-date
                    if (isRobotOccludingSourceContainer) { // skip detection if robot occludes camera
                        if (GetMilliTime() - lastocclusionwarningts > 1000.0) {
                            VISIONMANAGER_LOG_INFO("robot is picking now (occluding camera), stop capturing");
                            lastocclusionwarningts = GetMilliTime();
                        }
                        if (binpickingstateTimestamp > _lastocclusionTimestamp) {
                            _lastocclusionTimestamp = binpickingstateTimestamp;
                        }
                        _pImagesubscriberManager->StopCaptureThread();
                        continue;
                    } else { // detect when robot is not occluding camera
                        std::stringstream ss;
                        ss << "need to detect for this picking attempt, starting image capturing... " << numPickAttempt << " " << lastPickedId << " " << int(forceRequestDetectionResults) << " " << lastDetectedId << std::endl;
                        VISIONMANAGER_LOG_INFO(ss.str());
                        _pImagesubscriberManager->StartCaptureThread();
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

        if (_bStopDetectionThread) {
            break;
        }
        std::vector<DetectedObjectPtr> detectedobjects;
        std::string resultstate;
        try {
            if (numfastdetection > 0) {
                while (detectedobjects.size() == 0 && numfastdetection > 0) {
                    VISIONMANAGER_LOG_DEBUG("DetectObjects() in fast mode");
                    _DetectObjects(TT_Detector, regionname, cameranames, detectedobjects, resultstate, ignoreocclusion, maxage, 0, true, true);
                    if (detectedobjects.size() == 0) {
                        numfastdetection -= 1;
                    } else {
                        numfastdetection = 0;
                    }
                }
                if (detectedobjects.size() == 0 && numfastdetection == 0) {
                    VISIONMANAGER_LOG_DEBUG("DetectObjects() in fast mode found no object, detect in normal mode");
                    _DetectObjects(TT_Detector, regionname, cameranames, detectedobjects, resultstate, ignoreocclusion, maxage, 0, false, false, false, true);
                }
                numfastdetection -= 1;
            } else {
                _DetectObjects(TT_Detector, regionname, cameranames, detectedobjects, resultstate, ignoreocclusion, maxage, 0, false, false);
            }
            _vDetectedObject = detectedobjects;
            std::vector<std::string> cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);
            for (unsigned int i=0; i<cameranamestobeused.size(); i++) {
                std::string cameraname = cameranamestobeused[i];
                std::vector<Real> points;
                std::stringstream ss;
                uint64_t starttime = GetMilliTime();
                {
                    boost::mutex::scoped_lock lock(_mutexDetector);
                    _pDetector->GetPointCloudObstacle(regionname, cameraname, _vDetectedObject, points, voxelsize, false, false);
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
            if (detectedobjects.size() > 0) {
                VISIONMANAGER_LOG_INFO("detected at least 1 object, stop image capturing...");
                _pImagesubscriberManager->StopCaptureThread();
            } else {
                VISIONMANAGER_LOG_INFO("detected no object, do not stop image capturing...");
            }
        }
        catch(const std::exception& ex) {
            std::stringstream ss;
            ss << "Caught exception in the detection loop: " << ex.what();
            //std::string errstr = ParametersBase::GetExceptionJsonString(GetErrorCodeString(MVE_RecognitionError), ss.str());
            VISIONMANAGER_LOG_ERROR(ss.str());
            _SetDetectorStatusMessage(ss.str(), GetErrorCodeString(MVE_RecognitionError));
            continue;
        }

        // process results
        if (_bStopDetectionThread) {
            break;
        }

        std::vector<DetectedObjectPtr> newdetectedobjects;

        {
            boost::mutex::scoped_lock lock(_mutexDetectedInfo);
            if (_pVisionServerParameters->numDetectionsToKeep>0) {
                for (unsigned int i=0; i<detectedobjects.size(); i++) {
                    unsigned long long timestamp = detectedobjects[i]->timestamp;
                    std::string confidence = detectedobjects[i]->confidence;
                    Transform transform = detectedobjects[i]->transform;
                    TransformMatrix mat(transform);
                    Vector position = transform.trans;
                    Vector rotation = transform.rot;
                    double minDist = 999;
                    int minIndex = -1;
                    if (_pVisionServerParameters->numDetectionsToKeep>0) {
                        // make sure the z-axis of the detected rotation's origin is pointing up in the world frame, so that upside-down flipping is considered the same
                        double dotproductX = mat.m[0]+mat.m[4]+mat.m[8];
                        double dotproductY = mat.m[1]+mat.m[5]+mat.m[9];
                        double dotproductZ = mat.m[2]+mat.m[6]+mat.m[10];
                        if (dotproductZ<0  // if z pointing down
                            || (dotproductZ == 0 && dotproductX <0) // or if z pointing flat, but x pointing down
                            || (dotproductZ == 0 && dotproductX == 0 && dotproductY<0) // or both z and x pointing flat, but y pointing down
                            ) {
                            std::stringstream ss;
                            ss << "Upside-down detection (" << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << ", " << rotation[3] << "), flip rotation.";
                            VISIONMANAGER_LOG_DEBUG(ss.str());
                            // rotate around x axis by 180
                            rotation[0] = -transform.rot[1];
                            rotation[1] = transform.rot[0];
                            rotation[2] = transform.rot[3];
                            rotation[3] = -transform.rot[2];
                        }
                    }
                    for (unsigned int j=0; j<_vDetectedInfo.size(); j++) {
                        double dist = std::sqrt(((position-_vDetectedInfo[j].meanPosition)*weights).lengthsqr3());
                        if (dist < minDist) {
                            minDist = dist;
                            minIndex = j;
                        }
                    }
                    if (minDist < _pVisionServerParameters->maxPositionError && _pVisionServerParameters->numDetectionsToKeep>0) {
                        _vDetectedInfo.at(minIndex).count++;
                        unsigned int numDetections;
                        // only keep track of the last n detection results
                        if (_vDetectedInfo.at(minIndex).count <= _pVisionServerParameters->numDetectionsToKeep) {
                            _vDetectedInfo.at(minIndex).positions.push_back(position);
                            _vDetectedInfo.at(minIndex).rotations.push_back(rotation);
                            _vDetectedInfo.at(minIndex).confidences.push_back(confidence);
                            numDetections = _vDetectedInfo.at(minIndex).count;
                        } else {
                            numDetections = _pVisionServerParameters->numDetectionsToKeep;
                            unsigned int newindex = _vDetectedInfo.at(minIndex).count% numDetections;
                            _vDetectedInfo.at(minIndex).positions.at(newindex) = position;
                            _vDetectedInfo.at(minIndex).rotations.at(newindex) = rotation;
                            _vDetectedInfo.at(minIndex).confidences.at(newindex) = confidence;
                        }
                        std::stringstream ss;
                        ss << "Part " << minIndex << " is known (minDist " << minDist << "), updating its mean position averaging " << numDetections << " detections.";
                        VISIONMANAGER_LOG_DEBUG(ss.str());

                        // update timestamp
                        _vDetectedInfo.at(minIndex).timestamp = timestamp;
                        // update means
                        Vector sumPosition(0,0,0);
                        for (unsigned int j=0; j<numDetections; j++) {
                            sumPosition += _vDetectedInfo.at(minIndex).positions.at(j);
                        }
                        _vDetectedInfo.at(minIndex).meanPosition = sumPosition * (1.0f/numDetections);
                        double minQuatDotProduct = 999;
                        int minQuatIndex = -1;
                        for (unsigned int j=0; j<numDetections; j++) {
                            double sum = 0;
                            for (unsigned int k=0; k<numDetections; k++) {
                                sum += 1- _vDetectedInfo.at(minIndex).rotations.at(j).dot(_vDetectedInfo.at(minIndex).rotations.at(k));
                            }
                            double quatDotProduct = sum / numDetections;
                            if (quatDotProduct < minQuatDotProduct) {
                                minQuatDotProduct = quatDotProduct;
                                minQuatIndex = j;
                            }
                        }
                        _vDetectedInfo.at(minIndex).meanRotation = _vDetectedInfo.at(minIndex).rotations.at(minQuatIndex);
                    } else { // new object is detected
                        //std::cout << "New object is detected at (" << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << ", " << rotation[3] << " ," <<  position[0] << ", " << position[1] << ", " << position[2] << ")" << std::endl;
                        std::vector<Vector> positions;
                        positions.push_back(position);
                        std::vector<Vector> rotations;
                        rotations.push_back(rotation);
                        std::vector<std::string> confidences;
                        confidences.push_back(confidence);
                        DetectedInfo info;
                        info.timestamp = timestamp;
                        info.count = 1;
                        info.meanPosition = position;
                        info.meanRotation = rotation;
                        info.positions = positions;
                        info.rotations = rotations;
                        info.confidences = confidences;
                        _vDetectedInfo.push_back(info);
                    }
                }
                if (_vDetectedInfo.size()>0 && _pVisionServerParameters->numDetectionsToKeep>0) {
                    // remove old detection results
                    for (int i=_vDetectedInfo.size()-1; i>=0; i--) {
                        if (GetMilliTime() - _vDetectedInfo.at(i).timestamp > _pVisionServerParameters->timeToRemember) {
                            std::stringstream ss;
                            ss << "Part " << i << " has not been seen for " << _pVisionServerParameters->timeToRemember << " ms, removing its records.";
                            VISIONMANAGER_LOG_DEBUG(ss.str());
                            _vDetectedInfo.erase(_vDetectedInfo.begin()+i);
                        }
                    }
                }

                // create new results
                if (detectedobjects.size()>0) {
                    for (unsigned int i=0; i<_vDetectedInfo.size(); ++i) {
                        Transform transform;
                        transform.trans = _vDetectedInfo.at(i).meanPosition;
                        transform.rot = _vDetectedInfo.at(i).meanRotation;
                        DetectedObjectPtr obj(new DetectedObject(detectedobjects[0]->name, detectedobjects[0]->objecturi, transform, _vDetectedInfo.at(i).confidences.at(0), _vDetectedInfo.at(i).timestamp, detectedobjects[0]->extra));
                        newdetectedobjects.push_back(obj);
                        //obj->Print();
                    }
                }
            } else {
                newdetectedobjects = detectedobjects;
                _vDetectedObject = detectedobjects;
            }
            _resultState = resultstate;
            _resultTimestamp = GetMilliTime();
        }
        // send results to mujin controller
        if (_bStopDetectionThread) {
            break;
        }

        VISIONMANAGER_LOG_INFO("Cycle time: " + boost::lexical_cast<std::string>((GetMilliTime() - time0)/1000.0f) + " secs");
        VISIONMANAGER_LOG_INFO(" ------------------------");
    }
}

void MujinVisionManager::_UpdateEnvironmentThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const std::string& obstaclename, const unsigned int waitinterval, const std::string& locale)
{
    uint64_t lastUpdateTimestamp = GetMilliTime();
    std::vector<std::string> cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);

    BinPickingTaskResourcePtr pBinpickingTask = _pSceneResource->GetOrCreateBinPickingTaskFromName_UTF8(_tasktype+std::string("task1"), _tasktype, TRO_EnableZMQ);
    std::string userinfo_json = "{\"username\": " + ParametersBase::GetJsonString(_pControllerClient->GetUserName()) + ", \"locale\": " + ParametersBase::GetJsonString(locale) + "}";
    VISIONMANAGER_LOG_DEBUG("initialzing binpickingtask in UpdateEnvironmentThread with userinfo " + userinfo_json);
    pBinpickingTask->Initialize(_robotControllerUri, _robotDeviceIOUri, _binpickingTaskZmqPort, _binpickingTaskHeartbeatPort, _zmqcontext, false, _binpickingTaskHeartbeatTimeout, _controllerCommandTimeout, userinfo_json);
    uint64_t starttime;
    uint64_t lastwarnedtimestamp = 0;
    while (!_bStopUpdateEnvironmentThread) {
        bool update = false;

        {
            boost::mutex::scoped_lock lock(_mutexDetectedInfo);
            update = _resultTimestamp != 0 && _resultTimestamp > lastUpdateTimestamp;
        }

        if (!update) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
            continue;
        } else {
            lastUpdateTimestamp = _resultTimestamp;
            std::vector<BinPickingTaskResource::DetectedObject> detectedobjects;
            std::vector<Real> totalpoints;
            //uint64_t starttime = GetMilliTime();
            std::string resultstate;
            {
                boost::mutex::scoped_lock lock(_mutexDetectedInfo);
                for (unsigned int i=0; i<_vDetectedObject.size(); i++) {
                    mujinclient::Transform transform;
                    transform.quaternion[0] = _vDetectedObject[i]->transform.rot[0];
                    transform.quaternion[1] = _vDetectedObject[i]->transform.rot[1];
                    transform.quaternion[2] = _vDetectedObject[i]->transform.rot[2];
                    transform.quaternion[3] = _vDetectedObject[i]->transform.rot[3];
                    transform.translate[0] = _vDetectedObject[i]->transform.trans[0];
                    transform.translate[1] = _vDetectedObject[i]->transform.trans[1];
                    transform.translate[2] = _vDetectedObject[i]->transform.trans[2];

                    BinPickingTaskResource::DetectedObject detectedobject;
                    std::stringstream name_ss;
                    name_ss << _vDetectedObject[i]->name << "_" << i;
                    detectedobject.name = name_ss.str();
                    detectedobject.object_uri = _vDetectedObject[i]->objecturi;
                    detectedobject.transform = transform;
                    detectedobject.confidence = _vDetectedObject[i]->confidence;
                    detectedobject.timestamp = _vDetectedObject[i]->timestamp;
                    detectedobject.extra = _vDetectedObject[i]->extra;
                    detectedobjects.push_back(detectedobject);
                }
                for(unsigned int i=0; i<cameranamestobeused.size(); i++) {
                    std::string cameraname = cameranamestobeused[i];
                    // get point cloud obstacle
                    std::vector<Real> points = _mResultPoints[cameraname];
                    totalpoints.insert(totalpoints.end(), points.begin(), points.end());
                }
                resultstate = _resultState;
            }
            if (totalpoints.size()>0) {
                try {
                    starttime = GetMilliTime();
                    pBinpickingTask->UpdateEnvironmentState(_targetname, "mujin:/" + _targetname + ".mujin.dae", detectedobjects, totalpoints, resultstate, pointsize, obstaclename, "m");
                    std::stringstream ss;
                    ss << "UpdateEnvironmentState with " << detectedobjects.size() << " objects " << (totalpoints.size()/3.) << " points, took " << (GetMilliTime() - starttime) / 1000.0f << " secs";
                    _SetStatusMessage(TT_UpdateEnvironment, ss.str());
                } catch(const std::exception& ex) {
                    if (GetMilliTime() - lastwarnedtimestamp > 1000.0) {
                        lastwarnedtimestamp = GetMilliTime();
                        std::stringstream ss;
                        ss << "Failed to update environment state: " << ex.what() << ".";
                        //std::string errstr = ParametersBase::GetExceptionJsonString(GetErrorCodeString(MVE_ControllerError), ss.str());
                        _SetStatusMessage(TT_UpdateEnvironment, ss.str(), GetErrorCodeString(MVE_ControllerError));
                        VISIONMANAGER_LOG_WARN(ss.str());
                    }
                    boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                }
            }
        }
    }
}

void MujinVisionManager::_ControllerMonitorThread(const unsigned int waitinterval, const std::string& locale)
{
    BinPickingTaskResourcePtr pBinpickingTask = _pSceneResource->GetOrCreateBinPickingTaskFromName_UTF8(_tasktype+std::string("task1"), _tasktype, TRO_EnableZMQ);
    std::string userinfo_json = "{\"username\": " + ParametersBase::GetJsonString(_pControllerClient->GetUserName()) + ", \"locale\": " + ParametersBase::GetJsonString(_locale) + "}";
    pBinpickingTask->Initialize(_robotControllerUri, _robotDeviceIOUri, _binpickingTaskZmqPort, _binpickingTaskHeartbeatPort, _zmqcontext, false, _binpickingTaskHeartbeatTimeout, _controllerCommandTimeout, userinfo_json);

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
                    _SetStatusMessage(TT_ControllerMonitor, ss.str(), GetErrorCodeString(MVE_ControllerError));
                    VISIONMANAGER_LOG_WARN(ss.str());
                }
                boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                continue;
            }
            _bIsControllerPickPlaceRunning = (binpickingstate.statusPickPlace == "Running");
            _bIsRobotOccludingSourceContainer = binpickingstate.isRobotOccludingSourceContainer;
            _bForceRequestDetectionResults = binpickingstate.forceRequestDetectionResults;
            _numPickAttempt = binpickingstate.pickAttemptFromSourceId;
            _binpickingstateTimestamp = binpickingstate.timestamp * 1000; // s -> ms
            lastUpdateTimestamp = GetMilliTime();
        }

        uint64_t dt = GetMilliTime() - lastUpdateTimestamp;

        if (dt < waitinterval) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval- dt));
        }
    }
}

mujinvision::Transform MujinVisionManager::_GetTransform(const std::string& instobjname)
{
    mujinclient::Transform t;
    _pBinpickingTask->GetTransform(instobjname,t,"m");
    return _GetTransform(t);
}

void MujinVisionManager::_SyncCamera(const std::string& regionname, const std::string& cameraname)
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

void MujinVisionManager::_SyncCamera(const std::string& regionname, const std::string& cameraname, const mujinclient::Transform& t)
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

unsigned int MujinVisionManager::_GetColorImages(ThreadType tt, const std::string& regionname, const std::vector<std::string>& cameranames, std::vector<ImagePtr>& colorimages, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request, const unsigned int waitinterval)
{
    return _GetImages(tt, regionname, cameranames, colorimages, ignoreocclusion, maxage, fetchimagetimeout, request, waitinterval, true);
}

unsigned int MujinVisionManager::_GetDepthImages(ThreadType tt, const std::string& regionname, const std::vector<std::string>& cameranames, std::vector<ImagePtr>& depthimages, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request, const unsigned int waitinterval)
{
    return _GetImages(tt, regionname, cameranames, depthimages, ignoreocclusion, maxage, fetchimagetimeout, request, waitinterval, false);
}

unsigned int MujinVisionManager::_GetImages(ThreadType tt, const std::string& regionname, const std::vector<std::string>& cameranames, std::vector<ImagePtr>& images, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request, const unsigned int waitinterval, const bool iscolor)
{
    uint64_t start0 = GetMilliTime();
    unsigned long long starttime = 0, endtime = 0;
    bool isoccluding = !ignoreocclusion;
    std::string cameraname;
    //uint64_t lastfirstimagecheckfailurets = 0;
    uint64_t lastfirstimagecheckfailurewarnts = 0;
    uint64_t lastocclusioncheckfailurets = 0;
    uint64_t lastocclusioncheckfailurewarnts = 0;
    bool usecache = !request;
    double snaptimeout = fetchimagetimeout / 1000.0;

    //
    // Enforce a minimum timeout for zmq socket req rep.
    // Otherwise, zmq may be blocked on recv forever, preventing
    // client from cancelling detection.
    //
    if (snaptimeout < 2.0) {
        snaptimeout = 2.0;
    }

    images.clear();

    while (!_bCancelCommand &&
           !_bShutdown &&
           ((fetchimagetimeout == 0) ||
            (fetchimagetimeout > 0 && GetMilliTime() - start0 < fetchimagetimeout))) {

        cameraname = cameranames.at(images.size());
        ImagePtr image;

        if (usecache) {
            //VISIONMANAGER_LOG_DEBUG("getting images from buffer");
            if (iscolor) {
                image = _pImagesubscriberManager->GetColorImageFromBuffer(cameraname, starttime, endtime);
            } else {
                image = _pImagesubscriberManager->GetDepthImageFromBuffer(cameraname, starttime, endtime);
            }
        } else {
            VISIONMANAGER_LOG_DEBUG("snapping images");
            if (iscolor) {
                image = _pImagesubscriberManager->SnapColorImage(cameraname, starttime, endtime, snaptimeout);
            } else {
                image = _pImagesubscriberManager->SnapDepthImage(cameraname, starttime, endtime, snaptimeout);
            }
        }

        if (_bStopDetectionThread) {
            break;
        }

        if (!image) {
            std::stringstream msg_ss;
            msg_ss << "Could not get image, will try again"
                   << ": camera = " << cameraname
                   << ", is_color = " << iscolor
                   << ", use_cache = " << usecache
                   << ", images_size = " << images.size();
            VISIONMANAGER_LOG_WARN(msg_ss.str());
            boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
            // usecache = false;  // do not force snap, because images come in pairs, and snap can only get one new image
            continue;
        }

        if (GetMilliTime()  < starttime - 100) {
            std::stringstream msg_ss;
            msg_ss << "Image timestamp is more than 100ms in the future, please ensure that clocks are synchronized"
                   << ": camera = " << cameraname
                   << ", is_color = " << iscolor
                   << ", use_cache = " << usecache
                   << ", images_size = " << images.size();
            VISIONMANAGER_LOG_WARN(msg_ss.str());
            // usecache = false; // do not force snap, because images come in pairs, and snap can only get one new image
            continue;
        }

        if (maxage>0 && GetMilliTime()-starttime>maxage) {
            std::stringstream msg_ss;
            msg_ss << "Image is more than " << maxage << " ms old (" << (GetMilliTime() - starttime) << "), will try to get again"
                   << ": camera = " << cameraname
                   << ", is_color = " << iscolor
                   << ", use_cache = " << usecache
                   << ", images_size = " << images.size();
            VISIONMANAGER_LOG_WARN(msg_ss.str());
            if (images.size() > 0) {
                images.clear(); // need to start over, all color images need to be equally new
            }
            //usecache = false;  // do not force snap, because images come in pairs, and snap can only get one new image
            continue;
        }

        if (!request && _tsStartDetection > 0 && starttime < _tsStartDetection) {
            if (GetMilliTime() - lastfirstimagecheckfailurewarnts > 1000.0) {
                lastfirstimagecheckfailurewarnts = GetMilliTime();
                std::stringstream msg_ss;
                msg_ss << "Image was taken " << (_tsStartDetection - starttime) << " ms before _tsStartDetection " << _tsStartDetection << ", will try to get again"
                       << ": camera = " << cameraname
                       << ", is_color = " << iscolor
                       << ", use_cache = " << usecache
                       << ", images_size = " << images.size();
                VISIONMANAGER_LOG_WARN(msg_ss.str());
            }
            if (images.size() > 0) {
                images.clear(); // need to start over, all color images need to be equally new
            }
            //usecache = false;  // do not force snap, because images come in pairs, and snap can only get one new image
            continue;
        }

        //std::stringstream msg_ss;
        //msg_ss << "Got image that is " << (GetMilliTime()-starttime) << " ms old, took " << ((GetMilliTime()-start0)/1000.0f) << " secs. " << iscolor;
        //VISIONMANAGER_LOG_DEBUG(msg_ss.str());
        if (!ignoreocclusion) {
            try {
                if (starttime > lastocclusioncheckfailurets) {
                    _pBinpickingTask->IsRobotOccludingBody(regionname, cameraname, starttime, endtime, isoccluding);
                }
            } catch (...) {
                std::stringstream ss;
                ss << "Failed to check for occlusion, will try again.";
                VISIONMANAGER_LOG_WARN(ss.str());
                boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                //usecache = false; // do not force snap, because images come in pairs, and snap can only get one new image
                _SetStatusMessage(tt, "", ss.str());
                continue;
            }
        }

        if (isoccluding) {
            if (GetMilliTime() - lastocclusioncheckfailurewarnts > 1000.0) {
                lastocclusioncheckfailurewarnts = GetMilliTime();
                std::stringstream msg_ss;
                msg_ss << "Region is occluded in the view of camera, will try again"
                       << ": camera = " << cameraname
                       << ", is_color = " << iscolor
                       << ", use_cache = " << usecache
                       << ", images_size = " << images.size();
                VISIONMANAGER_LOG_WARN(msg_ss.str());
            }
            lastocclusioncheckfailurets = starttime;
            if (starttime > _lastocclusionTimestamp) {
                _lastocclusionTimestamp = starttime;
            }
            //usecache = false; // do not force snap, because images come in pairs, and snap can only get one new image
            continue;
        }

        images.push_back(image);
        if (images.size() == cameranames.size()) {
            // got one image for each camera, exit
            break;
        }

        // move on to get image for the next camera       
    }

    if (images.size() < cameranames.size()) {
        std::stringstream msg_ss;
        msg_ss << "Got " << images.size() << " out of " << cameranames.size() << " images"
               << ": cancel = " << int(_bCancelCommand)
               << ", shutdown = " << int(_bShutdown)
               << ", stop_detection = " << int(_bStopDetectionThread);
        VISIONMANAGER_LOG_WARN(msg_ss.str());

        std::stringstream exception_ss;
        exception_ss << "Failed to get " << cameranames.size() << " images, got " << images.size();
        throw MujinVisionException(exception_ss.str(), MVE_ImageAcquisitionError);
    }

    return images.size();
}

void MujinVisionManager::_GetImages(ThreadType tt, const std::string& regionname, const std::vector<std::string>& colorcameranames, const std::vector<std::string>& depthcameranames, std::vector<ImagePtr>& colorimages, std::vector<ImagePtr>& depthimages, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request, const bool useold, const unsigned int waitinterval)
{
    if (useold && _lastcolorimages.size() == colorcameranames.size() && _lastdepthimages.size() == depthcameranames.size()) {
        VISIONMANAGER_LOG_INFO("using last images");
        colorimages = _lastcolorimages;
        depthimages = _lastdepthimages;
        return;
    }

    uint64_t start0 = GetMilliTime();
    unsigned long long starttime = 0, endtime = 0;
    std::string cameraname;
    uint64_t lastimageagecheckfailurets = 0;
    uint64_t lastfirstimagecheckfailurewarnts = 0;
    uint64_t lastocclusioncheckfailurewarnts = 0;
    bool usecache = !request;

    while (!_bCancelCommand &&
           !_bShutdown &&
           ((fetchimagetimeout == 0) ||
           (fetchimagetimeout > 0 && GetMilliTime() - start0 < fetchimagetimeout)) &&
           colorimages.size() < colorcameranames.size() &&
           depthimages.size() < depthcameranames.size()
           ) {
        if (usecache) {
            _pImagesubscriberManager->GetImagePackFromBuffer(colorcameranames, depthcameranames, colorimages, depthimages, starttime, endtime);
        } else {
            throw MujinVisionException("snapping is not supported", MVE_NotImplemented);
        }

        if (_bStopDetectionThread) {
            break;
        }

        if (colorimages.size() < colorcameranames.size() || depthimages.size() < depthcameranames.size()) {
            std::stringstream msg_ss;
            msg_ss << "Could not get all images, will try again"
                   << ": # color images = " << colorimages.size()
                   << ", # depth images = " << depthimages.size()
                   << ", use_cache = " << usecache;
            VISIONMANAGER_LOG_WARN(msg_ss.str());
            boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
            colorimages.clear();
            depthimages.clear();
            continue;
        }

        if (GetMilliTime()  < starttime - 100) {
            std::stringstream msg_ss;
            msg_ss << "Image timestamp is more than 100ms in the future, please ensure that clocks are synchronized"
                   << ", use_cache = " << usecache;
            VISIONMANAGER_LOG_ERROR(msg_ss.str());
            colorimages.clear();
            depthimages.clear();
            throw MujinVisionException(msg_ss.str(), MVE_ImageAcquisitionError);
        }

        if (maxage>0 && GetMilliTime()-starttime>maxage) {
            if (GetMilliTime() - lastimageagecheckfailurets > 1000.0) {
                std::stringstream msg_ss;
                msg_ss << "Image is more than " << maxage << " ms old (" << (GetMilliTime() - starttime) << "), will try to get again"
                       << ", use_cache = " << usecache;
                VISIONMANAGER_LOG_WARN(msg_ss.str());
                lastimageagecheckfailurets = GetMilliTime();
            }
            colorimages.clear();
            depthimages.clear();
            continue;
        }

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
            continue;
        }

        bool isoccluding = false;
        if (!ignoreocclusion) {
            try {
                for (size_t i=0; i<colorcameranames.size() && !isoccluding; ++i) {
                    //uint64_t time0 = GetMilliTime();
                    //std::stringstream ss;
                    _pBinpickingTask->IsRobotOccludingBody(regionname, colorcameranames.at(i), starttime, endtime, isoccluding);
                    //ss << "_pBinpickingTask->IsRobotOccludingBody() took " << (GetMilliTime() - time0) / 1000.0f << " secs";
                    //VISIONMANAGER_LOG_DEBUG(ss.str());
                }
                // skip checking for depth camera, assuming depth image is derived from color
            } catch (...) {
                std::stringstream ss;
                ss << "Failed to check for occluded, will try again";
                VISIONMANAGER_LOG_WARN(ss.str());
                boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                colorimages.clear();
                depthimages.clear();
                _SetStatusMessage(tt, "", ss.str());
                continue;
            }
        }

        if (isoccluding) {
            if (GetMilliTime() - lastocclusioncheckfailurewarnts > 1000.0) {
                lastocclusioncheckfailurewarnts = GetMilliTime();
                std::stringstream msg_ss;
                msg_ss << "Region is occluded in the view of camera, will try again"
                       << " starttime " << starttime
                       << " endtime " << endtime
                       << ", use_cache = " << usecache;
                VISIONMANAGER_LOG_WARN(msg_ss.str());
            }
            colorimages.clear();
            depthimages.clear();
            continue;
        } else {
            std::stringstream ss;
            ss << "imagepack starttime " << starttime << " endtime " << endtime;
            VISIONMANAGER_LOG_DEBUG(ss.str());
            _lastcolorimages = colorimages;
            _lastdepthimages = depthimages;
        }
    }
}

void MujinVisionManager::Initialize(const std::string& visionmanagerconfigname, const std::string& detectorconfigname, const std::string& imagesubscriberconfigname, const std::string& controllerIp, const unsigned int controllerPort, const std::string& controllerUsernamePass, const std::string& robotControllerUri, const std::string& robotDeviceIOUri, const unsigned int binpickingTaskZmqPort, const unsigned int binpickingTaskHeartbeatPort, const double binpickingTaskHeartbeatTimeout, const std::string& binpickingTaskScenePk, const std::string& robotname, const std::string& targetname, const std::string& streamerIp, const unsigned int streamerPort, const std::string& tasktype, const double controllertimeout, const std::string& locale)
{
    uint64_t time0 = GetMilliTime();
    uint64_t starttime = GetMilliTime();
    _binpickingTaskZmqPort = binpickingTaskZmqPort;
    _binpickingTaskHeartbeatPort = binpickingTaskHeartbeatPort;
    _binpickingTaskHeartbeatTimeout = binpickingTaskHeartbeatTimeout;
    _binpickingTaskScenePk = binpickingTaskScenePk;
    _controllerCommandTimeout = controllertimeout;
    _robotControllerUri = robotControllerUri;
    _robotDeviceIOUri = robotDeviceIOUri;
    _tasktype = tasktype;

    ptree pt;

    // load visionserver configuration
    std::string visionmanagerconfig;
    _LoadConfig(_GetConfigFileName("visionmanager", visionmanagerconfigname), visionmanagerconfig);
    std::stringstream visionmanagerconfigss;
    visionmanagerconfigss << visionmanagerconfig;
    read_json(visionmanagerconfigss, pt);
    _pVisionServerParameters.reset(new VisionServerParameters(pt.get_child("visionserver")));
    // set up regions
    std::vector<RegionParametersPtr > vRegionParameters;
    RegionParametersPtr pRegionParameters;
    FOREACH(v, pt.get_child("regions")) {
        RegionParametersPtr pregionparameters(new RegionParameters(v->second));
        vRegionParameters.push_back(pregionparameters);
        _mNameRegion[pregionparameters->instobjectname] = RegionPtr(new Region(pregionparameters));
    }
    // set up camera parameters
    FOREACH(v, pt.get_child("cameras")) {
        _mNameCameraParameters[v->first].reset(new CameraParameters(v->second));
    }

    // connect to mujin controller
    std::stringstream url_ss;
    url_ss << "http://" << controllerIp << ":" << controllerPort;
    ControllerClientPtr controller = CreateControllerClient(controllerUsernamePass, url_ss.str());
    _pControllerClient = controller;
    _userinfo_json = "{\"username\": " + ParametersBase::GetJsonString(_pControllerClient->GetUserName()) + ", \"locale\": " + ParametersBase::GetJsonString(locale) + "}";

    _SetStatusMessage(TT_Command, "Connected to mujin controller at " + url_ss.str());
    SceneResourcePtr scene(new SceneResource(controller,binpickingTaskScenePk));
    _pSceneResource = scene;
    _pBinpickingTask = scene->GetOrCreateBinPickingTaskFromName_UTF8(tasktype+std::string("task1"), tasktype, TRO_EnableZMQ);
    VISIONMANAGER_LOG_DEBUG("initialzing binpickingtask in Initialize() with userinfo " + _userinfo_json);
    _pBinpickingTask->Initialize(robotControllerUri, robotDeviceIOUri, binpickingTaskZmqPort, binpickingTaskHeartbeatPort, _zmqcontext, false, _binpickingTaskHeartbeatTimeout, _controllerCommandTimeout, _userinfo_json);

    // sync regions and cameras
    _SetStatusMessage(TT_Command, "Syncing regions and cameras");
    std::vector<std::string> regionnames, cameranames;
    FOREACH(it, _mNameRegion) {
        regionnames.push_back(it->first);
    }
    FOREACH(it, _mNameCameraParameters) {
        cameranames.push_back(it->first);
    }
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
                _SyncCamera(regionname, cameraname, resultgetinstobjectandsensorinfo.msensortransform[cameraname]);
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
        }
        _mRegionColorCameraMap[regionname] = mNameColorCamera;
        _mRegionDepthCameraMap[regionname] = mNameDepthCamera;
    }
    VISIONMANAGER_LOG_DEBUG("sync cameras took: " + boost::lexical_cast<std::string>((GetMilliTime() - starttime)/1000.0f) + " secs");

    // set up subscribers
    _SetStatusMessage(TT_Command, "Loading subscriber configuration.");
    // load subscriber configuration
    std::string imagesubscriberconfig;
    _LoadConfig(_GetConfigFileName("imagesubscriber", imagesubscriberconfigname), imagesubscriberconfig);
    _imagesubscriberconfig = imagesubscriberconfig;
    std::stringstream imagesubscriberconfigss;
    imagesubscriberconfigss << imagesubscriberconfig;
    read_json(imagesubscriberconfigss, pt);

    // set up image subscriber manager
    _SetStatusMessage(TT_Command, "Setting up image manager.");
    _pImagesubscriberManager->Initialize(_mNameCamera, streamerIp, streamerPort, pt.get_child("zmq_subscriber"), _zmqcontext);

    // set up detectors
    starttime = GetMilliTime();
    _SetStatusMessage(TT_Command, "Setting up detector.");
    std::string detectorconfig;
    _LoadConfig(_GetConfigFileName("detector", detectorconfigname), detectorconfig);
    _detectorconfig = detectorconfig;
    std::stringstream detectorconfigss;
    detectorconfigss << detectorconfig;
    read_json(detectorconfigss, pt);
    _pDetector = _pDetectorManager->CreateObjectDetector(pt.get_child("object"),pt.get_child("detection"), _mNameRegion, _mRegionColorCameraMap, _mRegionDepthCameraMap, boost::bind(&MujinVisionManager::_SetDetectorStatusMessage, this, _1, _2));
    _targetname = targetname;
    VISIONMANAGER_LOG_DEBUG("detector initialization took: " + boost::lexical_cast<std::string>((GetMilliTime() - starttime)/1000.0f) + " secs");
    VISIONMANAGER_LOG_DEBUG("Initialize() took: " + boost::lexical_cast<std::string>((GetMilliTime() - time0)/1000.0f) + " secs");
    VISIONMANAGER_LOG_DEBUG(" ------------------------");

    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::_DeInitialize()
{
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
    _DetectObjects(TT_Command, regionname, cameranames, detectedobjects, resultstate, ignoreocclusion, maxage, fetchimagetimeout, fastdetection, bindetection, request, useold);
}
void MujinVisionManager::_DetectObjects(ThreadType tt, const std::string& regionname, const std::vector<std::string>&cameranames, std::vector<DetectedObjectPtr>& detectedobjects, std::string& resultstate, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool fastdetection, const bool bindetection, const bool request, const bool useold)
{
    boost::mutex::scoped_lock lock(_mutexDetector);
    uint64_t starttime = GetMilliTime();

    std::vector<std::string> colorcameranames = _GetColorCameraNames(regionname, cameranames);
    std::vector<std::string> depthcameranames = _GetDepthCameraNames(regionname, cameranames);

    // set up images
    std::vector<ImagePtr> colorimages;
    std::vector<ImagePtr> depthimages;
    _GetImages(tt, regionname, colorcameranames, depthcameranames, colorimages, depthimages, ignoreocclusion, maxage, fetchimagetimeout, request, useold);
    //_GetColorImages(regionname, colorcameranames, colorimages, ignoreocclusion, maxage, fetchimagetimeout, request, useold);
    //VISIONMANAGER_LOG_DEBUG("Getting color images took " + boost::lexical_cast<std::string>((GetMilliTime() - starttime) / 1000.0f));
    //uint64_t starttime1 = GetMilliTime();
    //_GetDepthImages(regionname, depthcameranames, depthimages, ignoreocclusion, maxage, fetchimagetimeout, request, useold);
    //VISIONMANAGER_LOG_DEBUG("Getting depth images took " + boost::lexical_cast<std::string>((GetMilliTime() - starttime1) / 1000.0f));
    VISIONMANAGER_LOG_INFO("Getting images took " + boost::lexical_cast<std::string>((GetMilliTime() - starttime) / 1000.0f));
    starttime = GetMilliTime();
    if (colorimages.size() == colorcameranames.size() && depthimages.size() == depthcameranames.size()) {
        for (size_t i=0; i<colorimages.size(); ++i) {
            std::string cameraname = colorcameranames.at(i);
            CameraPtr camera = _mNameCamera[cameraname];
            _pDetector->SetColorImage(cameraname, colorimages.at(i));
        }
        for (size_t i=0; i<depthimages.size(); ++i) {
            std::string cameraname = depthcameranames.at(i);
            CameraPtr camera= _mNameCamera[cameraname];
            _pDetector->SetDepthImage(cameraname, depthimages.at(i));
        }
        // detect objects
        _pDetector->DetectObjects(regionname, colorcameranames, depthcameranames, detectedobjects, resultstate, fastdetection, bindetection);
        std::stringstream msgss;
        msgss << "Detected " << detectedobjects.size() << " objects, state: " << resultstate <<". Took " << (GetMilliTime()-starttime)/1000.0f << " seconds.";
        _SetStatusMessage(tt, msgss.str());
    }
    _SetStatus(tt, MS_Succeeded);
}

void MujinVisionManager::StartDetectionLoop(const std::string& regionname, const std::vector<std::string>&cameranames,const double voxelsize, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const std::string& obstaclename, const unsigned long long& starttime, const std::string& locale)
{
    if (!!_pImagesubscriberManager) {
        _pImagesubscriberManager->StartCaptureThread();
    } else {
        throw MujinVisionException("image subscriber manager is not initialzied", MVE_Failed);
    }
    _StartDetectionThread(regionname, cameranames, voxelsize, pointsize, ignoreocclusion, maxage, obstaclename, starttime);
    _StartUpdateEnvironmentThread(regionname, cameranames, voxelsize, pointsize, obstaclename, 50, locale);
    _StartControllerMonitorThread(50, locale);
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::StopDetectionLoop()
{
    _StopDetectionThread();
    _StopUpdateEnvironmentThread();
    _StopControllerMonitorThread();
    if (!!_pImagesubscriberManager) {
        _pImagesubscriberManager->StopCaptureThread();
    }
    _SetStatus(TT_Command, MS_Succeeded);
}

void MujinVisionManager::SendPointCloudObstacleToController(const std::string& regionname, const std::vector<std::string>&cameranames, const std::vector<DetectedObjectPtr>& detectedobjectsworld, const unsigned int maxage, const unsigned int fetchimagetimeout, const double voxelsize, const double pointsize, const std::string& obstaclename, const bool fast, const bool request, const bool async, const std::string& locale)
{
    _SendPointCloudObstacleToController(TT_Command, regionname, cameranames, detectedobjectsworld, maxage, fetchimagetimeout, voxelsize, pointsize, obstaclename, fast, request, async, locale);
}

void MujinVisionManager::_SendPointCloudObstacleToController(ThreadType tt, const std::string& regionname, const std::vector<std::string>&cameranames, const std::vector<DetectedObjectPtr>& detectedobjectsworld, const unsigned int maxage, const unsigned int fetchimagetimeout, const double voxelsize, const double pointsize, const std::string& obstaclename, const bool fast, const bool request, const bool async, const std::string& locale)
{
    uint64_t starttime = GetMilliTime();
    if (!async) {
        std::vector<std::string> depthcameranames = _GetDepthCameraNames(regionname, cameranames);
        // set up images
        std::vector<ImagePtr> depthimages;
        bool ignoreocclusion = true;
        _GetDepthImages(TT_SendPointcloudObstacle, regionname, depthcameranames, depthimages, ignoreocclusion, maxage, fetchimagetimeout, request);
        if (depthimages.size() == depthcameranames.size()) {
            for (size_t i=0; i<depthimages.size(); ++i) {
                std::string cameraname = depthcameranames.at(i);
                CameraPtr camera= _mNameCamera[cameraname];
                // get point cloud obstacle
                std::vector<Real> points;
                {
                    boost::mutex::scoped_lock lock(_mutexDetector);
                    _pDetector->SetDepthImage(cameraname, depthimages.at(i));
                    _pDetector->GetPointCloudObstacle(regionname, cameraname, detectedobjectsworld, points, voxelsize, fast, true);
                }
                if (points.size() / 3 == 0) {
                    _SetStatusMessage(tt, "got 0 point from GetPointCloudObstacle()");
                    int numretries = 3;
                    std::vector<std::string> depthcameranames1;
                    std::vector<ImagePtr> depthimages1;
                    depthcameranames1.push_back(cameraname);
                    while (numretries > 0 && points.size() / 3 == 0) {
                        points.clear();
                        _SetStatusMessage(tt, "re-try getting depthimage and pointcloudobstacle");
                        _GetDepthImages(TT_SendPointcloudObstacle, regionname, depthcameranames1, depthimages1, ignoreocclusion, maxage, fetchimagetimeout, request);
                        {
                            boost::mutex::scoped_lock lock(_mutexDetector);
                            _pDetector->SetDepthImage(cameraname, depthimages1.at(0));
                            _pDetector->GetPointCloudObstacle(regionname, cameraname, detectedobjectsworld, points, voxelsize, fast, true);
                        }
                        numretries--;
                    }
                    if (points.size() / 3 == 0) {
                        throw MujinVisionException("got 0 point from GetPointCloudObstacle() after retries", MVE_Failed);
                    }
                }
                std::stringstream ss;
                ss <<"Sending over " << (points.size()/3) << " points from " << cameraname << ".";
                _SetStatusMessage(tt, ss.str());
                _pBinpickingTask->AddPointCloudObstacle(points, pointsize, obstaclename);
            }
        }
    }  else {
        _pSendPointCloudObstacleThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_SendPointCloudObstacleToControllerThread, this, regionname, cameranames, detectedobjectsworld, maxage, fetchimagetimeout, voxelsize, pointsize, obstaclename)));   
    }
    std::stringstream ss;
    ss << "SendPointCloudObstacleToController async " << int(async) << " took " << (GetMilliTime() - starttime) / 1000.0f << " secs";
    VISIONMANAGER_LOG_INFO(ss.str());
    _SetStatus(tt, MS_Succeeded);
}

void MujinVisionManager::_SendPointCloudObstacleToControllerThread(const std::string& regionname, const std::vector<std::string>&cameranames, const std::vector<DetectedObjectPtr>& detectedobjectsworld, const unsigned int maxage, const unsigned int fetchimagetimeout, const double voxelsize, const double pointsize, const std::string& obstaclename)
{
    BinPickingTaskResourcePtr pBinpickingTask = _pSceneResource->GetOrCreateBinPickingTaskFromName_UTF8(_tasktype+std::string("task1"), _tasktype, TRO_EnableZMQ);
    std::string userinfo_json = "{\"username\": " + ParametersBase::GetJsonString(_pControllerClient->GetUserName()) + ", \"locale\": " + ParametersBase::GetJsonString(_locale) + "}";
    VISIONMANAGER_LOG_DEBUG("initialzing binpickingtask in _SendPointCloudObstacleToControllerThread with userinfo " + userinfo_json);
    pBinpickingTask->Initialize(_robotControllerUri, _robotDeviceIOUri, _binpickingTaskZmqPort, _binpickingTaskHeartbeatPort, _zmqcontext, false, _binpickingTaskHeartbeatTimeout, _controllerCommandTimeout, userinfo_json);

    std::vector<std::string> depthcameranames = _GetDepthCameraNames(regionname, cameranames);
    // set up images
    std::vector<ImagePtr> depthimages;
    bool ignoreocclusion = true;
    _GetDepthImages(TT_SendPointcloudObstacle, regionname, depthcameranames, depthimages, ignoreocclusion, maxage, fetchimagetimeout, true);
    if (depthimages.size() == depthcameranames.size()) {
        for (size_t i=0; i<depthimages.size(); ++i) {
            std::string cameraname = depthcameranames.at(i);
            CameraPtr camera= _mNameCamera[cameraname];
            // get point cloud obstacle
            std::vector<Real> points;
            {
                boost::mutex::scoped_lock lock(_mutexDetector);
                _pDetector->SetDepthImage(cameraname, depthimages.at(i));
                _pDetector->GetPointCloudObstacle(regionname, cameraname, detectedobjectsworld, points, voxelsize, false, true);
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
                    _GetDepthImages(TT_SendPointcloudObstacle, regionname, depthcameranames, depthimages, ignoreocclusion, maxage, fetchimagetimeout, true);
                    {
                        boost::mutex::scoped_lock lock(_mutexDetector);
                        _pDetector->SetDepthImage(cameraname, depthimages1.at(0));
                        _pDetector->GetPointCloudObstacle(regionname, cameraname, detectedobjectsworld, points, voxelsize, false, true);
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
    }
    _SetStatus(TT_SendPointcloudObstacle, MS_Succeeded);
}

void MujinVisionManager::DetectRegionTransform(const std::string& regionname, const std::vector<std::string>& cameranames, mujinvision::Transform& regiontransform, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request)
{
    // TODO: use actual cameras
    std::vector<ImagePtr> colorimages;
    std::vector<std::string> ccamnames;
    ccamnames.push_back(_GetColorCameraNames(regionname, cameranames).at(0));
    _GetColorImages(TT_Command, regionname, ccamnames, colorimages, ignoreocclusion, maxage, fetchimagetimeout, request);

    std::vector<ImagePtr> depthimages;
    std::vector<std::string> dcamnames;
    dcamnames.push_back(_GetDepthCameraNames(regionname, cameranames).at(0));
    _GetDepthImages(TT_Command, regionname, dcamnames, depthimages, ignoreocclusion, maxage, fetchimagetimeout, request);
    mujinvision::Transform regiontransform0 = regiontransform;
    {
        boost::mutex::scoped_lock lock(_mutexDetector);
        _pDetector->SetColorImage(ccamnames.at(0), colorimages.at(0));
        _pDetector->SetDepthImage(dcamnames.at(0), depthimages.at(0));
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

void MujinVisionManager::VisualizePointCloudOnController(const std::string& regionname, const std::vector<std::string>&cameranames, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const unsigned int fetchimagetimeout, const bool request)
{
    std::vector<std::string> cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);
    std::vector<std::vector<Real> > pointslist;
    std::vector<std::string> names;
    std::vector<double> points;
    for (unsigned int i=0; i<cameranamestobeused.size(); i++) {
        points.resize(0);
        std::string cameraname = cameranamestobeused.at(i);
        CameraPtr camera = _mNameCamera[cameraname];
        std::vector<ImagePtr> depthimages;
        std::vector<std::string> dcamnames;
        dcamnames.push_back(cameraname);
        _GetDepthImages(TT_Command, regionname, dcamnames, depthimages, ignoreocclusion, maxage, fetchimagetimeout, request);
        {
            boost::mutex::scoped_lock lock(_mutexDetector);
            _pDetector->GetCameraPointCloud(regionname, cameranamestobeused[i], depthimages.at(0), points);
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

void MujinVisionManager::UpdateDetectedObjects(const std::vector<DetectedObjectPtr>&detectobjectsworld, const std::string& resultstate, const bool sendtocontroller)
{
    if (detectobjectsworld.size()==0) {
        _SetStatus(TT_Command, MS_Succeeded);
    }
    if (sendtocontroller) {
        _SendDetectedObjectsToController(detectobjectsworld, resultstate);
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
    std::vector<std::string> cameranamestobeused = _GetCameraNames(regionname, cameranames);
    for (unsigned int i=0; i<cameranamestobeused.size(); i++) {
        VISIONMANAGER_LOG_DEBUG("updating " + boost::lexical_cast<std::string>(cameranamestobeused[i]));
        _SyncCamera(regionname, cameranamestobeused[i]);
    }
    // TODO: update cameras in detector
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
    boost::mutex::scoped_lock lock(_mutexDetectedInfo);
    detectedobjectsworld = _vDetectedObject;
    resultstate = _resultState;
    if (returnpoints) {
        points.clear();
        FOREACH(it, _mResultPoints) {
            points.insert(points.end(), it->second.begin(), it->second.end());
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
        throw MujinVisionException("Region "+regionname+ " is unknown!", MVE_InvalidArgument);
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
    for(std::vector<std::string>::const_iterator itr = cameranamescandidates.begin(); itr != cameranamescandidates.end(); itr++) {
        if(_mNameCameraParameters[*itr]->isColorCamera) {
            colorcameranames.push_back(*itr);
        }
    }
    return colorcameranames;
}

std::vector<std::string> MujinVisionManager::_GetDepthCameraNames(const std::string& regionname, const std::vector<std::string>& cameranames)
{
    std::vector<std::string> cameranamescandidates= _GetCameraNames(regionname, cameranames);
    std::vector<std::string> colorcameranames;
    for(unsigned int i = 0; i < cameranamescandidates.size(); ++i) {
        std::string cameraname = cameranamescandidates.at(i);
        if (_mNameCameraParameters.find(cameraname) == _mNameCameraParameters.end()) {
            throw MujinVisionException(cameraname + " is not defined in visionmanager config file.", MVE_ConfigurationFileError);
        }
        if (_mNameCameraParameters[cameraname]->isDepthCamera) {
            colorcameranames.push_back(cameraname);
        }
    }
    return colorcameranames;
}

void MujinVisionManager::_SendDetectedObjectsToController(const std::vector<DetectedObjectPtr>& detectedobjectsworld, const std::string& resultstate)
{
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
