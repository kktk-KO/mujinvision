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

namespace mujinvision {

MujinVisionManager::MujinVisionManager(ImageSubscriberManagerPtr imagesubscribermanager, DetectorManagerPtr detectormanager, const unsigned int statusport, const unsigned int commandport, const unsigned configport, const std::string& configdir)
{
    _bInitialized = false;
    _pImagesubscriberManager = imagesubscribermanager;
    _pDetectorManager = detectormanager;
    _bShutdown = false;
    _zmqcontext.reset(new zmq::context_t(6));
    _statusport = statusport;
    _commandport = commandport;
    _configport = configport;
    _configdir = configdir;
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
    std::cout << "[DEBUG] Destroying MujinVisionManager" << std::endl;
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
        std::cout << "[DEBUG]" << " Loading file from " << filename << std::endl;
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

void MujinVisionManager::_SetStatus(ManagerStatus status, const std::string& msg, const bool allowInterrupt)
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
    std::cout << "[INFO] " << GetMilliTime() << " " << _GetManagerStatusString(status) << ": " << msg << std::endl;
    boost::mutex::scoped_lock lock(_mutexStatusQueue);
    _statusQueue.push(status);
    _messageQueue.push(msg);
    _timestampQueue.push(GetMilliTime());
}

void MujinVisionManager::_SetStatusMessage(const std::string& msg)
{
    if (_statusQueue.size()>0) {
        _SetStatus(_statusQueue.front(), msg);
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
    _SetStatus(MS_Pending);
}

void MujinVisionManager::_PublishStopStatus()
{
    StatusPublisherPtr pStatusPublisher = _pStatusPublisher;
    if( !!pStatusPublisher ) {
        pStatusPublisher->Publish(_GetStatusJsonString(GetMilliTime(), _GetManagerStatusString(MS_Lost), ""));
        std::cout << "[DEBUG] " << "Stopped status publisher" << std::endl;
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
        std::cout << "[DEBUG] " << "Stopped status thread" << std::endl;
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
    std::cout << "[DEBUG] " << "Stopped command thread (port: " << port << ")." << std::endl;
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
    std::cout << "[DEBUG] " << "Stopped command server (port: " << port << ")." << std::endl;
}

void MujinVisionManager::_ExecuteConfigurationCommand(const ptree& command_pt, std::stringstream& result_ss)
{
    std::string command = command_pt.get<std::string>("command");
    if (command == "Cancel") {
        boost::mutex::scoped_lock lock(_mutexCancelCommand);
        if (_bExecutingUserCommand) { // only cancel when user command is being executed
            _bCancelCommand = true;
            _SetStatus(MS_Preempting, "", false);
        } else {
            _SetStatusMessage("No command is being excuted, do nothing.");
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
    _SetStatus(MS_Active);
    {
        // only one command thread is running, so _bExecutingUserCommand must be false at this point, and _bCancelCommand must not be true, therefore no race condition of setting _bCancelCommand from true to false
        boost::mutex::scoped_lock lock(_mutexCancelCommand);
        _bCancelCommand = false;
        _bExecutingUserCommand = true;
    }
    std::string command = command_pt.get<std::string>("command");
    if (command == "StartDetectionLoop" || command == "StopDetectionLoop" || (command.size() >= 3 && command.substr(0,3) == "Get")) {
        if (command == "StartDetectionLoop") {
            if (command_pt.count("regionname") == 0) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = command_pt.get<std::string>("regionname");
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first.", MVE_Failed);
            }

            std::vector<std::string> cameranames;
            boost::optional<const ptree&> cameranames_pt(command_pt.get_child_optional("cameranames"));
            if (!!cameranames_pt) {
                FOREACH(v, *cameranames_pt) {
                    cameranames.push_back(v->second.get<std::string>(""));
                }
            }
            double voxelsize = command_pt.get("voxelsize",0.01);
            double pointsize = command_pt.get("pointsize",0.005);
            bool ignoreocclusion = command_pt.get("ignoreocclusion",false);
            unsigned int maxage = command_pt.get("maxage",0);
            std::string obstaclename = command_pt.get("obstaclename", "__dynamicobstacle__");
            StartDetectionLoop(regionname, cameranames, voxelsize, pointsize, ignoreocclusion, maxage, obstaclename);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "StopDetectionLoop") {
            StopDetectionLoop();
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "GetCameraId") {
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first.", MVE_Failed);
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
        throw MujinVisionException("Detection thread is running, please stop it first.", MVE_Busy);
    } else {
        if (command == "Initialize") {
            if (_bInitialized) {
                _SetStatusMessage("Vision manager was initialized, de-initialize it first.");
                _DeInitialize();
            } else {
                _bInitialized = true;
            }
            Initialize(command_pt.get<std::string>("visionmanagerconfigname"),
                       command_pt.get<std::string>("detectorconfigname"),
                       command_pt.get<std::string>("imagesubscriberconfigname"),
                       command_pt.get<std::string>("mujinControllerIp", ""),
                       command_pt.get<unsigned int>("mujinControllerPort", 0),
                       command_pt.get<std::string>("mujinControllerUsernamePass"),
                       command_pt.get<std::string>("robotControllerUri"),
                       command_pt.get<unsigned int>("binpickingTaskZmqPort"),
                       command_pt.get<unsigned int>("binpickingTaskHeartbeatPort"),
                       command_pt.get<double>("binpickingTaskHeartbeatTimeout"),
                       command_pt.get<std::string>("binpickingTaskScenePk"),
                       command_pt.get<std::string>("robotname", ""),
                       command_pt.get<std::string>("targetname"),
                       command_pt.get<std::string>("tasktype","binpicking")
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
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first.", MVE_Failed);
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
            std::vector<DetectedObjectPtr> detectedobjects;
            DetectObjects(regionname, cameranames, detectedobjects, ignoreocclusion, maxage);
            result_ss << "{";
            result_ss << _GetJsonString(detectedobjects) << ", ";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "SendPointCloudObstacleToController") {
            if (command_pt.count("regionname") == 0) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = command_pt.get<std::string>("regionname");
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first.", MVE_Failed);
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
            double voxelsize = command_pt.get("voxelsize", 0.01);
            double pointsize = command_pt.get("pointsize", 0.005);
            std::string obstaclename = command_pt.get("obstaclename", "__dynamicobstacle__");
            SendPointCloudObstacleToController(regionname, cameranames, detectedobjects, voxelsize, pointsize, obstaclename);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "VisualizePointCloudOnController") {
            if (command_pt.count("regionname") == 0) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = command_pt.get<std::string>("regionname");
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first.", MVE_Failed);
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
            VisualizePointCloudOnController(regionname, cameranames, pointsize, ignoreocclusion, maxage);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "ClearVisualizationOnController") {
            if (!_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first.", MVE_Failed);
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
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first.", MVE_Failed);
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
            mujinvision::Transform regiontransform;
            DetectRegionTransform(regionname, cameranames, regiontransform, ignoreocclusion, maxage);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString(regiontransform) << ", ";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "SaveSnapshot") {
            if (!_pBinpickingTask || !_pImagesubscriberManager) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first.", MVE_Failed);
            }

            bool ignoreocclusion = command_pt.get("ignoreocclusion",false);
            unsigned int maxage = command_pt.get("maxage",0);
            SaveSnapshot(command_pt.get<std::string>("regionname"), ignoreocclusion, maxage);
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "UpdateDetectedObjects") {
            if (command_pt.count("regionname") == 0) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = command_pt.get<std::string>("regionname");
            if (!_pDetector || !_pBinpickingTask) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first.", MVE_Failed);
            }

            std::vector<DetectedObjectPtr> detectedobjects;
            boost::optional<const ptree&> detectedobjects_pt(command_pt.get_child_optional("detectedobjects"));
            if (!!detectedobjects_pt) {
                FOREACH(v, *detectedobjects_pt) {
                    detectedobjects.push_back(DetectedObjectPtr(new DetectedObject(v->second.get_child(""))));
                }
            }
            UpdateDetectedObjects(detectedobjects, command_pt.get<bool>("sendtocontroller"));
            result_ss << "{";
            result_ss << ParametersBase::GetJsonString("computationtime") << ": " << GetMilliTime()-starttime;
            result_ss << "}";
        } else if (command == "SyncRegion") {
            if (command_pt.count("regionname") == 0) {
                throw MujinVisionException("regionname is not specified.", MVE_InvalidArgument);
            }
            std::string regionname = command_pt.get<std::string>("regionname");
            if (!_pDetector || !_pBinpickingTask || !_pImagesubscriberManager) {
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first.", MVE_Failed);
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
                throw MujinVisionException("visionmanager is not initialized, please call Initialize() first.", MVE_Failed);
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
    _SetStatus(MS_Pending);
}

void MujinVisionManager::_StatusThread(const unsigned int port, const unsigned int ms)
{
    _StartStatusPublisher(port);
    std::cout << "[DEBUG] " << "Started status thread (port: " << port << ")."<< std::endl;
    std::vector<ManagerStatus> vstatus;
    std::vector<std::string> vmessage;
    std::vector<unsigned long long> vtimestamp;
    boost::shared_ptr<void> onexit = boost::shared_ptr<void>((void*)0, boost::bind(&MujinVisionManager::_PublishStopStatus, this));
    while (!_bStopStatusThread) {
        {
            boost::mutex::scoped_lock lock(_mutexStatusQueue);
            vstatus.resize(0);
            while (_statusQueue.size()>1) {
                vstatus.push_back(_statusQueue.front());
                _statusQueue.pop();
                vmessage.push_back(_messageQueue.front());
                _messageQueue.pop();
                vtimestamp.push_back(_timestampQueue.front());
                _timestampQueue.pop();
            }
            if (vstatus.size()==0) {
                vstatus.push_back(_statusQueue.front());
                vmessage.push_back(_messageQueue.front());
                vtimestamp.push_back(_timestampQueue.front());
            }
        }
        for (unsigned int i=0; i<vstatus.size(); i++) {
            _pStatusPublisher->Publish(_GetStatusJsonString(vtimestamp.at(i), _GetManagerStatusString(vstatus.at(i)), vmessage.at(i)));
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
    }
}

std::string MujinVisionManager::_GetStatusJsonString(const unsigned long long timestamp, const std::string& status, const std::string& message)
{
    std::stringstream ss;
    ss << "{";
    ss << ParametersBase::GetJsonString("timestamp") << ": " << timestamp << ", ";
    ss << ParametersBase::GetJsonString("status", status) << ", ";
    ss << ParametersBase::GetJsonString("message",message);
    ss << "}";
    return ss.str();
}

void MujinVisionManager::_CommandThread(const unsigned int port)
{
    _StartCommandServer(port);
    std::cout << "[DEBUG] " << "Started command thread (port: " << port << ")." << std::endl;
    std::string incomingmessage;
    ptree command_pt;
    std::stringstream command_ss, result_ss;
    std::string resultstr;
    while (!_mPortStopCommandThread[port]) {
        try {
            // receive message
            if( _mPortCommandServer[port]->Recv(incomingmessage) > 0 ) {
                std::cout << "[INFO] " << "Received command message: " << incomingmessage << "." << std::endl;
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
                        std::cerr << "[INFO] User requested program exit." << std::endl;
                    } else {
                        _SetStatus(MS_Preempted, "", false);
                        std::cerr << "[INFO] User interruped command execution." << std::endl;
                        result_ss << "{" << ParametersBase::GetJsonString("status", _GetManagerStatusString(MS_Preempted)) << "}";
                    }
                }
                catch (const MujinVisionException& e) {
                    std::cerr << "[ERROR] MujinVisionException " << e.message() << std::endl;
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
                    _SetStatus(MS_Aborted, e.message(), false);
                }
                catch (std::exception& e) {
                    result_ss << "{\"error\": \"" << e.what() << "\"}";
                    std::cerr << "[ERROR] unhandled exception, " << e.what() << std::endl; 
                    _SetStatus(MS_Aborted, e.what(), false);
                }

                // send output
                // TODO: verify validity
                resultstr = result_ss.str();
                boost::replace_all(resultstr, "\n", "\\n");
                _mPortCommandServer[port]->Send(resultstr);

            } else {
                // wait for command
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            }
        }
        catch (const UserInterruptException& ex) {
            _SetStatus(MS_Aborted, "", false);
            std::cerr << "[INFO] User requested program exit." << std::endl;
            throw;
        }
    }
    _StopCommandServer(port);
}

void MujinVisionManager::_StartDetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const std::string& obstaclename)
{
    if (!!_pDetectionThread && !_bStopDetectionThread) {
        _SetStatusMessage("Detection thread is already running, do nothing.");
    } else {
        _bStopDetectionThread = false;
        _pDetectionThread.reset(new boost::thread(boost::bind(&MujinVisionManager::_DetectionThread, this, regionname, cameranames, voxelsize, pointsize, ignoreocclusion, maxage, obstaclename)));
    }
}

void MujinVisionManager::_StopDetectionThread()
{
    _SetStatusMessage("Stopping detectoin thread.");
    if (!_bStopDetectionThread) {
        _bStopDetectionThread = true;
        if (!!_pDetectionThread) {
            _pDetectionThread->join();
            _pDetectionThread.reset();
            _SetStatusMessage("Stopped detection thread.");
        }
        _bStopDetectionThread = false; // reset so that _GetImage works properly afterwards
    }
}

void MujinVisionManager::_DetectionThread(const std::string& regionname, const std::vector<std::string>& cameranames, const double voxelsize, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const std::string& obstaclename)
{
    uint64_t time0;
    while (!_bStopDetectionThread) {
        time0 = GetMilliTime();
        // update picked positions
        if (_bStopDetectionThread) {
            break;
        }
        Vector weights(2,2,1); // prioritize XY over Z
        if (_pVisionServerParameters->clearRadius > 0) {
            BinPickingTaskResource::ResultGetPickedPositions pickedpositions;
            try {
                _pBinpickingTask->GetPickedPositions(pickedpositions,"m");
            }
            catch(const std::exception& ex) {
                std::cerr << "[WARN] failed to get picked positions from mujin controller: " << ex.what() << std::endl;
                continue;
            }
            const unsigned int numPickedPositions = pickedpositions.transforms.size();
            std::cout << "[DEBUG] Got " << numPickedPositions << " picked positions" << std::endl;

            // remove saved detection results near picked positions
            //bool pickedRecently = false;
            for (unsigned int i=0; i<numPickedPositions; i++) {
                unsigned long long timestamp = pickedpositions.timestamps[i];
                // if timestamp is known
                if (_sTimestamp.find(timestamp)!=_sTimestamp.end()) {
                    if (GetMilliTime() - timestamp < _pVisionServerParameters->timeToIgnore) {
                        std::cout << "[DEBUG] Just picked up an object, keep ignoring detection in this region (" << (_pVisionServerParameters->timeToIgnore - (GetMilliTime()-timestamp)) << " ms left)." << std::endl;
                    } else {
                        //std::cout << "Already cleared picked position at timestamp " << (GetMilliTime() - timestamp) << " ms ago." << std::endl;
                        continue;
                    }
                } else { // for new timestamp
                    _sTimestamp.insert(timestamp);
                    std::cout << "[DEBUG] Added timestamp " << timestamp << " to cleared set." << std::endl;
                    //pickedRecently = true;
                }
                std::cout << "[DEBUG] An object was picked " << (GetMilliTime()-timestamp) << " ms ago, clear known detection results that are nearby." << std::endl;
                Transform transform = _GetTransform(pickedpositions.transforms[i]);
                Vector position = transform.trans;

                if (_vDetectedInfo.size()>0) {
                    for (int j = _vDetectedInfo.size() - 1; j >= 0; j--) { // have to iterate from the end to remove items from the vectors
                        double dist = std::sqrt(((position-_vDetectedInfo.at(j).meanPosition)*weights).lengthsqr3());
                        std::cout << "[DEBUG] Part " << j << " distance to object " << dist << std::endl;
                        if (dist < _pVisionServerParameters->clearRadius) {
                            std::cout << "[DEBUG] Part " << j << " is within the clear radius of picked position, clear its records." << std::endl;
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

        std::vector<DetectedObjectPtr> detectedobjects;
        try {
            DetectObjects(regionname, cameranames, detectedobjects, ignoreocclusion, maxage);
        }
        catch(const std::exception& ex) {
            std::cerr << "[ERROR] caugh unhandled exception while debugging: " << ex.what() << std::endl;
            continue;
        }

        // process results
        if (_bStopDetectionThread) {
            break;
        }

        std::vector<DetectedObjectPtr> newdetectedobjects;

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
                        std::cout << "[DEBUG] Upside-down detection (" << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << ", " << rotation[3] << "), flip rotation." << std::endl;
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
                    std::cout << "[DEBUG] Part " << minIndex << " is known (minDist " << minDist << "), updating its mean position averaging " << numDetections << " detections." << std::endl;

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
                        std::cout << "[DEBUG] Part " << i << " has not been seen for " << _pVisionServerParameters->timeToRemember << " ms, removing its records." << std::endl;
                        _vDetectedInfo.erase(_vDetectedInfo.begin()+i);
                    }
                }
            }

            // create new results
            if (detectedobjects.size()>0) {
                for (unsigned int i=0; i<_vDetectedInfo.size(); i++) {
                    Transform transform;
                    transform.trans = _vDetectedInfo.at(i).meanPosition;
                    transform.rot = _vDetectedInfo.at(i).meanRotation;
                    DetectedObjectPtr obj(new DetectedObject(detectedobjects[0]->name, transform, _vDetectedInfo.at(i).confidences.at(0), _vDetectedInfo.at(i).timestamp));
                    newdetectedobjects.push_back(obj);
                    //obj->Print();
                }
            }
        } else {
            newdetectedobjects = detectedobjects;
        }
        // send results to mujin controller
        if (_bStopDetectionThread) {
            break;
        }
        _UpdateEnvironmentState(regionname, cameranames, newdetectedobjects, voxelsize, pointsize, obstaclename);
        // visualize results
        if (_bStopDetectionThread) {
            break;
        }
        std::cout << "[DEBUG] Cycle time: " << (GetMilliTime() - time0)/1000.0f << " secs" << std::endl;
        std::cout << "[DEBUG] ------------------------" << std::endl;
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
    std::cout << "[DEBUG] setting camera transform to: " << std::endl;
    std::cout << _GetString(_mNameCamera[cameraname]->GetWorldTransform());
}

void MujinVisionManager::_SyncRegion(const std::string& regionname)
{
    if (_mNameRegion.find(regionname) == _mNameRegion.end()) {
        throw MujinVisionException("Region "+regionname+ " is unknown!", MVE_InvalidArgument);
    }
    _mNameRegion[regionname]->SetWorldTransform(_GetTransform(regionname));
    std::cout << "[DEBUG] setting region transform to: " << std::endl;
    std::cout << _GetString(_mNameRegion[regionname]->GetWorldTransform());
    // update globalroi3d from mujin controller
    if (!_mNameRegion[regionname]->pRegionParameters->bInitializedRoi) {
        std::cout << "[DEBUG] Computing globalroi3d from mujin controller." << std::endl;
        // get axis aligned bounding box for region
        BinPickingTaskResource::ResultAABB raabb;
        _pBinpickingTask->GetAABB(regionname, raabb, "m");
        if (raabb.extents.size()!=3 || raabb.pos.size()!=3) {
            throw MujinVisionException("ResultAABB from Mujin controller is invalid!", MVE_ControllerError);
        }

        Transform B_T_O = _GetTransform(regionname).inverse();
        Vector mins = B_T_O *(Vector(raabb.pos[0]-raabb.extents[0],raabb.pos[1]-raabb.extents[1],raabb.pos[2]-raabb.extents[2]));
        Vector maxs = B_T_O *(Vector(raabb.pos[0]+raabb.extents[0],raabb.pos[1]+raabb.extents[1],raabb.pos[2]+raabb.extents[2]));

        _mNameRegion[regionname]->pRegionParameters->minx = std::min(mins[0], maxs[0]);
        _mNameRegion[regionname]->pRegionParameters->maxx = std::max(mins[0], maxs[0]);
        _mNameRegion[regionname]->pRegionParameters->miny = std::min(mins[1], maxs[1]);
        _mNameRegion[regionname]->pRegionParameters->maxy = std::max(mins[1], maxs[1]);
        _mNameRegion[regionname]->pRegionParameters->minz = std::min(mins[2], maxs[2]);
        _mNameRegion[regionname]->pRegionParameters->maxz = std::max(mins[2], maxs[2]);
        _mNameRegion[regionname]->pRegionParameters->bInitializedRoi = true;
        std::cout << _mNameRegion[regionname]->pRegionParameters->GetJsonString() << std::endl;
    }
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

ColorImagePtr MujinVisionManager::_GetColorImage(const std::string& regionname, const std::string& cameraname, const bool ignoreocclusion, const unsigned int maxage, const unsigned int waitinterval)
{
    uint64_t start0=GetMilliTime();
    ColorImagePtr colorimage;
    unsigned long long timestamp, endtimestamp;
    bool isoccluding = true;
    while (!_bCancelCommand && !_bShutdown && !_bStopDetectionThread) {
        colorimage = _pImagesubscriberManager->GetColorImage(cameraname,timestamp, endtimestamp);
        if (_bStopDetectionThread) {
            break;
        } else {
            if (!colorimage) {
                std::cerr << "[WARN] Could not get color image for camera: " << cameraname << ", try again in  " << waitinterval << " ms." << std::endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                continue;
            } else {
                if (GetMilliTime() < timestamp) {
                    std::cerr << "[ERROR]: Image timestamp is in the future, please ensure that clocks are synchronized." << std::endl;
                    continue;
                }
                else if (maxage>0 && GetMilliTime() - timestamp > maxage) {
                    std::cerr << "[WARN] Image is more than " << maxage << " ms old (" << GetMilliTime()-timestamp << "), will try to get again." << std::endl;
                    continue;
                } else {
                    std::cout << "[DEBUG] Got color image that is " << GetMilliTime()-timestamp << " ms old, took " << (GetMilliTime()-start0)/1000.0f << std::endl;
                    if (!ignoreocclusion) {
                        try {
                            _pBinpickingTask->IsRobotOccludingBody(regionname, cameraname, timestamp, endtimestamp, isoccluding);
                        } catch (...) {
                            std::cerr << "[WARN] Failed to check for occlusion, will try again in " << waitinterval << " ms." << std::endl;
                            boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                            continue;
                        }
                    } else {
                        isoccluding = false;
                    }
                    if (!isoccluding) {
                        break;
                    } else {
                        std::cerr << "[WARN] Region is occluded in the view of " << cameraname << ", will try again." << std::endl;
                    }
                }
            }
        }
    }
    if (!colorimage) {
        std::cerr << "[WARN] returning empty image. _bCancelCommand " << int(_bCancelCommand) << " _bShutdown " << int(_bShutdown) << " _bStopDetectionThread " << int(_bStopDetectionThread) << std::endl;
    }
    return colorimage;
}

DepthImagePtr MujinVisionManager::_GetDepthImage(const std::string& regionname, const std::string& cameraname, const bool ignoreocclusion, const unsigned int maxage, const unsigned int waitinterval)
{
    uint64_t start0 = GetMilliTime();
    DepthImagePtr depthimage;
    unsigned long long starttime, endtime;
    bool isoccluding = true;
    while (!_bCancelCommand && !_bShutdown && !_bStopDetectionThread) {
        depthimage = _pImagesubscriberManager->GetDepthImage(cameraname, _numDepthImagesToAverage, starttime, endtime);
        if (_bStopDetectionThread) {
            break;
        } else {
            if (!depthimage) {
                std::cerr << "[WARN] Could not get depth image for camera: " << cameraname << ", try again in  " << waitinterval << " ms." << std::endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                continue;
            } else {
                if (GetMilliTime() < starttime) {
                    std::cerr << "[ERROR]: Image timestamp is in the future, please ensure that clocks are synchronized." << std::endl;
                    continue;
                }
                else if (maxage>0 && GetMilliTime()-starttime>maxage) {
                    std::cerr << "[WARN] Image is more than " << maxage << " ms old (" << GetMilliTime()-starttime << "), will try to get again." << std::endl;
                    continue;
                } else {
                    std::cout << "[DEBUG] Got depth image that is " << GetMilliTime()-starttime << " ms old, took " << (GetMilliTime()-start0)/1000.0f << std::endl;
                    if (!ignoreocclusion) {
                        try {
                            _pBinpickingTask->IsRobotOccludingBody(regionname, cameraname, starttime, endtime, isoccluding);
                        } catch (...) {
                            std::cerr << "[WARN] Failed to check for occlusion, will try again in " << waitinterval << " ms." << std::endl;
                            boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                            continue;
                        }
                    } else {
                        isoccluding = false;
                    }
                    if (!isoccluding) {
                        break;
                    } else {
                        std::cerr << "[WARN] Region is occluded in the view of " << cameraname << ", will try again." << std::endl;
                    }
                }
            }
        }
    }
    if (!depthimage) {
        std::cerr << "[WARN] returning empty image. _bCancelCommand " << int(_bCancelCommand) << " _bShutdown " << int(_bShutdown) << " _bStopDetectionThread " << int(_bStopDetectionThread) << std::endl;
    }
    return depthimage;
}

unsigned int MujinVisionManager::_GetColorImages(const std::string& regionname, const std::vector<std::string>& cameranames, std::vector<ColorImagePtr>& colorimages, const bool ignoreocclusion, const unsigned int maxage, const unsigned int waitinterval)
{
    uint64_t start0 = GetMilliTime();
    colorimages.resize(0);
    unsigned long long timestamp, endtimestamp;
    bool isoccluding = true;
    std::string cameraname;
    while (!_bCancelCommand && !_bShutdown) {
        cameraname = cameranames.at(colorimages.size());
        ColorImagePtr colorimage = _pImagesubscriberManager->GetColorImage(cameraname, timestamp, endtimestamp);
        if (_bStopDetectionThread) {
            break;
        } else {
            if (!colorimage) {
                std::cerr << "[WARN] Could not get color image for camera: " << cameraname << ", will try again in  " << waitinterval << " ms." << std::endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                continue;
            } else {
                if (GetMilliTime() < timestamp) {
                    std::cerr << "[ERROR]: Image timestamp is in the future, please ensure that clocks are synchronized." << std::endl;
                    continue;
                }
                else if (maxage>0 && GetMilliTime()-timestamp>maxage) {
                    std::cerr << "[WARN] Image is more than " << maxage << " ms old (" << GetMilliTime()-timestamp << "), will try to get again." << std::endl;
                    if (colorimages.size()>0) {
                        std::cerr << "[WARN] One of the color images is more than " << maxage << " ms old (" << GetMilliTime()-timestamp << "), start over." << std::endl;
                        colorimages.resize(0); // need to start over, all color images need to be equally new
                    }
                    continue;
                } else {
                    std::cout << "[DEBUG] Got color image that is " << GetMilliTime()-timestamp << " ms old, took " << (GetMilliTime()-start0)/1000.0f << std::endl;
                    if (!ignoreocclusion) {
                        try {
                            _pBinpickingTask->IsRobotOccludingBody(regionname, cameraname, timestamp, endtimestamp, isoccluding);
                        } catch (...) {
                            std::cerr << "[WARN] Failed to check for occlusion, will try again in " << waitinterval << " ms." << std::endl;
                            boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                            continue;
                        }
                    } else {
                        isoccluding = false;
                    }
                    if (!isoccluding) {
                        colorimages.push_back(colorimage);
                        if (colorimages.size() == cameranames.size()) {
                            // got one image for each camera, exit
                            break;
                        } else {
                            // move on to get image for the next camera
                            continue;
                        }
                    } else {
                        std::cerr << "[WARN] Region is occluded in the view of " << cameraname << ", will try again." << std::endl;
                        continue;
                    }
                }
            }
        }
    }
    if (colorimages.size()<cameranames.size()) {
        std::cerr << "[DEBUG] got " << colorimages.size() << "/" << cameranames.size() << " color images. _bCancelCommand " << int(_bCancelCommand) << " _bShutdown " << int(_bShutdown) << " _bStopDetectionThread " << int(_bStopDetectionThread) << std::endl;
    }
    return colorimages.size();
}

unsigned int MujinVisionManager::_GetDepthImages(const std::string& regionname, const std::vector<std::string>& cameranames, std::vector<DepthImagePtr>& depthimages, const bool ignoreocclusion, const unsigned int maxage, const unsigned int waitinterval)
{
    uint64_t start0 = GetMilliTime();
    depthimages.resize(0);
    unsigned long long starttime, endtime;
    bool isoccluding = true;
    std::string cameraname;
    while (!_bCancelCommand && !_bShutdown) {
        cameraname = cameranames.at(depthimages.size());
        DepthImagePtr depthimage = _pImagesubscriberManager->GetDepthImage(cameraname, _numDepthImagesToAverage, starttime, endtime);
        if (_bStopDetectionThread) {
            break;
        } else {
            if (!depthimage) {
                std::cerr << "[WARN] Could not get depth image for camera: " << cameraname << ", will try again in  " << waitinterval << " ms." << std::endl;
                boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                continue;
            } else {
                if (GetMilliTime() < starttime) {
                    std::cerr << "[ERROR]: Image timestamp is in the future, please ensure that clocks are synchronized." << std::endl;
                    continue;
                }
                else if (maxage>0 && GetMilliTime()-starttime>maxage) {
                    std::cerr << "[WARN] Image is more than " << maxage << " ms old (" << GetMilliTime()-starttime << "), will try to get again." << std::endl;
                    if (depthimages.size()>0) {
                        std::cerr << "[WARN] One of the depth images is more than " << maxage << " ms old (" << GetMilliTime()-starttime << "), start over." << std::endl;
                        depthimages.resize(0); // need to start over, all color images need to be equally new
                    }
                    continue;
                } else {
                    std::cout << "[DEBUG] Got depth image that is " << GetMilliTime()-starttime << " ms old, took " << (GetMilliTime()-start0)/1000.0f << std::endl;
                    if (!ignoreocclusion) {
                        try {
                            _pBinpickingTask->IsRobotOccludingBody(regionname, cameraname, starttime, endtime, isoccluding);
                        } catch (...) {
                            std::cerr << "[WARN] Failed to check for occlusion, will try again in " << waitinterval << " ms." << std::endl;
                            boost::this_thread::sleep(boost::posix_time::milliseconds(waitinterval));
                            continue;
                        }
                    } else {
                        isoccluding = false;
                    }
                    if (!isoccluding) {
                        depthimages.push_back(depthimage);
                        if (depthimages.size() == cameranames.size()) {
                            // got one image for each camera, exit
                            break;
                        } else {
                            // move on to get image for the next camera
                            continue;
                        }
                    } else {
                        std::cerr << "[WARN] Region is occluded in the view of " << cameraname << ", will try again." << std::endl;
                        continue;
                    }
                } 
            }
        }
    }
    if (depthimages.size()<cameranames.size()) {
        std::cerr << "[DEBUG] got " << depthimages.size() << "/" << cameranames.size() << " depth images. _bCancelCommand " << int(_bCancelCommand) << " _bShutdown " << int(_bShutdown) << " _bStopDetectionThread " << int(_bStopDetectionThread) << std::endl;
    }
    return depthimages.size();
}

void MujinVisionManager::Initialize(const std::string& visionmanagerconfigname, const std::string& detectorconfigname, const std::string& imagesubscriberconfigname, const std::string& controllerIp, const unsigned int controllerPort, const std::string& controllerUsernamePass, const std::string& robotControllerUri, const unsigned int binpickingTaskZmqPort, const unsigned int binpickingTaskHeartbeatPort, const double binpickingTaskHeartbeatTimeout, const std::string& binpickingTaskScenePk, const std::string& robotname, const std::string& targetname, const std::string& tasktype)
{
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
    url_ss << "http://"<< controllerIp << ":" << controllerPort;
    ControllerClientPtr controller = CreateControllerClient(controllerUsernamePass, url_ss.str());
    _pControllerClient = controller;
    _SetStatusMessage("Connected to mujin controller at " + url_ss.str());
    SceneResourcePtr scene(new SceneResource(controller,binpickingTaskScenePk));
    _pSceneResource = scene;
    _pBinpickingTask = scene->GetOrCreateBinPickingTaskFromName_UTF8(tasktype+std::string("task1"), tasktype, TRO_EnableZMQ);
    _pBinpickingTask->Initialize(robotControllerUri, binpickingTaskZmqPort, binpickingTaskHeartbeatPort, _zmqcontext, binpickingTaskHeartbeatTimeout);

    // sync regions
    _SetStatusMessage("Syncing regions.");
    std::string regionname;
    FOREACH(it, _mNameRegion) {
        regionname = it->first;
        _SyncRegion(regionname);
    }

    // set up cameras
    _SetStatusMessage("Setting up cameras.");
    FOREACH(it, _mNameCameraParameters) {
        std::string cameraname = it->first;
        CameraParametersPtr pcameraparameters = it->second;
        // std::vector<RobotResource::AttachedSensorResourcePtr> attachedsensors;
        // utils::GetAttachedSensors(_pControllerClient, _pSceneResource, name, attachedsensors);
        // RobotResource::AttachedSensorResource::SensorData sensordata = attachedsensors.at(0)->sensordata; // TODO: using first attached sensor for now
        RobotResource::AttachedSensorResource::SensorData sensordata;
        std::string camerabodyname,sensorname;
        _ParseCameraName(cameraname, camerabodyname, sensorname);

        utils::GetSensorData(_pControllerClient, _pSceneResource, camerabodyname, sensorname, sensordata);

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
        regionname = itr->first;
        std::map<std::string, CameraPtr> mNameColorCamera, mNameDepthCamera;
        RegionPtr region = _mNameRegion[regionname];
        std::string cameraname;
        CameraParametersPtr pcameraparameters;
        for (unsigned int i=0; i<region->pRegionParameters->cameranames.size(); ++i) {
            cameraname = region->pRegionParameters->cameranames.at(i);
            pcameraparameters = _mNameCamera[cameraname]->pCameraParameters;
            if (std::find(syncedcamera.begin(), syncedcamera.end(), cameraname) == syncedcamera.end()) {
                _SyncCamera(regionname, cameraname);
                syncedcamera.push_back(cameraname);
            } else {
                throw MujinVisionException("does not support same camera mapped to more than one region.", MVE_CommandNotSupported);
            }
            if (pcameraparameters->isColorCamera) {
                _SetStatusMessage("Loading parameters for color camera " + cameraname +" for region " + regionname +".");
                mNameColorCamera[cameraname] = _mNameCamera[cameraname];
            }
            if (pcameraparameters->isDepthCamera) {
                _SetStatusMessage("Loading parameters for depth camera " + cameraname +" for region " + regionname +".");
                mNameDepthCamera[cameraname] = _mNameCamera[cameraname];
            }
        }
        _mRegionColorCameraMap[regionname] = mNameColorCamera;
        _mRegionDepthCameraMap[regionname] = mNameDepthCamera;
    }

    // set up subscribers
    _SetStatusMessage("Loading subscriber configuration.");
    // load subscriber configuration
    std::string imagesubscriberconfig;
    _LoadConfig(_GetConfigFileName("imagesubscriber", imagesubscriberconfigname), imagesubscriberconfig);
    _imagesubscriberconfig = imagesubscriberconfig;
    std::stringstream imagesubscriberconfigss;
    imagesubscriberconfigss << imagesubscriberconfig;
    read_json(imagesubscriberconfigss, pt);

    // set up image manager
    _SetStatusMessage("Setting up image manager.");
    for (unsigned int i=0; i<_pVisionServerParameters->streamerConnections.size(); i++) {
        _vSubscribers.push_back(_pImagesubscriberManager->CreateImageSubscriber(_pVisionServerParameters->streamerConnections[i]->ip, _pVisionServerParameters->streamerConnections[i]->port, pt.get_child("zmq_subscriber")));
    }
    _pImagesubscriberManager->Initialize(_mNameCamera, _vSubscribers);

    _numDepthImagesToAverage = pt.get_child("zmq_subscriber").get<unsigned int>("num_depth_images_to_average"); // assuming each message has one depth image


    // set up detectors
    _SetStatusMessage("Setting up detector.");
    std::string detectorconfig;
    _LoadConfig(_GetConfigFileName("detector", detectorconfigname), detectorconfig);
    _detectorconfig = detectorconfig;
    std::stringstream detectorconfigss;
    detectorconfigss << detectorconfig;
    read_json(detectorconfigss, pt);
    _pDetector = _pDetectorManager->CreateObjectDetector(pt.get_child("object"),pt.get_child("detection"), _mNameRegion, _mRegionColorCameraMap, _mRegionDepthCameraMap, boost::bind(&MujinVisionManager::_SetStatusMessage,this,_1));
    _targetname = targetname;
    _SetStatus(MS_Succeeded);
}

void MujinVisionManager::_DeInitialize()
{
    _StopDetectionThread();

    std::string regionname;
    if (!!_pDetector) {
        _pDetector.reset();
        std::cout << "[DEBUG] reset detector" << std::endl;
    }
    if (!!_pImagesubscriberManager) {
        _pImagesubscriberManager->DeInitialize();
        _vSubscribers.clear();
        std::cout << "[DEBUG] cleared _vSubscribers" << std::endl;
    }
    _SetStatusMessage("DeInitialized vision manager.");
}

void MujinVisionManager::DetectObjects(const std::string& regionname, const std::vector<std::string>&cameranames, std::vector<DetectedObjectPtr>&detectedobjects, const bool ignoreocclusion, const unsigned int maxage)
{
    uint64_t starttime = GetMilliTime();

    std::vector<std::string> colorcameranames = _GetColorCameraNames(regionname, cameranames);
    std::vector<std::string> depthcameranames = _GetDepthCameraNames(regionname, cameranames);

    // set up images
    std::vector<ColorImagePtr> colorimages;
    _GetColorImages(regionname, colorcameranames, colorimages, ignoreocclusion, maxage);
    std::vector<DepthImagePtr> depthimages;
    _GetDepthImages(regionname, depthcameranames, depthimages, ignoreocclusion, maxage);
    std::cout << "[DEBUG] Getting images took " << ((GetMilliTime() - starttime) / 1000.0f) << std::endl;
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
        _pDetector->DetectObjects(regionname, colorcameranames, depthcameranames, detectedobjects);
        std::stringstream msgss;
        msgss << "Detected " << detectedobjects.size() << " objects. Took " << (GetMilliTime()-starttime)/1000.0f << " seconds.";
        _SetStatusMessage(msgss.str());
    }
    _SetStatus(MS_Succeeded);
}

void MujinVisionManager::StartDetectionLoop(const std::string& regionname, const std::vector<std::string>&cameranames,const double voxelsize, const double pointsize, const bool ignoreocclusion, const unsigned int maxage, const std::string& obstaclename)
{
    _StartDetectionThread(regionname, cameranames, voxelsize, pointsize, ignoreocclusion, maxage, obstaclename);
    _SetStatus(MS_Succeeded);
}

void MujinVisionManager::StopDetectionLoop()
{
    _StopDetectionThread();
    _SetStatus(MS_Succeeded);
}

void MujinVisionManager::SendPointCloudObstacleToController(const std::string& regionname, const std::vector<std::string>&cameranames, const std::vector<DetectedObjectPtr>& detectedobjectsworld, const double voxelsize, const double pointsize, const std::string obstaclename)
{
    std::vector<std::string> depthcameranames = _GetDepthCameraNames(regionname, cameranames);
    // set up images
    std::vector<DepthImagePtr> depthimages;
    bool ignoreocclusion = true;
    unsigned int maxage = 0;
    _GetDepthImages(regionname, depthcameranames, depthimages, ignoreocclusion, maxage);
    if (depthimages.size() == depthcameranames.size()) {
        for (size_t i=0; i<depthimages.size(); ++i) {
            std::string cameraname = depthcameranames.at(i);
            CameraPtr camera= _mNameCamera[cameraname];
            _pDetector->SetDepthImage(cameraname, depthimages.at(i));
            // get point cloud obstacle
            std::vector<Real> points;
            _pDetector->GetPointCloudObstacle(regionname, cameraname, detectedobjectsworld, points, voxelsize);
            std::stringstream ss;
            ss <<"Sending over " << (points.size()/3) << " points from " << cameraname << ".";
            _SetStatusMessage(ss.str());
            _pBinpickingTask->AddPointCloudObstacle(points, pointsize, obstaclename);
        }
    }
    _SetStatus(MS_Succeeded);
}

void MujinVisionManager::_UpdateEnvironmentState(const std::string& regionname, const std::vector<std::string>&cameranames, const std::vector<DetectedObjectPtr>& detectedobjectsworld, const double voxelsize, const double pointsize, const std::string& obstaclename)
{
    std::vector<mujinclient::Transform> transformsworld;
    std::vector<std::string> confidences;
    std::vector<uint64_t> timestamps;
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
        timestamps.push_back(detectedobjectsworld[i]->timestamp);
    }
    std::vector<std::string> cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);
    std::vector<Real> totalpoints;
    for(unsigned int i=0; i<cameranamestobeused.size(); i++) {
        std::string cameraname = cameranamestobeused[i];
        // get point cloud obstacle
        std::vector<Real> points;
        _pDetector->GetPointCloudObstacle(regionname, cameraname, detectedobjectsworld, points, voxelsize);
        totalpoints.insert(totalpoints.end(), points.begin(), points.end());
    }
    std::stringstream ss;
    if (totalpoints.size()>0) {
        _pBinpickingTask->UpdateEnvironmentState(_targetname, transformsworld, confidences, timestamps, totalpoints, pointsize, obstaclename,"m");
        ss << "Updating environment with " << detectedobjectsworld.size() << " detected objects and " << (totalpoints.size()/3) << " points.";
    } else {
        ss << "Got 0 points, something is wrong with the streamer. Is robot occluding the camera?" << std::endl;
    }
    _SetStatusMessage(ss.str());

}

void MujinVisionManager::DetectRegionTransform(const std::string& regionname, const std::vector<std::string>& cameranames, mujinvision::Transform& regiontransform, const bool ignoreocclusion, const unsigned int maxage)
{
    // TODO: use actual cameras
    std::string colorcameraname = _GetColorCameraNames(regionname, cameranames).at(0);
    std::string depthcameraname = _GetDepthCameraNames(regionname, cameranames).at(0);
    CameraPtr colorcamera = _mNameCamera[colorcameraname];
    CameraPtr depthcamera = _mNameCamera[depthcameraname];
    ColorImagePtr originalcolorimage = _GetColorImage(regionname, colorcameraname);
    DepthImagePtr depthimage = _GetDepthImage(regionname, depthcameraname, ignoreocclusion, maxage);
    _pDetector->SetColorImage(colorcameraname, originalcolorimage);
    _pDetector->SetDepthImage(depthcameraname, depthimage);

    mujinvision::Transform regiontransform0 = regiontransform;
    _pDetector->DetectRegionTransform(regionname, colorcameraname, depthcameraname, regiontransform);

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
    _SetStatus(MS_Succeeded);
}

void MujinVisionManager::VisualizePointCloudOnController(const std::string& regionname, const std::vector<std::string>&cameranames, const double pointsize, const bool ignoreocclusion, const unsigned int maxage)
{
    std::vector<std::string> cameranamestobeused = _GetDepthCameraNames(regionname, cameranames);
    std::vector<std::vector<Real> > pointslist;
    std::vector<std::string> names;
    std::vector<double> points;
    for (unsigned int i=0; i<cameranamestobeused.size(); i++) {
        points.resize(0);
        std::string cameraname = cameranamestobeused.at(i);
        CameraPtr camera = _mNameCamera[cameraname];
        _pDetector->GetCameraPointCloud(regionname, cameranamestobeused[i], _GetDepthImage(regionname, cameraname, ignoreocclusion, maxage), points);
        if (points.size()>0) {
            pointslist.push_back(points);
            std::stringstream name_ss;
            name_ss << "__pointcloud_" << i;
            names.push_back(name_ss.str());
        }
    }
    _pBinpickingTask->VisualizePointCloud(pointslist, pointsize*1000.0f, names); // need to convert pointsize to millimeter
    _SetStatus(MS_Succeeded);
}

void MujinVisionManager::ClearVisualizationOnController()
{
    _pBinpickingTask->ClearVisualization();
    _SetStatus(MS_Succeeded);
}

void MujinVisionManager::SaveSnapshot(const std::string& regionname, const bool ignoreocclusion, const unsigned int maxage)
{
    std::vector<std::string> cameranames;
    std::vector<std::string> cameranamestobeused = _GetCameraNames(regionname, cameranames);
    FOREACH(iter,_mRegionColorCameraMap[regionname]) {
        std::string colorcameraname = iter->first;
        std::string camerabodyname, sensorname;
        _ParseCameraName(colorcameraname, camerabodyname, sensorname);
       CameraPtr colorcamera = _mNameCamera[colorcameraname];
        if (std::find(cameranamestobeused.begin(), cameranamestobeused.end(), colorcameraname) != cameranamestobeused.end()) {
            std::stringstream filename_ss;
            filename_ss << camerabodyname << "-" << sensorname << "-2d-" << GetMilliTime() << ".png";
            ColorImagePtr colorimage;
            colorimage = _GetColorImage(regionname, colorcameraname, ignoreocclusion, maxage);
            if (!!colorimage) {
                _pImagesubscriberManager->WriteColorImage(colorimage, filename_ss.str());
            } else {
                std::cerr << "[ERROR] Failed to get colorimage, please try again." << std::endl;
            }
        }
    }
    FOREACH(iter,_mRegionDepthCameraMap[regionname]) {
        std::string depthcameraname = iter->first;
        std::string camerabodyname, sensorname;
        _ParseCameraName(depthcameraname, camerabodyname, sensorname);
        CameraPtr depthcamera = _mNameCamera[depthcameraname];
        if (std::find(cameranamestobeused.begin(), cameranamestobeused.end(), depthcameraname) != cameranamestobeused.end()) {
            std::stringstream filename_ss;
            filename_ss << camerabodyname << "-" << sensorname << "-3d-" << GetMilliTime() << ".pcd";
            DepthImagePtr depthimage;
            depthimage = _GetDepthImage(regionname, depthcameraname, ignoreocclusion, maxage);
            if (!!depthimage) {
                _pImagesubscriberManager->WriteDepthImage(depthimage, filename_ss.str());
            } else {
                std::cerr << "[ERROR] Failed to get depthimage, please try again." << std::endl;
            }
        }
    }
    _SetStatus(MS_Succeeded);
}

void MujinVisionManager::UpdateDetectedObjects(const std::vector<DetectedObjectPtr>&detectobjectsworld, const bool sendtocontroller)
{
    if (detectobjectsworld.size()==0) {
        _SetStatus(MS_Succeeded);
    }
    if (sendtocontroller) {
        _SendDetectedObjectsToController(detectobjectsworld);
    }
    _SetStatus(MS_Succeeded);
}

void MujinVisionManager::SyncRegion(const std::string& regionname)
{
    _SyncRegion(regionname);
    _SetStatus(MS_Succeeded);
}

void MujinVisionManager::SyncCameras(const std::string& regionname, const std::vector<std::string>&cameranames)
{
    std::vector<std::string> cameranamestobeused = _GetCameraNames(regionname, cameranames);
    for (unsigned int i=0; i<cameranamestobeused.size(); i++) {
        std::cout << "[DEBUG] updating " << cameranamestobeused[i] << std::endl;
        _SyncCamera(regionname, cameranamestobeused[i]);
    }
    // TODO: update cameras in detector
    _SetStatus(MS_Succeeded);
}

void MujinVisionManager::GetCameraId(const std::string& cameraname, std::string& cameraid)
{
    if (_mNameCameraParameters.find(cameraname) == _mNameCameraParameters.end()) {
        throw MujinVisionException(cameraname + " is not defined in visionmanager config file.", MVE_ConfigurationFileError);
    }
    cameraid = _mNameCameraParameters[cameraname]->id;
    _SetStatus(MS_Succeeded);
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
    for(std::vector<std::string>::const_iterator itr = cameranamescandidates.begin(); itr != cameranamescandidates.end(); itr++) {
        if (_mNameCameraParameters.find(*itr) == _mNameCameraParameters.end()) {
            throw MujinVisionException(*itr + " is not defined in visionmanager config file.", MVE_ConfigurationFileError);
        }
        if (_mNameCameraParameters[*itr]->isDepthCamera) {
            colorcameranames.push_back(*itr);
        }
    }
    return colorcameranames;
}

void MujinVisionManager::_SendDetectedObjectsToController(const std::vector<DetectedObjectPtr>& detectedobjectsworld)
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
    _pBinpickingTask->UpdateObjects(detectedobjectsworld[0]->name, transformsworld, confidences,"m");
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
        DetectedObjectPtr detectedobj(new DetectedObject(name, G_T_A, detectedobjectsfrom.at(i)->confidence, detectedobjectsfrom.at(i)->timestamp));
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
