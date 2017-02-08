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
/** \file detectormanager.h
    \brief Public headers of DetectorManager.
 */
#ifndef MUJIN_VISION_DETECTOR_MANAGER_H
#define MUJIN_VISION_DETECTOR_MANAGER_H

#include "detector.h"
#include <mujincontrollerclient/mujinzmq.h>

namespace mujinvision {

class MUJINVISION_API DetectorManager
{
public:
    DetectorManager() {
    }

    virtual ~DetectorManager(){
    }

    /** Creates object detector.
        \param detectorconfig json string describing detection parameters
        \param mNameRegion map to detection regions from region names
        \param mRegionnameCameramap regionname -> cameraname -> camera
        \param setstatusfn function the detector can use to set status
        \param extraInitializationOptions extra initialization options that can override the values in the contructor
     */
    virtual ObjectDetectorPtr CreateObjectDetector(const std::string& detectorconfig, const std::string& targetname, const std::map<std::string, RegionPtr >& mNameRegion, const std::map<std::string, std::map<std::string, CameraPtr> >& mRegionnameCameramap, const boost::function<void(const std::string& msg, const std::string& err)>& setstatusfn, const std::map<std::string, std::string>& extraInitializationOptions = std::map<std::string, std::string>(), const mujinzmq::CheckPreemptFn& preemptfn=mujinzmq::CheckPreemptFn()) = 0;
};

typedef boost::shared_ptr<DetectorManager> DetectorManagerPtr;
typedef boost::shared_ptr<DetectorManager const> DetectorManagerConstPtr;
typedef boost::weak_ptr<DetectorManager> DetectorManagerWeakPtr;

} // namespace mujinvision
#endif
