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
/** \file imagesubscribermanager.h
    \brief Public headers of ImageSubscriberManager.
 */
#ifndef MUJIN_VISION_IMAGE_SUBSCRIBER_MANAGER_H
#define MUJIN_VISION_IMAGE_SUBSCRIBER_MANAGER_H

#include <mujincontrollerclient/mujinzmq.hpp>
#include "visionparameters.h"

namespace mujinvision {

class MUJINVISION_API ImageSubscriberManager
{
public:
    ImageSubscriberManager() {
    }

    virtual ~ImageSubscriberManager() {
    }

    /** \brief Initializes the image subscriber manager.
        \param mNameCamera map to cameras the subscribers subscribe to
        \params_pt boost property tree defining the image subscriber parameters
     */
    virtual void Initialize(const std::map<std::string, CameraPtr >&mNameCamera, const std::string& streamerIp, const unsigned int streamerPort, const ptree& params_pt, boost::shared_ptr<zmq::context_t> context) = 0;

    virtual void DeInitialize() = 0;

    /** \brief Gets the latest color image from camera and its timestamp.
        \param cameraname name of the camera
        \param timestamp timestamp of the color image
        \param endtimestamp endtimestamp of the color image
        \return pointer to the color image
     */
    virtual ImagePtr GetColorImageFromBuffer(const std::string& cameraname, unsigned long long& timestamp, unsigned long long& endtimestamp) = 0;
    /** \brief Gets the latest color image from camera and its timestamp.
        \param cameraname name of the camera
        \param timestamp timestamp of the color image
        \param endtimestamp endtimestamp of the color image
        \return pointer to the color image
     */
    virtual ImagePtr SnapColorImage(const std::string& cameraname, unsigned long long& timestamp, unsigned long long& endtimestamp, const double& timeout=1.0/*sec*/) = 0;

    /** \brief Gets the depth image from the latest n images with depth data, and the min/max timestamps of the images used.
        \param cameraname name of the camera
        \param starttime timestamp of the earliest image
        \param endtime timestamp of the latest image
        \return pointer to the depth image
     */
    virtual ImagePtr GetDepthImageFromBuffer(const std::string& cameraname, unsigned long long& starttime, unsigned long long& endtime) = 0;

    virtual void GetImagePackFromBuffer(const std::vector<std::string>& colorcameranames, const std::vector<std::string>& depthcameranames, std::vector<ImagePtr>& colorimages, std::vector<ImagePtr>& depthimages, unsigned long long& starttime, unsigned long long& endtime) = 0;

    /** \brief Gets the depth image from the latest n images with depth data, and the min/max timestamps of the images used.
        \param cameraname name of the camera
        \param starttime timestamp of the earliest image
        \param endtime timestamp of the latest image
        \return pointer to the depth image
     */
    virtual ImagePtr SnapDepthImage(const std::string& cameraname, unsigned long long& starttime, unsigned long long& endtime, const double& timeout=1.0/*sec*/, const int numimages=-1) = 0;

    virtual void StartCaptureThread(const double& timeout=5.0, const int numimages=-1) = 0;
    virtual void StopCaptureThread(const double& timeout=5.0) = 0;

    /** \brief Writes color image to disk.
     */
    virtual void WriteColorImage(const std::string& cameraname, const std::string& filename) = 0;

    /** \brief Writes depth image to disk.
     */
    virtual void WriteDepthImage(const std::string& cameraname, const std::string& filename) = 0;

protected:

    std::map<std::string, CameraPtr> _mNameCamera; ///< name -> camera
};

typedef boost::shared_ptr<ImageSubscriberManager> ImageSubscriberManagerPtr;
typedef boost::shared_ptr<ImageSubscriberManager const> ImageSubscriberManagerConstPtr;
typedef boost::weak_ptr<ImageSubscriberManager> ImageSubscriberManagerWeakPtr;

} // namespace mujinvision
#endif
