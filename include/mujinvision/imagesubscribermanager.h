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
/** \file imagesubscribermanager.h
    \brief Public headers of ImageSubscriberManager.
 */
#ifndef MUJIN_VISION_IMAGE_SUBSCRIBER_MANAGER_H
#define MUJIN_VISION_IMAGE_SUBSCRIBER_MANAGER_H

#include <mujincontrollerclient/mujinzmq.h>
#include "visionparameters.h"

namespace mujinvision {

class MUJINVISION_API ImageSubscriberManager
{
public:
    ImageSubscriberManager() {
    }

    virtual ~ImageSubscriberManager() {
    }

    virtual void SetPreemptFn(const boost::function<bool()>& preemptfn) {
        _preemptfn = preemptfn;
    }

    /** \brief Initializes the image subscriber manager.
        \param mNameCamera map to cameras the subscribers subscribe to
        \params_pt boost property tree defining the image subscriber parameters
     */
    virtual void Initialize(const std::map<std::string, CameraPtr >&mNameCamera, const std::string& streamerIp, const unsigned int streamerPort, const ptree& params_pt, boost::shared_ptr<zmq::context_t> context) = 0;

    virtual void DeInitialize() = 0;

    /** \brief Resets imagesubscriber connections
     */
    virtual void Reset() = 0;

    /** \brief Updates cameras
     */
    virtual void UpdateCameras(const std::map<std::string, CameraPtr >&mNameCamera) = 0;

    /** \brief Gets the latest color image from camera and its timestamp.
        \param cameraname name of the camera
        \param timestamp timestamp of the color image
        \param endtimestamp endtimestamp of the color image
        \return pointer to the color image
     */
    virtual ImagePtr GetColorImageFromBuffer(const std::string& cameraname, unsigned long long& timestamp, unsigned long long& endtimestamp, const double timeout=10.0) = 0;
    /** \brief Gets the latest color image from camera and its timestamp.
        \param cameraname name of the camera
        \param timestamp timestamp of the color image
        \param endtimestamp endtimestamp of the color image
        \return pointer to the color image
     */
    virtual ImagePtr SnapColorImage(const std::string& cameraname, unsigned long long& timestamp, unsigned long long& endtimestamp, const double timeout=1.0 /*sec*/) = 0;

    /** \brief Gets the depth image from the latest n images with depth data, and the min/max timestamps of the images used.
        \param cameraname name of the camera
        \param starttime timestamp of the earliest image
        \param endtime timestamp of the latest image
        \return pointer to the depth image
     */
    virtual ImagePtr GetDepthImageFromBuffer(const std::string& cameraname, unsigned long long& starttime, unsigned long long& endtime, const double timeout=10.0) = 0;

    /** \brief Gets image pack from buffer from specified cameras
        \param starttime min capture starting timestamp of all images in the pack
        \param endtime max capture ending timestamp of all images in the pack
        \param imagepacktimestamp timestamp of the image pack
        \param newerthan imagepack must be taken later than this timestamp
        \param mCameraIdRegionName camera id regionname mapping
     */
    virtual void GetImagePackFromBuffer(const std::vector<std::string>& colorcameranames, const std::vector<std::string>& depthcameranames, std::vector<ImagePtr>& colorimages, std::vector<ImagePtr>& depthimages, std::vector<ImagePtr>& resultimages, unsigned long long& starttime, unsigned long long& endtime, unsigned long long& imagepacktimestamp, const double timeout=10.0, const unsigned long long newerthan=0, const std::map<std::string, std::string>& mCameraIdRegionName=std::map<std::string, std::string>()) = 0;

    /** \brief Gets a point cloud from the camera name and prunes and subsamples it.
        \param regionnameforocclusionchecking regionname for occlusion checking, if not specified, do not check for occlusion
        \param timeout in seconds
        \param newerthan imagepack must be taken later than this timestamp
        \param subsample how much to subsample, default is 1 meaning no subsampling
        \return occlusion status of robot with the container: -2 if did not get any depth image, -1 if unknown, 0 if not occluding, 1 if robot is occluding container in camera
     */
    virtual int GetCollisionPointCloud(const std::string& cameraname, std::vector<double>& points, unsigned long long& starttime, unsigned long long& endtime, const double voxelsize=0.01, const double stddev=0.01, const size_t numnn=80, const std::string& regionnameforocclusionchecking="", const double timeout=20.0, unsigned long long newerthan=0, const size_t subsample=1) = 0;

    /** \brief Gets a point cloud from the camera name and prunes and subsamples it.
        \param regionnameforocclusionchecking regionname for occlusion checking, if not specified, do not check for occlusion
        \param timeout in seconds
        \param newerthan imagepack must be taken later than this timestamp
        \param subsample how much to subsample, default is 1 meaning no subsampling
        \return occlusion status of robot with the container: -2 if did not get any depth image, -1 if unknown, 0 if not occluding, 1 if robot is occluding container in camera
     */
    virtual int GetCollisionPointCloud(const std::string& cameraname, ImagePtr depthimage, std::vector<double>& points, const double voxelsize=0.01, const double stddev=0.01, const size_t numnn=80, const std::string& regionnameforocclusionchecking="", const double timeout=20.0, unsigned long long newerthan=0, const size_t subsample=1) = 0;

    /** \brief Gets the depth image from the latest n images with depth data, and the min/max timestamps of the images used.
        \param cameraname name of the camera
        \param starttime timestamp of the earliest image
        \param endtime timestamp of the latest image
        \return pointer to the depth image
     */
    virtual ImagePtr SnapDepthImage(const std::string& cameraname, unsigned long long& starttime, unsigned long long& endtime, const double timeout=1.0 /*sec*/, const int numimages=-1) = 0;

    virtual void SnapColorAndDepthImages(const std::string& cameraname, unsigned long long& starttime, unsigned long long& endtime, std::vector<ImagePtr>& colorimages, std::vector<ImagePtr>& depthimages, const double timeout=1.0 /*sec*/, const int numimages=-1, const std::string& extraoptions="") = 0;

    /** \brief gets detection result
     */
    virtual ImagePtr SnapDetectionResult(const std::string& cameraname, const double timeout=20.0 /*sec*/, const std::string& extraoptions="") = 0;

    virtual void StartCaptureThread(const std::vector<std::string>& cameranames, const double timeout=5.0, const int numimages=-1, const std::string& extraoptions="") = 0;
    virtual void StopCaptureThread(const std::vector<std::string>& cameranames, const double timeout=5.0) = 0;

    /// \brief ms timestamp of when the subscrbe thread was started. Used for throttling how fast subscriber resets itself. If 0, then subscriber thread is not running
    virtual uint64_t GetSubscribeStartedTimeStamp() const = 0;
        
protected:

    std::map<std::string, CameraPtr> _mNameCamera; ///< name -> camera
    boost::function<bool()> _preemptfn;
};

typedef boost::shared_ptr<ImageSubscriberManager> ImageSubscriberManagerPtr;
typedef boost::shared_ptr<ImageSubscriberManager const> ImageSubscriberManagerConstPtr;
typedef boost::weak_ptr<ImageSubscriberManager> ImageSubscriberManagerWeakPtr;

} // namespace mujinvision
#endif
