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
/** \file detector.h
    \brief Public headers of ObjectDetector.
 */
#ifndef MUJIN_DETECTOR_H
#define MUJIN_DETECTOR_H

#include "mujinvision/visionparameters.h"

namespace mujinvision {

typedef boost::function<bool(const unsigned int)> CheckPreemptFn;

class MUJINVISION_API ObjectDetector : public MujinInterruptable
{
public:
    ObjectDetector() {
    }

    virtual ~ObjectDetector() {
    }

    /** \brief sets up object detector
        \param detectorconf json string describing detector config
        \param mNameRegion map to detection regions from names
        \param mRegionColorCameraMap map to color camera maps from region names
        \param mRegionDepthCameraMap map to depth camera maps from region names
        \param extraInitializationOptions optional extra options
        \param whether to get python gil
     */
    virtual void Initialize(const std::string& detectorconf, const std::string& targetname, const std::map<std::string, RegionPtr >& mNameRegion, const std::map<std::string, std::map<std::string, CameraPtr > >& mRegionColorCameraMap, const std::map<std::string, std::map<std::string, CameraPtr > >& mRegionDepthCameraMap, const std::map< std::string, std::string>& extraInitializationOptions = std::map< std::string, std::string>(), const CheckPreemptFn& preemptfn=CheckPreemptFn(), const bool getgil=true) = 0;

    virtual void DeInitialize() = 0;

    /** Detects objects from color and depth images.
        \param regionname
        \param colorcameranames
        \param depthcameranames
        \param resultimages results could come from the streamer
        \param detectedobjects in world frame
        \param resultstate additional information about the detection result
        \param fastdetection whether to prioritize speed
        \param bindetection whether to detect bin
        \param checkpreemptbits bits to check for preempt
     */
    virtual void DetectObjects(const std::string& regionname, const std::vector<std::string>& colorcameranames, const std::vector<std::string>& depthcameranames, std::vector<DetectedObjectPtr>& detectedobjects, std::string& resultstate, const bool fastdetection=false, const bool bindetection=false, const bool checkcontaineremptyonly=false, const unsigned int checkpreemptbits=0) = 0;

    virtual void DetectObjects(const std::string& regionname, const std::vector<std::string>& colorcameranames, const std::vector<std::string>& depthcameranames, const std::vector<ImagePtr>& resultimages, std::vector<DetectedObjectPtr>& detectedobjects, std::string& resultstate, const bool fastdetection=false, const bool bindetection=false, const bool checkcontaineremptyonly=false, const unsigned int checkpreemptbits=0) = 0;

    /** \brief Gets point cloud obstacle from depth data and detection result.
        \param regionname
        \param depthcameraname name of the depth camera
        \param resultsworld detection result in world frame
        \param points result points representing the point cloud obstacle in world frame
        \param voxelsize size of the voxel grid in meters used for simplifying the cloud
        \param fast whether to prioritize speed
        \param whether to get python gil
        \param checkpreemptbits bits to check for preempt
     */
    virtual void GetPointCloudObstacle(const std::string& regionname, const std::string& depthcameraname, const std::vector<DetectedObjectPtr>& resultsworld, std::vector<double>& points, const double voxelsize=0.01, const bool fast=false, const bool getgil=false, const double stddev=0.01, const size_t numnn=80, const unsigned int checkpreemptbits=1) = 0;

    /** \brief Gets point cloud in world frame from depth image.
        \param regionname
        \param depthcameraname name of the depth camera
        \param depthimage depth image
        \param points result points representing the point cloud in world frame
     */
    virtual void GetCameraPointCloud(const std::string& regionname, const std::string& depthcameraname, ImageConstPtr depthimage, std::vector<double>& points, const double voxelsize=0.01) = 0;

    virtual void SetColorImage(ImagePtr colorimage) = 0;
    virtual void SetDepthImage(ImagePtr depthimage) = 0;

    virtual ImagePtr GetDepthImage(const std::string& depthcameraname) {
        return _mDepthImage[depthcameraname];
    }

    virtual void AddColorImage(const std::string& cameraname, ImagePtr image) {
        if (_mColorImages.find(cameraname) == _mColorImages.end()) {
            std::vector<ImagePtr> images;
            images.push_back(image);
            _mColorImages[cameraname] = images;
        } else {
            _mColorImages[cameraname].push_back(image);
        }
    }

    virtual void AddDepthImage(const std::string& cameraname, ImagePtr image) {
        if (_mDepthImages.find(cameraname) == _mDepthImages.end()) {
            std::vector<ImagePtr> images;
            images.push_back(image);
            _mDepthImages[cameraname] = images;
        } else {
            _mDepthImages[cameraname].push_back(image);
        }
    }

protected:

    std::map<std::string, RegionPtr > _mNameRegion; ///< name->region
    std::map<std::string, ImagePtr> _mColorImage; ///< cameraname -> image
    std::map<std::string, ImagePtr> _mDepthImage; ///< cameraname -> image
    std::map<std::string, std::vector<ImagePtr> > _mColorImages; ///< cameraname -> images
    std::map<std::string, std::vector<ImagePtr> > _mDepthImages; ///< cameraname -> images
    std::map<std::string, std::map<std::string, CameraPtr > > _mRegionColorCameraMap; ///< regionname -> name->camera
    std::map<std::string, std::map<std::string, CameraPtr > > _mRegionDepthCameraMap; ///< regionname -> name->camera

    CheckPreemptFn _preemptfn; ///< function the detector can call to be interrupted by user
};

typedef boost::shared_ptr<ObjectDetector> ObjectDetectorPtr;
typedef boost::shared_ptr<ObjectDetector const> ObjectDetectorConstPtr;
typedef boost::weak_ptr<ObjectDetector> ObjectDetectorWeakPtr;

} // namespace mujinvision
#endif
