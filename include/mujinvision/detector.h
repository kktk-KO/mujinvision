// -*- coding: utf-8 -*-
// Copyright (C) 2012-2014 MUJIN Inc. <rosen.diankov@mujin.co.jp>
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

#include <mujinvision/visionparameters.h>

namespace mujinvision {

class MUJINVISION_API ObjectDetector : public MujinInterruptable
{
public:
    ObjectDetector() {
    }

    virtual ~ObjectDetector() {
    }

    /** \brief sets up object detector
        \param oparams_pt boost property tree describing object parameters
        \param dparams_pt boost property tree describing detection parameters
        \param region where objects are to be detected
        \param mColorCamera map to color cameras from names
        \param mDepthCamera map to depth cameras from names
     */
    virtual void Initialize(const ptree& oparams_pt,  const ptree& dparams_pt, RegionConstPtr region, std::map<std::string, CameraPtr > mColorCamera, std::map<std::string, CameraPtr > mDepthCamera ) = 0;

    virtual void DeInitialize() = 0;

    /** Detects objects from color and depth images. Assuming SetColorImage and mMergedDepthImage were called.
        \param colorcameraname
        \param depthcameraname
        \param detectedobjects in world frame
     */
    virtual void DetectObjects(const std::string& colorcameraname, const std::string& depthcameraname, std::vector<DetectedObjectPtr>& detectedobjects) = 0;

    /** \brief Gets point cloud obstacle from depth data and detection result.
        \param depthcameraname name of the depth camera
        \param resultsworld detection result in world frame
        \param points result points representing the point cloud obstacle in world frame
        \param voxelsize size of the voxel grid in meters used for simplifying the cloud
     */
    virtual void GetPointCloudObstacle(const std::string& depthcameraname, const std::vector<DetectedObjectPtr>& resultsworld, std::vector<double>& points, const double voxelsize=0.01) = 0;

    /** \brief Gets point cloud in world frame from depth image.
        \param depthcameraname name of the depth camera
        \param depthimage depth image
        \param points result points representing the point cloud in world frame
     */
    virtual void GetCameraPointCloud(const std::string& depthcameraname, DepthImageConstPtr depthimage, std::vector<double>& points) = 0;

    /** \brief Detects the transform of the region.
        \param colorcameraname
        \param depthcameraname
        \param regiontransform transform of the region
     */
    virtual void DetectRegionTransform(const std::string& colorcameraname, const std::string& depthcameraname, mujinvision::Transform& regiontransform) {
    }

    /** \brief Sets the color image for detector to use.
        \param colorcameraname name of the color camera
        \param colorimage color image
        \param minu min vertical pixel defining the region of interest of the image
        \param maxu max vertical pixel defining the region of interest of the image
        \param minv min horizontal pixel defining the region of interest of the image
        \param maxv max horizontal pixel defining the region of interest of the image
     */
    virtual void SetColorImage(const std::string& colorcameraname, ColorImageConstPtr colorimage, const unsigned int minu, const unsigned int maxu, const unsigned int minv, const unsigned int maxv) = 0;

    virtual void SetDepthImage(const std::string& depthcameraname, DepthImagePtr depthimage) {
        mMergedDepthImage[depthcameraname] = depthimage;
    }

    virtual DepthImagePtr GetDepthImage(const std::string& depthcameraname) {
        return mMergedDepthImage[depthcameraname];
    }

    virtual bool DepthImageIsSet(const std::string& depthcameraname) {
        return !!mMergedDepthImage[depthcameraname];
    }

    std::map<std::string, ColorImagePtr> mColorImage; ///< cameraname -> image
    std::map<std::string, DepthImagePtr> mMergedDepthImage; ///< cameraname -> image
    std::map<std::string, std::vector<DepthImagePtr > > mDepthImages; ///< cameraname -> images

};

typedef boost::shared_ptr<ObjectDetector> ObjectDetectorPtr;
typedef boost::shared_ptr<ObjectDetector const> ObjectDetectorConstPtr;
typedef boost::weak_ptr<ObjectDetector> ObjectDetectorWeakPtr;

} // namespace mujinvision
#endif
