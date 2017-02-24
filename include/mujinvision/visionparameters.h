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
/** \file visionparameters.h
    \brief Common parameters definitions.
 */
#ifndef MUJIN_VISION_PARAMETERS_H
#define MUJIN_VISION_PARAMETERS_H

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>
#include <stdint.h>
#include <stdexcept>
#include <rapidjson/pointer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "geometry.h"
#include "visionexceptions.h"

#include <mujincontrollerclient/mujinjson.h>

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); )

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

namespace mujinvision {

#include <mujinvision/config.h>

using geometry::MathTransform;
typedef MathTransform<double> Transform;
using geometry::MathTransformMatrix;
typedef MathTransformMatrix<double> TransformMatrix;
using geometry::MathVector;
typedef MathVector<double> Vector;

using namespace mujinjson;

/// \brief ip and port of a connection
struct MUJINVISION_API ConnectionParameters : public JsonSerializable
{
    ConnectionParameters() {
    }

    ConnectionParameters(const rapidjson::Value& value) {
        LoadFromJson(value);
    }

    void LoadFromJson(const rapidjson::Value& value) {
        LoadJsonValueByKey(value, "ip", ip);
        LoadJsonValueByKey(value, "port", port);
    }

    void SaveToJson(rapidjson::Value& d, rapidjson::Document::AllocatorType& alloc) const {
        d.SetObject();
        SetJsonValueByKey(d, "ip", ip, alloc);
        SetJsonValueByKey(d, "port", port, alloc);
    }

    virtual ~ConnectionParameters() {
    }

    std::string ip;
    int port;
};

typedef boost::shared_ptr<ConnectionParameters> ConnectionParametersPtr;
typedef boost::shared_ptr<ConnectionParameters const> ConnectionParametersConstPtr;
typedef boost::weak_ptr<ConnectionParameters> ConnectionParametersWeakPtr;

/// \brief information about camera
struct MUJINVISION_API CameraParameters : public JsonSerializable
{
    CameraParameters(): isColorCamera(true), isDepthCamera(true){
    }

    CameraParameters(const std::string cameraid): id(cameraid), isColorCamera(true), isDepthCamera(true) {

    }

    CameraParameters(const rapidjson::Value& value): id(""), isColorCamera(true), isDepthCamera(true) {
        LoadFromJson(value);
    }

    void LoadFromJson(const rapidjson::Value& value) {
        LoadJsonValueByKey(value, "id", id);
        LoadJsonValueByKey(value, "is_color_camera", isColorCamera);
        LoadJsonValueByKey(value, "is_depth_camera", isDepthCamera);
    }

    void SaveToJson(rapidjson::Value& d, rapidjson::Document::AllocatorType& alloc) const {
        d.SetObject();
        SetJsonValueByKey(d, "id", id, alloc);
        SetJsonValueByKey(d, "is_color_camera", isColorCamera, alloc);
        SetJsonValueByKey(d, "is_depth_camera", isDepthCamera, alloc);
    }

    virtual ~CameraParameters() {
    }

    std::string id;
    bool isColorCamera;
    bool isDepthCamera;
};
typedef boost::shared_ptr<CameraParameters> CameraParametersPtr;
typedef boost::shared_ptr<CameraParameters const> CameraParametersConstPtr;
typedef boost::weak_ptr<CameraParameters> CameraParametersWeakPtr;

/// \brief sensor calibration data
struct MUJINVISION_API CalibrationData : public JsonSerializable
{

    CalibrationData() : distortion_coeffs(5) {
    }

    CalibrationData(const rapidjson::Value& value) : distortion_coeffs(5) {
        LoadFromJson(value);
    }

    void LoadFromJson(const rapidjson::Value& value) {
        LoadJsonValueByKey(value, "fx", fx);
        LoadJsonValueByKey(value, "fy", fx);
        LoadJsonValueByKey(value, "pu", pv);
        LoadJsonValueByKey(value, "pv", pv);
        LoadJsonValueByKey(value, "s", s);
        LoadJsonValueByKey(value, "focal_length", focal_length);
        LoadJsonValueByKey(value, "kappa", kappa);
        LoadJsonValueByKey(value, "image_width", image_width);
        LoadJsonValueByKey(value, "image_height", image_height);
        LoadJsonValueByKey(value, "pixel_width", pixel_width);
        LoadJsonValueByKey(value, "pixel_height", pixel_height);
        LoadJsonValueByKey(value, "distortion_model", distortion_model);
        LoadJsonValueByKey(value, "distortion_coeffs", distortion_coeffs);
    }

    void SaveToJson(rapidjson::Value& d, rapidjson::Document::AllocatorType& alloc) const {
        d.SetObject();
        SetJsonValueByKey(d, "fx", fx, alloc);
        SetJsonValueByKey(d, "fy", fx, alloc);
        SetJsonValueByKey(d, "pu", pv, alloc);
        SetJsonValueByKey(d, "pv", pv, alloc);
        SetJsonValueByKey(d, "s", s, alloc);
        SetJsonValueByKey(d, "focal_length", focal_length, alloc);
        SetJsonValueByKey(d, "kappa", kappa, alloc);
        SetJsonValueByKey(d, "image_width", image_width, alloc);
        SetJsonValueByKey(d, "image_height", image_height, alloc);
        SetJsonValueByKey(d, "pixel_width", pixel_width, alloc);
        SetJsonValueByKey(d, "pixel_height", pixel_height, alloc);
        SetJsonValueByKey(d, "distortion_model", distortion_model, alloc);
        SetJsonValueByKey(d, "distortion_coeffs", distortion_coeffs, alloc);
    }

    virtual ~CalibrationData() {
    }

    double fx;
    double fy;
    double pu;
    double pv;
    double s;
    double focal_length;
    double kappa;
    double image_width;
    double image_height;
    double pixel_width;
    double pixel_height;
    std::string distortion_model;
    std::vector<double> distortion_coeffs;
    std::vector<double> extra_parameters;
};
typedef boost::shared_ptr<CalibrationData> CalibrationDataPtr;
typedef boost::shared_ptr<CalibrationData const> CalibrationDataConstPtr;
typedef boost::weak_ptr<CalibrationData> CalibrationDataWeakPtr;


/// \brief converts ptree to Transform in meters
inline Transform GetTransform(const rapidjson::Value& config) {
    Transform transform;
    std::string unit =  "m";
    LoadJsonValueByKey(config, "unit", unit);
    double scale = 1.0;
    if (unit == "mm") {
        scale = 0.001;
    } else if (unit == "m") {
        scale = 1.0;
    } else {
        throw MujinVisionException("got unsupported unit " + unit, MVE_Failed);
    }

    if (config.HasMember("translation_")) {
        const rapidjson::Value &translation = config["translation_"];
        if (translation.IsArray() && translation.Size() == 3) {
            int i = 0;
            for (rapidjson::Value::ConstValueIterator it = translation.Begin(); it != translation.End(); ++it) {
                transform.trans[i] = it->GetDouble() * scale;
                i++;
            }
        }
    }
    if (config.HasMember("quat_")) {
        const rapidjson::Value &quat = config["quat_"];
        if (quat.IsArray() && quat.Size() == 4) {
            int i = 0;
            for (rapidjson::Value::ConstValueIterator it = quat.Begin(); it != quat.End(); ++it) {
                transform.rot[i] = it->GetDouble();
                i++;
            }
        }
    }
    return transform;
}

/// \brief information about the detected object
struct MUJINVISION_API DetectedObject : public JsonSerializable
{
    DetectedObject(): confidence("0"), extra("null"){
    }

    /// assume input is in milimeter
//    DetecetedObject(const rapidjson::Value& value): extra("null") {
//        LoadJsonValueByKey(value, "name", name);
//        LoadJsonValueByKey(value, "object_uri", objecturi);
//        vector<double> trans, rot;
//        LoadJsonValueByKey(value, "translation_", trans);
//        if (trans.size() >= 3) {
//            for (size_t i = 0; i < 3; i++) {
//                transform.trans[i] = trans[i] * 0.001;
//            }
//        }
//        LoadJsonValueByKey(value, "quat_", rot);
//        LoadJsonValueByKey(value, "dir_", rot);
//        if (rot.size() >= 4) {
//            for (size_t i = 0; i < 4; i ++){
//                transform.rot[i] = rot[i];
//            }
//        }
//        LoadJsonValueByKey(value, "confidence", confidence);
//        LoadJsonValueByKey(vlaue, "timestamp", timestamp);
//    }

    /// assume input is in meter
    DetectedObject(const std::string& n, const std::string& u, const Transform& t, const std::string& c, const uint64_t ts, const std::string& e)
    {
        name = n;
        objecturi = u;
        transform = t;
        confidence = c;
        timestamp = ts;
        extra = e;
        //type = ty;
    }

    void LoadFromJson(const rapidjson::Value& d) {} // dummpy

    void SaveToJson(rapidjson::Value& d, rapidjson::Document::AllocatorType& alloc) const {
        d.SetObject();
        SetJsonValueByKey(d, "name", name, alloc);
        //SetJsonValueByKey(d, "type", type);
        SetJsonValueByKey(d, "object_uri", objecturi, alloc);
        std::vector<double> trans, quat;
        for (size_t i = 0; i < 3; ++i) {
            trans.push_back(transform.trans[i]);
        }
        for (size_t i = 0; i < 4; ++i) {
            quat.push_back(transform.rot[i]);
        }
        SetJsonValueByKey(d, "translation_", trans, alloc);
        SetJsonValueByKey(d, "quat_", quat, alloc);
        rapidjson::Document confidencejson;
        confidencejson.Parse(confidence.c_str());
        SetJsonValueByKey(d, "confidence", confidencejson, alloc);
        SetJsonValueByKey(d, "timestamp", timestamp, alloc);
        SetJsonValueByKey(d, "extra", extra, alloc);
    }

    virtual ~DetectedObject() {
    }

    std::string name;
    //std::string type; ///< default is "part", could be "container"
    std::string objecturi;  ///< could be empty for "container" type
    Transform transform; ///< in meter
    std::string confidence; ///< detection confidence, dumped string of a json object
    unsigned long long timestamp; ///< timestamp of the detection
    std::string extra; ///< could be used to define random box dimensions, dumped string of a json object

};
typedef boost::shared_ptr<DetectedObject> DetectedObjectPtr;
typedef boost::shared_ptr<DetectedObject const> DetectedObjectConstPtr;
typedef boost::weak_ptr<DetectedObject> DetectedObjectWeakPtr;

/// \brief Specifies region where vision system performs detection with a set of cameras
struct MUJINVISION_API RegionParameters : public JsonSerializable
{
    RegionParameters(): containerEmptyDivisor(150), pointsize(3), filteringstddev(0), filteringnumnn(0), filteringsubsample(0) {
        memset(cropContainerMarginsXYZXYZ, 0, sizeof(cropContainerMarginsXYZXYZ));
        memset(cropContainerEmptyMarginsXYZXYZ, 0, sizeof(cropContainerEmptyMarginsXYZXYZ));
        memset(containerRoiMarginsXYZXYZ, 0, sizeof(containerRoiMarginsXYZXYZ));
    }

    RegionParameters(const rapidjson::Value& value): containerEmptyDivisor(150), pointsize(3), filteringstddev(0), filteringnumnn(0), filteringsubsample(0) {
        memset(cropContainerMarginsXYZXYZ, 0, sizeof(cropContainerMarginsXYZXYZ));
        memset(cropContainerEmptyMarginsXYZXYZ, 0, sizeof(cropContainerEmptyMarginsXYZXYZ));
        memset(containerRoiMarginsXYZXYZ, 0, sizeof(containerRoiMarginsXYZXYZ));
        LoadFromJson(value);
    }

    void LoadFromJson(const rapidjson::Value& value) {
        LoadJsonValueByKey(value, "instobjectname", instobjectname);
        LoadJsonValueByKey(value, "locationIOName", locationIOName);
        LoadJsonValueByKey(value, "cameranames", cameranames);
        LoadJsonValueByKey(value, "type", type);
        BOOST_ASSERT(value.HasMember("cropContainerMarginsXYZXYZ") && value["cropContainerMarginsXYZXYZ"].GetArray().Size() == 6);
        LoadJsonValueByKey(value, "cropContainerMarginsXYZXYZ", cropContainerMarginsXYZXYZ);
        BOOST_ASSERT(value.HasMember("containerRoiMarginsXYZXYZ") && value["containerRoiMarginsXYZXYZ"].GetArray().Size() == 6);
        LoadJsonValueByKey(value, "containerRoiMarginsXYZXYZ", containerRoiMarginsXYZXYZ);
        BOOST_ASSERT(value.HasMember("cropContainerEmptyMarginsXYZXYZ") && value["cropContainerEmptyMarginsXYZXYZ"].GetArray().Size() == 6);
        LoadJsonValueByKey(value, "cropContainerEmptyMarginsXYZXYZ", cropContainerEmptyMarginsXYZXYZ);
        LoadJsonValueByKey(value, "containerEmptyDivisor", containerEmptyDivisor, 150);
        LoadJsonValueByKey(value, "pointsize", pointsize);
        LoadJsonValueByKey(value, "filteringsubsample", filteringsubsample);
        LoadJsonValueByKey(value, "filteringstddev", filteringstddev);
        LoadJsonValueByKey(value, "filteringnumnn", filteringnumnn);
    }

    virtual ~RegionParameters() {
    }

    // the following params are initialized in constructor
    std::string instobjectname; ///< instobject name in mujin controller that defines the container of the objects to be detected
    std::string locationIOName; ///< the location IO name to send to the controller to notify IO variables
    std::vector<std::string> cameranames;
    std::string type; ///< the type of the container, by default it is boxAxisAligned
    double cropContainerMarginsXYZXYZ[6]; ///< Margins of the container to be cropped (or enlarged if negative), in order to define 3D container region under (calibration & shape) uncertainty - for pointcloud processing.
    double cropContainerEmptyMarginsXYZXYZ[6]; ///< Margins of the container to be cropped (or enlarged if negative), in order to define 3D container region under (calibration & shape) uncertainty - to compute container empty variable
    double containerRoiMarginsXYZXYZ[6]; ///< Margins of the container to be cropped (or enlarged if negative), in order to define a 2D container region under (calibration & shape) uncertainty - for 2D processing.
    double containerEmptyDivisor; ///< Paramater that controls the maximum number of points allowed for the container to be empty after cropping the internal walls.
    std::string visualizationuri; ///< visualiation URI for the container for debugging purposes.

    // the following params are initialized later during visionmanager initialization
    std::vector<double> innerTranslation; ///< the center of the box defining the inner region in the world frame
    std::vector<double> innerExtents; ///< the half extents inner region where parts can be found
    std::vector<double> innerRotationmat; ///< defining rotation of inner box in the world frame, 3x3 row major
    std::vector<double> outerTranslation; ///< the center of the box defining the outer extents of the base link (physical) in the world frame
    std::vector<double> outerExtents; ///< the outer extents of the region defining the outer walls of container (physical).
    std::vector<double> outerRotationmat; ///< defining rotation of outer box (physical) in the world frame, 3x3 row major
    double pointsize; ///< pointcloud pointsize in millimeter
    double filteringstddev;  ///< if not 0, the point cloud filting param for exec verification. invalid if 0.
    int filteringnumnn;  ///< if not 0, point cloud filting param for exec verification. invalid if 0.
    int filteringsubsample;  ///< if not 0, point cloud filting param for exec verification. invalid if 0.

    Transform baselinkcenter_T_region; ///< transform of the container link's coordinate system with respect to the inner region's center top face (baselinkcenter_T_region)

    void SaveToJson(rapidjson::Value& d, rapidjson::Document::AllocatorType& alloc) const {
        d.SetObject();
        SetJsonValueByKey(d, "instobjectname", instobjectname, alloc);
        SetJsonValueByKey(d, "cameranames", cameranames, alloc);
        SetJsonValueByKey(d, "type", type, alloc);
        SetJsonValueByKey(d, "cropContainerMarginsXYZXYZ", std::vector<double>(cropContainerMarginsXYZXYZ, cropContainerMarginsXYZXYZ + 6), alloc);
        SetJsonValueByKey(d, "cropContainerEmptyMarginsXYZXYZ", std::vector<double>(cropContainerEmptyMarginsXYZXYZ, cropContainerEmptyMarginsXYZXYZ + 6), alloc);
        SetJsonValueByKey(d, "containerRoiMarginsXYZXYZ", std::vector<double>(containerRoiMarginsXYZXYZ, containerRoiMarginsXYZXYZ + 6), alloc);
        SetJsonValueByKey(d, "containerEmptyDivisor", containerEmptyDivisor, alloc);
        SetJsonValueByKey(d, "visualizationuri", visualizationuri, alloc);
        SetJsonValueByKey(d, "pointsize", pointsize, alloc);
        SetJsonValueByKey(d, "filteringstddev", filteringstddev, alloc);
        SetJsonValueByKey(d, "filteringnumnn", filteringnumnn, alloc);
        SetJsonValueByKey(d, "filteringsubsample", filteringsubsample, alloc);
        if (!innerTranslation.empty()) {
            SetJsonValueByKey(d, "innerTranslation", innerTranslation, alloc);
        }
        if (!innerExtents.empty()) {
            SetJsonValueByKey(d, "innerExtents", innerExtents, alloc);
        }
        if (!innerRotationmat.empty()) {
            SetJsonValueByKey(d, "innerRotationmat", innerRotationmat, alloc);
        }
        if (!outerTranslation.empty()) {
            SetJsonValueByKey(d, "outerTranslation", outerTranslation, alloc);
        }
        if (!outerExtents.empty()) {
            SetJsonValueByKey(d, "outerExtents", outerExtents, alloc);
        }
        if (!outerRotationmat.empty()) {
            SetJsonValueByKey(d, "outerRotationmat", outerRotationmat, alloc);
        }
    }
};
typedef boost::shared_ptr<RegionParameters> RegionParametersPtr;
typedef boost::shared_ptr<RegionParameters const> RegionParametersConstPtr;
typedef boost::weak_ptr<RegionParameters> RegionParametersWeakPtr;

/// \brief camera class
class MUJINVISION_API Camera
{
public:
    Camera(const std::string& n, CameraParametersPtr params, CalibrationDataPtr calibdata)
    {
        name = n;
        pCameraParameters = params;
        pCalibrationData = calibdata;
    }

    virtual ~Camera() {
    }

    void SetWorldTransform(const Transform& worldtransform)
    {
        worldTransform = worldtransform;
    }

    const Transform& GetWorldTransform() const
    {
        return worldTransform;
    }

    std::string name;
    CameraParametersPtr pCameraParameters;

    /// \brief calibration data
    CalibrationDataPtr pCalibrationData;

    CameraParametersPtr GetCameraParameters() const
    {
        return pCameraParameters;
    }

    std::vector<double> GetKK() const
    {
        std::vector<double> KK;
        KK.resize(9);
        KK[0] = pCalibrationData->fx; KK[1] = pCalibrationData->s; KK[2] = pCalibrationData->pu;
        KK[3] = 0; KK[4] = pCalibrationData->fy; KK[5] = pCalibrationData->pv;
        KK[6] = 0; KK[7] = 0; KK[8] = 1;
        return KK;
    }

protected:

    /// \brief camera transform in world frame
    Transform worldTransform;

};
typedef boost::shared_ptr<Camera> CameraPtr;
typedef boost::shared_ptr<Camera const> CameraConstPtr;
typedef boost::weak_ptr<Camera> CameraWeakPtr;

/// \brief information about the region
class MUJINVISION_API Region
{
public:
    Region() {
    }

    Region(RegionParametersPtr params)
    {
        pRegionParameters = params;
    }

    virtual ~Region() {
    }

    void SetWorldTransform(const Transform& t)
    {
        //worldTransform = t;
        //toRegionTransform = worldTransform.inverse();
    }

//    const Transform& GetWorldTransform() const
//    {
//        return worldTransform;
//    }

    RegionParametersPtr pRegionParameters;

private:

    /// \brief used to transform a point in the world frame to the box frame
    //Transform toRegionTransform;
    //Transform worldTransform;

};
typedef boost::shared_ptr<Region> RegionPtr;
typedef boost::shared_ptr<Region const> RegionConstPtr;
typedef boost::weak_ptr<Region> RegionWeakPtr;

class MUJINVISION_API MujinInterruptable
{
public:
    MujinInterruptable() {
    }

    virtual ~MujinInterruptable() {
    }

    virtual void SetSetStatusFn(const boost::function<void(const std::string& msg, const std::string& err)>& setstatusfn) {
        _setstatusfn = setstatusfn;
    }

    virtual void SetStatus(const std::string& msg, const std::string& err="")
    {
        if( !!_setstatusfn ) {
            _setstatusfn(msg, err);
        }
    }

protected:
    boost::function<void(const std::string& msg, const std::string& err)> _setstatusfn;
};

class MUJINVISION_API Utils
{
public:
    /** \brief Transforms detected objects.
     */
    static void TransformDetectedObjects(const std::vector<DetectedObjectPtr>& detectedobjectsfrom, std::vector<DetectedObjectPtr>& detectedobjectsto, const Transform& worldtransformfrom, const Transform& worldtransformto);
};

class Image
{
public:
    Image() {
    }

    virtual ~Image() {
    }

    virtual std::string GetCameraId() = 0;
    virtual uint64_t GetStartTimestamp() = 0;
    virtual uint64_t GetEndTimestamp() = 0;
    virtual std::string GetMetadata() = 0;
};

typedef boost::shared_ptr<Image> ImagePtr;
typedef boost::shared_ptr<Image const> ImageConstPtr;
typedef boost::weak_ptr<Image> ImageWeakPtr;

enum ImageUserType {
    IUT_Detecion = 0,
    IUT_ExecutionVerification = 1,
};

} // namespace mujinvision
#endif
