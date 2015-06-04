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
/** \file visionparameters.h
    \brief Common parameters definitions.
 */
#ifndef MUJIN_VISION_PARAMETERS_H
#define MUJIN_VISION_PARAMETERS_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/function.hpp>

#include "geometry.h"
#include "visionexceptions.h"

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
using boost::property_tree::ptree;

/// \brief base class of parameters
struct MUJINVISION_API ParametersBase
{
    ParametersBase() {
    }
    virtual ~ParametersBase() {
    }

    virtual std::string GetJsonString() = 0;
    virtual ptree GetPropertyTree() = 0;

    void Print();

    static std::string GetJsonString(const std::string& str)
    {
        return "\""+str+"\"";
    }

    static std::string GetJsonString(const std::string& key, const std::string& value)
    {
        return GetJsonString(key) + ": " + GetJsonString(value);
    }

    static std::string GetJsonString(const std::string& key, const unsigned int value)
    {
        std::stringstream ss;
        ss << ParametersBase::GetJsonString(key) << ": " << value;
        return ss.str();
    }

    static std::string GetJsonString(const std::vector<std::string>& vec)
    {
        std::stringstream ss;
        ss << "[";
        for (unsigned int i = 0; i < vec.size(); i++) {
            ss << vec[i];
            if( i != vec.size()-1) {
                ss << ", ";
            }
        }
        ss << "]";
        return ss.str();
    }

    static std::string GetJsonString(const std::vector<double>& vec)
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<double>::digits10+1);
        ss << "[";
        for (unsigned int i = 0; i < vec.size(); i++) {
            ss << vec[i];
            if( i != vec.size()-1) {
                ss << ", ";
            }
        }
        ss << "]";
        return ss.str();
    }

    static std::string GetJsonString (const std::vector<int>& vec)
    {
        std::stringstream ss;
        ss << "[";
        for (unsigned int i = 0; i < vec.size(); i++) {
            ss << vec[i];
            if( i != vec.size()-1) {
                ss << ", ";
            }
        }
        ss << "]";
        return ss.str();
    }

    static std::string GetJsonString(const Transform& transform)
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<double>::digits10+1);
        // \"translation\":[%.15f, %.15f, %.15f], \"quaternion\":[%.15f, %.15f, %.15f, %.15f]
        ss << "\"translation\": [";
        for (unsigned int i=0; i<3; i++) {
            ss << transform.trans[i];
            if (i!=3-1) {
                ss << ", ";
            }
        }
        ss << "], ";
        ss << "\"quaternion\": [";
        for (unsigned int i=0; i<4; i++) {
            ss << transform.rot[i];
            if (i!=4-1) {
                ss << ", ";
            }
        }
        ss << "]";
        return ss.str();
    }

    static std::string GetJsonString(const MujinVisionException& exception)
    {
        std::stringstream ss;
        ss << GetJsonString("error") << ": {";
        ss << GetJsonString("type") << ": " << GetJsonString(exception.GetCodeString()) << ",";
        ss << GetJsonString("desc") << ": " << GetJsonString(exception.message());
        ss << "}";
        return ss.str();
    }

    static std::string GetExceptionJsonString(const std::string& type, const std::string& desc)
    {
        std::stringstream ss;
        ss << GetJsonString("error") << ": {";
        ss << GetJsonString("type") << ": " << GetJsonString(type) << ",";
        ss << GetJsonString("desc") << ": " << GetJsonString(desc);
        ss << "}";
        return ss.str();
    }

protected:
    ptree _pt;

};

/// \brief ip and port of a connection
struct MUJINVISION_API ConnectionParameters : public ParametersBase
{
    ConnectionParameters() {
    }

    ConnectionParameters(const ptree& pt)
    {
        _pt = pt;
        ip = pt.get<std::string>("ip");
        port = pt.get<int>("port");
    }

    virtual ~ConnectionParameters() {
    }

    std::string ip;
    int port;

    std::string GetJsonString()
    {
        std::stringstream ss;
        ss << "{";
        ss << "\"ip\": \"" << ip << "\", \"port\": " << port;
        ss << "}";
        return ss.str();
    }

    ptree GetPropertyTree()
    {
        if (_pt.empty()) {
            _pt.put<std::string>("ip", ip);
            _pt.put<int>("port", port);
        }
        return _pt;
    }
};
typedef boost::shared_ptr<ConnectionParameters> ConnectionParametersPtr;
typedef boost::shared_ptr<ConnectionParameters const> ConnectionParametersConstPtr;
typedef boost::weak_ptr<ConnectionParameters> ConnectionParametersWeakPtr;

/// \brief image data structure
class Image;
typedef boost::shared_ptr<Image> ImagePtr;
typedef boost::shared_ptr<Image const> ImageConstPtr;
typedef boost::weak_ptr<Image> ImageWeakPtr;

/// \brief information about camera
struct MUJINVISION_API CameraParameters : public ParametersBase
{
    CameraParameters() {
    }

    CameraParameters(const ptree& pt)
    {
        _pt = pt;
        id = pt.get<std::string>("id");
        isColorCamera = pt.get<bool>("is_color_camera", true);
        isDepthCamera = pt.get<bool>("is_depth_camera", true);
    }

    virtual ~CameraParameters() {
    }

    std::string id;
    bool isColorCamera;
    bool isDepthCamera;

    std::string GetJsonString()
    {
        std::stringstream ss;
        ss << "{";
        ss << "\"id\": \"" << id << "\"";
        if (!isColorCamera) {
            ss << ", \"is_color_camera\": false";
        }
        if (!isDepthCamera) {
            ss << ", \"is_depth_camera\": false";
        }
        ss << "}";
        return ss.str();
    }

    ptree GetPropertyTree()
    {
        if (_pt.empty()) {
            _pt.put<std::string>("id", id);
            if (!isColorCamera) {
                _pt.put<bool>("isColorCamera", isColorCamera);
            }
            if (!isDepthCamera) {
                _pt.put<bool>("isDepthCamera", isDepthCamera);
            }
        }
        return _pt;
    }
};
typedef boost::shared_ptr<CameraParameters> CameraParametersPtr;
typedef boost::shared_ptr<CameraParameters const> CameraParametersConstPtr;
typedef boost::weak_ptr<CameraParameters> CameraParametersWeakPtr;

/// \brief sensor calibration data
struct MUJINVISION_API CalibrationData : public ParametersBase
{

    CalibrationData() {
        distortion_coeffs.resize(5);
    }

    CalibrationData(const ptree& pt)
    {
        _pt = pt;
        fx = pt.get<double>("fx");
        fy = pt.get<double>("fy");
        pu = pt.get<double>("pu");
        pv = pt.get<double>("pv");
        s = pt.get<double>("s");
        focal_length = pt.get<double>("focal_length");
        kappa = pt.get<double>("kappa");
        image_width = pt.get<double>("image_width");
        image_height = pt.get<double>("image_height");
        pixel_width = pt.get<double>("pixel_width");
        pixel_height = pt.get<double>("pixel_height");
        distortion_model = pt.get<std::string>("distortion_model");
        unsigned int i=0;
        FOREACH(v, pt.get_child("distortion_coeffs")) {
            distortion_coeffs[i] = boost::lexical_cast<double>(v->second.data());
            i++;
        }
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

    std::string GetJsonString()
    {
        std::stringstream ss;
        ss << "{";
        ss << "fx: " << fx << ", ";
        ss << "fy: " << fy << ", ";
        ss << "pu: " << pu << ", ";
        ss << "pv: " << pv << ", ";
        ss << "s: " << s << ", ";
        ss << "focal_length: " << focal_length << ", ";
        ss << "kappa: " << kappa << ", ";
        ss << "image_width: " << image_width << ", ";
        ss << "image_height: " << image_height << ", ";
        ss << "pixel_width: " << pixel_width << ", ";
        ss << "pixel_height: " << pixel_height << ", ";
        ss << "distortion_model: " << distortion_model << ", ";
        ss << "distortion_coeffs: [";
        for (size_t i = 0; i < distortion_coeffs.size(); i++) {
            ss << distortion_coeffs[i];
            if (i != distortion_coeffs.size()) {
                ss << ", ";
            }
        }
        ss << "]";        
        ss << "extra_parameters: [";
        for (size_t iparam = 0; iparam < extra_parameters.size(); iparam++) {
            ss << extra_parameters[iparam];
            if (iparam != extra_parameters.size()) {
                ss << ", ";
            }
        }
        ss << "]";
        ss << "}";
        return ss.str();
    }

    ptree GetPropertyTree()
    {
        if (_pt.empty()) {
            _pt.put<double>("fx", fx);
            _pt.put<double>("fy", fy);
            _pt.put<double>("pu", pu);
            _pt.put<double>("pv", pv);
            _pt.put<double>("s", s);
            _pt.put<double>("focal_length", focal_length);
            _pt.put<double>("kappa", kappa);
            _pt.put<double>("image_width", image_width);
            _pt.put<double>("image_height", image_height);
            _pt.put<double>("pixel_width", pixel_width);
            _pt.put<double>("pixel_height", pixel_height);
            ptree extrapt;
            for (unsigned int i=0; i<extra_parameters.size(); i++) {
                ptree p;
                p.put<double>("",extra_parameters[i]);
                extrapt.push_back(std::make_pair("",p));
            }
            _pt.put_child("extra_parameters", extrapt);
        }
        return _pt;
    }
};
typedef boost::shared_ptr<CalibrationData> CalibrationDataPtr;
typedef boost::shared_ptr<CalibrationData const> CalibrationDataConstPtr;
typedef boost::weak_ptr<CalibrationData> CalibrationDataWeakPtr;

/// \brief information about the detected object
struct MUJINVISION_API DetectedObject : public ParametersBase
{
    DetectedObject() {
        confidence = "0";
        extra = "null";
    }

    /// assume input is in milimeter
    DetectedObject(const ptree& pt)
    {
        _pt = pt;
        name = pt.get<std::string>("name");
        objecturi = pt.get<std::string>("object_uri");
        unsigned int i=0;
        FOREACH(v, pt.get_child("translation_")) {
            transform.trans[i] = boost::lexical_cast<double>(v->second.data()); // assuming in milimeter
            i++;
        }
        i=0;
        boost::optional< const boost::property_tree::ptree& > optchild;
        optchild = pt.get_child_optional( "quat_" );
        if (!!optchild) {
            FOREACH(v, pt.get_child("quat_")) {
                transform.rot[i] = boost::lexical_cast<double>(v->second.data());
                i++;
            }
        }
        i=0;
        optchild = pt.get_child_optional( "dir_" );
        if (!!optchild) {
            FOREACH(v, pt.get_child("dir_")) {
                transform.rot[i] = boost::lexical_cast<double>(v->second.data());
                i++;
            }
        }
        confidence = pt.get<std::string>("confidence");
        timestamp = pt.get<uint64_t>("timestamp");
        extra = "null";
    }

    /// assume input is in meter
    DetectedObject(const std::string& n, const std::string& u, const Transform& t, const std::string& c, const uint64_t ts, const std::string& e)
    {
        name = n;
        objecturi = u;
        transform = t;
        confidence = c;
        timestamp = ts;
        extra = e;
    }

    virtual ~DetectedObject() {
    }

    std::string name;
    std::string objecturi;
    Transform transform; ///< in meter
    std::string confidence; ///< detection confidence
    uint64_t timestamp; ///< timestamp of the detection
    std::string extra;

    std::string GetJsonString()
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<double>::digits10+1);
        //"{\"name\": \"obj\",\"translation_\":[100,200,300],\"quat_\":[1,0,0,0],\"confidence\":{}}"
        ss << "{";
        ss << "\"name\": \"" << name << "\", ";
        ss << "\"object_uri\": \"" << objecturi << "\", ";
        ss << "\"translation_\": [";
        for (unsigned int i=0; i<3; i++) {
            ss << transform.trans[i];
            if (i!=3-1) {
                ss << ", ";
            }
        }
        ss << "], ";
        ss << "\"quat_\": [";
        for (unsigned int i=0; i<4; i++) {
            ss << transform.rot[i];
            if (i!=4-1) {
                ss << ", ";
            }
        }
        ss << "], ";
        ss << "\"confidence\": " << confidence;
        ss << ",\"timestamp\": " << timestamp;
        ss << "}";
        return ss.str();
    }

    ptree GetPropertyTree()
    {
        if (_pt.empty()) {
            _pt.put<std::string>("name", name);
            _pt.put<std::string>("object_uri", objecturi);
            ptree translation_pt;
            for (unsigned int i=0; i<3; i++) {
                ptree p;
                p.put<double>("",transform.trans[i]*1000.0f); // convert from meter to milimeter
                translation_pt.push_back(std::make_pair("",p));
            }
            _pt.put_child("translation_", translation_pt);
            ptree quat_pt;
            for (unsigned int i=0; i<4; i++) {
                ptree p;
                p.put<double>("",transform.rot[i]);
                quat_pt.push_back(std::make_pair("",p));
            }
            _pt.put_child("quat_", quat_pt);
            _pt.put<std::string>("confidence", confidence);
            _pt.put<uint64_t>("timestamp", timestamp);
        }
        return _pt;
    }
};
typedef boost::shared_ptr<DetectedObject> DetectedObjectPtr;
typedef boost::shared_ptr<DetectedObject const> DetectedObjectConstPtr;
typedef boost::weak_ptr<DetectedObject> DetectedObjectWeakPtr;

/// \brief Specifies region where vision system performs detection with a set of cameras
struct MUJINVISION_API RegionParameters : public ParametersBase
{
    RegionParameters() {
    }

    RegionParameters(const ptree& pt)
    {
        _pt = pt;
        instobjectname = pt.get<std::string>("instobjectname");
        FOREACH(cv, pt.get_child("cameranames")) {
            cameranames.push_back(cv->second.data());
        }
        boost::optional<const ptree&> globalroi3d_pt(pt.get_child_optional("globalroi3d"));
        if (!!globalroi3d_pt) {
            std::vector<double> roi;
            FOREACH(rv, *globalroi3d_pt) {
                roi.push_back(boost::lexical_cast<double>(rv->second.data()));
            }
            minx = roi[0];
            maxx = roi[1];
            miny = roi[2];
            maxy = roi[3];
            minz = roi[4];
            maxz = roi[5];
            bInitializedRoi = true;
        } else {
            bInitializedRoi = false;
        }
    }

    virtual ~RegionParameters() {
    }

    std::string instobjectname; // instobject name in mujin controller that defines the container of the objects to be detected
    std::vector<std::string> cameranames;

    /// \brief global roi in meter
    double minx,maxx,miny,maxy,minz,maxz;
    bool bInitializedRoi;

    std::string GetJsonString()
    {
        std::stringstream ss;
        ss << "{";
        ss << "\"name\": \"" <<  instobjectname << "\", ";
        ss << "\"cameranames\": " << ParametersBase::GetJsonString(cameranames) << ",";
        ss << "\"globalroi3d\": [" << minx << ", " << maxx << ", " << miny << ", " << maxy << ", " << minz << ", " << maxz << "]";
        ss << "}";
        return ss.str();
    }

    ptree GetPropertyTree()
    {
        if (_pt.empty()) {
            _pt.put<std::string>("name", instobjectname);
            ptree cameranames_pt;
            for (size_t i=0; i<cameranames.size(); i++) {
                cameranames_pt.put<std::string>("", cameranames[i]);
            }
            _pt.put_child("cameranames", cameranames_pt);
            ptree globalroi3d_pt;
            ptree p;
            p.put("",minx);
            globalroi3d_pt.push_back(std::make_pair("", p));
            p.put("",maxx);
            globalroi3d_pt.push_back(std::make_pair("", p));
            p.put("",miny);
            globalroi3d_pt.push_back(std::make_pair("", p));
            p.put("",maxy);
            globalroi3d_pt.push_back(std::make_pair("", p));
            p.put("",minz);
            globalroi3d_pt.push_back(std::make_pair("", p));
            p.put("",maxz);
            globalroi3d_pt.push_back(std::make_pair("", p));
            _pt.put_child("globalroi3d", globalroi3d_pt);
        }
        return _pt;
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
        worldTransform = t;
        toRegionTransform = worldTransform.inverse();
    }

    const Transform& GetWorldTransform() const
    {
        return worldTransform;
    }

    std::vector<double> GetROI() const
    {
        std::vector<double> roi;
        roi.push_back(pRegionParameters->minx);
        roi.push_back(pRegionParameters->maxx);
        roi.push_back(pRegionParameters->miny);
        roi.push_back(pRegionParameters->maxy);
        roi.push_back(pRegionParameters->minz);
        roi.push_back(pRegionParameters->maxz);
        return roi;
    }

    bool IsPointInROI(const double px, const double py, const double pz) const
    {
        if (!pRegionParameters->bInitializedRoi) {
            throw MujinVisionException("globalroi3d is not initialized!", MVE_Failed);
        }
        Vector p(px,py,pz);
        Vector v = toRegionTransform*p;

        if (v.x >= pRegionParameters->minx && v.x <= pRegionParameters->maxx && v.y >= pRegionParameters->miny && v.y <= pRegionParameters->maxy && v.z >= pRegionParameters->minz && v.z <= pRegionParameters->maxz) {
            return true;
        } else {
            return false;
        }
    }

    RegionParametersPtr pRegionParameters;

private:

    /// \brief used to transform a point in the world frame to the box frame
    Transform toRegionTransform;
    Transform worldTransform;

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

    virtual void SetSetStatusFn(const boost::function<void(const std::string& msg)>& setstatusfn) {
        _setstatusfn = setstatusfn;
    }

    virtual void SetStatus(const std::string& msg)
    {
        if( !!_setstatusfn ) {
            _setstatusfn(msg);
        }
    }

protected:
    boost::function<void(const std::string& msg)> _setstatusfn;
};

class MUJINVISION_API Utils
{
public:
    /** \brief Transforms detected objects.
     */
    static void TransformDetectedObjects(const std::vector<DetectedObjectPtr>& detectedobjectsfrom, std::vector<DetectedObjectPtr>& detectedobjectsto, const Transform& worldtransformfrom, const Transform& worldtransformto);
};

} // namespace mujinvision
#endif
