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

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>

#include "geometry.h"
#include "visionexceptions.h"
#include "mujinimage.h"

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

    static void ValidateJsonString(const std::string& str)
    {
        try {
            std::stringstream ss;
            ss << str;
            ptree tmppt;
            read_json(ss, tmppt);
        } catch (...) {
            throw MujinVisionException("json string " + str + " is invalid.", MVE_InvalidArgument);
        }
    }

    static std::string GetJsonString(const std::string& str)
    {
        std::string newstr = str;
        if (newstr.find("\\\"") == std::string::npos) {
            boost::replace_all(newstr, "\"", "\\\"");
        }
        if (newstr.find("\\n") == std::string::npos) {
            boost::replace_all(newstr, "\n", "\\n");
        }
        return "\""+newstr+"\"";
    }

    static std::string GetJsonString(const std::string& key, const std::string& value)
    {
        return GetJsonString(key) + ": " + GetJsonString(value);
    }

    static std::string GetJsonString(const std::string& key, const unsigned int& value)
    {
        std::stringstream ss;
        ss << ParametersBase::GetJsonString(key) << ": " << value;
        return ss.str();
    }

    static std::string GetJsonString(const std::string& key, const bool value)
    {
        std::stringstream ss;
        ss << ParametersBase::GetJsonString(key) << ": " << value;
        return ss.str();
    }

    static std::string GetJsonString(const std::string& key, const double& value) {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<double>::digits10+1);
        ss << ParametersBase::GetJsonString(key) << ": " << value;
        return ss.str();
    }

    static std::string GetJsonString(const std::string& key, int value)
    {
        std::stringstream ss;
        ss << ParametersBase::GetJsonString(key) << ": " << value;
        return ss.str();
    }

    static std::string GetJsonString(const std::string& key, const unsigned long long& value)
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
            ss << GetJsonString(vec[i]);
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

    static std::string GetJsonString(const double(&array)[6]) {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<double>::digits10+1);
        ss << "[";
        for (unsigned int i=0; i<6; ++i) {
            ss << array[i];
            if (i != 5) {
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

/// \brief information about camera
struct MUJINVISION_API CameraParameters : public ParametersBase
{
    CameraParameters() {
    }

    CameraParameters(const std::string cameraid)
    {
        id = cameraid;
        isColorCamera = true;
        isDepthCamera = true;
    }

    CameraParameters(const ptree& pt)
    {
        _pt = pt;
        id = pt.get<std::string>("id", "");
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
        } else {
            ss << ", \"is_color_camera\": true";
        }
        if (!isDepthCamera) {
            ss << ", \"is_depth_camera\": false";
        } else {
            ss << ", \"is_depth_camera\": true";
        }
        ss << "}";
        return ss.str();
    }

    ptree GetPropertyTree()
    {
        if (_pt.empty()) {
            _pt.put<std::string>("id", id);
            _pt.put<bool>("isColorCamera", isColorCamera);
            _pt.put<bool>("isDepthCamera", isDepthCamera);
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

/// \brief converts ptree to Transform in meters
inline Transform GetTransform(const ptree& pt) {
    Transform transform;
    std::string unit = pt.get<std::string>("unit", "m");
    double scale = 1.0;
    if (unit == "mm") {
        scale = 0.001;
    } else if (unit == "m") {
        scale = 1.0;
    } else {
        throw MujinVisionException("got unsupported unit " + unit, MVE_Failed);
    }
    unsigned int i=0;
    FOREACH(v, pt.get_child("translation_")) {
        transform.trans[i] = boost::lexical_cast<double>(v->second.data()) * scale;
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
    return transform;
}

/// \brief information about the detected object
struct MUJINVISION_API DetectedObject : public ParametersBase
{
    DetectedObject() {
        //type = "part";
        confidence = "0";
        extra = "null";
    }

    /// assume input is in milimeter
    DetectedObject(const ptree& pt) {
        _pt = pt;
        name = pt.get<std::string>("name");
        //type = pt.get<std::string>("type", "part");
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
        //type = ty;
    }

    virtual ~DetectedObject() {
    }

    std::string name;
    //std::string type; ///< default is "part", could be "container"
    std::string objecturi;  ///< could be empty for "container" type
    Transform transform; ///< in meter
    std::string confidence; ///< detection confidence
    uint64_t timestamp; ///< timestamp of the detection
    std::string extra; ///< could be used to define random box dimensions

    std::string GetJsonString()
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<double>::digits10+1);
        //"{\"name\": \"obj\",\"translation_\":[100,200,300],\"quat_\":[1,0,0,0],\"confidence\":{}}"
        ss << "{";
        ss << ParametersBase::GetJsonString("name") << ": " << ParametersBase::GetJsonString(name) << ", ";
        //ss << ParametersBase::GetJsonString("type") << ": " << ParametersBase::GetJsonString(type) << ", ";
        ss << ParametersBase::GetJsonString("object_uri") << ": " << ParametersBase::GetJsonString(objecturi) << ", ";
        ss << ParametersBase::GetJsonString("translation_") << ": [";
        for (unsigned int i=0; i<3; i++) {
            ss << transform.trans[i];
            if (i!=3-1) {
                ss << ", ";
            }
        }
        ss << "], ";
        ss << ParametersBase::GetJsonString("quat_") << ": [";
        for (unsigned int i=0; i<4; i++) {
            ss << transform.rot[i];
            if (i!=4-1) {
                ss << ", ";
            }
        }
        ss << "], ";
        ss << ParametersBase::GetJsonString("confidence") << ": " << confidence << ", ";
        ss << ParametersBase::GetJsonString("timestamp") << ": " << timestamp << ", ";
        ss << ParametersBase::GetJsonString("extra") << ": " << ParametersBase::GetJsonString(extra);
        ss << "}";
        return ss.str();
    }

    ptree GetPropertyTree()
    {
        if (_pt.empty()) {
            _pt.put<std::string>("name", name);
            //_pt.put<std::string>("type", type);
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
        memset(cropContainerMarginsXYZXYZ, 0, sizeof(cropContainerMarginsXYZXYZ));
        memset(cropContainerEmptyMarginsXYZXYZ, 0, sizeof(cropContainerEmptyMarginsXYZXYZ));
        memset(containerRoiMarginsXYZXYZ, 0, sizeof(containerRoiMarginsXYZXYZ));
        containerEmptyDivisor = 150;
    }

    RegionParameters(const ptree& pt)
    {
        _pt = pt;
        instobjectname = pt.get<std::string>("instobjectname");
        FOREACH(cv, pt.get_child("cameranames")) {
            cameranames.push_back(cv->second.data());
        }
        type = pt.get<std::string>("type", "boxAxisAligned");
        unsigned int i=0;
        FOREACH(v, pt.get_child("cropContainerMarginsXYZXYZ")) {
            cropContainerMarginsXYZXYZ[i] = boost::lexical_cast<double>(v->second.data());
            i++;
        }
        BOOST_ASSERT(i == 6);
        i=0;
        FOREACH(v, pt.get_child("containerRoiMarginsXYZXYZ")) {
            containerRoiMarginsXYZXYZ[i] = boost::lexical_cast<double>(v->second.data());
            i++;
        }
        BOOST_ASSERT(i == 6);
        i=0;
        FOREACH(v, pt.get_child("cropContainerEmptyMarginsXYZXYZ")) {
            cropContainerEmptyMarginsXYZXYZ[i] = boost::lexical_cast<double>(v->second.data());
            i++;
        }
        BOOST_ASSERT(i == 6);
        containerEmptyDivisor = pt.get<double>("containerEmptyDivisor", 150);
        visualizationuri = pt.get<std::string>("visualizationuri", "");
    }

    virtual ~RegionParameters() {
    }

    // the following params are initialized in constructor
    std::string instobjectname; // instobject name in mujin controller that defines the container of the objects to be detected
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

    Transform tBaseLinkInInnerRegionTopCenter; ///< transform of the container link's coordinate system with respect to the inner region's center top face (firstlinkcenter_T_region)
    std::string GetJsonString()
    {
        std::stringstream ss;
        ss << "{";
        ss << "\"instobjectname\": \"" <<  instobjectname << "\", ";
        ss << "\"cameranames\": " << ParametersBase::GetJsonString(cameranames);
        ss << ", " << ParametersBase::GetJsonString("type") << ": " << ParametersBase::GetJsonString(type);
        ss << ", " << ParametersBase::GetJsonString("cropContainerMarginsXYZXYZ") << ": " << ParametersBase::GetJsonString(cropContainerMarginsXYZXYZ);
        ss << ", " << ParametersBase::GetJsonString("cropContainerEmptyMarginsXYZXYZ") << ": " << ParametersBase::GetJsonString(cropContainerEmptyMarginsXYZXYZ);
        ss << ", " << ParametersBase::GetJsonString("containerRoiMarginsXYZXYZ") << ": " << ParametersBase::GetJsonString(containerRoiMarginsXYZXYZ);
        ss << ", " << ParametersBase::GetJsonString("containerEmptyDivisor") << ": " << containerEmptyDivisor;
        ss << ", " << ParametersBase::GetJsonString("visualizationuri") << ": " << ParametersBase::GetJsonString(visualizationuri);
        if (innerTranslation.size() > 0) {
            ss << ", " << ParametersBase::GetJsonString("innerTranslation") << ": " << ParametersBase::GetJsonString(innerTranslation);
        }
        if (innerExtents.size() > 0) {
            ss << ", " << ParametersBase::GetJsonString("innerExtents") << ": " << ParametersBase::GetJsonString(innerExtents);
        }
        if (innerRotationmat.size() > 0) {
            ss << ", " << ParametersBase::GetJsonString("innerRotationmat") << ": " << ParametersBase::GetJsonString(innerRotationmat);
        }
        if (outerTranslation.size() > 0) {
            ss << ", " << ParametersBase::GetJsonString("outerTranslation") << ": " << ParametersBase::GetJsonString(outerTranslation);
        }
        if (outerExtents.size() > 0) {
            ss << ", " << ParametersBase::GetJsonString("outerExtents") << ": " << ParametersBase::GetJsonString(outerExtents);
        }
        if (outerRotationmat.size() > 0) {
            ss << ", " << ParametersBase::GetJsonString("outerRotationmat") << ": " << ParametersBase::GetJsonString(outerRotationmat);
        }
        ss << "}";
        return ss.str();
    }

    ptree GetPropertyTree()
    {
        if (_pt.empty()) {
            _pt.put<std::string>("instobjectname", instobjectname);
            ptree cameranames_pt;
            for (size_t i=0; i<cameranames.size(); i++) {
                cameranames_pt.put<std::string>("", cameranames[i]);
            }
            _pt.put_child("cameranames", cameranames_pt);
            _pt.put<std::string>("type", type);
            ptree crop_pt;
            for (size_t i=0; i<6; i++) {
                crop_pt.put<double>("", cropContainerMarginsXYZXYZ[i]);
            }
            _pt.put_child("cropContainerMarginsXYZXYZ", crop_pt);
            ptree crop_empty_pt;
            for (size_t i=0; i<6; i++) {
                crop_empty_pt.put<double>("", cropContainerEmptyMarginsXYZXYZ[i]);
            }
            _pt.put_child("cropContainerEmptyMarginsXYZXYZ", crop_empty_pt);
            ptree container_pt;
            for (size_t i=0; i<6; i++) {
                container_pt.put<double>("", containerRoiMarginsXYZXYZ[i]);
            }
            _pt.put_child("containerRoiMarginsXYZXYZ", container_pt);
            _pt.put<std::string>("visualizationuri", visualizationuri);
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

} // namespace mujinvision
#endif
