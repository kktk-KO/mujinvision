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

/// \brief gets a string of the Value type for debugging purposes 
inline std::string GetJsonTypeName(const rapidjson::Value& v) {
    int type = v.GetType();
    switch (type) {
        case 0:
            return "Null";
        case 1:
            return "False";
        case 2:
            return "True";
        case 3:
            return "Object";
        case 4:
            return "Array";
        case 5:
            return "String";
        case 6:
            return "Number";
        default:
            return "Unknown";
    }
}

inline std::string DumpJson(const rapidjson::Value& value) {
    rapidjson::StringBuffer stringbuffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(stringbuffer);
    value.Accept(writer);
    return std::string(stringbuffer.GetString(), stringbuffer.GetSize());
}

inline void ParseJson(rapidjson::Document&d, const std::string& str) {
    d.Parse(str.c_str());
    if (d.HasParseError()) {
        std::string substr;
        if (str.length()> 200) {
            substr = str.substr(0, 200);
        } else {
            substr = str;
        }
        throw MujinVisionException("Json string is invalid: " + substr, MVE_InvalidArgument);
    }
}

/// \brief base class of parameters
struct MUJINVISION_API ParametersBase
{
    virtual ~ParametersBase() {
    }
    inline std::string GetJsonString() {
        rapidjson::Document d;
        GetJson(d);
        return DumpJson(d);

    }
    virtual void GetJson(rapidjson::Document& d) const = 0;
    virtual void LoadFromJson(const rapidjson::Value& value) = 0;
};

//store a json value to local data structures
//for compatibility with ptree, type conversion is made. will remove them in the future
inline void LoadJsonValue(const rapidjson::Value& v, std::string& t) {
    if (v.IsString()) {
        t = v.GetString();
    } else if (v.IsInt64()) {
        t = boost::lexical_cast<std::string>(v.GetInt64());
    } else {
        throw MujinVisionException("Cannot convert json type " + GetJsonTypeName(v) + " to String", MVE_InvalidArgument);
    }

}

inline void LoadJsonValue(const rapidjson::Value& v, int& t) {
    if (v.IsInt()) {
        t = v.GetInt();
    } else if (v.IsString()) {
        t = boost::lexical_cast<int>(v.GetString());
    } else {
        throw MujinVisionException("Cannot convert json type " + GetJsonTypeName(v) + " to Int", MVE_InvalidArgument);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, unsigned int& t) {
    if (v.IsUint()) {
        t = v.GetUint();
    } else if (v.IsString()) {
        t = boost::lexical_cast<unsigned int>(v.GetString());
    } else {
        throw MujinVisionException("Cannot convert json type " + GetJsonTypeName(v) + " to Int", MVE_InvalidArgument);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, unsigned long long& t) {
    if (v.IsUint64()) {
        t = v.GetUint64();
    } else if (v.IsString()) {
        t = boost::lexical_cast<unsigned long long>(v.GetString());
    } else {
        throw MujinVisionException("Cannot convert json type " + GetJsonTypeName(v) + " to Int64", MVE_InvalidArgument);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, uint64_t& t) {
    if (v.IsUint64()) {
        t = v.GetUint64();
    } else if (v.IsString()) {
        t = boost::lexical_cast<uint64_t>(v.GetString());
    } else {
        throw MujinVisionException("Cannot convert json type " + GetJsonTypeName(v) + " to Int64", MVE_InvalidArgument);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, double& t) {
    if (v.IsNumber()) {
        t = v.GetDouble();
    } else if (v.IsString()) {
        t = boost::lexical_cast<double>(v.GetString());
    } else {
        throw MujinVisionException("Cannot convert json type " + GetJsonTypeName(v) + " to Double", MVE_InvalidArgument);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, bool& t) {
    if (v.IsInt()) t = v.GetInt();
    else if (v.IsBool()) t = v.GetBool();
    else if (v.IsString())  {
        t = boost::lexical_cast<bool>(v.GetString());
    } else {
        throw MujinVisionException("Cannot convert json type " + GetJsonTypeName(v) + " to Bool", MVE_InvalidArgument);
    }
}

inline void LoadJsonValue(const rapidjson::Value& v, ParametersBase& t) {
    if (v.IsObject()) {
        t.LoadFromJson(v);
    } else {
        throw MujinVisionException("Cannot convert json type " + GetJsonTypeName(v) + " to Object", MVE_InvalidArgument);
    }
}

template<class T> inline void LoadJsonValue(const rapidjson::Value& v, boost::shared_ptr<T>& ptr) {
    if (v.IsObject()) {
        T t;
        LoadJsonValue(v, t);
        ptr = boost::shared_ptr<T>(new T(t));
    } else {
        throw MujinVisionException("Cannot convert json type " + GetJsonTypeName(v) + " to Object", MVE_InvalidArgument);
    }

}
template<class T, size_t N> inline void LoadJsonValue(const rapidjson::Value& v, T (&p)[N]) {
    if (v.IsArray()) {
        if (v.GetArray().Size() != N) {
            throw MujinVisionException("Json array size doesn't match", MVE_InvalidArgument);
        }
        size_t i = 0;
        for (rapidjson::Value::ConstValueIterator it = v.Begin(); it != v.End(); ++it) {
            LoadJsonValue(*it, p[i]);
            i++;
        }
    } else {
        throw MujinVisionException("Cannot convert json type " + GetJsonTypeName(v) + " to Array", MVE_InvalidArgument);
    }
}

template<class T> inline void LoadJsonValue(const rapidjson::Value& v, std::vector<T>& t) {
    if (v.IsArray()) {
        t.clear();
        t.resize(v.GetArray().Size());
        size_t i = 0;
        for (rapidjson::Value::ConstValueIterator it = v.Begin(); it != v.End(); ++it) {
            LoadJsonValue(*it, t[i]);
            i++;
        }
    } else {
        throw MujinVisionException("Cannot convert json type " + GetJsonTypeName(v) + " to Array", MVE_InvalidArgument);
    }
}

//Save a data structure to rapidjson::Value format

inline void SaveJsonValue(rapidjson::Value& v, const std::string& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetString(t.c_str(), alloc);
}

inline void SaveJsonValue(rapidjson::Value& v, const char* t, rapidjson::Document::AllocatorType& alloc) {
    v.SetString(t, alloc);
}

inline void SaveJsonValue(rapidjson::Value& v, int t, rapidjson::Document::AllocatorType& alloc) {
    v.SetInt(t);
}

inline void SaveJsonValue(rapidjson::Value& v, unsigned int t, rapidjson::Document::AllocatorType& alloc) {
    v.SetUint(t);
}

inline void SaveJsonValue(rapidjson::Value& v, long long t, rapidjson::Document::AllocatorType& alloc) {
    v.SetInt64(t);
}

inline void SaveJsonValue(rapidjson::Value& v, unsigned long long t, rapidjson::Document::AllocatorType& alloc) {
    v.SetUint64(t);
}

inline void SaveJsonValue(rapidjson::Value& v, uint64_t t, rapidjson::Document::AllocatorType& alloc) {
    v.SetUint64(t);
}

inline void SaveJsonValue(rapidjson::Value& v, bool t, rapidjson::Document::AllocatorType& alloc) {
    v.SetBool(t);
}

inline void SaveJsonValue(rapidjson::Value& v, double t, rapidjson::Document::AllocatorType& alloc) {
    v.SetDouble(t);
}

inline void SaveJsonValue(rapidjson::Value& v, const rapidjson::Value& t, rapidjson::Document::AllocatorType& alloc) {
    v.CopyFrom(t, alloc);
}

inline void SaveJsonValue(rapidjson::Value& v, const ParametersBase& p, rapidjson::Document::AllocatorType& alloc) {
    rapidjson::Document d;
    p.GetJson(d);
    v.CopyFrom(d, alloc);
}

template<class T> inline void SaveJsonValue(rapidjson::Value& v, const std::vector<T>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(t.size(), alloc);
    for (size_t i = 0; i < t.size(); ++i) {
        rapidjson::Value tmpv;
        SaveJsonValue(tmpv, t[i], alloc);
        v.PushBack(tmpv, alloc);
    }
}

template<> inline void SaveJsonValue(rapidjson::Value& v, const std::vector<double>& t, rapidjson::Document::AllocatorType& alloc) {
    v.SetArray();
    v.Reserve(t.size(), alloc);
    for (size_t i = 0; i < t.size(); ++i) {
        v.PushBack(t[i], alloc);
    }
}

template<class T> inline void SaveJsonValue(rapidjson::Document& v, const T& t) {
    SaveJsonValue(v, t, v.GetAllocator());
}

/** do not remove: otherwise boost::shared_ptr could be treated as bool
 */
template<class T> inline void SaveJsonValue(rapidjson::Value& v, const boost::shared_ptr<T>& ptr, rapidjson::Document::AllocatorType& alloc) {
    SaveJsonValue(v, *ptr, alloc);
}

template<class T, class U> inline void SetJsonValueByKey(rapidjson::Value& v, const U& key, const T& t, rapidjson::Document::AllocatorType& alloc);

inline void SaveJsonValue(rapidjson::Value& v, const MujinVisionException& e, rapidjson::Document::AllocatorType& alloc) {
    v.SetObject();
    SetJsonValueByKey(v, "type", (int)(e.GetCode()), alloc);
    SetJsonValueByKey(v, "desc", e.GetCodeString(), alloc);
}

//get one json value by key, and store it in local data structures
template<class T> void inline LoadJsonValueByKey(const rapidjson::Value& v, const char* key, T& t) {
    if (v.HasMember(key)) {
        LoadJsonValue(v[key], t);
    }
}
template<class T, class U> inline void LoadJsonValueByKey(const rapidjson::Value& v, const char* key, T& t, const U& d) {
    if (v.HasMember(key)) {
        LoadJsonValue(v[key], t);
    }
    else {
        t = d;
    }
}

//work the same as LoadJsonValueByKey, but the value is returned instead of being passed as reference
template<class T, class U> T GetJsonValueByKey(const rapidjson::Value& v, const char* key, const U& t) {
    if (v.HasMember(key)) {
        T r;
        LoadJsonValue(v[key], r);
        return r;
    }
    else {
        return T(t);
    }
}
template<class T> inline T GetJsonValueByKey(const rapidjson::Value& v, const char* key) {
    T r;
    if (v.HasMember(key)) {
        LoadJsonValue(v[key], r);
    }
    return r;
}

template<class T> inline T GetJsonValueByPath(const rapidjson::Value& v, const char* key) {
    T r;
    const rapidjson::Value *child = rapidjson::Pointer(key).Get(v);
    if (child) {
        LoadJsonValue(*child, r);
    }
    return r;
}

template<class T, class U> T GetJsonValueByPath(const rapidjson::Value& v, const char* key, const U& t) {
    const rapidjson::Value *child = rapidjson::Pointer(key).Get(v);
    if (child) {
        T r;
        LoadJsonValue(*child, r);
        return r;
    }
    else {
        return T(t);
    }
}

template<class T, class U> inline void SetJsonValueByKey(rapidjson::Value& v, const U& key, const T& t, rapidjson::Document::AllocatorType& alloc)
{
    if (v.HasMember(key)) {
        SaveJsonValue(v[key], t, alloc);
    }
    else {
        rapidjson::Value value, name;
        SaveJsonValue(name, key, alloc);
        SaveJsonValue(value, t, alloc);
        v.AddMember(name, value, alloc);
    }
}

template<class T>
inline void SetJsonValueByKey(rapidjson::Document& d, const char* key, const T& t)
{
    SetJsonValueByKey(d, key, t, d.GetAllocator());
}

template<class T>
inline void SetJsonValueByKey(rapidjson::Document& d, const std::string& key, const T& t)
{
    SetJsonValueByKey(d, key.c_str(), t, d.GetAllocator());
}

inline void ValidateJsonString(const std::string& str) {
    rapidjson::Document d;
    if (d.Parse(str.c_str()).HasParseError()) {
        throw MujinVisionException("json string " + str + " is invalid.", MVE_InvalidArgument);
    }
}
template<class T> inline std::string GetJsonString(const T& t) {
    rapidjson::Document d;
    SaveJsonValue(d, t);
    return DumpJson(d);
}


/// \brief ip and port of a connection
struct MUJINVISION_API ConnectionParameters : public ParametersBase
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

    void GetJson(rapidjson::Document& d) const {
        d.SetObject();
        SetJsonValueByKey(d, "ip", ip);
        SetJsonValueByKey(d, "port", port);
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
struct MUJINVISION_API CameraParameters : public ParametersBase
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

    void GetJson(rapidjson::Document& d) const {
        d.SetObject();
        SetJsonValueByKey(d, "id", id);
        SetJsonValueByKey(d, "is_color_camera", isColorCamera);
        SetJsonValueByKey(d, "is_depth_camera", isDepthCamera);
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
struct MUJINVISION_API CalibrationData : public ParametersBase
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

    void GetJson(rapidjson::Document& d) const {
        d.SetObject();
        SetJsonValueByKey(d, "fx", fx);
        SetJsonValueByKey(d, "fy", fx);
        SetJsonValueByKey(d, "pu", pv);
        SetJsonValueByKey(d, "pv", pv);
        SetJsonValueByKey(d, "s", s);
        SetJsonValueByKey(d, "focal_length", focal_length);
        SetJsonValueByKey(d, "kappa", kappa);
        SetJsonValueByKey(d, "image_width", image_width);
        SetJsonValueByKey(d, "image_height", image_height);
        SetJsonValueByKey(d, "pixel_width", pixel_width);
        SetJsonValueByKey(d, "pixel_height", pixel_height);
        SetJsonValueByKey(d, "distortion_model", distortion_model);
        SetJsonValueByKey(d, "distortion_coeffs", distortion_coeffs);
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

    unsigned int i=0;
    const rapidjson::Value& translation = config["translation_"];
    for (rapidjson::Value::ConstMemberIterator it = translation.MemberBegin(); it != translation.MemberEnd(); ++it) {
        transform.trans[i] = boost::lexical_cast<double>(it->value.GetString()) * scale;
        i++;
    }
    i=0;
    const rapidjson::Value& quat = config["quat_"];
    for (rapidjson::Value::ConstMemberIterator it = quat.MemberBegin(); it != quat.MemberEnd(); ++it) {
        transform.rot[i] = boost::lexical_cast<double>(it->value.GetString());
    }
    return transform;
}

/// \brief information about the detected object
struct MUJINVISION_API DetectedObject: public ParametersBase
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

    void LoadFromJson(const rapidjson::Value& value) {
        //dummpy function
    }

    void GetJson(rapidjson::Document& d) const {
        d.SetObject();
        SetJsonValueByKey(d, "name", name);
        //SetJsonValueByKey(d, "type", type);
        SetJsonValueByKey(d, "object_uri", objecturi);
        std::vector<double> trans, quat;
        for (size_t i = 0; i < 3; ++i) {
            trans.push_back(transform.trans[i]);
        }
        for (size_t i = 0; i < 4; ++i) {
            quat.push_back(transform.rot[i]);
        }
        SetJsonValueByKey(d, "translation_", trans);
        SetJsonValueByKey(d, "quat_", quat);
        rapidjson::Document confidencejson;
        confidencejson.Parse(confidence.c_str());
        SetJsonValueByKey(d, "confidence", confidencejson);
        SetJsonValueByKey(d, "timestamp", timestamp);
        SetJsonValueByKey(d, "extra", extra);
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
struct MUJINVISION_API RegionParameters : public ParametersBase
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

    void GetJson(rapidjson::Document& d) const {
        d.SetObject();
        SetJsonValueByKey(d, "instobjectname", instobjectname);
        SetJsonValueByKey(d, "cameranames", cameranames);
        SetJsonValueByKey(d, "type", type);
        SetJsonValueByKey(d, "cropContainerMarginsXYZXYZ", std::vector<double>(cropContainerMarginsXYZXYZ, cropContainerMarginsXYZXYZ + 6));
        SetJsonValueByKey(d, "cropContainerEmptyMarginsXYZXYZ", std::vector<double>(cropContainerEmptyMarginsXYZXYZ, cropContainerEmptyMarginsXYZXYZ + 6));
        SetJsonValueByKey(d, "containerRoiMarginsXYZXYZ", std::vector<double>(containerRoiMarginsXYZXYZ, containerRoiMarginsXYZXYZ + 6));
        SetJsonValueByKey(d, "containerEmptyDivisor", containerEmptyDivisor);
        SetJsonValueByKey(d, "visualizationuri", visualizationuri);
        SetJsonValueByKey(d, "pointsize", pointsize);
        SetJsonValueByKey(d, "filteringstddev", filteringstddev);
        SetJsonValueByKey(d, "filteringnumnn", filteringnumnn);
        SetJsonValueByKey(d, "filteringsubsample", filteringsubsample);
        if (!innerTranslation.empty()) {
            SetJsonValueByKey(d, "innerTranslation", innerTranslation);
        }
        if (!innerExtents.empty()) {
            SetJsonValueByKey(d, "innerExtents", innerExtents);
        }
        if (!innerRotationmat.empty()) {
            SetJsonValueByKey(d, "innerRotationmat", innerRotationmat);
        }
        if (!outerTranslation.empty()) {
            SetJsonValueByKey(d, "outerTranslation", outerTranslation);
        }
        if (!outerExtents.empty()) {
            SetJsonValueByKey(d, "outerExtents", outerExtents);
        }
        if (!outerRotationmat.empty()) {
            SetJsonValueByKey(d, "outerRotationmat", outerRotationmat);
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


} // namespace mujinvision
#endif
