// -*- coding: utf-8 -*-
// Copyright (C) 2014-2016 MUJIN Inc.
#ifndef __MUJIN_IMAGE_H__
#define __MUJIN_IMAGE_H__

#include <stdint.h>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include "opencv_types_c.h"

namespace mujinvision {

/// \brief the semantic image type specifying what the image is used for.
enum MujinImageType
{
    MIT_None=0,
    MIT_Color=1,
    MIT_Depth=2,
    MIT_IR=3,
    MIT_DepthXYZ=4,
    MIT_DepthNormal=5,
    MIT_Disparity=6,
    MIT_DetectionResult=7,
    MIT_ColorRaw=8,
    MIT_IRRaw=9,
    MIT_Any=100,
    MIT_Force = 0xffffffff, // force to 4bytes
};

inline const char* GetImageTypeString(MujinImageType type)
{
    switch (type) {
    case MIT_None: return "None";
    case MIT_Color: return "Color";
    case MIT_Depth: return "Depth";
    case MIT_IR: return "IR";
    case MIT_DepthXYZ: return "DepthXYZ";
    case MIT_DepthNormal: return "DepthNormal";
    case MIT_Disparity: return "Disparity";
    case MIT_DetectionResult: return "DetectionResult";
    case MIT_ColorRaw: return "ColorRaw";
    case MIT_IRRaw: return "IRRaw";
    case MIT_Any: return "Any";
    case MIT_Force: return "Force";
    }
    // throw an exception?
    return "";
};

inline const char* GetDataTypeString(int type)
{
    switch (type) {
    case CV_8UC1: return "CV_8UC1";
    case CV_32FC3: return "CV_32FC3";
    case CV_16SC1: return "CV_16SC1";
    case CV_64FC(8): return "CV_64FC8";
    }
    // throw an exception?
    return "";
};

/// \brief base class defining common attributes of an image
class Image
{
public:
    Image();
    Image(const Image& r);
    Image(int width, int height, uint32_t datatype, MujinImageType imagetype, const std::string& cameraid, const uint64_t& timestamp, const uint64_t& endtimestamp, const std::string& metadata);

    virtual ~Image();

    /// \brief returns a string describing the image
    std::string GetBasicInfoString() const;

    /// \brief return the size in bytes of one pixel
    static int GetPixelSize(uint32_t datatype);
    int GetPixelSize() const;

    /// \brief return the number of channels
    int GetNumberOfChannels() const;

    /// \brief serialize the image and save all the data to dst
    size_t GetSerializationSize() const;

    /// \brief serialize the image data to dst. resize dst to fit the data. returns the size of data written
    size_t Serialize(std::vector<uint8_t>& dst) const;

    /// \brief assuming dst has enough data, save all image data to it. returns the size of data written
    size_t Serialize(uint8_t* pdst, size_t length) const;

    /// \brief initialize the image from serialized data. returns number of bytes consumed
    size_t Deserialize(const uint8_t *src, size_t length);

    uint32_t version;
    int width;
    int height;
    uint32_t datatype; // opencv type like CV_8UC1 ,.....
    MujinImageType imagetype; ///< semantic image type
    std::string cameraid; ///< unique camera identifier
    uint64_t timestamp; ///< timestamp of the beginning of the image aquisition process in milliseconds
    uint64_t endtimestamp; ///< timestamp of the end of the image aquisition process in milliseconds
    std::string metadata; ///< json string of image metadata
    std::vector<uint8_t> data; ///< the image data
};

typedef boost::shared_ptr<Image> ImagePtr;
typedef boost::shared_ptr<Image const> ImageConstPtr;
typedef boost::weak_ptr<Image> ImageWeakPtr;

} // namespace mujinvision

#endif
