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
/** \file imagesubscriber.h
    \brief Public headers of ImageSubscriber.
 */
#ifndef MUJIN_IMAGE_SUBSCRIBER_H
#define MUJIN_IMAGE_SUBSCRIBER_H

#include "mujinvision/visionparameters.h"

namespace mujinvision {

class MUJINVISION_API ImageSubscriber
{
public:
    ImageSubscriber() {
    }
    virtual ~ImageSubscriber() {
    }
};

typedef boost::shared_ptr<ImageSubscriber> ImageSubscriberPtr;
typedef boost::shared_ptr<ImageSubscriber const> ImageSubscriberConstPtr;
typedef boost::weak_ptr<ImageSubscriber> ImageSubscriberWeakPtr;

} // namespace mujinvision
#endif
