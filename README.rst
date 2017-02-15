.. image:: https://travis-ci.org/mujin/mujinvision.svg?branch=master
    :target: https://travis-ci.org/mujin/mujinvision

mujinvision
-----------

Library for Coordinating Vision Application and MUJIN Controller

ZeroMQ (REQ) RPC Messages
~~~~~~~~~~~~~~~~~~~~~~~~~

Everything is in JSON.

Input
=====

::

  {
    "command":"RunMyCommand",
    "param1":1
  }

param1 - command specific parameters

Output
======

When everything is ok, return parameters depending on command::

  {
    "retvalue":"something",
    "retvalue2":[],
  }

When error happens, response will be filled with an error field::

  {
    "error":
    {
      "type":"InvalidArguments",
      "desc":"more thorough description of what happened"
    }
  }

- type - comes from the MujinVisionErrorCode enum. Use GetErrorCodeString(MVC_X).
- desc - context specific

Handling Errors
===============

For command implementation in the vision server:

- whenever any inputs is bad, throw an exception with MVE_InvalidArgument
- if vision server is busy doing something else and cannot process command, throw MVE_Busy

Regular Command List
====================

Regular commands controls the detection process:

- Initialize
- StartDetectionLoop
- StopDetectionLoop
- DetectObjects
- SendPointCloudObstacleToController
- VisualizePointCloudOnController
- ClearVisualizationOnController
- UpdateDetectedObjects
- SyncRegion
- SyncCameras
- GetCameraId

Configuration Command List
==========================

Configuration commands controls the vision manager itself:

- Cancel
- Quit


ZeroMQ Status Messages
~~~~~~~~~~~~~~~~~~~~~~

::

  {
    "timestamp":1000,
    "status":"Active",
    "message":"detail status message"
  }

- status - the string part of MujinVisionManager::ManagerStatus


- timestamp - linux time in ms
