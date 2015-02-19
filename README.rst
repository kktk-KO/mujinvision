mujinvision
-----------

Library for Coordinating Vision Application and MUJIN Controller

ZeroMQ (REQ) RPC Messages
-------------------------

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
    "error":{
      "type":"InvalidArguments",
      "desc":"more thorough description of what happened"
    }
  }

- type - comes from the MujinVisionErrorCode enum. Use GetErrorCodeString(MVC_X).
- desc - context specific

Details
=======

For command implementation in the vision server:

- whenever any inputs are bad, throw an exception with MVE_InvalidArgument
- if vision server is busy doing something else and cannot process command, throw MVE_Busy

Regular Command List
====================

Initialize
++++++++++

DetectObjects
+++++++++++++

SendPointCloudObstacleToController
++++++++++++++++++++++++++++++++++

VisualizePointCloudOnController
+++++++++++++++++++++++++++++++

ClearVisualizationOnController
++++++++++++++++++++++++++++++

DetectRegionTransform
+++++++++++++++++++++

SaveSnapshot
++++++++++++

UpdateDetectedObjects
+++++++++++++++++++++

SyncRegion
++++++++++

SyncCameras
++++++++++++

StartDetectionLoop
++++++++++++++++++

StopDetectionLoop
+++++++++++++++++

GetCameraId
+++++++++++

Configuration Command List
==========================

Cancel
++++++

Quit
++++


ZeroMQ Status Messages
----------------------

::

  {
    "timestamp":1000,
    "status":"Active",
    "message":"detail status message"
  }

- status - the string part of MujinVisionManager::ManagerStatus
- timestamp - linux time in ms
