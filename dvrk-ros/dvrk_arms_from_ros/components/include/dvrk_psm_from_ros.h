/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-01-13

  (C) Copyright 2020-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _dvrk_psm_from_ros_h
#define _dvrk_psm_from_ros_h

#include <dvrk_arm_from_ros.h>

class dvrk_psm_from_ros: public dvrk_arm_from_ros
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    typedef mtsROSBridge BaseType;

    dvrk_psm_from_ros(const std::string & componentName,
                      ros::NodeHandle * _node_handle,
                      const double periodInSeconds);
    dvrk_psm_from_ros(const mtsTaskPeriodicConstructorArg & arg);
    ~dvrk_psm_from_ros() {}

    void InitPSM(void);

protected:
};

CMN_DECLARE_SERVICES_INSTANTIATION(dvrk_psm_from_ros);

#endif // _dvrk_psm_from_ros_h
