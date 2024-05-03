/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-03-24

  (C) Copyright 2020-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#ifndef _mtsROSToCISST_CRTK_h
#define _mtsROSToCISST_CRTK_h

// cisst include
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmCartesianImpedance.h>
#include <cisstParameterTypes/prmForwardKinematicsRequest.h>
#include <cisstParameterTypes/prmInverseKinematicsRequest.h>

// crtk
#include <crtk_msgs/OperatingState.h>
#include <crtk_msgs/StringStamped.h>
#include <crtk_msgs/CartesianImpedance.h>
#include <crtk_msgs/QueryForwardKinematics.h>
#include <crtk_msgs/QueryInverseKinematics.h>

// crtk_msgs
void mtsROSToCISST(const crtk_msgs::OperatingState & rosData, prmOperatingState & cisstData);
void mtsROSToCISST(const crtk_msgs::StringStamped & rosData, std::string & cisstData);
void mtsROSToCISST(const crtk_msgs::CartesianImpedance & rosData, prmCartesianImpedance & cisstData);
void mtsROSToCISST(const crtk_msgs::QueryForwardKinematics::Request & rosData,
                   prmForwardKinematicsRequest & cisstData);
void mtsROSToCISST(const crtk_msgs::QueryInverseKinematics::Request & rosData,
                   prmInverseKinematicsRequest & cisstData);

#endif // _mtsROSToCISST_CRTK_h
