/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Preetham Chalasani
  Created on: 2016-11-07

  (C) Copyright 2016-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _osaCartesianImpedanceController_h
#define _osaCartesianImpedanceController_h

#include <cisstParameterTypes/prmCartesianImpedance.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>

// Always include last
#include <sawControllers/sawControllersExport.h>

class CISST_EXPORT osaCartesianImpedanceController
{
public:
    osaCartesianImpedanceController(void);
    ~osaCartesianImpedanceController(){}

    void SetGoal(const prmCartesianImpedance & goal);
    void ResetGoal(void);
    void Update(const prmPositionCartesianGet & pose,
                const prmVelocityCartesianGet & twist,
                prmForceCartesianSet & wrenchBody,
                const bool needWrenchInBody = false);

private:
    prmCartesianImpedance m_goal;
};

#endif // _osaCartesianImpedanceController_h
