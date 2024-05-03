/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-08-26

  (C) Copyright 2016-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _sawRobotIO1394ForwardDeclarations_h
#define _sawRobotIO1394ForwardDeclarations_h

#include <cisstCommon/cmnUnits.h>

class AmpIO;
class BasePort;

namespace sawRobotIO1394 {

    class mtsRobot1394;
    class mtsDigitalInput1394;
    class mtsDigitalOutput1394;
    class mtsDallasChip1394;

    const double WatchdogTimeout = 30.0 * cmn_ms;

    const double MaximumTimeForMVGood = 0.2 * cmn_s;
    const double MaximumTimeToPower = 3.0 * cmn_s;

    const double TimeBetweenPotentiometerMissingErrors = 5.0 * cmn_s;

    const double TimingMaxRatio = 2.0;
    const double TimeBetweenTimingWarnings = 120.0 * cmn_s;

    //! Temperature thresholds
    const double TemperatureWarningThreshold = 60.0;
    const double TemperatureErrorThreshold = 65.0;
    const double TimeBetweenTemperatureWarnings = 120.0 * cmn_s;

} // namespace sawRobotIO1394

#endif // _sawRobotIO1394ForwardDeclarations_h
