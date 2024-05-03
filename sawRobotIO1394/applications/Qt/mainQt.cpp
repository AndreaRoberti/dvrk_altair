/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*

  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-07

  (C) Copyright 2013-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

// system
#include <iostream>
// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstMultiTask/mtsQtWidgetComponent.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>

// Qt includes
#include <QApplication>

int main(int argc, char ** argv)
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create a Qt application
    QApplication application(argc, argv);

    // parse options
    cmnCommandLineOptions options;
    std::string port = mtsRobotIO1394::DefaultPort();
    std::string protocol;
    std::list<std::string> configFiles;
    std::string robotName = "Robot";
    double periodInSeconds = 1.0 * cmn_ms;
    options.AddOptionMultipleValues("c", "config",
                                    "configuration file",
                                    cmnCommandLineOptions::REQUIRED_OPTION, &configFiles);
    options.AddOptionOneValue("p", "port",
                              "port used to communicate with the dVRK controllers (fw, udp)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &port);
    options.AddOptionOneValue("f", "firewire-protocol",
                              "FireWire protocol",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &protocol);
    options.AddOptionOneValue("n", "name",
                              "robot name",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &robotName);
    options.AddOptionOneValue("i", "io-period",
                              "IO read/write period interval in seconds (default is 1 ms, 0.001)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &periodInSeconds);
    options.AddOptionNoValue("C", "calibration-mode",
                             "run in calibration mode, doesn't require lookup table for pots/encoder on Si arms",
                             cmnCommandLineOptions::OPTIONAL_OPTION);

    if (!options.Parse(argc, argv, std::cerr)) {
        return -1;
    }

    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments;

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // RobotIO
    mtsRobotIO1394 * robotIO = new mtsRobotIO1394("robotIO", periodInSeconds, port);
    if (options.IsSet("firewire-protocol")) {
        robotIO->SetProtocol(protocol);
    }
    robotIO->SetCalibrationMode(options.IsSet("calibration-mode"));

    mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");

    componentManager->AddComponent(robotIO);
    componentManager->AddComponent(robotWidgetFactory);

    for (const auto & configFile : configFiles) {
        if (!cmnPath::Exists(configFile)) {
            std::cerr << "Can't find file \"" << configFile << "\"" << std::endl;
            return -1;
        }
        robotIO->Configure(configFile);
    }

    componentManager->Connect("robotWidgetFactory", "RobotConfiguration", "robotIO", "Configuration");
    robotWidgetFactory->Configure();

    // create the components
    componentManager->CreateAllAndWait(2.0 * cmn_s);

    // start the periodic Run
    componentManager->StartAllAndWait(2.0 * cmn_s);

    // run Qt app
    application.exec();

    // cleanup
    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    // delete dvgc robot
    delete robotWidgetFactory;
    delete robotIO;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
