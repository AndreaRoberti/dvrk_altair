/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-04-28

  (C) Copyright 2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTMQtWidget.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitMTMQtWidget, mtsComponent, std::string);

mtsIntuitiveResearchKitMTMQtWidget::mtsIntuitiveResearchKitMTMQtWidget(const std::string & componentName, double periodInSeconds):
    mtsIntuitiveResearchKitArmQtWidget(componentName, periodInSeconds)
{
    CMN_ASSERT(InterfaceRequired);
    InterfaceRequired->AddFunction("gripper/measured_js", gripper_measured_js);
}

void mtsIntuitiveResearchKitMTMQtWidget::setupUiDerived(void)
{
    QFont font;
    font.setBold(true);

    QFrame * gripperFrame = new QFrame();
    gripperFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    MainLayout->addWidget(gripperFrame);

    QHBoxLayout * gripperLayout = new QHBoxLayout;
    gripperLayout->setContentsMargins(1, 1, 1, 1);
    gripperFrame->setLayout(gripperLayout);

    QLabel * titleGripper = new QLabel("Gripper");
    titleGripper->setFont(font);
    gripperLayout->addWidget(titleGripper);

    QLEGripperPosition = new QLineEdit();
    gripperLayout->addWidget(QLEGripperPosition);
}

void mtsIntuitiveResearchKitMTMQtWidget::timerEventDerived(void)
{
    gripper_measured_js(m_gripper_measured_js);
    if (m_gripper_measured_js.Position().size() > 0) {
        QString text;
        text.setNum(m_gripper_measured_js.Position().at(0) * cmn180_PI, 'f', 3);
        QLEGripperPosition->setText(text);
    } else {
        QLEGripperPosition->setText("");
    }
}
