/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2019-10-10

  (C) Copyright 2019-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveResearchKitECMQtWidget_h
#define _mtsIntuitiveResearchKitECMQtWidget_h

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>


class CISST_EXPORT mtsIntuitiveResearchKitECMQtWidget: public mtsIntuitiveResearchKitArmQtWidget
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitECMQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsIntuitiveResearchKitECMQtWidget() {}

protected:
    void setupUiDerived(void);

signals:
    void SignalEndoscopeType(QString toolType);

private slots:
    void SlotEndoscopeTypeEventHandler(QString toolType);
    void SlotEndoscopeTypeSelected(QString toolType);

private:
    void EndoscopeTypeEventHandler(const std::string & toolType);

    QLineEdit * QLEEndoscopeType;
    QComboBox * QCBEndoscopeOptions;
    mtsFunctionWrite set_endoscope_type;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitECMQtWidget);

#endif // _mtsIntuitiveResearchKitECMQtWidget_h
