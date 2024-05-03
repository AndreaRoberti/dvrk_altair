/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-08-24

  (C) Copyright 2013-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitArmQtWidget_h
#define _mtsIntuitiveResearchKitArmQtWidget_h

#include <cisstVector/vctForceTorqueQtWidget.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsEventReceiver.h>
#include <cisstMultiTask/mtsMessageQtWidget.h>
#include <cisstMultiTask/mtsIntervalStatisticsQtWidget.h>

#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmStateJointQtWidget.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianGetQtWidget.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmPositionJointSetQtWidget.h>
#include <cisstParameterTypes/prmOperatingStateQtWidget.h>

#include <QWidget>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>

class QCheckBox;
class QPushButton;
class QTextEdit;

class CISST_EXPORT mtsIntuitiveResearchKitArmQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitArmQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsIntuitiveResearchKitArmQtWidget() {}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    virtual void closeEvent(QCloseEvent * event);

signals:
    void SignalDesiredState(QString state);
    void SignalTrajectoryJointRatio(double ratio);

private slots:
    void timerEvent(QTimerEvent * event);
    void SlotDesiredStateEventHandler(QString state);
    void SlotTrajectoryJointRatio(double ratio);
    void SlotTrajectoryJointRatioEventHandler(double ratio);
    void SlotLogEnabled(void);
    void SlotEnableDirectControl(bool toggle);

private:
    //! setup GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

protected:
    struct ArmStruct {
        mtsFunctionRead configuration_js;
        mtsFunctionRead measured_js;
        mtsFunctionRead measured_cp;
        mtsFunctionRead measured_cf_body;
        mtsFunctionWrite move_jp;
        mtsFunctionRead period_statistics;
        mtsEventReceiverWrite trajectory_j_ratio;
        mtsFunctionWrite trajectory_j_set_ratio;
    } Arm;

    // so derived class has access to custom parts of widget
    QVBoxLayout * MainLayout;
    QHBoxLayout * EffortLayout;

    mtsInterfaceRequired * InterfaceRequired;
    inline virtual void setupUiDerived(void) {};
    inline virtual void timerEventDerived(void) {};
    virtual void SetDirectControl(const bool direct);

    bool DirectControl;
    prmConfigurationJoint ConfigurationJoint;
    prmStateJoint StateJoint;
    prmStateJointQtWidget * QSJWidget;
    QDoubleSpinBox * QSBTrajectoryRatio;

    prmPositionCartesianGet Position;
    prmPositionCartesianGetQtWidget * QCPGWidget;

    prmForceCartesianGet Wrench;
    vctForceTorqueQtWidget * QFTWidget;

    QFrame * QFJoints;
    prmPositionJointSetQtWidget * QPJSWidget;
    void TrajectoryJointRatioEventHandler(const double & ratio);

    // timing
    mtsIntervalStatistics IntervalStatistics;
    mtsIntervalStatisticsQtWidget * QMIntervalStatistics;

    // state
    QCheckBox * QCBEnableDirectControl;

    QLineEdit * QLEDesiredState;
    void DesiredStateEventHandler(const std::string & state);

    prmOperatingStateQtWidget * QPOState;

    // messages
    bool LogEnabled;
    QPushButton * QPBLog;
    mtsMessageQtWidget * QMMessage;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitArmQtWidget);

#endif // _mtsIntuitiveResearchKitArmQtWidget_h
