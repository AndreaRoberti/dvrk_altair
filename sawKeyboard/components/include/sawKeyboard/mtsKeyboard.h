/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Gorkem Sevinc, Anton Deguet
  Created on: 2009-09-17

  (C) Copyright 2009-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsKeyboard_h
#define _mtsKeyboard_h

#include <cisstCommon/cmnGetChar.h>
#include <cisstMultiTask/mtsForwardDeclarations.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <sawKeyboard/sawKeyboardRevision.h>
#include <map>

// Always include last
#include <sawKeyboard/sawKeyboardExport.h>

/*! Class used to create trigger events and commands using a keyboard.
  The user has to bind keys to a given required interface function or
  to a provided interface event.

  This class also maintains an internal flag for a special "Quit" key.
  The user can define the quit key using SetQuitKey and query if the
  key has been pressed using if (keyboard.Done()). */
class CISST_EXPORT mtsKeyboard: public mtsTaskContinuous {
    // declare services, requires dynamic creation
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);
 public:
    inline mtsKeyboard(const std::string & componentName = "keyboard"):
        mtsTaskContinuous(componentName, 256) {
        Init();
    }

    inline mtsKeyboard(const mtsTaskContinuousConstructorArg & arg):
        mtsTaskContinuous(arg) {
        Init();
    }

    void Configure(const std::string & filename = "");
    inline void Startup(void) {
        mGetCharEnvironment.Activate();
    };
    void Run(void);
    inline void Cleanup(void) {
        mGetCharEnvironment.DeActivate();
    };

    enum TriggerType {BUTTON_EVENT, VOID_EVENT, VOID_FUNCTION, WRITE_FUNCTION};

    /*! Add a write event with a payload of type prmEventButton to the
      provided interface "interfaceName".  If the keyboard component doesn't
      already have such interface, it will create one.

      The toggle flag allows to emulate a continuously pressed button
      and will emit alternatively prmEventButton::PRESSED and
      prmEventButton::RELEASED.  It the toggle is set to false, the
      event is always of type prmEventButton::RELEASED.  */
    void AddKeyButtonEvent(char key, const std::string & interfaceName, bool toggle);

    /*! Add a void event to the provided interface "interfaceName".
      If the keyboard component doesn't already have such interface,
      it will create one. */
    void AddKeyVoidEvent(char key, const std::string & interfaceName, const std::string & eventName);

    /*! Add a void function to required interface "interfaceName".
      If the keyboard component doesn't already have such interface,
      it will create one. */
    void AddKeyVoidFunction(char key, const std::string & interfaceName, const std::string & commandName);

    /*! Add a write function to required interface "interfaceName".
      If the keyboard component doesn't already have such interface,
      it will create one.  The function will send an mtsBool,
      alternatively 'true' and 'false'. */
    void AddKeyWriteFunction(char key, const std::string & interfaceName, const std::string & commandName, bool initialState);

    // for backward compatibility
    inline CISST_DEPRECATED void AddKeyWriteCommand(char key, const std::string & interfaceName, const std::string & commandName, bool initialState) {
        AddKeyWriteFunction(key, interfaceName, commandName, initialState);
    }

    /*! Set the quit key.  Once the quit key has been pressed the
      keyboard component stop triggering events and commands.  State
      can be checked using the Done() method. */
    void SetQuitKey(char quitKey);

    /*! Data used internally to store all the possible actions
      associated to a key pressed. */
    struct KeyData {
        bool Toggle;
        bool State;
        mtsFunctionWrite WriteTrigger;
        mtsFunctionVoid VoidTrigger;
        TriggerType Type;
    };

 protected:

    /*! Code called by all constructors. */
    void Init(void);

    typedef std::multimap<char, KeyData *> KeyDataType;
    KeyDataType mKeyboardDataMap;

    mtsInterfaceProvided * mInterface;
    mtsFunctionWrite mKeyEvent;

    cmnGetCharEnvironment mGetCharEnvironment;
    char mQuitKey;
    CMN_DECLARE_MEMBER_AND_ACCESSORS(bool, Done);
};


CMN_DECLARE_SERVICES_INSTANTIATION(mtsKeyboard);

#endif // _mtsKeyboard_h
