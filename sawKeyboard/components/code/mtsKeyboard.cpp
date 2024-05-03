/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):	Gorkem Sevinc, Anton Deguet
  Created on:   2009-09-17

  (C) Copyright 2009-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#include <sawKeyboard/mtsKeyboard.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <cisstConfig.h>

#if CISST_HAS_JSON
#include <json/json.h>
#endif // CISST_HAS_JSON

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsKeyboard, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

void mtsKeyboard::Init(void)
{
    mQuitKey = 'q';
    DoneMember = false;
    mInterface = this->AddInterfaceProvided("Keyboard");
    if (mInterface) {
        mInterface->AddMessageEvents();
        mInterface->AddEventWrite(mKeyEvent, "Key", 'c');
    }
}

void mtsKeyboard::Configure(const std::string & filename)
{
#if CISST_HAS_JSON
    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());

    Json::Value jsonConfig, jsonValue;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                 << "File: " << filename << std::endl << "Error(s):" << std::endl
                                 << jsonReader.getFormattedErrorMessages();
        return;
    }

    // search for array of keys
    const Json::Value keys = jsonConfig["keys"];
    if (keys.size() == 0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: configuration files needs \"keys\"" << std::endl;
        return;
    }
    for (unsigned int index = 0; index < keys.size(); ++index) {
        Json::Value key = keys[index];
        std::string character, interfaceType, interfaceName, eventName;
        // key is required
        jsonValue = key["key"];
        if (!jsonValue.empty()) {
            character = jsonValue.asString();
            if (character.size() != 1) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: \"key\" for keys["
                                         << index << "] should be a single character, found \""
                                         << character << "\"" << std::endl;
                return;
            }
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find \"key\" for keys["
                                     << index << "]" << std::endl;
            return;
        }
        // interface-name
        jsonValue = key["interface-name"];
        if (!jsonValue.empty()) {
            interfaceName = jsonValue.asString();
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find \"interface-name\" for keys["
                                     << index << "]" << std::endl;
            return;
        }
        // interface-type
        jsonValue = key["interface-type"];
        if (!jsonValue.empty()) {
            interfaceType = jsonValue.asString();
            if (!((interfaceType == "void-event")
                  || (interfaceType == "void-function")
                  || (interfaceType == "write-function")
                  || (interfaceType == "button-event"))) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: \"interface-type\" for keys["
                                         << index
                                         << "] must be one of \"void-event\", \"void-function\","
                                         << "\"write-function\" or \"button-event\", found: "
                                         << interfaceType << std::endl;
                return;
            }
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find \"interface-type\" for keys["
                                     << index << "]" << std::endl;
            return;
        }
        // create interface based on type
        if (interfaceType == "button-event") {
            jsonValue = key["toggle"];
            if (!jsonValue.empty()) {
                bool toggle = jsonValue.asBool();
                AddKeyButtonEvent(character[0], interfaceName, toggle);
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find \"toggle\" for keys["
                                         << index << "]" << std::endl;
                return;
            }
        } else {
            // this will require a name
            jsonValue = key["name"];
            if (!jsonValue.empty()) {
                std::string name = jsonValue.asString();
                if (interfaceType == "void-event") {
                    AddKeyVoidEvent(character[0], interfaceName, name);
                } else if (interfaceType == "void-function") {
                    AddKeyVoidFunction(character[0], interfaceName, name);
                } else if (interfaceType == "write-function") {
                    AddKeyWriteFunction(character[0], interfaceName, name, false);
                }
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find \"name\" for keys["
                                         << index << "]" << std::endl;
                return;
            }
        }
    }
#else
    CMN_LOG_CLASS_INIT_ERROR << "Configure: this method requires CISST_HAS_JSON, reconfigure cisst with CISST_HAS_JSON" << std::endl;
#endif
}

void mtsKeyboard::AddKeyButtonEvent(char key, const std::string & interfaceName, bool toggle)
{
    // create new key data and add to list
    KeyData * keyData = new KeyData;
    mKeyboardDataMap.insert(std::make_pair(key, keyData));

    // set key action
    keyData->Type = BUTTON_EVENT;
    keyData->Toggle = toggle;
    keyData->State = false;

    // add interface
    mtsInterfaceProvided * providedInterface = this->GetInterfaceProvided(interfaceName);
    if (!providedInterface) {
        providedInterface = this->AddInterfaceProvided(interfaceName);
    }
    keyData->WriteTrigger.Bind(providedInterface->AddEventWrite("Button",
                                                                prmEventButton()));
}

void mtsKeyboard::AddKeyVoidEvent(char key, const std::string & interfaceName, const std::string & eventName)
{
    // create new key data and add to list
    KeyData * keyData = new KeyData;
    mKeyboardDataMap.insert(std::make_pair(key, keyData));

    // set key action
    keyData->Type = VOID_EVENT;
    keyData->State = false;

    // add interface
    mtsInterfaceProvided * providedInterface = this->GetInterfaceProvided(interfaceName);
    if (!providedInterface) {
        providedInterface = this->AddInterfaceProvided(interfaceName);
    }
    keyData->VoidTrigger.Bind(providedInterface->AddEventVoid(eventName));
}

void mtsKeyboard::AddKeyVoidFunction(char key, const std::string & interfaceName, const std::string & commandName)
{
    // create new key data and add to list
    KeyData * keyData = new KeyData;
    mKeyboardDataMap.insert(std::make_pair(key, keyData));

    // set key action
    keyData->Type = VOID_FUNCTION;
    keyData->State = false;

    // add interface
    mtsInterfaceRequired * requiredInterface = this->GetInterfaceRequired(interfaceName);
    if (!requiredInterface) {
        requiredInterface = this->AddInterfaceRequired(interfaceName);
    }
    requiredInterface->AddFunction(commandName, keyData->VoidTrigger);
}

void mtsKeyboard::AddKeyWriteFunction(char key, const std::string & interfaceName, const std::string & commandName, bool initialState)
{
    // create new key data and add to list
    KeyData * keyData = new KeyData;
    mKeyboardDataMap.insert(std::make_pair(key, keyData));

    // set key action
    keyData->Type = WRITE_FUNCTION;
    keyData->State = initialState;

    // add interface
    mtsInterfaceRequired * requiredInterface = this->GetInterfaceRequired(interfaceName);
    if (!requiredInterface) {
        requiredInterface = this->AddInterfaceRequired(interfaceName);
    }
    requiredInterface->AddFunction(commandName, keyData->WriteTrigger);
}

void mtsKeyboard::SetQuitKey(char quitKey)
{
    mQuitKey = quitKey;
}

void mtsKeyboard::Run(void)
{
    // make sure we don't hang or wait for a key press when we are done
    if (Done()) {
        this->Thread.Sleep(10.0 * cmn_ms);
        return;
    }

    // get a key using the blocking cmnGetChar
    const int keyboardInput = mGetCharEnvironment.GetChar();
    if (keyboardInput == mQuitKey) {
        DoneMember = true;
    }

    // emit general event through default provided interface
    mKeyEvent(keyboardInput);

    // see if there is any other event or command to trigger
    if (!this->Done()) {
        prmEventButton event;
        KeyData * keyData;
        KeyDataType::iterator iterator;
        const KeyDataType::iterator end = mKeyboardDataMap.end();
        for (iterator = mKeyboardDataMap.begin();
             iterator != end;
             iterator++) {
            if (iterator->first == keyboardInput) {
                keyData = iterator->second;
                switch (keyData->Type) {
                case BUTTON_EVENT:
                    if (keyData->Toggle == true) {
                        if (keyData->State == true) {
                            event.SetType(prmEventButton::RELEASED);
                            keyData->State = false;
                        } else {
                            event.SetType(prmEventButton::PRESSED);
                            keyData->State = true;
                        }
                    } else {
                        event.SetType(prmEventButton::RELEASED);
                        keyData->State = true;
                    }
                    keyData->WriteTrigger(event);
                    CMN_LOG_CLASS_RUN_DEBUG << "Run " << keyboardInput << " sending button event " << event << std::endl;
                    break;
                case VOID_EVENT:
                    keyData->VoidTrigger();
                    CMN_LOG_CLASS_RUN_DEBUG << "Run " << keyboardInput << " sending void event " << std::endl;
                    break;
                case VOID_FUNCTION:
                    keyData->VoidTrigger();
                    CMN_LOG_CLASS_RUN_DEBUG << "Run " << keyboardInput << " calling void function " << std::endl;
                    break;
                case WRITE_FUNCTION:
                    mtsBool value = keyData->State;
                    keyData->WriteTrigger(value);
                    keyData->State = !(keyData->State);
                    CMN_LOG_CLASS_RUN_DEBUG << "Run " << keyboardInput << " calling write command " << value << std::endl;
                    break;
                }
            }
        }
    }
}
