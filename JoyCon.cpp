/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com

 IR camera support added by Allan Glover (adglover9.81@gmail.com) and Kristian Lauszus
 */

#include "JoyCon.h"
// To enable serial debugging see "settings.h"
#define EXTRADEBUG // Uncomment to get even more debugging data
#define PRINTREPORT // Uncomment to print the report send by the Joy-Con controllers

const uint8_t JOYCON_LEDS[] PROGMEM = {
        0x00, // OFF
        0x10, // LED1
        0x20, // LED2
        0x40, // LED3
        0x80, // LED4

        0x90, // LED5
        0xA0, // LED6
        0xC0, // LED7
        0xD0, // LED8
        0xE0, // LED9
        0xF0, // LED10
};

const uint32_t JOYCON_BUTTONS[] PROGMEM = {
        0x00000004, // UP 0
        0x00000008, // RIGHT 1
        0x00000002, // DOWN 2
        0x00000001, // LEFT 3
                    // Skip 4
        0x00000,    // PLUS 5
                    // Skip 6
                    // Skip 7
        0x00000100, // MINUS 8
        0x00000,    // HOME 9
                    // Skip 10
                    // Skip 11
        0x00000,    // B 12
        0x00000,    // A 13
        0x00000,    // Y 14
        0x00000,    // X 15

        0x00020,    // L 16
        0x00002,    // R 17
        0x08000,    // ZL 18
        0x00400,    // ZR 19
        0x00000,    // LSL = 20
        0x00000,    // LSR = 21
        0x00000,    // RSL = 22
        0x00000,    // RSR = 23
        //0x00000400, // LSTICK = 19,
        //0x00000,    // RSTICK = 20
};

JOYCON::JOYCON(BTD *p, bool pair) :
BluetoothService(p) // Pointer to USB class instance - mandatory
{
        pBtd->pairWithJoyCon = pair;

        HIDBuffer[0] = 0xA2; // HID BT DATA_request (0xA0) | Report Type (Output 0x02)

        /* Set device cid for the control and intterrupt channelse - LSB */
        control_dcid[0] = 0x60; // 0x0060
        control_dcid[1] = 0x00;
        interrupt_dcid[0] = 0x61; // 0x0061
        interrupt_dcid[1] = 0x00;

        Reset();
}

void JOYCON::Reset() {
        joyconConnected = false;
        motionValuesReset = false;
        activeConnection = false;
        l2cap_event_flag = 0; // Reset flags
        l2cap_state = L2CAP_WAIT;
}

void JOYCON::disconnect() { // Use this void to disconnect any of the controllers
        timer = (uint32_t)millis(); // Don't wait
        // First the HID interrupt channel has to be disconnected, then the HID control channel and finally the HCI connection
        pBtd->l2cap_disconnection_request(hci_handle, ++identifier, interrupt_scid, interrupt_dcid);
        Reset();
        l2cap_state = L2CAP_INTERRUPT_DISCONNECT;
}

void JOYCON::ACLData(uint8_t* l2capinbuf) {
        Serial.printf("\r\nACLDATA %d %d %d %d\n",pBtd->l2capConnectionClaimed , pBtd->incomingJoyCon , joyconConnected , activeConnection);
        for(uint8_t i = 0; i < 20; i++) {
                D_PrintHex<uint8_t > (l2capinbuf[i], 0x80);
                Notify(PSTR(" "), 0x80);
        }
        if(!pBtd->l2capConnectionClaimed && pBtd->incomingJoyCon && !joyconConnected && !activeConnection) {
                if(l2capinbuf[8] == L2CAP_CMD_CONNECTION_REQUEST) {
                        if((l2capinbuf[12] | (l2capinbuf[13] << 8)) == HID_CTRL_PSM) {
                                pBtd->incomingJoyCon = false;
                                pBtd->l2capConnectionClaimed = true; // Claim that the incoming connection belongs to this service
                                activeConnection = true;
                                hci_handle = pBtd->hci_handle; // Store the HCI Handle for the connection
                                l2cap_state = L2CAP_WAIT;
                        }
                }
        }

        if(checkHciHandle(l2capinbuf, hci_handle)) { // acl_handle_ok
                if((l2capinbuf[6] | (l2capinbuf[7] << 8)) == 0x0001U) { // l2cap_control - Channel ID for ACL-U
                  Notify(PSTR("\r\nL2CAP Command: "), 0x80);
                  D_PrintHex<uint8_t > (l2capinbuf[8], 0x80);
                        if(l2capinbuf[8] == L2CAP_CMD_COMMAND_REJECT) {
#ifdef DEBUG_USB_HOST
                                Notify(PSTR("\r\nL2CAP Command Rejected - Reason: "), 0x80);
                                D_PrintHex<uint8_t > (l2capinbuf[13], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t > (l2capinbuf[12], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t > (l2capinbuf[17], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t > (l2capinbuf[16], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t > (l2capinbuf[15], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t > (l2capinbuf[14], 0x80);
#endif
                        } else if(l2capinbuf[8] == L2CAP_CMD_CONNECTION_RESPONSE) {
                                Notify(PSTR("\r\nL2CAP Connection Response"), 0x80);
                                if(((l2capinbuf[16] | (l2capinbuf[17] << 8)) == 0x0000) && ((l2capinbuf[18] | (l2capinbuf[19] << 8)) == SUCCESSFUL)) { // Success
                                        if(l2capinbuf[14] == control_dcid[0] && l2capinbuf[15] == control_dcid[1]) {
                                                Notify(PSTR("\r\nHID Control Connection Complete"), 0x80);
                                                identifier = l2capinbuf[9];
                                                control_scid[0] = l2capinbuf[12];
                                                control_scid[1] = l2capinbuf[13];
                                                l2cap_set_flag(L2CAP_FLAG_CONTROL_CONNECTED);
                                        } else if(l2capinbuf[14] == interrupt_dcid[0] && l2capinbuf[15] == interrupt_dcid[1]) {
                                                Notify(PSTR("\r\nHID Interrupt Connection Complete"), 0x80);
                                                identifier = l2capinbuf[9];
                                                interrupt_scid[0] = l2capinbuf[12];
                                                interrupt_scid[1] = l2capinbuf[13];
                                                l2cap_set_flag(L2CAP_FLAG_INTERRUPT_CONNECTED);
                                        }
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_CONNECTION_REQUEST) {
#ifdef EXTRADEBUG
                                Notify(PSTR("\r\nL2CAP Connection Request - PSM: "), 0x80);
                                D_PrintHex<uint8_t > (l2capinbuf[13], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t > (l2capinbuf[12], 0x80);
                                Notify(PSTR(" SCID: "), 0x80);
                                D_PrintHex<uint8_t > (l2capinbuf[15], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t > (l2capinbuf[14], 0x80);
                                Notify(PSTR(" Identifier: "), 0x80);
                                D_PrintHex<uint8_t > (l2capinbuf[9], 0x80);
#endif
                                if((l2capinbuf[12] | (l2capinbuf[13] << 8)) == HID_CTRL_PSM) {
                                        Notify(PSTR("\r\nL2CAP CTRL PSM "), 0x80);
                                        identifier = l2capinbuf[9];
                                        control_scid[0] = l2capinbuf[14];
                                        control_scid[1] = l2capinbuf[15];
                                        l2cap_set_flag(L2CAP_FLAG_CONNECTION_CONTROL_REQUEST);
                                } else if((l2capinbuf[12] | (l2capinbuf[13] << 8)) == HID_INTR_PSM) {
                                        Notify(PSTR("\r\nL2CAP INTR PSM "), 0x80);
                                        identifier = l2capinbuf[9];
                                        interrupt_scid[0] = l2capinbuf[14];
                                        interrupt_scid[1] = l2capinbuf[15];
                                        l2cap_set_flag(L2CAP_FLAG_CONNECTION_INTERRUPT_REQUEST);
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_CONFIG_RESPONSE) {
                                if((l2capinbuf[16] | (l2capinbuf[17] << 8)) == 0x0000) { // Success
                                        if(l2capinbuf[12] == control_dcid[0] && l2capinbuf[13] == control_dcid[1]) {
                                                Notify(PSTR("\r\nHID Control Configuration Complete "), 0x80);
                                                D_PrintHex<uint8_t > (l2capinbuf[9], 0x80);
                                                identifier = l2capinbuf[9];
                                                l2cap_set_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS);
                                        } else if(l2capinbuf[12] == interrupt_dcid[0] && l2capinbuf[13] == interrupt_dcid[1]) {
                                                Notify(PSTR("\r\nHID Interrupt Configuration Complete "), 0x80);
                                                D_PrintHex<uint8_t > (l2capinbuf[9], 0x80);
                                                identifier = l2capinbuf[9];
                                                l2cap_set_flag(L2CAP_FLAG_CONFIG_INTERRUPT_SUCCESS);
                                        }
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_CONFIG_REQUEST) {
                                if(l2capinbuf[12] == control_dcid[0] && l2capinbuf[13] == control_dcid[1]) {
                                        Notify(PSTR("\r\nHID Control Configuration Request"), 0x80);
                                        pBtd->l2cap_config_response(hci_handle, l2capinbuf[9], control_scid);
                                } else if(l2capinbuf[12] == interrupt_dcid[0] && l2capinbuf[13] == interrupt_dcid[1]) {
                                        Notify(PSTR("\r\nHID Interrupt Configuration Request"), 0x80);
                                        pBtd->l2cap_config_response(hci_handle, l2capinbuf[9], interrupt_scid);
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_DISCONNECT_REQUEST) {
                                if(l2capinbuf[12] == control_dcid[0] && l2capinbuf[13] == control_dcid[1]) {
#ifdef DEBUG_USB_HOST
                                        Notify(PSTR("\r\nDisconnect Request: Control Channel"), 0x80);
#endif
                                        identifier = l2capinbuf[9];
                                        pBtd->l2cap_disconnection_response(hci_handle, identifier, control_dcid, control_scid);
                                        Reset();
                                } else if(l2capinbuf[12] == interrupt_dcid[0] && l2capinbuf[13] == interrupt_dcid[1]) {
#ifdef DEBUG_USB_HOST
                                        Notify(PSTR("\r\nDisconnect Request: Interrupt Channel"), 0x80);
#endif
                                        identifier = l2capinbuf[9];
                                        pBtd->l2cap_disconnection_response(hci_handle, identifier, interrupt_dcid, interrupt_scid);
                                        Reset();
                                }
                        } else if(l2capinbuf[8] == L2CAP_CMD_DISCONNECT_RESPONSE) {
                                if(l2capinbuf[12] == control_scid[0] && l2capinbuf[13] == control_scid[1]) {
                                        Notify(PSTR("\r\nDisconnect Response: Control Channel"), 0x80);
                                        identifier = l2capinbuf[9];
                                        l2cap_set_flag(L2CAP_FLAG_DISCONNECT_CONTROL_RESPONSE);
                                } else if(l2capinbuf[12] == interrupt_scid[0] && l2capinbuf[13] == interrupt_scid[1]) {
                                        Notify(PSTR("\r\nDisconnect Response: Interrupt Channel"), 0x80);
                                        identifier = l2capinbuf[9];
                                        l2cap_set_flag(L2CAP_FLAG_DISCONNECT_INTERRUPT_RESPONSE);
                                }
                        }
#ifdef EXTRADEBUG
                        else {
                                identifier = l2capinbuf[9];
                                Notify(PSTR("\r\nL2CAP Unknown Signaling Command: "), 0x80);
                                D_PrintHex<uint8_t > (l2capinbuf[8], 0x80);
                        }
#endif
                } else if(l2capinbuf[6] == interrupt_dcid[0] && l2capinbuf[7] == interrupt_dcid[1]) { // l2cap_interrupt

#ifdef PRINTREPORT
                        Notify(PSTR("\r\nL2CAP Interrupt: "), 0x80);
                        D_PrintHex<uint32_t > (l2capinbuf[9], 0x80);
                        Notify(PSTR("\r\n"), 0x80);
#endif
                        if(l2capinbuf[8] == 0xA1) { // HID_THDR_DATA_INPUT

                                if((l2capinbuf[9] >= 0x20 && l2capinbuf[9] <= 0x22) || (l2capinbuf[9] >= 0x30 && l2capinbuf[9] <= 0x37) || l2capinbuf[9] == 0x3e || l2capinbuf[9] == 0x3f) { // These reports include the buttons
                                        if((l2capinbuf[9] >= 0x20 && l2capinbuf[9] <= 0x22) || l2capinbuf[9] == 0x31 || l2capinbuf[9] == 0x33) // These reports have no extensions bytes
                                                ButtonState = (uint32_t)((l2capinbuf[10] & 0x1F) | ((uint16_t)(l2capinbuf[11] & 0x9F) << 8));

                                        else if(!unknownExtensionConnected)
                                                ButtonState = (uint32_t)((l2capinbuf[10] & 0x1F) | ((uint16_t)(l2capinbuf[11] & 0x9F) << 8));
#ifdef PRINTREPORT
                                        Notify(PSTR("ButtonState: "), 0x80);
                                        D_PrintHex<uint32_t > (ButtonState, 0x80);
                                        Notify(PSTR("\r\n"), 0x80);
#endif
                                        if(ButtonState != OldButtonState) {
                                                ButtonClickState = ButtonState & ~OldButtonState; // Update click state variable
                                                OldButtonState = ButtonState;
                                        }
                                }
                                if(l2capinbuf[9] == 0x31 || l2capinbuf[9] == 0x33 || l2capinbuf[9] == 0x35 || l2capinbuf[9] == 0x37) { // Read the accelerometer
                                        accXwiimote = ((l2capinbuf[12] << 2) | (l2capinbuf[10] & 0x60 >> 5)) - 500;
                                        accYwiimote = ((l2capinbuf[13] << 2) | (l2capinbuf[11] & 0x20 >> 4)) - 500;
                                        accZwiimote = ((l2capinbuf[14] << 2) | (l2capinbuf[11] & 0x40 >> 5)) - 500;
                                }
                                switch(l2capinbuf[9]) {
                                        case 0x20: // Status Information - (a1) 20 BB BB LF 00 00 VV
#ifdef EXTRADEBUG
                                                Notify(PSTR("\r\nStatus report was received"), 0x80);
#endif
                                                wiiState = l2capinbuf[12]; // (0x01: Battery is nearly empty), (0x02:  An Extension Controller is connected), (0x04: Speaker enabled), (0x08: IR enabled), (0x10: LED1, 0x20: LED2, 0x40: LED3, 0x80: LED4)
                                                batteryLevel = l2capinbuf[15]; // Update battery level

                                                if(!checkBatteryLevel) { // If this is true it means that the user must have called getBatteryLevel()
                                                        if(l2capinbuf[12] & 0x02) { // Check if a extension is connected
#ifdef DEBUG_USB_HOST
                                                                if(!unknownExtensionConnected)
                                                                        Notify(PSTR("\r\nExtension connected"), 0x80);
#endif
                                                                unknownExtensionConnected = true;
#ifdef WIICAMERA
                                                                if(!isIRCameraEnabled()) // Don't activate the Motion Plus if we are trying to initialize the IR camera
#endif
                                                                        setReportMode(false, 0x35); // Also read the extension
                                                        } else {
#ifdef DEBUG_USB_HOST
                                                                Notify(PSTR("\r\nExtension disconnected"), 0x80);
#endif

                                                        }
                                                }
                                                else {
#ifdef EXTRADEBUG
                                                        Notify(PSTR("\r\nChecking battery level"), 0x80);
#endif
                                                        checkBatteryLevel = false; // Check for extensions by default
                                                }
#ifdef DEBUG_USB_HOST
                                                if(l2capinbuf[12] & 0x01)
                                                        Notify(PSTR("\r\nWARNING: Battery is nearly empty"), 0x80);
#endif

                                                break;
                                        case 0x21: // Read Memory Data
                                                if((l2capinbuf[12] & 0x0F) == 0) { // No error
                                                        uint8_t reportLength = (l2capinbuf[12] >> 4) + 1; // // Bit 4-7 is the length - 1
                                                        // See: http://wiibrew.org/wiki/Wiimote/Extension_Controllers

#ifdef DEBUG_USB_HOST
                                                        {
                                                                Notify(PSTR("\r\nUnknown Device: "), 0x80);
                                                                D_PrintHex<uint8_t > (l2capinbuf[13], 0x80);
                                                                D_PrintHex<uint8_t > (l2capinbuf[14], 0x80);
                                                                Notify(PSTR("\r\nData: "), 0x80);
                                                                for(uint8_t i = 0; i < reportLength; i++) {
                                                                        D_PrintHex<uint8_t > (l2capinbuf[15 + i], 0x80);
                                                                        Notify(PSTR(" "), 0x80);
                                                                }
                                                        }
#endif
                                                }
#ifdef EXTRADEBUG
                                                else {
                                                        Notify(PSTR("\r\nReport Error: "), 0x80);
                                                        D_PrintHex<uint8_t > (l2capinbuf[13], 0x80);
                                                        D_PrintHex<uint8_t > (l2capinbuf[14], 0x80);
                                                }
#endif
                                                break;
                                        case 0x22: // Acknowledge output report, return function result
#ifdef DEBUG_USB_HOST
                                                if(l2capinbuf[13] != 0x00) { // Check if there is an error
                                                        Notify(PSTR("\r\nCommand failed: "), 0x80);
                                                        D_PrintHex<uint8_t > (l2capinbuf[12], 0x80);
                                                }
#endif
                                                break;
                                        case 0x30: // Core buttons - (a1) 30 BB BB
                                                break;
                                        case 0x31: // Core Buttons and Accelerometer - (a1) 31 BB BB AA AA AA
                                                break;
                                        case 0x32:
                                                break;
                                        case 0x33: // Core Buttons with Accelerometer and 12 IR bytes - (a1) 33 BB BB AA AA AA II II II II II II II II II II II II
#ifdef WIICAMERA
                                                // Read the IR data
                                                IR_object_x1 = (l2capinbuf[15] | ((uint16_t)(l2capinbuf[17] & 0x30) << 4)); // x position
                                                IR_object_y1 = (l2capinbuf[16] | ((uint16_t)(l2capinbuf[17] & 0xC0) << 2)); // y position
                                                IR_object_s1 = (l2capinbuf[17] & 0x0F); // Size value, 0-15

                                                IR_object_x2 = (l2capinbuf[18] | ((uint16_t)(l2capinbuf[20] & 0x30) << 4));
                                                IR_object_y2 = (l2capinbuf[19] | ((uint16_t)(l2capinbuf[20] & 0xC0) << 2));
                                                IR_object_s2 = (l2capinbuf[20] & 0x0F);

                                                IR_object_x3 = (l2capinbuf[21] | ((uint16_t)(l2capinbuf[23] & 0x30) << 4));
                                                IR_object_y3 = (l2capinbuf[22] | ((uint16_t)(l2capinbuf[23] & 0xC0) << 2));
                                                IR_object_s3 = (l2capinbuf[23] & 0x0F);

                                                IR_object_x4 = (l2capinbuf[24] | ((uint16_t)(l2capinbuf[26] & 0x30) << 4));
                                                IR_object_y4 = (l2capinbuf[25] | ((uint16_t)(l2capinbuf[26] & 0xC0) << 2));
                                                IR_object_s4 = (l2capinbuf[26] & 0x0F);
#endif
                                                break;
                                        case 0x34: // Core Buttons with 19 Extension bytes - (a1) 34 BB BB EE EE EE EE EE EE EE EE EE EE EE EE EE EE EE EE EE EE EE
                                                break;
                                                /* 0x3e and 0x3f both give unknown report types when report mode is 0x3e or 0x3f with mode number 0x05 */
                                        case 0x3E: // Core Buttons with Accelerometer and 32 IR bytes
                                                // (a1) 31 BB BB AA AA AA II II II II II II II II II II II II II II II II II II II II II II II II II II II II II II II II
                                                // corresponds to output report mode 0x3e

                                                /**** for reading in full mode: DOES NOT WORK YET ****/
                                                /* When it works it will also have intensity and bounding box data */
                                                /*
                                                IR_object_x1 = (l2capinbuf[13] | ((uint16_t)(l2capinbuf[15] & 0x30) << 4));
                                                IR_object_y1 = (l2capinbuf[14] | ((uint16_t)(l2capinbuf[15] & 0xC0) << 2));
                                                IR_object_s1 = (l2capinbuf[15] & 0x0F);
                                                 */
                                                break;
                                        case 0x3F:
                                                /*
                                                IR_object_x1 = (l2capinbuf[13] | ((uint16_t)(l2capinbuf[15] & 0x30) << 4));
                                                IR_object_y1 = (l2capinbuf[14] | ((uint16_t)(l2capinbuf[15] & 0xC0) << 2));
                                                IR_object_s1 = (l2capinbuf[15] & 0x0F);
                                                 */
                                                break;

#ifdef DEBUG_USB_HOST
                                        default:
                                                Notify(PSTR("\r\nUnknown Report type: "), 0x80);
                                                D_PrintHex<uint8_t > (l2capinbuf[9], 0x80);
                                                break;
#endif
                                }
                        }
                }
                L2CAP_task();
        }
}

void JOYCON::L2CAP_task() {
        switch(l2cap_state) {
                        /* These states are used if the Wiimote is the host */
                case L2CAP_CONTROL_SUCCESS:
                        if(l2cap_check_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS)) {
#ifdef DEBUG_USB_HOST
                                Notify(PSTR("\r\nHID Control Successfully Configured"), 0x80);
#endif
                                l2cap_state = L2CAP_INTERRUPT_SETUP;
                        }
                        break;

                case L2CAP_INTERRUPT_SETUP:
                        if(l2cap_check_flag(L2CAP_FLAG_CONNECTION_INTERRUPT_REQUEST)) {
#ifdef DEBUG_USB_HOST
                                Notify(PSTR("\r\nHID Interrupt Incoming Connection Request"), 0x80);
#endif
                                pBtd->l2cap_connection_response(hci_handle, identifier, interrupt_dcid, interrupt_scid, PENDING);
                                delay(1);
                                pBtd->l2cap_connection_response(hci_handle, identifier, interrupt_dcid, interrupt_scid, SUCCESSFUL);
                                identifier++;
                                delay(1);
                                pBtd->l2cap_config_request(hci_handle, identifier, interrupt_scid);

                                l2cap_state = L2CAP_INTERRUPT_CONFIG_REQUEST;
                        }
                        break;

                        /* These states are used if the Arduino is the host */
                case L2CAP_CONTROL_CONNECT_REQUEST:
                        if(l2cap_check_flag(L2CAP_FLAG_CONTROL_CONNECTED)) {
#ifdef DEBUG_USB_HOST
                                Notify(PSTR("\r\nSend HID Control Config Request"), 0x80);
#endif
                                identifier++;
                                pBtd->l2cap_config_request(hci_handle, identifier, control_scid);
                                l2cap_state = L2CAP_CONTROL_CONFIG_REQUEST;
                        }
                        break;

                case L2CAP_CONTROL_CONFIG_REQUEST:
                        if(l2cap_check_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS)) {
#ifdef DEBUG_USB_HOST
                                Notify(PSTR("\r\nSend HID Interrupt Connection Request"), 0x80);
#endif
                                identifier++;
                                pBtd->l2cap_connection_request(hci_handle, identifier, interrupt_dcid, HID_INTR_PSM);
                                l2cap_state = L2CAP_INTERRUPT_CONNECT_REQUEST;
                        }
                        break;

                case L2CAP_INTERRUPT_CONNECT_REQUEST:
                        if(l2cap_check_flag(L2CAP_FLAG_INTERRUPT_CONNECTED)) {
#ifdef DEBUG_USB_HOST
                                Notify(PSTR("\r\nSend HID Interrupt Config Request"), 0x80);
#endif
                                identifier++;
                                pBtd->l2cap_config_request(hci_handle, identifier, interrupt_scid);
                                l2cap_state = L2CAP_INTERRUPT_CONFIG_REQUEST;
                        }
                        break;

                case L2CAP_INTERRUPT_CONFIG_REQUEST:
                        if(l2cap_check_flag(L2CAP_FLAG_CONFIG_INTERRUPT_SUCCESS)) { // Now the HID channels is established
#ifdef DEBUG_USB_HOST
                                Notify(PSTR("\r\nHID Channels Established"), 0x80);
#endif
                                pBtd->connectToJoyCon = false;
                                pBtd->pairWithJoyCon = false;
                                stateCounter = 0;
                                //l2cap_state = WII_CHECK_MOTION_PLUS_STATE;
                                l2cap_state = TURN_ON_LED;
                        }
                        break;

                        /* The next states are in run() */

                case L2CAP_INTERRUPT_DISCONNECT:
                        if(l2cap_check_flag(L2CAP_FLAG_DISCONNECT_INTERRUPT_RESPONSE) && ((int32_t)((uint32_t)millis() - timer) >= 0L)) {
#ifdef DEBUG_USB_HOST
                                Notify(PSTR("\r\nDisconnected Interrupt Channel"), 0x80);
#endif
                                identifier++;
                                pBtd->l2cap_disconnection_request(hci_handle, identifier, control_scid, control_dcid);
                                l2cap_state = L2CAP_CONTROL_DISCONNECT;
                        }
                        break;

                case L2CAP_CONTROL_DISCONNECT:
                        if(l2cap_check_flag(L2CAP_FLAG_DISCONNECT_CONTROL_RESPONSE)) {
#ifdef DEBUG_USB_HOST
                                Notify(PSTR("\r\nDisconnected Control Channel"), 0x80);
#endif
                                pBtd->hci_disconnect(hci_handle);
                                hci_handle = -1; // Reset handle
                                l2cap_event_flag = 0; // Reset flags
                                l2cap_state = L2CAP_WAIT;
                        }
                        break;
        }
}

void JOYCON::Run() {
        if(l2cap_state == L2CAP_INTERRUPT_DISCONNECT && ((int32_t)((uint32_t)millis() - timer) >= 0L))
                L2CAP_task(); // Call the rest of the disconnection routine after we have waited long enough

        switch(l2cap_state) {
                case L2CAP_WAIT:
                        if(pBtd->connectToJoyCon && !pBtd->l2capConnectionClaimed && !joyconConnected && !activeConnection) {
                                pBtd->l2capConnectionClaimed = true;
                                activeConnection = true;
#ifdef DEBUG_USB_HOST
                                Notify(PSTR("\r\nSend HID Control Connection Request"), 0x80);
#endif
                                hci_handle = pBtd->hci_handle; // Store the HCI Handle for the connection
                                l2cap_event_flag = 0; // Reset flags
                                identifier = 0;
                                pBtd->l2cap_connection_request(hci_handle, identifier, control_dcid, HID_CTRL_PSM);
                                l2cap_state = L2CAP_CONTROL_CONNECT_REQUEST;
                        } else if(l2cap_check_flag(L2CAP_FLAG_CONNECTION_CONTROL_REQUEST)) {
#ifdef DEBUG_USB_HOST
                                Notify(PSTR("\r\nHID Control Incoming Connection Request"), 0x80);
#endif
                                pBtd->l2cap_connection_response(hci_handle, identifier, control_dcid, control_scid, PENDING);
                                delay(1);
                                pBtd->l2cap_connection_response(hci_handle, identifier, control_dcid, control_scid, SUCCESSFUL);
                                identifier++;
                                delay(1);
                                pBtd->l2cap_config_request(hci_handle, identifier, control_scid);
                                l2cap_state = L2CAP_CONTROL_SUCCESS;
                        }
                        break;


                case WII_CHECK_EXTENSION_STATE: // This is used to check if there is anything plugged in to the extension port
#ifdef DEBUG_USB_HOST
                        if(stateCounter == 0) // Only print onnce
                                Notify(PSTR("\r\nChecking if there is any extension connected"), 0x80);
#endif
                        stateCounter++; // We use this counter as there has to be a short delay between the commands
                        if(stateCounter == 1)
                                statusRequest(); // See if a new device has connected
                        if(stateCounter == 100) {
                                if(unknownExtensionConnected) // Check if there is a extension is connected to the port
                                        initExtension1();
                                else
                                        stateCounter = 499;
                        } else if(stateCounter == 200)
                                initExtension2();
                        else if(stateCounter == 300) {
                                readExtensionType();
                                unknownExtensionConnected = false;
                        } else if(stateCounter == 400) {
                                stateCounter = 499;
                        } else if(stateCounter == 500) {
                                stateCounter = 0;
                                l2cap_state = TURN_ON_LED;
                        }
                        break;


                case TURN_ON_LED:

                        joyconConnected = true;
                        onInit();
                        l2cap_state = L2CAP_DONE;
                        break;

                case L2CAP_DONE:
                        if(unknownExtensionConnected) {
#ifdef DEBUG_USB_HOST
                                if(stateCounter == 0) // Only print once
                                        Notify(PSTR("\r\nChecking extension port"), 0x80);
#endif
                                stateCounter++; // We will use this counter as there has to be a short delay between the commands
                                if(stateCounter == 50)
                                        statusRequest();
                                else if(stateCounter == 100)
                                        initExtension1();
                                else if(stateCounter == 150)
                                        stateCounter = 299; // There is no extension connected
                                else if(stateCounter == 200)
                                        readExtensionType();
                                else if(stateCounter == 250) {
//TODO XXXXXXXXXX
                                }else if(stateCounter == 400)
                                        readExtensionType(); // Check if it has been activated
                                else if(stateCounter == 450) {
                                        onInit();
                                        stateCounter = 0;
                                        unknownExtensionConnected = false;
                                }
                        } else
                                stateCounter = 0;
                        break;
        }
}

/************************************************************/
/*                    HID Commands                          */
/************************************************************/

void JOYCON::HID_Command(uint8_t* data, uint8_t nbytes) {
        // if(motionPlusInside)
        //         pBtd->L2CAP_Command(hci_handle, data, nbytes, interrupt_scid[0], interrupt_scid[1]); // It's the new Wiimote with the Motion Plus Inside or Wii U Pro controller
        // else
                  pBtd->L2CAP_Command(hci_handle, data, nbytes, control_scid[0], control_scid[1]);

        Notify(PSTR("\r\n"), 0x80);
        for(uint8_t i = 0; i < nbytes; i++) {
                D_PrintHex<uint8_t > (data[i], 0x80);
                Notify(PSTR(" "), 0x80);
        }


        //packetNum++;
}

void JOYCON::setAllOff() {
        HIDBuffer[1] = 0x11;
        HIDBuffer[2] = 0x00;
        HID_Command(HIDBuffer, 3);
}

void JOYCON::setRumbleOff() {
        HIDBuffer[1] = 0x11;
        HIDBuffer[2] &= ~0x01; // Bit 0 control the rumble
        HID_Command(HIDBuffer, 3);
}

void JOYCON::setRumbleOn() {
        HIDBuffer[1] = 0x11;
        HIDBuffer[2] |= 0x01; // Bit 0 control the rumble
        HID_Command(HIDBuffer, 3);
}

void JOYCON::setRumbleToggle() {
        HIDBuffer[1] = 0x11;
        HIDBuffer[2] ^= 0x01; // Bit 0 control the rumble
        HID_Command(HIDBuffer, 3);
}

void JOYCON::setLedRaw(uint8_t value) {
        HIDBuffer[1] = 0x11;
        HIDBuffer[2] = value | (HIDBuffer[2] & 0x01); // Keep the rumble bit
        HID_Command(HIDBuffer, 3);
}

void JOYCON::setLedOff(LEDEnum a) {
        HIDBuffer[1] = 0x11;
        HIDBuffer[2] &= ~(pgm_read_byte(&JOYCON_LEDS[(uint8_t)a]));
        HID_Command(HIDBuffer, 3);
}

void JOYCON::setLedOn(LEDEnum a) {
        if(a == OFF)
                setLedRaw(0);
        else {
                HIDBuffer[1] = 0x11;
                HIDBuffer[2] |= pgm_read_byte(&JOYCON_LEDS[(uint8_t)a]);
                HID_Command(HIDBuffer, 3);
        }
}

void JOYCON::setLedToggle(LEDEnum a) {
        HIDBuffer[1] = 0x11;
        HIDBuffer[2] ^= pgm_read_byte(&JOYCON_LEDS[(uint8_t)a]);
        HID_Command(HIDBuffer, 3);
}

void JOYCON::setLedStatus() {
  #ifdef EXTRADEBUG
          Notify(PSTR("\r\nSetting LED Status"), 0x80);
  #endif
  uint8_t cmd_buf[0x40];
  bzero(cmd_buf, 0x40);
  cmd_buf[0] = 0x01;
  send_subcommand(0x01, 0x30, cmd_buf, 1);


        // uint8_t cmd_buf[0x40];
        // bzero(cmd_buf, 0x40);
        // cmd_buf[0] = 0x01;
        // cmd_buf[1] = 1;
        // memcpy(cmd_buf + 2, rumbledata, 8);
        // cmd_buf[10] = 0x30;
        // cmd_buf[11] = 0x01;
        // HID_Command(cmd_buf, 0x40);
        // HIDBuffer[1] = 0x11;
        // HIDBuffer[2] = (HIDBuffer[2] & 0x01); // Keep the rumble bit
        // if(joyconConnected)
        //         HIDBuffer[2] |= 0x10; // If it's connected LED1 will light up
        //
        // HID_Command(HIDBuffer, 3);
}

uint8_t JOYCON::getBatteryLevel() {
        checkBatteryLevel = true; // This is needed so the library knows that the status response is a response to this function
        statusRequest(); // This will update the battery level
        return batteryLevel;
};

// Credit to mfosse https://github.com/mfosse/JoyCon-Driver/blob/master/joycon-driver/include/Joycon.hpp
void JOYCON::send_command(int command, uint8_t *data, int len) {
  unsigned char buf[0x40];
  memset(buf, 0, 0x40);
  uint8_t bluetooth = 1; //hard code for now

  if (!bluetooth) {
    buf[0x00] = 0x80;
    buf[0x01] = 0x92;
    buf[0x03] = 0x31;
  }

  buf[bluetooth ? 0x0 : 0x8] = command;
  if (data != nullptr && len != 0) {
    memcpy(buf + (bluetooth ? 0x1 : 0x9), data, len);
  }

  //hid_exchange(this->handle, buf, len + (bluetooth ? 0x1 : 0x9));
  HID_Command( buf, len + (bluetooth ? 0x1 : 0x9));

  if (data) {
    memcpy(data, buf, 0x40);
  }
}

void JOYCON::send_subcommand(int command, int subcommand, uint8_t *data, int len) {
  unsigned char buf[0x40];
  memset(buf, 0, 0x40);

  uint8_t rumble_base[9] = { (++packetNum) & 0xF, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40 };
  memcpy(buf, rumble_base, 9);

  if (packetNum > 0xF) {
    packetNum = 0x0;
  }

  // set neutral rumble base only if the command is vibrate (0x01)
  // if set when other commands are set, might cause the command to be misread and not executed
  //if (subcommand == 0x01) {
  //	uint8_t rumble_base[9] = { (++global_count) & 0xF, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40 };
  //	memcpy(buf + 10, rumble_base, 9);
  //}

  buf[9] = subcommand;
  if (data && len != 0) {
    memcpy(buf + 10, data, len);
  }

  send_command(command, buf, 10 + len);

  if (data) {
    memcpy(data, buf, 0x40); //TODO
  }
}
void JOYCON::setReportMode(bool continuous, uint8_t mode) {
#ifdef EXTRADEBUG
        Notify(PSTR("\r\nReport mode was changed to: "), 0x80);
        D_PrintHex<uint8_t > (mode, 0x80);
#endif
        // uint8_t cmd_buf[0x40];
        // bzero(cmd_buf, 0x40);
        //
        // cmd_buf[0] = 0x01;
        // cmd_buf[1] = packetNum & 0xF;
        // memcpy(cmd_buf + 2, rumbledata, 8);
        // cmd_buf[10] = 0x03;
        // cmd_buf[11] = mode;
        // HID_Command(cmd_buf, 0x40);
uint8_t cmd_buf[0x40];
bzero(cmd_buf, 0x40);
cmd_buf[0] = 0x30;
// Enable vibration
Serial.printf("\r\nEnabling vibration...");
cmd_buf[0] = 0x01; // Enabled
send_subcommand(0x1, 0x48, cmd_buf, 1);

// Enable IMU data
Serial.printf("\r\nEnabling IMU data...");
cmd_buf[0] = 0x01; // Enabled
send_subcommand(0x01, 0x40, cmd_buf, 1);


// Set input report mode (to push at 60hz)
// x00	Active polling mode for IR camera data. Answers with more than 300 bytes ID 31 packet
// x01	Active polling mode
// x02	Active polling mode for IR camera data.Special IR mode or before configuring it ?
// x21	Unknown.An input report with this ID has pairing or mcu data or serial flash data or device info
// x23	MCU update input report ?
// 30	NPad standard mode. Pushes current state @60Hz. Default in SDK if arg is not in the list
// 31	NFC mode. Pushes large packets @60Hz
Serial.printf("\r\nSet input report mode to 0x30...");
cmd_buf[0] = 0x30;
send_subcommand(0x01, 0x03, cmd_buf, 1);


        // uint8_t cmd_buf[4];
        // cmd_buf[0] = 0xA2; // HID BT DATA_request (0xA0) | Report Type (Output 0x02)
        // // cmd_buf[1] = 0x12;
        // // if(continuous)
        // //         cmd_buf[2] = 0x04 | (HIDBuffer[2] & 0x01); // Keep the rumble bit
        // // else
        // //         cmd_buf[2] = 0x00 | (HIDBuffer[2] & 0x01); // Keep the rumble bit
        // // cmd_buf[3] = mode;
        // cmd_buf[1] = 0x03;
        // cmd_buf[2] = mode;
        // HID_Command(cmd_buf, 3);
}

void JOYCON::statusRequest() {
        uint8_t cmd_buf[3];
        cmd_buf[0] = 0xA2; // HID BT DATA_request (0xA0) | Report Type (Output 0x02)
        cmd_buf[1] = 0x15;
        cmd_buf[2] = (HIDBuffer[2] & 0x01); // Keep the rumble bit
        HID_Command(cmd_buf, 3);
}

/************************************************************/
/*                    Memmory Commands                      */
/************************************************************/

void JOYCON::writeData(uint32_t offset, uint8_t size, uint8_t* data) {
        uint8_t cmd_buf[23];
        cmd_buf[0] = 0xA2; // HID BT DATA_request (0xA0) | Report Type (Output 0x02)
        cmd_buf[1] = 0x16; // Write data
        cmd_buf[2] = 0x04 | (HIDBuffer[2] & 0x01); // Write to memory, clear bit 2 to write to EEPROM
        cmd_buf[3] = (uint8_t)((offset & 0xFF0000) >> 16);
        cmd_buf[4] = (uint8_t)((offset & 0xFF00) >> 8);
        cmd_buf[5] = (uint8_t)(offset & 0xFF);
        cmd_buf[6] = size;
        uint8_t i = 0;
        for(; i < size; i++)
                cmd_buf[7 + i] = data[i];
        for(; i < 16; i++) // Set the rest to zero
                cmd_buf[7 + i] = 0x00;
        HID_Command(cmd_buf, 23);
}

void JOYCON::initExtension1() {
        uint8_t buf[1];
        buf[0] = 0x55;
        writeData(0xA400F0, 1, buf);
}

void JOYCON::initExtension2() {
        uint8_t buf[1];
        buf[0] = 0x00;
        writeData(0xA400FB, 1, buf);
}


void JOYCON::readData(uint32_t offset, uint16_t size, bool EEPROM) {
        uint8_t cmd_buf[8];
        cmd_buf[0] = 0xA2; // HID BT DATA_request (0xA0) | Report Type (Output 0x02)
        cmd_buf[1] = 0x17; // Read data
        if(EEPROM)
                cmd_buf[2] = 0x00 | (HIDBuffer[2] & 0x01); // Read from EEPROM
        else
                cmd_buf[2] = 0x04 | (HIDBuffer[2] & 0x01); // Read from memory
        cmd_buf[3] = (uint8_t)((offset & 0xFF0000) >> 16);
        cmd_buf[4] = (uint8_t)((offset & 0xFF00) >> 8);
        cmd_buf[5] = (uint8_t)(offset & 0xFF);
        cmd_buf[6] = (uint8_t)((size & 0xFF00) >> 8);
        cmd_buf[7] = (uint8_t)(size & 0xFF);

        HID_Command(cmd_buf, 8);
}

void JOYCON::readExtensionType() {
        readData(0xA400FA, 6, false);
}

void JOYCON::readCalData() {
        readData(0x0016, 8, true);
}

void JOYCON::checkMotionPresent() {
        readData(0xA600FA, 6, false);
}

/************************************************************/
/*                    Joy-Con Commands                          */
/************************************************************/

bool JOYCON::getButtonPress(ButtonEnum b) { // Return true when a button is pressed
  return (ButtonState & pgm_read_dword(&JOYCON_BUTTONS[(uint8_t)b]));
}

bool JOYCON::getButtonClick(ButtonEnum b) { // Only return true when a button is clicked
        uint32_t button;
        button = pgm_read_dword(&JOYCON_BUTTONS[(uint8_t)b]);
        bool click = (ButtonClickState & button);
        ButtonClickState &= ~button; // clear "click" event
        return click;
}

uint8_t JOYCON::getAnalogHat(HatEnum a) {
        // if(!nunchuckConnected)
        //         return 127; // Return center position
        // else {
        //         uint8_t output = hatValues[(uint8_t)a];
        //         if(output == 0xFF || output == 0x00) // The joystick will only read 255 or 0 when the cable is unplugged or initializing, so we will just return the center position
        //                 return 127;
        //         else
        //                 return output;
        // }
        return 0;
}

uint16_t JOYCON::getAnalogHat(AnalogHatEnum a) {

                uint16_t output = hatValues[(uint8_t)a];
                if(output == 0x00) // The joystick will only read 0 when it is first initializing, so we will just return the center position
                        return 2000;
                else
                        return output;

}

void JOYCON::onInit() {

        setReportMode(false, 0x30);

        if(pFuncOnInit)
                pFuncOnInit(); // Call the user function
        else
                setLedStatus();
}
