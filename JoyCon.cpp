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
#define EXTRADEBUG  // Uncomment to get even more debugging data
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

/*
Byte	Bit x01	x02	x04	x08	x10	x20	x40	x80
3 (Right)	Y	X	B	A	SR	SL	R	ZR
4 (Shared)	Minus	Plus	R Stick	L Stick	Home	Capture	--	Charging Grip
5 (Left)	Down	Up	Right	Left	SR	SL	L	ZL
*/
const uint32_t JOYCON_BUTTONS[] PROGMEM = {
    0x00020000, // UP 0
    0x00040000, // RIGHT 1
    0x00010000, // DOWN 2
    0x00080000, // LEFT 3
    0x0,        // Skip 4
    0x00000200, // PLUS 5
    0x0,        // Skip 6
    0x0,        // Skip 7
    0x00000100, // MINUS 8
    0x00001000, // HOME 9
    0x0,        // Skip 10
    0x0,        // Skip 11
    0x00000004, // B 12
    0x00000008, // A 13
    0x00000001, // Y 14
    0x00000002,  // X 15
    0x00400000, // L 16
    0x00000040, // R 17
    0x00800000, // ZL 18
    0x00000080, // ZR 19
    0x00200000, // LSL = 20
    0x00100000, // LSR = 21
    0x00000020, // RSL = 22
    0x00000010, // RSR = 23
    0x00000800, // LSTICK = 19,
    0x00000400,    // RSTICK = 20
};

JOYCON::JOYCON(BTD *p, bool pair) : BluetoothService(p) // Pointer to USB class instance - mandatory
{
        pBtd->btdName = "Nintendo Switch";
        pBtd->pairWithJoyCon = pair;

        //HIDBuffer[0] = 0xA2; // HID BT DATA_request (0xA0) | Report Type (Output 0x02)

        /* Set device cid for the control and intterrupt channelse - LSB */
        control_dcid[0] = 0x40; // 0x0040
        control_dcid[1] = 0x00;
        interrupt_dcid[0] = 0x41; // 0x0041
        interrupt_dcid[1] = 0x00;
        sdp_dcid[0] = 0x42;
        sdp_dcid[1] = 0x00;
        ButtonState = 0;

        Reset();
}

void JOYCON::Reset()
{
        joyconConnected = false;
        motionValuesReset = false;
        activeConnection = false;
        l2cap_event_flag = 0; // Reset flags
        l2cap_state = L2CAP_WAIT;
}

void JOYCON::disconnect()
{                                   // Use this void to disconnect any of the controllers
        timer = (uint32_t)millis(); // Don't wait
        // First the HID interrupt channel has to be disconnected, then the HID control channel and finally the HCI connection
        pBtd->l2cap_disconnection_request(hci_handle, ++identifier, interrupt_scid, interrupt_dcid);
        Reset();
        l2cap_state = L2CAP_INTERRUPT_DISCONNECT;
}

void JOYCON::ACLData(uint8_t *l2capinbuf)
{
#ifdef EXTRADEBUG

        //Serial.printf("\r\nACLDATA %d %d %d %d\n", pBtd->l2capConnectionClaimed, pBtd->incomingJoyCon, joyconConnected, activeConnection);
        Notify(PSTR("\r\n\tL2CAP <- : "), 0x80);
        for (uint8_t i = 0; i < l2capinbuf[2]+40; i++)
        {
                //Serial.printf(" %d:",i);
                D_PrintHex<uint8_t>(l2capinbuf[i], 0x80);
                if( i == 7 ){
                        Notify(PSTR(" : "), 0x80);
                } else {
                        Notify(PSTR(" "), 0x80);
                }
        }
#endif
        if (!pBtd->l2capConnectionClaimed && pBtd->incomingJoyCon && !joyconConnected && !activeConnection)
        {
                if (l2capinbuf[8] == L2CAP_CMD_CONNECTION_REQUEST)
                {
                        if ((l2capinbuf[12] | (l2capinbuf[13] << 8)) == HID_CTRL_PSM)
                        {
                                pBtd->incomingJoyCon = false;
                                pBtd->l2capConnectionClaimed = true; // Claim that the incoming connection belongs to this service
                                activeConnection = true;
                                hci_handle = pBtd->hci_handle; // Store the HCI Handle for the connection
                                l2cap_state = L2CAP_WAIT;
                        }
                }
        }

        if (checkHciHandle(l2capinbuf, hci_handle))
        { // acl_handle_ok
                if ((l2capinbuf[6] | (l2capinbuf[7] << 8)) == 0x0001U)
                { // l2cap_control - Channel ID for ACL-U
                        Notify(PSTR("\r\nL2CAP Command: "), 0x80);
                        D_PrintHex<uint8_t>(l2capinbuf[8], 0x80);
                        if (l2capinbuf[8] == L2CAP_CMD_COMMAND_REJECT)
                        {
#ifdef DEBUG_USB_HOST
                                Notify(PSTR("\r\nL2CAP Command Rejected - Reason: "), 0x80);
                                D_PrintHex<uint8_t>(l2capinbuf[13], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t>(l2capinbuf[12], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t>(l2capinbuf[17], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t>(l2capinbuf[16], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t>(l2capinbuf[15], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t>(l2capinbuf[14], 0x80);
#endif
                        }
                        else if (l2capinbuf[8] == L2CAP_CMD_CONNECTION_RESPONSE)
                        {
                                Notify(PSTR("\r\nL2CAP Connection Response"), 0x80);
                                if (((l2capinbuf[16] | (l2capinbuf[17] << 8)) == 0x0000) && ((l2capinbuf[18] | (l2capinbuf[19] << 8)) == SUCCESSFUL))
                                { // Success
                                        if (l2capinbuf[14] == control_dcid[0] && l2capinbuf[15] == control_dcid[1])
                                        {
                                                Notify(PSTR("\r\nHID Control Connection Complete"), 0x80);
                                                identifier = l2capinbuf[9];
                                                control_scid[0] = l2capinbuf[12];
                                                control_scid[1] = l2capinbuf[13];
                                                l2cap_set_flag(L2CAP_FLAG_CONTROL_CONNECTED);
                                        }
                                        else if (l2capinbuf[14] == interrupt_dcid[0] && l2capinbuf[15] == interrupt_dcid[1])
                                        {
                                                Notify(PSTR("\r\nHID Interrupt Connection Complete"), 0x80);
                                                identifier = l2capinbuf[9];
                                                interrupt_scid[0] = l2capinbuf[12];
                                                interrupt_scid[1] = l2capinbuf[13];
                                                l2cap_set_flag(L2CAP_FLAG_INTERRUPT_CONNECTED);
                                        }
                                        else if (l2capinbuf[14] == sdp_dcid[0] && l2capinbuf[15] == sdp_dcid[1])
                                        {
                                                Notify(PSTR("\r\nHID SDP Connection Complete"), 0x80);
                                                identifier = l2capinbuf[9];
                                                sdp_scid[0] = l2capinbuf[12];
                                                sdp_scid[1] = l2capinbuf[13];
                                                l2cap_set_flag(L2CAP_FLAG_SDP_CONNECTED);
                                        }
                                }
                        }
                        else if (l2capinbuf[8] == L2CAP_CMD_CONNECTION_REQUEST)
                        {
#ifdef EXTRADEBUG
                                Notify(PSTR("\r\nL2CAP Connection Request - PSM: "), 0x80);
                                D_PrintHex<uint8_t>(l2capinbuf[13], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t>(l2capinbuf[12], 0x80);
                                Notify(PSTR(" SCID: "), 0x80);
                                D_PrintHex<uint8_t>(l2capinbuf[15], 0x80);
                                Notify(PSTR(" "), 0x80);
                                D_PrintHex<uint8_t>(l2capinbuf[14], 0x80);
                                Notify(PSTR(" Identifier: "), 0x80);
                                D_PrintHex<uint8_t>(l2capinbuf[9], 0x80);
#endif
                                if ((l2capinbuf[12] | (l2capinbuf[13] << 8)) == HID_CTRL_PSM)
                                {
                                        Notify(PSTR("\r\nL2CAP CTRL PSM "), 0x80);
                                        identifier = l2capinbuf[9];
                                        control_scid[0] = l2capinbuf[14];
                                        control_scid[1] = l2capinbuf[15];
                                        l2cap_set_flag(L2CAP_FLAG_CONNECTION_CONTROL_REQUEST);
                                }
                                else if ((l2capinbuf[12] | (l2capinbuf[13] << 8)) == HID_INTR_PSM)
                                {
                                        Notify(PSTR("\r\nL2CAP INTR PSM "), 0x80);
                                        identifier = l2capinbuf[9];
                                        interrupt_scid[0] = l2capinbuf[14];
                                        interrupt_scid[1] = l2capinbuf[15];
                                        l2cap_set_flag(L2CAP_FLAG_CONNECTION_INTERRUPT_REQUEST);
                                }else if ((l2capinbuf[12] | (l2capinbuf[13] << 8)) == SDP_PSM)
                                {
                                        Notify(PSTR("\r\nL2CAP SDP PSM "), 0x80);
                                        identifier = l2capinbuf[9];
                                        sdp_scid[0] = l2capinbuf[14];
                                        sdp_scid[1] = l2capinbuf[15];
                                        l2cap_set_flag(L2CAP_FLAG_CONNECTION_SDP_REQUEST);
                                }
                        }
                        else if (l2capinbuf[8] == L2CAP_CMD_CONFIG_RESPONSE)
                        {
                                if ((l2capinbuf[16] | (l2capinbuf[17] << 8)) == 0x0000)
                                { // Success
                                        if (l2capinbuf[12] == control_dcid[0] && l2capinbuf[13] == control_dcid[1])
                                        {
                                                Notify(PSTR("\r\nHID Control Configuration Complete "), 0x80);
                                                D_PrintHex<uint8_t>(l2capinbuf[9], 0x80);
                                                identifier = l2capinbuf[9];
                                                l2cap_set_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS);
                                        }
                                        else if (l2capinbuf[12] == interrupt_dcid[0] && l2capinbuf[13] == interrupt_dcid[1])
                                        {
                                                Notify(PSTR("\r\nHID Interrupt Configuration Complete "), 0x80);
                                                D_PrintHex<uint8_t>(l2capinbuf[9], 0x80);
                                                identifier = l2capinbuf[9];
                                                l2cap_set_flag(L2CAP_FLAG_CONFIG_INTERRUPT_SUCCESS);
                                                //l2cap_set_flag(L2CAP_FLAG_CONFIG_SDP_SUCCESS);

                                        }
                                        else if (l2capinbuf[12] == sdp_dcid[0] && l2capinbuf[13] == sdp_dcid[1])
                                        {
                                                Notify(PSTR("\r\nHID SDP Configuration Complete "), 0x80);
                                                D_PrintHex<uint8_t>(l2capinbuf[9], 0x80);
                                                identifier = l2capinbuf[9];
                                                l2cap_set_flag(L2CAP_FLAG_CONFIG_SDP_SUCCESS);
                                        }
                                
                                }
                        }
                        else if (l2capinbuf[8] == L2CAP_CMD_CONFIG_REQUEST)
                        {
                                if (l2capinbuf[12] == control_dcid[0] && l2capinbuf[13] == control_dcid[1])
                                {
                                        Notify(PSTR("\r\nReceived HID Control Configuration Request"), 0x80);
                                        pBtd->l2cap_config_response(hci_handle, l2capinbuf[9], control_scid);
                                }
                                else if (l2capinbuf[12] == interrupt_dcid[0] && l2capinbuf[13] == interrupt_dcid[1])
                                {
                                        Notify(PSTR("\r\nReceived HID Interrupt Configuration Request"), 0x80);
                                        pBtd->l2cap_config_response(hci_handle, l2capinbuf[9], interrupt_scid);
                                }
                                else if (l2capinbuf[12] == sdp_dcid[0] && l2capinbuf[13] == sdp_dcid[1])
                                {
                                        Notify(PSTR("\r\nReceived HID SDP Configuration Request"), 0x80);
                                        pBtd->l2cap_config_response(hci_handle, l2capinbuf[9], sdp_scid);
                                }
                        }
                        else if (l2capinbuf[8] == L2CAP_CMD_DISCONNECT_REQUEST)
                        {
                                if (l2capinbuf[12] == control_dcid[0] && l2capinbuf[13] == control_dcid[1])
                                {
#ifdef DEBUG_USB_HOST
                                        Notify(PSTR("\r\nDisconnect Request: Control Channel"), 0x80);
#endif
                                        identifier = l2capinbuf[9];
                                        pBtd->l2cap_disconnection_response(hci_handle, identifier, control_dcid, control_scid);
                                        Reset();
                                }
                                else if (l2capinbuf[12] == interrupt_dcid[0] && l2capinbuf[13] == interrupt_dcid[1])
                                {
#ifdef DEBUG_USB_HOST
                                        Notify(PSTR("\r\nDisconnect Request: Interrupt Channel"), 0x80);
#endif
                                        identifier = l2capinbuf[9];
                                        pBtd->l2cap_disconnection_response(hci_handle, identifier, interrupt_dcid, interrupt_scid);
                                        Reset();
                                }
                        }
                        else if (l2capinbuf[8] == L2CAP_CMD_DISCONNECT_RESPONSE)
                        {
                                if (l2capinbuf[12] == control_scid[0] && l2capinbuf[13] == control_scid[1])
                                {
                                        Notify(PSTR("\r\nDisconnect Response: Control Channel"), 0x80);
                                        identifier = l2capinbuf[9];
                                        l2cap_set_flag(L2CAP_FLAG_DISCONNECT_CONTROL_RESPONSE);
                                }
                                else if (l2capinbuf[12] == interrupt_scid[0] && l2capinbuf[13] == interrupt_scid[1])
                                {
                                        Notify(PSTR("\r\nDisconnect Response: Interrupt Channel"), 0x80);
                                        identifier = l2capinbuf[9];
                                        l2cap_set_flag(L2CAP_FLAG_DISCONNECT_INTERRUPT_RESPONSE);
                                }
                        }
#ifdef EXTRADEBUG
                        else
                        {
                                identifier = l2capinbuf[9];
                                Notify(PSTR("\r\nL2CAP Unknown Signaling Command: "), 0x80);
                                D_PrintHex<uint8_t>(l2capinbuf[8], 0x80);
                        }
#endif
                }
                else if (l2capinbuf[6] == interrupt_dcid[0] && l2capinbuf[7] == interrupt_dcid[1])
                { // l2cap_interrupt

#ifdef EXTRADEBUG
                        Notify(PSTR("\r\nL2CAP Interrupt: "), 0x80);
                        D_PrintHex<uint32_t>(l2capinbuf[9], 0x80);
                        Notify(PSTR("\r\n"), 0x80);
#endif
                        if (l2capinbuf[8] == 0xA1 && l2capinbuf[9] == 0x21){
                                responseDebug(l2capinbuf);
                                // Standard input reports used for subcommand replies.
                                switch(l2capinbuf[23]){
                                        case 0x01:
                                                switch(l2capinbuf[24]){
                                                case 0x01:
                                                        Notify(PSTR("Pair Reply!"), 0x80);
                                                        l2cap_state = JOYCON_AQUIRE_LTK;
                                                        break;
                                                case 0x02:
                                                        Notify(PSTR("Aquire LTK Reply!"), 0x80);
                                                        l2cap_state = JOYCON_SAVE_PAIR;
                                                        break;
                                                case 0x03:
                                                        Notify(PSTR("Save Pair Reply!!"), 0x80);
                                                        l2cap_state = TURN_ON_LED;
                                                        break;
                                                }
                                                break;
                                        case 0x07:
                                                Notify(PSTR("Reset Pairing Reply!!"), 0x80);
                                                l2cap_state = JOYCON_PAIR_COMMAND;
                                                break;
                                }
                        } else if (l2capinbuf[8] == 0xA1 && l2cap_state == L2CAP_DONE)
                        { // HID_THDR_DATA_INPUT

                                switch (l2capinbuf[9])
                                {
                                case 0x30: // Standard full mode

                                        ButtonState = (uint32_t)(l2capinbuf[12] | l2capinbuf[13] << 8 | l2capinbuf[14] << 16);
#ifdef PRINTREPORT
                                        Notify(PSTR("ButtonState: "), 0x80);
                                        D_PrintHex<uint32_t>(ButtonState, 0x80);
                                        Notify(PSTR("\r\n"), 0x80);
#endif
                                        if (ButtonState != OldButtonState)
                                        {
                                                ButtonClickState = ButtonState & ~OldButtonState; // Update click state variable
                                                OldButtonState = ButtonState;
                                        }
                                        setLedStatus();
                                        break;
                                case 0x3f:
                                        setLedStatus();
                                        break;
                                case 0x31: // NFC/IR MCU mode
                                        break;
                                case 0x32: // Unknown
                                        break;
                                case 0x33: // Unknown
                                        break;
#ifdef DEBUG_USB_HOST
                                default:
                                        Notify(PSTR("\r\nUnknown Report type: "), 0x80);
                                        D_PrintHex<uint8_t>(l2capinbuf[9], 0x80);
                                        break;
#endif
                                }
                        }
                }
                L2CAP_task();
        }
}

void JOYCON::L2CAP_task()
{
        switch (l2cap_state)
        {
                /* These states are used if the Wiimote is the host */
        case L2CAP_CONTROL_SUCCESS:
                if (l2cap_check_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS))
                {
#ifdef DEBUG_USB_HOST
                        Notify(PSTR("\r\nHID Control Successfully Configured"), 0x80);
#endif
                        l2cap_state = L2CAP_INTERRUPT_SETUP;
                }
                break;

        case L2CAP_INTERRUPT_SETUP:
                if (l2cap_check_flag(L2CAP_FLAG_CONNECTION_INTERRUPT_REQUEST))
                {
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
                        /* SKIP SDP See also above for FLAG!!!!*/
                        //l2cap_state = L2CAP_SDP_CONFIG_REQUEST;
                }
                break;

                /* These states are used if the Arduino is the host */
        case L2CAP_CONTROL_CONNECT_REQUEST:
                if (l2cap_check_flag(L2CAP_FLAG_CONTROL_CONNECTED))
                {
#ifdef DEBUG_USB_HOST
                        Notify(PSTR("\r\nSend HID Control Config Request"), 0x80);
#endif
                        identifier++;
                        pBtd->l2cap_config_request(hci_handle, identifier, control_scid);
                        l2cap_state = L2CAP_CONTROL_CONFIG_REQUEST;
                }
                break;

        case L2CAP_CONTROL_CONFIG_REQUEST:
                if (l2cap_check_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS))
                {
#ifdef DEBUG_USB_HOST
                        Notify(PSTR("\r\nSend HID Interrupt Connection Request"), 0x80);
#endif
                        identifier++;
                        pBtd->l2cap_connection_request(hci_handle, identifier, interrupt_dcid, HID_INTR_PSM);
                        l2cap_state = L2CAP_INTERRUPT_CONNECT_REQUEST;
                }
                break;

        case L2CAP_INTERRUPT_CONNECT_REQUEST:
                if (l2cap_check_flag(L2CAP_FLAG_INTERRUPT_CONNECTED))
                {
#ifdef DEBUG_USB_HOST
                        Notify(PSTR("\r\nSend HID Interrupt Config Request"), 0x80);
#endif
                        identifier++;
                        pBtd->l2cap_config_request(hci_handle, identifier, interrupt_scid);
                        l2cap_state = L2CAP_INTERRUPT_CONFIG_REQUEST;

                        /* SKIP SDP See also above for FLAG!!!!*/
                        //l2cap_state = L2CAP_SDP_CONFIG_REQUEST;
                }
                break;

        case L2CAP_INTERRUPT_CONFIG_REQUEST:
                if (l2cap_check_flag(L2CAP_FLAG_CONFIG_INTERRUPT_SUCCESS))
                { 
#ifdef DEBUG_USB_HOST
                        Notify(PSTR("\r\nSend HID SDP Connection Request"), 0x80);
#endif
                        identifier++;
                        pBtd->l2cap_connection_request(hci_handle, identifier, sdp_dcid, SDP_PSM);
                        l2cap_state = L2CAP_SDP_CONNECT_REQUEST;
                }
                break;

        case L2CAP_SDP_CONNECT_REQUEST:
                if (l2cap_check_flag(L2CAP_FLAG_SDP_CONNECTED))
                { 
#ifdef DEBUG_USB_HOST
                        Notify(PSTR("\r\nSend HID SDP Config Request"), 0x80);
#endif
                        identifier++;
                        pBtd->l2cap_config_request(hci_handle, identifier, sdp_scid);

                        l2cap_state = L2CAP_SDP_CONFIG_REQUEST;
                }
                break;
        case L2CAP_SDP_CONFIG_REQUEST:
                if (l2cap_check_flag(L2CAP_FLAG_CONFIG_SDP_SUCCESS))
                { 

#ifdef DEBUG_USB_HOST
                        Notify(PSTR("\r\nHID Channels Established"), 0x80);
#endif
                        if(pBtd->pairWithJoyCon){
                                //l2cap_state = JOYCON_RESET_PAIRING;
                                l2cap_state = JOYCON_PAIR_COMMAND;
                        } else {
                                l2cap_state = TURN_ON_LED;
                        }
                        
                        
                }
                break;
                /* The next states are in run() */
        
        case L2CAP_INTERRUPT_DISCONNECT:
                if (l2cap_check_flag(L2CAP_FLAG_DISCONNECT_INTERRUPT_RESPONSE) && ((int32_t)((uint32_t)millis() - timer) >= 0L))
                {
#ifdef DEBUG_USB_HOST
                        Notify(PSTR("\r\nDisconnected Interrupt Channel"), 0x80);
#endif
                        identifier++;
                        pBtd->l2cap_disconnection_request(hci_handle, identifier, control_scid, control_dcid);
                        l2cap_state = L2CAP_CONTROL_DISCONNECT;
                }
                break;

        case L2CAP_CONTROL_DISCONNECT:
                if (l2cap_check_flag(L2CAP_FLAG_DISCONNECT_CONTROL_RESPONSE))
                {
#ifdef DEBUG_USB_HOST
                        Notify(PSTR("\r\nDisconnected Control Channel"), 0x80);
#endif
                        pBtd->hci_disconnect(hci_handle);
                        hci_handle = -1;      // Reset handle
                        l2cap_event_flag = 0; // Reset flags
                        l2cap_state = L2CAP_WAIT;
                }
                break;
        }
}

void JOYCON::Run()
{
        if (l2cap_state == L2CAP_INTERRUPT_DISCONNECT && ((int32_t)((uint32_t)millis() - timer) >= 0L))
                L2CAP_task(); // Call the rest of the disconnection routine after we have waited long enough

        switch (l2cap_state)
        {
        case L2CAP_WAIT:
                if (pBtd->connectToJoyCon && !pBtd->l2capConnectionClaimed && !joyconConnected && !activeConnection)
                {
                        pBtd->l2capConnectionClaimed = true;
                        activeConnection = true;
#ifdef DEBUG_USB_HOST
                        Notify(PSTR("\r\nSend HID Control Connection Request"), 0x80);
#endif
                        hci_handle = pBtd->hci_handle; // Store the HCI Handle for the connection
                        l2cap_event_flag = 0;          // Reset flags
                        identifier = 1;
                        pBtd->l2cap_connection_request(hci_handle, identifier, control_dcid, HID_CTRL_PSM);
                        l2cap_state = L2CAP_CONTROL_CONNECT_REQUEST;
                }
                else if (l2cap_check_flag(L2CAP_FLAG_CONNECTION_CONTROL_REQUEST))
                {
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
        case JOYCON_RESET_PAIRING:
        {
                uint8_t cmd_buf[0x40];
                bzero(cmd_buf, 0x40);
                Notify(PSTR("\r\nReset Pairing Information"), 0x80);
                cmd_buf[0] = 0x00;
                send_subcommand(0x01, 0x07, cmd_buf, 0);
                l2cap_state = JOYCON_PAIR_RESPONSE;        
                break;
        }
        case JOYCON_PAIR_COMMAND:
        {
                uint8_t cmd_buf[0x40];
                bzero(cmd_buf, 0x40);
                Notify(PSTR("\r\nManual Pairing Subcommand "), 0x80);
                cmd_buf[0] = 0x01;
                cmd_buf[1] = pBtd->my_bdaddr[0]; // 6 octet bdaddr (LSB)
                cmd_buf[2] = pBtd->my_bdaddr[1];
                cmd_buf[3] = pBtd->my_bdaddr[2];
                cmd_buf[4] = pBtd->my_bdaddr[3];
                cmd_buf[5] = pBtd->my_bdaddr[4];
                cmd_buf[6] = pBtd->my_bdaddr[5];

                cmd_buf[1] = pBtd->my_bdaddr[5]; // 6 octet bdaddr (LSB)
                cmd_buf[2] = pBtd->my_bdaddr[4];
                cmd_buf[3] = pBtd->my_bdaddr[3];
                cmd_buf[4] = pBtd->my_bdaddr[2];
                cmd_buf[5] = pBtd->my_bdaddr[1];
                cmd_buf[6] = pBtd->my_bdaddr[0];
                send_subcommand(0x01, 0x01, cmd_buf, 7);
                l2cap_state = JOYCON_PAIR_RESPONSE;        
                break;
        }
        case JOYCON_AQUIRE_LTK:
        {
                uint8_t cmd_buf[0x40];
                bzero(cmd_buf, 0x40);
                Notify(PSTR("\r\nManual Pairing Subcommand Aquire LTK"), 0x80);
                cmd_buf[0] = 0x02;
                send_subcommand(0x01, 0x01, cmd_buf, 1);
                l2cap_state = JOYCON_PAIR_RESPONSE;        
                break;
        }
        case JOYCON_SAVE_PAIR:
        {
                uint8_t cmd_buf[0x40];
                bzero(cmd_buf, 0x40);
                Notify(PSTR("\r\nManual Pairing Subcommand Save Pairing"), 0x80);
                cmd_buf[0] = 0x03;
                send_subcommand(0x01, 0x01, cmd_buf, 1);
                l2cap_state = JOYCON_PAIR_RESPONSE;        
                break;
        }
        case JOYCON_RECONNECT:
                pBtd->pairWithJoyCon = false;
                uint8_t cmd_buf[0x40];
                bzero(cmd_buf, 0x40);
                Notify(PSTR("\r\nReboot and ReConnect"), 0x80);
                cmd_buf[0] = 0x03;
                send_subcommand(0x01, 0x06, cmd_buf, 1);
                l2cap_state = L2CAP_WAIT;        
                break;
                

        case TURN_ON_LED:
                pBtd->connectToJoyCon = false;
                pBtd->pairWithJoyCon = false;
                joyconConnected = true;
                l2cap_state = L2CAP_DONE;

                onInit();
                break;

        case L2CAP_DONE:
                stateCounter = 0;
                break;
        }
}

/************************************************************/
/*                    HID Commands                          */
/************************************************************/

void JOYCON::HID_Command(uint8_t *data, uint8_t nbytes)
{
        // if(motionPlusInside)
        //         pBtd->L2CAP_Command(hci_handle, data, nbytes, interrupt_scid[0], interrupt_scid[1]); // It's the new Wiimote with the Motion Plus Inside or Wii U Pro controller
        // else
        //         pBtd->L2CAP_Command(hci_handle, data, nbytes, control_scid[0], control_scid[1]);

        //pBtd->L2CAP_Command(hci_handle, data, nbytes, control_scid[0], control_scid[1]);
        pBtd->L2CAP_Command(hci_handle, data, nbytes, interrupt_scid[0], interrupt_scid[1]);
        //pBtd->L2CAP_Command(hci_handle, data, nbytes, sdp_scid[0], sdp_scid[1]);

        Notify(PSTR("\r\nHID Command: "), 0x80);
        for (uint8_t i = 0; i < nbytes; i++)
        {
                D_PrintHex<uint8_t>(data[i], 0x80);
                Notify(PSTR(" "), 0x80);
        }

        //packetNum++;
}

void JOYCON::setAllOff()
{
        // HIDBuffer[1] = 0x11;
        // HIDBuffer[2] = 0x00;
        // HID_Command(HIDBuffer, 3);
}

void JOYCON::setRumbleOff()
{
        setRumble(0x00);
}

void JOYCON::setRumbleOn()
{
        setRumble(0x01);
}
void JOYCON::setRumble(uint8_t mode)
{
#ifdef EXTRADEBUG
        Notify(PSTR("\r\nSetting rumble: "), 0x80);
        D_PrintHex<uint8_t>(mode, 0x80);

#endif

        uint8_t cmd_buf[0x40];
        bzero(cmd_buf, 0x40);

        // Enable vibration
        cmd_buf[0] = mode;
        send_subcommand(0x1, 0x48, cmd_buf, 1);
}

void JOYCON::setLedRaw(uint8_t value)
{
        // HIDBuffer[1] = 0x11;
        // HIDBuffer[2] = value | (HIDBuffer[2] & 0x01); // Keep the rumble bit
        // HID_Command(HIDBuffer, 3);
}

void JOYCON::setLedOff(LEDEnum a)
{
        // HIDBuffer[1] = 0x11;
        // HIDBuffer[2] &= ~(pgm_read_byte(&JOYCON_LEDS[(uint8_t)a]));
        // HID_Command(HIDBuffer, 3);
}

void JOYCON::setLedOn(LEDEnum a)
{
        // if (a == OFF)
        //         setLedRaw(0);
        // else
        // {
        //         HIDBuffer[1] = 0x11;
        //         HIDBuffer[2] |= pgm_read_byte(&JOYCON_LEDS[(uint8_t)a]);
        //         HID_Command(HIDBuffer, 3);
        // }
}

void JOYCON::setLedToggle(LEDEnum a)
{
        // HIDBuffer[1] = 0x11;
        // HIDBuffer[2] ^= pgm_read_byte(&JOYCON_LEDS[(uint8_t)a]);
        // HID_Command(HIDBuffer, 3);
}

void JOYCON::setLedStatus()
{
#ifdef EXTRADEBUG
        Notify(PSTR("\r\nSetting LED Status"), 0x80);
#endif
        uint8_t cmd_buf[0x40];
        bzero(cmd_buf, 0x40);
        cmd_buf[0] = 0x03;
        send_subcommand(0x01, 0x30, cmd_buf, 1);

}

uint8_t JOYCON::getBatteryLevel()
{
        checkBatteryLevel = true; // This is needed so the library knows that the status response is a response to this function
        statusRequest();          // This will update the battery level
        return batteryLevel;
};

// Credit to mfosse https://github.com/mfosse/JoyCon-Driver/blob/master/joycon-driver/include/Joycon.hpp
void JOYCON::send_command(int command, uint8_t *data, int len)
{
        unsigned char buf[0x40];
        memset(buf, 0, 0x40);
 
        // buf[0] = command;
        // if (data != nullptr && len != 0)
        // {
        //         memcpy(buf + 0x01, data, len);
        // }

        buf[0] = 0xA2;
        buf[1] = command;
        if (data != nullptr && len != 0)
        {
                memcpy(buf + 2, data, len);
        }


        //hid_exchange(this->handle, buf, len + (bluetooth ? 0x1 : 0x9));
        HID_Command(buf, len + 2);

        if (data)
        {
                memcpy(data, buf, 0x40);
        }
}

void JOYCON::send_subcommand(int command, int subcommand, uint8_t *data, int len)
{
        unsigned char buf[0x40];
        memset(buf, 0, 0x40);

        uint8_t rumble_base[9] = {(uint8_t)((++packetNum) & 0xF), 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40};
        memcpy(buf, rumble_base, 9);

        if (packetNum > 0xF)
        {
                packetNum = 0x0;
        }

        // set neutral rumble base only if the command is vibrate (0x01)
        // if set when other commands are set, might cause the command to be misread and not executed
        //if (subcommand == 0x01) {
        //	uint8_t rumble_base[9] = { (++global_count) & 0xF, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40 };
        //	memcpy(buf + 10, rumble_base, 9);
        //}

        buf[9] = subcommand;
        if (data && len != 0)
        {
                memcpy(buf + 10, data, len);
        }

        send_command(command, buf, 10 + len);

        if (data)
        {
                memcpy(data, buf, 0x40); //TODO
        }
}
void JOYCON::setReportMode( uint8_t mode )
{
#ifdef EXTRADEBUG
        Notify(PSTR("\r\nReport mode was changed to: "), 0x80);
        D_PrintHex<uint8_t>(mode, 0x80);
#endif
        uint8_t cmd_buf[0x40];
        bzero(cmd_buf, 0x40);

        // Enable IMU data
        // Serial.printf("\r\nEnabling IMU data...");
        // cmd_buf[0] = 0x01; // Enabled
        // send_subcommand(0x01, 0x40, cmd_buf, 1);

        cmd_buf[0] = mode;
        send_subcommand(0x01, 0x03, cmd_buf, 1);
}

void JOYCON::statusRequest()
{
        // uint8_t cmd_buf[3];
        // cmd_buf[0] = 0xA2; // HID BT DATA_request (0xA0) | Report Type (Output 0x02)
        // cmd_buf[1] = 0x15;
        // cmd_buf[2] = (HIDBuffer[2] & 0x01); // Keep the rumble bit
        // HID_Command(cmd_buf, 3);
}

/************************************************************/
/*                    Memmory Commands                      */
/************************************************************/

void JOYCON::writeData(uint32_t offset, uint8_t size, uint8_t *data)
{
        // uint8_t cmd_buf[23];
        // cmd_buf[0] = 0xA2;                         // HID BT DATA_request (0xA0) | Report Type (Output 0x02)
        // cmd_buf[1] = 0x16;                         // Write data
        // cmd_buf[2] = 0x04 | (HIDBuffer[2] & 0x01); // Write to memory, clear bit 2 to write to EEPROM
        // cmd_buf[3] = (uint8_t)((offset & 0xFF0000) >> 16);
        // cmd_buf[4] = (uint8_t)((offset & 0xFF00) >> 8);
        // cmd_buf[5] = (uint8_t)(offset & 0xFF);
        // cmd_buf[6] = size;
        // uint8_t i = 0;
        // for (; i < size; i++)
        //         cmd_buf[7 + i] = data[i];
        // for (; i < 16; i++) // Set the rest to zero
        //         cmd_buf[7 + i] = 0x00;
        // HID_Command(cmd_buf, 23);
}

void JOYCON::initExtension1()
{
        uint8_t buf[1];
        buf[0] = 0x55;
        writeData(0xA400F0, 1, buf);
}

void JOYCON::initExtension2()
{
        uint8_t buf[1];
        buf[0] = 0x00;
        writeData(0xA400FB, 1, buf);
}

void JOYCON::readData(uint32_t offset, uint8_t size)
{
        // uint8_t cmd_buf[8];
        // cmd_buf[0] = 0xA2; // HID BT DATA_request (0xA0) | Report Type (Output 0x02)
        // cmd_buf[1] = 0x17; // Read data
        // if (EEPROM)
        //         cmd_buf[2] = 0x00 | (HIDBuffer[2] & 0x01); // Read from EEPROM
        // else
        //         cmd_buf[2] = 0x04 | (HIDBuffer[2] & 0x01); // Read from memory
        // cmd_buf[3] = (uint8_t)((offset & 0xFF0000) >> 16);
        // cmd_buf[4] = (uint8_t)((offset & 0xFF00) >> 8);
        // cmd_buf[5] = (uint8_t)(offset & 0xFF);
        // cmd_buf[6] = (uint8_t)((size & 0xFF00) >> 8);
        // cmd_buf[7] = (uint8_t)(size & 0xFF);

        // HID_Command(cmd_buf, 8);

#ifdef EXTRADEBUG
        Notify(PSTR("\r\nRead data: "), 0x80);
        D_PrintHex<uint32_t>(offset, 0x80);
#endif
        uint8_t cmd_buf[0x40];
        bzero(cmd_buf, 0x40);
        cmd_buf[0] = (uint8_t)((offset & 0xFF000000) >> 24);
        cmd_buf[1] = (uint8_t)((offset & 0xFF0000) >> 16);
        cmd_buf[2] = (uint8_t)((offset & 0xFF00) >> 8);
        cmd_buf[3] = (uint8_t)(offset & 0xFF);
        cmd_buf[4] = size;
        

        send_subcommand(0x01, 0x10, cmd_buf, 5);
}

void JOYCON::readCalData()
{
        //readData(0x0016, 8, true);
}

void JOYCON::checkMotionPresent()
{
        //readData(0xA600FA, 6, false);
}

/************************************************************/
/*                    Joy-Con Commands                          */
/************************************************************/

bool JOYCON::getButtonPress(ButtonEnum b)
{ // Return true when a button is pressed
        return (ButtonState & pgm_read_dword(&JOYCON_BUTTONS[(uint8_t)b]));
}

bool JOYCON::getButtonClick(ButtonEnum b)
{ // Only return true when a button is clicked
        uint32_t button;
        button = pgm_read_dword(&JOYCON_BUTTONS[(uint8_t)b]);
        bool click = (ButtonClickState & button);
        ButtonClickState &= ~button; // clear "click" event
        return click;
}

uint8_t JOYCON::getAnalogHat(HatEnum a)
{
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

uint16_t JOYCON::getAnalogHat(AnalogHatEnum a)
{

        uint16_t output = hatValues[(uint8_t)a];
        if (output == 0x00) // The joystick will only read 0 when it is first initializing, so we will just return the center position
                return 2000;
        else
                return output;
}

void JOYCON::onInit()
{
        delay(1);
        
        //00:1A:7D:DA:71:11

        setReportMode(0x3F);
        delay(10);
        readData(0x00200000,0x1D);
        //readData(0x26200000,0x1D);
        Notify(PSTR("\n\rDEBUG A "), 0x80);
        if (pFuncOnInit){
                Notify(PSTR("\n\rDEBUG B "), 0x80);
                pFuncOnInit(); // Call the user function
        } else {
                setLedStatus();
        }
        Notify(PSTR("\n\rDEBUG B "), 0x80);

}

void JOYCON::responseDebug( uint8_t buf[350] ){

        Notify(PSTR("BT Handle: "), 0x80);
        D_PrintHex<uint16_t>(buf[0], 0x80);
        Notify(PSTR("\r\n"), 0x80);

        Notify(PSTR("BT Data Length: "), 0x80);
        D_PrintHex<uint16_t>(buf[2], 0x80);
        Notify(PSTR("\r\n"), 0x80);

        Notify(PSTR("L2CAP Data Length: "), 0x80);
        D_PrintHex<uint16_t>(buf[4], 0x80);
        Notify(PSTR("\r\n"), 0x80);

        Notify(PSTR("L2CAP CID: "), 0x80);
        D_PrintHex<uint16_t>(buf[6], 0x80);
        Notify(PSTR("\r\n"), 0x80);

        Notify(PSTR("JoyCon Report Type: "), 0x80);
        D_PrintHex<uint8_t>(buf[9], 0x80);
        Notify(PSTR(" Timer: "), 0x80);
        D_PrintHex<uint8_t>(buf[10], 0x80);
        Notify(PSTR(" Battery: "), 0x80);
        D_PrintHex<uint8_t>(buf[11], 0x80);
        Notify(PSTR("\r\n"), 0x80);

        Notify(PSTR("Buttons: Right: "), 0x80);
        D_PrintHex<uint8_t>(buf[12], 0x80);
        Notify(PSTR(" Shared: "), 0x80);
        D_PrintHex<uint8_t>(buf[13], 0x80);
        Notify(PSTR(" Left: "), 0x80);
        D_PrintHex<uint8_t>(buf[14], 0x80);
        Notify(PSTR("\r\n"), 0x80);
        
        Notify(PSTR("Sticks: Left: "), 0x80);
        D_PrintHex<uint8_t>(buf[15], 0x80);
        Notify(PSTR(", "), 0x80);
        D_PrintHex<uint8_t>(buf[16], 0x80);
        Notify(PSTR(", "), 0x80);
        D_PrintHex<uint8_t>(buf[17], 0x80);
        Notify(PSTR(" Right: "), 0x80);
        D_PrintHex<uint8_t>(buf[18], 0x80);
        Notify(PSTR(", "), 0x80);
        D_PrintHex<uint8_t>(buf[19], 0x80);
        Notify(PSTR(", "), 0x80);
        D_PrintHex<uint8_t>(buf[20], 0x80);
        Notify(PSTR("\r\n"), 0x80);
        
        Notify(PSTR("Rumble: "), 0x80);
        D_PrintHex<uint8_t>(buf[21], 0x80);
        Notify(PSTR("\r\n"), 0x80);

        Notify(PSTR("SubCommand: ACK: "), 0x80);
        D_PrintHex<uint8_t>(buf[22] && 0x80, 0x80);
        Notify(PSTR(" TYPE: "), 0x80);
        D_PrintHex<uint8_t>(buf[22] & 0x7F, 0x80);
        Notify(PSTR(" ID: "), 0x80);
        D_PrintHex<uint8_t>(buf[23], 0x80);
        Notify(PSTR("\r\n"), 0x80);


}
