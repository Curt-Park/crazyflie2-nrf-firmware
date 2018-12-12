/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie 2.0 NRF Firmware
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */
#include <nrf.h>

#ifdef BLE
#include <nrf_soc.h>
#endif

#include <stdio.h>
#include <string.h>

#include "uart.h"
#include "esb.h"
#include "syslink.h"
#include "led.h"
#include "button.h"
#include "pm.h"
#include "pinout.h"
#include "systick.h"
#include "uart.h"

#include "memory.h"
#include "ownet.h"

#ifdef BLE
#include "ble_crazyflies.h"
#endif

extern void  initialise_monitor_handles(void);
extern int ble_init(void);

#ifndef SEMIHOSTING
#define printf(...)
#endif

#ifndef DEFAULT_RADIO_RATE
  #define DEFAULT_RADIO_RATE  esbDatarate2M
#endif
#ifndef DEFAULT_RADIO_CHANNEL
  #define DEFAULT_RADIO_CHANNEL 40
#endif

static void mainloop(void);

#if BLE==0
#undef BLE
#endif

#define ISPTX
static bool inBootloaderMode=false;
//uint8_t beacon_rssi =44;

static bool boottedFromBootloader;

static void handleRadioCmd(struct esbPacket_s * packet);
static void handleBootloaderCmd(struct esbPacket_s *packet);

int main()
{
  systickInit();
  memoryInit();

#ifdef BLE
  ble_init();
#else
  NRF_CLOCK->TASKS_HFCLKSTART = 1UL;
  while(!NRF_CLOCK->EVENTS_HFCLKSTARTED);
#endif

#ifdef SEMIHOSTING
  initialise_monitor_handles();
#endif

  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSTAT_SRC_Synth;

  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;
  while(!NRF_CLOCK->EVENTS_LFCLKSTARTED);

  LED_INIT();
  if ((NRF_POWER->GPREGRET & 0x80) && ((NRF_POWER->GPREGRET&(0x3<<1))==0)) {
    buttonInit(buttonShortPress);
  } else {
    buttonInit(buttonIdle);
  }

  if  (NRF_POWER->GPREGRET & 0x20) {
    boottedFromBootloader = true;
    NRF_POWER->GPREGRET &= ~0x20;
  }

  pmInit();

  if ((NRF_POWER->GPREGRET&0x01) == 0) {
		  pmSetState(pmSysRunning);
  }

  LED_ON();


  NRF_GPIO->PIN_CNF[RADIO_PAEN_PIN] |= GPIO_PIN_CNF_DIR_Output | (GPIO_PIN_CNF_DRIVE_S0H1<<GPIO_PIN_CNF_DRIVE_Pos);

#ifndef BLE
  esbInit();

  esbSetDatarate(DEFAULT_RADIO_RATE);
  esbSetChannel(DEFAULT_RADIO_CHANNEL);
#endif	esbSetChannel(drone_id*10);

  mainloop();

  // The main loop should never end
  // TODO see if we should shut-off the system there?
  while(1);

  return 0;
}

static int32_t find_minimum(uint8_t a[], int32_t n) {
	int32_t c, min, index;

	min = a[0];
	index = 0;

	for (c = 1; c < n; c++) {
		if (a[c] < min) {
			index = c;
			min = a[c];
		}
	}

	return index;
}

static uint8_t rssi = 160;

void mainloop()
{
  static struct syslinkPacket slRxPacket;
  static struct syslinkPacket slTxPacket;
  static EsbPacket esbRxPacket;
  bool esbReceived = false;
  bool slReceived;
  static int vbatSendTime;
	static int radioRSSISendTime;

	bool in_ptx = false;

  static bool broadcast;



  //For PTX mode
	static uint8_t inter_rssi=150;
	static int radioPTXSendTime;
    static int radioPTXtoPRXSendTime;
    static int radioInterRSSISendTime;
    static bool send_inter_rssi_to_stm = false;
    static bool interdrone = true;
    static uint8_t inter_beacon_rssi = 140;

    static uint8_t rssi_array_other_drones[9] = {150,150,150,150,150,150,150,150,150};
    static unsigned int time_array_other_drones[9] = {0};
    static unsigned int channels_other_drones[8] = {20,30 ,40,50,60,70,80,90};
    int number_of_channels = sizeof(channels_other_drones) / sizeof(unsigned int);
    static count_switch_channel = 0;
    static uint8_t state_gbug = 0;
    static float rssi_angle_gbug = 0;

    static float rssi_angle_other_drone = 0;
    static float rssi_angle_array_other_drones[9] = {500.0f};


  while(1)
  {

#ifdef BLE
    if ((esbReceived == false) && bleCrazyfliesIsPacketReceived()) {
      EsbPacket* packet = bleCrazyfliesGetRxPacket();
      memcpy(esbRxPacket.data, packet->data, packet->size);
      esbRxPacket.size = packet->size;
      esbReceived = true;
      bleCrazyfliesReleaseRxPacket(packet);
    }

#endif
#ifndef CONT_WAVE_TEST

/* If esb packet is received (from computer
 * 	  AND esb packet is a received packet
 * 	  AND radio not in PTX mode
 */


	/*if(esbIsRxPacket())
	{
	      LED_ON();
	}else
		LED_OFF();*/

    if (esbReceived == false && esbIsRxPacket() && in_ptx==false )
    {
      EsbPacket* packet = esbGetRxPacket();
      //Store RSSI here so that we can send it to STM later
      // The received packet was a broadcast, if received on local address 1
      broadcast = packet->match == ESB_MULTICAST_ADDRESS_MATCH;
      // The received packet was a interdrone transmission, if received on local address 2
      interdrone = packet->match == ESB_INTERDRONE_ADDRESS_MATCH;

      // Turn off LED if interdrone address has been encountered (for debugging)
      if(interdrone==true)
       {
     	  inter_rssi = packet->data[2];
     	  int drone_id = packet->data[4];
     	  inter_beacon_rssi = packet->data[3];

          memcpy(&rssi_angle_other_drone, &packet->data[5], sizeof(float));

          // exception for drones that don't have a defined ID
     	  if(drone_id==231)
     	  {
     		 // rssi = packet->rssi;
          	 rssi_array_other_drones[0]=inter_rssi;
          	 time_array_other_drones[0] = systickGetTick();
          	rssi_angle_array_other_drones[0] = rssi_angle_other_drone;
     	  }else{
     	 rssi_array_other_drones[drone_id]=inter_rssi;
      	 time_array_other_drones[drone_id] = systickGetTick();
       	rssi_angle_array_other_drones[drone_id] = rssi_angle_other_drone;

     	  }
     	  LED_OFF();
       }else {
           rssi = packet->rssi;
    	   LED_ON();
       }


      memcpy(esbRxPacket.data, packet->data, packet->size);
      esbRxPacket.size = packet->size;
      esbReceived = true;
      esbReleaseRxPacket(packet);
    }


    if (esbReceived)
    {
      EsbPacket* packet = &esbRxPacket;
      esbReceived = false;
      if((packet->size >= 4) && (packet->data[0]&0xf3) == 0xf3 && (packet->data[1]==0x03))
      {
          // Change radio channel, addres and/or power as commanded from groundstation

        handleRadioCmd(packet);

      }
      else if ((packet->size >2) && (packet->data[0]&0xf3) == 0xf3 && (packet->data[1]==0xfe))
      {
          // Command for the nrf bootloader (like reseting after flashing)

        handleBootloaderCmd(packet);

      }
      else  // Handle the esb package a normal data package to be send to stm
      {
    	 // Copy all the data of esb packet to systemlink(sl) packet
        memcpy(slTxPacket.data, packet->data, packet->size);
        slTxPacket.length = packet->size;

        if (broadcast) { // If message was broadcast (ground station to multiple drones)
          slTxPacket.type = SYSLINK_RADIO_RAW_BROADCAST;
        } /*else if (interdrone){ // If message is from another drone on the same channel
            slTxPacket.type = SYSLINK_RADIO_RAW_INTER;
        }*/else{
        	// If message is normal radio raw (one groundstation to one drone)
          slTxPacket.type = SYSLINK_RADIO_RAW;
        }
        // if not interdrone message, send to stm
        // TODO enable interdrone communication to STM!
        if(!interdrone){
        syslinkSend(&slTxPacket);
        }
      }
    }

    /*Detect if message received from STM*/
    slReceived = syslinkReceive(&slRxPacket);
    if (slReceived && in_ptx == false)
    {
      switch (slRxPacket.type)
      {
      // To send ACK message back to ground station
        case SYSLINK_RADIO_RAW:
          if (esbCanTxPacket() && (slRxPacket.length < SYSLINK_MTU))
          {

            EsbPacket* packet = esbGetTxPacket();

            if (packet) {
              memcpy(packet->data, slRxPacket.data, slRxPacket.length);
              packet->size = slRxPacket.length;
              esbSendTxPacket(packet);
            }
            bzero(slRxPacket.data, SYSLINK_MTU);
          }
#ifdef BLE
          if (slRxPacket.length < SYSLINK_MTU) {
            static EsbPacket pk;
            memcpy(pk.data,  slRxPacket.data, slRxPacket.length);
            pk.size = slRxPacket.length;
            bleCrazyfliesSendPacket(&pk);
          }
#endif

          break;
        case SYSLINK_RADIO_CHANNEL:
          if(slRxPacket.length == 1)
          {
            esbSetChannel(slRxPacket.data[0]);

            slTxPacket.type = SYSLINK_RADIO_CHANNEL;
            slTxPacket.data[0] = slRxPacket.data[0];
            slTxPacket.length = 1;
            syslinkSend(&slTxPacket);
          }
          break;
        case SYSLINK_RADIO_DATARATE:
          if(slRxPacket.length == 1)
          {
            esbSetDatarate(slRxPacket.data[0]);

            slTxPacket.type = SYSLINK_RADIO_DATARATE;
            slTxPacket.data[0] = slRxPacket.data[0];
            slTxPacket.length = 1;
            syslinkSend(&slTxPacket);
          }
          break;
        case SYSLINK_RADIO_CONTWAVE:
          if(slRxPacket.length == 1) {
            esbSetContwave(slRxPacket.data[0]);

            slTxPacket.type = SYSLINK_RADIO_CONTWAVE;
            slTxPacket.data[0] = slRxPacket.data[0];
            slTxPacket.length = 1;
            syslinkSend(&slTxPacket);
          }
          break;
        case SYSLINK_RADIO_ADDRESS:
          if(slRxPacket.length == 5)
          {
            uint64_t address = 0;
            memcpy(&address, &slRxPacket.data[0], 5);
            esbSetAddress(address);

            slTxPacket.type = SYSLINK_RADIO_ADDRESS;
            memcpy(slTxPacket.data, slRxPacket.data, 5);
            slTxPacket.length = 5;
            syslinkSend(&slTxPacket);
          }
          break;
        case SYSLINK_RADIO_POWER:
          if(slRxPacket.length == 1)
          {
            esbSetTxPowerDbm((int8_t)slRxPacket.data[0]);

            slTxPacket.type = SYSLINK_RADIO_POWER;
            slTxPacket.data[0] = slRxPacket.data[0];
            slTxPacket.length = 1;
            syslinkSend(&slTxPacket);
          }
          break;
        case SYSLINK_PM_ONOFF_SWITCHOFF:
          pmSetState(pmAllOff);
          break;
        case SYSLINK_OW_GETINFO:
        case SYSLINK_OW_READ:
        case SYSLINK_OW_SCAN:
        case SYSLINK_OW_WRITE:
          if (memorySyslink(&slRxPacket)) {
            syslinkSend(&slRxPacket);
          }
	  break;
        case SYSLINK_GRADIENT_BUG:
          if(slRxPacket.length > 0){
            state_gbug = slRxPacket.data[0];
            memcpy(&rssi_angle_gbug, slRxPacket.data +1, sizeof(float));
          }
          break;
      }
    }

    // Wait a while to start pushing over the syslink since UART pins are used to launch STM32 i bootloader as well
    if (systickGetTick() > SYSLINK_STARTUP_DELAY_TIME_MS) {
      // Send the battery voltage and state to the STM every SYSLINK_SEND_PERIOD_MS
      if (systickGetTick() >= vbatSendTime + SYSLINK_SEND_PERIOD_MS) {
        float fdata;
        uint8_t flags = 0;

        vbatSendTime = systickGetTick();
        slTxPacket.type = SYSLINK_PM_BATTERY_STATE;
        slTxPacket.length = 9;

        flags |= (pmIsCharging() == true)?0x01:0;
        flags |= (pmUSBPower() == true)?0x02:0;

        slTxPacket.data[0] = flags;

        fdata = pmGetVBAT();
        memcpy(slTxPacket.data+1, &fdata, sizeof(float));

        fdata = pmGetISET();
        memcpy(slTxPacket.data+1+4, &fdata, sizeof(float));

        syslinkSend(&slTxPacket);
      }
      //Send an RSSI sample to the STM every 10ms(100Hz)
      if (systickGetTick() >= radioRSSISendTime + 101) {
        radioRSSISendTime = systickGetTick();
        slTxPacket.type = SYSLINK_RADIO_RSSI;
        //This message contains only the RSSI measurement which consist
        //of a single uint8_t
        slTxPacket.length = sizeof(uint8_t);
        memcpy(slTxPacket.data, &rssi, sizeof(uint8_t));

        syslinkSend(&slTxPacket);
      }


      // Sent interrssi of another drone to the STM every 100 ms
      if (systickGetTick() >= radioInterRSSISendTime + (80+drone_id*2)) {
    	  radioInterRSSISendTime = systickGetTick();
    	  uint8_t index = (uint8_t)find_minimum(rssi_array_other_drones,9);
    	  uint8_t inter_rssi_min = rssi_array_other_drones[index];
    	  float rssi_angle_inter_min = rssi_angle_array_other_drones[index];
    	  slTxPacket.type = SYSLINK_RADIO_RSSI_INTER;
    	  slTxPacket.length = 8*sizeof(uint8_t);
    	  memcpy(slTxPacket.data, &inter_rssi_min, sizeof(uint8_t));
    	  memcpy(slTxPacket.data+1, &index, sizeof(uint8_t));
    	  memcpy(slTxPacket.data+2, &inter_beacon_rssi, sizeof(uint8_t));
    	  memcpy(slTxPacket.data+3, &rssi_angle_inter_min, sizeof(float));

          syslinkSend(&slTxPacket);

          // For every 1000 ticks, reset the rssi value if it hasn't been recieved for a while
    	  for(uint8_t it = 0; it<9;it++) if (systickGetTick() >= time_array_other_drones[it]+3000)
    		  {
    		  time_array_other_drones[it] =systickGetTick()+3001;
    		  rssi_array_other_drones[it] = 150;
    		  rssi_angle_array_other_drones[it]=500.0f;
    		  }
      }



#ifdef ISPTX
      // if in PTX mode, send a message which lasts for 10 ms
      // TODO find out why it doesn't allow connection anymore with the dongle in ISPTX mode

      if(!(state_gbug==0)){
      if(inBootloaderMode == false)
      {
    	  // After 1000 ticks Start going into TX mode
		  if (in_ptx==false &&systickGetTick() >= radioPTXSendTime + (200+drone_id*2+1)) {
			  LED_OFF();
			  radioPTXSendTime = systickGetTick();
			  radioPTXtoPRXSendTime = radioPTXSendTime;
			  setupPTXTx(channels_other_drones[count_switch_channel%number_of_channels],rssi_angle_gbug);
			  count_switch_channel ++;
			  //stopPTXTx();
			  //LED_ON();

			  in_ptx = true;
		  }

		  // Indicate by the LEDS if something is send
		  // TODO: find in broadcast (in_ptx), the NRF chip keeps sending messages.
		  if(in_ptx) LED_OFF(); else LED_ON();

		  // After 10 ticks, go back to business as usual
		  if (in_ptx==true && systickGetTick() >= radioPTXtoPRXSendTime + 3) {
			  stopPTXTx();
			  in_ptx = false;
          }
      }
      }
#endif


    }
#endif

    // Button event handling
    ButtonEvent be = buttonGetState();
    bool usbConnected = pmUSBPower();
    if ((pmGetState() != pmSysOff) && (be == buttonShortPress) && !usbConnected)
    {
      inBootloaderMode =false;

      pmSetState(pmAllOff);
      /*swdInit();
      swdTest();*/
    }
    else if ((pmGetState() != pmSysOff) && (be == buttonShortPress)
                                        && usbConnected)
    {
      inBootloaderMode =false;

    	//pmSetState(pmSysOff);
      pmSetState(pmAllOff);
        /*swdInit();
        swdTest();*/
    }
    else if ((pmGetState() == pmSysOff) && (be == buttonShortPress))
    {
      //Normal boot
      inBootloaderMode =false;

      pmSysBootloader(false);
      pmSetState(pmSysRunning);
    }
    else if ((pmGetState() == pmSysOff) && boottedFromBootloader)
    {
      inBootloaderMode =false;
      //Normal boot after bootloader
      pmSysBootloader(false);
      pmSetState(pmSysRunning);
    }
    else if ((pmGetState() == pmSysOff) && (be == buttonLongPress))
    {
      inBootloaderMode =true;

      //stm bootloader
      pmSysBootloader(true);
      pmSetState(pmSysRunning);

    }
    boottedFromBootloader = false;

    // processes loop
    buttonProcess();
    pmProcess();
    //owRun();       //TODO!
  }
}

#define RADIO_CTRL_SET_CHANNEL 1
#define RADIO_CTRL_SET_DATARATE 2
#define RADIO_CTRL_SET_POWER 3
#define ONLY_RSSI_NO_ACK 8

static void handleRadioCmd(struct esbPacket_s *packet)
{
  switch (packet->data[2]) {
    case RADIO_CTRL_SET_CHANNEL:
      esbSetChannel(packet->data[3]);
      break;
    case RADIO_CTRL_SET_DATARATE:
      esbSetDatarate(packet->data[3]);
      break;
    case RADIO_CTRL_SET_POWER:
      esbSetTxPower(packet->data[3]);
      break;
    case ONLY_RSSI_NO_ACK:
    	rssi = packet->rssi;
    	break;
    default:
      break;
  }
}

#define BOOTLOADER_CMD_RESET_INIT 0xFF
#define BOOTLOADER_CMD_RESET      0xF0
#define BOOTLOADER_CMD_ALLOFF     0x01
#define BOOTLOADER_CMD_SYSOFF     0x02
#define BOOTLOADER_CMD_SYSON      0x03
#define BOOTLOADER_CMD_GETVBAT    0x04

static void handleBootloaderCmd(struct esbPacket_s *packet)
{
  static bool resetInit = false;
  static struct esbPacket_s txpk;

  switch (packet->data[2]) {
    case BOOTLOADER_CMD_RESET_INIT:

      resetInit = true;

      txpk.data[0] = 0xff;
      txpk.data[1] = 0xfe;
      txpk.data[2] = BOOTLOADER_CMD_RESET_INIT;

      memcpy(&(txpk.data[3]), (uint32_t*)NRF_FICR->DEVICEADDR, 6);

      txpk.size = 9;
#if BLE
      bleCrazyfliesSendPacket(&txpk);
#endif
      if (esbCanTxPacket()) {
        struct esbPacket_s *pk = esbGetTxPacket();
        memcpy(pk, &txpk, sizeof(struct esbPacket_s));
        esbSendTxPacket(pk);
      }

      break;
    case BOOTLOADER_CMD_RESET:
      if (resetInit && (packet->size == 4)) {
        msDelay(100);
        if (packet->data[3] == 0) {
          NRF_POWER->GPREGRET |= 0x40;
        } else {
          //Set bit 0x20 forces boot to firmware
          NRF_POWER->GPREGRET |= 0x20U;
        }
#ifdef BLE
        sd_nvic_SystemReset();
#else
        NVIC_SystemReset();
#endif
      }
      break;
    case BOOTLOADER_CMD_ALLOFF:
      pmSetState(pmAllOff);
      break;
    case BOOTLOADER_CMD_SYSOFF:
      pmSetState(pmSysOff);
      break;
    case BOOTLOADER_CMD_SYSON:
      pmSysBootloader(false);
      pmSetState(pmSysRunning);
      break;
    case BOOTLOADER_CMD_GETVBAT:
      if (esbCanTxPacket()) {
        float vbat = pmGetVBAT();
        struct esbPacket_s *pk = esbGetTxPacket();

        pk->data[0] = 0xff;
        pk->data[1] = 0xfe;
        pk->data[2] = BOOTLOADER_CMD_GETVBAT;

        memcpy(&(pk->data[3]), &vbat, sizeof(float));
        pk->size = 3 + sizeof(float);

        esbSendTxPacket(pk);
      }
      break;
    default:
      break;
  }
}
