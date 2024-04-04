#ifdef ENABLE_LORAWAN
#include <string>
#include <esp_task_wdt.h>
#include <CayenneLpp.h>

#include "LoRaWAN_functions.h"
#include "LoRaWANConfig.h" 

// include the library
#include <RadioLib.h>

// include the hardware abstraction layer
#include <RadioLibEspHal.h>

static const char *LORAWAN_LOG_TAG = "LoRaWAN";

// create a new instance of the HAL class
EspHal* hal = new EspHal(42, 47, 41);

// SX1262 has the following pin order:
// Module(NSS/CS, DIO1, RESET, BUSY)
// SX1262 radio = new Module(8, 14, 12, 13);
SX1262 radio = new Module(hal, 1, 3,21,14);

// create the node instance on the EU-868 band
// using the radio module and the encryption key
// make sure you are using the correct band
// based on your geographical location!
LoRaWANNode node(&radio, &EU868);

// for fixed bands with subband selection
// such as US915 and AU915, you must specify
// the subband that matches the Frequency Plan
// that you selected on your LoRaWAN console
/*
  LoRaWANNode node(&radio, &US915, 2);
*/

TaskHandle_t xHandle_task_lorawan_communication = NULL;

CayenneLPP CayenneLpp(250);

uint16_t UnsuccessfulJoinCount = 0;
#ifdef LORAWAN_USE_PERSISTENT_STORAGE
bool newlyJoined = true;
#endif
int Lorawan_counter_value = 0;

void task_lorawan_communication(void *pvParameter) {
  while (1)
  {  
    ESP_LOGD(LORAWAN_LOG_TAG, "Loop started\n");
    int state = RADIOLIB_ERR_NONE;

    // set battery fill level - the LoRaWAN network server
    // may periodically request this information
    // 0 = external power source
    // 1 = lowest (empty battery)
    // 254 = highest (full battery)
    // 255 = unable to measure
    uint8_t battLevel = 146;
    node.setDeviceStatus(battLevel);


    // Build payload byte array
    uint8_t uplinkPayload[250];
    size_t uplinkLen;

    //reset TXFrame
		for (int i=0; i<250; i++) {
		  uplinkPayload[i] = 0;
		}

    CayenneLpp.reset();
    Lorawan_counter_value += 1;
    CayenneLpp.addAnalogOutput(1,Lorawan_counter_value);
    uplinkLen = CayenneLpp.getSize();
    CayenneLpp.copy(uplinkPayload);

    uint8_t downlinkPayload[251];  // Make sure this fits your plans!
    size_t  downlinkSize = 0;         // To hold the actual payload size rec'd

    // you can also retrieve additional information about an uplink or 
    // downlink by passing a reference to LoRaWANEvent_t structure
    LoRaWANEvent_t uplinkDetails;
    LoRaWANEvent_t downlinkDetails;

    uint8_t Port = 2;

    // retrieve the last uplink frame counter
    uint32_t fcntUp = node.getFcntUp();

    ESP_LOGD(LORAWAN_LOG_TAG, "Sending uplink packet and receiving downlink ...\n");  
    // Send a confirmed uplink every 64th frame
    // and also request the LinkCheck and DeviceTime MAC commands
    if(fcntUp % 64 == 0) {
      ESP_LOGD(LORAWAN_LOG_TAG, "Requesting LinkCheck and DeviceTime\n");
      node.sendMacCommandReq(RADIOLIB_LORAWAN_MAC_LINK_CHECK);
      node.sendMacCommandReq(RADIOLIB_LORAWAN_MAC_DEVICE_TIME);
      state = node.sendReceive(uplinkPayload, uplinkLen, Port, downlinkPayload, &downlinkSize, true, &uplinkDetails, &downlinkDetails); 
    } else {
      state = node.sendReceive(uplinkPayload, uplinkLen, Port, downlinkPayload, &downlinkSize);    
    }
    if(state == RADIOLIB_ERR_NONE) {
      ESP_LOGD(LORAWAN_LOG_TAG, "success!\n");
    } else {
      ESP_LOGD(LORAWAN_LOG_TAG, "failed, code %i\n", state);
    }
    
    if(state != RADIOLIB_LORAWAN_NO_DOWNLINK) {
      //strDown = (char*)dataDown;
      std::string strDown = std::string(&downlinkPayload[0],&downlinkPayload[downlinkSize]);
      // print data of the packet (if there are any)
      ESP_LOGD(LORAWAN_LOG_TAG, "Data:\t\t");     
      if (downlinkSize > 0){
        JsonDocument jsonBuffer;
        JsonArray root = jsonBuffer.to<JsonArray>();
        CayenneLpp.decode(downlinkPayload, downlinkSize, root);

        std::string output;
        serializeJson(jsonBuffer, output);
        ESP_LOGD(LORAWAN_LOG_TAG, "%s\n",output.c_str());

        //check if it is a command
        if ((root[0]["type"]) == 2 && (root[0]["channel"]) == 1 ) {
          if ((root[0]["value"]) == 1){
            ESP_LOGD(LORAWAN_LOG_TAG, "Trurning Wifi on");
          } else {
            ESP_LOGD(LORAWAN_LOG_TAG, "Trurning Wifi off");
          }
        }
      } else {
        ESP_LOGD(LORAWAN_LOG_TAG, "<MAC commands only>\n");
      }

      // print RSSI (Received Signal Strength Indicator)
      ESP_LOGD(LORAWAN_LOG_TAG, "RSSI:\t\t%f dbm\n", radio.getRSSI());

      // print SNR (Signal-to-Noise Ratio)
      ESP_LOGD(LORAWAN_LOG_TAG, "SNR:\t\t%f db\n", radio.getSNR());

      // print frequency error
      ESP_LOGD(LORAWAN_LOG_TAG, "Frequency error:\t%f Hz\n", radio.getFrequencyError());

      // print extra information about the event
      ESP_LOGD(LORAWAN_LOG_TAG, "Event information:\n");
      ESP_LOGD(LORAWAN_LOG_TAG, "Direction:\t");
      if(downlinkDetails.dir == RADIOLIB_LORAWAN_CHANNEL_DIR_UPLINK) {
        ESP_LOGD(LORAWAN_LOG_TAG, "uplink\n");
      } else {
        ESP_LOGD(LORAWAN_LOG_TAG, "downlink\n");
      }
      ESP_LOGD(LORAWAN_LOG_TAG, "Confirmed:\t%s\n",downlinkDetails.confirmed?"true":"false");
      ESP_LOGD(LORAWAN_LOG_TAG, "Confirming:\t%s\n",downlinkDetails.confirming?"true":"false");
      ESP_LOGD(LORAWAN_LOG_TAG, "Datarate:\t%u\n",downlinkDetails.datarate);
      ESP_LOGD(LORAWAN_LOG_TAG, "Frequency:\t%f MHz\n",downlinkDetails.freq);
      ESP_LOGD(LORAWAN_LOG_TAG, "Output power:\t%i dbm\n",downlinkDetails.power);
      ESP_LOGD(LORAWAN_LOG_TAG, "Frame count:\t%lu\n",downlinkDetails.fcnt);
      ESP_LOGD(LORAWAN_LOG_TAG, "Port:\t\t%u\n",downlinkDetails.port);

      uint8_t margin = 0;
      uint8_t gwCnt = 0;
      if(node.getMacLinkCheckAns(&margin, &gwCnt) == RADIOLIB_ERR_NONE) {
        ESP_LOGD(LORAWAN_LOG_TAG, "LinkCheck margin:\t%u\n",margin);
        ESP_LOGD(LORAWAN_LOG_TAG, "LinkCheck count:\t%u\n", gwCnt);
      }

      uint32_t networkTime = 0;
      uint8_t fracSecond = 0;
      if(node.getMacDeviceTimeAns(&networkTime, &fracSecond, true) == RADIOLIB_ERR_NONE) {
        ESP_LOGD(LORAWAN_LOG_TAG, "DeviceTime Unix:\t%lu\n", networkTime);
        ESP_LOGD(LORAWAN_LOG_TAG, "DeviceTime second:\t1/%u\n", fracSecond);
      }

    }

    // on EEPROM enabled boards, you should save the current session
    // by calling "saveSession" which allows retrieving the session after reboot or deepsleep
    //node.saveSession();

    // wait before sending another packet
    uint32_t minimumDelay = uplinkIntervalSeconds * 1000UL;
    uint32_t interval = node.timeUntilUplink();     // calculate minimum duty cycle delay (per law!)
    uint32_t delayMs = std::max(interval, minimumDelay); // cannot send faster than duty cycle allows

    ESP_LOGD(LORAWAN_LOG_TAG, "Next uplink in %lu s\n", delayMs / 1000UL);

    hal->delay(delayMs);
  }
  vTaskDelete(xHandle_task_lorawan_communication);
}

void lorawan_initialize() {
  // initialize radio (SX1262 / SX1278 / ... ) with default settings
  ESP_LOGD(LORAWAN_LOG_TAG, "[Radio] Initializing radio ... \n");
  //int state = radio.begin(868.0F,12.05F,9,7,0x12,10,8,1.6F,false);
  int state = radio.begin();
  if(state == RADIOLIB_ERR_NONE) {
    ESP_LOGD(LORAWAN_LOG_TAG, "success!\n");
  } else {
    ESP_LOGD(LORAWAN_LOG_TAG, "failed, code %i\n",state);
    while(true);
  }

#ifdef LORAWAN_USE_PERSISTENT_STORAGE
  ##### if we have previously saved nonces, restore them
  uint8_t noncesBuffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];										            // create somewhere to store nonces
  esp_err_t status = hal->readPersistentStorage("nonces", noncesBuffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE); // get them to the store
  if (status == ESP_OK) { 
    state = node.setBufferNonces(noncesBuffer); 															            // send them to LoRaWAN
    if(state == RADIOLIB_ERR_NONE) {
      ESP_LOGD(LORAWAN_LOG_TAG, "Successfully restored nonces buffer\n");
      newlyJoined = false;
    } else {
      ESP_LOGD(LORAWAN_LOG_TAG, "Restoring nonces buffer failed\n");
    }
  } else if (status == ESP_ERR_NVS_NOT_FOUND){
    ESP_LOGD(LORAWAN_LOG_TAG, "It seems that no nonces are stored\n");
  } else {
    ESP_LOGD(LORAWAN_LOG_TAG, "Something went wrong while restoring the nonces\n");
  }
#endif
  
  ESP_LOGD(LORAWAN_LOG_TAG, "Setup LoRaWAN session\n");
  ESP_LOGD(LORAWAN_LOG_TAG, "Join ('login') to the LoRaWAN Network\n");
  state = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey, true,5);

   // loop until successful join
  while (state != RADIOLIB_ERR_NONE) {
    ESP_LOGD(LORAWAN_LOG_TAG, "Join failed: %i\n", state);
    // how long to wait before join attempts. This is an interim solution pending 
    // implementation of TS001 LoRaWAN Specification section #7 - this doc applies to v1.0.4 & v1.1
    // it sleeps for longer & longer durations to give time for any gateway issues to resolve
    // or whatever is interfering with the device <-> gateway airwaves.
    uint32_t sleepForSeconds = std::min((UnsuccessfulJoinCount++ + 1UL) * 60UL, 3UL * 60UL);
    ESP_LOGD(LORAWAN_LOG_TAG, "Boots since unsuccessful join: %u\n", UnsuccessfulJoinCount);
    ESP_LOGD(LORAWAN_LOG_TAG, "Retrying join in %lu seconds\n", sleepForSeconds);

    //hal->delay(sleepForSeconds*1000);
    hal->delay(1000);

    ESP_LOGD(LORAWAN_LOG_TAG, "Join ('login') to the LoRaWAN Network\n");
    state = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey, true,5);  
  } // while join
  ESP_LOGD(LORAWAN_LOG_TAG, "Joined\n");

#ifdef LORAWAN_USE_PERSISTENT_STORAGE
  if (newlyJoined){
    // ##### save the join counters (nonces) to permanent store
    ESP_LOGD(LORAWAN_LOG_TAG, "Saving nonces to flash\n");
    uint8_t *persist = node.getBufferNonces();                                        // get pointer to nonces
    memcpy(noncesBuffer, persist, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);                        // copy in to buffer
    hal->writePersistentStorage("nonces", noncesBuffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);  // send them to the store
  }
#endif

  // reset the failed join count
  UnsuccessfulJoinCount = 0;

  hal->delay(1000);  // hold off off hitting the airwaves again too soon - an issue in the US

  ESP_LOGD(LORAWAN_LOG_TAG, "DevAddr: %llx",node.getDevAddr());

  // disable the ADR algorithm
  //node.setADR(false);

  // Set a fixed datarate & make it persistent (not normal)
  //node.setDatarate(4);

  // enable CSMA
  // this tries to minimize packet loss by searching for a free channel
  // before actually sending an uplink
  //node.setCSMA(6, 2, true);

  // enable or disable the dutycycle
  // the second argument specific allowed airtime per hour in milliseconds
  // 1250 = TTN FUP (30 seconds / 24 hours)
  // if not called, this corresponds to setDutyCycle(true, 0)
  // setting this to 0 corresponds to the band's maximum allowed dutycycle by law
  //node.setDutyCycle(true, 1250);

  // enable or disable the dwell time limits
  // the second argument specifies the allowed airtime per uplink in milliseconds
  // unless specified, this argument is set to 0
  // setting this to 0 corresponds to the band's maximum allowed dwell time by law
  node.setDwellTime(false, 0);

  // start communication task
  BaseType_t xReturned = xTaskCreate(task_lorawan_communication, "task_lorawan_communication", 16384, NULL, tskIDLE_PRIORITY+1, &xHandle_task_lorawan_communication);
  if(xReturned != pdPASS)
		{
			xHandle_task_lorawan_communication = NULL;
			// LogFile.WriteToFile(ESP_LOG_ERROR, LORAWAN_LOG_TAG, "task_lorawan_communication failed to create");
      //   	LogFile.WriteHeapInfo("task_lorawan_communication failed");
		}
}
#endif //ENABLE_LORAWAN