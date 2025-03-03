#ifdef ENABLE_LORAWAN
#include <string>
#include <esp_task_wdt.h>
#include <CayenneLpp.h>

#include "../../include/defines.h"

#include "interface_lorawan.h"

// include the library
#include <RadioLib.h>

// include the hardware abstraction layer
#include "RadioLibEspHal.h"

static const char *LORAWAN_LOG_TAG = "LoRaWAN";

// create a new instance of the HAL class
EspHal* hal = new EspHal(LORAWAN_SPI_SCK, LORAWAN_SPI_MISO, LORAWAN_SPI_MOSI);

// SX1262 has the following pin order:
// Module(NSS/CS, DIO1, RESET, BUSY)
// SX1262 radio = new Module(8, 14, 12, 13);
SX1262 radio = new Module(hal, LORAWAN_RADIO_CS, LORAWAN_RADIO_IRQ, LORAWAN_RADIO_RST, LORAWAN_RADIO_GPIO);


LoRaWANNode *node;


TaskHandle_t xHandle_task_lorawan_communication = NULL;

CayenneLPP CayenneLpp(250);

RTC_DATA_ATTR uint16_t bootCount = 0;
RTC_DATA_ATTR uint16_t bootCountSinceUnsuccessfulJoin = 0;
RTC_DATA_ATTR uint8_t LWsession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];

int Lorawan_counter_value = 0;

LoRaWANBand_t region;
uint8_t subBand;
uint32_t uplinkInterval;
uint8_t initialDatarate;
bool ADRActive;
bool CSMAActive;
uint8_t CSMAMaxChanges;
uint8_t CSMABackoffMax;
uint8_t CSMADifsSlots;
bool dutyCycleLimitsActive;
RadioLibTime_t dutyCycleMsPerHour;
bool dwellTimeLimitsActive;
RadioLibTime_t dwellTimeMsPerUplink;
DeviceActivcationMethod deviceActivationMethod;
// OTAA parameters
uint64_t joinEUI;
uint64_t devEUI;
uint8_t* nwkKey;
uint8_t* appKey;
// ABP parameters
uint32_t devAddr;
uint8_t* fNwkSIntKey;
uint8_t* sNwkSIntKey;
uint8_t* nwkSEncKey;
uint8_t* appSKey;

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
    uint8_t battLevel = 255;
    node->setDeviceStatus(battLevel);


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

    uint8_t fPort = 1;

    // retrieve the last uplink frame counter
    uint32_t fcntUp = node->getFCntUp();

    ESP_LOGD(LORAWAN_LOG_TAG, "Sending uplink packet and receiving downlink ...\n");  
    // Send a confirmed uplink every 64th frame
    // and also request the LinkCheck and DeviceTime MAC commands
    if(fcntUp % 64 == 0) {
      ESP_LOGD(LORAWAN_LOG_TAG, "Requesting LinkCheck and DeviceTime\n");
      node->sendMacCommandReq(RADIOLIB_LORAWAN_MAC_LINK_CHECK);
      node->sendMacCommandReq(RADIOLIB_LORAWAN_MAC_DEVICE_TIME);
      state = node->sendReceive(uplinkPayload, uplinkLen, fPort, downlinkPayload, &downlinkSize, true, &uplinkDetails, &downlinkDetails); 
    } else {
      state = node->sendReceive(uplinkPayload, uplinkLen, fPort, downlinkPayload, &downlinkSize, false, &uplinkDetails, &downlinkDetails);    
    }
    if(state == RADIOLIB_ERR_NONE) {
      ESP_LOGD(LORAWAN_LOG_TAG, "success!\n");
    } else {
      ESP_LOGD(LORAWAN_LOG_TAG, "failed, code %i\n", state);
    }
    
    // Check if a downlink was received 
    // (state 0 = no downlink, state 1/2 = downlink in window Rx1/Rx2)
    if(state > 0) {
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
      ESP_LOGD(LORAWAN_LOG_TAG, "Confirmed:\t%s\n",downlinkDetails.confirmed?"true":"false");
      ESP_LOGD(LORAWAN_LOG_TAG, "Confirming:\t%s\n",downlinkDetails.confirming?"true":"false");
      ESP_LOGD(LORAWAN_LOG_TAG, "Datarate:\t%u\n",downlinkDetails.datarate);
      ESP_LOGD(LORAWAN_LOG_TAG, "Frequency:\t%f MHz\n",downlinkDetails.freq);
      ESP_LOGD(LORAWAN_LOG_TAG, "Output power:\t%i dbm\n",downlinkDetails.power);
      ESP_LOGD(LORAWAN_LOG_TAG, "Frame count:\t%lu\n",downlinkDetails.fCnt);
      ESP_LOGD(LORAWAN_LOG_TAG, "Port:\t\t%u\n",downlinkDetails.fPort);
      ESP_LOGD(LORAWAN_LOG_TAG, "Time-on-air:\t\t%u ms\n",node->getLastToA());
      ESP_LOGD(LORAWAN_LOG_TAG, "Rx window:\t\t%u\n",state);

      uint8_t margin = 0;
      uint8_t gwCnt = 0;
      if(node->getMacLinkCheckAns(&margin, &gwCnt) == RADIOLIB_ERR_NONE) {
        ESP_LOGD(LORAWAN_LOG_TAG, "LinkCheck margin:\t%u\n",margin);
        ESP_LOGD(LORAWAN_LOG_TAG, "LinkCheck count:\t%u\n", gwCnt);
      }

      uint32_t networkTime = 0;
      uint8_t fracSecond = 0;
      if(node->getMacDeviceTimeAns(&networkTime, &fracSecond, true) == RADIOLIB_ERR_NONE) {
        ESP_LOGD(LORAWAN_LOG_TAG, "DeviceTime Unix:\t%lu\n", networkTime);
        ESP_LOGD(LORAWAN_LOG_TAG, "DeviceTime second:\t1/%u\n", fracSecond);
      }

    } else {
      ESP_LOGD(LORAWAN_LOG_TAG, "No downlink received\n");
    }

    // now save session to RTC memory
#ifdef LORAWAN_USE_PERSISTENT_STORAGE
    uint8_t *persist = node->getBufferSession();
    memcpy(LWsession, persist, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
#endif

    // wait before sending another packet
    uint32_t minimumDelay = uplinkInterval * 1000UL;
    uint32_t interval = node->timeUntilUplink();     // calculate minimum duty cycle delay (per law!)
    uint32_t delayMs = std::max(interval, minimumDelay); // cannot send faster than duty cycle allows

    ESP_LOGD(LORAWAN_LOG_TAG, "Next uplink in %lu seconds\n", delayMs / 1000UL);

    hal->delay(delayMs);
  }
  vTaskDelete(xHandle_task_lorawan_communication);
}

int16_t lwActivate() {
  int16_t state = RADIOLIB_ERR_UNKNOWN;

  if (deviceActivationMethod = DeviceActivcationMethod::OTAA) {
    // setup the OTAA session information
    state = node->beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  } 
  if (deviceActivationMethod = DeviceActivcationMethod::ABP) {
    // setup the ABP session information
    state = node->beginABP(devAddr, fNwkSIntKey, sNwkSIntKey, nwkSEncKey, appSKey);
  }
  
#ifdef LORAWAN_USE_PERSISTENT_STORAGE
  // if we have previously saved nonces, restore them and try to restore session as well
  uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];										                                  // create somewhere to store nonces
  esp_err_t status = hal->readPersistentStorage("nonces", buffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);    // get them to the store
  if (status == ESP_OK) { 
    state = node->setBufferNonces(buffer); 															                                // send them to LoRaWAN
    if(state == RADIOLIB_ERR_NONE) {
      ESP_LOGD(LORAWAN_LOG_TAG, "Successfully restored nonces buffer\n");
    } else {
      ESP_LOGD(LORAWAN_LOG_TAG, "Restoring nonces buffer failed\n");
    }

    // recall session from RTC deep-sleep preserved variable
    state = node->setBufferSession(LWsession); // send them to LoRaWAN stack

    // if we have booted more than once we should have a session to restore, so report any failure
    // otherwise no point saying there's been a failure when it was bound to fail with an empty LWsession var.
    if ((state != RADIOLIB_ERR_NONE) && (bootCount > 1)) {
      ESP_LOGD(LORAWAN_LOG_TAG, "Restoring session buffer failed, code %i\n", state);
    }

    // if Nonces and Session restored successfully, activation is just a formality
    // moreover, Nonces didn't change so no need to re-save them
    if (state == RADIOLIB_ERR_NONE) {
      ESP_LOGD(LORAWAN_LOG_TAG, "Succesfully restored session - now activating\n");
      if (deviceActivationMethod = DeviceActivcationMethod::OTAA) {
        state = node->activateOTAA(initialDatarate);
      } 
      if (deviceActivationMethod = DeviceActivcationMethod::ABP) {
        state = node->activateABP(initialDatarate);
      }     
      if (state != RADIOLIB_LORAWAN_SESSION_RESTORED) {
        ESP_LOGD(LORAWAN_LOG_TAG, "Failed to activate restored session, code %i\n", state);
      }

      return(state);
    }
  } else if (status == ESP_ERR_NVS_NOT_FOUND){
    ESP_LOGD(LORAWAN_LOG_TAG, "It seems that no nonces are stored\n");
  } else {
    ESP_LOGD(LORAWAN_LOG_TAG, "Something went wrong while restoring the nonces\n");
  }
#endif
  
  // if we got here, there was no session to restore, so start trying to join
  state = RADIOLIB_ERR_NETWORK_NOT_JOINED;
  // loop until successful join
  while (state != RADIOLIB_LORAWAN_NEW_SESSION) {
    ESP_LOGD(LORAWAN_LOG_TAG, "Join ('login') to the LoRaWAN Network\n");
    if (deviceActivationMethod = DeviceActivcationMethod::OTAA) {
      state = node->activateOTAA(initialDatarate);
    } 
    if (deviceActivationMethod = DeviceActivcationMethod::ABP) {
      state = node->activateABP(initialDatarate);
    }   

#ifdef LORAWAN_USE_PERSISTENT_STORAGE
    // save the join counters (nonces) to permanent store
    ESP_LOGD(LORAWAN_LOG_TAG, "Saving nonces to flash\n");
    uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];           // create somewhere to store nonces
    uint8_t *persist = node->getBufferNonces();                  // get pointer to nonces
    memcpy(buffer, persist, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);  // copy in to buffer
    hal->writePersistentStorage("nonces", buffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE); // send them to the store   
#endif

    // we'll save the session after an uplink

    if (state != RADIOLIB_LORAWAN_NEW_SESSION) {
      ESP_LOGD(LORAWAN_LOG_TAG, "Join failed: %i\n", state);
      // how long to wait before join attempts. This is an interim solution pending 
      // implementation of TS001 LoRaWAN Specification section #7 - this doc applies to v1.0.4 & v1.1
      // it sleeps for longer & longer durations to give time for any gateway issues to resolve
      // or whatever is interfering with the device <-> gateway airwaves.
      uint32_t sleepForSeconds = std::min((bootCountSinceUnsuccessfulJoin++ + 1UL) * 60UL, 3UL * 60UL);
      ESP_LOGD(LORAWAN_LOG_TAG, "Boots since unsuccessful join: %u\n", bootCountSinceUnsuccessfulJoin);
      ESP_LOGD(LORAWAN_LOG_TAG, "Retrying join in %lu seconds\n", sleepForSeconds);

      hal->delay(sleepForSeconds*1000);

    } // if activateOTAA state
  } // while join
  ESP_LOGD(LORAWAN_LOG_TAG, "Joined\n");

  // reset the failed join count
  bootCountSinceUnsuccessfulJoin = 0;

  hal->delay(1000);  // hold off off hitting the airwaves again too soon - an issue in the US

  ESP_LOGD(LORAWAN_LOG_TAG, "DevAddr: %llx",node->getDevAddr());
  return(state);
}

int16_t LoRaWAN_Init(LoRaWANBand_t _region, uint8_t _subBand, uint32_t _uplinkInterval, uint8_t _initialDatarate, bool _ADRActive, 
                bool _CSMAActive, uint8_t _CSMAMaxChanges, uint8_t _CSMABackoffMax, uint8_t _CSMADifsSlots, 
                bool _dutyCycleLimitsActive, RadioLibTime_t _dutyCycleMsPerHour, bool _dwellTimeLimitsActive, RadioLibTime_t _dwellTimeMsPerUplink,
                DeviceActivcationMethod _deviceActivationMethod, 
                uint64_t _joinEUI, uint64_t _devEUI, uint8_t* _nwkKey, uint8_t* _appKey,
                uint32_t _devAddr, uint8_t* _fNwkSIntKey, uint8_t* _sNwkSIntKey, uint8_t* _nwkSEncKey, uint8_t* _appSKey) {
  region = _region;
  subBand = _subBand;
  uplinkInterval = _uplinkInterval;
  initialDatarate = _initialDatarate;
  ADRActive = _ADRActive;
  CSMAActive = _CSMAActive;
  CSMAMaxChanges = _CSMAMaxChanges;
  CSMABackoffMax = _CSMABackoffMax;
  CSMADifsSlots = _CSMADifsSlots;
  dutyCycleLimitsActive = _dutyCycleLimitsActive;
  dutyCycleMsPerHour = _dutyCycleMsPerHour;
  dwellTimeLimitsActive = _dwellTimeLimitsActive;
  dwellTimeMsPerUplink = _dwellTimeMsPerUplink;
  deviceActivationMethod = _deviceActivationMethod;
  // OTAA parameters
  joinEUI = _joinEUI;
  devEUI = _devEUI;
  nwkKey = _nwkKey;
  appKey = _appKey;
  // ABP parameters
  devAddr = _devAddr;
  fNwkSIntKey = _fNwkSIntKey;
  sNwkSIntKey = _sNwkSIntKey;
  nwkSEncKey = _nwkSEncKey;
  appSKey = _appSKey;

  // create the node instance on the EU-868 band
  // using the radio module and the encryption key
  // make sure you are using the correct band
  // based on your geographical location!
  // for fixed bands with subband selection
  // such as US915 and AU915, you must specify
  // the subband that matches the Frequency Plan
  // that you selected on your LoRaWAN console
  /*
    LoRaWANNode node(&radio, &US915, 2);
  */
  node = new LoRaWANNode(&radio, &region, subBand);

 
  int16_t state = 0;  						// return value for calls to RadioLib

  // initialize radio based on configured pinmap
  ESP_LOGD(LORAWAN_LOG_TAG, "[Radio] Initializing radio ... \n");
  state = radio.begin();
  if(state == RADIOLIB_ERR_NONE) {
    ESP_LOGD(LORAWAN_LOG_TAG, "success!\n");
  } else {
    ESP_LOGD(LORAWAN_LOG_TAG, "failed, code %i\n",state);
    while(true);
  }

  // activate node by restoring session or otherwise joining the network
  state = lwActivate();
  // state is one of RADIOLIB_LORAWAN_NEW_SESSION or RADIOLIB_LORAWAN_SESSION_RESTORED

  // disable the ADR algorithm
  node->setADR(ADRActive);

  // enable CSMA
  // this tries to minimize packet loss by searching for a free channel
  // before actually sending an uplink
  node->setCSMA(CSMAActive, CSMAMaxChanges, CSMABackoffMax, CSMADifsSlots);

  // enable or disable the dutycycle
  // the second argument specific allowed airtime per hour in milliseconds
  // 1250 = TTN FUP (30 seconds / 24 hours)
  // if not called, this corresponds to setDutyCycle(true, 0)
  // setting this to 0 corresponds to the band's maximum allowed dutycycle by law
  node->setDutyCycle(dutyCycleLimitsActive, dutyCycleMsPerHour);

  // enable or disable the dwell time limits
  // the second argument specifies the allowed airtime per uplink in milliseconds
  // unless specified, this argument is set to 0
  // setting this to 0 corresponds to the band's maximum allowed dwell time by law
  node->setDwellTime(dwellTimeLimitsActive, dwellTimeMsPerUplink);

  // start communication task
  BaseType_t xReturned = xTaskCreate(task_lorawan_communication, "task_lorawan_communication", 16384, NULL, tskIDLE_PRIORITY+1, &xHandle_task_lorawan_communication);
  if(xReturned != pdPASS)	{
		xHandle_task_lorawan_communication = NULL;
	}

  return 0;
}

// result code to text - these are error codes that can be raised when using LoRaWAN
// however, RadioLib has many more - see https://jgromes.github.io/RadioLib/group__status__codes.html for a complete list
std::string stateDecode(const int16_t result) {
  switch (result) {
  case RADIOLIB_ERR_NONE:
    return "ERR_NONE";
  case RADIOLIB_ERR_CHIP_NOT_FOUND:
    return "ERR_CHIP_NOT_FOUND";
  case RADIOLIB_ERR_PACKET_TOO_LONG:
    return "ERR_PACKET_TOO_LONG";
  case RADIOLIB_ERR_RX_TIMEOUT:
    return "ERR_RX_TIMEOUT";
  case RADIOLIB_ERR_CRC_MISMATCH:
    return "ERR_CRC_MISMATCH";
  case RADIOLIB_ERR_INVALID_BANDWIDTH:
    return "ERR_INVALID_BANDWIDTH";
  case RADIOLIB_ERR_INVALID_SPREADING_FACTOR:
    return "ERR_INVALID_SPREADING_FACTOR";
  case RADIOLIB_ERR_INVALID_CODING_RATE:
    return "ERR_INVALID_CODING_RATE";
  case RADIOLIB_ERR_INVALID_FREQUENCY:
    return "ERR_INVALID_FREQUENCY";
  case RADIOLIB_ERR_INVALID_OUTPUT_POWER:
    return "ERR_INVALID_OUTPUT_POWER";
  case RADIOLIB_ERR_NETWORK_NOT_JOINED:
	  return "RADIOLIB_ERR_NETWORK_NOT_JOINED";
  case RADIOLIB_ERR_DOWNLINK_MALFORMED:
    return "RADIOLIB_ERR_DOWNLINK_MALFORMED";
  case RADIOLIB_ERR_INVALID_REVISION:
    return "RADIOLIB_ERR_INVALID_REVISION";
  case RADIOLIB_ERR_INVALID_PORT:
    return "RADIOLIB_ERR_INVALID_PORT";
  case RADIOLIB_ERR_NO_RX_WINDOW:
    return "RADIOLIB_ERR_NO_RX_WINDOW";
  case RADIOLIB_ERR_INVALID_CID:
    return "RADIOLIB_ERR_INVALID_CID";
  case RADIOLIB_ERR_UPLINK_UNAVAILABLE:
    return "RADIOLIB_ERR_UPLINK_UNAVAILABLE";
  case RADIOLIB_ERR_COMMAND_QUEUE_FULL:
    return "RADIOLIB_ERR_COMMAND_QUEUE_FULL";
  case RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND:
    return "RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND";
  case RADIOLIB_ERR_JOIN_NONCE_INVALID:
    return "RADIOLIB_ERR_JOIN_NONCE_INVALID";
  case RADIOLIB_ERR_N_FCNT_DOWN_INVALID:
    return "RADIOLIB_ERR_N_FCNT_DOWN_INVALID";
  case RADIOLIB_ERR_A_FCNT_DOWN_INVALID:
    return "RADIOLIB_ERR_A_FCNT_DOWN_INVALID";
  case RADIOLIB_ERR_DWELL_TIME_EXCEEDED:
    return "RADIOLIB_ERR_DWELL_TIME_EXCEEDED";
  case RADIOLIB_ERR_CHECKSUM_MISMATCH:
    return "RADIOLIB_ERR_CHECKSUM_MISMATCH";
  case RADIOLIB_ERR_NO_JOIN_ACCEPT:
    return "RADIOLIB_ERR_NO_JOIN_ACCEPT";
  case RADIOLIB_LORAWAN_SESSION_RESTORED:
    return "RADIOLIB_LORAWAN_SESSION_RESTORED";
  case RADIOLIB_LORAWAN_NEW_SESSION:
    return "RADIOLIB_LORAWAN_NEW_SESSION";
  case RADIOLIB_ERR_NONCES_DISCARDED:
    return "RADIOLIB_ERR_NONCES_DISCARDED";
  case RADIOLIB_ERR_SESSION_DISCARDED:
    return "RADIOLIB_ERR_SESSION_DISCARDED";
  }
  return "See https://jgromes.github.io/RadioLib/group__status__codes.html";
}

#endif //ENABLE_LORAWAN