#ifndef INTERFACE_LORAWAN_H
#define INTERFACE_LORAWAN_H

#include <time.h>
#include <RadioLib.h>

enum DeviceActivcationMethod {
  OTAA,
  ABP
};

int16_t LoRaWAN_Init(LoRaWANBand_t _region, uint8_t _subBand, float _roundInterval, uint32_t _uplinkInterval, uint8_t _initialDatarate, bool _ADRActive, 
                  bool _CSMAActive, uint8_t _CSMAMaxChanges, uint8_t _CSMABackoffMax, uint8_t _CSMADifsSlots, 
                  bool _dutyCycleLimitsActive, RadioLibTime_t _dutyCycleMsPerHour, bool _dwellTimeLimitsActive, RadioLibTime_t _dwellTimeMsPerUplink,
                  DeviceActivcationMethod _deviceActivationMethod, 
                  uint64_t _joinEUI, uint64_t _devEUI, uint8_t _nwkKey[16], uint8_t _appKey[16],
                  uint32_t _devAddr, uint8_t _fNwkSIntKey[16], uint8_t _sNwkSIntKey[16], uint8_t _nwkSEncKey[16], uint8_t _appSKey[16]);

void LoRaWANQueueMessage(uint8_t _numberIndex, time_t _resultTimeUTC, double _result,  double _resultRate, uint8_t _errorCode);

bool getLoRaWANisSessionActive();
#endif