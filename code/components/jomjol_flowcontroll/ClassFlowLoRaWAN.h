#ifdef ENABLE_LORAWAN

#pragma once

#ifndef CLASSFFLOWLORAWAN_H
#define CLASSFFLOWLORAWAN_H

#include "ClassFlow.h"

#include "ClassFlowPostProcessing.h"

#include <string>

#include "interface_lorawan.h"

#include <RadioLib.h>

class ClassFlowLoRaWAN :
    public ClassFlow
{
protected:
      LoRaWANBand_t region;
      uint8_t subBand;
      float roundInterval; // Minutes
      uint32_t uplinkInterval;
      uint8_t fixedDatarate;
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
      uint8_t *nwkKey;
      uint8_t *appKey;
      // ABP parameters
      uint32_t devAddr;
      uint8_t *fNwkSIntKey;
      uint8_t *sNwkSIntKey;
      uint8_t *nwkSEncKey;
      uint8_t *appSKey;
      std::string OldValue;
      ClassFlowPostProcessing* flowpostprocessing;
      bool LoRaWANenable;

	  void SetInitialParameter(void);        

public:
    ClassFlowLoRaWAN();
    ClassFlowLoRaWAN(std::vector<ClassFlow*>* lfc);
    ClassFlowLoRaWAN(std::vector<ClassFlow*>* lfc, ClassFlow *_prev);

    bool Start(float AutoInterval);

    bool ReadParameter(FILE* pfile, string& aktparamgraph);
    bool doFlow(string time);
    string name(){return "ClassFlowLoRaWAN";};
};

#endif //CLASSFFLOWLORAWAN_H
#endif //ENABLE_LORAWAN