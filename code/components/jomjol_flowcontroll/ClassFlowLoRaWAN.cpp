#ifdef ENABLE_LORAWAN

#include "ClassFlowLoRaWAN.h"
#include "Helper.h"
#include "ClassLogFile.h"

#include "ClassFlowPostProcessing.h"
#include "ClassFlowControll.h"

static const char *TAG = "LoRaWAN";

extern const char* libfive_git_version(void);
extern const char* libfive_git_revision(void);
extern const char* libfive_git_branch(void);

void ClassFlowLoRaWAN::SetInitialParameter(void)
{
      region = EU868;
      subBand = 0;
      uplinkInterval = 60;
      initialDatarate = RADIOLIB_LORAWAN_DATA_RATE_UNUSED;
      ADRActive = true;
      CSMAActive = false;
      CSMAMaxChanges = 4;
      CSMABackoffMax = 0;
      CSMADifsSlots = 2;
      dutyCycleLimitsActive = true;
      dutyCycleMsPerHour = 0;
      dwellTimeLimitsActive = true;
      dwellTimeMsPerUplink = 0;
      deviceActivationMethod = DeviceActivcationMethod::OTAA;
      // OTAA parameters
      joinEUI = NULL;
      devEUI = NULL;
      nwkKey = NULL;
      appKey = NULL;
      // ABP parameters
      devAddr = NULL;
      fNwkSIntKey = NULL;
      sNwkSIntKey = NULL;
      nwkSEncKey = NULL;
      appSKey = NULL;
      
      OldValue = "";
      flowpostprocessing = NULL;  
      previousElement = NULL;
      ListFlowControll = NULL; 
      disabled = false;
      LoRaWANenable = false;
}   

ClassFlowLoRaWAN::ClassFlowLoRaWAN()
{
    SetInitialParameter();
}

ClassFlowLoRaWAN::ClassFlowLoRaWAN(std::vector<ClassFlow*>* lfc)
{
    SetInitialParameter();

    ListFlowControll = lfc;
    for (int i = 0; i < ListFlowControll->size(); ++i)
    {
        if (((*ListFlowControll)[i])->name().compare("ClassFlowPostProcessing") == 0)
        {
            flowpostprocessing = (ClassFlowPostProcessing*) (*ListFlowControll)[i];
        }
    }
}

ClassFlowLoRaWAN::ClassFlowLoRaWAN(std::vector<ClassFlow*>* lfc, ClassFlow *_prev)
{
    SetInitialParameter();

    previousElement = _prev;
    ListFlowControll = lfc;

    for (int i = 0; i < ListFlowControll->size(); ++i)
    {
        if (((*ListFlowControll)[i])->name().compare("ClassFlowPostProcessing") == 0)
        {
            flowpostprocessing = (ClassFlowPostProcessing*) (*ListFlowControll)[i];
        }
    }
}


std::vector<uint8_t> hexStringToByteArray(const std::string& hexString)
{
    std::vector<uint8_t> byteArray;

    // Loop through the hex string, two characters at a time
    for (size_t i = 0; i < hexString.length(); i += 2) {
        // Extract two characters representing a byte
        std::string byteString = hexString.substr(i, 2);

        // Convert the byte string to a uint8_t value
        uint8_t byteValue = static_cast<uint8_t>(
            stoi(byteString, nullptr, 16));

        // Add the byte to the byte array
        byteArray.push_back(byteValue);
    }

    return byteArray;
}

bool ClassFlowLoRaWAN::ReadParameter(FILE* pfile, string& aktparamgraph)
{
    std::vector<string> splitted;

    aktparamgraph = trim(aktparamgraph);

    if (aktparamgraph.size() == 0)
        if (!this->GetNextParagraph(pfile, aktparamgraph))
            return false;

    if (toUpper(aktparamgraph).compare("[LORAWAN]") != 0)
        return false;

    while (this->getNextLine(pfile, &aktparamgraph) && !this->isNewParagraph(aktparamgraph))
    {
        splitted = ZerlegeZeile(aktparamgraph);
        std::string _param = GetParameterName(splitted[0]);

        if ((toUpper(_param) == "REGION") && (splitted.size() > 1))
        {
            if (splitted[1] == "EU868")
                region = EU868;
            if (splitted[1] == "US915")
                region = US915;
            if (splitted[1] == "EU433")
                region = EU433;
            if (splitted[1] == "AU915")
                region = AU915;
            if (splitted[1] == "CN470")
                region = CN470;
            if (splitted[1] == "AS923")
                region = AS923;
            if (splitted[1] == "AS923_2")
                region = AS923_2;
            if (splitted[1] == "AS923_3")
                region = AS923_3;
            if (splitted[1] == "AS923_4")
                region = AS923_4;
            if (splitted[1] == "KR920")
                region = KR920;
            if (splitted[1] == "IN865")
                region = IN865;
        }
        if ((toUpper(_param) == "SUBBAND") && (splitted.size() > 1))
        {
            this->subBand = std::strtoul(splitted[1].c_str(), NULL, 10);
        }
        if ((toUpper(_param) == "UPLINKINTERVAL") && (splitted.size() > 1))
        {
            this->uplinkInterval = std::strtoul(splitted[1].c_str(), NULL, 10);
        }
        if ((toUpper(_param) == "INITIALDATARATE") && (splitted.size() > 1))
        {
            this->initialDatarate = std::strtoul(splitted[1].c_str(), NULL, 10);
        }
        if ((toUpper(_param) == "ADRACTIVE") && (splitted.size() > 1))
        {
            if (toUpper(splitted[1]) == "FALSE") {
                ADRActive = false;
            }
        }
        if ((toUpper(_param) == "CSMAACTIVE") && (splitted.size() > 1))
        {
            if (toUpper(splitted[1]) == "TRUE") {
                CSMAActive = true;
            }
        }
        if ((toUpper(_param) == "CSMAMAXCHANGES") && (splitted.size() > 1))
        {
            this->CSMAMaxChanges = std::strtoul(splitted[1].c_str(), NULL, 10);
        }
        if ((toUpper(_param) == "CSMABACKOFFMAX") && (splitted.size() > 1))
        {
            this->CSMABackoffMax = std::strtoul(splitted[1].c_str(), NULL, 10);
        }
        if ((toUpper(_param) == "CSMADIFSSLOTS") && (splitted.size() > 1))
        {
            this->CSMADifsSlots = std::strtoul(splitted[1].c_str(), NULL, 10);
        }
        if ((toUpper(_param) == "DUTYCYCLELIMITSACTIVE") && (splitted.size() > 1))
        {
            if (toUpper(splitted[1]) == "FALSE") {
                dutyCycleLimitsActive = false;
            }
        }
        if ((toUpper(_param) == "DUTYCYCLEMSPERHOUR") && (splitted.size() > 1))
        {
            this->dutyCycleMsPerHour = std::strtoul(splitted[1].c_str(), NULL, 10);
        }
        if ((toUpper(_param) == "DWELLTIMELIMITSACTIVE") && (splitted.size() > 1))
        {
            if (toUpper(splitted[1]) == "FALSE") {
                dwellTimeLimitsActive = false;
            }
        }
        if ((toUpper(_param) == "DWELLTIMEMSPERUPLINK") && (splitted.size() > 1))
        {
            this->dwellTimeMsPerUplink = std::strtoul(splitted[1].c_str(), NULL, 10);
        }
        if ((toUpper(_param) == "DEVICEACTIVATIONMETHOD") && (splitted.size() > 1))
        {
            if (splitted[1] == "ABP")
                this->deviceActivationMethod = DeviceActivcationMethod::ABP;               
            else {
                this->deviceActivationMethod = DeviceActivcationMethod::OTAA;
            }
        }
        // OTAA parameters
        if ((toUpper(_param) == "JOINEUI") && (splitted.size() > 1))
        {
            if (splitted[1] != "undefined") {
                this->joinEUI = std::strtoull(splitted[1].c_str(),NULL,16);
            } else {
                this->joinEUI = NULL;
            }
        }
        if ((toUpper(_param) == "DEVEUI") && (splitted.size() > 1))
        {
            if (splitted[1] != "undefined") {
                this->devEUI = std::strtoull(splitted[1].c_str(),NULL,16);
            } else {
                this->devEUI = NULL;
            }
        }
        if ((toUpper(_param) == "NWKKEY") && (splitted.size() > 1))
        {           
            if (splitted[1] != "undefined") {
                this->nwkKey = new uint8_t[16];
                std::vector<uint8_t> nwkKeyVector = hexStringToByteArray(splitted[1]);
                copy(nwkKeyVector.begin(), nwkKeyVector.end(), this->nwkKey);
            } else {
                this->nwkKey = NULL;
            }
        }
        if ((toUpper(_param) == "APPKEY") && (splitted.size() > 1))
        {          
            if (splitted[1] != "undefined") {
                this->appKey = new uint8_t[16];
                std::vector<uint8_t> appKeyVector = hexStringToByteArray(splitted[1]);
                copy(appKeyVector.begin(), appKeyVector.end(), this->appKey);
            } else {
                this->appKey = NULL;
            }
        }
        // ABP parameters
        if ((toUpper(_param) == "DEVADDR") && (splitted.size() > 1))
        {
            if (splitted[1] != "undefined") {
                this->devAddr = std::strtoull(splitted[1].c_str(),NULL,16);
            } else {
                this->devAddr = NULL;
            }
        }
        if ((toUpper(_param) == "FNWKSINTKEY") && (splitted.size() > 1))
        {
            if (splitted[1] != "undefined") {
                this->fNwkSIntKey = new uint8_t[16];
                std::vector<uint8_t> fNwkSIntKeyVector = hexStringToByteArray(splitted[1]);
                copy(fNwkSIntKeyVector.begin(), fNwkSIntKeyVector.end(), this->fNwkSIntKey);
            } else {
                this->fNwkSIntKey = NULL;
            }
        }
        if ((toUpper(_param) == "SNWKSINTKEY") && (splitted.size() > 1))
        {        
            if (splitted[1] != "undefined") {
                this->sNwkSIntKey = new uint8_t[16];
                std::vector<uint8_t> sNwkSIntKeyVector = hexStringToByteArray(splitted[1]);
                copy(sNwkSIntKeyVector.begin(), sNwkSIntKeyVector.end(), this->sNwkSIntKey);
            } else {
                this->sNwkSIntKey = NULL;
            }
        }
        if ((toUpper(_param) == "NWKSENCKEY") && (splitted.size() > 1))
        {           
            if (splitted[1] != "undefined") {
                this->nwkSEncKey = new uint8_t[16];
                std::vector<uint8_t> nwkSEncKeyVector = hexStringToByteArray(splitted[1]);
                copy(nwkSEncKeyVector.begin(), nwkSEncKeyVector.end(), this->nwkSEncKey);
            } else {
                this->nwkSEncKey = NULL;
            }
        }
        if ((toUpper(_param) == "APPSKEY") && (splitted.size() > 1))
        {         
            if (splitted[1] != "undefined") {
                this->appSKey = new uint8_t[16];
                std::vector<uint8_t> appSKeyVector = hexStringToByteArray(splitted[1]);
                copy(appSKeyVector.begin(), appSKeyVector.end(), this->appSKey);
            } else {
                this->appSKey = NULL;
            }
        }
    }

    return true;
}

bool ClassFlowLoRaWAN::Start(float AutoInterval) 
{
    LogFile.WriteToFile(ESP_LOG_DEBUG, TAG, "Starting LoRaWAN communication");

    roundInterval = AutoInterval; // Minutes
    uint16_t LoRaWANInitializeCheck = LoRaWAN_Init(region, subBand, roundInterval, uplinkInterval, initialDatarate, ADRActive,
                                                CSMAActive,  CSMAMaxChanges, CSMABackoffMax, CSMADifsSlots,
                                                dutyCycleLimitsActive, dutyCycleMsPerHour, dwellTimeLimitsActive, dwellTimeMsPerUplink,
                                                deviceActivationMethod, 
                                                joinEUI, devEUI, nwkKey, appKey, 
                                                devAddr, fNwkSIntKey, sNwkSIntKey, nwkSEncKey, appSKey);

    if (LoRaWANInitializeCheck != 0) {
        return false;
    }

    LoRaWANenable = true;

    return true;
}

bool ClassFlowLoRaWAN::doFlow(string zwtime)
{
    if (!LoRaWANenable)
        return true;

    std::string result;
    std::string resulterror = "";
    uint8_t resulterrorCode = 0;
    std::string resultraw = "";
    std::string resultpre = "";
    std::string resultrate = ""; // Always Unit / Minute
    std::string resultRatePerTimeUnit = ""; // According to selection
    std::string resultchangabs = "";
    std::string resulttimestamp = "";        
    time_t resulttimeutc = 0;
    string namenumber = "";
    double resultValue = -1;
    double resultrateValue = -1;

    if (flowpostprocessing && getLoRaWANisSessionActive())
    {
        std::vector<NumberPost*>* NUMBERS = flowpostprocessing->GetNumbers();

        for (int i = 0; i < (*NUMBERS).size(); ++i)
        {
            result =  (*NUMBERS)[i]->ReturnValue;
            resultraw =  (*NUMBERS)[i]->ReturnRawValue;
            resulterror = (*NUMBERS)[i]->ErrorMessageText;
            resultrate = (*NUMBERS)[i]->ReturnRateValue; // Unit per minutes
            resulttimeutc = (*NUMBERS)[i]->timeStampTimeUTC;

            namenumber = (*NUMBERS)[i]->name;

            if (result == "") {
                resultValue = -1;
            } else {
                resultValue = std::stod(result);
            }

            if (resultrate == "") {
                resultrateValue = -1;
            } else {
                resultrateValue = std::stod(resultrate);
            }

            if (resulterror == "no error") {
                resulterrorCode = 0;
            } else {
                if (resulterror.rfind("Neg. Rate", 0) == 0){
                    resulterrorCode = 1;
                }
                if (resulterror.rfind("Rate too high", 0) == 0){
                    resulterrorCode = 2;
                }
            }

            if (result.length() > 0)  {} 
                LoRaWANQueueMessage(i, resulttimeutc, resultValue, resultrateValue, resulterrorCode);
        }
    }
    
    OldValue = result;
    
    return true;
}

#endif //ENABLE_LORAWAN