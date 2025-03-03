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
      nwkKey[16] = NULL;
      appKey[16] = NULL;
      // ABP parameters
      devAddr = NULL;
      fNwkSIntKey[16] = NULL;
      sNwkSIntKey[16] = NULL;
      nwkSEncKey[16] = NULL;
      appSKey[16] = NULL;
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

bool ClassFlowLoRaWAN::ReadParameter(FILE* pfile, string& aktparamgraph)
{
    std::vector<string> splitted;

    aktparamgraph = trim(aktparamgraph);
    printf("akt param: %s\n", aktparamgraph.c_str());

    if (aktparamgraph.size() == 0)
        if (!this->GetNextParagraph(pfile, aktparamgraph))
            return false;

    if (toUpper(aktparamgraph).compare("[LoRaWAN]") != 0)
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
            if (splitted[1] == "CN500")
                region = CN500;
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
            this->joinEUI = std::strtoull(splitted[1].c_str(),NULL,16);
        }
        if ((toUpper(_param) == "DEVEUI") && (splitted.size() > 1))
        {
            this->devEUI = std::strtoull(splitted[1].c_str(),NULL,16);
        }
        if ((toUpper(_param) == "NWKKEY") && (splitted.size() > 1))
        {
            std::vector<uint8_t> nwkKeyVector = hexStringToByteArray(splitted[1]);
            copy(nwkKeyVector.begin(), nwkKeyVector.end(), this->nwkKey);
        }
        if ((toUpper(_param) == "APPKEY") && (splitted.size() > 1))
        {
            std::vector<uint8_t> appKeyVector = hexStringToByteArray(splitted[1]);
            copy(appKeyVector.begin(), appKeyVector.end(), this->appKey);
        }
        // ABP parameters
        if ((toUpper(_param) == "DEVADDR") && (splitted.size() > 1))
        {
            this->devAddr = std::strtoull(splitted[1].c_str(),NULL,16);
        }
        if ((toUpper(_param) == "FNWKSINTKEY") && (splitted.size() > 1))
        {
            std::vector<uint8_t> fNwkSIntKeyVector = hexStringToByteArray(splitted[1]);
            copy(fNwkSIntKeyVector.begin(), fNwkSIntKeyVector.end(), this->fNwkSIntKey);
        }
        if ((toUpper(_param) == "SNWKSINTKEY") && (splitted.size() > 1))
        {
            std::vector<uint8_t> sNwkSIntKeyVector = hexStringToByteArray(splitted[1]);
            copy(sNwkSIntKeyVector.begin(), sNwkSIntKeyVector.end(), this->sNwkSIntKey);
        }
        if ((toUpper(_param) == "NWKSENCKEY") && (splitted.size() > 1))
        {
            std::vector<uint8_t> nwkSEncKeyVector = hexStringToByteArray(splitted[1]);
            copy(nwkSEncKeyVector.begin(), nwkSEncKeyVector.end(), this->nwkSEncKey);
        }
        if ((toUpper(_param) == "APPSKEY") && (splitted.size() > 1))
        {
            std::vector<uint8_t> appSKeyVector = hexStringToByteArray(splitted[1]);
            copy(appSKeyVector.begin(), appSKeyVector.end(), this->appSKey);
        }
    }

    return true;
}

bool ClassFlowLoRaWAN::Start(float AutoInterval) 
{
    // std::stringstream stream;
    // stream << std::fixed << std::setprecision(1) << "Digitizer interval is " << roundInterval <<
    //         " minutes => setting MQTT LWT timeout to " << ((float)keepAlive/60) << " minutes.";
    // LogFile.WriteToFile(ESP_LOG_DEBUG, TAG, stream.str());

    // mqttServer_setParameter(flowpostprocessing->GetNumbers(), keepAlive, roundInterval);

    // bool MQTTConfigCheck = MQTT_Configure(uri, clientname, user, password, maintopic, LWT_TOPIC, LWT_CONNECTED,
    //                                  LWT_DISCONNECTED, caCertFilename, clientCertFilename, clientKeyFilename,
    //                                  keepAlive, SetRetainFlag, (void *)&GotConnected);

    // if (!MQTTConfigCheck) {
    //     return false;
    // }

    // return (MQTT_Init() == 1);
    uint16_t LoRaWANInitializeCheck = LoRaWAN_Init(region, subBand, uplinkInterval, initialDatarate, ADRActive,
                                                CSMAActive,  CSMAMaxChanges, CSMABackoffMax, CSMADifsSlots,
                                                dutyCycleLimitsActive, dutyCycleMsPerHour, dwellTimeLimitsActive, dwellTimeMsPerUplink,
                                                deviceActivationMethod, 
                                                joinEUI, devEUI, nwkKey, appKey, 
                                                devAddr, fNwkSIntKey, sNwkSIntKey, nwkSEncKey, appSKey);
    return true;
}

bool ClassFlowLoRaWAN::doFlow(string zwtime)
{
    bool success;
    std::string result;
    std::string resulterror = "";
    std::string resultraw = "";
    std::string resultpre = "";
    std::string resultrate = ""; // Always Unit / Minute
    std::string resultRatePerTimeUnit = ""; // According to selection
    std::string resulttimestamp = "";
    std::string resultchangabs = "";
    string zw = "";
    string namenumber = "";
    int qos = 1;

    /* Send the the Homeassistant Discovery and the Static Topics in case they where scheduled */
    // sendDiscovery_and_static_Topics();

    // success = publishSystemData(qos);

    // if (flowpostprocessing && getMQTTisConnected())
    // {
    //     std::vector<NumberPost*>* NUMBERS = flowpostprocessing->GetNumbers();

    //     LogFile.WriteToFile(ESP_LOG_DEBUG, TAG, "Publishing MQTT topics...");

    //     for (int i = 0; i < (*NUMBERS).size(); ++i)
    //     {
    //         result =  (*NUMBERS)[i]->ReturnValue;
    //         resultraw =  (*NUMBERS)[i]->ReturnRawValue;
    //         resultpre =  (*NUMBERS)[i]->ReturnPreValue;
    //         resulterror = (*NUMBERS)[i]->ErrorMessageText;
    //         resultrate = (*NUMBERS)[i]->ReturnRateValue; // Unit per minutes
    //         resultchangabs = (*NUMBERS)[i]->ReturnChangeAbsolute; // Units per round
    //         resulttimestamp = (*NUMBERS)[i]->timeStamp;

    //         namenumber = (*NUMBERS)[i]->name;
    //         if (namenumber == "default")
    //             namenumber = maintopic + "/";
    //         else
    //             namenumber = maintopic + "/" + namenumber + "/";


    //         if (result.length() > 0)   
    //             success |= MQTTPublish(namenumber + "value", result, qos, SetRetainFlag);

    //         if (resulterror.length() > 0)  
    //             success |= MQTTPublish(namenumber + "error", resulterror, qos, SetRetainFlag);

    //         if (resultrate.length() > 0) {
    //             success |= MQTTPublish(namenumber + "rate", resultrate, qos, SetRetainFlag);
                
    //             std::string resultRatePerTimeUnit;
    //             if (getTimeUnit() == "h") { // Need conversion to be per hour
    //                 resultRatePerTimeUnit = resultRatePerTimeUnit = to_string((*NUMBERS)[i]->FlowRateAct * 60); // per minutes => per hour
    //             }
    //             else { // Keep per minute
    //                 resultRatePerTimeUnit = resultrate;
    //             }
    //             success |= MQTTPublish(namenumber + "rate_per_time_unit", resultRatePerTimeUnit, qos, SetRetainFlag);
    //         }

    //         if (resultchangabs.length() > 0) {
    //             success |= MQTTPublish(namenumber + "changeabsolut", resultchangabs, qos, SetRetainFlag); // Legacy API
    //             success |= MQTTPublish(namenumber + "rate_per_digitalization_round", resultchangabs, qos, SetRetainFlag);
    //         }

    //         if (resultraw.length() > 0)   
    //             success |= MQTTPublish(namenumber + "raw", resultraw, qos, SetRetainFlag);

    //         if (resulttimestamp.length() > 0)
    //             success |= MQTTPublish(namenumber + "timestamp", resulttimestamp, qos, SetRetainFlag);

    //         std::string json = flowpostprocessing->getJsonFromNumber(i, "\n");
    //         success |= MQTTPublish(namenumber + "json", json, qos, SetRetainFlag);
    //     }
    // }
    
    // OldValue = result;

    // if (!success) {
    //     LogFile.WriteToFile(ESP_LOG_WARN, TAG, "One or more MQTT topics failed to be published!");
    // }
    
    return true;
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

#endif //ENABLE_LORAWAN