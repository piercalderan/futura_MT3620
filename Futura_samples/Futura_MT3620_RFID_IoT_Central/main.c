// Futura MT3620 RFID IoT Central example.
// Copyright 2020 Pier Calderan.
// Apre il pulsante 1 come ingresso.
// Apre il sensore RFID.
// Premendo il pulsante 1 si mandano i dati di telemetria RFID a IoT Central.

#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <applibs/i2c.h>
#include "map.h" // Map Datastructure

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/uart.h>
#include <applibs/networking.h>
#include <applibs/gpio.h>
#include <applibs/storage.h>
#include <applibs/eventloop.h>

//CUSTOM libs
#include "mfrc522.h"
//HARDWARE DEFINITION
#include <hw/sample_hardware.h>
//IOTHUB libs
#include "eventloop_timer_utilities.h"
#include <iothub_client_core_common.h>
#include <iothub_device_client_ll.h>
#include <iothub_client_options.h>
#include <iothubtransportmqtt.h>
#include <iothub.h>
#include <azure_sphere_provisioning.h>
#include "parson.h" // used to parse Device Twin messages.

uint8_t str_rfid[MAX_LEN]; //card ID
uint8_t str_dump[10]; //card dump
void bytetohex(char* xp, const char* bb, int n); // Convert the byte array to a string of bytes
map_str_t priceMap; // map for RFID PICC

static char eventBuffer[100] = { 0 };

// Convert the byte array to a string of bytes
void bytetohex(char* xp, const char* bb, int n)
{
    const char xx[] = "0123456789ABCDEF";
    while (--n >= 0) xp[n] = xx[(bb[n >> 1] >> ((1 - (n & 1)) << 2)) & 0xF];
}

void delay(int s)
{
    sleep(s);
}

static void RFIDTimerEventHandler(void)
{
    // Prepare for reading tags
    uint8_t byte;
    byte = mfrc522_read(ComIEnReg);
    mfrc522_write(ComIEnReg, byte | 0x20);
    byte = mfrc522_read(DivIEnReg);
    mfrc522_write(DivIEnReg, byte | 0x80);

    // Try to read the RFID card    
    byte = mfrc522_request(PICC_REQALL, str_rfid); // Find ID

    if (byte == CARD_FOUND)
    {
        Log_Debug("[MFRC522][INFO] Found a card: %x\n", byte);

        byte = mfrc522_get_card_serial(str_rfid);
        
        if (byte == CARD_FOUND)
        {            
            Log_Debug("Card dumping: "); 
            for (byte = 0; byte < 8; byte++)
            {                
                Log_Debug("%x", str_rfid[byte]); // dump ID in byte array
            }
            // Convert the byte array to a string of bytes
            char hexstr[8];
            bytetohex(hexstr, str_rfid, 8);
            hexstr[8] = 0;
            Log_Debug("\nhex: %x\n", hexstr);
            int* val = map_get(&priceMap, hexstr);
            Log_Debug("[MFRC522][INFO] Serial: %s\n", hexstr);
            Log_Debug("[MAP][INFO] Map Price: %s\n", *val);

            //static char eventBuffer[100] = { 0 };
            static const char* EventMsgTemplate = "%s";
            int len = snprintf(eventBuffer, sizeof(eventBuffer), EventMsgTemplate, *val);
            Log_Debug("[IoTCentral][INFO] Sending IoT Central Message: %s\n", eventBuffer);
        }
    }
}

//ExitCode enum
typedef enum {
    ExitCode_Success = 0,
    ExitCode_TermHandler_SigTerm = 1,
    ExitCode_Main_EventLoopFail = 2,
    ExitCode_ButtonTimer_Consume = 3,
    ExitCode_AzureTimer_Consume = 4,
    ExitCode_Init_EventLoop = 5,
    ExitCode_Init_MessageButton = 6,
    ExitCode_Init_OrientationButton = 7,
    ExitCode_Init_TwinStatusLed = 8,
    ExitCode_Init_ButtonPollTimer = 9,
    ExitCode_Init_AzureTimer = 10,
    ExitCode_IsButtonPressed_GetValue = 11,    
    ExitCode_Init_OpenMaster = 12,
    ExitCode_Init_SetBusSpeed = 13,
    ExitCode_Init_SetTimeout = 14,
    ExitCode_Init_SetDefaultTarget = 15,
    ExitCode_Main_Led = 16,    
    ExitCode_Init_RegisterIo = 17,
} ExitCode;

// function declarations
static volatile sig_atomic_t exitCode = ExitCode_Success;
static void TerminationHandler(int signalNumber);

// Azure IoT Central/Central defines.
#define SCOPEID_LENGTH 20
static char scopeId[SCOPEID_LENGTH]; // ScopeId for the Azure IoT Central application, set in app_manifest.json, CmdArgs
static IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle = NULL;
static const int keepalivePeriodSeconds = 20;
static bool iothubAuthenticated = false;
static void sendRFIDCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context);
static void TwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payload,size_t payloadSize, void *userContextCallback);
static void TwinReportBoolState(const char *propertyName, bool propertyValue);
static void ReportStatusCallback(int result, void *context);
static const char *GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason);
static const char *getAzureSphereProvisioningResultString(AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult);
static void SendTelemetry(const unsigned char *key, const unsigned char *value);
static void SetupAzureClient(void);

// Initialization/Cleanup
static ExitCode InitPeripheralsAndHandlers(void);
static void CloseFdAndPrintError(int fd, const char *fdName);
static void ClosePeripheralsAndHandlers(void);

// File descriptors - initialized to invalid value Buttons
static int sendRFIDButtonGpioFd = -1;

// LED
static int deviceTwinStatusLedGpioFd = -1;

// Timer / polling
static EventLoop *eventLoop = NULL;
static EventLoopTimer *buttonPollTimer = NULL;
static EventLoopTimer *azureTimer = NULL;

// Azure IoT poll periods
static const int AzureIoTDefaultPollPeriodSeconds = 5;
static const int AzureIoTMinReconnectPeriodSeconds = 60;
static const int AzureIoTMaxReconnectPeriodSeconds = 10 * 60;
static int azureIoTPollPeriodSeconds = -1;

// Button state variables
static GPIO_Value_Type sendRFIDButtonState = GPIO_Value_High;

static void ButtonPollTimerEventHandler(EventLoopTimer *timer);
static bool IsButtonPressed(int fd, GPIO_Value_Type *oldState);
static void sendRFIDButtonHandler(void);


static bool deviceIsUp = false; // Orientation
static void AzureTimerEventHandler(EventLoopTimer *timer);

// Signal handler for termination requests. This handler must be async-signal-safe.
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    exitCode = ExitCode_TermHandler_SigTerm;
}

// Main entry point 
int main(int argc, char *argv[])
{
            
    map_init(&priceMap);
    map_set(&priceMap, "291C0EA3", "Model 1: €100");
    map_set(&priceMap, "D098C71A", "Model 2: €200");
    map_set(&priceMap, "31AC6A1C", "Model 3: €300");

    // Start the RFID Scanner
    if (mfrc522_init())
    {
        Log_Debug("RFID Scanner found!\n");
    }
    else
    {
        Log_Debug("RFID Scanner not found!\n");
        return -1;
    }

    Log_Debug("RFID application starting.\n");

    bool isNetworkingReady = false;
    if ((Networking_IsNetworkingReady(&isNetworkingReady) == -1) || !isNetworkingReady) {
        Log_Debug("WARNING: Network is not ready. Device cannot connect until network is ready.\n");
    }

    if (argc == 2) {
        Log_Debug("Setting Azure Scope ID %s\n", argv[1]);
        strncpy(scopeId, argv[1], SCOPEID_LENGTH);
    } else {
        Log_Debug("ScopeId needs to be set in the app_manifest CmdArgs\n");
        return -1;
    }

    exitCode = InitPeripheralsAndHandlers();

    Log_Debug("Trying to get version\n");
    uint8_t byte = mfrc522_read(VersionReg);
    Log_Debug("Detected version %d (Hex: %x)\n", byte, byte); // RFID Version must be 0x92
    delay(2);
    RFIDTimerEventHandler();
    delay(2);

    // Main loop
    while (exitCode == ExitCode_Success) {
        EventLoop_Run_Result result = EventLoop_Run(eventLoop, -1, true);
        // Continue if interrupted by signal, e.g. due to breakpoint being set.
        if (result == EventLoop_Run_Failed && errno != EINTR) {
            exitCode = ExitCode_Main_EventLoopFail; 
        }
    }

    ClosePeripheralsAndHandlers();
    Log_Debug("Application exiting.\n");
    return exitCode;
}


// Button timer event:  Check the status of buttons 1 2 3

static void ButtonPollTimerEventHandler(EventLoopTimer *timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_ButtonTimer_Consume;
        return;
    }
    sendRFIDButtonHandler();        
}

// // // // // // // // // // // // // // // // // // 
// Azure timer event:  Check connection status and send telemetry
// // // // // // // // // // // // // // // // // // 
static void AzureTimerEventHandler(EventLoopTimer *timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_AzureTimer_Consume;
        return;
    }

    bool isNetworkReady = false;
    if (Networking_IsNetworkingReady(&isNetworkReady) != -1) {
        if (isNetworkReady && !iothubAuthenticated) {
            SetupAzureClient();
        }
    } else {
        Log_Debug("Failed to get Network state\n");
    }

    if (iothubAuthenticated) {        
        IoTHubDeviceClient_LL_DoWork(iothubClientHandle);
    }
}


//     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
static ExitCode InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    eventLoop = EventLoop_Create();
    if (eventLoop == NULL) {
        Log_Debug("Could not create event loop.\n");
        return ExitCode_Init_EventLoop;
    }

    
    // Open button 1 GPIO as input
    Log_Debug("Opening SAMPLE_BUTTON_1 as input\n");
    sendRFIDButtonGpioFd = GPIO_OpenAsInput(SAMPLE_BUTTON_1);
    if (sendRFIDButtonGpioFd < 0) {
        Log_Debug("ERROR: Could not open button 1: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_MessageButton;
    }



    // Set up a timer to poll for button events
    static const struct timespec buttonPressCheckPeriod = {.tv_sec = 0, .tv_nsec = 1000 * 1000};
    buttonPollTimer = CreateEventLoopPeriodicTimer(eventLoop, &ButtonPollTimerEventHandler,
                                                   &buttonPressCheckPeriod);
    if (buttonPollTimer == NULL) {
        return ExitCode_Init_ButtonPollTimer;
    }

    azureIoTPollPeriodSeconds = AzureIoTDefaultPollPeriodSeconds;
    struct timespec azureTelemetryPeriod = {.tv_sec = azureIoTPollPeriodSeconds, .tv_nsec = 0};
    azureTimer =
        CreateEventLoopPeriodicTimer(eventLoop, &AzureTimerEventHandler, &azureTelemetryPeriod);
    if (azureTimer == NULL) {
        return ExitCode_Init_AzureTimer;
    }

    
    return ExitCode_Success;
}


//     Closes a file descriptor and prints an error on failure.
static void CloseFdAndPrintError(int fd, const char *fdName)
{
    if (fd >= 0) {
        int result = close(fd);
        if (result != 0) {
            Log_Debug("ERROR: Could not close fd %s: %s (%d).\n", fdName, strerror(errno), errno);
        }
    }
}


//     Close peripherals and handlers.
static void ClosePeripheralsAndHandlers(void)
{
    DisposeEventLoopTimer(buttonPollTimer);
    DisposeEventLoopTimer(azureTimer);
    EventLoop_Close(eventLoop);

    Log_Debug("Closing file descriptors\n");
        
    
    CloseFdAndPrintError(sendRFIDButtonGpioFd, "sendRFIDButton");
    
}


//     Sets the IoT Central authentication state for the app
//     The SAS Token expires which will set the authentication state

static void HubConnectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result,
                                        IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason,
                                        void *userContextCallback)
{
    iothubAuthenticated = (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED);
    Log_Debug("IoT Central Authenticated: %s\n", GetReasonString(reason));
}


//     Sets up the Azure IoT Central connection (creates the iothubClientHandle)
//     When the SAS Token for a device expires the connection needs to be recreated
//     which is why this is not simply a one time call.

static void SetupAzureClient(void)
{
    if (iothubClientHandle != NULL) {
        IoTHubDeviceClient_LL_Destroy(iothubClientHandle);
    }

    AZURE_SPHERE_PROV_RETURN_VALUE provResult =
        IoTHubDeviceClient_LL_CreateWithAzureSphereDeviceAuthProvisioning(scopeId, 10000,
                                                                          &iothubClientHandle);
    Log_Debug("IoTHubDeviceClient_LL_CreateWithAzureSphereDeviceAuthProvisioning returned '%s'.\n",
              getAzureSphereProvisioningResultString(provResult));

    if (provResult.result != AZURE_SPHERE_PROV_RESULT_OK) {

        // If fail to connect, reduce the polling frequency, starting at
        // AzureIoTMinReconnectPeriodSeconds and with a backoff up to
        // AzureIoTMaxReconnectPeriodSeconds
        if (azureIoTPollPeriodSeconds == AzureIoTDefaultPollPeriodSeconds) {
            azureIoTPollPeriodSeconds = AzureIoTMinReconnectPeriodSeconds;
        } else {
            azureIoTPollPeriodSeconds *= 2;
            if (azureIoTPollPeriodSeconds > AzureIoTMaxReconnectPeriodSeconds) {
                azureIoTPollPeriodSeconds = AzureIoTMaxReconnectPeriodSeconds;
            }
        }

        struct timespec azureTelemetryPeriod = {azureIoTPollPeriodSeconds, 0};
        SetEventLoopTimerPeriod(azureTimer, &azureTelemetryPeriod);

        Log_Debug("ERROR: failure to create IoTHub Handle - will retry in %i seconds.\n",
                  azureIoTPollPeriodSeconds);
        return;
    }

    // Successfully connected, so make sure the polling frequency is back to the default
    azureIoTPollPeriodSeconds = AzureIoTDefaultPollPeriodSeconds;
    struct timespec azureTelemetryPeriod = {.tv_sec = azureIoTPollPeriodSeconds, .tv_nsec = 0};
    SetEventLoopTimerPeriod(azureTimer, &azureTelemetryPeriod);

    iothubAuthenticated = true;

    if (IoTHubDeviceClient_LL_SetOption(iothubClientHandle, OPTION_KEEP_ALIVE,
                                        &keepalivePeriodSeconds) != IOTHUB_CLIENT_OK) {
        Log_Debug("ERROR: failure setting option \"%s\"\n", OPTION_KEEP_ALIVE);
        return;
    }

    IoTHubDeviceClient_LL_SetDeviceTwinCallback(iothubClientHandle, TwinCallback, NULL);
    IoTHubDeviceClient_LL_SetConnectionStatusCallback(iothubClientHandle,
                                                      HubConnectionStatusCallback, NULL);
}


//     Callback invoked when a Device Twin update is received from IoT Central.
//     Updates local state for 'showEvents' (bool).
// <param name="payload">contains the Device Twin JSON document (desired and reported)</param>
// <param name="payloadSize">size of the Device Twin JSON document</param>
static void TwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payload,
                         size_t payloadSize, void *userContextCallback)
{
    size_t nullTerminatedJsonSize = payloadSize + 1;
    char *nullTerminatedJsonString = (char *)malloc(nullTerminatedJsonSize);
    if (nullTerminatedJsonString == NULL) {
        Log_Debug("ERROR: Could not allocate buffer for twin update payload.\n");
        abort();
    }

    // Copy the provided buffer to a null terminated buffer.
    memcpy(nullTerminatedJsonString, payload, payloadSize);
    // Add the null terminator at the end.
    nullTerminatedJsonString[nullTerminatedJsonSize - 1] = 0;

    JSON_Value *rootProperties = NULL;
    rootProperties = json_parse_string(nullTerminatedJsonString);
    if (rootProperties == NULL) {
        Log_Debug("WARNING: Cannot parse the string as JSON content.\n");
        goto cleanup;
    }

    JSON_Object *rootObject = json_value_get_object(rootProperties);
    JSON_Object *desiredProperties = json_object_dotget_object(rootObject, "desired");
    if (desiredProperties == NULL) {
        desiredProperties = rootObject;
    }


cleanup:
    // Release the allocated memory.
    json_value_free(rootProperties);
    free(nullTerminatedJsonString);
}

//     Converts the IoT Central connection status reason to a string.
static const char *GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason)
{
    static char *reasonString = "unknown reason";
    switch (reason) {
    case IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN:
        reasonString = "IOTHUB_CLIENT_CONNECTION_EXPIRED_SAS_TOKEN";
        break;
    case IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED:
        reasonString = "IOTHUB_CLIENT_CONNECTION_DEVICE_DISABLED";
        break;
    case IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL:
        reasonString = "IOTHUB_CLIENT_CONNECTION_BAD_CREDENTIAL";
        break;
    case IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED:
        reasonString = "IOTHUB_CLIENT_CONNECTION_RETRY_EXPIRED";
        break;
    case IOTHUB_CLIENT_CONNECTION_NO_NETWORK:
        reasonString = "IOTHUB_CLIENT_CONNECTION_NO_NETWORK";
        break;
    case IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR:
        reasonString = "IOTHUB_CLIENT_CONNECTION_COMMUNICATION_ERROR";
        break;
    case IOTHUB_CLIENT_CONNECTION_OK:
        reasonString = "IOTHUB_CLIENT_CONNECTION_OK"; // must be always this
        break;
    }
    return reasonString;
}


//     Converts AZURE_SPHERE_PROV_RETURN_VALUE to a string.
static const char *getAzureSphereProvisioningResultString(
    AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult)
{
    switch (provisioningResult.result) {
    case AZURE_SPHERE_PROV_RESULT_OK:
        return "AZURE_SPHERE_PROV_RESULT_OK"; // must be this
    case AZURE_SPHERE_PROV_RESULT_INVALID_PARAM:
        return "AZURE_SPHERE_PROV_RESULT_INVALID_PARAM";
    case AZURE_SPHERE_PROV_RESULT_NETWORK_NOT_READY:
        return "AZURE_SPHERE_PROV_RESULT_NETWORK_NOT_READY";
    case AZURE_SPHERE_PROV_RESULT_DEVICEAUTH_NOT_READY:
        return "AZURE_SPHERE_PROV_RESULT_DEVICEAUTH_NOT_READY";
    case AZURE_SPHERE_PROV_RESULT_PROV_DEVICE_ERROR:
        return "AZURE_SPHERE_PROV_RESULT_PROV_DEVICE_ERROR";
    case AZURE_SPHERE_PROV_RESULT_GENERIC_ERROR:
        return "AZURE_SPHERE_PROV_RESULT_GENERIC_ERROR";
    default:
        return "UNKNOWN_RETURN_VALUE";
    }
}


//     Sends telemetry to IoT Central
// <param name="key">The telemetry item to update</param>
// <param name="value">new telemetry value</param>
static void SendTelemetry(const unsigned char *key, const unsigned char *value)
{
    static char eventBuffer[100] = {0};
    static const char *EventMsgTemplate = "{ \"%s\": \"%s\" }";
    int len = snprintf(eventBuffer, sizeof(eventBuffer), EventMsgTemplate, key, value);
    if (len < 0)
        return;

    Log_Debug("Sending IoT Central Message: %s\n", eventBuffer);

    bool isNetworkingReady = false;

    if ((Networking_IsNetworkingReady(&isNetworkingReady) == -1) || !isNetworkingReady) {
        Log_Debug("WARNING: Cannot send IoTHubMessage because network is not up.\n");
        return;
    }

    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(eventBuffer);

    if (messageHandle == 0) {
        Log_Debug("WARNING: unable to create a new IoTHubMessage\n");
        return;
    }

    if (IoTHubDeviceClient_LL_SendEventAsync(iothubClientHandle, messageHandle, sendRFIDCallback,
                                             /*&callback_param*/ 0) != IOTHUB_CLIENT_OK) {
        Log_Debug("WARNING: failed to hand over the message to IoTHubClient\n");
    } else {
        Log_Debug("INFO: IoTHubClient accepted the message for delivery\n");
    }
    
    IoTHubMessage_Destroy(messageHandle);
}


//     Callback confirming message delivered to IoT Central.

// <param name="result">Message delivery status</param>
// <param name="context">User specified context</param>
static void sendRFIDCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context)
{
    Log_Debug("INFO: Message received by IoT Central. Result is: %d\n", result);
}


//     Creates and enqueues a report containing the name and value pair of a Device Twin reported
//     property. The report is not sent immediately, but it is sent on the next invocation of
//     IoTHubDeviceClient_LL_DoWork().
// <param name="propertyName">the IoT Central Device Twin property name</param>
// <param name="propertyValue">the IoT Central Device Twin property value</param>
static void TwinReportBoolState(const char *propertyName, bool propertyValue)
{
    if (iothubClientHandle == NULL) {
        Log_Debug("ERROR: client not initialized\n");
    } else {
        static char reportedPropertiesString[30] = {0};
        int len = snprintf(reportedPropertiesString, 30, "{\"%s\":%s}", propertyName,
                           (propertyValue == true ? "true" : "false"));
        if (len < 0)
            return;

        if (IoTHubDeviceClient_LL_SendReportedState(
                iothubClientHandle, (unsigned char *)reportedPropertiesString,
                strlen(reportedPropertiesString), ReportStatusCallback, 0) != IOTHUB_CLIENT_OK) {
            Log_Debug("ERROR: failed to set reported state for '%s'.\n", propertyName);
        } else {
            Log_Debug("INFO: Reported state for '%s' to value '%s'.\n", propertyName,
                      (propertyValue == true ? "true" : "false"));
        }
    }
}


//     Callback invoked when the Device Twin reported properties are accepted by IoT Central.

static void ReportStatusCallback(int result, void *context)
{
    Log_Debug("INFO: Device Twin reported properties update result: HTTP status code %d\n", result);
}



// Check button been pressed.
// <param name="fd">The button file descriptor</param>
// <param name="oldState">Old state of the button (pressed or released)</param>
// <returns>true if pressed, false otherwise</returns>
static bool IsButtonPressed(int fd, GPIO_Value_Type *oldState)
{
    bool isButtonPressed = false;
    GPIO_Value_Type newState;
    int result = GPIO_GetValue(fd, &newState);
    if (result != 0) {
        Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
        exitCode = ExitCode_IsButtonPressed_GetValue;
    } else {
        // Button is pressed if it is low and different than last known state.
        isButtonPressed = (newState != *oldState) && (newState == GPIO_Value_Low);
        *oldState = newState;
    }
    return isButtonPressed;
}


// Pressing button 1 will send RFID event to Azure IoT Central
static void sendRFIDButtonHandler(void)
{
    if (IsButtonPressed(sendRFIDButtonGpioFd, &sendRFIDButtonState)) {       
        RFIDTimerEventHandler();
        SendTelemetry("RFID", eventBuffer); //RFID CARD CONTENT                
    }
}


