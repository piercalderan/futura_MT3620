#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <applibs/log.h>
#include <applibs/application.h>
#include "eventloop_timer_utilities.h"
#include <stdlib.h>
#include <stdarg.h>

#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/uart.h>
#include <applibs/networking.h>
#include <applibs/gpio.h>
#include <applibs/storage.h>
#include <applibs/eventloop.h>

//HARDWARE DEFINITION
#include <hw/sample_hardware.h>

//IOTHUB libs
#include <iothub_client_core_common.h>
#include <iothub_device_client_ll.h>
#include <iothub_client_options.h>
#include <iothubtransportmqtt.h>
#include <iothub.h>
#include <azure_sphere_provisioning.h>
#include "parson.h" // used to parse Device Twin messages.

static char eventBuffer[100] = { 0 };
// Timer 
static EventLoopTimer* azureTimer = NULL;

// Azure IoT poll periods
static const int AzureIoTDefaultPollPeriodSeconds = 5;
static const int AzureIoTMinReconnectPeriodSeconds = 60;
static const int AzureIoTMaxReconnectPeriodSeconds = 10 * 60;

static int azureIoTPollPeriodSeconds = -1;



static void AzureTimerEventHandler(EventLoopTimer* timer);


typedef enum {
    ExitCode_Success = 0,
    ExitCode_TermHandler_SigTerm = 1,
    ExitCode_TimerHandler_Consume = 2,
    ExitCode_SendMsg_Send = 3,
    ExitCode_SocketHandler_Recv = 4,
    ExitCode_Init_EventLoop = 5,
    ExitCode_Init_SendTimer = 6,
    ExitCode_Init_Connection = 7,
    ExitCode_Init_SetSockOpt = 8,
    ExitCode_Init_RegisterIo = 9,
    ExitCode_Main_EventLoopFail = 10,    
    ExitCode_AzureTimer_Consume = 12,  
    ExitCode_Init_TwinStatusLed = 15,    
    ExitCode_Init_AzureTimer = 17,
} ExitCode;

static int sockFd = -1;
static EventLoop* eventLoop = NULL;
static EventLoopTimer* sendTimer = NULL;
static EventRegistration* socketEventReg = NULL;
static volatile sig_atomic_t exitCode = ExitCode_Success;

static const char rtAppComponentId[] = "005180bc-402f-4cb3-a662-72937dbcde47";

static void TerminationHandler(int signalNumber);
static void SendTimerEventHandler(EventLoopTimer* timer);
static void SendMessageToRTApp(void);
static void SocketEventHandler(EventLoop* el, int fd, EventLoop_IoEvents events, void* context);
static ExitCode InitHandlers(void);
static void CloseHandlers(void);
// Azure IoT Central/Central defines.
#define SCOPEID_LENGTH 20
static char scopeId[SCOPEID_LENGTH]; // ScopeId for the Azure IoT Central application, set in app_manifest.json, CmdArgs
static IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle = NULL;
static const int keepalivePeriodSeconds = 20;
static bool iothubAuthenticated = false;
static void SendMessageCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* context);
static void TwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char* payload,
    size_t payloadSize, void* userContextCallback);
static void TwinReportBoolState(const char* propertyName, bool propertyValue);
static void ReportStatusCallback(int result, void* context);
static const char* GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason);
static const char* getAzureSphereProvisioningResultString(
    AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult);
static void SendTelemetry(const unsigned char* key, const unsigned char* value);
static void SetupAzureClient(void);
/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    exitCode = ExitCode_TermHandler_SigTerm;
}

/// <summary>
///     Handle send timer event by writing data to the real-time capable application.
/// </summary>
static void SendTimerEventHandler(EventLoopTimer* timer)
{
    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_TimerHandler_Consume;
        return;
    }

    SendMessageToRTApp();
}

/// <summary>
///     Helper function for TimerEventHandler sends message to real-time capable application.
/// </summary>
static void SendMessageToRTApp(void)
{
    // Send "hl-app-to-rt-app-%02d" message to RTApp, where the number cycles from 00 to 99.
    static int iter = 0;

    static char txMessage[32];
    snprintf(txMessage, sizeof(txMessage), "hl-app-to-rt-app-%02d", iter);
    iter = (iter + 1) % 100;
    Log_Debug("Sending: %s\n", txMessage);

    int bytesSent = send(sockFd, txMessage, strlen(txMessage), 0);
    if (bytesSent == -1) {
        Log_Debug("ERROR: Unable to send message: %d (%s)\n", errno, strerror(errno));
        exitCode = ExitCode_SendMsg_Send;
        return;
    }
}

/// <summary>
///     Handle socket event by reading incoming data from real-time capable application.
/// </summary>
static void SocketEventHandler(EventLoop* el, int fd, EventLoop_IoEvents events, void* context)
{
    static int iter = 0;
    const size_t receiveBufferSize = 256;
    uint8_t receiveBuffer[receiveBufferSize + 1]; // allow extra byte for string termination
    int bytesRead;

    // Read incoming RTA data 
    bytesRead = recv(fd, receiveBuffer, sizeof(receiveBuffer), 0);

    if (bytesRead > 0) {
        // print and sendtelemetry
        receiveBuffer[bytesRead] = 0;
        Log_Debug("RTA received %d bytes: '%s'.\n", bytesRead, (char*)receiveBuffer);
        SendTelemetry("RTA", (char*)receiveBuffer);
        
    }
    Log_Debug("\n");
}

/// <summary>
///     Set up SIGTERM termination handler and event handlers for send timer
///     and to receive data from real-time capable application.
/// </summary>
/// <returns>
///     ExitCode_Success if all resources were allocated successfully; otherwise another
///     ExitCode value which indicates the specific failure.
/// </returns>
static ExitCode InitHandlers(void)
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

    azureIoTPollPeriodSeconds = AzureIoTDefaultPollPeriodSeconds;
    struct timespec azureTelemetryPeriod = { .tv_sec = azureIoTPollPeriodSeconds, .tv_nsec = 0 };
    azureTimer =
        CreateEventLoopPeriodicTimer(eventLoop, &AzureTimerEventHandler, &azureTelemetryPeriod);
    if (azureTimer == NULL) {
        return ExitCode_Init_AzureTimer;
    }

    // Register a one second timer to send a message to the RTApp.
    /*static const struct timespec sendPeriod = { .tv_sec = 1, .tv_nsec = 0 };
    sendTimer = CreateEventLoopPeriodicTimer(eventLoop, &SendTimerEventHandler, &sendPeriod);
    if (sendTimer == NULL) {
        return ExitCode_Init_SendTimer;
    }*/

    // Open a connection to the RTApp.
    sockFd = Application_Connect(rtAppComponentId);
    if (sockFd == -1) {
        Log_Debug("ERROR: Unable to create socket: %d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_Connection;
    }

    // Set timeout, to handle case where real-time capable application does not respond.
    static const struct timeval recvTimeout = { .tv_sec = 5, .tv_usec = 0 };
    int result = setsockopt(sockFd, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(recvTimeout));
    if (result == -1) {
        Log_Debug("ERROR: Unable to set socket timeout: %d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_SetSockOpt;
    }

    // Register handler for incoming messages from real-time capable application.
    socketEventReg = EventLoop_RegisterIo(eventLoop, sockFd, EventLoop_Input, SocketEventHandler,
        /* context */ NULL);
    if (socketEventReg == NULL) {
        Log_Debug("ERROR: Unable to register socket event: %d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_RegisterIo;
    }

    return ExitCode_Success;
}

/// <summary>
///     Closes a file descriptor and prints an error on failure.
/// </summary>
/// <param name="fd">File descriptor to close</param>
/// <param name="fdName">File descriptor name to use in error message</param>
static void CloseFdAndPrintError(int fd, const char* fdName)
{
    if (fd >= 0) {
        int result = close(fd);
        if (result != 0) {
            Log_Debug("ERROR: Could not close fd %s: %s (%d).\n", fdName, strerror(errno), errno);
        }
    }
}

/// <summary>
///     Clean up the resources previously allocated.
/// </summary>
static void CloseHandlers(void)
{
    DisposeEventLoopTimer(sendTimer);
    EventLoop_UnregisterIo(eventLoop, socketEventReg);
    EventLoop_Close(eventLoop);

    Log_Debug("Closing file descriptors.\n");
    CloseFdAndPrintError(sockFd, "Socket");
}



























// Main entry point 
//int main(int argc, char* argv[])
//{
//    Log_Debug("RX UART application starting.\n");
//
//    bool isNetworkingReady = false;
//    if ((Networking_IsNetworkingReady(&isNetworkingReady) == -1) || !isNetworkingReady) {
//        Log_Debug("WARNING: Network is not ready. Device cannot connect until network is ready.\n");
//    }
//
//    if (argc == 2) {
//        Log_Debug("Setting Azure Scope ID %s\n", argv[1]);
//        strncpy(scopeId, argv[1], SCOPEID_LENGTH);
//    }
//    else {
//        Log_Debug("ScopeId needs to be set in the app_manifest CmdArgs\n");
//        return -1;
//    }
//
//    exitCode = InitPeripheralsAndHandlers();
//
//    // Main loop
//    while (exitCode == ExitCode_Success) {
//        EventLoop_Run_Result result = EventLoop_Run(eventLoop, -1, true);
//        // Continue if interrupted by signal, e.g. due to breakpoint being set.
//        if (result == EventLoop_Run_Failed && errno != EINTR) {
//            exitCode = ExitCode_Main_EventLoopFail;
//        }
//    }
//
//    ClosePeripheralsAndHandlers();
//    Log_Debug("Application exiting.\n");
//    return exitCode;
//}



// // // // // // // // // // // // // // // // // // 
// Azure timer event:  Check connection status and send telemetry
// // // // // // // // // // // // // // // // // // 
static void AzureTimerEventHandler(EventLoopTimer* timer)
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
    }
    else {
        Log_Debug("Failed to get Network state\n");
    }

    if (iothubAuthenticated) {
        IoTHubDeviceClient_LL_DoWork(iothubClientHandle);
    }
}


// Sets the IoT Central authentication state for the app
// The SAS Token expires which will set the authentication state
static void HubConnectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result,
    IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason,
    void* userContextCallback)
{
    iothubAuthenticated = (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED);
    Log_Debug("IoT Central Authenticated: %s\n", GetReasonString(reason));
}


// Sets up the Azure IoT Central connection (creates the iothubClientHandle)
// When the SAS Token for a device expires the connection needs to be recreated
// which is why this is not simply a one time call.

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
        }
        else {
            azureIoTPollPeriodSeconds *= 2;
            if (azureIoTPollPeriodSeconds > AzureIoTMaxReconnectPeriodSeconds) {
                azureIoTPollPeriodSeconds = AzureIoTMaxReconnectPeriodSeconds;
            }
        }

        struct timespec azureTelemetryPeriod = { azureIoTPollPeriodSeconds, 0 };
        SetEventLoopTimerPeriod(azureTimer, &azureTelemetryPeriod);

        Log_Debug("ERROR: failure to create IoTHub Handle - will retry in %i seconds.\n",
            azureIoTPollPeriodSeconds);
        return;
    }

    // Successfully connected, so make sure the polling frequency is back to the default
    azureIoTPollPeriodSeconds = AzureIoTDefaultPollPeriodSeconds;
    struct timespec azureTelemetryPeriod = { .tv_sec = azureIoTPollPeriodSeconds, .tv_nsec = 0 };
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


// Callback invoked when a Device Twin update is received from IoT Central.
// Updates local state for 'showEvents' (bool).
// <param name="payload">contains the Device Twin JSON document (desired and reported)</param>
// <param name="payloadSize">size of the Device Twin JSON document</param>
static void TwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char* payload,
    size_t payloadSize, void* userContextCallback)
{
    size_t nullTerminatedJsonSize = payloadSize + 1;
    char* nullTerminatedJsonString = (char*)malloc(nullTerminatedJsonSize);
    if (nullTerminatedJsonString == NULL) {
        Log_Debug("ERROR: Could not allocate buffer for twin update payload.\n");
        abort();
    }

    // Copy the provided buffer to a null terminated buffer.
    memcpy(nullTerminatedJsonString, payload, payloadSize);
    // Add the null terminator at the end.
    nullTerminatedJsonString[nullTerminatedJsonSize - 1] = 0;

    JSON_Value* rootProperties = NULL;
    rootProperties = json_parse_string(nullTerminatedJsonString);
    if (rootProperties == NULL) {
        Log_Debug("WARNING: Cannot parse the string as JSON content.\n");
        goto cleanup;
    }

    JSON_Object* rootObject = json_value_get_object(rootProperties);
    JSON_Object* desiredProperties = json_object_dotget_object(rootObject, "desired");
    if (desiredProperties == NULL) {
        desiredProperties = rootObject;
    }


cleanup:
    // Release the allocated memory.
    json_value_free(rootProperties);
    free(nullTerminatedJsonString);
}

// Converts the IoT Central connection status reason to a string.
static const char* GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason)
{
    static char* reasonString = "unknown reason";
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


// Converts AZURE_SPHERE_PROV_RETURN_VALUE to a string.

static const char* getAzureSphereProvisioningResultString(
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


// Sends telemetry to IoT Central
// <param name="key">The telemetry item to update</param>
// <param name="value">new telemetry value</param>
static void SendTelemetry(const unsigned char* key, const unsigned char* value)
{
    static char eventBuffer[100] = { 0 };
    static const char* EventMsgTemplate = "{ \"%s\": \"%s\" }";
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

    if (IoTHubDeviceClient_LL_SendEventAsync(iothubClientHandle, messageHandle, SendMessageCallback,
        /*&callback_param*/ 0) != IOTHUB_CLIENT_OK) {
        Log_Debug("WARNING: failed to hand over the message to IoT Central Client\n");
    }
    else {
        Log_Debug("INFO: IoT Central Client accepted the message for delivery\n");
    }

    IoTHubMessage_Destroy(messageHandle);
}


// Callback confirming message delivered to IoT Central.
// <param name="result">Message delivery status</param>
// <param name="context">User specified context</param>
static void SendMessageCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* context)
{
    Log_Debug("INFO: Message received by IoT Central. Result is: %d\n", result);
}


// Creates and enqueues a report containing the name and value pair of a Device Twin reported
// property. The report is not sent immediately, but it is sent on the next invocation of
// IoTHubDeviceClient_LL_DoWork().
// <param name="propertyName">the IoT Central Device Twin property name</param>
// <param name="propertyValue">the IoT Central Device Twin property value</param>
static void TwinReportBoolState(const char* propertyName, bool propertyValue)
{
    if (iothubClientHandle == NULL) {
        Log_Debug("ERROR: client not initialized\n");
    }
    else {
        static char reportedPropertiesString[30] = { 0 };
        int len = snprintf(reportedPropertiesString, 30, "{\"%s\":%s}", propertyName,
            (propertyValue == true ? "true" : "false"));
        if (len < 0)
            return;

        if (IoTHubDeviceClient_LL_SendReportedState(
            iothubClientHandle, (unsigned char*)reportedPropertiesString,
            strlen(reportedPropertiesString), ReportStatusCallback, 0) != IOTHUB_CLIENT_OK) {
            Log_Debug("ERROR: failed to set reported state for '%s'.\n", propertyName);
        }
        else {
            Log_Debug("INFO: Reported state for '%s' to value '%s'.\n", propertyName,
                (propertyValue == true ? "true" : "false"));
        }
    }
}


// Callback invoked when the Device Twin reported properties are accepted by IoT Central.
static void ReportStatusCallback(int result, void* context)
{
    Log_Debug("INFO: Device Twin reported properties update result: HTTP status code %d\n", result);
}





int main(int argc, char* argv[])
{
    Log_Debug("Futura High-level intercore application starting\n");
    Log_Debug("Sends data to, and receives data from a real-time capable application.\n");
        bool isNetworkingReady = false;
        if ((Networking_IsNetworkingReady(&isNetworkingReady) == -1) || !isNetworkingReady) {
            Log_Debug("WARNING: Network is not ready. Device cannot connect until network is ready.\n");
        }
    
        if (argc == 2) {
            Log_Debug("Setting Azure Scope ID %s\n", argv[1]);
            strncpy(scopeId, argv[1], SCOPEID_LENGTH);
        }
        else {
            Log_Debug("ScopeId needs to be set in the app_manifest CmdArgs\n");
            return -1;
        }

    exitCode = InitHandlers();

    while (exitCode == ExitCode_Success) {
        EventLoop_Run_Result result = EventLoop_Run(eventLoop, -1, true);
        // Continue if interrupted by signal, e.g. due to breakpoint being set.
        if (result == EventLoop_Run_Failed && errno != EINTR) {
            exitCode = ExitCode_Main_EventLoopFail;
        }
    }

    CloseHandlers();
    Log_Debug("Application exiting.\n");
    return exitCode;
}


//static void UartEventHandler(EventLoop* el, int fd, EventLoop_IoEvents events, void* context)
//{
//    const size_t receiveBufferSize = 256;
//    uint8_t receiveBuffer[receiveBufferSize + 1]; // allow extra byte for string termination
//    ssize_t bytesRead;
//
//    // Read incoming UART data. 
//    bytesRead = read(uartFd, receiveBuffer, receiveBufferSize);
//    if (bytesRead < 0) {
//        Log_Debug("ERROR: Could not read UART: %s (%d).\n", strerror(errno), errno);
//        exitCode = ExitCode_UartEvent_Read;
//        return;
//    }
//
//    if (bytesRead > 0) {
//        // print and sendtelemetry
//        receiveBuffer[bytesRead] = 0;
//        Log_Debug("UART received %d bytes: '%s'.\n", bytesRead, (char*)receiveBuffer);
//        SendTelemetry("UART", (char*)receiveBuffer);
//    }
//}
