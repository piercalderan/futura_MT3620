// Futura MT3620 MPU6050 IoT Central example.
// Copyright 2020 Pier Calderan.
// Apre il pulsante 2 come ingresso.
// Apre il sensore MPU6050.
// Premendo il pulsante 2 si mandano i dati di telemetria dell'MPU6050 a IoT Central.

#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include <applibs/i2c.h>
#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/networking.h>
#include <applibs/gpio.h>
#include <applibs/storage.h>
#include <applibs/eventloop.h>

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


static char eventBuffer[100] = { 0 };

// MPU6050 Accel XYZ Gyro XYZ
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

double temp_MPU6050;

// MPU6050 address
static const uint8_t MPU6050Address = 0x68;

//ExitCode enum
typedef enum {
    ExitCode_Success = 0,
    ExitCode_TermHandler_SigTerm = 1,
    ExitCode_Main_EventLoopFail = 2,
    ExitCode_ButtonTimer_Consume = 3,
    ExitCode_AzureTimer_Consume = 4,
    ExitCode_Init_EventLoop = 5,
    ExitCode_Init_MessageButton = 6,
    ExitCode_Init_TwinStatusLed = 8,
    ExitCode_Init_ButtonPollTimer = 9,
    ExitCode_Init_AzureTimer = 10,
    ExitCode_IsButtonPressed_GetValue = 11,
    ExitCode_AccelTimer_ReadStatus = 12,
    ExitCode_AccelTimer_ReadZAccel = 13,
    ExitCode_AccelTimer_Consume = 24,
    ExitCode_Init_AccelTimer = 25,
    ExitCode_Init_OpenMaster = 26,
    ExitCode_Init_SetBusSpeed = 27,
    ExitCode_Init_SetTimeout = 28,
    ExitCode_Init_SetDefaultTarget = 29,
    ExitCode_Main_Led = 30,    
    ExitCode_Init_RegisterIo = 33,
} ExitCode;

// function declarations
static volatile sig_atomic_t exitCode = ExitCode_Success;
static void TerminationHandler(int signalNumber);
static void AccelTimerEventHandler(EventLoopTimer* timer);

// Azure IoT Hub/Central defines.
#define SCOPEID_LENGTH 20
static char scopeId[SCOPEID_LENGTH]; // ScopeId for the Azure IoT Central application, set in app_manifest.json, CmdArgs
static IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle = NULL;
static const int keepalivePeriodSeconds = 20;
static bool iothubAuthenticated = false;
static void SendMessageCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context);
static void TwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payload,
                         size_t payloadSize, void *userContextCallback);
static void TwinReportBoolState(const char *propertyName, bool propertyValue);
static void ReportStatusCallback(int result, void *context);
static const char *GetReasonString(IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason);
static const char *getAzureSphereProvisioningResultString(
    AZURE_SPHERE_PROV_RETURN_VALUE provisioningResult);
static void SendTelemetry(const unsigned char *key, const unsigned char *value);
static void SetupAzureClient(void);

// Function to generate simulated Temperature data/telemetry, uncomment optionally
// static void SendSimulatedTemperature(void); 
// File descriptors - initialized to invalid value
static int i2cFd = -1; //File Descriptor for i2c
static EventLoopTimer* accelTimer = NULL;

double vertical_accel = 0.0;

// Print latest data from accelerometer.
static void AccelTimerEventHandler(EventLoopTimer* timer)
{

    static int iter = 1;

    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_AccelTimer_Consume;
        return;
    }


    static const uint8_t statusRegId = 0x00; // 
    uint8_t status;
    ssize_t transferredBytes = I2CMaster_WriteThenRead(
        i2cFd, MPU6050Address, &statusRegId, sizeof(statusRegId), &status, sizeof(status));
    if ((status & 0x1) == 0) {
        Log_Debug("INFO: %d: No accelerometer data.\n", iter);
    }
    else {

        //constants for mpu6050 registers
#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42

        const uint8_t Command1[] = { MPU6050_SMPLRT_DIV, 0x00 };
        ssize_t transferredBytes1 = I2CMaster_Write(i2cFd, MPU6050Address, Command1, sizeof(Command1));

        const uint8_t Command2[] = { MPU6050_CONFIG, 0x00 };
        ssize_t transferredBytes2 = I2CMaster_Write(i2cFd, MPU6050Address, Command2, sizeof(Command2));

        const uint8_t Command3[] = { MPU6050_GYRO_CONFIG, 0x08 };
        ssize_t transferredBytes3 = I2CMaster_Write(i2cFd, MPU6050Address, Command3, sizeof(Command3));

        const uint8_t Command4[] = { MPU6050_ACCEL_CONFIG, 0x00 };
        ssize_t transferredBytes4 = I2CMaster_Write(i2cFd, MPU6050Address, Command4, sizeof(Command4));

        const uint8_t Command5[] = { MPU6050_PWR_MGMT_1, 0x01 };
        ssize_t transferredBytes5 = I2CMaster_Write(i2cFd, MPU6050Address, Command5, sizeof(Command5));

        static const uint8_t outTmp = 0x41; //Temperature 
        static const uint8_t outAcX = 0x3b; //Accelerometer X 
        static const uint8_t outAcY = 0x3d; //Accelerometer Y 
        static const uint8_t outAcZ = 0x3f; //Accelerometer Z 
        static const uint8_t outGyX = 0x43; //Gyroscope X 
        static const uint8_t outGyY = 0x45; //Gyroscope Y 
        static const uint8_t outGyZ = 0x47; //Gyroscope Z 

        uint8_t zRaw, zRaw1;

        transferredBytes = I2CMaster_Write(i2cFd, MPU6050Address, &outTmp, sizeof(outTmp));
        ssize_t hi = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw, sizeof(zRaw));
        ssize_t lo = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw1, sizeof(zRaw1));

        Tmp = zRaw << 8 | zRaw1; //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
        // Formula from datasheet 
        // Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity) / 340 + 36.53
        temp_MPU6050 = Tmp / 340.00 + 36.53;
        Log_Debug("Temp: %.2lf\n", temp_MPU6050);

        transferredBytes = I2CMaster_Write(i2cFd, MPU6050Address, &outAcX, sizeof(outAcX));
        hi = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw, sizeof(zRaw));
        lo = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw1, sizeof(zRaw1));
        AcX = zRaw << 8 | zRaw1; //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        Log_Debug("AcX: %d\n", AcX);

        transferredBytes = I2CMaster_Write(i2cFd, MPU6050Address, &outAcY, sizeof(outAcY));
        hi = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw, sizeof(zRaw));
        lo = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw1, sizeof(zRaw1));
        AcY = zRaw << 8 | zRaw1; //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        Log_Debug("AcY: %d\n", AcY);

        transferredBytes = I2CMaster_Write(i2cFd, MPU6050Address, &outAcZ, sizeof(outAcZ));
        hi = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw, sizeof(zRaw));
        lo = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw1, sizeof(zRaw1));
        AcZ = zRaw << 8 | zRaw1; //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        Log_Debug("AcZ: %d\n", AcZ);

        transferredBytes = I2CMaster_Write(i2cFd, MPU6050Address, &outGyX, sizeof(outGyX));
        hi = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw, sizeof(zRaw));
        lo = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw1, sizeof(zRaw1));
        GyX = zRaw << 8 | zRaw1; //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        Log_Debug("GyX: %d\n", GyX);

        transferredBytes = I2CMaster_Write(i2cFd, MPU6050Address, &outGyY, sizeof(outGyY));
        hi = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw, sizeof(zRaw));
        lo = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw1, sizeof(zRaw1));
        GyY = zRaw << 8 | zRaw1; //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        Log_Debug("GyY: %d\n", GyY);

        transferredBytes = I2CMaster_Write(i2cFd, MPU6050Address, &outGyZ, sizeof(outGyZ));
        hi = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw, sizeof(zRaw));
        lo = I2CMaster_Read(i2cFd, MPU6050Address, (uint8_t*)&zRaw1, sizeof(zRaw1));
        GyZ = zRaw << 8 | zRaw1; //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
        Log_Debug("GyZ: %d\n", GyZ);
    }
    ++iter;
}


// Initialization/Cleanup
static ExitCode InitPeripheralsAndHandlers(void);
static void CloseFdAndPrintError(int fd, const char *fdName);
static void ClosePeripheralsAndHandlers(void);

// File descriptors - initialized to invalid value Buttons
static int sendMessageButtonGpioFd = -1;
static int sendMessageButtonGpioFdAccel = -1;
static int sendOrientationButtonGpioFd = -1;

// LED
static int deviceTwinStatusLedGpioFd = -1;
static bool statusLedOn = false;

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
static GPIO_Value_Type sendMessageButtonState = GPIO_Value_High;
static GPIO_Value_Type sendMessageButtonStateAccel = GPIO_Value_High;
static GPIO_Value_Type sendOrientationButtonState = GPIO_Value_High;

static void ButtonPollTimerEventHandler(EventLoopTimer *timer);
static bool IsButtonPressed(int fd, GPIO_Value_Type *oldState);
static void SendAccelButtonHandler(void);
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
    Log_Debug("I2C MPU6050 application starting.\n");

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
    SendAccelButtonHandler();    
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


// Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
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

    // Open button 2 GPIO as input
    Log_Debug("Opening SAMPLE_BUTTON_2 as input\n");
    sendMessageButtonGpioFdAccel = GPIO_OpenAsInput(SAMPLE_BUTTON_2);
    if (sendMessageButtonGpioFdAccel < 0) {
        Log_Debug("ERROR: Could not open button 2: %s (%d).\n", strerror(errno), errno);
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

    // Print accelerometer data every 0.9 second. Change optionally
    static const struct timespec accelReadPeriod = { .tv_sec = 0, .tv_nsec = 900000000 };
    accelTimer = CreateEventLoopPeriodicTimer(eventLoop, &AccelTimerEventHandler, &accelReadPeriod);
    if (accelTimer == NULL) {
        return ExitCode_Init_AccelTimer;
    }

    i2cFd = I2CMaster_Open(SAMPLE_ISU0_I2C); //MPU6050
    if (i2cFd < 0) {
        Log_Debug("ERROR: I2CMaster_Open: errno=%d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_OpenMaster;
    }
    else Log_Debug("Open ok\n");//OPEN OK

    int result = I2CMaster_SetBusSpeed(i2cFd, I2C_BUS_SPEED_STANDARD);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_SetBusSpeed;
    }
    else Log_Debug("Set Speed ok\n");//OPEN OK

    result = I2CMaster_SetTimeout(i2cFd, 100);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_SetTimeout;
    }
    else Log_Debug("Set Time Out ok\n");//OPEN OK

    // This default address is used for POSIX read and write calls.  The AppLibs APIs take a target address argument for each read or write.
    result = I2CMaster_SetDefaultTargetAddress(i2cFd, MPU6050Address);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetDefaultTargetAddress: errno=%d (%s)\n", errno,
            strerror(errno));
        return ExitCode_Init_SetDefaultTarget;
    }
    else Log_Debug("Set Default Target Address ok\n");//OPEN OK

    return ExitCode_Success;
}


// Closes a file descriptor and prints an error on failure.
static void CloseFdAndPrintError(int fd, const char *fdName)
{
    if (fd >= 0) {
        int result = close(fd);
        if (result != 0) {
            Log_Debug("ERROR: Could not close fd %s: %s (%d).\n", fdName, strerror(errno), errno);
        }
    }
}


// Close peripherals and handlers.
static void ClosePeripheralsAndHandlers(void)
{
    DisposeEventLoopTimer(buttonPollTimer);
    DisposeEventLoopTimer(azureTimer);
    EventLoop_Close(eventLoop);
    Log_Debug("Closing file descriptors\n");
    DisposeEventLoopTimer(accelTimer);
    CloseFdAndPrintError(i2cFd, "i2c");
    CloseFdAndPrintError(sendMessageButtonGpioFdAccel, "SendMessageButtonAccel");
}


// Sets the IoT Hub authentication state for the app
// The SAS Token expires which will set the authentication state
static void HubConnectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result,
                                        IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason,
                                        void *userContextCallback)
{
    iothubAuthenticated = (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED);
    Log_Debug("IoT Hub Authenticated: %s\n", GetReasonString(reason));
}


// Sets up the Azure IoT Hub connection (creates the iothubClientHandle)
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


// Callback invoked when a Device Twin update is received from IoT Hub.
// Updates local state for 'showEvents' (bool).
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

// Converts the IoT Hub connection status reason to a string.
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


// Converts AZURE_SPHERE_PROV_RETURN_VALUE to a string.

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


// Sends telemetry to IoT Hub
// <param name="key">The telemetry item to update</param>
// <param name="value">new telemetry value</param>
static void SendTelemetry(const unsigned char *key, const unsigned char *value)
{
    static char eventBuffer[100] = {0};
    static const char *EventMsgTemplate = "{ \"%s\": \"%s\" }";
    int len = snprintf(eventBuffer, sizeof(eventBuffer), EventMsgTemplate, key, value);
    if (len < 0)
        return;

    Log_Debug("Sending IoT Hub Message: %s\n", eventBuffer);

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
        Log_Debug("WARNING: failed to hand over the message to IoTHubClient\n");
    } else {
        Log_Debug("INFO: IoTHubClient accepted the message for delivery\n");
    }
    
    IoTHubMessage_Destroy(messageHandle);
}


// Callback confirming message delivered to IoT Hub.
// <param name="result">Message delivery status</param>
// <param name="context">User specified context</param>
static void SendMessageCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *context)
{
    Log_Debug("INFO: Message received by IoT Hub. Result is: %d\n", result);
}


// Creates and enqueues a report containing the name and value pair of a Device Twin reported
// property. The report is not sent immediately, but it is sent on the next invocation of
// IoTHubDeviceClient_LL_DoWork().

// <param name="propertyName">the IoT Hub Device Twin property name</param>
// <param name="propertyValue">the IoT Hub Device Twin property value</param>
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


// Callback invoked when the Device Twin reported properties are accepted by IoT Hub.
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


// Pressing button 2 will send Accel event to Azure IoT Central
static void SendAccelButtonHandler(void)
{
    if (IsButtonPressed(sendMessageButtonGpioFdAccel, &sendMessageButtonStateAccel)) {
        char tempBuffer[20];
        int len = snprintf(tempBuffer, 20, "%d", AcX);
        if (len > 0) {
            SendTelemetry("AccelX", tempBuffer);      //AcX
            Log_Debug("AcX: %s\n", tempBuffer);
        }
                
        len = snprintf(tempBuffer, 20, "%d", AcY);
        if (len > 0) SendTelemetry("AccelY", tempBuffer);      //AcY
        Log_Debug("AcY: %d\n", AcY);

        len = snprintf(tempBuffer, 20, "%d", AcZ);
        if (len > 0) SendTelemetry("AccelZ", tempBuffer);      //AcZ
        Log_Debug("AcZ: %d\n", AcZ);

        len = snprintf(tempBuffer, 20, "%d", GyX);
        if (len > 0) SendTelemetry("GyroX", tempBuffer);      //GyX
        Log_Debug("GyX: %d\n", GyX);

        len = snprintf(tempBuffer, 20, "%d", GyY);
        if (len > 0) SendTelemetry("GyroY", tempBuffer);      //GyY
        Log_Debug("GyY: %d\n", GyY);

        len = snprintf(tempBuffer, 20, "%d", GyZ);
        if (len > 0) SendTelemetry("GyroZ", tempBuffer);      //GyZ
        Log_Debug("GyZ: %d\n", GyZ);
        
        len = snprintf(tempBuffer, 20, "%.2lf", temp_MPU6050);
        if (len > 0) SendTelemetry("TempMPU6050", tempBuffer);      // tempMPU6050
        Log_Debug("Temp MPU6050: %i\n", tempBuffer);
    }
}

