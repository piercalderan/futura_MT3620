#define IOT_CENTRAL_APPLICATION

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h> 
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>

#include "applibs_versions.h"
#include "epoll_timerfd_utilities.h"

#include "hw/sample_hardware.h"
#include "deviceTwin.h"
#include "azure_iot_utilities.h"
#include <applibs/log.h>
#include <applibs/gpio.h>
#include <applibs/wificonfig.h>
#include <azureiot/iothub_device_client_ll.h>


// Provide local access to variables in other files
extern twin_t twinArray[];
extern int twinArraySize;
extern IOTHUB_DEVICE_CLIENT_LL_HANDLE iothubClientHandle;

// Support functions.
static void TerminationHandler(int signalNumber);
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);

// File descriptors - initialized to invalid value
int epollFd = -1;

static int buttonPollTimerFd = -1;
static int button1GpioFd = -1;
static int button2GpioFd = -1;
static int button3GpioFd = -1;

int userLedRedFd = -1;
int userLedGreenFd = -1;
int userLedYellowFd = -1;
int user1Fd = -1;
int user2Fd = -1;
int user3Fd = -1;

//int wifiLedFd = -1;
//int clickSocket1Relay1Fd = -1;
//int clickSocket1Relay2Fd = -1;

// Azure IoT Hub/Central defines.
#define SCOPEID_LENGTH 20
char scopeId[SCOPEID_LENGTH]; // ScopeId for the Azure IoT Central application and DPS set in
							  // app_manifest.json, CmdArgs

// Button state variables, initilize them to button not-pressed (High)
static GPIO_Value_Type button1State = GPIO_Value_High;
static GPIO_Value_Type button2State = GPIO_Value_High;
static GPIO_Value_Type button3State = GPIO_Value_High;

#if (defined(IOT_CENTRAL_APPLICATION) )
bool versionStringSent = false;
#endif

// Define the Json string format for the accelerator button press data
static const char cstrButtonTelemetryJson[] = "{\"%s\":\"%d\"}";

// Termination state
volatile sig_atomic_t terminationRequired = false;

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
	// Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
	terminationRequired = true;
}

static void* SetupHeapMessage(const char* messageFormat, size_t maxLength, ...)
{
	va_list args;
	va_start(args, maxLength);
	char* message =
		malloc(maxLength + 1); // Ensure there is space for the null terminator put by vsnprintf.
	if (message != NULL) {
		vsnprintf(message, maxLength, messageFormat, args);
	}
	va_end(args);
	return message;
}

static int DirectMethodCall(const char* methodName, const char* payload, size_t payloadSize, char** responsePayload, size_t* responsePayloadSize)
{
	Log_Debug("\nDirect Method called %s\n", methodName);
	int result = 404; // HTTP status code.
	if (payloadSize < 32) {
		char directMethodCallContent[payloadSize + 1];
		*responsePayload = NULL;  // Reponse payload content.
		*responsePayloadSize = 0; // Response payload content size.
		// USER FUNCTIONS
		if (strcmp(methodName, "LightOff") == 0) { GPIO_SetValue(userLedRedFd, GPIO_Value_Low);	GPIO_SetValue(user1Fd, GPIO_Value_Low);
			Log_Debug("Light Off Application() Direct Method called\n"); result = 200;
			static const char resetOkResponse[] = "{ \"success\" : true, \"message\" : \"LightOff Application\" }";
			size_t responseMaxLength = sizeof(resetOkResponse);
			*responsePayload = SetupHeapMessage(resetOkResponse, responseMaxLength);
			if (*responsePayload == NULL) {Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");	abort();}
			*responsePayloadSize = strlen(*responsePayload); return result; 
		}
		if (strcmp(methodName, "LightOn") == 0) {GPIO_SetValue(userLedRedFd, GPIO_Value_High);GPIO_SetValue(user1Fd, GPIO_Value_High);
			Log_Debug("Light On Application() Direct Method called\n"); result = 200;
			static const char resetOkResponse[] = "{ \"success\" : true, \"message\" : \"LightOn Application\" }";
			size_t responseMaxLength = sizeof(resetOkResponse); *responsePayload = SetupHeapMessage(resetOkResponse, responseMaxLength);
			if (*responsePayload == NULL) {Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");	abort(); }
			*responsePayloadSize = strlen(*responsePayload); return result; 
		}

		if (strcmp(methodName, "FanOff") == 0) {
			GPIO_SetValue(userLedGreenFd, GPIO_Value_Low);	GPIO_SetValue(user2Fd, GPIO_Value_Low);
			Log_Debug("Fan Off Application() Direct Method called\n"); result = 200;
			static const char resetOkResponse[] = "{ \"success\" : true, \"message\" : \"FanOff Application\" }";
			size_t responseMaxLength = sizeof(resetOkResponse);
			*responsePayload = SetupHeapMessage(resetOkResponse, responseMaxLength);
			if (*responsePayload == NULL) { Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");	abort(); }
			*responsePayloadSize = strlen(*responsePayload); return result;
		}
		if (strcmp(methodName, "FanOn") == 0) {
			GPIO_SetValue(userLedGreenFd, GPIO_Value_High); GPIO_SetValue(user2Fd, GPIO_Value_High);
			Log_Debug("Fan On Application() Direct Method called\n"); result = 200;
			static const char resetOkResponse[] = "{ \"success\" : true, \"message\" : \"FanOn Application\" }";
			size_t responseMaxLength = sizeof(resetOkResponse); *responsePayload = SetupHeapMessage(resetOkResponse, responseMaxLength);
			if (*responsePayload == NULL) { Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");	abort(); }
			*responsePayloadSize = strlen(*responsePayload); return result;
		}

		if (strcmp(methodName, "MotorOff") == 0) {
			GPIO_SetValue(userLedYellowFd, GPIO_Value_Low);	GPIO_SetValue(user3Fd, GPIO_Value_Low);
			Log_Debug("Motor Off Application() Direct Method called\n"); result = 200;
			static const char resetOkResponse[] = "{ \"success\" : true, \"message\" : \"MotorOff Application\" }";
			size_t responseMaxLength = sizeof(resetOkResponse);
			*responsePayload = SetupHeapMessage(resetOkResponse, responseMaxLength);
			if (*responsePayload == NULL) { Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");	abort(); }
			*responsePayloadSize = strlen(*responsePayload); return result;
		}
		if (strcmp(methodName, "MotorOn") == 0) {
			GPIO_SetValue(userLedYellowFd, GPIO_Value_High); GPIO_SetValue(user3Fd, GPIO_Value_High);
			Log_Debug("Motor On Application() Direct Method called\n"); result = 200;
			static const char resetOkResponse[] = "{ \"success\" : true, \"message\" : \"MotorOn Application\" }";
			size_t responseMaxLength = sizeof(resetOkResponse); *responsePayload = SetupHeapMessage(resetOkResponse, responseMaxLength);
			if (*responsePayload == NULL) { Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");	abort(); }
			*responsePayloadSize = strlen(*responsePayload); return result;
		}

		else if (strcmp(methodName, "setSensorPollTime") == 0) {
			Log_Debug("setSensorPollTime() Direct Method called\n");
			result = 200;
			if (directMethodCallContent == NULL) {
				Log_Debug("ERROR: Could not allocate buffer for direct method request payload.\n");
				abort();
			}
			memcpy(directMethodCallContent, payload, payloadSize);
			directMethodCallContent[payloadSize] = 0; // Null terminated string.
			JSON_Value* payloadJson = json_parse_string(directMethodCallContent);
			if (payloadJson == NULL) {
				goto payloadError;
			}
			JSON_Object* pollTimeJson = json_value_get_object(payloadJson);
			if (pollTimeJson == NULL) {
				goto payloadError;
			}
			int newPollTime = (int)json_object_get_number(pollTimeJson, "pollTime");
			if (newPollTime < 1) {
				goto payloadError;
			}
			else {
				Log_Debug("New PollTime %d\n", newPollTime);
				static const char newPollTimeResponse[] =
					"{ \"success\" : true, \"message\" : \"New Sensor Poll Time %d seconds\" }";
				size_t responseMaxLength = sizeof(newPollTimeResponse) + strlen(payload);
				*responsePayload = SetupHeapMessage(newPollTimeResponse, responseMaxLength, newPollTime);
				if (*responsePayload == NULL) {
					Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");
					abort();
				}
				*responsePayloadSize = strlen(*responsePayload);
				return result;
			}
		}
		else {
			result = 404;
			Log_Debug("INFO: Direct Method called \"%s\" not found.\n", methodName);
			static const char noMethodFound[] = "\"method not found '%s'\"";
			size_t responseMaxLength = sizeof(noMethodFound) + strlen(methodName);
			*responsePayload = SetupHeapMessage(noMethodFound, responseMaxLength, methodName);
			if (*responsePayload == NULL) {
				Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");
				abort();
			}
			*responsePayloadSize = strlen(*responsePayload);
			return result;
		}
	}
	else {
		Log_Debug("Payload size > 32 bytes, aborting Direct Method execution\n");
		goto payloadError;
	}
payloadError:
	result = 400; // Bad request.
	Log_Debug("INFO: Unrecognised direct method payload format.\n");
	static const char noPayloadResponse[] =
		"{ \"success\" : false, \"message\" : \"request does not contain an identifiable "
		"payload\" }";
	size_t responseMaxLength = sizeof(noPayloadResponse) + strlen(payload);
	responseMaxLength = sizeof(noPayloadResponse);
	*responsePayload = SetupHeapMessage(noPayloadResponse, responseMaxLength);
	if (*responsePayload == NULL) {
		Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");
		abort();
	}
	*responsePayloadSize = strlen(*responsePayload);
	return result;
}

static void ButtonTimerEventHandler(EventData* eventData)
{

	bool sendTelemetrybutton1 = false;
	bool sendTelemetrybutton2 = false;
	bool sendTelemetrybutton3 = false;

	if (ConsumeTimerFdEvent(buttonPollTimerFd) != 0) {
		terminationRequired = true;
		return;
	}
	
	GPIO_Value_Type newbutton1State;
	int result = GPIO_GetValue(button1GpioFd, &newbutton1State);
	if (result != 0) {
		Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (newbutton1State != button1State) {
		if (newbutton1State == GPIO_Value_Low) {
			Log_Debug("Button 1 pressed!\n");
			sendTelemetrybutton1 = true;
		}
		else {
			Log_Debug("Button 1 released!\n");
			sendTelemetrybutton1 = true;
		}		
		button1State = newbutton1State;
	}

	GPIO_Value_Type newbutton2State;
	result = GPIO_GetValue(button2GpioFd, &newbutton2State);
	if (result != 0) {
		Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (newbutton2State != button2State) 
	{
		if (newbutton2State == GPIO_Value_Low) {			
			Log_Debug("Button 2 pressed!\n");
			sendTelemetrybutton2 = true;

		}
		else {
			Log_Debug("Button 2 released!\n");
			sendTelemetrybutton2 = true;
		}
		// Update the static variable to use next time we enter this routine
		button2State = newbutton2State;
	}

	GPIO_Value_Type newbutton3State;
	result = GPIO_GetValue(button3GpioFd, &newbutton3State);
	if (result != 0) {
		Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (newbutton3State != button3State)
	{
		if (newbutton3State == GPIO_Value_Low) {
			Log_Debug("Button 3 pressed!\n");
			sendTelemetrybutton3 = true;

		}
		else {
			Log_Debug("Button 3 released!\n");
			sendTelemetrybutton3 = true;
		}
		// Update the static variable to use next time we enter this routine
		button3State = newbutton3State;
	}

	// If either button was pressed, then enter the code to send the telemetry message
	if (sendTelemetrybutton1 || sendTelemetrybutton2 || sendTelemetrybutton3) {

		char* pjsonBuffer = (char*)malloc(JSON_BUFFER_SIZE);
		if (pjsonBuffer == NULL) {
			Log_Debug("ERROR: not enough memory to send telemetry");
		}

		if (sendTelemetrybutton1) {
			// construct the telemetry message  for Button 1
			snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrButtonTelemetryJson, "button1", !newbutton1State);
			Log_Debug("\n[Info] Sending telemetry %s\n", pjsonBuffer);
			AzureIoT_SendMessage(pjsonBuffer);
		}

		if (sendTelemetrybutton2) {
			// construct the telemetry message for Button 2
			snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrButtonTelemetryJson, "button2", newbutton2State);
			Log_Debug("\n[Info] Sending telemetry %s\n", pjsonBuffer);
			AzureIoT_SendMessage(pjsonBuffer);
		}

		if (sendTelemetrybutton3) {
			// construct the telemetry message for Button 3
			snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrButtonTelemetryJson, "button3", newbutton3State);
			Log_Debug("\n[Info] Sending telemetry %s\n", pjsonBuffer);
			AzureIoT_SendMessage(pjsonBuffer);
		}

		free(pjsonBuffer);
	}

}

// event handler data structures. Only the event handler field needs to be populated.
static EventData buttonEventData = { .eventHandler = &ButtonTimerEventHandler };



/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{

	// Open LED RED 1 as output
	Log_Debug("Opening SAMPLE_LED_1 as output\n");
	userLedRedFd = GPIO_OpenAsOutput(SAMPLE_LED_1, GPIO_OutputMode_PushPull, GPIO_Value_Low);
	if (userLedRedFd < 0) {
		Log_Debug("ERROR: Could not open LED 1: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	// Open LED YELLOW 2 as output
	Log_Debug("Opening SAMPLE_LED_2 as output\n");
	userLedYellowFd = GPIO_OpenAsOutput(SAMPLE_LED_2, GPIO_OutputMode_PushPull, GPIO_Value_Low);
	if (userLedYellowFd < 0) {
		Log_Debug("ERROR: Could not open LED 1: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	// Open LED GREEN 3 as output
	Log_Debug("Opening SAMPLE_LED_3 as output\n");
	userLedGreenFd = GPIO_OpenAsOutput(SAMPLE_LED_3, GPIO_OutputMode_PushPull, GPIO_Value_Low);
	if (userLedGreenFd < 0) {
		Log_Debug("ERROR: Could not open LED 1: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	// Open USER 1 as output
	Log_Debug("Opening SAMPLE_USER_1 as output\n");
	user1Fd = GPIO_OpenAsOutput(SAMPLE_USER_1, GPIO_OutputMode_PushPull, GPIO_Value_Low);
	if (user1Fd < 0) {
		Log_Debug("ERROR: Could not open USER 1: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	// Open USER 2 as output
	Log_Debug("Opening SAMPLE_USER_2 as output\n");
	user2Fd = GPIO_OpenAsOutput(SAMPLE_USER_2, GPIO_OutputMode_PushPull, GPIO_Value_Low);
	if (user2Fd < 0) {
		Log_Debug("ERROR: Could not open USER 2: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	// Open USER 3 as output
	Log_Debug("Opening SAMPLE_USER_3 as output\n");
	user3Fd = GPIO_OpenAsOutput(SAMPLE_USER_3, GPIO_OutputMode_PushPull, GPIO_Value_Low);
	if (user3Fd < 0) {
		Log_Debug("ERROR: Could not open USER 3: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_handler = TerminationHandler;
	sigaction(SIGTERM, &action, NULL);

	epollFd = CreateEpollFd();
	if (epollFd < 0) {
		return -1;
	}




	// Traverse the twin Array and for each GPIO item in the list open the file descriptor
	for (int i = 0; i < twinArraySize; i++) {

		// Verify that this entry is a GPIO entry
		if (twinArray[i].twinGPIO != NO_GPIO_ASSOCIATED_WITH_TWIN) {

			*twinArray[i].twinFd = -1;

			// For each item in the data structure, initialize the file descriptor and open the GPIO for output.  Initilize each GPIO to its specific inactive state.
			*twinArray[i].twinFd = (int)GPIO_OpenAsOutput(twinArray[i].twinGPIO, GPIO_OutputMode_PushPull, twinArray[i].active_high ? GPIO_Value_Low : GPIO_Value_High);

			if (*twinArray[i].twinFd < 0) {
				Log_Debug("ERROR: Could not open LED %d: %s (%d).\n", twinArray[i].twinGPIO, strerror(errno), errno);
				return -1;
			}
		}
	}

	// Open button 1 GPIO as input
	Log_Debug("Opening SAMPLE_BUTTON_1 as input.\n");
	button1GpioFd = GPIO_OpenAsInput(SAMPLE_BUTTON_1);
	if (button1GpioFd < 0) {
		Log_Debug("ERROR: Could not open button 1 GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	// Open button 2 GPIO as input
	Log_Debug("Opening SAMPLE_BUTTON_2 as input.\n");
	button2GpioFd = GPIO_OpenAsInput(SAMPLE_BUTTON_2);
	if (button2GpioFd < 0) {
		Log_Debug("ERROR: Could not open button 2 GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	// Open button 3 GPIO as input
	Log_Debug("Opening SAMPLE_BUTTON_3 as input.\n");
	button3GpioFd = GPIO_OpenAsInput(SAMPLE_BUTTON_3);
	if (button3GpioFd < 0) {
		Log_Debug("ERROR: Could not open button 3 GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	// Set up a timer to poll the buttons
	struct timespec buttonPressCheckPeriod = { 0, 1000000 };
	buttonPollTimerFd =
		CreateTimerFdAndAddToEpoll(epollFd, &buttonPressCheckPeriod, &buttonEventData, EPOLLIN);
	if (buttonPollTimerFd < 0) {
		return -1;
	}

	// Tell the system about the callback function that gets called when we receive a device twin update message from Azure
	AzureIoT_SetDeviceTwinUpdateCallback(&deviceTwinChangedHandler);

	// Tell the system about the callback function to call when we receive a Direct Method message from Azure
	AzureIoT_SetDirectMethodCallback(&DirectMethodCall);

	return 0;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
	Log_Debug("Closing file descriptors.\n");

	
	CloseFdAndPrintError(epollFd, "Epoll");
	CloseFdAndPrintError(buttonPollTimerFd, "buttonPoll");
	CloseFdAndPrintError(button1GpioFd, "button1");
	CloseFdAndPrintError(button2GpioFd, "button2");
	CloseFdAndPrintError(button3GpioFd, "button3");

	// Traverse the twin Array and for each GPIO item in the list the close the file descriptor
	for (int i = 0; i < twinArraySize; i++) {

		// Verify that this entry has an open file descriptor
		if (twinArray[i].twinGPIO != NO_GPIO_ASSOCIATED_WITH_TWIN) {

			CloseFdAndPrintError(*twinArray[i].twinFd, twinArray[i].twinKey);
		}
	}
}

/// <summary>
///     Main entry point for this application.
/// </summary>
int main(int argc, char* argv[])
{
	// Variable to help us send the version string up only once
	bool networkConfigSent = false;
	char ssid[128];
	uint32_t frequency;
	char bssid[20];

	// Clear the ssid array
	memset(ssid, 0, 128);

#if (defined(IOT_CENTRAL_APPLICATION) )
	if (argc == 2) {
		Log_Debug("Setting Azure Scope ID %s\n", argv[1]);
		strncpy(scopeId, argv[1], SCOPEID_LENGTH);
	}
	else {
		Log_Debug("ScopeId needs to be set in the app_manifest CmdArgs\n");
		return -1;
	}
#endif 

	Log_Debug("GPIO Direct method application starting.\n");
	if (InitPeripheralsAndHandlers() != 0) {
		terminationRequired = true;
	}

	// Use epoll to wait for events and trigger handlers, until an error or SIGTERM happens
	while (!terminationRequired) {
		if (WaitForEventAndCallHandler(epollFd) != 0) {
			terminationRequired = true;
		}

#if (defined(IOT_CENTRAL_APPLICATION) )
		// Setup the IoT Hub client.
		// Notes:
		// - it is safe to call this function even if the client has already been set up, as in
		//   this case it would have no effect;
		// - a failure to setup the client is a fatal error.
		if (!AzureIoT_SetupClient()) {
			Log_Debug("ERROR: Failed to set up IoT Hub client\n");
			Log_Debug("ERROR: Verify network connection and Azure Resource configurations\n");
		}
#endif 

		WifiConfig_ConnectedNetwork network;
		int result = WifiConfig_GetCurrentNetwork(&network);

#if (defined(IOT_CENTRAL_APPLICATION) )
		if (iothubClientHandle != NULL && !versionStringSent) {

			#warning "If you need to update the version string do so in main.c ~line 740!"
				checkAndUpdateDeviceTwin("versionString", "FUTURA MT3620", TYPE_STRING, false);
			versionStringSent = true;
		}

		AzureIoT_DoPeriodicTasks();
#endif
	}

	ClosePeripheralsAndHandlers();
	Log_Debug("Application exiting.\n");
	return 0;
}
