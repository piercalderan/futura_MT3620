#  Copyright (c) Microsoft Corporation. All rights reserved.
#  Licensed under the MIT License.

CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
PROJECT(HelloWorld_HighLevelApp C)
azsphere_configure_tools(TOOLS_REVISION "20.07")
azsphere_configure_api(TARGET_API_SET "6")

# Create executable
ADD_EXECUTABLE(${PROJECT_NAME} main.c)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} applibs pthread gcc_s c)

azsphere_target_hardware_definition(${PROJECT_NAME} TARGET_DIRECTORY "../../Hardware/futura_mt3620" TARGET_DEFINITION "sample_hardware.json")

# Add MakeImage post-build command
INCLUDE("${AZURE_SPHERE_MAKE_IMAGE_FILE}")
