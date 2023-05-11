# Changelog

# v0.4.9
- Added support HEX streaming mode.
- Added support for listing the schema information of the loaded sensors
- Added support for Head Orientation virtual sensors.
- Added support for Head Orientation Parameter Configuration.
- Removed the support for PDR
- Rectified parsing callback to s16_to_float for Temperature Sensor

Known Limitations -
- 'gyrogettat' does not return expected output.
- For Head Orientation sensors, for subsequent sensor activation with different ODRs, the ODR change is not reflected.
- For 'dactse' command, parsing flag is not updated.

# v0.4.8
- Optimised the Data Injection feature for generic use by removing Sensor ID dependnecy
- Integrated generic file parser for txt/bin files and resolved length dependency, for Data Injection
- Resolved Data Injection Issue in PC Mode
- Resolved EOF check failing for Data Injection in PC Mode
- Changed the Pattern Size check condtion for Klio, to accomadate for loading Adaptive Patterns
- Added notification for File Transfer status for 'wrfile' command
- Added support for getting the list of active sensors along with their configurations and currently open log file
- Added support for retrieving Post Mortem data (Currently supported only for MCU mode)
- Added support for BHI3 Sensor API
- Added support for new sensors -
	- Multi-Tap Detector
	- Activity recognition for wearables
	- No Motion
	- Wrist wear wake-up
	- Wrist gesture detector
- Added support for configuring the Physical Sensor Control Parameters
- Added support for reading the Physical Sensor Information
- Increase BLE Transmission Power to +8dBM
- Updated the Wrist Gesture output as per new firmware.

# v0.4.7
- Added support for Head_Orientation virtual sensor
- Added support for PDR_Log virtual sensor
- Corrected the output format for Klio_Log sensor
- Set COINES_BRIDGE as the default for PC mode.
- Branched COINES submodule to master branch. Updated the connection interval to 15ms from 37.5 ms.
- Added support for Data Injection feature
- Added support for Deactivating and Listing all the active sensors
- Added support for Read/Write file over BLE/USB
- Extended support for '\n' as a string termination parameter
- Added error hnadling for file operation commands

Known Limitations -
- Certain commands are not tested (addse, strbuff, kswpatt), owing to lack of requisite firmware images for testing.

# v0.4.6
- Fixed a bug in setting the PDR reference heading
- Fixed typos, most Lint and compiler warnings
- Fixed a bug in the PC build where Ctrl+C didn't exit immediately
- Added legacy command support and shortened CLI command string length from 32 to 16
- Migrated bhy2cli.exe to COINES Bridge beta for better performance
- Updated generic parsing to send the Hex string without any spaces
- Added run-time board recognition to select the correct GPIOs
- Updated close_interfaces() to include bus deconfig
- Cleaned up interrupt config in common and corrected the configuration in the reset function
- Added a first run check with activating a sensor (actse, logse) to query the available sensors from the BHy260, avoids having to call the info command
- Fixed a bug with debug message parsing
- Added dynamic switching support between COM port and BLE, with higher priority to BLE for the MCU bhy2cli
- Added Head tracking and BSEC output frame parsing support 
- Added reset cause code for Reset event 
- Added COINES multi support
- Added New Virtual and Physical sensor IDs
- Fixed the issue with generic parsing when activating (actse) the custom sensors

# v0.4.5
- Fixed a bug where the internal look up table depended on the info command to be called resulting in an erroneous parsing of sensor data
- Fixed a typo in the addse command's help
- Reverted vt100 support
- Updated to latest COINES master
- Made a specific change in COINES to increase BLE throughput
- Added a new command strbuf that helps buffering data to stream over BLE allowing for higher ODR streaming
- Fixed a bug with out-of-bound array index access
- Added new commands swimver, swimsetfreq, swimgetfreq, swimsetaxes, swimgetaxes, kdisapatt
- Added support for new Klio logging virtual sensor

# v0.4.4
- Fixed a bug where the Wake up Meta events callback was not linked
- Added a clear screen to the initial boot output for the MCU_APP30 target
- Switched COINES submodule to the latest master branch
- Added changes to the sensor API to make it easier to check if a sensor is available
- Changed checking for valid firmware to reading the Feature status rather than Kernel version
- Reduced prints of the bhy2cli PC infos when loading a firmware effectively speeding up loading time
- Added support for Swim
- Refactored some code
- Fixed an issue where the initial accuracy reported a junk value
- Updated to the latest COINES master that added support for unique names for each board
- Fixed a bug where the Green LED didn't flash for the logtxt command
- Fixed an issue where the formatting of the info and ls commands was not uniform
- Completed support for logse (binary logging) by updating the logging format and creating a decompressor
- Deprecated the logtxt command
- Added a command to erase the flash descriptor
- Added more information in the info command
- Moved the cls callback into the common callbacks
- Added support for heartbeat message. The heartbeat message being, '[H]<mcu timestamp ms><\r><\n>
- Moved updating the virtual sensor list from actse to info to reduce actse execution time

# v0.4.3
- Added more Klio commands
- Added new command for version
- Fixed scaling value for PDR outputs

# v0.4.2
- Added PDR commands. 
- Fixed an issue where verbosity worked incorrectly for the PC.
- Updated COINES submodule to latest master

# v0.4.0-beta
 - Fixed verbosity not working as expected
 - Fixed verbose level not reported correctly. Also set default verbose to 0 or no verbose outputs, i.e., no warning or info messages
 - Removed unwanted commands from the PC build
 - Added build option switch for I2C support

# v0.3.0
 - Started to baseline changes.
 - Fixed an issue where disabling the sensor didn't work as expected.