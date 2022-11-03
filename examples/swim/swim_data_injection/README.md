# COINES example for Data Injection

> This example contains Injection of sensor data from field trial log and logging swim outputs to csv file.
Pre-requisite:
-------------
> Make sure Coines submodule updated to feature/SOLTEAM-1472-migrate-to-coines-sensor-bridge branch
> If not, then checkout this branch before compilation.
> Update the coines-bridge firmware to APP3.0
To Compile:
----------
> mingw32-make

To Execute:
----------
> Connect the device to PC.
> data_injection.exe <path-to-firmware> <injection_log> <pool_length> <Hand_selection> <path-to-output-log.csv>
<pool_length> -> 25/50
<Hand_selection> -> l/r

> Swim sensor data logs to the output log file
---
#### Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved
