# Trinamic_Monopack_V2
Repository for testing a manipulator with two axis each controlled by Trinamic Monopack V2 driver.
Main methods for one driver are implemented in the MonoPack class in monopack_v2.py, in stages_monopack.py the class StagesPI is implemented which uses two MonoPack objects to control two axis. The class StagesPI is used to control the manipulator with two axis.

## monopack_v2.py
This file contains the MonoPack class which is used to control a single Trinamic Monopack V2 driver.