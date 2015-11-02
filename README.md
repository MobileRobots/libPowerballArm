
This is a simple library for communicating with the Schunk Powerball LWA4P arm.

It is based on code from the `ipa_canopen` ROS node
<https://github.com/ipa320/ipa_canopen>, with some modifications to use 
SocketCAN instead of the proprietary Peak `pcan` driver.  `ipa_canopen`
was written by Thiago de Freitas, Tobias Sing, Eduard Herkel and others
at Fraunhofer Institute for Manufacturing Engineering and Automation
(IPA Fraunhofer). This was chosen because it is the simplest/smallest
open source partial-CanOPEN implementation easily available as of
October 2015.  This may be replaced by other CanOPEN implementations,
or the whole libPowerballArm library may be replaced by a more complete 
Powerball/CanOPEN-based arm interface, in the future.

This library also includes an interface for the CPR Force-Torque Sensor
for Schunk LWA arms (called FTS-LWA or FTL in some documentation).

This library relies on Linux kernel CAN support (SocketCAN), i.e. the 
`can0` socket interface must be available through Linux.  See
<http://robots.mobilerobots.com/wiki/Linux_SocketCAN> for information on 
SocketCAN including how to configure the CAN interface(s) using the Linux
`ip` command. To bring up the first CAN interface up, use these commands:

   sudo ip link set can0 type can bitrate 500000
   sudo ip link set up can0

The Schunk LWA4P arm and FTS-LWA force-torque sensor both use bitrates
of 500000 (500K) on the CAN bus.

To bring up a second CAN interface (e.g. force-torque sensor or hand):

   sudo ip link set can1 type can bitrate 500000
   sudo ip link set up can1

You can put these commands in `/etc/rc.local` to run when the system boots up. 

The library defaults to expecting the arm to be on can0, and the FTS to be
on a second CAN bus, can1.  Alternate interfaces can be specified in the API.

It has been teasted with the ESD USB-CAN adapter (`esdcan` Linux driver module)
and Peak USB-CAN adapter (`pcan_usb` Linux driver module).  

See `include/Arm.h` for easy to use API for the arm.  Only some
features of the arm are currently available in this API.  You may implement
additiontal arm features by adding to this class.  If you do make any 
improvements or fixes, please submit a pull request from your fork, or
post the patches on github, or send them to support@mobilerobots.com.

Some simple example programs are available in the `examples` directory.

The Arm class uses the basic canopen implementation contained in the `canopen`
class defined in `ipa_canopen_core.h`.  Some test programs for this CANopen
layer can be found in `tests`.  Each joint in the Powerball arm has a CAN
device ID, starting at 3. The final joint (wrist rotation) is 8. 
The gripper is device id 12. 

It has been teasted with the ESD USB-CAN adapter (`esdcan` Linux driver module)
and Peak USB-CAN adapter (`pcan_usb` Linux driver module).  

This code is distributed under the terms of the GNU Lesser General Public
License (LGPL).  See the LICENSE file for full license details.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer. 
    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution. 
    - Neither the name of the Fraunhofer Institute for Manufacturing
      Engineering and Automation (IPA) nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission. 

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License LGPL as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License LGPL for more details.

You should have received a copy of the GNU Lesser General Public
License LGPL along with this program.
If not, see <http://www.gnu.org/licenses/>.
 
