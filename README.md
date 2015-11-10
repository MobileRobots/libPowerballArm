
This is some standalone code to communicate with the Schunk PowerBall LWA4P arm
and CommonplaceRobotics force-torque sensor (FTS).

The code repository is published at
<http://github.com/MobileRobots/libPowerballArm>.  To make and share changes,
you can create a fork of that repository and submit pull requests or 
discuss changes on the aria-users mailing list.

It is based on code from the `ipa_canopen` ROS node
<https://github.com/ipa320/ipa_canopen>, with some modifications to use 
SocketCAN instead of the proprietary Peak `pcan` driver.  `ipa_canopen`
was written by Thiago de Freitas, Tobias Sing, Eduard Herkel and others
at Fraunhofer Institute for Manufacturing Engineering and Automation
(IPA Fraunhofer). This was chosen because it is the simplest/smallest
open source partial-CanOPEN implementation specifically intended for
use with the Schunk LWA4P arm that was easily available as of
October 2015.  This may be replaced by another CanOPEN implementation,
or the whole libPowerballArm library may be replaced by a more complete 
Powerball/CanOPEN-based arm interface, in the future. Notices of
changes or updates will be made at <http://robots.mobilerobotts.com>,
the aria-users mailing list, and/or at <http://github.com/MobileRobots/libPowerballArm>.

To enable some debugging output in the canopen library edit `src/ipa_canopen_core.cpp`
and change the definition of the `DEBUG` symbol to `true`, then rebuild.

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

`can0` will be the device plugged into a prior USB port (e.g. USB port
labelled "J2" on a Mamba onboard computer), and `can1` will be the later
USB port (e.g. USB port labelled "J3" on a Mamba onboard computer).  You
can list USB devices on Linux with the command `lsusb -t`, e.g.:

   lsusb -t

It will show USB CAN interfaces in the order they are attached to USB.
For example, two ESD CAN-USB interfaces attached to J2 and J3 on a Mamba
onboard computer will be listed in the output of `lsusb -t` like this:

    /:  Bus 02.Port 1: Dev 1, Class=root_hub, Driver=ehci_hcd/8p, 480M
        |__ Port 1: Dev 2, If 0, Class=vend., Driver=esd_usb2, 480M
        |__ Port 3: Dev 3, If 0, Class=vend., Driver=esd_usb2, 480M

The library defaults to expecting the arm to be on can0, and the FTS to be
on a second CAN bus, can1.  Alternate interfaces can be specified in the API.

It has been teasted with the ESD USB-CAN adapter (`esdcan` Linux driver module)
and Peak USB-CAN adapter (`pcan_usb` Linux driver module).  

Some simple example programs are available in the `examples` directory.

Each motor in the Powerball arm has a CAN device ID, starting at 3. 
The final joint (wrist rotation) is 8.  The gripper is device id 12. 

    ERB module (ball)        Motor axis    CAN node ID
    --------------------------------------------------
    base                     rotation      0x3
    base                     pivot         0x4
    middle (elbow)           pivot         0x5
    middle (elbow)           rotation      0x6
    end (wrist)              pivot         0x7
    end (wrist)              rotation      0x8

The `ipa_canpen_core` library is capable of moving arm motor axis modules using the
"interpolated position" mode.  Starting at the arm modules's current position,
new positions are sent to the device according to a desired velocity. 
`ipa_canopen_core` automatically increments the position and sends
the new position to the arm in the background.  Your program must initiate
a "sync" message which, when received by the module, causes it to start
or continue moving towards the next position. If no sync messages are
sent the module times out and stops.

Boost thread and system development libraries are requited to build. Boost-system is
required at runtime. To install these on Ubuntu or Debian to build,
    sudo apt-get install libboost-thread-dev libboost-system-dev

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
 
