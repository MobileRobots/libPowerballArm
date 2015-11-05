/*!
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <utility>
#include "ipa_canopen_core/canopen.h"

int main(int argc, char *argv[]) {

    if (argc != 6) {
        std::cout << "Arguments:" << std::endl
                  << "(1) CAN interface name" << std::endl
                  << "(2) CAN device ID" << std::endl
                  << "(3) sync rate [msec]" << std::endl
                  << "(4) target position [rad]" << std::endl
                  << "(5) velocity [rad/sec]" << std::endl
                  << "Example: ./move_device can0 8 10 0.7 0.2" << std::endl;
        return -1;
    }
    std::cout << "Interrupt motion with Ctrl-C" << std::endl;
    std::string canif = std::string(argv[1]);
    uint16_t CANid = std::stoi(std::string(argv[2]));
    canopen::syncInterval = std::chrono::milliseconds(std::stoi(std::string(argv[3])));
    double targetPos = std::stod(std::string(argv[4]));
    double vel = std::stod(std::string(argv[5]));

    canopen::devices[ CANid ] = canopen::Device(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    canopen::incomingPDOHandlers[ 0x180 + CANid ] = [CANid](const TPCANRdMsg m) { canopen::defaultPDO_incoming_status( CANid, m ); };
    canopen::incomingPDOHandlers[ 0x480 + CANid ] = [CANid](const TPCANRdMsg m) { canopen::defaultPDO_incoming_pos( CANid, m ); };
    canopen::sendPos = canopen::defaultPDOOutgoing_interpolated;

    std::string chainName = "test_chain";
    std::vector <uint8_t> ids;
    ids.push_back(CANid);
    std::vector <std::string> j_names;
    j_names.push_back("joint_1");
    canopen::deviceGroups[ chainName ] = canopen::DeviceGroup(ids, j_names);

    canopen::init(canif, chainName, canopen::syncInterval);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    canopen::sendSync();

    canopen::setMotorState((uint16_t)CANid, canopen::MS_OPERATION_ENABLED);

    canopen::Device& dev = canopen::devices[CANid];

    //Necessary otherwise sometimes Schunk devices complain for Position Track Error
    dev.setDesiredPos((double)dev.getActualPos());
    dev.setDesiredVel(0);
    canopen::sendPos((uint16_t)CANid, (double)dev.getDesiredPos());

    std::cout << "Current actual position is " << dev.getActualPos() << std::endl;

    canopen::controlPDO((uint16_t)CANid, canopen::CONTROLWORD_ENABLE_MOVEMENT, 0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    dev.setInitialized(true);

    std::cout << "Sending target position " << targetPos << std::endl;
    dev.setDesiredVel(vel);
    dev.setDesiredPos((double)targetPos);
    canopen::sendPos((uint16_t)CANid, (double)targetPos);
    
    std::cout << "Run loop with Sync messages... press ctrl-c to interrupt" << std::endl;
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        canopen::sendSync();
    }
}
