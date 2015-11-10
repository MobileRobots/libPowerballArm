/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2013 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: ipa_canopen
 * \note
 *   ROS stack name: ipa_canopen
 * \note
 *   ROS package name: ipa_canopen_core
 *
 * \author
 *   Author: Thiago de Freitas Oliveira Araujo, email:tdf@ipa.fhg.de
 * \author
 *   Supervised by: Thiago de Freitas Oliveira Araujo, email:tdf@ipa.fhg.de
 *
 * \date Date of creation: July 2013
 *
 * \brief
 *   Get errors from the canopen device
 *
 *****************************************************************
 *
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
 ****************************************************************/

#include <utility>
#include <iostream>
#include <iomanip>
#include "ipa_canopen_core/canopen.h"
#include <sstream>
#include <stdlib.h>

int main(int argc, char *argv[])
{

    if (argc != 3) {
        std::cout << "Arguments:" << std::endl
        << "(1) CAN interface" << std::endl
        << "(2) CAN deviceID" << std::endl
        << "Example: ./get_error can0 12" << std::endl;
        return -1;
    }

    canopen::NMTmsg.ID = 0;
    canopen::NMTmsg.MSGTYPE = 0x00;
    canopen::NMTmsg.LEN = 2;

    canopen::syncMsg.ID = 0x80;
    canopen::syncMsg.MSGTYPE = 0x00;

    canopen::syncMsg.LEN = 0x00;

    std::string ifname = std::string(argv[1]);

    if (!canopen::openConnection(ifname, "")) {
        std::cout << "Cannot open CAN interface; aborting." << std::endl;

        exit(EXIT_FAILURE);
    }
    else{
        std::cout << "Connection to CAN bus established" << std::endl;
    }

    uint16_t CANid = strtol(argv[2], NULL, 0);
    canopen::syncInterval = std::chrono::milliseconds((100));


    canopen::devices[ CANid ] = canopen::Device(CANid);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::string chainName = "test_chain";
    std::vector <uint8_t> ids;
    ids.push_back(CANid);
    std::vector <std::string> j_names;
    j_names.push_back("joint_1");
    canopen::deviceGroups[ chainName ] = canopen::DeviceGroup(ids, j_names);

    canopen::init(ifname ,chainName, canopen::syncInterval);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    std::shared_ptr<TPCANRdMsg> m;

    /***************************************************************/
    //		Manufacturer specific errors register
    /***************************************************************/

    std::vector<char> dev_name = canopen::devices[CANid].getManufacturerDevName();

    std::cout << dev_name[0] << std::endl;

//    /**************************
//     * Hardware and Software Information
//    *************************/

//    std::vector<uint16_t> vendor_id = canopen::obtainVendorID(CANid);
//    uint16_t rev_number = canopen::obtainRevNr(CANid);
//    std::vector<uint16_t> product_code = canopen::obtainProdCode(CANid);
//    std::vector<char> manufacturer_device_name = canopen::obtainManDevName(CANid);
//    std::vector<char> manufacturer_hw_version =  canopen::obtainManHWVersion(CANid);
//    std::vector<char> manufacturer_sw_version =  canopen::obtainManSWVersion(CANid);

//        /****
//         *Printing the data
//         */

//        std::cout << "vendor_id=0x";

//        for (auto it : vendor_id)
//        {
//           std::cout <<  std::hex << it;
//        }

//        std::cout << std::endl;

//        std::cout << "revision_number: "<< std::hex << int(rev_number) << std::dec << std::endl;
//        std::cout << "device_name:";

//        for (auto it : manufacturer_device_name)
//        {
//           std::cout << it;
//        }

//        std::cout << std::endl;

//        std::cout << "hardware_version:";

//        for (auto it : manufacturer_hw_version)
//        {
//           std::cout << it;
//        }

//        std::cout << std::endl;

//        std::cout << "software_version:";

//        for (auto it : manufacturer_sw_version)
//        {
//           std::cout << it;
//        }

//        std::cout << std::endl;


}
