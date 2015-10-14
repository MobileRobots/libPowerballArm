/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * FTSLWA
 * Version 1.2	June 12th, 2013    
 *
 * Test program to show how to interface with the force torque sensor Schunk FTL. 
 * The program has to be started in a terminal window.
 * Written and tested on Ubuntu 11.10, gcc
 * Libraries: boost
 *
 * Functionalities: 
 * 	- establish a connection to the sensor 
 * 	- establish a communication loop and request force measurements
 *
 *
 */


#ifndef FTSLWA_H
#define FTSLWA_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <iostream>
#include <stdexcept>
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/lexical_cast.hpp>


#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/gregorian/gregorian.hpp"


#include "ipa_canopen_core/pcan_compat.h"


#define FTSLWA_MSG_BUFFER_SIZE 256

class FTSLWA
{
public:
 	FTSLWA(int canID = 80, int baud = 460800);

  // call to open connection. return false on error
	bool Connect(const char* canif = "can1");
	void Disconnect(void);

  // call periodically to get new data
	void DoComm(void);

  ~FTSLWA() {
    Disconnect();
  }

  // calibrated received sensor data.  first six values are translation, last
  // six are rotations.
	double xyz[6];

  // raw uncalibrated data
	double raw[6];

  // callibration values.  access this to set calibration for an individual sensor
	double calibMat[36];

	bool active; // remains true while this object is still operating

  void ReadData();

private:
  	
	void ApplyCalibMatrix();
	void Wait(int ms);

	HANDLE itf;			// CAN interface

	int sensorID;			// CAN ID of the sensor
	int cnt;
  int baud;

	double rawBias[6];
	int startUpCnt;
	double temp;  

	void WriteMsg(int id, int length, unsigned  char* data);
	//int EvaluateBuffer(unsigned char* buf);
  int RecvMsg(TPCANMsg *msg);
	int GetMsg(int id, int *length, unsigned  char* data);

	TPCANMsg msgBuffer[FTSLWA_MSG_BUFFER_SIZE];
	

};


#endif
