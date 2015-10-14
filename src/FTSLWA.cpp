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

// XXX TODO remove use of pcan_compat functions and just use CAN socket
// interface directly; use BCM (broadcast manager) configuration to 
// have Linux automatically send requests for data


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <iostream>
#include <stdexcept>
//#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
//#include <boost/lexical_cast.hpp>


#include "boost/date_time/posix_time/posix_time.hpp"
//#include "boost/date_time/gregorian/gregorian.hpp"

#include "ipa_canopen_core/pcan_compat.h"

#include "FTSLWA.h"



//********************************************
// Initialization
FTSLWA::FTSLWA(int _canID, int _baud)
{
	sensorID = _canID;			// 80 (0x50) is standard CAN ID
  baud = _baud;
	cnt = 0;

	for(int i=0; i<6; i++){
		xyz[i] = 0.0;
		raw[i] = 0.0;
		rawBias[i] = 0.0;
	}
	temp = 0.0;

	// the calibration matrix has to be adapted to the specific sensor!
	// It can be found on the CD root directory, on the quality assurance sheet, and in the file configuration.xml of SensorViz
	calibMat[0] = 0.00074; calibMat[1] = 0.15425; calibMat[2] = -0.01197; calibMat[3] = 0.26927; calibMat[4] = -0.00961; calibMat[5] = 0.16185;
	calibMat[6] = -0.00297; calibMat[7] = 0.18613; calibMat[8] = -0.00195; calibMat[9] = -0.19274; calibMat[10] = 0.01098; calibMat[11] = -0.39257;
	calibMat[12] = -0.07788; calibMat[13] = -0.07254; calibMat[14] = 0.08492; calibMat[15] = -0.02224; calibMat[16] = -0.07898; calibMat[17] = 0.04599;
	calibMat[18] = -0.05767; calibMat[19] = -0.17203; calibMat[20] = 1.16446; calibMat[21] = -0.14635; calibMat[22] = 1.29975; calibMat[23] = -0.04851;
	calibMat[24] = 1.76953; calibMat[25] = 1.36467; calibMat[26] = 0.15277; calibMat[27] = 0.39187; calibMat[28] = -0.37144; calibMat[29] = -0.89138;
	calibMat[30] = -0.04327; calibMat[31] = -1.33824; calibMat[32] = -0.10269; calibMat[33] = 2.66601; calibMat[34] = -0.26077; calibMat[35] = -0.65777;
	
	startUpCnt = 15;	// 15 values will be used to gather the bias

	// init the message buffer
	for(int i=0; i< FTSLWA_MSG_BUFFER_SIZE; i++){
		msgBuffer[i].LEN = 0;
		msgBuffer[i].ID = i;
		for(int j=0; j<8; j++)
			msgBuffer[i].DATA[j] = 0x00;
	}
}



//********************************************
// Main communication loop. 
// Reads the sensor values and shows them on the terminal
void FTSLWA::DoComm(){

	// storage necessary for the CAN communication
	int l = 2;				// length for request: 2 byte. When the length is not correct, the sensor will not answer!
	unsigned char data[8] = {7, 1, 0, 0, 0, 0, 0, 0};	
	int tmpTC = 0;

	// Request measurements with an incrementing message identifier
	data[0] = 7;						// Request measurements
	data[1] = cnt;
	WriteMsg(sensorID, l, data);			// write the message to the joint controller
	Wait(10);						// wait a short time to avoid a crowded bus

	// read the answers from the sensor, two messages
	GetMsg(sensorID+1, &l, data);			// get the last message that has been received
	tmpTC = data[0];					// byte 0 contains the timecode from above and error bits	
	raw[0] = 256 * data[1] + data[2];			// three raw measureemnts in this messages, three in the next
	raw[1] = 256.0 * data[3] + data[4];
	raw[2] = 256.0 * data[5] + data[6];
	temp = 256.0 * data[7];					// and the temp-highbyte

	GetMsg(sensorID+2, &l, data);	
	raw[3] = 256.0 * data[1] + data[2];
	raw[4] = 256.0 * data[3] + data[4];
	raw[5] = 256.0 * data[5] + data[6];
	temp += data[7];					// temp low-byte
	temp = 0.12 * temp - 240.0;				

	// Error Check:
	// tmpTC is the first byte of the answers
	// it contains the time code (second byte sent to the sensor) and
	// error messages: bit 6 for Overload, bit 7 for general sensor error
        if (tmpTC > 127)
        {
		std::cout << "FTSLWA: !!! Sensor Error !!!"<< std::endl;               
		tmpTC -= 127;
        }
    	if (tmpTC > 63)
    	{
		std::cout << "FTSLWA: !!! Overload !!!"<< std::endl;               
        	tmpTC -= 63;
    	}


	
	// Use the first 15 measurements to compute the bias value
	if(startUpCnt > 0){
		std::cout << "FTSLWA: Setting bias..." << std::endl;
		if(startUpCnt <= 10){		// the first 5 measurements are not used

			for(int i=0; i<6; i++)		
				rawBias[i] += raw[i];
		
			if(startUpCnt == 1){
				for(int i=0; i<6; i++)
					rawBias[i] /= 10;
				std::cout<<"FTSLWA: Bias values: "<<rawBias[0]<<" - "<<rawBias[1]<<" - "<<rawBias[2]<<" - "<<rawBias[3]<<" - "<<rawBias[4]<<" - "<<rawBias[5]<<std::endl;
			}		
		}
		startUpCnt--;

	// but then generate the XYZ values
	// first subtract the bias from the current values,
	// then multiply with the calibration matrix
	}else{
		for(int i=0; i<6; i++)
			raw[i] -= rawBias[i];
		ApplyCalibMatrix();
		//std::cout << raw[0] << " - " << raw[1] << " - "  << raw[2] << " - "  << raw[3] << " - "  << raw[4] << " - "  << raw[5] << std::endl;
		//std::cout << "   x: " << xyz[0] << "   y: " << xyz[1] << "   z: " << xyz[2];
		//std::cout << "  rx: " << xyz[3] << "  ry: " << xyz[4] << "  rz: " << xyz[5] << std::endl;
	}

	cnt++;							//increment the message identifier 0-127
	if(cnt >= 64)
		cnt = 0;


	return;
}



//*************************************************+
// Matrix multiplication: xyz = caliMat * raw
void FTSLWA::ApplyCalibMatrix(){

	for(int i=0; i<6; i++){
		xyz[i] = 0.0;
		for(int j=0; j<6; j++){
			xyz[i] += calibMat[6*i+j] * raw[j]; 
		}
	}
}




//********************************************
void FTSLWA::Wait(int ms){
	using namespace boost::posix_time;
	ptime start, now;
	time_duration passed;

	start = microsec_clock::universal_time();
	now = microsec_clock::universal_time();
	passed = now - start;
	while( passed.total_milliseconds() < ms){
		now = microsec_clock::universal_time();
		passed = now - start;
	}

}





//***************************************************************
// waits for incoming bytes and stores them when a complete messages (11 bytes)
// has been received.
void readLoop(void * context )
{
	FTSLWA *ctx = (FTSLWA*)context;

  //unsigned char bu[11];
  //unsigned char buffer[11];
  //int bufferCnt = 0;
  //int tmpid = 0;
  
  std::cout << "FTSLWA: readLoop started" << std::endl;

  while (true)
  {

    ctx->ReadData();
  }

  std::cout << "FTSLWA: readLoop: finished" << std::endl;
}


void FTSLWA::ReadData()
{
  if(!active) return;
  boost::posix_time::ptime start = boost::posix_time::microsec_clock::universal_time();
  TPCANRdMsg rdmsg;
  int r = LINUX_CAN_Read(itf, &rdmsg);
  //boost::asio::read(*(ctx->port), boost::asio::buffer(bu, 1));
  boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
  boost::posix_time::time_duration passed = now - start;
  if(passed.total_milliseconds() > 2){		//out of sync!!!
    std::cout << "FTSLWA: too long to read message " << std::endl; 
  } else {
    RecvMsg(&(rdmsg.Msg));
  }

}




//***************************************************************
// Connect to a virtual com port with the USB2CAN adapter

bool FTSLWA::Connect(const char* canif)
{ 
  itf = LINUX_CAN_Open(canif, 0);
  if(!itf) return false;
  int e = CAN_Init(itf, baud, CAN_INIT_TYPE_ST);
  if(e == -1)
    return false;
  active = true;

/*
	io_service io;
	const char *PORT = COMPORT;
	serial_port_base::baud_rate baud_option(460800);
	active = false;
	string s;
	try{
		port = new serial_port(io);
		port->open(PORT);
		port->set_option(baud_option); // set the baud rate

		active = true;
		std::cerr << "Port " << PORT << " opened\n";
	
	}catch (boost::system::system_error &e){
		boost::system::error_code ec = e.code();
		std::cerr << "Cannot open port " << PORT << " - error code: " << ec.category().name() << std::endl;
	}catch(std::exception e){
		std::cerr << "Cannot open port " << PORT << " - error code: " << e.what() << endl;
		
	}
*/

	boost::thread readThread(readLoop, (void*)this);

  std::cout << "FTSLWA: connected and running" << std::endl;

	return true;
}

//***************************************************************
void FTSLWA::Disconnect()
{
  if(active)
    CAN_Close(itf);
  active = false;
/*
	if(port->is_open()){
		active = false;
		port->close();
		std::cout << "FTSLWA: Port closed";
	}
*/
}


//***************************************************************
/*
 * Checks for validity of the messages and stores them in a buffer for later access
 * Validity check here is based on checsum bit; 
 * !!! Check out for conversion types, thes checksum has to be added from unsigned chars!!!
 */
//int FTSLWA::EvaluateBuffer(unsigned char* buf){
//
//	int i = 0;
//	int mid = (int)buf[0];
//	int length = (int)buf[1];
//	int cs = (unsigned char)buf[10];		// checksum byte
//
//	int sum = 0;					// genereate a simple checksum bit from the
//	for(i=0; i<10; i++)				// received data
//		sum += (unsigned char)buf[i];
//	sum = sum % 256;
//
//	
//	if(sum == cs){					// compare it with the remote generated
//		msgBuffer[mid].length = length; 	// checksum bit
//		msgBuffer[mid].id = mid;
//		for(i=0; i<8; i++)
//			msgBuffer[mid].data[i] = buf[i+2];
//	}
//
//	return 0;
//}

int FTSLWA::RecvMsg(TPCANMsg *msg)
{
  int mid = msg->DATA[0];
  assert(mid < FTSLWA_MSG_BUFFER_SIZE);
  printf("rcv message id %d\n", mid);
  memcpy(&(msgBuffer[mid]), msg, sizeof(TPCANMsg));
  return 0;
}


//***************************************************************
void FTSLWA::WriteMsg(int id, int length, unsigned char* data)
{
  assert(length <= 8);
  if(!active) return;
  TPCANMsg msg;
  msg.ID = id;
  msg.LEN = length;
  memcpy(msg.DATA, data, length);
  CAN_Write(itf, &msg);
/*

	unsigned char commands[11] = {0,0,0,0,0,0,0,0,0,0,0};
	int sum = 0;

	commands[0] = id;
	commands[1] = length;
	for(int i=0; i<8; i++)
		commands[2+i] = data[i];

	for(int i=0; i<10; i++)			// compute the check byte
		sum += commands[i];
	sum = sum % 256;
	commands[10] = sum;	

	if(active)
		boost::asio::write(*port, boost::asio::buffer(commands, 11));

	//std::cerr << "write: "<<id<<"\n";
	return;
*/
}



//***************************************************************
// Forwards a received message from the buffer
int FTSLWA::GetMsg(int id, int *length, unsigned char* data)
{
	if(id>255)
		throw std::string("invalid message id!");


	length[0] = msgBuffer[id].LEN;
	for(int i=0; i<8; i++)
		data[i] = msgBuffer[id].DATA[i];
	
	return 0;
	
}








