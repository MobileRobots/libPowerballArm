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

// XXX TODO function to load calibration data from a .dat file


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

#define DEBUG false


//********************************************
// Initialization
FTSLWA::FTSLWA(int _canID, unsigned _rate, bool _applybias)
{
	sensorID = _canID;			// 80 (0x50) is standard CAN ID
	request_counter = 0;
  rate = _rate;
  read_timeout = 50;
  apply_bias = _applybias;
  system_error = false;
  overload_error = false;
  calibration_file_serial_number = -1;
  last_request_counter_received = -1;
  last_request_counter_sent = -1;

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
/*
	for(int i=0; i< FTSLWA_MSG_BUFFER_SIZE; i++){
		msgBuffer[i].LEN = 0;
		msgBuffer[i].ID = i;
		for(int j=0; j<8; j++)
			msgBuffer[i].DATA[j] = 0x00;
	}
*/
  memset(&msg51, 0, sizeof(TPCANMsg));
  memset(&msg52, 0, sizeof(TPCANMsg));
}



//********************************************
// Main communication loop. 
// Reads the sensor values and shows them on the terminal
void FTSLWA::DoComm(){

	// storage necessary for the CAN communication
	int l = 2;				// length for request: 2 byte. When the length is not correct, the sensor will not answer!
	unsigned char req[8] = {7, 1, 0, 0, 0, 0, 0, 0};	
	int tmpTC = 0;

	// Request measurements with an incrementing message identifier
	req[0] = 0x07;						// Request measurements
	req[1] = request_counter;
  last_request_counter_sent = request_counter;
  Wait(rate);						// wait a short time to avoid a crowded bus TODO: don't wait a fixed time, check a timer, in case caller of this function (user program) is waiting or delayed
	WriteMsg(sensorID, l, req);			// write the message to the joint controller

  // Wait for matching replies
  boost::posix_time::ptime start = boost::posix_time::microsec_clock::universal_time();
  msgMutex.lock();
  while(msg51.DATA[0] != request_counter && msg52.DATA[0] != request_counter)
  {
    if(msg51.DATA[0] > 127 || msg52.DATA[0] > 127)
    {
      overload_error = true;
      std::cout << "FTSLWA: Warning: Sensor overload error!" << std::endl;
    }
    else if(msg51.DATA[0] > 63 || msg52.DATA[0] > 63)
    {
      system_error = true;
      std::cout << "FTSLWA: Warning: System error!" << std::endl;
    }
    if(overload_error || system_error) {
      msgMutex.unlock();
      return;
    }
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::time_duration passed = now - start;
    if(passed.total_milliseconds() > rate)
    {
      std::cout << "FTSLWA: Warning: took more than " << passed.total_milliseconds() << "ms to get matching reply messages!" << std::endl;
    }
    if(passed.total_milliseconds() > rate*3)
    {
      std::cout << "FTSLWA: Error: took more than " << passed.total_milliseconds() << "ms to get matching reply messages! Aborting" << std::endl;
      msgMutex.unlock();
      return;
    }
    msgMutex.unlock();
    Wait(5);
    msgMutex.lock(); // for next while condition clause, or for code after while
  }

	// read the answers from the sensor, two messages

	//GetMsg(sensorID+1, &l, data);			// get the last message that has been received
  char *data = (msg51.DATA);
	tmpTC = data[0];					// byte 0 contains the timecode from above and error bits	
	raw[0] = 256.0 * data[1] + data[2];			// three raw measureemnts in this messages, three in the next
	raw[1] = 256.0 * data[3] + data[4];
	raw[2] = 256.0 * data[5] + data[6];
	temp = 256.0 * data[7];					// and the temp-highbyte
  if(DEBUG) printf("FTSLWA: Got raw data from sensorID 0x%x message ID 0x%x: TC+flags=%d tx=%f ty=%f tz=%f\n", sensorID, sensorID+1, tmpTC, raw[0], raw[1], raw[2]);

	//GetMsg(sensorID+2, &l, data);	
  data = (msg52.DATA);
	raw[3] = 256.0 * data[1] + data[2];
	raw[4] = 256.0 * data[3] + data[4];
	raw[5] = 256.0 * data[5] + data[6];
	temp += data[7];					// temp low-byte
	temp = 0.12 * temp - 240.0;				
  if(DEBUG) printf("FTSLWA: Got raw data from sensorID 0x%x message ID 0x%x: rx=%f ry=%f rz=%f temp=%f\n", sensorID, sensorID+2, raw[3], raw[4], raw[5], temp);

  msgMutex.unlock();

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
		--startUpCnt;

	// but then generate the XYZ values
	// first subtract the bias from the current values,
	// then multiply with the calibration matrix
	}else{
    if(apply_bias)
      for(int i=0; i<6; i++)
        raw[i] -= rawBias[i];
		ApplyCalibMatrix();
		//std::cout << raw[0] << " - " << raw[1] << " - "  << raw[2] << " - "  << raw[3] << " - "  << raw[4] << " - "  << raw[5] << std::endl;
		//std::cout << "   x: " << xyz[0] << "   y: " << xyz[1] << "   z: " << xyz[2];
		//std::cout << "  rx: " << xyz[3] << "  ry: " << xyz[4] << "  rz: " << xyz[5] << std::endl;
	}

	request_counter++;							//increment the counter 0-127
	if(request_counter >= 64)
		request_counter = 0;


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
  usleep(ms * 1000l);
/*
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
*/
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
    ctx->ReadData(); // read should block on data 
  }

  std::cout << "FTSLWA: readLoop: finished" << std::endl;
}


void FTSLWA::ReadData()
{
  if(!active) return;
  boost::posix_time::ptime start = boost::posix_time::microsec_clock::universal_time();
  TPCANRdMsg rdmsg;
  int r = LINUX_CAN_Read(itf, &rdmsg); // should block
  //boost::asio::read(*(ctx->port), boost::asio::buffer(bu, 1));
  boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
  boost::posix_time::time_duration passed = now - start;
  long int ms = passed.total_milliseconds();
  //printf("LINUX_CAN_READ returned after %ld ms\n", ms);
  if(ms > read_timeout)
  {
    std::cout << "FTSLWA: Error: took " << ms << " ms to read message, skipping.  timeout is " << read_timeout << std::endl; 
  } else {
    // printf("calling RecvMsg...\n");
    RecvMsg(&(rdmsg.Msg));
  }

}




//***************************************************************
// Connect to a virtual com port with the USB2CAN adapter

bool FTSLWA::Connect(const char* canif)
{ 
  itf = LINUX_CAN_Open(canif, 0);
  if(!itf) return false;
  int e = CAN_Init(itf, 0, CAN_INIT_TYPE_ST);
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
  assert(msg->ID == 0x51 || msg->ID == 0x52);
  int reqctr = msg->DATA[0];
  last_request_counter_received = reqctr;
  if(DEBUG) printf("FTSLWA::RecvMsg called ID=0x%x, len=%d, data[0] (counter)=%d\n", msg->ID, msg->LEN, reqctr);
  assert(reqctr < FTSLWA_MSG_BUFFER_SIZE);
  msgMutex.lock();
  if(msg->ID == 0x51)
    memcpy(&msg51, msg, sizeof(TPCANMsg));
  else
    memcpy(&msg52, msg, sizeof(TPCANMsg));
  msgMutex.unlock();
  // TODO currently assumes messages are received in order and so after reading
  // 0x52, both will have the same counter from the same request.
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
/*
int FTSLWA::GetMsg(int i, int *length, unsigned char* data)
{
  printf("GetMsg(%d)...\n", i);
	if(i>255)
		throw std::string("invalid message index!");
  assert(length);
  assert(data);
	*length = msgBuffer[i].LEN;
  memcpy(data, msgBuffer[i].DATA, 8);
	return 0;
	
}
*/



bool FTSLWA::loadCalibration(const char *filename)
{
  // this uses strtok to to a very hackish parsing of the xml: we lexically check only
  // element names, attribute names, and values, no xml syntax parsing is done.
  // First we expect a token with the value FTSTransferMatrix, the root element
  // name.  Next we expect the attribute SerialNumber, then a token with the
  // serial number value, which is stored.  Next, for each Row* element, we expect the element
  // name, in the form Row[T|R][X|Y|Z], then six attributes with attribute name
  // token in the form ColN where N is a number [1..6].  If we ever reach a
  // token starting with /, we ignore it.  Each attribute starting with the
  // chahracters "Col" selects a column of the matrix for which the next token
  // encountered will be the value.  If a token beginning with "Row" is
  // reached, we select a new row, and continue finding "Col" attributes.
  // Whitespace and XML syntax characters (<, >, ", etc.) are token delimiters
  // and are ignored.  Empty tokens are ignored.
  FILE *fp = fopen(filename, "r");
  if(!fp)
  {
    printf("FTSLWA: loadCalibration: error opening file \"%s\" for reading.\n", filename);
    return false;
  }
  printf("FTSLWA: loading calibration data from \"%s\"...\n", filename);
  char line[1024];
  const char *delim = " \n\t\r<>\"=";
  enum {ROOT, SERIALNUMBERATTR, SERIALNUMBERVAL, ROW, COLATTR, COLVAL} expect;
  expect = ROOT;
  size_t n;
  int row = -1;
  int col = -1;
  clearerr(fp);
  while((n = fread(&line, 1, 1024, fp)) > 0 && !ferror(fp))
  {
    clearerr(fp);
    char *tok = strtok(line, delim);
    if(!tok) 
    {
      printf("FTSLWA: parse error: no valid tokens\n");
      return false;
    }

    do
    {
      if(strcmp(tok, "/FTSTransferMatrix") == 0)
      {
        // end of XML
        return true;
      }

      if(expect == ROOT)
      {
        if(strcmp(tok, "FTSTransferMatrix") == 0)
        {
          expect = SERIALNUMBERATTR;
          continue;
        }
        else
        {
          puts("FTSLWA: loadCalibration: parse error: expected <FTSTransferMatrix...");
          return false;
        }
      } 

      if(expect == SERIALNUMBERATTR)
      {
        if(strcmp(tok, "SerialNumber") == 0)
        {
          expect = SERIALNUMBERVAL;
          continue;
        }
        else
        {
          puts("FTSLWA: loadCalibration: parse error: expected SerialNumber=");
          return false;
        }
      }

      if(expect == SERIALNUMBERVAL)
      {
        calibration_file_serial_number = atol(tok);
        expect = ROW;
        continue; 
      }

      if(expect == ROW)
      {

        if(strncmp(tok, "Row", 3) == 0)
        {
          row = -1;
          if(tok[3] == 'T')
            row = 0;
          else if(tok[3] == 'R')
            row = 3;
          else
          {
            puts("FTSLWA: loadCalibration: parse error: expected <RowT... or <RowR...");
            return false;
          }
          if(tok[4] == 'X')
            row += 0;
          else if(tok[4] == 'Y')
            row += 1;
          else if(tok[4] == 'Z')
            row += 2;
          else
          {
            printf("FTSLWA: loadCalibration: parse error: expected <Row%cX..., <Row%cY..., or <Row%cZ...\n", tok[3], tok[3], tok[3]);
            return false;
          }
          //printf("%s means row %d\n", tok, row);
          expect = COLATTR;
          continue;
        }
        else if (expect == ROW)
        {
          puts("FTSLWA: loadCalibration: parse error: expected <Row...");
          return false;
        }

      }

      if(expect == COLATTR)
      {
        if(strcmp(tok, "/") == 0)
        {
          // got end of RowXX element instead of Col, skip to next Row
          expect = ROW;
          continue;
        }

        assert(strlen(tok) == 4);
        assert(strncmp(tok, "Col", 3) == 0);
        col = tok[3] - 48 - 1; // assume ascii. '0' == 48, but Col1 should be array index 0
        expect = COLVAL;
        continue;
      }  

      if(expect == COLVAL)
      {
        double val = atof(tok);
        //printf("--- calibration matrix [%d][%d] (%d) is %f ---\n", row, col, 6*row+col, val);
        calibMat[6*row+col] = val;
        expect = COLATTR;
        continue;
      }

      if(expect == ROW || expect == COLATTR)
      {
        if(strncmp(tok, "Row", 3) != 0 && strncmp(tok, "Col", 3) != 0 && strcmp(tok, "FTSTransferMatrix") != 0)
        {
          puts("FTSLWA: loadCalibration: parse error: expect Row or Col token, or </FTSTransferMatrix>");
          return false;
        }
      }

    }
    while(tok = strtok(NULL, delim));

  }
  // TODO warn if incomplete parse
  if(ferror(fp))
  {
    printf("FTSLWA: loadCalibration: error reading from file\n");
    return false;
  }
  if(n == 0)
    printf("FTSLWA: loadCalibration: no data\n");
  if(feof(fp))
    printf("FTSLWA: loadCalibration: unexpected end of file, missing </FTSTransferMatrix>?");
  return false;
}

