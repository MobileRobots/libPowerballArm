#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "FTSLWA.h"

/* This program shows how to get data from the Commonplace Robotics / Schunk
 * force-torque sensor (FTS).  By default will try to use the can1 CAN
 * interface, to use an alternate, give its name on the command line, e.g.:
 *    ./force_sensor can0
 * The CAN interface must be configured and up. See the README file for details.
 * DoComm() must be called as frequently as possible to get frequent data from
 * the sensor. Data is printed every 1/4 of a second on one line, so make
 * your terminal window wide.
 */

int main( int argc, char** argv )
{
		
	using namespace boost::posix_time;
	
	FTSLWA mySensor;	
  mySensor.apply_bias = false;

  if(!mySensor.loadCalibration("force_sensor_calib.dat"))
  {
    puts("error loading calibration data");
    return -2;
  }
  	
  const char *canif = "can1";
  if(argc > 1)
  {
    if(strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)
    {
      puts("Usage: force_sesnsor [can_interface]");
      return 0;
    }
    canif = argv[1];
  }

  if(mySensor.Connect(canif))
  {
    puts("connected");
  }
  else
  {
    puts("error opening connection to senson or CANBUS");
    return -1;
  }


  ptime last = microsec_clock::universal_time();
  ptime now;
	time_duration passed;
	while (true)		
	{		
		mySensor.DoComm(); // checks for new messages and stores them, periodically sends request for new messages
		now = microsec_clock::universal_time();
		passed = now - last;
		if( passed.total_milliseconds() >= 250) {	
      last = microsec_clock::universal_time();
      printf("x:% 8.2f y:% 8.2f z:% 8.2f rx:% 8.2f ry:% 8.2f rz:% 8.2f temp:% 3.1f sent: %2d recvd: %2d   \r", 
        mySensor.xyz[0],
        mySensor.xyz[1],
        mySensor.xyz[2],
        mySensor.xyz[3],
        mySensor.xyz[4],
        mySensor.xyz[5],
        mySensor.temp,
        mySensor.last_request_counter_sent, 
        mySensor.last_request_counter_received
      );
      fflush(stdout);
		}
	}



}



