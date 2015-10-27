#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "FTSLWA.h"


int main( int argc, char** argv )
{
		
	using namespace boost::posix_time;
	
	FTSLWA mySensor;	

  if(!mySensor.loadCalibration("force_sensor_calib.dat"))
  {
    puts("error loading calibration data");
    return -2;
  }
  	
  if(mySensor.Connect("can0"))
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
		if( passed.total_milliseconds() >= 1000){	// print output only every 1 sec
      last = microsec_clock::universal_time();
      std::cout << "   x: " << mySensor.xyz[0] << "   y: " << mySensor.xyz[1] << "   z: " << mySensor.xyz[2];
      std::cout << "  rx: " << mySensor.xyz[3] << "  ry: " << mySensor.xyz[4] << "  rz: " << mySensor.xyz[5] << std::endl;
		}
	}



}



