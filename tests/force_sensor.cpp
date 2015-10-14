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
	ptime start, now;
	time_duration passed;
	
	FTSLWA mySensor;	
  	
  if(mySensor.Connect("can0"))
  {
    puts("connected");
  }
  else
  {
    puts("error opening connection to senson or CANBUS");
    return -1;
  }
	
	while (true)			// this is the main loop
	{				// but all interesting things are done in DoComm()
		start = microsec_clock::universal_time();
		mySensor.DoComm();
		now = microsec_clock::universal_time();
		passed = now - start;

		while( passed.total_milliseconds() < 100){	// set the cycle time here
			now = microsec_clock::universal_time();
			passed = now - start;
      std::cout << "   x: " << mySensor.xyz[0] << "   y: " << mySensor.xyz[1] << "   z: " << mySensor.xyz[2];
      std::cout << "  rx: " << mySensor.xyz[3] << "  ry: " << mySensor.xyz[4] << "  rz: " << mySensor.xyz[5] << std::endl;
      usleep(1000*1000);

		}

	}



}



