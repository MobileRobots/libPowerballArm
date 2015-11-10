
#include <unistd.h>
#include "Arm.h"

int main(int argc, char **argv)
{
  Arm arm;
  if(!arm.open())
  {
    std::cerr << "Error connecting to arm modules\n";
    return 1;
  }
  arm.moveJointsTo(0, 130, 750, 0, 60, 0);
  while(true)
  {
    arm.sync();
    usleep(1000*1000);
    std::vector<float> pos = arm.getJointPositions();
    for(auto i = pos.begin(); i != pos.end(); ++i)
    {
      std::cout << *i << ", ";
    }
    std::cout << "gripper: " << arm.getGripperPosition() << std::endl;
  }
  return 0;
}
