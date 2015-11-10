

#include "Arm.h"
#include "Aria.h"
#include <stdio.h>

/* This example uses ArKeyHandler from ARIA. ARIA is not required to control the
 * arm however. */

#define DISPLAY_POSITIONS false

Arm arm;
int selectedPivotJoint = 0;
int selectedRotateJoint = 1;
std::vector<float> desiredVels;
float desiredGrip = 0;

const float increment = 1; // degrees/s
const float gripIncrement = 10; // mm/s

void increasePivot()
{
  desiredVels[selectedPivotJoint] += increment;
  arm.setJointVels(desiredVels);
}

void decreasePivot()
{
  desiredVels[selectedPivotJoint] -= increment;
  arm.setJointVels(desiredVels);
}

void increaseRotate()
{
  desiredVels[selectedRotateJoint] += increment;
  arm.setJointVels(desiredVels);
}

void decreaseRotate()
{
  desiredVels[selectedRotateJoint] -= increment;
  arm.setJointVels(desiredVels);
}

void increaseGrip()
{
  desiredGrip += gripIncrement; 
  arm.setGripperVel(desiredGrip);
}

void decreaseGrip()
{
  desiredGrip -= gripIncrement; 
  arm.setGripperVel(desiredGrip);
}

void selectBase()
{
  arm.haltAll();
  selectedRotateJoint = 0; // first joint can id 3
  selectedPivotJoint = 1; // Second joint can id 4
}

void selectElbow()
{
  arm.haltAll();
  selectedPivotJoint = 2; // third joint can id 5
  selectedRotateJoint = 3; // forth joint can id 6
}

void selectWrist()
{
  arm.haltAll();
  selectedPivotJoint = 4; // fifth joint can id 7
  selectedRotateJoint = 5; // sixth joint can id 8
}

int main(int argc, char **argv)
{

  desiredVels.resize(5);

  puts("Connecting... press ctrl-c to cancel");
  if(!arm.open())
  {
    puts("Error opening arm");
    return 1;
  }

  puts("\nConnected to arm.\nPress spacebar or exit program (ESC) to stop arm.\n");
  puts("  b/1/2      Select base joints (joint 1 rotation, joint 2 pivot)");
  puts("  e/2/3      Select elbow joints (joint 3 pivot, joint 4 rotation)");
  puts("  w/4/5      Select wrist joints (joint 5 pivot, joint 6 rotation)");
  puts("  UP/DOWN    Change pivot speed");
  puts("  LEFT/RIGHT Change rotate speed");
  puts("  SPACE      Stop");
  puts("  g/G        Gripper");
  puts("\npress ESC or q to exit program\n");

  ArKeyHandler keys;
  keys.addKeyHandler(ArKeyHandler::SPACE, new ArFunctorC<Arm>(&arm, &Arm::haltAll));
  keys.addKeyHandler(ArKeyHandler::UP,  new ArGlobalFunctor(&increasePivot));
  keys.addKeyHandler(ArKeyHandler::DOWN,  new ArGlobalFunctor(&decreasePivot));
  keys.addKeyHandler(ArKeyHandler::LEFT,  new ArGlobalFunctor(&decreaseRotate));
  keys.addKeyHandler(ArKeyHandler::RIGHT,  new ArGlobalFunctor(&increaseRotate));
  keys.addKeyHandler('G', new ArGlobalFunctor(&increaseGrip));
  keys.addKeyHandler('g', new ArGlobalFunctor(&decreaseGrip));
  keys.addKeyHandler('b', new ArGlobalFunctor(&selectBase));
  keys.addKeyHandler('e', new ArGlobalFunctor(&selectElbow));
  keys.addKeyHandler('w', new ArGlobalFunctor(&selectWrist));

  keys.addKeyHandler('1', new ArGlobalFunctor(&selectBase));
  keys.addKeyHandler('2', new ArGlobalFunctor(&selectBase));
  keys.addKeyHandler('3', new ArGlobalFunctor(&selectElbow));
  keys.addKeyHandler('4', new ArGlobalFunctor(&selectElbow));
  keys.addKeyHandler('5', new ArGlobalFunctor(&selectWrist));
  keys.addKeyHandler('6', new ArGlobalFunctor(&selectWrist));
  
  keys.addKeyHandler(ArKeyHandler::ESCAPE, new ArGlobalFunctor1<int>(&Aria::exit, 0));
  keys.addKeyHandler('q', new ArGlobalFunctor1<int>(&Aria::exit, 0));

  std::vector<float> pos;
  int ctr = 0;
  while(true)
  {
    ArUtil::sleep(20);
    keys.checkKeys();
    arm.sync();

    if(DISPLAY_POSITIONS && ++ctr > 500)
    {
      pos = arm.getJointPositions();
      for(std::vector<float>::const_iterator i = pos.begin(); i != pos.end(); ++i)
      {
        int n = (i - pos.begin());
        if(n == selectedPivotJoint || n == selectedRotateJoint)
          printf("[% 3.1f]", *i);
        else
          printf(" % 3.1f ", *i);
      }
      printf(" press ESC to exit, SPACE to halt arm\r");
      ctr = 0;
    }
    
  }

  return 0;
}

