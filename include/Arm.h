
#ifndef POWERBALLARM_H
#define POWERBALLARM_H

#include <vector>
#include <canopen_402/motor.h>


class Arm
{
public:
	Arm(const char* canif = "can0");
  virtual ~Arm();
  bool open();
  bool isOpen();
  void halt();
  void moveJointsTo(float pos1, float pos2, float pos3, float pos4, float pos5) 
  {
    std::vector<float> p;
    p.push_back(pos1);
    p.push_back(pos2);
    p.push_back(pos3);
    p.push_back(pos4);
    p.push_back(pos5);
    moveArm(p);
  }

  void moveJointsTo(std::vector<float> pos)
  {
    for(size_t i = 0; i < joint_motors.size() && i < pos.size(); ++i)
    {
      moveJointTo(i, pos[i]);
    }
  }

  void moveJointTo(size_t joint, float pos)
  {
    joint_motors[joint].setTarget(pos);
  }

  void getJointPositions(float *pos1, float *pos2, float *pos3, float *pos4, float *pos5)
  {
    std::vector<float> pos = getJointPositions();
    if(pos1) *pos1 = pos[0];
    if(pos2) *pos2 = pos[1];
    if(pos3) *pos3 = pos[2];
    if(pos4) *pos4 = pos[3];
    if(pos5) *pos5 = pos[4];
  }

  void getJointPositions(std::vector<float>& pos)
  {
    pos = getJointPositions();
  }
  std::vector<float> getJointPositions();
protected:
  std::vector<canopen::Motor402> joint_motors;
};


#endif
