
#ifndef POWERBALLARM_H
#define POWERBALLARM_H

#include <vector>
#include "ipa_canopen_core/canopen.h"


/** Simplified interface to the Schunk Powerball arm. Small layer over the
 * ipa_canopen library, but this could be replaced in the future by a different
 * canopen/301 implementation.  
 * 
 * TODO: direct position control
 */
class Arm
{
public:
	Arm(const char* canif = "can0", size_t numjoints = 6, int firstCANID = 3, unsigned int syncInterval = 20);
  virtual ~Arm();

  /// Open connection to the arm hardware components. Return false on error.
  bool open();

  /// Instead of connecting to hardware, emulate active connection. Call instead
  ///of open() to 
  void emulate() {
    emulated = true;
  }

  /// stop all joints in this arm. 
  void haltAll();

  /** Call regularly to cause all joints to update for any new movement data
 * received. Joints will stop if they stop receiving sync messages. */
  void sync() {
    if(!emulated)
      canopen::sendSync();
  }

  /// Get current joint positions in degrees.
  void getJointPositions(float *pos1, float *pos2, float *pos3, float *pos4, float *pos5, float *pos6)
  {
    std::vector<float> pos = getJointPositions();
    if(pos1) *pos1 = pos[0];
    if(pos2) *pos2 = pos[1];
    if(pos3) *pos3 = pos[2];
    if(pos4) *pos4 = pos[3];
    if(pos5) *pos5 = pos[4];
    if(pos6) *pos6 = pos[5];
  }

  /// Get current joint positions in degrees.
  void getJointPositions(std::vector<float>& pos)
  {
    pos = getJointPositions();
  }

  /// Get current joint positions in degrees.
  std::vector<float> getJointPositions();

  // Begin sending incremented positions to joint over time according to speed
  // (degrees/sec)
  void moveJointBy(size_t joint, float speed);

  // Set or replace desired joint velocity values. 
  // (degrees/sec)
  void setJointVels(const std::vector<float>& speeds);

  void setGripperVel(float speed);

  float getGripperPosition()
  {
    // todo
    return 0;
  }

protected:
  std::vector<canopen::Device> devices;
  std::vector <uint8_t> ids;
  std::string canifName;
  std::string chainName;
  unsigned int syncInterval;
  uint8_t gripCANid;
  canopen::Device gripDevice;
  bool emulated;

public:
  float maxSpeed;
  std::vector<float> minPos;
  std::vector<float> maxPos;
  float gripMinPos;
  float gripMaxPos;
  float gripMaxSpeed;
};


#endif
