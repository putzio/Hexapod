#pragma once
#include "Servo.h"
// SERVO POSITIONS
#define MASTER_SERVO_MIN_POS 100
#define MASTER_SERVO_MAX_POS 130
#define SLAVE_UP_POSITION 5
enum State
{
    Stop,
    Forward,
    MasterLoop,
    SlaveLoop,
    Back,
    Up,
    Up1,
    Up2,
    Down,
    Down1,
    Down2,
    Left,
    Right,
    ResetMaster,
    ResetSlave,
};
/*
    Servos arrangement in the leg:
    |___O__ <-- Master Servo
       /
      /
     O      <-- Slave Servo
      \
       \
*/
class Leg
{
public:
    Servo master;
    SlaveServo slave;
    int maxPos = MASTER_SERVO_MAX_POS;
    int minPos = MASTER_SERVO_MIN_POS;
    int upPos = SLAVE_UP_POSITION;

    void initLeg()
    {
        master.Enable();
        master.Write(90);
        slave.enableSlave = true;
        slave.Enable();
        slave.SlavePosition(master.position);
    }

public:
    Leg(int pinMaster = 2, int pinSlave = 3, bool leftLeg = false, bool sBack = true, int16_t calibrationMaster = 0, int16_t calibrationSlave = 0)
    {
        master = Servo(pinMaster, leftLeg, calibrationMaster);
        slave = SlaveServo(pinSlave, leftLeg, calibrationSlave, sBack);
        if (!sBack)
        {
            maxPos = 180 - MASTER_SERVO_MIN_POS;
            minPos = 180 - MASTER_SERVO_MAX_POS;
            upPos = 180 - SLAVE_UP_POSITION;
        }
    }
    void ChangeLegVelocityLimits(int v)
    {
        master.ChangeVelocityLimits(v);
        slave.ChangeVelocityLimits(v);
    }
    // Writes master position and if the slave is enabled slave adjusts its angle,
    // so the height of the leg does not change
    void WriteMaster(int position, bool slaveEnabled)
    {
        master.Write(position);
        if (slaveEnabled)
        {
            slave.enableSlave = true;
            slave.SlavePosition(position);
        }
        else
        {
            slave.enableSlave = false;
        }
    }
    // Changes the desired position of the master
    void ChangePosition(uint8_t pos, bool slaveEnabled)
    {
        master.ChangePosition(pos);
        slave.enableSlave = slaveEnabled;
    }

    void GoToPositionMaster()
    {
        master.GoToPosition();
        slave.SlavePosition(map(master.currentPosition, SERVO_MIN_MS, SERVO_MAX_MS, 0.0, 180.0));
    }
    // Changes only Slave desired position
    void ChangePositionSlave(uint8_t pos)
    {
        slave.ChangePosition(pos);
    }
    // Both servos are getting closer to the position set in ChangePosition()
    void GoToPosition()
    {
        if (!master.done)
            master.GoToPosition();
        if (!slave.done)
            slave.GoToPosition();
    }

    // automatically change position depending on the State, enableSlave and Leg properties (minPos, maxPos, upPos)
    void ChooseMove(State s, bool enableSlave)
    {
        switch (s)
        {
        case Forward:
        {
            ChangePosition(maxPos, enableSlave);
            break;
        }
        case Down:
        {
            ChangePositionSlave(slave.Calculate(master.position));
            break;
        }
        case Back:
        {
            ChangePosition(minPos, enableSlave);
            break;
        }
        case Up:
        {
            ChangePositionSlave(upPos);
            break;
        }
        }
    }
    bool Move()
    {
        if (!master.done)
        {
            GoToPositionMaster();
            return false;
        }
        else if (!slave.done)
        {
            GoToPosition();
            return false;
        }
        else
            return true;
    }
    bool MoveDone()
    {
        if (master.done && slave.done)
            return true;
        else
            return false;
    }
    void DisableLeg()
    {
        master.Disable();
        slave.Disable();
    }
};