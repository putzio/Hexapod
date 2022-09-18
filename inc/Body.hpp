#include "Leg.hpp"

enum Mode
{
    StopMode,
    ForwardMode,
    BackMode,
    LeftMode,
    RightMode,
    ResetMode,
    Pos90Mode,
};
Mode mode = Pos90Mode;
const State forwardStates[] = {
    Stop,
    Forward,
    MasterLoop,
    Down1,
    SlaveLoop,
    Down2,
    SlaveLoop,
    Back,
    MasterLoop,
    Up1,
    SlaveLoop,
    Up2,
    SlaveLoop};
const State backStates[] = {
    Stop,
    Forward,
    MasterLoop,
    Up1,
    SlaveLoop,
    Up2,
    SlaveLoop,
    Back,
    MasterLoop,
    Down1,
    SlaveLoop,
    Down2,
    SlaveLoop};
const State leftStates[] = {
    Stop,
    Right,
    MasterLoop,
    Up1,
    SlaveLoop,
    Up2,
    SlaveLoop,
    Left,
    MasterLoop,
    Down1,
    SlaveLoop,
    Down2,
    SlaveLoop};
const State rightStates[] = {
    Stop,
    Right, // L1 Back
    MasterLoop,
    Down1,
    SlaveLoop,
    Down2,
    SlaveLoop,
    Left, // L1 Forward
    MasterLoop,
    Up1,
    SlaveLoop,
    Up2,
    SlaveLoop};
const State resetStates[] = {
    Stop,
    ResetMaster,
    MasterLoop,
    ResetSlave,
    SlaveLoop,
    Stop};
const State pos90States[] = {
    Stop,
    SlaveLoop,
    Stop};
/*
    Leg arrangement:
              ____
    leg 0   -|    |-    leg 1
    leg 2   -|    |-    leg 3
    leg 4   -|____|-    leg 5
*/
class Body
{
private:
    uint8_t legTeam1[3] = {0, 3, 4};
    uint8_t legTeam2[3] = {1, 2, 5};
    State movingStates[ARRAY_SIZE(forwardStates)];
    Leg legs[6];
    uint8_t slowModeLegMove = 0; // leg:0-5 is movedover the ground
public:
    Mode modeType = StopMode;
    int step = 0;
    bool reset = false;
    Body(uint8_t masterPins[6], uint8_t slavePins[6], const int16_t calibration[12])
    {
        for (int i = 0; i < ARRAY_SIZE(legs); i++)
        {
            bool left = (i % 2 == 1);      // left legs are leg 1,3,5
            bool oppositeSlave = !(i < 4); // for first 4 legs the slave servo is positioned the opposite way
            legs[i] = Leg(masterPins[i], slavePins[i], left, oppositeSlave, calibration[i * 2], calibration[i * 2 + 1]);
        }

        for (int i = 0; i < ARRAY_SIZE(legs); i++)
        {
            legs[i].initLeg();
            sleep_ms(POS_90_TIME);
        }
    }
    void ChangeBodyVelocityLimits(int v)
    {
        for (int i = 0; i < ARRAY_SIZE(legs); i++)
        {
            legs[i].ChangeLegVelocityLimits(v);
        }
    }
    void ChangeToForward()
    {
        for (int i = 0; i < ARRAY_SIZE(forwardStates); i++)
        {
            movingStates[i] = forwardStates[i];
        }
        modeType = ForwardMode;
        step = 1;
        if (reset == false)
        {
            ChangeToResetTemp();
        }
        else
            reset = false;
    }
    void ChangeToBack()
    {
        for (int i = 0; i < ARRAY_SIZE(forwardStates); i++)
        {
            movingStates[i] = backStates[i];
        }
        modeType = BackMode;
        step = 1;
        if (reset == false)
        {
            ChangeToResetTemp();
        }
        else
            reset = false;
    }
    void ChangeToLeft()
    {
        for (int i = 0; i < ARRAY_SIZE(forwardStates); i++)
        {
            movingStates[i] = leftStates[i];
        }
        modeType = LeftMode;
        step = 1;
        if (reset == false)
        {
            ChangeToResetTemp();
        }
        else
            reset = false;
    }
    void ChangeToRight()
    {
        for (int i = 0; i < ARRAY_SIZE(forwardStates); i++)
        {
            movingStates[i] = rightStates[i];
        }
        modeType = RightMode;
        step = 1;
        if (reset == false)
        {
            ChangeToResetTemp();
        }
        else
            reset = false;
    }
    void ChangeToStop()
    {
        step = 0;
        modeType = StopMode;
    }
    void ChangeToReset()
    {
        for (int i = 0; i < ARRAY_SIZE(resetStates); i++)
        {
            movingStates[i] = resetStates[i];
        }
        modeType = ResetMode;
        step = 1;
    }
    void ChangeToResetTemp()
    {
        for (int i = 0; i < ARRAY_SIZE(resetStates); i++)
        {
            movingStates[i] = resetStates[i];
        }
        step = 1;
    }
    void ChangeTo90()
    {
        for (int i = 0; i < ARRAY_SIZE(pos90States); i++)
        {
            movingStates[i] = pos90States[i];
        }
        step = 1;
        modeType = Pos90Mode;
        for (int i = 0; i < ARRAY_SIZE(legs); i++)
        {
            legs[i].ChangePosition(90, false);
            legs[i].ChangePositionSlave(90);
        }
    }
    void Move()
    {
        switch (movingStates[step])
        {
        case Stop:
        {
            // if ChangeToResetTemp(), the robot is already reset, so we can move on
            if (modeType != ResetMode && reset == true)
            {
                ModeChanged(modeType);
            }

            break;
        }
        case ResetMaster:
        {
            for (int i = 0; i < ARRAY_SIZE(legs); i++)
                legs[i].ChooseMove(Back, false);
            step++;
            break;
        }
        case ResetSlave:
        {
            for (int i = 0; i < ARRAY_SIZE(legs); i++)
                legs[i].ChooseMove(Down, false);
            step++;
            reset = true;
            break;
        }
        case Forward:
        {
            // 1st leg is moving forward
            bool back = (modeType == BackMode); // Forward => false
            for (int i = 0; i < ARRAY_SIZE(legTeam1); i++)
            {
                legs[legTeam1[i]].ChooseMove(Forward, back); // Forward => false
                legs[legTeam2[i]].ChooseMove(Back, !back);   // Forward => true
            }
            step++;
            break;
        }
        case MasterLoop:
        {
            for (int i = 0; i < ARRAY_SIZE(legs); i++)
                legs[i].GoToPositionMaster();
            if (MovesDone())
                step++;
            break;
        }
        case Down1:
        {
            for (int i = 0; i < ARRAY_SIZE(legTeam1); i++)
            {
                legs[legTeam1[i]].ChooseMove(Down, false);
            }
            step++;
            break;
        }
        case SlaveLoop:
        {
            for (int i = 0; i < ARRAY_SIZE(legs); i++)
                legs[i].GoToPosition();
            if (MovesDone())
                step++;
            break;
        }
        case Down2:
        {
            for (int i = 0; i < ARRAY_SIZE(legTeam1); i++)
            {
                legs[legTeam2[i]].ChooseMove(Up, false);
            }
            step++;
            break;
        }
        case Back:
        {
            // 1st leg is moving back
            bool forward = (modeType == ForwardMode); // Forward => true
            for (int i = 0; i < ARRAY_SIZE(legTeam1); i++)
            {
                legs[legTeam1[i]].ChooseMove(Back, forward);     // Forward => true
                legs[legTeam2[i]].ChooseMove(Forward, !forward); // Forward => false
            }
            step++;
            break;
        }
        case Up1:
        {
            for (int i = 0; i < ARRAY_SIZE(legTeam1); i++)
            {
                legs[legTeam2[i]].ChooseMove(Down, false);
            }
            step++;
            break;
        }
        case Up2:
        {
            for (int i = 0; i < ARRAY_SIZE(legTeam1); i++)
            {
                legs[legTeam1[i]].ChooseMove(Up, false);
            }
            step++;
            break;
        }

        // Left/Right:
        case Left:
        {

            bool right = (modeType == RightMode);
            legs[0].ChooseMove(Forward, right);
            legs[1].ChooseMove(Forward, !right);
            legs[2].ChooseMove(Back, !right);
            legs[3].ChooseMove(Back, right);
            legs[4].ChooseMove(Forward, right);
            legs[5].ChooseMove(Forward, !right);
            step++;
            break;
        }
        case Right:
        {
            bool right = (modeType == RightMode);
            legs[0].ChooseMove(Back, !right);
            legs[1].ChooseMove(Back, right);
            legs[2].ChooseMove(Forward, right);
            legs[3].ChooseMove(Forward, !right);
            legs[4].ChooseMove(Back, !right);
            legs[5].ChooseMove(Back, right);
            step++;
            break;
        }
        }
        if (step >= ARRAY_SIZE(movingStates))
            step = 1;
    }
    bool MovesDone()
    {
        for (int i = 0; i < ARRAY_SIZE(legs); i++)
        {
            if (!legs[i].MoveDone())
                return false;
        }
        return true;
    }
    void ModeChanged(Mode s)
    {
        switch (s)
        {
        case StopMode:
        {
            ChangeToStop();
            break;
        }
        case ResetMode:
        {
            ChangeToReset();
            break;
        }
        case ForwardMode:
        {
            ChangeToForward();
            break;
        }
        case BackMode:
        {
            ChangeToBack();
            break;
        }
        case LeftMode:
        {
            ChangeToLeft();
            break;
        }
        case RightMode:
        {
            ChangeToRight();
            break;
        }
        case Pos90Mode:
        {
            ChangeTo90();
            break;
        }
        }
    }
    void DisableLegs()
    {
        for (int i = 0; i < ARRAY_SIZE(legs); i++)
        {
            legs[i].DisableLeg();
        }
    }
};