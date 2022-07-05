#include "trajectory.h"

namespace trajectory
{
    // TurnLeft
    TurnLeft90::TurnLeft90()
        : index(0) {}

    int TurnLeft90::GetTrajectoryIndex()
    {
        return index;
    }

    void TurnLeft90::ResetTrajectoryIndex()
    {
        index = 0;
    }

    void TurnLeft90::UpdateRef()
    {
        // ref_pos = ref_pos_csv[index];
        // ref_vel = ref_vel_csv[index];
        index++;
    }

    std::vector<float> TurnLeft90::GetRefPosition()
    {
        return ref_pos;
    }

    std::vector<float> TurnLeft90::GetRefVelocity()
    {
        return ref_vel;
    }

    // PivotTurn
    PivotTurn::PivotTurn()
        : index(0) {}

    int PivotTurn::GetTrajectoryIndex()
    {
        return index;
    }

    void PivotTurn::ResetTrajectoryIndex()
    {
        index = 0;
    }

    void PivotTurn::UpdateRef()
    {
        ref = ref_u_w[index];
        index++;
    }

    float PivotTurn::GetRefVelocity()
    {
        return ref;
    }

    // M Sequence
    M_sequence::M_sequence()
        : index(0),
          flag(true)
    {
        ref_size = GetRefSize();
    }

    int M_sequence::GetTrajectoryIndex()
    {
        return index;
    }

    void M_sequence::ResetTrajectoryIndex()
    {
        index = 0;
    }

    int M_sequence::GetRefSize()
    {
        return ref_u_w.size();
    }

    void M_sequence::UpdateRef()
    {
        if (index < ref_size)
        {
            ref = ref_u_w[index];
            index++;
        }
        else if (index == ref_size)
        {
            flag = false;
        }
    }

    float M_sequence::GetRef()
    {
        return ref;
    }

    bool M_sequence::GetFlag()
    {
        return flag;
    }
}