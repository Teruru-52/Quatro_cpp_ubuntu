#include "controller/controller.h"

namespace undercarriage
{
    Controller::Controller(float control_period)
        : pid_angle(2.0, 2.0, 0.05, 0.0, control_period),
          pid_rotational_vel(0.80877, 37.1615, -0.0010359, 0.0060725, control_period),
          pid_traslational_vel(19.72, 52.0, 1.548, 0.0, control_period),
          pid_ir_sensor_left(1.0, 0.0, 0.0, 0.0, control_period),
          pid_ir_sensor_right(1.0, 0.0, 0.0, 0.0, control_period),
          kanayama(1.0, 1.0, 1.0),
          flag(true) {}

    void Controller::UpdateBatteryVoltage(float bat_vol)
    {
        motor.UpdateBatteryVoltage(bat_vol);
    }

    void Controller::PartyTrick(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel)
    {
        // u_v = pid_traslational_vel.Update(-cur_vel[0]);
        u_v = 0.0;
        u_w = pid_angle.Update(-cur_pos[2]) + pid_rotational_vel.Update(-cur_vel[1]);
        tmp = cur_pos[2];
        InputVelocity(u_v, u_w);
    }

    // void Controller::PivotTurn180(std::vector<float> cur_vel)
    // {
    //     pivot_turn.UpdateRef();
    //     ref_w = pivot_turn.GetRefVelocity();
    //     u_w = pid_rotational_vel.Update(ref_w - cur_vel[1]) + Tp1 * ref_w / Kp;
    //     InputVelocity(0.0, u_w);
    // }

    // void Controller::KanayamaUpdateRef()
    // {
    //     kanayama.UpdateRef();
    // }

    // void Controller::KanayamaTurnLeft90(std::vector<float> cur_pos, std::vector<float> cur_vel)
    // {
    //     ref_vel = kanayama.CalcInput(cur_pos);
    //     u_v = pid_traslational_vel.Update(ref_vel[0] - cur_vel[0]);
    //     u_w = pid_rotational_vel.Update(ref_vel[1] - cur_vel[1]) + Tp1 * ref_w / Kp;
    //     InputVelocity(u_v, u_w);
    // }

    // void Controller::GoStraight(std::vector<float> cur_pos, std::vector<float> cur_vel, std::vector<uint32_t> ir_data)
    // {
    //     u_v = pid_traslational_vel.Update(v_straight - cur_vel[0]);
    //     u_w = pid_ir_sensor_left.Update((float)(ir_straight - ir_data[2])) + pid_ir_sensor_right.Update((float)(ir_straight - ir_data[3])) + pid_angle.Update(-cur_pos[0]);
    //     InputVelocity(u_v, u_w);
    // }

    void Controller::InputVelocity(float input_v, float input_w)
    {
        v_left = input_v - input_w;
        v_right = input_v + input_w;
        motor.Drive(v_left, v_right);
    }

    float Controller::GetInput()
    {
        // return u_v;
        return u_w;
    }

    void Controller::MotorTest(float v_left, float v_right)
    {
        motor.Drive(v_left, v_right); // voltage [V]
    }

    void Controller::OutputLog()
    {
        printf("%f, %f\n", u_v, u_w);
        // pid_angle.OutputLog();
        // printf("%f\n", tmp);
    }
}