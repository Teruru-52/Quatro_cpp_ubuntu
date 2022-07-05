#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include "main.h"
#include <vector>
#include "hardware/motor.h"
#include "controller/pid_controller.h"
#include "controller/kanayama.h"

namespace undercarriage
{
    class Controller
    {
    public:
        Controller(float control_period);

        void UpdateBatteryVoltage(float bat_vol);
        void PartyTrick(const std::vector<float> &cur_pos, const std::vector<float> &cur_vel);
        void PivotTurn180(std::vector<float> cur_vel);
        void KanayamaUpdateRef();
        void KanayamaTurnLeft90(std::vector<float> cur_pos, std::vector<float> cur_vel);
        void GoStraight(std::vector<float> cur_pos, std::vector<float> cur_vel, std::vector<uint32_t> ir_data);
        void InputVelocity(float input_v, float input_w);
        bool GetFlag();
        float GetInput();
        void MotorTest(float v_left, float v_right);
        void OutputLog();

    private:
        hardware::Motor motor;
        PID pid_angle;
        PID pid_rotational_vel;
        PID pid_traslational_vel;
        PID pid_ir_sensor_left;
        PID pid_ir_sensor_right;
        undercarriage::Kanayama kanayama;
        trajectory::PivotTurn pivot_turn;

        float tmp;
        float v_left;
        float v_right;
        float u_w;
        float u_v;
        // std::vector<float> ref_pos;
        std::vector<float> ref_vel;
        float ref_w;
        const float Tp1 = 20.56;
        const float Kp = 137.9;
        const float v_straight = 0.936196798079597;
        const float ir_straight = 1000;
        bool flag;
    };
} // namespace undercarriage

#endif //  CONTROLLER_HPP_