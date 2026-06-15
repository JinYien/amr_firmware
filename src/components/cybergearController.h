#ifndef CYBERGEAR_CONTROLLER_H
#define CYBERGEAR_CONTROLLER_H

#include <FlexCAN_T4.h>
#include <cybergearCommands.h>
#include <cybergearVariables.h>
#include "components/serialManager.h"

class cybergearController
{
public:
    cybergearController(uint8_t num_of_motors, uint8_t master_id, uint8_t *motor_ids, FlexCAN_T4_Base *my_can);
    uint8_t num_of_motors;
    uint8_t master_id;
    cybergearVariables *motors;
    uint8_t *motor_ids;

    void on_receive(const CAN_message_t &msg);

    void setup_motors();
    void update_motor_statuses();

    void reset_motor(uint8_t motor_no, bool wait_reply = false);
    void enable_motor(uint8_t motor_no, bool wait_reply = false);
    void set_mech_position_to_zero(uint8_t motor_no, bool wait_reply = false);
    void set_run_mode(uint8_t motor_no, CONTROL_MODES run_mode, bool wait_reply = false);
    void change_motor_can_id(uint8_t motor_no, uint8_t new_can_id, bool wait_reply = false);

    void read_control_params(uint8_t motor_no, CONTROL_PARAMS param, bool wait_reply = true);
    void write_control_params(uint8_t motor_no, CONTROL_PARAMS param, float value, bool wait_reply = false);
    void update_motor_params(uint8_t motor_no, char id, int val_t, float val_f, serialManager *ser = nullptr);

    void reset_all_motors(bool wait_reply = false);
    void set_all_mech_position_to_zero(bool wait_reply = false);
    void set_all_run_mode(CONTROL_MODES run_mode, bool wait_reply = false);
    void update_all_motor_params(char id, int val_t, float val_f, serialManager *ser = nullptr);

private:
    FlexCAN_T4_Base *my_can;
    bool wait_for_reply(unsigned int num_of_replies = 1, uint32_t timeout_us = 3000);
    void send_command(const cybergearCommand *cmd) const;

    cybergearCommand cybergear_cmd{};
    volatile unsigned int replies_received = 0;
};

#endif // CYBERGEAR_CONTROLLER_H
