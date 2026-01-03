#ifndef SERIAL_MANAGER_H
#define SERIAL_MANAGER_H

#include "defs.h"

struct txBuffer
{
    double data[TX_BUFFER_SIZE];
    char msg[TX_BUFFER_SIZE * sizeof(double)];
    uint8_t var_index;
    uint8_t chunk_index;
};

struct rxBuffer
{
    char data[RX_BUFFER_SIZE];
    uint8_t index;
    bool complete;
};

class serialManager
{
public:
    explicit serialManager();
    void start_serial();
    void set_callback(void (*callback)(char id1, char id2, int64_t val_i, double val_f));

    void read_serial();
    void parse_serial();
    void clear();

    void enable_stream(bool enable);
    void fill_chunk(const double variables[NUM_OF_VARIABLES]);
    void send_msg(const char *msg);
    void send_msg(const char *msg, double val_f);
    void send_msg(const char *msg, int val_d);
    void send_msg(const char *msg, int32_t val_d);
    void send_msg(const char *msg, int64_t val_d);
    void send_msg(const char *msg, char val_s);

private:
    txBuffer tx_buffer{.var_index = 0, .chunk_index = 0};
    rxBuffer rx_buffer{.data = {}, .index = 0, .complete = false};
    bool stream_data = false;

    unsigned long baudrate;
    void (*on_complete)(char id1, char id2, int64_t val_i, double val_f);
    void send_data();
    void send_msg();
};

#endif // SERIAL_MANAGER_H