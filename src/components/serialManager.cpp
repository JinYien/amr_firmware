#include "serialManager.h"

serialManager::serialManager() : baudrate(115200), on_complete(nullptr) {}

void serialManager::start_serial()
{
    Serial.begin(this->baudrate);
    while (!Serial)
    {
    }
}

void serialManager::read_serial()
{
    while (Serial.available())
    {
        this->rx_buffer.data[this->rx_buffer.index++] = Serial.read();

        if (this->rx_buffer.index >= RX_BUFFER_SIZE)
        {
            this->rx_buffer.index = 0;
            if (!(this->rx_buffer.data[0] == '@' || this->rx_buffer.data[0] == '<' || this->rx_buffer.data[0] == '>' ||
                  this->rx_buffer.data[0] == '*' || this->rx_buffer.data[0] == '%') ||
                this->rx_buffer.data[RX_BUFFER_SIZE - 1] != '\n')
            {
                clear();
            }
            else
            {
                this->rx_buffer.complete = true;
            }
        }
    }
    if (this->rx_buffer.complete)
    {
        parse_serial();
        clear();
    }
}

void serialManager::clear()
{
    Serial.clear();
    memset(this->rx_buffer.data, 0, sizeof(this->rx_buffer.data));
    this->rx_buffer.complete = false;
    this->rx_buffer.index = 0;
}

void serialManager::set_callback(void (*callback)(char id1, char id2, int64_t val_t, double val_f))
{
    this->on_complete = callback;
}

void serialManager::enable_stream(const bool enable) { this->stream_data = enable; }

void serialManager::parse_serial()
{
    const char id1 = this->rx_buffer.data[0];
    const char id2 = this->rx_buffer.data[1];

    int64_t val_i = 0;
    double val_f = 0.0;
    memcpy(&val_i, &this->rx_buffer.data[2], RX_BUFFER_SIZE - 3);
    memcpy(&val_f, &this->rx_buffer.data[2], RX_BUFFER_SIZE - 3);

    on_complete(id1, id2, val_i, val_f);
}

void serialManager::fill_chunk(const double variables[NUM_OF_VARIABLES])
{
    for (int i = 0; i < NUM_OF_VARIABLES; i++)
    {
        this->tx_buffer.data[this->tx_buffer.var_index++] = variables[i];
    }
    this->tx_buffer.chunk_index++;
    if (this->tx_buffer.chunk_index >= CHUNK_SIZE)
    {
        if (this->stream_data)
        {
            send_data();
        }
        this->tx_buffer.chunk_index = 0;
        this->tx_buffer.var_index = 0;
    }
}

void serialManager::send_data()
{
    if (this->stream_data)
    {
        Serial.write("@s");
        Serial.write(reinterpret_cast<uint8_t *>(&(this->tx_buffer.data)), TX_BUFFER_SIZE * sizeof(double));
        Serial.write('\n');
        Serial.send_now();

        memset(this->tx_buffer.data, 0, sizeof(this->tx_buffer.data));
    }
}

void serialManager::send_msg()
{
    Serial.write("@m");
    Serial.write(reinterpret_cast<uint8_t *>(&(this->tx_buffer.msg)), TX_BUFFER_SIZE * sizeof(double));
    Serial.write('\n');
    Serial.send_now();

    memset(this->tx_buffer.msg, 0, sizeof(this->tx_buffer.msg));
}

void serialManager::send_msg(const char *msg)
{
    strcpy(this->tx_buffer.msg, msg);
    send_msg();
}

void serialManager::send_msg(const char *msg, const double val_f)
{
    sprintf(this->tx_buffer.msg, msg, val_f);
    send_msg();
}

void serialManager::send_msg(const char *msg, const int val_d)
{
    sprintf(this->tx_buffer.msg, msg, val_d);
    send_msg();
}

void serialManager::send_msg(const char *msg, const int32_t val_d)
{
    sprintf(this->tx_buffer.msg, msg, val_d);
    send_msg();
}
void serialManager::send_msg(const char *msg, const int64_t val_d)
{
    sprintf(this->tx_buffer.msg, msg, val_d);
    send_msg();
}

void serialManager::send_msg(const char *msg, const char val_s)
{
    sprintf(this->tx_buffer.msg, msg, val_s);
    send_msg();
}