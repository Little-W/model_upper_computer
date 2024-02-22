#ifndef _COMM_H_
#define _COMM_H_

#include "serial/serial.h"
#include <chrono>
#include <ctime>

extern serial::Serial ser;
extern float speed_result;
extern float angle_result;
extern std::chrono::time_point<std::chrono::high_resolution_clock> start_time_stamp;

typedef struct comm_data
{
    int val;
    unsigned char cnt;
} comm_data_t;

void encode_and_send(void);
int get_speed_enc(void);

#endif