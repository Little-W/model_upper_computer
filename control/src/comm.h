#ifndef _COMM_H_
#define _COMM_H_

#include "serial/serial.h"
#include <chrono>
#include <ctime>


// CRC-8 polynomial: x^8 + x^2 + x^1 + 1 (0x07)
#define POLYNOMIAL 0x07
#define INITIAL_REMAINDER 0x00

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