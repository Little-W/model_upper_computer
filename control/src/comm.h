#ifndef _COMM_H_
#define _COMM_H_

#include "serial/serial.h"
#include <chrono>
#include <ctime>


// CRC-8 polynomial: x^8 + x^2 + x^1 + 1 (0x07)
#define POLYNOMIAL 0x07
#define INITIAL_REMAINDER 0x00
#define ANGLE_TRANS_COUNT 20
#define SPEED_TRANS_COUNT 20
#define ENC_SPEED_COUNT 10
#define BATCH_TRANS_BEGIN 0xf8
// #define SPEED_TRANS_BEGIN 0xf3
#define SINGEL_TRANS_BEGIN 0xc0
#define TRANS_OVER 0xc7
#define TRANS_SPLIT 0xff

#define MAX_ANGLE_VAL 1023
#define MAX_SPEED_VAL 682

#define ENC_SPEED_SCALE 14.0
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