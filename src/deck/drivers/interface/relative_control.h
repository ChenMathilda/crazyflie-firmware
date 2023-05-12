#ifndef RELATIVECONT_H_
#define RELATIVECONT_H_
#include <math.h>
//for AI //////////////////////////////
#define histSize 5
const float alpha = 0.7;
const float beta = 0.5;
const float PI = 180.0;
const float velMax = 0.1f;
typedef struct
{
    float steer_ctl[histSize];
    float coll_ctl[histSize];
    float sign_ctl[histSize];
    uint8_t index;
} latest3_data_t;

// for AI //////////////////////////////
void relativeControlInit(void);
void relativeControlTask(void* arg);

#endif