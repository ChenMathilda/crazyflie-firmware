#ifndef RELATIVECONT_H_
#define RELATIVECONT_H_
#include <math.h>
//for AI //////////////////////////////

const float alpha = 0.7;
const float beta = 0.5;
const float PI = 180.0;
const float velMax = 0.1f;

// for AI //////////////////////////////
void relativeControlInit(void);
void relativeControlTask(void* arg);

#endif