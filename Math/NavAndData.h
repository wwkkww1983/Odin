
#ifndef __NAVANDDATA_H
#define __NAVANDDATA_H

#ifndef M_PI
#define M_PI			3.14159265f
#define M_PI_2			(M_PI / 2.0f)
#endif

#define RAD_TO_DEG		(180.0f / M_PI)					//弧度到角度
#define DEG_TO_RAD		(M_PI / 180.0f)					//角度到弧度

#define GRAVITY			9.80565f	// m/s^2

#define AQ_US_PER_SEC		1000000

#ifndef NAN
#define NAN	__float32_nan
#endif

#include "arm_math.h"																//移植到瑞萨时需要进行库转换
#include "aq_math.h"
#include "Matrix.h"
#include "compass.h"
#include "srcdkf.h"
#include "Util.h"

#endif









