

#ifndef SENSOR_ST_HTS221_H__
#define SENSOR_ST_HTS221_H__

#include "sensor.h"
#include "hts221.h"

#define HTS221_ADDR_DEFAULT (0xBE >> 1)

int rt_hw_hts221_init(const char *name, struct rt_sensor_config *cfg);

#endif
