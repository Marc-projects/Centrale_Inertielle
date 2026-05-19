#ifndef MAIN_H
#define MAIN_H

#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "freertos/queue.h"

typedef struct handle_structure {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t device;
    QueueHandle_t           queue;
} handle_structure;

#endif