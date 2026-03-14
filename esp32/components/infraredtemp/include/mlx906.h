#ifndef MLX906_H
#define MLX906_H

#include <stdint.h>
#include "esp_err.h"

// Initialize I2C for the sensor
esp_err_t mlx906_init(void);

// Read object temperature in Celsius
esp_err_t mlx906_read_object_temp(float *temp);

// Read ambient temperature in Celsius
esp_err_t mlx906_read_ambient_temp(float *temp);

#endif