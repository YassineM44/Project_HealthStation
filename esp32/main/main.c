#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mlx906.h"

void app_main(void)
{
    if (mlx906_init() != ESP_OK)
    {
        printf("Failed to init MLX906 sensor\n");
        return;
    }

    while (1)
    {
        float obj_temp, amb_temp;

        if (mlx906_read_object_temp(&obj_temp) == ESP_OK &&
            mlx906_read_ambient_temp(&amb_temp) == ESP_OK)
        {
            printf("Object: %.2f C | Ambient: %.2f C\n",
                   obj_temp, amb_temp);
        }
        else
        {
            printf("Failed to read sensor\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}