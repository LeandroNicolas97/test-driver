
#ifndef LOCAL_SENSORS_H
#define LOCAL_SENSORS_H

#include <stdint.h>

/**
 * @brief Initialize the humidity sensor device
 *
 * @return 0 if successful, negative errno code if failure.
 */
int local_sensors_init(void);

/**
 * @brief Get humidity and temperature readings
 *
 * @param h A ponter to a int16_t, where humidity is stored.
 * @param t A pointer to a float, where temperature is stored.
 *
 * @return 0 if successful, negative errno code if failure.
 */
int local_sensors_get_hum_and_temp(int16_t *h, float *t);

#endif /* end of include guard: LOCAL_SENSORS_H_*/
