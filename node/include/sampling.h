
#ifndef _SAMPLING_H
#define _SAMPLING_H

#include "sampling.h"
int sampling(int communication_tries, int n_of_sensors, struct measurement *measurements);
int should_start_sampling(uint32_t sampling_interval);
void acquire_local_sensors(struct measurement *node_measurement, struct measurement *valve_measurements);

#endif /* end of include guard: _SAMPLING_H */
