
#ifndef MEASUREMENT_STORAGE_H
#define MEASUREMENT_STORAGE_H

int measurement_storage_mount(void);
uint16_t unsended_data_get(void);
void unsended_data_flush_last(void);
int measurement_storage_append(uint8_t *meas_data, size_t size);
int measurement_storage_get(uint8_t *meas_data, size_t size, uint16_t n_from_last);
void measurement_storage_format(void);
uint32_t get_free_space(void);

#endif /* MEASUREMENT_STORAGE_H */
