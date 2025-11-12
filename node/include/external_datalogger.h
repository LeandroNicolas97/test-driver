
#ifndef EXTERNAL_DATALOGGER_H
#define EXTERNAL_DATALOGGER_H

int datalogger_mount(void);
uint16_t datalogger_unsended_data_get(void);
void datalogger_unsended_data_flush_last(void);
int datalogger_append(uint8_t *meas_data, size_t size);
int datalogger_get(uint8_t *meas_data, size_t size, uint16_t n_from_last);
void datalogger_format(void);
uint32_t datalogger_get_free_space(void);

#endif /* EXTERNAL_DATALOGGER_H */
