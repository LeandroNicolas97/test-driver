#include "smart_sensor.h"
#define RECEIVING      1
#define SLEEPING       0
#define RECEPTION_TIME 1000

void send_data_from_storage(int time_of_last_measurement);
void send_data_from_datalogger(int time_of_last_measurement);
int receiving_commands(char *data);
int is_channel_free(void);
int data_reception(char *data);
int if_received_data(char *data);
int check_acknowledgment(char *data, char *name, int time_of_last_measurement);
void send_ping(void);
void send_adcp_measurements(int time_of_last_measurement, enum sensor_manufacturer sensor_manufacturer);
bool check_for_adcp(void);
