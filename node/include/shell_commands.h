
#ifndef _SHELL_COMMANDS_H
#define _SHELL_COMMANDS_H

#include "multishell.h"

int show_status(char *str);
int wake_up(char *str);
int to_sleep(char *str);
int set_name(char *str);
int set_sampling_interval(char *str);
int set_display_contrast(char *str);
int save_configuration(char *str);
int cmd_set_saturation_conf(char *str);
int cmd_reboot(char *s);
int cmd_injection(char *args);
int cmd_set_channel(char *args);
int cmd_set_distance(char *str);
int cmd_get_mac_address(char *str);
int cmd_tunnel(char *str);
int cmd_temp_offset(char *str);
int cmd_date(char *str);
int cmd_current_sensor_status(char *str);
int cmd_erase_data_flash(char *str);
int cmd_debug_level(char *str);
int cmd_set_totalized(char *str);
#if CONFIG_EXTERNAL_DATALOGGER
int cmd_external_flash_datalogger_dump(char *str);
int cmd_externalflash_datalogger_format(char *str);
int cmd_external_flash_datalogger_put(char *str);
#endif
int cmd_jiangsu_config(char *str);
int cmd_set_sensor_driver(char *str);
int cmd_set_sensor_config(char *str);
int cmd_detect_sensors(char *str);
int cmd_volume_porcentage(char *str);

#define SIZE_COMMAND 40

extern struct shell_command command_list[];

struct received_command {
    char command[SIZE_COMMAND];
};

int detect_sensors(void);

void processes_init_complete(void);

#endif /* end of include guard: SHELL_COMMANDS_H_YGP28D14 */
