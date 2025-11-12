#ifndef WTVB01_H
#define WTVB01_H

/* Manufacturer ID */
#define WTVB01_MANUFACTURER 0x01

/* Startup time */
#define WTVB01_STARTUP_TIME_MS 500

/* Register addresses */
#define WTVB01_REG_SAVE         0x00  /* Save / Restart / Factory */
#define WTVB01_REG_BAUD         0x04  /* Serial baud rate */
#define WTVB01_REG_ADDR         0x1A  /* Device Modbus address */
#define WTVB01_REG_AX           0x34  /* Acceleration X */
#define WTVB01_REG_AY           0x35  /* Acceleration Y */
#define WTVB01_REG_AZ           0x36  /* Acceleration Z */
#define WTVB01_REG_GX           0x37  /* Angular velocity X */
#define WTVB01_REG_GY           0x38  /* Angular velocity Y */
#define WTVB01_REG_GZ           0x39  /* Angular velocity Z */
#define WTVB01_REG_VX           0x3A  /* Velocity X */
#define WTVB01_REG_VY           0x3B  /* Velocity Y */
#define WTVB01_REG_VZ           0x3C  /* Velocity Z */
#define WTVB01_REG_ADX          0x3D  /* Angle X */
#define WTVB01_REG_ADY          0x3E  /* Angle Y */
#define WTVB01_REG_ADZ          0x3F  /* Angle Z */
#define WTVB01_REG_TEMP         0x40  /* Temperature */
#define WTVB01_REG_DX           0x41  /* Displacement X */
#define WTVB01_REG_DY           0x42  /* Displacement Y */
#define WTVB01_REG_DZ           0x43  /* Displacement Z */
#define WTVB01_REG_HZX          0x44  /* Frequency X */
#define WTVB01_REG_HZY          0x45  /* Frequency Y */
#define WTVB01_REG_HZZ          0x46  /* Frequency Z */

/* Filter and sampling parameters */
#define WTVB01_REG_CUTOFF_INT   0x63  /* Cutoff frequency integer part */
#define WTVB01_REG_CUTOFF_FRAC  0x64  /* Cutoff frequency fractional part */
#define WTVB01_REG_SAMPLE_FREQ  0x65  /* Sampling frequency */

/* Commands */
#define WTVB01_CMD_SAVE         0x0000  /* Save configuration */
#define WTVB01_CMD_RESTART      0x00FF  /* Restart device */
#define WTVB01_CMD_FACTORY      0x0001  /* Restore factory defaults */

/* Limits */
#define WTVB01_MIN_CUTOFF_FREQ  0.0f
#define WTVB01_MAX_CUTOFF_FREQ  200.0f
#define WTVB01_MIN_SAMPLE_FREQ  1
#define WTVB01_MAX_SAMPLE_FREQ  200

#endif /* WTVB01_H */
