#ifndef __LINUX_FT6X06_TS_H__
#define __LINUX_FT6X06_TS_H__

#define FT6X06_NAME 	"ft6x06_ts"
#define FT6X06_I2C_ADDR 0x38

struct ft6x06_platform_data {
	unsigned int x_max;
	unsigned int y_max;
	unsigned long irqflags;
	unsigned int irq;
	unsigned int reset;
    const char *vdd;
};

#endif
