#ifndef __LINUX_GSLX680_TS_H__
#define __LINUX_GSLX680_TS_H__

#define GSLX680_NAME "gslX680_ts"
#define GSLX680_I2C_ADDR 0x40

struct gslx680_platform_data {
	unsigned int x_max;
	unsigned int y_max;
	unsigned long irqflags;
	unsigned int irq;
	unsigned int reset;
    char* vdd;
};

#endif
