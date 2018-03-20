/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/regulator/consumer.h>

#include <linux/input/gslX680_ts.h>
#include "GSL1680_FW_360X480.h"

// #undef pr_debug
// #define pr_debug pr_info

#define GSL_REG_FINGER		0x80
#define GSL_REG_TOUCH		0x84 //0x84~0x87, 0x88~0x8B, ...
#define GSL_REG_CHECK 		0xB0
#define GSL_REG_POWER		0xBC //0xBC~0xBF
#define GSL_REG_RESET		0xE0
#define GSL_REG_CLOCK		0xE4
#define GSL_REG_PAGE		0xF0

#define GSL_CKECK32			0x5A5A5A5A

#define JOIN_2BYTES(d0, d1) ((d0) | (d1) << 8)
#define GSL_XDATA(arr) (JOIN_2BYTES(arr[0], arr[1]) & 0xFFF)
#define GSL_YDATA(arr) (JOIN_2BYTES(arr[2], arr[3]) & 0xFFF)
#define GSL_IDATA(arr) ((arr[3] >> 4) & 0xF)

#define GSL_PRESS				0x7F

#define GSL_PRESS_MAX    		255
#define GSL_MAX_FINGERS 		10
#define GSL_MAX_CONTACTS 		10
#define GSL_DMA_TRANS_LEN		0x20

#define GSL_FINGERS(f) ((f) < GSL_MAX_FINGERS ? (f) : GSL_MAX_FINGERS)

struct gsl_ts_event {
	u16 x[GSL_MAX_FINGERS];
	u16 y[GSL_MAX_FINGERS];
	u8 id[GSL_MAX_FINGERS];
};

struct gsl_ts {
	struct i2c_client *client;
	struct input_dev *input_dev;

	int irq;
	struct gslx680_platform_data *pdata;

	struct delayed_work reset_work;
	struct delayed_work data_work;
	struct workqueue_struct *workqueue;

	struct gsl_ts_event event;

	u8 finger;
	u8 touch[4*GSL_MAX_FINGERS];
};

#ifdef CONFIG_TOUCHSCREEN_GSLX680_ANTI_SHAKE
struct gsl_touch_info {
	int x[GSL_MAX_FINGERS];
	int y[GSL_MAX_FINGERS];
	int id[GSL_MAX_FINGERS];
	int finger_num;
};

extern void gsl_DataInit(unsigned int * conf_in);
extern void gsl_alg_id_main(struct gsl_touch_info *cinfo);
extern unsigned int gsl_mask_tiaoping(void);
#endif

static int gsl_read_i2c(struct i2c_client *client, u8 reg, u8 *pdata, u8 datalen)
{
	return  i2c_smbus_read_i2c_block_data(client, reg, datalen, pdata);
}

static int gsl_write_i2c(struct i2c_client *client, u8 reg, u8 *pdata, u8 datalen)
{
	return  i2c_smbus_write_i2c_block_data(client, reg, datalen, pdata);
}

static int gsl_write_interface(struct i2c_client *client, u8 reg, u8 *buf, u8 datalen)
{
	struct i2c_msg msg[1];

	buf[0] = reg;

	msg[0].addr = client->addr;
	msg[0].len = datalen + 1;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].buf = buf;
	return i2c_transfer(client->adapter, msg, 1);
}

static int gsl_init_gpio(struct gslx680_platform_data *pdata)
{
	int err;

	if((err = gpio_request(pdata->reset, "gslx680 reset")) < 0) {
		pr_err("GSLX680: failed to request reset gpio %d\n", pdata->reset);
		return err;
	}
	gpio_direction_output(pdata->reset, 1);

	if((err = gpio_request(pdata->irq, "gslx680 int")) < 0) {
		pr_err("GSLX680: failed to request int gpio %d\n", pdata->irq);
		goto free_reset;
	}
	gpio_direction_input(pdata->irq);

	return 0;

free_reset:
	gpio_free(pdata->reset);
	return err;
}

static int set_power(char* _vdd, int enable)
{
	struct regulator *vdd = regulator_get(NULL, _vdd);
	int err;

	if(IS_ERR(vdd)) {
		pr_err("GSLX680: failed to get the power\n");
		return -EINVAL;
	}

	err = enable ? regulator_enable(vdd) : regulator_disable(vdd);
	regulator_put(vdd);
	return err;
}

static int gsl_power_on(struct gslx680_platform_data *pdata)
{
	return set_power(pdata->vdd, 1);
}

static int gsl_power_off(struct gslx680_platform_data *pdata)
{
	return set_power(pdata->vdd, 0);
}

static void gsl_reset(struct gslx680_platform_data *pdata)
{
	gpio_set_value(pdata->reset, 0);
	msleep(50);
	gpio_set_value(pdata->reset, 1);
	msleep(30);
}

static void gsl_shutdown(struct gslx680_platform_data *pdata)
{
	gpio_set_value(pdata->reset, 0);
}

static __inline__ void fw2buf(u8 *buf, const u32 *fw)
{
	u32 *u32_buf = (u32 *)buf;
	*u32_buf = *fw;
}

static void gsl_load_fwx680(struct i2c_client *client)
{
	u8 buf[GSL_DMA_TRANS_LEN*4 + 1] = {0};
	u8 send_flag = 1;
	u8 *cur = buf + 1;
	u32 source_line = 0;
	u32 source_len = ARRAY_SIZE(GSLX680_FW);
	const struct fw_data *fw = GSLX680_FW;
	u32 now = jiffies_to_msecs(jiffies);

	pr_debug("GSLX680: load fw...\n");

	for (source_line = 0; source_line < source_len; source_line++) {
		/* init page trans, set the page val */
		if(GSL_REG_PAGE == fw[source_line].offset) {
			fw2buf(cur, &fw[source_line].val);
			gsl_write_interface(client, GSL_REG_PAGE, buf, 4);
			send_flag = 1;
		}
		else {
			if(1 == send_flag % (GSL_DMA_TRANS_LEN < 0x20 ? GSL_DMA_TRANS_LEN : 0x20))
				buf[0] = (u8)fw[source_line].offset;

				fw2buf(cur, &fw[source_line].val);
				cur += 4;

				if(0 == send_flag % (GSL_DMA_TRANS_LEN < 0x20 ? GSL_DMA_TRANS_LEN : 0x20)) {
					gsl_write_interface(client, buf[0], buf, cur - buf - 1);
					cur = buf + 1;
			}
			send_flag++;
		}
	}

	pr_debug("GSLX680: load fw expenses %d mesc\n", jiffies_to_msecs(jiffies)-now);
}

static void gsl_setup_chip(struct i2c_client *client)
{
	u8 tmp = 0x00;
	gsl_write_i2c(client, GSL_REG_RESET, &tmp, sizeof(tmp));
#ifdef CONFIG_TOUCHSCREEN_GSLX680_ANTI_SHAKE
	gsl_DataInit(gsl_config_data_id);
#endif
	msleep(10);
}

static void gsl_reset_chip(struct i2c_client *client)
{
	u32 pwr = 0x0;
	u8 tmp = 0x88;
	gsl_write_i2c(client, GSL_REG_RESET, &tmp, sizeof(tmp));
	msleep(10);

	tmp = 0x04;
	gsl_write_i2c(client, GSL_REG_CLOCK, &tmp, sizeof(tmp));
	msleep(10);

	gsl_write_i2c(client, GSL_REG_POWER, (u8 *)&pwr, sizeof(pwr));
	msleep(10);
}

static void gsl_clear_status(struct i2c_client *client)
{
	u8 tmp = 0x88;
	gsl_write_i2c(client, GSL_REG_RESET, &tmp, sizeof(tmp));
	msleep(20);

	tmp = 0x03;
	gsl_write_i2c(client, GSL_REG_FINGER, &tmp, sizeof(tmp));
	msleep(5);

	tmp = 0x04;
	gsl_write_i2c(client, GSL_REG_CLOCK, &tmp, sizeof(tmp));
	msleep(5);

	tmp = 0x00;
	gsl_write_i2c(client, GSL_REG_RESET, &tmp, sizeof(tmp));
	msleep(20);
}

static void gsl_init_chip(struct i2c_client *client)
{
	gsl_clear_status(client);
	gsl_reset_chip(client);
	gsl_load_fwx680(client);
	gsl_setup_chip(client);
	gsl_reset_chip(client);
	gsl_setup_chip(client);
}

static int gsl_check32(struct i2c_client *client)
{
	u32 ck32;
	int err;

	if((err = gsl_read_i2c(client, GSL_REG_CHECK, (u8 *)&ck32, sizeof(ck32))) >= 0) {
		pr_debug("GSLX680: check32 0x%08X\n", ck32);
		if(ck32 != GSL_CKECK32) {
			pr_warning("GSLX680: check 0x%08X not pass\n", ck32);
			return -EINVAL;
		}
	}
	return err;
}

static void gsl_ts_report_event(struct gsl_ts *ts)
{
	u8 id = 0; // use single point only

	if(ts->finger > 0) {
		struct gslx680_platform_data* pdata = ts->pdata;
		u16 x = ts->event.x[id];
		u16 y = ts->event.y[id];

		if(x <= pdata->x_max && y <= pdata->y_max) {
			// pr_debug("GSLX680: report (%d, %d)\n", x, y);
			input_report_abs(ts->input_dev, ABS_X, x);
			input_report_abs(ts->input_dev, ABS_Y, y);
			input_report_abs(ts->input_dev, ABS_PRESSURE, GSL_PRESS);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
			input_sync(ts->input_dev);
		}
		// else {
		// 	pr_debug("GSLX680: outside (%d, %d)\n", x, y);
		// }
	}
	else {
		// pr_debug("GSLX680: report release finger %d\n", id);
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_sync(ts->input_dev);
	}
}

static void gsl_ts_filter_data(struct gsl_ts *ts)
{
	u8 finger = GSL_FINGERS(ts->finger);
	struct gsl_ts_event *ev = &ts->event;
	u8 *touch = ts->touch;
	u8 i;

	for(i = 0; i < finger; i++) {
		ev->id[i] = GSL_IDATA(touch);
		ev->x[i] = GSL_XDATA(touch);
		ev->y[i] = GSL_YDATA(touch);
		// pr_debug("GSLX680: touch (%d,%d) at %d\n", ev->x[0], ev->y[0], ev->id[0]);

		touch += 4;
	}
}

static int gsl_ts_read_data(struct gsl_ts *ts)
{
	struct i2c_client *client = ts->client;
	// u8 buf[4] = {0};
	int err;

	if((err = gsl_read_i2c(client, GSL_REG_FINGER, &ts->finger, sizeof(ts->finger))) < 0) {
		pr_err("GSLX680: failed to read finger\n");
		return err;
	}

	if(ts->finger > 0) {
		u8 finger = GSL_FINGERS(ts->finger);
		u8 datalen = 4*finger;

		if((err = gsl_read_i2c(client, GSL_REG_TOUCH, ts->touch, datalen)) < 0) {
			pr_err("GSLX680: failed to read touch\n");
			return err;
		}
	}

	// check power lost
	// if((err = gsl_read_i2c(ts->client, GSL_REG_POWER, buf, sizeof(buf))) < 0) {
	// 	pr_err("GSLX680: failed to read power status\n");
	// 	return err;
	// }
	return 0;
}

static int gsl_ts_init_input(struct gsl_ts *ts)
{
	struct gslx680_platform_data *pdata = ts->pdata;
	struct i2c_client *client = ts->client;
	struct input_dev *idev;
	int err;

	idev = input_allocate_device();
	if(!idev) {
		pr_err("GSLX680: failed to allocate input device\n");
		return -ENOMEM;
	}

	ts->input_dev = idev;
	idev->uniq = "main_ts";
	idev->name = GSLX680_NAME;
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &client->dev;
	input_set_drvdata(idev, ts);

	set_bit(EV_KEY, idev->evbit);
	set_bit(EV_ABS, idev->evbit);
	set_bit(EV_SYN, idev->evbit);
	set_bit(ABS_PRESSURE, idev->absbit);
	set_bit(ABS_X, idev->absbit);
	set_bit(ABS_Y, idev->absbit);
	set_bit(BTN_TOUCH, idev->keybit);

	input_set_abs_params(idev, ABS_X, 0, pdata->x_max, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, pdata->y_max, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE, 0, GSL_PRESS_MAX, 0 , 0);

	if((err = input_register_device(idev)) < 0) {
		pr_err("GSLX680: failed to regist input device\n");
		goto free_input;
	}
	return 0;

free_input:
	input_free_device(idev);
	return err;
}

static irqreturn_t gsl_ts_do_irq(int irq, void *dev_id)
{
	struct gsl_ts *ts = (struct gsl_ts *)dev_id;

	// pr_debug("GSLX680: interrupt\n");

	disable_irq_nosync(ts->irq);

	if(!delayed_work_pending(&ts->data_work))
		queue_delayed_work(ts->workqueue, &ts->data_work, msecs_to_jiffies(30));
	else
		enable_irq(ts->irq);
	return IRQ_HANDLED;
}

static int gsl_ts_init_irq(struct gsl_ts *ts)
{
	struct gslx680_platform_data *pdata = ts->pdata;
	int err;

	if((err = request_irq(ts->irq, gsl_ts_do_irq, pdata->irqflags, "gsl irq", ts)) < 0) {
		pr_err("GSLX680: failed to request irq\n");
		return err;
	}
	disable_irq(ts->irq);
	return 0;
}

#ifdef CONFIG_TOUCHSCREEN_GSLX680_ANTI_SHAKE
static void gsl_ts_anti_shake(struct gsl_ts *ts)
{
	u8 finger = GSL_FINGERS(ts->finger);
	struct gsl_touch_info cinfo = {
		.x = {0},
		.y = {0},
		.id = {0},
		.finger_num = ts->finger,
	};
	u32 tmp;
	int i;

	for(i = 0; i < finger; i++) {
		cinfo.x[i] = ts->event.y[i]; // important!!
		cinfo.y[i] = ts->event.x[i]; // important!!
		cinfo.id[i] = ts->event.id[i];
	}

	gsl_alg_id_main(&cinfo);
	tmp = gsl_mask_tiaoping();
	// pr_debug("GSLX680: tmp=%x\n",tmp);
	if((tmp > 0) && (tmp < 0xffffffff)) {
		u8 buf[4] = {0xa, 0, 0, 0};

		gsl_write_i2c(ts->client, GSL_REG_PAGE, buf, sizeof(buf));
		buf[0] = (u8)(tmp & 0xff);
		buf[1] = (u8)((tmp >> 8) & 0xff);
		buf[2] = (u8)((tmp >> 16) & 0xff);
		buf[3] = (u8)((tmp >> 24) & 0xff);
		gsl_write_i2c(ts->client, 0x8, buf, sizeof(buf));
	}

	pr_debug("GSLX680: anti-shake finger %d (%d, %d) -> (%d, %d)\n",
		ts->finger, ts->event.x[0], ts->event.y[0], cinfo.x[0], cinfo.y[0]);

	ts->event.x[0] = cinfo.x[0];
	ts->event.y[0] = cinfo.y[0];
	ts->event.id[0] = cinfo.id[0];
}
#endif

static void gsl_ts_do_data_work(struct work_struct *work)
{
	struct gsl_ts *ts = container_of((struct delayed_work *)work, struct gsl_ts, data_work);
	int err;

	if((err = gsl_ts_read_data(ts)) == 0) {
		gsl_ts_filter_data(ts);
#ifdef CONFIG_TOUCHSCREEN_GSLX680_ANTI_SHAKE
		gsl_ts_anti_shake(ts);
#endif
		gsl_ts_report_event(ts);
	}

	enable_irq(ts->irq);
}

static void gsl_ts_do_reset_work(struct work_struct *work)
{
	struct gsl_ts *ts = container_of((struct delayed_work *)work, struct gsl_ts, reset_work);
	struct gslx680_platform_data *pdata = ts->pdata;
	struct i2c_client *client = ts->client;

	pr_debug("GSLX680: reset_work\n");

	gsl_power_on(pdata);
	gsl_reset(pdata);
	gsl_init_chip(client);
	msleep(30);
	if(gsl_check32(client) < 0) {
		pr_debug("GSLX680: failed to check\n");
		gsl_reset(pdata);
		gsl_init_chip(client);
	}

	enable_irq(ts->irq);
}

static int gsl_ts_init_work(struct gsl_ts *ts)
{
	INIT_DELAYED_WORK(&ts->data_work, gsl_ts_do_data_work);
	INIT_DELAYED_WORK(&ts->reset_work, gsl_ts_do_reset_work);

	ts->workqueue = create_singlethread_workqueue("gsl_ts");
	if(IS_ERR(ts->workqueue)) {
		pr_err("GSLX680: failed to create workqueue\n");
		return -ENOMEM;
	}
	return 0;
}

static int gsl_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct gslx680_platform_data *pdata = client->dev.platform_data;
	struct gsl_ts *ts;
	int err;

	pr_debug("GSLX680: Enter probe\n");

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C functionality not supported\n");
		return -ENODEV;
	}

	if(!pdata) {
		pr_err("GSLX680: failed to handle null platform data!\n");
		return -ENODEV;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if(!ts) {
        pr_err("GSLX680: allocate data fail!\n");
		return -ENOMEM;
	}
	ts->pdata = pdata;
	ts->client = client;
	ts->irq = gpio_to_irq(pdata->irq);
	i2c_set_clientdata(ts->client, ts);

	if((err = gsl_init_gpio(ts->pdata)) < 0)
		goto free_ts;

	if ((err = gsl_ts_init_input(ts)) < 0)
		goto free_gpio;

	if((err = gsl_ts_init_work(ts)) < 0)
		goto free_input;

	if((err = gsl_ts_init_irq(ts)) < 0)
		goto free_workqueue;

	queue_delayed_work(ts->workqueue, &ts->reset_work, msecs_to_jiffies(500));

	pr_info("GSLX680: init OK\n");
	return 0;

free_workqueue:
	destroy_workqueue(ts->workqueue);
free_input:
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
free_gpio:
	gpio_free(pdata->reset);
	gpio_free(pdata->irq);
free_ts:
	kfree(ts);
	pr_err("GSLX680: error code %d\n", err);
	return err;
}

static int gsl_ts_remove(struct i2c_client *client)
{
	struct gsl_ts *ts = i2c_get_clientdata(client);

	pr_info("GSLX680: remove\n");

	device_init_wakeup(&client->dev, 0);

	destroy_workqueue(ts->workqueue);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

#ifdef CONFIG_PM
static int gsl_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gsl_ts *ts = i2c_get_clientdata(client);

	pr_info("GSLX680: suspend\n");

	disable_irq(ts->irq);
	gsl_shutdown(ts->pdata);
	gsl_power_off(ts->pdata);
	return 0;
}

static int gsl_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gsl_ts *ts = i2c_get_clientdata(client);

	pr_info("GSLX680: resume\n");

	queue_delayed_work(ts->workqueue, &ts->reset_work, msecs_to_jiffies(500));
	return 0;
}
#else
#define gsl_ts_suspend	NULL
#define gsl_ts_resume	NULL
#endif

static const struct i2c_device_id gsl_ts_id[] = {
	{GSLX680_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, gsl_ts_id);

static const struct dev_pm_ops gsl_ts_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(gsl_ts_suspend, gsl_ts_resume)
};

static struct i2c_driver gsl_ts_driver = {
	.probe		= gsl_ts_probe,
	.remove		= gsl_ts_remove,
	.id_table	= gsl_ts_id,
	.driver = {
		.name = GSLX680_NAME,
		.owner = THIS_MODULE,
		.pm = &gsl_ts_pm_ops,
	},
};

static int __init gsl_ts_init(void)
{
	return i2c_add_driver(&gsl_ts_driver);
}

static void __exit gsl_ts_exit(void)
{
	i2c_del_driver(&gsl_ts_driver);
}

module_init(gsl_ts_init);
module_exit(gsl_ts_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GSLX680 touchscreen controller driver");
MODULE_AUTHOR("GoffTang gofftang@gmail.com");
