/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
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

#include <linux/input/ft6x06_ts.h>

// #undef pr_debug
// #define pr_debug pr_info

// registers
#define FT_REG_FINGER		0x02
#define FT_REG_TOUCH		0x03 //6bytes 0X03~0x08, 0x09~0x0E, ...
#define FT_REG_FIRMID		0xA6
#define FT_REG_FOCALTECH_ID	0xA8

#define FT_TOUCH_SIZE		6

// filters
#define FT_F_FINGER(f) ((f) & 0xF)

#define FT_F_EVENT(t) 	 ((t[0] >> 6) & 0x3)
#define FT_F_X(t) 		 (((t[0] & 0xF) << 8) | (t[1] & 0xFF))
#define FT_F_ID(t) 		 ((t[2] >> 4) & 0xF)
#define FT_F_Y(t) 		 (((t[2] & 0xF) << 8) | (t[3] & 0xFF))
#define FT_F_PRESSURE(t) (t[4] & 0xFF)
#define FT_F_AREA(t) 	 (t[5] & 0xFF)

// event flag
#define FT_EV_PRESS_DOWN	0
#define FT_EV_LEFT_UP		1
#define FT_EV_CONTACT		2
#define FT_EV_NONE			3
#define FT_IS_PRESS(x) ((x) == FT_EV_PRESS_DOWN || (x) == FT_EV_CONTACT)

#define FT_ID_INVALID(id) ((id) == 0xF)

//
#define FT_PRESS_MAX    		255

#define FT_MAX_FINGERS 	CONFIG_TOUCHSCREEN_FT6X06_POINT
#define FT_FINGERS(f) ((f) < FT_MAX_FINGERS ? (f) : FT_MAX_FINGERS)
#if FT_MAX_FINGERS > 1
#define FT_MULTI_TOUCH_POINT
#endif

struct ts_event {
	u8 event[FT_MAX_FINGERS];
	u8 id[FT_MAX_FINGERS];
	u16 x[FT_MAX_FINGERS];
	u16 y[FT_MAX_FINGERS];
	u8 pressure[FT_MAX_FINGERS];
	u8 area[FT_MAX_FINGERS];
};

struct ft_ts {
	struct i2c_client *client;
	struct input_dev *input_dev;

	int irq;
	struct regulator* vdd;
	struct ft6x06_platform_data *pdata;

	struct delayed_work reset_work;
	struct delayed_work data_work;
	struct workqueue_struct *workqueue;

	u8 finger;
	u8 touch[FT_TOUCH_SIZE * FT_MAX_FINGERS];

	struct ts_event event;
} *my_ts;

static int ft_read_i2c(struct i2c_client *client, u8 reg, u8 *pdata, u8 datalen)
{
	return  i2c_smbus_read_i2c_block_data(client, reg, datalen, pdata);
}

// static int ft_write_i2c(struct i2c_client *client, u8 reg, u8 *pdata, u8 datalen)
// {
// 	return  i2c_smbus_write_i2c_block_data(client, reg, datalen, pdata);
// }

static int ft_init_gpio(struct ft6x06_platform_data *pdata)
{
	int err;

	if((err = gpio_request(pdata->reset, "gslx680 reset")) < 0) {
		pr_err("FT6X06: failed to request reset gpio %d\n", pdata->reset);
		return err;
	}
	gpio_direction_output(pdata->reset, 1);

	if((err = gpio_request(pdata->irq, "gslx680 int")) < 0) {
		pr_err("FT6X06: failed to request int gpio %d\n", pdata->irq);
		goto free_reset;
	}
	gpio_direction_input(pdata->irq);

	return 0;

free_reset:
	gpio_free(pdata->reset);
	return err;
}

static int ft_init_power(struct ft_ts *ts)
{
	struct ft6x06_platform_data *pdata = ts->pdata;
	if(IS_ERR_OR_NULL(pdata->vdd)) {
		pr_err("FT6X06: failed to null power name\n");
		return -EINVAL;
	}

	ts->vdd = regulator_get(NULL, pdata->vdd);
	if(IS_ERR_OR_NULL(ts->vdd)) {
		pr_err("FT6X06: failed to get the power\n");
		return -ENODEV;
	}
	return 0;
}

static int ft_power_on(struct ft_ts *ts)
{
	return regulator_enable(ts->vdd);
}

static int ft_power_off(struct ft_ts *ts)
{
	return regulator_disable(ts->vdd);
}

static void ft_reset(struct ft6x06_platform_data *pdata)
{
	gpio_set_value(pdata->reset, 1);
	msleep(5);
	gpio_set_value(pdata->reset, 0);
	msleep(10);
	gpio_set_value(pdata->reset, 1);
	msleep(15);
}

static void ft_shutdown(struct ft6x06_platform_data *pdata)
{
	gpio_set_value(pdata->reset, 0);
}

static int ft_check_firmware(struct i2c_client *client)
{
	u8 fw;
	int err;

	if((err = ft_read_i2c(client, FT_REG_FIRMID, &fw, sizeof(fw))) < 0) {
		pr_err("FT6X06: failed to get fw version\n");
		return -EINVAL;
	}
	else {
		pr_info("FT6X06: fw version %d\n", fw);
#ifdef CONFIG_TOUCHSCREEN_FT6X06_FW_UPG
		{
		 	extern void fts_get_upgrade_array(struct i2c_client *client);
			extern int fts_ctpm_auto_upgrade(struct i2c_client *client);

			fts_get_upgrade_array(client);
		    fts_ctpm_auto_upgrade(client);
		}
#endif
		return 0;
	}
}

static void ft_init_chip(struct i2c_client *client)
{
	ft_check_firmware(client);
}

static int ft_ts_init_input(struct ft_ts *ts)
{
	struct ft6x06_platform_data *pdata = ts->pdata;
	struct i2c_client *client = ts->client;
	struct input_dev *idev;
	int err;

	idev = input_allocate_device();
	if(!idev) {
		pr_err("FT6X06: failed to allocate input device\n");
		return -ENOMEM;
	}

	ts->input_dev = idev;
	idev->uniq = "main_ts";
	idev->name = FT6X06_NAME;
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &client->dev;
	input_set_drvdata(idev, ts);

	set_bit(EV_ABS, idev->evbit);
	set_bit(EV_KEY, idev->evbit);
	set_bit(EV_SYN, idev->evbit);
	set_bit(BTN_TOUCH, idev->keybit);
#ifdef FT_MULTI_TOUCH_POINT
	set_bit(ABS_MT_PRESSURE, idev->absbit);
	set_bit(ABS_MT_POSITION_X, idev->absbit);
	set_bit(ABS_MT_POSITION_Y, idev->absbit);
	set_bit(ABS_MT_TRACKING_ID, idev->absbit);

	input_set_abs_params(idev, ABS_MT_PRESSURE, 0, FT_PRESS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_MT_POSITION_X, 0, pdata->x_max, 0, 0);
	input_set_abs_params(idev, ABS_MT_POSITION_Y, 0, pdata->y_max, 0, 0);
#else
	set_bit(ABS_X, idev->absbit);
	set_bit(ABS_Y, idev->absbit);
	set_bit(ABS_PRESSURE, idev->absbit);

	input_set_abs_params(idev, ABS_X, 0, pdata->x_max, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, pdata->y_max, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE, 0, FT_PRESS_MAX, 0 , 0);
#endif

	if((err = input_register_device(idev)) < 0) {
		pr_err("FT6X06: failed to regist input device\n");
		goto free_input;
	}
	return 0;

free_input:
	input_free_device(idev);
	return err;
}

static irqreturn_t ft_ts_do_irq(int irq, void *dev_id)
{
	struct ft_ts *ts = (struct ft_ts *)dev_id;

	// pr_debug("FT6X06: interrupt\n");

	disable_irq_nosync(ts->irq);

	if(!delayed_work_pending(&ts->data_work))
		queue_delayed_work(ts->workqueue, &ts->data_work, msecs_to_jiffies(30));
	else
		enable_irq(ts->irq);
	return IRQ_HANDLED;
}

static int ft_ts_init_irq(struct ft_ts *ts)
{
	struct ft6x06_platform_data *pdata = ts->pdata;
	int err;

	if((err = request_irq(ts->irq, ft_ts_do_irq, pdata->irqflags, "gsl irq", ts)) < 0) {
		pr_err("FT6X06: failed to request irq\n");
		return err;
	}
	disable_irq(ts->irq);
	return 0;
}

static void ft_ts_filter_data(struct ft_ts *ts)
{
	struct ts_event *ev = &ts->event;
	u8 *touch = ts->touch;
	u8 i;

	ts->finger = FT_FINGERS(FT_F_FINGER(ts->finger));

	// pr_debug("FT6X06: finger(%d)\n", ts->finger);

	for(i = 0; i < ts->finger; i++) {
		ev->event[i] = FT_F_EVENT(touch);
		ev->id[i] = FT_F_ID(touch);
		ev->x[i] = FT_F_X(touch);
		ev->y[i] = FT_F_Y(touch);
		ev->pressure[i] = FT_F_PRESSURE(touch);
		ev->area[i] = FT_F_AREA(touch);

		touch += FT_TOUCH_SIZE;

		// pr_debug("FT6X06: event(%d) id(%d) xy(%d,%d) pressure(%d) area(%d)\n",
		// 	ev->event[i], ev->id[i], ev->x[i], ev->y[i], ev->pressure[i], ev->area[i]);
	}
}

static void ft_ts_report_event(struct ft_ts *ts)
{
#ifdef FT_MULTI_TOUCH_POINT
	struct ft6x06_platform_data *pdata = ts->pdata;
	u8 i;

	for(i = 0; i < ts->finger; i++) {
		u16 x = ts->event.x[i];
		u16 y = ts->event.y[i];
		u8 id = ts->event.id[i];
		u8 event = ts->event.event[i];
		u8 pressure = ts->event.pressure[i];

		// pr_debug("FT6X06: id(%d) xy(%d,%d) pressure(%d)\n", id, x, y, pressure);

		if(x > pdata->x_max) continue;
		if(y > pdata->y_max) continue;
		if(FT_ID_INVALID(id)) continue;

		pr_debug("FT6X06: report id(%d) xy(%d,%d)\n", id, x, y);

		if(FT_IS_PRESS(event)) {
			input_mt_slot(ts->input_dev, id);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, pressure);
		}
		else if(event == FT_EV_LEFT_UP) {
			input_mt_slot(ts->input_dev, id);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
		}
	}
#else
	if(ts->finger == 0) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
	}
	else {
		struct ft6x06_platform_data *pdata = ts->pdata;
		u8 pressure = ts->event.pressure[0];
		u16 x = ts->event.x[0];
		u16 y = ts->event.y[0];

		// pr_debug("FT6X06: xy(%d,%d)\n", x, y);
		if(x > pdata->x_max) return;
		if(y > pdata->y_max) return;

		pr_debug("FT6X06: report xy(%d,%d)\n", x, y);

		input_report_abs(ts->input_dev, ABS_X, x);
		input_report_abs(ts->input_dev, ABS_Y, y);
		input_report_abs(ts->input_dev, ABS_PRESSURE, pressure);
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
	}
#endif
	input_sync(ts->input_dev);
}

static int ft_ts_read_data(struct ft_ts *ts)
{
	struct i2c_client *client = ts->client;
	int err;

	if((err = ft_read_i2c(client, FT_REG_FINGER, &ts->finger, sizeof(ts->finger))) < 0) {
		pr_err("FT6X06: failed to read finger\n");
		return err;
	}

	if((err = ft_read_i2c(client, FT_REG_TOUCH, ts->touch, sizeof(ts->touch))) < 0) {
		pr_err("FT6X06: failed to read touch\n");
		return err;
	}
	return 0;
}

static void ft_ts_do_data_work(struct work_struct *work)
{
	struct ft_ts *ts = container_of((struct delayed_work *)work, struct ft_ts, data_work);
	int err;

	if((err = ft_ts_read_data(ts)) == 0) {
		ft_ts_filter_data(ts);
		ft_ts_report_event(ts);
	}

	enable_irq(ts->irq);
}

static void ft_ts_do_reset_work(struct work_struct *work)
{
	struct ft_ts *ts = container_of((struct delayed_work *)work, struct ft_ts, reset_work);
	struct ft6x06_platform_data *pdata = ts->pdata;
	struct i2c_client *client = ts->client;

	pr_debug("FT6X06: reset_work\n");

	ft_power_on(ts);
	ft_reset(pdata);
	msleep(150);
	ft_init_chip(client);
	enable_irq(ts->irq);
}

static int ft_ts_init_work(struct ft_ts *ts)
{
	INIT_DELAYED_WORK(&ts->data_work, ft_ts_do_data_work);
	INIT_DELAYED_WORK(&ts->reset_work, ft_ts_do_reset_work);

	ts->workqueue = create_singlethread_workqueue("ft_ts");
	if(IS_ERR(ts->workqueue)) {
		pr_err("FT6X06: failed to create workqueue\n");
		return -ENOMEM;
	}
	return 0;
}

static int ft_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft6x06_platform_data *pdata = client->dev.platform_data;
	struct ft_ts *ts;
	int err;

	pr_debug("FT6X06: Enter probe\n");

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C functionality not supported\n");
		return -ENODEV;
	}

	if(!pdata) {
		pr_err("FT6X06: failed to handle null platform data\n");
		return -ENODEV;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if(!ts) {
        pr_err("FT6X06: failed to allocate data\n");
		return -ENOMEM;
	}
	ts->pdata = pdata;
	ts->client = client;
	ts->irq = gpio_to_irq(pdata->irq);
	i2c_set_clientdata(ts->client, ts);

	ts->vdd = regulator_get(NULL, "");

	if((err = ft_init_gpio(ts->pdata)) < 0)
		goto free_ts;

	if((err = ft_init_power(ts)) < 0)
		goto free_gpio;

	if ((err = ft_ts_init_input(ts)) < 0)
		goto free_power;

	if((err = ft_ts_init_work(ts)) < 0)
		goto free_input;

	if((err = ft_ts_init_irq(ts)) < 0)
		goto free_workqueue;

	queue_delayed_work(ts->workqueue, &ts->reset_work, msecs_to_jiffies(500));

	pr_info("FT6X06: init OK\n");
	return 0;

free_workqueue:
	destroy_workqueue(ts->workqueue);
free_input:
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
free_power:
	regulator_put(ts->vdd);
free_gpio:
	gpio_free(pdata->reset);
	gpio_free(pdata->irq);
free_ts:
	kfree(ts);
	pr_err("FT6X06: error code %d\n", err);
	return err;
}

static int ft_ts_remove(struct i2c_client *client)
{
	struct ft_ts *ts = i2c_get_clientdata(client);

	pr_info("FT6X06: remove\n");

	device_init_wakeup(&client->dev, 0);

	destroy_workqueue(ts->workqueue);
	input_unregister_device(ts->input_dev);
	regulator_put(ts->vdd);
	kfree(ts);
	return 0;
}

#ifdef CONFIG_PM
static int ft_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft_ts *ts = i2c_get_clientdata(client);

	pr_info("FT6X06: suspend\n");

	disable_irq(ts->irq);
	ft_shutdown(ts->pdata);
	ft_power_off(ts);
	return 0;
}

static int ft_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft_ts *ts = i2c_get_clientdata(client);

	pr_info("FT6X06: resume\n");

	queue_delayed_work(ts->workqueue, &ts->reset_work, msecs_to_jiffies(500));
	return 0;
}
#else
#define ft_ts_suspend	NULL
#define ft_ts_resume	NULL
#endif

static const struct i2c_device_id ft_ts_id[] = {
	{FT6X06_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ft_ts_id);

static const struct dev_pm_ops ft_ts_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ft_ts_suspend, ft_ts_resume)
};

static struct i2c_driver ft_ts_driver = {
	.probe		= ft_ts_probe,
	.remove		= ft_ts_remove,
	.id_table	= ft_ts_id,
	.driver = {
		.name = FT6X06_NAME,
		.owner = THIS_MODULE,
		.pm = &ft_ts_pm_ops,
	},
};

static int __init ft_ts_init(void)
{
	return i2c_add_driver(&ft_ts_driver);
}

static void __exit ft_ts_exit(void)
{
	i2c_del_driver(&ft_ts_driver);
}

module_init(ft_ts_init);
module_exit(ft_ts_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FT6X06 touchscreen controller driver");
MODULE_AUTHOR("GoffTang gofftang@gmail.com");
