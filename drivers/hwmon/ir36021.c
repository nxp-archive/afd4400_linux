/*
 * Driver for IR36021 buck controller chip
 *
 * IR36021:
 * Digital POL Buck Controller with I2C and PMBus Interface
 * Datasheet: http://www.irf.com/product-info/datasheets/data/ir36021.pdf
 *
 * Copyright (C) 2015 Freescale
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/jiffies.h>

/* I2C register addresses */
#define IR36021_VOUT_MODE	(0x20) /* read / write byte */
#define IR36021_READ_VIN	(0x88) /* read word */
#define IR36021_READ_IIN	(0x89) /* read word */
#define IR36021_READ_VOUT	(0x8B) /* read word */
#define IR36021_READ_IOUT	(0x8C) /* read word */
#define IR36021_READ_TEMP_1	(0x8D) /* read word */
#define IR36021_READ_TEMP_2	(0x8E) /* read word */
#define IR36021_READ_POUT	(0x96) /* read word */
#define IR36021_READ_PIN	(0x97) /* read word */
#define IR36021_MFR_MODEL	(0x9A) /* read word */
#define IR36021_MFR_MODEL_VALUE	(0x2D01) /* IR36021 */

#define ISCALE_BASE		(256) /* Base I scale factor */

#define LIN11			(0) /* 11 bit Linear */
#define LIN11_1000		(1) /* 11 bit linear in thousandths */
#define LIN16			(2) /* 16 bit linear */
#define LIN16_1000		(3) /* 16 bit linear in thousandths */
#define ILIN11_1000		(5) /* 11 bit linear in thousandths * iscale */

struct ir36021_data {
	struct device *hwmon_dev;
	struct mutex update_lock;
	int    vout_exponent;
	int    iscale;
};

static ssize_t ir36021_show_value(struct device *dev,
				 struct device_attribute *da, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ir36021_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int command = attr->index & 0xFF; // Extract SMBUS command
	int format = (attr->index >> 8) & 3; // Extract data/presentation format
	int iadjust = (attr->index >> 10); // Extract iadjust command
	int val;
	int exp = data->vout_exponent;
	s16 y;

	mutex_lock(&data->update_lock);
	val = i2c_smbus_read_word_data(client, command);
	mutex_unlock(&data->update_lock);

	dev_dbg(dev, "ir36021 read command %02X => 0x%04X\n", command, val);
	if (val < 0)
		return 0;

	if (format == LIN16 || format == LIN16_1000) {
		exp = data->vout_exponent;
		y = val;
	} else {
		y = val;
		exp = y >> 11;
		y = val << 6;
		y = y >> 6;
	}

	val = y;
	/* Convert to thousandths */
	if (format == LIN11_1000 || format == LIN16_1000)
		val *= 1000;
	dev_dbg(dev, "y = %d, exp = %d => %d\n", y, exp, val);

	/* Apply iadjust scaling */
	if (iadjust) {
		val *= data->iscale;
		exp -= 8;
	}
	/* Apply exponent */
	val = exp >= 0 ? val << exp : val >> -exp;

	dev_dbg(dev, "y = %d, exp = %d => %d\n", y, exp, val);
	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

#define IR36021_ATTR(sensor, reg, format) \
static SENSOR_DEVICE_ATTR(sensor##_input, S_IRUGO, \
	ir36021_show_value, NULL, reg | (format<<8) )

IR36021_ATTR(in0,    IR36021_READ_VIN,    LIN11_1000); /* input voltage  (mV)*/
IR36021_ATTR(curr0,  IR36021_READ_IIN,    ILIN11_1000);/* input current  (mA)*/
IR36021_ATTR(power0, IR36021_READ_PIN,    ILIN11_1000);/* input power    (mW)*/
IR36021_ATTR(in1,    IR36021_READ_VOUT,   LIN16_1000); /* output voltage (mV)*/
IR36021_ATTR(curr1,  IR36021_READ_IOUT,   ILIN11_1000);/* output current (mA)*/
IR36021_ATTR(power1, IR36021_READ_POUT,   ILIN11_1000);/* output power   (mW)*/
IR36021_ATTR(temp1,  IR36021_READ_TEMP_1, LIN11);      /* temperature 1  (C) */
IR36021_ATTR(temp2,  IR36021_READ_TEMP_2, LIN11);      /* temperature 2  (C) */

/* pointers to created device attributes */
static struct attribute *ir36021_attributes[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_curr0_input.dev_attr.attr,
	&sensor_dev_attr_power0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_curr1_input.dev_attr.attr,
	&sensor_dev_attr_power1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	NULL,
};

static const struct attribute_group ir36021_group = {
	.attrs = ir36021_attributes,
};

static int ir36021_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = client->adapter;
	struct ir36021_data *data;
	int ret = -ENODEV;
	int val;
	s8 exp;
	char buf[32];
	long iscale = ISCALE_BASE;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		goto failed;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto failed;
	}

	if (client->dev.of_node) {
		const __be32 *property;
		int len;
		/* Get iscale value */
		property = of_get_property(client->dev.of_node,
						"iscale", &len);
		if (property && len == sizeof(int))
			iscale = be32_to_cpup(property);
	}
	if (iscale < 1)
		goto failed;

	data->iscale = iscale;
	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);

	/* check device model */
	val = i2c_smbus_read_word_data(client, IR36021_MFR_MODEL);
	dev_dbg(&client->dev, "ir36021 mfr_model 0x%04X\n", val);
	if (val != IR36021_MFR_MODEL_VALUE)
		goto failed;

	/* read device vout configuration */
	val = i2c_smbus_read_byte_data(client, IR36021_VOUT_MODE);
	dev_dbg(&client->dev, "ir36021 vout mode 0x%02X\n", val);
	if (val < 0 || val > 0x1F)
		goto failed;

	exp = val << 3; /* sign extend */
	exp = exp >> 3; /* sign extend */
	data->vout_exponent = exp;

	ret = sysfs_create_group(&client->dev.kobj, &ir36021_group);
	if (ret)
		goto failed;

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		ret = PTR_ERR(data->hwmon_dev);
		goto out_err_hwmon;
	}

	val = ir36021_show_value(&client->dev,
		&sensor_dev_attr_in1_input.dev_attr, buf);
	if (val) {
		buf[val-1] = 0;
		dev_info(&client->dev, "vout = %s mV (iscale = %d/%d)\n",
			 buf, data->iscale, ISCALE_BASE);
	}
	return 0;

out_err_hwmon:
	sysfs_remove_group(&client->dev.kobj, &ir36021_group);
failed:
	dev_warn(&client->dev, "probe failed\n");
	return ret;
}

static int ir36021_remove(struct i2c_client *client)
{
	struct ir36021_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &ir36021_group);

	return 0;
}

static const struct i2c_device_id ir36021_id[] = {
	{ "ir36021", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ir36021_id);

static struct i2c_driver ir36021_driver = {
	.driver = {
		.name	= "ir36021",
	},
	.probe		= ir36021_probe,
	.remove		= ir36021_remove,
	.id_table	= ir36021_id,
};

module_i2c_driver(ir36021_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ir36021 Driver");
