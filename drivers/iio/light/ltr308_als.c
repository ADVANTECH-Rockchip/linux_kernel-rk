#include <linux/time.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "ltr308_als.h"

static const struct iio_chan_spec ltr308_als_channels[] =
{
    {
        .type	= IIO_LIGHT,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
        BIT(IIO_CHAN_INFO_SCALE),
    }
};


static int ltr308_i2c_read_reg(struct i2c_client *client, u8 reg)
{
    int ret = 0;
    struct iio_dev *indio_dev = i2c_get_clientdata(client);
    struct LTR308_data *data = iio_priv(indio_dev);

    ret = i2c_smbus_read_byte_data(data->client, reg);
    return ret;
}

static int ltr308_i2c_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
    int ret = 0;
    struct iio_dev *indio_dev = i2c_get_clientdata(client);
    struct LTR308_data *data = iio_priv(indio_dev);
    ret = i2c_smbus_write_byte_data(data->client, reg, value);

    if (ret < 0){
		printk("i2c write error, ret: %d\n",ret);
        return ret;
	}
    else
        return 0;
}


static void ltr308_dump_regs(struct i2c_client *client){
	
	LTR308_DEBUG("LTR308_MAIN_CTRL[0x%02X]         = [%02X]\n", LTR308_MAIN_CTRL,ltr308_i2c_read_reg(client, LTR308_MAIN_CTRL));
	LTR308_DEBUG("LTR308_ALS_MEAS_RATE[0x%02X]     = [%02X]\n", LTR308_ALS_MEAS_RATE,ltr308_i2c_read_reg(client, LTR308_ALS_MEAS_RATE));
	LTR308_DEBUG("LTR308_ALS_GAIN[0x%02X]          = [%02X]\n", LTR308_ALS_GAIN,ltr308_i2c_read_reg(client, LTR308_ALS_GAIN));
	LTR308_DEBUG("LTR308_PART_ID[0x%02X]           = [%02X]\n", LTR308_PART_ID,ltr308_i2c_read_reg(client, LTR308_PART_ID));
	LTR308_DEBUG("LTR308_MAIN_STATUS[0x%02X]       = [%02X]\n", LTR308_MAIN_STATUS,ltr308_i2c_read_reg(client, LTR308_MAIN_STATUS));
	LTR308_DEBUG("LTR308_ALS_DATA_0[0x%02X]        = [%02X]\n", LTR308_ALS_DATA_0,ltr308_i2c_read_reg(client, LTR308_ALS_DATA_0));
	LTR308_DEBUG("LTR308_ALS_DATA_1[0x%02X]        = [%02X]\n", LTR308_ALS_DATA_1,ltr308_i2c_read_reg(client, LTR308_ALS_DATA_1));
	LTR308_DEBUG("LTR308_ALS_DATA_2[0x%02X]        = [%02X]\n", LTR308_ALS_DATA_2,ltr308_i2c_read_reg(client, LTR308_ALS_DATA_2));
	LTR308_DEBUG("LTR308_INT_CFG[0x%02X]           = [%02X]\n", LTR308_INT_CFG,ltr308_i2c_read_reg(client, LTR308_INT_CFG));
	LTR308_DEBUG("LTR308_INT_PST[0x%02X]           = [%02X]\n", LTR308_INT_PST,ltr308_i2c_read_reg(client, LTR308_INT_PST));
	LTR308_DEBUG("LTR308_ALS_THRES_UP_0[0x%02X]    = [%02X]\n", LTR308_ALS_THRES_UP_0,ltr308_i2c_read_reg(client, LTR308_ALS_THRES_UP_0));
	LTR308_DEBUG("LTR308_ALS_THRES_UP_1[0x%02X]    = [%02X]\n", LTR308_ALS_THRES_UP_1,ltr308_i2c_read_reg(client, LTR308_ALS_THRES_UP_1));
	LTR308_DEBUG("LTR308_ALS_THRES_UP_2[0x%02X]    = [%02X]\n", LTR308_ALS_THRES_UP_2,ltr308_i2c_read_reg(client, LTR308_ALS_THRES_UP_2));
	LTR308_DEBUG("LTR308_ALS_THRES_LOW_0[0x%02X]   = [%02X]\n", LTR308_ALS_THRES_LOW_0,ltr308_i2c_read_reg(client, LTR308_ALS_THRES_LOW_0));
	LTR308_DEBUG("LTR308_ALS_THRES_LOW_1[0x%02X]   = [%02X]\n", LTR308_ALS_THRES_LOW_1,ltr308_i2c_read_reg(client, LTR308_ALS_THRES_LOW_1));
	LTR308_DEBUG("LTR308_ALS_THRES_LOW_2[0x%02X]   = [%02X]\n", LTR308_ALS_THRES_LOW_2,ltr308_i2c_read_reg(client, LTR308_ALS_THRES_LOW_2));

	
	
}

static int ltr308_als_enable(struct i2c_client *client, int enable)
{
    int ret;
    u8 regdata = 0;

    regdata = ltr308_i2c_read_reg(client, LTR308_MAIN_CTRL);
    if (enable != 0)
    {
        LTR308_DEBUG("ALS(1): enable als only \n");
        regdata |= 0x02;
    }
    else
    {
        LTR308_DEBUG("ALS(1): disable als only \n");
        regdata &= 0xFD;
    }

    ret = ltr308_i2c_write_reg(client, LTR308_MAIN_CTRL, regdata);
    if (ret < 0)
    {
        LTR308_DEBUG("ALS: enable als err: %d en: %d \n", ret, enable);
        return ret;
    }

    LTR308_DEBUG("0x00  = [%02x]\n", ltr308_i2c_read_reg(client, 0x00));
    mdelay(WAKEUP_DELAY);

    return ret;
}


static int ltr308_als_reset(struct i2c_client *client)
{
    int ret=0;

    ret = ltr308_i2c_write_reg(client, LTR308_MAIN_CTRL, 0x10);
    
    LTR308_DEBUG("0x00  = [%02x]\n", ltr308_i2c_read_reg(client, 0x00));
    return ret;
}

static int ltr308_dev_init(struct i2c_client *client)
{
    int ret=0;

    msleep(PON_DELAY);
    LTR308_DEBUG("PART_ID[0x%02X]     = [%02X]\n", LTR308_PART_ID,ltr308_i2c_read_reg(client, LTR308_PART_ID));
    LTR308_DEBUG("STATUS [0x%02X]     = [%02X]\n", LTR308_MAIN_STATUS,ltr308_i2c_read_reg(client, LTR308_MAIN_STATUS));
    msleep(PON_DELAY);

 //   ret = ltr308_als_reset(client);
 //   if (ret < 0)
 //       return ret;

    mdelay(WAKEUP_DELAY);
    ltr308_als_enable(client,1); //ted add
#if LTR308DBG
    ltr308_dump_regs(client);
#endif
   return ret;
}

static ssize_t als_value_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    int alsval_0, alsval_1, alsval_2;
    long alsval;
    long  luxdata_int;
    struct i2c_client *client = container_of(dev,struct i2c_client,dev);

        alsval_0 = ltr308_i2c_read_reg(client, LTR308_ALS_DATA_0);
        alsval_1 = ltr308_i2c_read_reg(client, LTR308_ALS_DATA_1);
        alsval_2 = ltr308_i2c_read_reg(client, LTR308_ALS_DATA_2);
        alsval = (alsval_2<<16) + (alsval_1<<8) + alsval_0;
        if (alsval == 0)
            luxdata_int = 0;
        else
            luxdata_int = (alsval*8/10);

        

  *buf = 0;
  sprintf(buf, "%lu\n", luxdata_int);
  return strlen(buf);
  

}


static struct device_attribute dev_attr_als_value =
    __ATTR(als_data, S_IRUGO , als_value_show, NULL);

static struct attribute *sysfs_attrs[] =
{
    &dev_attr_als_value.attr,
    NULL
};

static struct attribute_group attribute_group =
{
    .attrs = sysfs_attrs,
};


static int ltr308_als_read_raw(struct iio_dev *indio_dev,
                               struct iio_chan_spec const *chan, int *val,
                               int *val2, long mask)
{

    int alsval_0, alsval_1, alsval_2, alsval;
    int luxdata_int;
    struct LTR308_data *data = iio_priv(indio_dev);
    switch (mask)
    {
    case IIO_CHAN_INFO_RAW:

        alsval_0 = ltr308_i2c_read_reg(data->client, LTR308_ALS_DATA_0);
        alsval_1 = ltr308_i2c_read_reg(data->client, LTR308_ALS_DATA_1);
        alsval_2 = ltr308_i2c_read_reg(data->client, LTR308_ALS_DATA_2);
        alsval = (alsval_2<<16) + (alsval_1<<8) + alsval_0;

        if (alsval == 0)
            luxdata_int = 0;
        else
            luxdata_int = (alsval*8/10);

        *val = luxdata_int;
        return IIO_VAL_INT;
    case IIO_CHAN_INFO_SCALE:


        return IIO_VAL_INT_PLUS_MICRO;
    }
    return -EINVAL;
}

static int ltr308_als_write_raw(struct iio_dev *indio_dev,
                                struct iio_chan_spec const *chan, int val,
                                int val2, long mask)
{

    switch (mask)
    {
    case IIO_CHAN_INFO_SCALE:

        break;
    }
    return -EINVAL;
}


static const struct iio_info ltr308_als_info =
{
    .driver_module	= THIS_MODULE,
    .read_raw	= ltr308_als_read_raw,
    .write_raw	= ltr308_als_write_raw,
};

static int ltr308_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    struct LTR308_data *data;
    struct iio_dev *indio_dev;
    struct i2c_adapter *adapter;

    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;


    adapter = to_i2c_adapter(client->dev.parent);

    /* Return 1 if adapter supports everything we need, 0 if not. */
    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE | I2C_FUNC_SMBUS_READ_BYTE_DATA))
    {
        printk(KERN_INFO "%s: LTR308-ALS functionality check failed.\n", __func__);
        ret = -EIO;
        return ret;
    }

    data = iio_priv(indio_dev);
    i2c_set_clientdata(client, indio_dev);
    data->client = client;

    indio_dev->dev.parent = &client->dev;
    indio_dev->info = &ltr308_als_info;
    indio_dev->name = LTR308_DEVICE_NAME;
    indio_dev->channels = ltr308_als_channels;
    indio_dev->num_channels = ARRAY_SIZE(ltr308_als_channels);
    indio_dev->modes = INDIO_DIRECT_MODE;

    ret = ltr308_dev_init(client);
    if (ret)
    {
        printk(KERN_INFO "%s: LTR308-ALS device init failed.\n", __func__);
        goto err_exit;
    }


    ret = devm_iio_device_register(&client->dev, indio_dev);
    if(ret < 0)
		goto err_exit;
	ret = sysfs_create_group(&indio_dev->dev.kobj, &attribute_group);
    if(ret < 0)
		goto err_exit;
		
    return ret;

err_exit:
    return ret;
}

static int ltr308_remove(struct i2c_client *client)
{
    struct iio_dev *indio_dev = i2c_get_clientdata(client);

    ltr308_als_reset(client);
    iio_device_unregister(indio_dev);
    return 0;
}

static const struct acpi_device_id ltr_acpi_match[] =
{
    {"LTER0308", ltr308},
    { },
};

MODULE_DEVICE_TABLE(acpi, ltr_acpi_match);

static const struct i2c_device_id ltr308_id[] =
{
    { LTR308_DEVICE_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, ltr308_id);

static const struct of_device_id ltr308_match_table[] =
{
    { .compatible = "lite_on,ltr308",},
    { },
};

MODULE_DEVICE_TABLE(of, ltr_match_table);

static struct i2c_driver ltr308_driver =
{
    .driver 	= {
        .owner = THIS_MODULE,
        .name  = LTR308_DEVICE_NAME,
        .of_match_table = of_match_ptr(ltr308_match_table),
    },
    .probe		= ltr308_probe,
    .remove	= ltr308_remove,
    .id_table	= ltr308_id,

};

module_i2c_driver(ltr308_driver);

MODULE_AUTHOR("Advantech 2019");
MODULE_DESCRIPTION("LTR308-ALS Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

