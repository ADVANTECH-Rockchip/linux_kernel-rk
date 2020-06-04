#include <linux/time.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "ltrx1082.h"

static const struct iio_event_spec ltrx1082_als_event_spec[] =
{
    {
        .type = IIO_EV_TYPE_THRESH,
        .dir = IIO_EV_DIR_RISING,
        .mask_separate = BIT(IIO_EV_INFO_VALUE),
    }, {
        .type = IIO_EV_TYPE_THRESH,
        .dir = IIO_EV_DIR_FALLING,
        .mask_separate = BIT(IIO_EV_INFO_VALUE),
    }, {
        .type = IIO_EV_TYPE_THRESH,
        .dir = IIO_EV_DIR_EITHER,
        .mask_separate = BIT(IIO_EV_INFO_ENABLE) |
        BIT(IIO_EV_INFO_PERIOD),
    },

};

static const struct iio_event_spec ltrx1082_pxs_event_spec[] =
{
    {
        .type = IIO_EV_TYPE_THRESH,
        .dir = IIO_EV_DIR_RISING,
        .mask_separate = BIT(IIO_EV_INFO_VALUE),
    }, {
        .type = IIO_EV_TYPE_THRESH,
        .dir = IIO_EV_DIR_FALLING,
        .mask_separate = BIT(IIO_EV_INFO_VALUE),
    }, {
        .type = IIO_EV_TYPE_THRESH,
        .dir = IIO_EV_DIR_EITHER,
        .mask_separate = BIT(IIO_EV_INFO_ENABLE) |
        BIT(IIO_EV_INFO_PERIOD),
    },
};


#define LTRX1082_INTENSITY_CHANNEL(_idx, _addr, _mod, _shared, \
				 _evspec, _evsize) { \
	.type = IIO_INTENSITY, \
	.modified = 1, \
	.address = (_addr), \
	.channel2 = (_mod), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = (_shared), \
	.scan_index = (_idx), \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 16, \
		.storagebits = 16, \
		.endianness = IIO_CPU, \
	}, \
	.event_spec = _evspec,\
	.num_event_specs = _evsize,\
}

#define LTRX1082_LIGHT_CHANNEL() { \
	.type = IIO_LIGHT, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED), \
	.scan_index = -1, \
}

static const struct iio_event_spec ltrx1082_events[] =
{
    /* Proximity event */
    {
        .type = IIO_EV_TYPE_THRESH,
        .dir = IIO_EV_DIR_RISING,
        .mask_separate = BIT(IIO_EV_INFO_VALUE) |
        BIT(IIO_EV_INFO_ENABLE),
    },
    /* Out-of-proximity event */
    {
        .type = IIO_EV_TYPE_THRESH,
        .dir = IIO_EV_DIR_FALLING,
        .mask_separate = BIT(IIO_EV_INFO_VALUE) |
        BIT(IIO_EV_INFO_ENABLE),
    },
};


static const struct iio_chan_spec ltrx1082_channels[] =
{

    {
        .type = IIO_LIGHT,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
        //BIT(IIO_CHAN_INFO_RAW) |
        //BIT(IIO_CHAN_INFO_SCALE) |
        //BIT(IIO_CHAN_INFO_INT_TIME),
    },
    {
        .type = IIO_PROXIMITY,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
        //BIT(IIO_CHAN_INFO_RAW) |
        //BIT(IIO_CHAN_INFO_SCALE) |
        //BIT(IIO_CHAN_INFO_INT_TIME),
        .event_spec = ltrx1082_events,
        .num_event_specs = ARRAY_SIZE(ltrx1082_events),
    }

};

static int i2c_read_reg(struct i2c_client *client, u8 reg)
{
    int ret = 0;
    struct iio_dev *indio_dev = i2c_get_clientdata(client);
    struct LTRX1082_data *data = iio_priv(indio_dev);
    //LTR_DEBUG("--> %s:%d, reg: %02xh\n",__func__,__LINE__,reg);
    ret = i2c_smbus_read_byte_data(data->client, reg);
    return ret;
}

static int i2c_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
    int ret = 0;
    struct iio_dev *indio_dev = i2c_get_clientdata(client);
    struct LTRX1082_data *data = iio_priv(indio_dev);
    ret = i2c_smbus_write_byte_data(data->client, reg, value);

    if (ret < 0)
    {
        printk("i2c write error, ret: %d\n",ret);
        return ret;
    }
    else
        return 0;
}

static void dump_regs(struct i2c_client *client)
{

    LTR_DEBUG("LTRX1082_ALS_CONTR[0x%02X]           = [%02X]\n", LTRX1082_ALS_CONTR,i2c_read_reg(client, LTRX1082_ALS_CONTR));
    LTR_DEBUG("LTRX1082_PS_CONTR[0x%02X]            = [%02X]\n", LTRX1082_PS_CONTR,i2c_read_reg(client, LTRX1082_PS_CONTR));
    LTR_DEBUG("LTRX1082_PS_LED[0x%02X]              = [%02X]\n", LTRX1082_PS_LED,i2c_read_reg(client, LTRX1082_PS_LED));
    LTR_DEBUG("LTRX1082_PS_N_PULSES[0x%02X]         = [%02X]\n", LTRX1082_PS_N_PULSES,i2c_read_reg(client, LTRX1082_PS_N_PULSES));
    LTR_DEBUG("LTRX1082_PS_MEAS_RATE[0x%02X]        = [%02X]\n", LTRX1082_PS_MEAS_RATE,i2c_read_reg(client, LTRX1082_PS_MEAS_RATE));
    LTR_DEBUG("LTRX1082_ALS_MEAS_RATE[0x%02X]       = [%02X]\n", LTRX1082_ALS_MEAS_RATE,i2c_read_reg(client, LTRX1082_ALS_MEAS_RATE));
    LTR_DEBUG("LTRX1082_PART_ID[0x%02X]             = [%02X]\n", LTRX1082_PART_ID,i2c_read_reg(client, LTRX1082_PART_ID));
    LTR_DEBUG("LTRX1082_MANUFACTURER_ID[0x%02X]     = [%02X]\n", LTRX1082_MANUFACTURER_ID,i2c_read_reg(client, LTRX1082_MANUFACTURER_ID));
    LTR_DEBUG("LTRX1082_ALS_DATA_0[0x%02X]          = [%02X]\n", LTRX1082_ALS_DATA_0,i2c_read_reg(client, LTRX1082_ALS_DATA_0));
    LTR_DEBUG("LTRX1082_ALS_DATA_1[0x%02X]          = [%02X]\n", LTRX1082_ALS_DATA_1,i2c_read_reg(client, LTRX1082_ALS_DATA_1));
    LTR_DEBUG("LTRX1082_ALS_PS_STATUS[0x%02X]       = [%02X]\n", LTRX1082_ALS_PS_STATUS,i2c_read_reg(client, LTRX1082_ALS_PS_STATUS));
    LTR_DEBUG("LTRX1082_PS_DATA_0[0x%02X]           = [%02X]\n", LTRX1082_PS_DATA_0,i2c_read_reg(client, LTRX1082_PS_DATA_0));
    LTR_DEBUG("LTRX1082_PS_DATA_1[0x%02X]           = [%02X]\n", LTRX1082_PS_DATA_1,i2c_read_reg(client, LTRX1082_PS_DATA_1));
    LTR_DEBUG("LTRX1082_ALS_DATA_CH1_0[0x%02X]      = [%02X]\n", LTRX1082_ALS_DATA_CH1_0,i2c_read_reg(client, LTRX1082_ALS_DATA_CH1_0));
    LTR_DEBUG("LTRX1082_ALS_DATA_CH1_1[0x%02X]      = [%02X]\n", LTRX1082_ALS_DATA_CH1_1,i2c_read_reg(client, LTRX1082_ALS_DATA_CH1_1));
    LTR_DEBUG("LTRX1082_ALS_DATA_CH1_2[0x%02X]      = [%02X]\n", LTRX1082_ALS_DATA_CH1_2,i2c_read_reg(client, LTRX1082_ALS_DATA_CH1_2));
    LTR_DEBUG("LTRX1082_ALS_DATA_CH2_0[0x%02X]      = [%02X]\n", LTRX1082_ALS_DATA_CH2_0,i2c_read_reg(client, LTRX1082_ALS_DATA_CH2_0));
    LTR_DEBUG("LTRX1082_ALS_DATA_CH2_1[0x%02X]      = [%02X]\n", LTRX1082_ALS_DATA_CH2_1,i2c_read_reg(client, LTRX1082_ALS_DATA_CH2_1));
    LTR_DEBUG("LTRX1082_ALS_DATA_CH2_2[0x%02X]      = [%02X]\n", LTRX1082_ALS_DATA_CH2_2,i2c_read_reg(client, LTRX1082_ALS_DATA_CH2_2));
    LTR_DEBUG("LTRX1082_ALS_COEFF1_DATA_0[0x%02X]   = [%02X]\n", LTRX1082_ALS_COEFF1_DATA_0,i2c_read_reg(client, LTRX1082_ALS_COEFF1_DATA_0));
    LTR_DEBUG("LTRX1082_ALS_COEFF1_DATA_1[0x%02X]   = [%02X]\n", LTRX1082_ALS_COEFF1_DATA_1,i2c_read_reg(client, LTRX1082_ALS_COEFF1_DATA_1));
    LTR_DEBUG("LTRX1082_ALS_COEFF2_DATA_0[0x%02X]   = [%02X]\n", LTRX1082_ALS_COEFF2_DATA_0,i2c_read_reg(client, LTRX1082_ALS_COEFF2_DATA_0));
    LTR_DEBUG("LTRX1082_ALS_COEFF2_DATA_1[0x%02X]   = [%02X]\n", LTRX1082_ALS_COEFF2_DATA_1,i2c_read_reg(client, LTRX1082_ALS_COEFF2_DATA_1));
    LTR_DEBUG("LTRX1082_ALS_IRF_CUT_OFF[0x%02X]     = [%02X]\n", LTRX1082_ALS_IRF_CUT_OFF,i2c_read_reg(client, LTRX1082_ALS_IRF_CUT_OFF));
    LTR_DEBUG("LTRX1082_INTERRUPT[0x%02X]           = [%02X]\n", LTRX1082_INTERRUPT,i2c_read_reg(client, LTRX1082_INTERRUPT));
    LTR_DEBUG("LTRX1082_PS_THRES_UP_0[0x%02X]       = [%02X]\n", LTRX1082_PS_THRES_UP_0,i2c_read_reg(client, LTRX1082_PS_THRES_UP_0));
    LTR_DEBUG("LTRX1082_PS_THRES_UP_1[0x%02X]       = [%02X]\n", LTRX1082_PS_THRES_UP_1,i2c_read_reg(client, LTRX1082_PS_THRES_UP_1));
    LTR_DEBUG("LTRX1082_PS_THRES_LOW_0[0x%02X]      = [%02X]\n", LTRX1082_PS_THRES_LOW_0,i2c_read_reg(client, LTRX1082_PS_THRES_LOW_0));
    LTR_DEBUG("LTRX1082_PS_THRES_LOW_1[0x%02X]      = [%02X]\n", LTRX1082_PS_THRES_LOW_1,i2c_read_reg(client, LTRX1082_PS_THRES_LOW_1));
    LTR_DEBUG("LTRX1082_ALS_THRES_UP_0[0x%02X]      = [%02X]\n", LTRX1082_ALS_THRES_UP_0,i2c_read_reg(client, LTRX1082_ALS_THRES_UP_0));
    LTR_DEBUG("LTRX1082_ALS_THRES_UP_1[0x%02X]      = [%02X]\n", LTRX1082_ALS_THRES_UP_1,i2c_read_reg(client, LTRX1082_ALS_THRES_UP_1));
    LTR_DEBUG("LTRX1082_ALS_THRES_LOW_0[0x%02X]     = [%02X]\n", LTRX1082_ALS_THRES_LOW_0,i2c_read_reg(client, LTRX1082_ALS_THRES_LOW_0));
    LTR_DEBUG("LTRX1082_ALS_THRES_LOW_1[0x%02X]     = [%02X]\n", LTRX1082_ALS_THRES_LOW_1,i2c_read_reg(client, LTRX1082_ALS_THRES_LOW_1));
    LTR_DEBUG("LTRX1082_INTERRUPT_PERSIST[0x%02X]   = [%02X]\n", LTRX1082_INTERRUPT_PERSIST,i2c_read_reg(client, LTRX1082_INTERRUPT_PERSIST));

}


uint8_t ltrx1082_dev_init(struct i2c_client *client)
{
    i2c_write_reg(client, LTRX1082_PS_N_PULSES, 0x7F);
    i2c_write_reg(client, LTRX1082_PS_LED, 0x6B);

    /*for interrup work mode support */
#ifdef PS_INTERRUPT_MODE
    i2c_write_reg(client, LTRX1082_INTERRUPT, 0x01);
    i2c_write_reg(client, LTRX1082_INTERRUPT_PERSIST, 0x10);
    ltrx1082_ps_set_threshold(client, PS_THRES_UP, PS_THRES_LOW);
#endif

    i2c_write_reg(client, LTRX1082_ALS_CONTR, 0x02);
    i2c_write_reg(client, LTRX1082_PS_CONTR , 0x0E);
#if LTRDBG
    dump_regs(client);
#endif

    return LTRX1082_SUCCESS;
}

uint8_t ltrx1082_ps_enable(struct i2c_client *client, uint8_t enable)
{
    uint8_t regdata = 0;

    regdata = i2c_read_reg(client, LTRX1082_PS_CONTR);
    if (enable != 0)
    {
        regdata |= 0x02;
    }
    else
    {
        regdata &= 0xfd;
    }

    i2c_write_reg(client, LTRX1082_PS_CONTR, regdata);

    return LTRX1082_SUCCESS;
}

uint8_t ltrx1082_als_enable(struct i2c_client *client, uint8_t enable)
{
    uint8_t regdata = 0;

    regdata = i2c_read_reg(client, LTRX1082_ALS_CONTR);
    if (enable != 0)
    {
        regdata |= 0x02;
    }
    else
    {
        regdata &= 0xfd;
    }

    i2c_write_reg(client, LTRX1082_ALS_CONTR, regdata);

    return LTRX1082_SUCCESS;
}

uint16_t ltrx1082_ps_read(struct i2c_client *client)
{
    uint8_t psval_lo, psval_hi;
    uint16_t psdata;

    psval_lo = i2c_read_reg(client, LTRX1082_PS_DATA_0);
    psval_hi = i2c_read_reg(client, LTRX1082_PS_DATA_1);

    psdata = ((psval_hi & 0x07) * 256) + psval_lo;
    LTR_DEBUG("%s: psdata: %d\n",__func__, psdata);
    return psdata;
}

uint16_t ltrx1082_als_read(struct i2c_client *client)
{
    uint8_t alsval_lo, alsval_hi;
    uint16_t alsdata;

    alsval_lo = i2c_read_reg(client, LTRX1082_ALS_DATA_0);
    alsval_hi = i2c_read_reg(client, LTRX1082_ALS_DATA_1);

    alsdata = (alsval_hi << 8) + alsval_lo;
    LTR_DEBUG("%s: alsdata: %d\n",__func__, alsdata);
    return alsdata;
}

uint8_t ltrx1082_ps_set_threshold(struct i2c_client *client, uint16_t high, uint16_t low)
{
    i2c_write_reg(client, LTRX1082_PS_THRES_UP_0, high & 0x00FF);
    i2c_write_reg(client, LTRX1082_PS_THRES_UP_1, (high >> 8) & 0x00FF);
    i2c_write_reg(client, LTRX1082_PS_THRES_LOW_0, low & 0x00FF);
    i2c_write_reg(client, LTRX1082_PS_THRES_LOW_1, (low >> 8) & 0x00FF);

    return LTRX1082_SUCCESS;
}


//----
static int ltrx1082_read_raw(struct iio_dev *indio_dev,
                             struct iio_chan_spec const *chan,
                             int *val, int *val2, long mask)
{
    struct LTRX1082_data *data = iio_priv(indio_dev);
    int ret, i;
    switch (mask)
    {
    case IIO_CHAN_INFO_PROCESSED:
        if (iio_buffer_enabled(indio_dev))
            return -EBUSY;

        switch (chan->type)
        {
        case IIO_LIGHT:
            *val = ltrx1082_als_read(data->client);

            return IIO_VAL_INT;
        default:
            return -EINVAL;
        }
    case IIO_CHAN_INFO_RAW:
        if (iio_buffer_enabled(indio_dev))
            return -EBUSY;

        switch (chan->type)
        {
        case IIO_LIGHT:
            *val = ltrx1082_als_read(data->client);
            return IIO_VAL_INT;
        case IIO_PROXIMITY:
            ret = ltrx1082_ps_read(data->client);
            if (ret < 0)
                return ret;
            *val = ret & LTRX1082_PS_DATA_MASK;
            return IIO_VAL_INT;
        default:
            return -EINVAL;
        }
    case IIO_CHAN_INFO_SCALE:
        switch (chan->type)
        {
        case IIO_LIGHT:
            i = (data->als_contr & data->chip_info->als_gain_mask)
                >> data->chip_info->als_gain_shift;
            *val = 0; //data->chip_info->als_gain[i].scale;
            *val2 = 0; //data->chip_info->als_gain[i].uscale;
            return IIO_VAL_INT_PLUS_MICRO;
        case IIO_PROXIMITY:
            i = (data->ps_contr & LTRX1082_CONTR_PS_GAIN_MASK) >>
                LTRX1082_CONTR_PS_GAIN_SHIFT;
            *val = 0; //data->chip_info->ps_gain[i].scale;
            *val2 = 0; //data->chip_info->ps_gain[i].uscale;
            return IIO_VAL_INT_PLUS_MICRO;
        default:
            return -EINVAL;
        }
    case IIO_CHAN_INFO_INT_TIME:
        switch (chan->type)
        {
        case IIO_LIGHT:
            return 0; //ltrx1082_read_it_time(data, val, val2);
        default:
            return -EINVAL;
        }
    case IIO_CHAN_INFO_SAMP_FREQ:
        switch (chan->type)
        {
        case IIO_LIGHT:
            return 0; //ltrx1082_als_read_samp_freq(data, val, val2);
        case IIO_PROXIMITY:
            return 0; //ltrx1082_ps_read_samp_freq(data, val, val2);
        default:
            return -EINVAL;
        }
    }
    return -EINVAL;
}


static int ltrx1082_write_raw(struct iio_dev *indio_dev,
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

/*
static IIO_CONST_ATTR_INT_TIME_AVAIL("0.05 0.1 0.2 0.4");
static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("20 10 5 2 1 0.5");

static IIO_DEVICE_ATTR(in_proximity_scale_available, S_IRUGO,
                       NULL, NULL, 0);

static IIO_DEVICE_ATTR(in_intensity_scale_available, S_IRUGO,
                       NULL, NULL, 0);

static struct attribute *ltrx1082_attributes[] = {
    &iio_dev_attr_in_proximity_scale_available.dev_attr.attr,
    &iio_dev_attr_in_intensity_scale_available.dev_attr.attr,
    //&iio_const_attr_integration_time_available.dev_attr.attr,
    //&iio_const_attr_sampling_frequency_available.dev_attr.attr,
    NULL
};

static const struct attribute_group ltrx1082_attribute_group = {
    .attrs = ltrx1082_attributes,
};
*/
static const struct iio_info ltrx1082_info_no_irq =
{
    .read_raw = ltrx1082_read_raw,
    .write_raw = ltrx1082_write_raw,
#if 0
    .attrs = &ltrx1082_attribute_group,
#endif
    .driver_module = THIS_MODULE,
};

static const struct iio_info ltrx1082_info =
{
    .read_raw = ltrx1082_read_raw,
    .write_raw = ltrx1082_write_raw,
#if 0
    .attrs = &ltrx1082_attribute_group,
    .read_event_value	= &ltrx1082_read_event,
    .write_event_value	= &ltrx1082_write_event,
    .read_event_config	= &ltrx1082_read_event_config,
    .write_event_config	= &ltrx1082_write_event_config,
#endif
    .driver_module = THIS_MODULE,
};



static int ltrx1082_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    struct LTRX1082_data *data;
    struct iio_dev *indio_dev;
    struct i2c_adapter *adapter;
    printk("--> %s:%d\n",__func__,__LINE__);
    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;


    adapter = to_i2c_adapter(client->dev.parent);

    /* Return 1 if adapter supports everything we need, 0 if not. */
    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE | I2C_FUNC_SMBUS_READ_BYTE_DATA))
    {
        printk(KERN_INFO "%s: LTRX1082-ADV functionality check failed.\n", __func__);
        ret = -EIO;
        return ret;
    }

    data = iio_priv(indio_dev);
    i2c_set_clientdata(client, indio_dev);
    data->client = client;

    indio_dev->dev.parent = &client->dev;
    indio_dev->info = &ltrx1082_info;
    indio_dev->name = LTRX1082_DEVICE_NAME;
    indio_dev->channels = ltrx1082_channels;
    indio_dev->num_channels = ARRAY_SIZE(ltrx1082_channels);
    indio_dev->modes = INDIO_DIRECT_MODE;

    ret = ltrx1082_dev_init(client);
    if (ret)
    {
        printk(KERN_INFO "%s: LTRX1082-ADV device init failed.\n", __func__);
        goto err_exit;
    }


    ret = devm_iio_device_register(&client->dev, indio_dev);
    if(ret < 0)
        goto err_exit;
#if 0
    ret = sysfs_create_group(&indio_dev->dev.kobj, &attribute_group);
    if(ret < 0)
        goto err_exit;
#endif
    return ret;

err_exit:
    return ret;
}

static int ltrx1082_remove(struct i2c_client *client)
{
    struct iio_dev *indio_dev = i2c_get_clientdata(client);

    //ltrx1082_als_reset(client);
    iio_device_unregister(indio_dev);
    return 0;
}

static const struct acpi_device_id ltr_acpi_match[] =
{
    {"LTERX1082", ltrx1082},
    { },
};

MODULE_DEVICE_TABLE(acpi, ltr_acpi_match);

static const struct i2c_device_id ltrx1082_id[] =
{
    { LTRX1082_DEVICE_NAME, 0 },
    {}
};

MODULE_DEVICE_TABLE(i2c, ltrx1082_id);

static const struct of_device_id ltrx1082_match_table[] =
{
    { .compatible = "lite_on,ltrx1082",},
    { },
};

MODULE_DEVICE_TABLE(of, ltr_match_table);

static struct i2c_driver ltrx1082_driver =
{
    .driver 	= {
        .owner = THIS_MODULE,
        .name  = LTRX1082_DEVICE_NAME,
        .of_match_table = of_match_ptr(ltrx1082_match_table),
    },
    .probe		= ltrx1082_probe,
    .remove	= ltrx1082_remove,
    .id_table	= ltrx1082_id,

};

module_i2c_driver(ltrx1082_driver);

MODULE_AUTHOR("Advantech 2020");
MODULE_DESCRIPTION("LTRX1082-ADV Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
