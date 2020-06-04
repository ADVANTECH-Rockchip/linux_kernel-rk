#ifndef __LTRX1082_H__
#define __LTRX1082_H__

#define DRIVER_VERSION 			"1.0"
#define LTRX1082_DEVICE_NAME 		"LTRX1082_ADV"

#define LTRDBG 1
#if LTRDBG
#define LTR_DEBUG(format, ...)	\
printk(KERN_INFO LTRX1082_DEVICE_NAME ": " format , ## __VA_ARGS__)
#else
#define LTR_DEBUG(format, ...)
#endif


enum
{
    ltrx1082 = 0,
};

struct ltrx1082_chip_info
{
    u8 partid;
    struct ltrx1082_gain *als_gain;
    int als_gain_tbl_size;
    struct ltrx1082_gain *ps_gain;
    int ps_gain_tbl_size;
    u8 als_mode_active;
    u8 als_gain_mask;
    u8 als_gain_shift;
    struct iio_chan_spec const *channels;
    const int no_channels;
    const struct iio_info *info;
    const struct iio_info *info_no_irq;
};

struct LTRX1082_data
{
    struct i2c_client *client;
    struct ltrx1082_chip_info *chip_info;
    u8 als_contr, ps_contr;
    int als_period, ps_period; /* period in micro seconds */
    struct delayed_work work;
    struct input_dev *input_dev;
    struct class ltr_cls;
    atomic_t delay;
    int     als_enabled;

};





/*LTRX1082 als/ps sensor register*/
#define LTRX1082_ALS_CONTR          0x80
#define LTRX1082_PS_CONTR           0x81
#define LTRX1082_PS_LED             0x82
#define LTRX1082_PS_N_PULSES        0x83
#define LTRX1082_PS_MEAS_RATE       0x84
#define LTRX1082_ALS_MEAS_RATE      0x85
#define LTRX1082_PART_ID            0x86
#define LTRX1082_MANUFACTURER_ID    0x87
#define LTRX1082_ALS_DATA_0         0x88
#define LTRX1082_ALS_DATA_1         0x89
#define LTRX1082_ALS_PS_STATUS      0x8A
#define LTRX1082_PS_DATA_0          0x8B
#define LTRX1082_PS_DATA_1          0x8C
#define LTRX1082_ALS_DATA_CH1_0     0x8D
#define LTRX1082_ALS_DATA_CH1_1     0x8E
#define LTRX1082_ALS_DATA_CH1_2     0x8F
#define LTRX1082_ALS_DATA_CH2_0     0x90
#define LTRX1082_ALS_DATA_CH2_1     0x91
#define LTRX1082_ALS_DATA_CH2_2     0x92
#define LTRX1082_ALS_COEFF1_DATA_0  0x93
#define LTRX1082_ALS_COEFF1_DATA_1  0x94
#define LTRX1082_ALS_COEFF2_DATA_0  0x95
#define LTRX1082_ALS_COEFF2_DATA_1  0x96
#define LTRX1082_ALS_IRF_CUT_OFF    0x97
#define LTRX1082_INTERRUPT          0x98
#define LTRX1082_PS_THRES_UP_0      0x99
#define LTRX1082_PS_THRES_UP_1      0x9A
#define LTRX1082_PS_THRES_LOW_0     0x9B
#define LTRX1082_PS_THRES_LOW_1     0x9C
#define LTRX1082_ALS_THRES_UP_0     0x9E
#define LTRX1082_ALS_THRES_UP_1     0x9F
#define LTRX1082_ALS_THRES_LOW_0    0xA0
#define LTRX1082_ALS_THRES_LOW_1    0xA1
#define LTRX1082_INTERRUPT_PERSIST  0xA4
/* LTR-X1082 Registers */

//#define PS_INTERRUPT_MODE  //we didn't connect interrupt
#define PS_THRES_UP					0x0400
#define PS_THRES_LOW				0x0200

#define LTRX1082_SUCCESS			0
#define LTRX1082_ERROR				0xFF

#define ALS_DATA					0
#define PS_DATA						1
//{
#define LTRX1082_PS_DATA_MASK 0x7ff
#define LTRX1082_PS_THRESH_MASK 0x7ff
#define LTRX1082_ALS_THRESH_MASK 0xffff

#define LTRX1082_ALS_DEF_PERIOD 500000
#define LTRX1082_PS_DEF_PERIOD 100000
#define LTRX1082_CONTR_PS_GAIN_MASK 0x00
#define LTRX1082_CONTR_PS_GAIN_SHIFT 0x00

//}
#if 1
//TODO: revise the function prototype after function ready
uint8_t ltrx1082_dev_init(struct i2c_client *client);
uint8_t ltrx1082_ps_enable(struct i2c_client *client, uint8_t enable);
uint8_t ltrx1082_als_enable(struct i2c_client *client, uint8_t enable);
uint16_t ltrx1082_ps_read(struct i2c_client *client);
uint16_t ltrx1082_als_read(struct i2c_client *client);
uint8_t ltrx1082_ps_set_threshold(struct i2c_client *client, uint16_t high, uint16_t low);
#endif

#endif
