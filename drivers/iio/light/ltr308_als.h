#ifndef __LTR308_H__
#define __LTR308_H__

#define DRIVER_VERSION 			"1.0"
#define LTR308_DEVICE_NAME 		"LTR308_ALS"

/* Power On response time in ms */
#define PON_DELAY				600
#define WAKEUP_DELAY			10

#define LTR308DBG 1
#if LTR308DBG
#define LTR308_DEBUG(format, ...)	\
printk(KERN_INFO "LTR308 " format , ## __VA_ARGS__)
#else
#define LTR308_DEBUG(format, ...)
#endif

enum
{
    ltr308 = 0,
};

struct LTR308_data
{
    struct i2c_client *client;
    struct delayed_work work;
    struct input_dev *input_dev;
    struct class ltr_cls;
    atomic_t delay;
    int     als_enabled;

};


/* LTR308 Registers */

#define LTR308_MAIN_CTRL			0x00
#define LTR308_ALS_MEAS_RATE		0x04
#define LTR308_ALS_GAIN				0x05
#define LTR308_PART_ID				0x06
#define LTR308_MAIN_STATUS			0x07
#define LTR308_ALS_DATA_0			0x0D
#define LTR308_ALS_DATA_1			0x0E
#define LTR308_ALS_DATA_2			0x0F
#define LTR308_INT_CFG				0x19
#define LTR308_INT_PST 				0x1A
#define LTR308_ALS_THRES_UP_0		0x21
#define LTR308_ALS_THRES_UP_1		0x22
#define LTR308_ALS_THRES_UP_2		0x23
#define LTR308_ALS_THRES_LOW_0		0x24
#define LTR308_ALS_THRES_LOW_1		0x25
#define LTR308_ALS_THRES_LOW_2		0x26

typedef union
{
    unsigned char v;
    struct
    {
        unsigned :1;
        unsigned als_enable:1;
        unsigned :2;
        unsigned sw_reset:1;
        unsigned :3;
    };
} MAIN_CTRL;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned meas_rate:3;
        unsigned :1;
        unsigned als_res:3;
        unsigned :1;
    };
} MEAS_RATE;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned gain_range:3;
        unsigned :5;
    };
} GAIN;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned rev:4;
        unsigned part:4;
    };
} PART_ID;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned :3;
        unsigned data:1;
        unsigned irq:1;
        unsigned pon:1;
        unsigned :2;
    };
} STATUS;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned data:8;
    };
} DATA_0;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned data:8;
    };
} DATA_1;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned data:4;
        unsigned :4;
    };
} DATA_2;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned :2;
        unsigned int_en:1;
        unsigned :1;
        unsigned int_sel:2;
        unsigned :1;
    };
} INT_CFG;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned :4;
        unsigned persist:4;
    };
} INT_PST;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned data:8;
    };
} THRES_UP_0;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned data:8;
    };
} THRES_UP_1;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned data:4;
        unsigned :4;
    };
} THRES_UP_2;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned data:8;
    };
} THRES_LOW_0;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned data:8;
    };
} THRES_LOW_1;

typedef union
{
    unsigned char v;
    struct
    {
        unsigned data:4;
        unsigned :4;
    };
} THRES_LOW_2;

#endif
