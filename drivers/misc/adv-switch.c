/*
 * Driver for adv switch control.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/switch.h>

#define ADV_SWITCH_MODNAME		"misc-adv-switch"

int g_duty = 200000;
int g_period = 400000;

struct misc_adv_switch {
	unsigned int gpio;
	unsigned int irq;
	struct pwm_device *pwm;
	unsigned int duty;
	unsigned int period;
	bool enable;
	struct switch_dev	switch_state;
} adv_switch;

/*struct adv_switch *ps = &adv_switch;*/
static ssize_t switch_show_duty(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n",adv_switch.duty);
}
static ssize_t switch_store_duty(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	adv_switch.duty = simple_strtoul(buf,NULL,0);
	printk("adv_switch.duty=%d\n",adv_switch.duty);
	pwm_config(adv_switch.pwm,adv_switch.duty,adv_switch.period);
	return count;
}

static ssize_t switch_show_period(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n",adv_switch.period);
}

static ssize_t switch_store_period(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	adv_switch.period = simple_strtoul(buf,NULL,0);
	printk("adv_switch.period=%d\n",adv_switch.period);
	pwm_config(adv_switch.pwm,adv_switch.duty,adv_switch.period);
	return count;
}

static ssize_t switch_show_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n",adv_switch.enable);
}
static ssize_t switch_store_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	adv_switch.enable = simple_strtoul(buf,NULL,0);
	if (adv_switch.enable == true )
		pwm_enable(adv_switch.pwm);
	else
		pwm_disable(adv_switch.pwm);
	return count;
}

static DEVICE_ATTR(duty, 0664, switch_show_duty, switch_store_duty);
static DEVICE_ATTR(period,0664, switch_show_period, switch_store_period);
static DEVICE_ATTR(enable,0664, switch_show_enable, switch_store_enable);

static irqreturn_t adv_switch_handle(int irq, void *dev_id)
{
	static bool state = true;
	 if(state)
	 {
		switch_set_state(&adv_switch.switch_state, 1);
		/*pwm_enable(adv_switch.pwm);*/ //userspace to control
		state = false;
	 }
	 else
	 {
		switch_set_state(&adv_switch.switch_state, 0);
		/*pwm_disable(adv_switch.pwm);*/ //userspace to control
		state = true;
	 }
	 return IRQ_HANDLED;
}

static int adv_switch_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
	device_remove_file(dev, &dev_attr_duty);
	device_remove_file(dev, &dev_attr_period);
	device_remove_file(dev, &dev_attr_enable);
	return 0;
}

static int adv_switch_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct device_node *np;
	int ret;
    np = dev->of_node;
	adv_switch.gpio = of_get_named_gpio_flags(np, "switch_gpio", 0, NULL);
	if (gpio_is_valid(adv_switch.gpio))
	{
			gpio_request_one(adv_switch.gpio, 
						GPIOF_DIR_IN, "adv switch");
	}
	else
	{
		printk("adv switch request gpio error\n");
		return -EFAULT;
	}
	adv_switch.irq = irq_of_parse_and_map(np,0);

	ret = request_irq(adv_switch.irq,adv_switch_handle,IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING ,"adv_siwtch",&adv_switch);
	if (ret)
	{
		printk("adv switch request interrupt error\n");
		goto ERR1;
	}

	adv_switch.switch_state.name = kzalloc(32, GFP_KERNEL);
	memset((char *)adv_switch.switch_state.name, 0, 32);
	sprintf((char *)adv_switch.switch_state.name, "switch%d", 0);

	ret = switch_dev_register(&adv_switch.switch_state);
	if(ret)
	{
		printk("adv switch request state error\n");
		goto ERR2;
	}

	adv_switch.duty = g_duty;
	adv_switch.period = g_period;
	adv_switch.enable = false;
	adv_switch.pwm = of_pwm_get(np, 0);
	if(!adv_switch.pwm)
	{
		printk("adv switch get switch adv_switch.pwm error\n");
		ret = -EPERM;
		goto ERR;
	}

	ret = pwm_config(adv_switch.pwm, adv_switch.duty, adv_switch.period);
	if(ret)
	{
		printk("adv switch config adv_switch.pwm error\n");
		ret = -EPERM;
		goto ERR;
	}

	ret = device_create_file(dev, &dev_attr_duty);
	if(ret)
	{
		printk("adv switch create duty error\n");
		goto ERR;
	}
	
	ret = device_create_file(dev, &dev_attr_period);
	if(ret)
	{
		printk("adv switch create period error\n");
		goto ERR;
	}

	ret = device_create_file(dev, &dev_attr_enable);	
	if(ret)
	{
		printk("adv switch create switch enable error\n");
		goto ERR;
	}
	printk("adv switch probe ok\n");
	return ret;
ERR:
	switch_dev_unregister(&adv_switch.switch_state);
ERR2:
	free_irq(adv_switch.irq, &adv_switch);
ERR1:
	gpio_free(adv_switch.gpio);
	return ret;
}

static const struct of_device_id adv_switch_of_match[] = {
	{ .compatible = ADV_SWITCH_MODNAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, adv_switch_of_match);

static struct platform_driver adv_switch_driver = {
	.probe		= adv_switch_probe,
	.remove		= adv_switch_remove,
	.driver		= {
		.name	= ADV_SWITCH_MODNAME,
		.of_match_table	=  of_match_ptr(adv_switch_of_match),
	}
};

static int __init adv_switch_init(void)
{
	return platform_driver_register(&adv_switch_driver);
}

static void __exit adv_switch_exit(void)
{
	pwm_free(adv_switch.pwm);
	gpio_free(adv_switch.gpio);
	free_irq(adv_switch.irq, &adv_switch);
	switch_dev_unregister(&adv_switch.switch_state);
	platform_driver_unregister(&adv_switch_driver);
}

module_init(adv_switch_init);
module_exit(adv_switch_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("adv switch driver for advantech");
