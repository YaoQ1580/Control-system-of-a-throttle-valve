#include <rtthread.h>
#include <rtdevice.h>

#define ADC_DEV_NAME        "adc1"      /* ADC ???? */
#define ADC_DEV_CHANNEL     15           /* ADC ?? */
#define REFER_VOLTAGE       330         /* ???? 3.3V,??????100??2???*/
#define CONVERT_BITS        (1 << 12)   /* ?????12? */

static int adc_vol_sample(int argc, char *argv[])
{
    rt_adc_device_t adc_dev;
    rt_uint32_t value, vol;
    rt_err_t ret = RT_EOK;

    /* ???? */
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        rt_kprintf("adc sample run failed! can't find %s device!\n", ADC_DEV_NAME);
        return RT_ERROR;
    }

    /* ???? */
    ret = rt_adc_enable(adc_dev, ADC_DEV_CHANNEL);

    /* ????? */
    value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL);
    rt_kprintf("the value is :%d \n", value);

    /* ???????? */
    vol = value * REFER_VOLTAGE / CONVERT_BITS;
    rt_kprintf("the voltage is :%d.%02d \n", vol / 100, vol % 100);

    /* ???? */
    ret = rt_adc_disable(adc_dev, ADC_DEV_CHANNEL);

    return ret;
}
/* ??? msh ????? */
MSH_CMD_EXPORT(adc_vol_sample, adc voltage convert sample);
