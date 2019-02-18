
#include "sensor_st_hts221.h"
#include "string.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_SECTION_NAME  "sensor.st.hts221"
#define DBG_COLOR
#include <rtdbg.h>

/***********  Common  *****************/

static HTS221_Object_t hts221;
static struct rt_i2c_bus_device *i2c_bus_dev;

static int rt_i2c_write_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static int rt_i2c_read_reg(uint16_t addr, uint16_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(i2c_bus_dev, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t _hts221_init(struct rt_sensor_intf *intf)
{
    HTS221_IO_t io_ctx;
    rt_uint8_t        id;

    i2c_bus_dev = (struct rt_i2c_bus_device *)rt_device_find(intf->dev_name);
    if (i2c_bus_dev == RT_NULL)
    {
        return -RT_ERROR;
    }

    /* Configure the humilero driver */
    io_ctx.BusType     = HTS221_I2C_BUS; /* I2C */
    io_ctx.Address     = (rt_uint32_t)(intf->user_data) & 0xff;
    io_ctx.Init        = RT_NULL;
    io_ctx.DeInit      = RT_NULL;
    io_ctx.ReadReg     = rt_i2c_read_reg;
    io_ctx.WriteReg    = rt_i2c_write_reg;
    io_ctx.GetTick     = RT_NULL;

    if (HTS221_RegisterBusIO(&hts221, &io_ctx) != HTS221_OK)
    {
        return -RT_ERROR;
    }
    else if (HTS221_ReadID(&hts221, &id) != HTS221_OK)
    {
        LOG_D("read id failed");
        return -RT_ERROR;
    }
    if (HTS221_Init(&hts221) != HTS221_OK)
    {
        LOG_D("hts221 init failed");
        return -RT_ERROR;
    }
    return RT_EOK;
}

/***********  Humi  *****************/

static rt_err_t _hts221_humi_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    if (power == RT_SENSOR_POWER_DOWN)
    {
        LOG_D("set power down");
        HTS221_HUM_Disable(&hts221);
    }
    else if (power == RT_SENSOR_POWER_NORMAL)
    {
        LOG_D("set power normal");
        HTS221_HUM_Enable(&hts221);
    }
    else
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}

/***********  Temp  *****************/

static rt_err_t _hts221_temp_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    if (power == RT_SENSOR_POWER_DOWN)
    {
        HTS221_TEMP_Disable(&hts221);
    }
    else if (power == RT_SENSOR_POWER_NORMAL)
    {
        HTS221_TEMP_Enable(&hts221);
    }
    else
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}

static rt_size_t hts221_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    struct rt_sensor_data *data = (struct rt_sensor_data *)buf;
    
    if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
    {
        float temp_value;

        HTS221_TEMP_GetTemperature(&hts221, &temp_value);

        data->type = RT_SENSOR_CLASS_TEMP;
        data->data.temp = temp_value * 10;
        data->timestamp = rt_sensor_get_ts();
    }
    else if (sensor->info.type == RT_SENSOR_CLASS_HUMI)
    {
        float humi_value;

        HTS221_HUM_GetHumidity(&hts221, &humi_value);

        data->type = RT_SENSOR_CLASS_HUMI;
        data->data.humi = humi_value * 10;
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
}

static rt_err_t hts221_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        HTS221_ReadID(&hts221, args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        {
            rt_uint16_t odr = (rt_uint32_t)args & 0xffff;
            if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
                HTS221_TEMP_SetOutputDataRate(&hts221, odr);
            else if(sensor->info.type == RT_SENSOR_CLASS_HUMI)
                HTS221_HUM_SetOutputDataRate(&hts221, odr);
        }
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        if (sensor->info.type == RT_SENSOR_CLASS_TEMP)
            result = _hts221_temp_set_power(sensor, (rt_uint32_t)args & 0xff);
        else if(sensor->info.type == RT_SENSOR_CLASS_HUMI)
            result = _hts221_humi_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        result =  -RT_EINVAL;
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    hts221_fetch_data,
    hts221_control
};

int rt_hw_hts221_temp_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor = RT_NULL;

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        return -1;

    sensor->info.type       = RT_SENSOR_CLASS_TEMP;
    sensor->info.vendor     = RT_SENSOR_VENDOR_STM;
    sensor->info.model      = "hts221_temp";
    sensor->info.unit       = RT_SENSOR_UNIT_DCELSIUS;
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor->info.range_max  = 120;
    sensor->info.range_min  = -40;
    sensor->info.period_min = 80;

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;
    
    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        rt_free(sensor);
        return -RT_ERROR;
    }

    LOG_I("temp sensor init success");
    return RT_EOK;
}

int rt_hw_hts221_humi_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor = RT_NULL;

    sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
    if (sensor == RT_NULL)
        return -1;

    sensor->info.type       = RT_SENSOR_CLASS_HUMI;
    sensor->info.vendor     = RT_SENSOR_VENDOR_STM;
    sensor->info.model      = "hts221_humi";
    sensor->info.unit       = RT_SENSOR_UNIT_PERMILLAGE;
    sensor->info.intf_type  = RT_SENSOR_INTF_I2C;
    sensor->info.range_max  = 1000;
    sensor->info.range_min  = 0;
    sensor->info.period_min = 80;

    rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
    sensor->ops = &sensor_ops;
    
    result = rt_hw_sensor_register(sensor, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
    if (result != RT_EOK)
    {
        LOG_E("device register err code: %d", result);
        rt_free(sensor);
        return -RT_ERROR;
    }

    LOG_I("humi sensor init success");
    return RT_EOK;
}

int rt_hw_hts221_init(const char *name, struct rt_sensor_config *cfg)
{
    _hts221_init(&cfg->intf);
    
#ifdef PKG_USING_HTS221_TEMP
    rt_hw_hts221_temp_init(name, cfg);
#endif
#ifdef PKG_USING_HTS221_HUMI
    rt_hw_hts221_humi_init(name, cfg);
#endif

    return 0;
}
