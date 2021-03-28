#include<stdio.h>
#include<string.h>
#include<stdlib.h>
//#include<glib.h>
#include<linux/types.h>
#include<linux/i2c.h>
#include<linux/i2c-dev.h>
#include<sys/ioctl.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<unistd.h>
#include<errno.h>
#include<inttypes.h>
#include<time.h>
//#include<sys/siginfo.h>
#include<signal.h>

#include"bme280.h"



#define I2C_DEVICE_ADDR	0x76
#define I2C_DEVICE_PATH "/dev/i2c- "
#define BUF_LEN 		100

/* variable definition */
int file;
int bme280_fd;
char read_buffer[BUF_LEN];
int errno;
struct {
	uint16_t t1;
	int16_t t2;
	int16_t t3;
	int32_t t_fine;

	uint16_t p1;
	int16_t p2;
	int16_t p3;
	int16_t p4;
	int16_t p5;
	int16_t p6;
	int16_t p7;
	int16_t p8;
	int16_t p9;

	uint8_t h1;
	int16_t h2;
	uint8_t h3;
	int16_t h4;
	int16_t h5;
	int8_t h6;
} calib_data;

double temperature;
double humidity;
double pressure;

/** Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

/* function prototypes */
static void i2c_read_reg(uint8_t *reg, uint8_t *buf, uint8_t len);
static void i2c_write_reg(uint8_t *reg, uint8_t len);

static uint8_t bme280_read_version(void);
static uint8_t bme280_read_status(void);
static uint8_t bme280_read_power_mode(void);
static void bme280_sleep_mode(void);
static void bme280_init(void);
static void bme280_normal_mode(void);
static void bme280_force_mode(void);
static void bme280_reset(void);
static void bme280_get_sensor_data(void);
static void bme280_read_calib_data(void);
static void bme280_parse_temp_press_calibration_data(uint8_t *reg_data);
static void bme280_parse_humid_calibration_data(const uint8_t *reg_data);
static void bme280_parse_sensor_data(uint8_t *reg_data);
static double bme280_calculate_temperature(uint32_t reg_data);
static double bme280_calculate_humidity(uint32_t reg_data);
static double bme280_calculate_pressure(uint32_t reg_data);

// delay = ((BME280_MEAS_OFFSET + (BME280_MEAS_DUR * temp_osr) +
//         ((BME280_MEAS_DUR * pres_osr) + BME280_PRES_HUM_MEAS_OFFSET) +
//		   ((BME280_MEAS_DUR * hum_osr) + BME280_PRES_HUM_MEAS_OFFSET)) / BME280_MEAS_SCALING_FACTOR)
// osr = 0, 2, 2, 4, 8, 16
#define DELAY_MAX (uint32_t)((BME280_MEAS_OFFSET + (BME280_MEAS_DUR * 16) + \
                  ((BME280_MEAS_DUR * 16) + BME280_PRES_HUM_MEAS_OFFSET) + \
                  ((BME280_MEAS_DUR * 16) + BME280_PRES_HUM_MEAS_OFFSET)) / BME280_MEAS_SCALING_FACTOR)

#define DELAY_MIN (uint32_t)((BME280_MEAS_OFFSET + (BME280_MEAS_DUR * 0) + \
                  ((BME280_MEAS_DUR * 0) + BME280_PRES_HUM_MEAS_OFFSET) + \
                  ((BME280_MEAS_DUR * 0) + BME280_PRES_HUM_MEAS_OFFSET)) / BME280_MEAS_SCALING_FACTOR)



/* i2c reg read/write routines */
static void i2c_read_reg(uint8_t *reg, uint8_t *buf, uint8_t len)
{
	uint8_t rx;

	write(bme280_fd, reg, 1);
	usleep(DELAY_MAX);
	read(bme280_fd, buf, len);
	usleep(DELAY_MAX);
}

static void i2c_write_reg(uint8_t *reg, uint8_t len)
{
	write(bme280_fd, reg, len);
	usleep(DELAY_MAX);
}



/* initialisation of the BME280 sensor */
static int i2c_init(char *device)
{
	printf("%s: device: %s, addr: 0x%02x\n", __FUNCTION__, device, I2C_DEVICE_ADDR);
	
	if( 0 == strcmp(device, "/dev/i2c-1")) {
		system("config-pin P9_17 i2c");
		system("config-pin P9_18 i2c");
	} else if (0 == strcmp(device, "/dev/i2c-2")) {
		system("config-pin P9_19 i2c");
		system("config-pin P9_20 i2c");
	} else {
		printf("unexpected device\n");
		return -1;
	}

	if(0 > (bme280_fd = open((const char*)device, O_RDWR))) {
		printf("%d: Failed to open the i2c bus (%s): %s\n", __LINE__, device, strerror(errno));
		return -1;
	}

	if(0 > ioctl(bme280_fd, I2C_SLAVE, I2C_DEVICE_ADDR)) {
		printf("%d: Failed to set the i2c addr (0x%02x): %s\n", __LINE__, I2C_DEVICE_ADDR, strerror(errno));
		close(bme280_fd);
		return -1;
	}
	
	return 1;
}

/*********************************************************
 * 					BME280 handlers 
 * *******************************************************/

static void bme280_init(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t version;
	uint8_t status;
	uint8_t mode;

	version = bme280_read_version();

	bme280_reset();
	usleep(10000);

	version = bme280_read_version();
	status = bme280_read_status();
	mode = bme280_read_power_mode();

	//bme280_sleep_mode();

	/* read compensation parameters */
	//bmp_read_temp_calib_data();
	//bmp_read_press_calib_data();
	//bmp_read_humid_calib_data();
	bme280_read_calib_data();

	printf("init completed (version: 0x%02x, status: 0x%02x, mode: 0x%02x)\n\n", version, status, mode);
}


/* set force mode. This triggers single measurement */
static void bme280_force_mode(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t reg[2];

	reg[0] = BME280_CTRL_HUM_REG;
	/* osrs_h */
	reg[1] = BME280_OVERSAMPLING_16X;
	i2c_write_reg(reg, 2);

	reg[0] = BME280_CTRL_MEAS_REG;
	/* osrs_t << 5 | osrs_p << 2 | mode */
	reg[1] = (BME280_OVERSAMPLING_16X << 2) | (BME280_OVERSAMPLING_16X << 5) | BME280_FORCE_MODE;
	i2c_write_reg(reg, 2);

	//usleep(100000);
}


static void bme280_normal_mode(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t reg[2];
	
	reg[0] = BME280_CTRL_HUM_REG;
	/* osrs_h */
	reg[1] = BME280_OVERSAMPLING_16X;
	i2c_write_reg(reg, 2);

	reg[0] = BME280_CONFIG_REG;
	reg[1] = (BME280_STANDBY_TIME_250_MS << 5) | (BME280_FILTER_COEFF_16 << 2) | BME280_NO_3WIRE_SPI;
	i2c_write_reg(reg, 2);

	reg[0] = BME280_CTRL_MEAS_REG;
	/* osrs_t << 5 | osrs_p << 2 | mode */
	reg[1] = (BME280_OVERSAMPLING_16X << 5) | (BME280_OVERSAMPLING_16X << 2) | BME280_NORMAL_MODE;
	i2c_write_reg(reg, 2);

	//usleep(100000);
}


static void bme280_sleep_mode(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t reg[2];

	reg[0] = BME280_CTRL_MEAS_REG;
	reg[1] = (BME280_NO_OVERSAMPLING << 5) | (BME280_NO_OVERSAMPLING << 2) | BME280_SLEEP_MODE;
	i2c_write_reg(reg, 2);

	//usleep(100000);
}


static uint8_t bme280_read_power_mode(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t reg = BME280_CTRL_MEAS_REG;
	uint8_t mode;

	i2c_read_reg(&reg, &mode, 1);

	return mode;
}


static uint8_t bme280_read_status(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t reg = BME280_STATUS_REG;
	uint8_t status;

	i2c_read_reg(&reg, &status, 1);

	return status;
}


static uint8_t bme280_read_version(void) 
{
	printf("%s\n", __FUNCTION__);
	uint8_t reg = BME280_VERSION_REG;
	uint8_t version;
	
	i2c_read_reg(&reg, &version, 1);

	return version;
}


static void bme280_reset(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t reg[2] = {BME280_RESET_REG, BME280_RESET_SET};

	i2c_write_reg(reg, 2);
}


static void bme280_get_sensor_data(void)
{
	//printf("%s\n", __FUNCTION__);
	uint8_t rx[BME280_MEASURE_P_T_H_LEN];
	uint8_t reg = BME280_MEASURE_P_T_H_REG_START;

	i2c_read_reg(&reg, rx, BME280_MEASURE_P_T_H_LEN);
	bme280_parse_sensor_data(rx);
}


static void bme280_parse_sensor_data(uint8_t *reg_data)
{
	//printf("%s\n", __FUNCTION__); 
	/* Variables to store the sensor data */
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;
	uint16_t humid_reg;
	uint32_t press_reg;
	uint32_t temp_reg;

	/* Store the parsed register values for pressure data */
	data_msb = (uint32_t)reg_data[0] << 12;
	data_lsb = (uint32_t)reg_data[1] << 4;
	data_xlsb = (uint32_t)reg_data[2] >> 4;
	press_reg = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for temperature data */
	data_msb = (uint32_t)reg_data[3] << 12;
	data_lsb = (uint32_t)reg_data[4] << 4;
	data_xlsb = (uint32_t)reg_data[5] >> 4;
	temp_reg = data_msb | data_lsb | data_xlsb;

	/* Store the parsed register values for humidity data */
	data_msb = (uint32_t)reg_data[6] << 8;
	data_lsb = (uint32_t)reg_data[7];
	humid_reg = data_msb | data_lsb;

	//printf("Sensor data register values (temp: 0x%04x, press: 0x%04x, humid: 0x%04x)\n", temp_reg, press_reg, humid_reg);

	temperature = bme280_calculate_temperature(temp_reg);
	pressure = bme280_calculate_pressure(press_reg);
	humidity = bme280_calculate_humidity(humid_reg);
}


/*********************************************************
 * 					calibration
 * *******************************************************/

static void bme280_read_calib_data(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t rx[BME280_TEMP_PRESS_CALIB_LEN];
	uint8_t reg;
	
	reg = BME280_TEMP_PRESS_CALIB_REG_START;
	i2c_read_reg(&reg, rx, BME280_TEMP_PRESS_CALIB_LEN);
	bme280_parse_temp_press_calibration_data(rx);

	reg = BME280_HUMIDITY_CALIB_REG_START;
	memset(rx, 0, BME280_HUMIDITY_CALIB_LEN);
	i2c_read_reg(&reg, rx, BME280_HUMIDITY_CALIB_LEN);
	bme280_parse_humid_calibration_data(rx);

	printf("temp:	%04x (%d), %04x (%d), %04x (%d), %04x (%d)\n", calib_data.t1, calib_data.t1, calib_data.t2, calib_data.t2,
		   calib_data.t3, calib_data.t3, calib_data.t_fine, calib_data.t_fine);
	printf("humid:	%04x (%d), %04x (%d), %04x (%d), %04x (%d), %04x (%d), %04x (%d)\n", calib_data.h1, calib_data.h1,
		   calib_data.h2, calib_data.h2, calib_data.h3, calib_data.h3, calib_data.h4, calib_data.h4, calib_data.h5, calib_data.h5, calib_data.h6, calib_data.h6);
	printf("press:	%04x (%d), %03x (%d), %04x (%d), %04x (%d), %04x (%d), %03x (%d), %04x (%d), %03x (%d), %04x (%d)\n",
		   calib_data.p1, calib_data.p1, calib_data.p2, calib_data.p2, calib_data.p3, calib_data.p3, calib_data.p4, calib_data.p4,
		   calib_data.p5, calib_data.p5, calib_data.p6, calib_data.p6, calib_data.p7, calib_data.p7, calib_data.p8, calib_data.p8, calib_data.p9, calib_data.p9);
}

static void bme280_parse_temp_press_calibration_data(uint8_t *reg_data)
{
	printf("%s\n", __FUNCTION__);
	calib_data.t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	calib_data.t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
	calib_data.t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
	calib_data.p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
	calib_data.p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
	calib_data.p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
	calib_data.p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
	calib_data.p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
	calib_data.p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
	calib_data.p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
	calib_data.p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
	calib_data.p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
	calib_data.h1 = reg_data[25];
}

static void bme280_parse_humid_calibration_data(const uint8_t *reg_data)
{
	printf("%s\n", __FUNCTION__);
	int16_t dig_h4_lsb;
	int16_t dig_h4_msb;
	int16_t dig_h5_lsb;
	int16_t dig_h5_msb;

	calib_data.h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	calib_data.h3 = reg_data[2];
	dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
	dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
	calib_data.h4 = dig_h4_msb | dig_h4_lsb;
	dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
	dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
	calib_data.h5 = dig_h5_msb | dig_h5_lsb;
	calib_data.h6 = (int8_t)reg_data[6];
}

/*********************************************************
 * 						humidity
 * *******************************************************/

static void bmp_read_humid_calib_data(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t rx[2];
	uint8_t reg;

	printf("read humidity compensation: \n");

	reg = BME280_H1_REG;
	i2c_read_reg(&reg, rx, 1);
	calib_data.h1 = rx[0];
	printf("rx: %02x,    h2=0x%04x\n", rx[0], calib_data.h1);

	reg = BME280_H2_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.h2 = (int16_t)(((uint16_t)rx[1] << 8) | ((uint16_t)rx[0]));
	printf("rx: %02x %02x, h2=0x%04x\n", rx[0], rx[1], calib_data.h2);

	reg = BME280_H3_REG;
	i2c_read_reg(&reg, rx, 1);
	calib_data.h3 = rx[0];
	printf("rx: %02x,    h3=0x%04x\n", rx[0], calib_data.h3);

	reg = BME280_H4_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.h4 = (int16_t)(((uint16_t)rx[0] << 4) | ((uint16_t)(rx[1] & 0x0F)));
	printf("rx: %02x %02x, h4=0x%04x\n", rx[0], rx[1], calib_data.h4);

	reg = BME280_H5_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.h5 = (int16_t)(((uint16_t)rx[1] << 4) | ((uint16_t)((rx[0] & 0xF0) >> 4)));
	printf("rx: %02x %02x, h5=0x%04x\n", rx[0], rx[1], calib_data.h5);

	reg = BME280_H6_REG;
	i2c_read_reg(&reg, rx, 1);
	calib_data.h6 = (int8_t)(rx[0]);
	printf("rx: %02x,    h6=0x%04x\n\n", rx[0], calib_data.h6);
}

static double bme280_calculate_humidity(uint32_t reg_data)
{
#if 1
/*
	//printf("%s\n", __FUNCTION__);
	double humid = 0;
	double humid_min = 0.0;
	double humid_max = 100.0;
	double var1;
	double var2;
	double var3;
	double var4;
	double var5;
	double var6;

	var1 = (double)calib_data.t_fine - 76800.0;
	var2 = ((double)calib_data.h4 * 64.0 + (((double)calib_data.h5) / 16384.0) * var1);
	var3 = (double)reg_data - var2;
	var4 = ((double)calib_data.h2) / 65536.0;
	var5 = (1.0 + (((double)calib_data.h3) / 67108864.0) * var1);
	var6 = 1.0 + (((double)calib_data.h6) / 67108864.0) * var1 * var5;
	var6 = var3 * var4 * var5 * var6;
	humid = (var6 * (1.0 - ((double)calib_data.h1) * var6 / 524288.0));// 1024.0;
	printf("humidity: 0x%04x \t%d\n", humid, humid);
	//printf("var1 %.2f, var2 %.2f, var3 %.2f, var4 %.2f, var5 %.2f, var6 %.2f, calib_data.t_fine %.2f, humid %.2f\n", var1, var2, var3, var4, var5, var6, calib_data.t_fine, humid);
*/
	double humid_min = 0.0;
	double humid_max = 100.0;
	double v1 = (double)calib_data.t_fine - 76800.0;
	double v2 = (double)reg_data - (((double)calib_data.h4) * 64.0 + ((double)calib_data.h5) / 16384.0 * v1);
	double v3 = (((double)calib_data.h2) / 65536.0 * (1.0 + ((double)calib_data.h6) / 67108864.0 * v1 * (1.0 + ((double)calib_data.h3) / 67108864.0 * v1)));
	double v4 = v2 * v3;
	double humid = v4 * (1.0 - ((double)calib_data.h1) * v4 / 524288.0);
	//printf("humidity: %.2f\n", humidity);

	if (humid > humid_max) 	{
		humid = humid_max;
	} else if (humid < humid_min) {
		humid = humid_min;
	}

	return humid;

#else

	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	uint32_t humid;
	uint32_t humid_max = 102400;

	var1 = calib_data.t_fine - ((int32_t)76800);
	var2 = (int32_t)(reg_data * 16384);
	var3 = (int32_t)(((int32_t)calib_data.h4) * 1048576);
	var4 = ((int32_t)calib_data.h5) * var1;
	var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
	var2 = (var1 * ((int32_t)calib_data.h6)) / 1024;
	var3 = (var1 * ((int32_t)calib_data.h3)) / 2048;
	var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
	var2 = ((var4 * ((int32_t)calib_data.h2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * ((int32_t)calib_data.h1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	humid = (uint32_t)(var5 / 4096);

	if (humid > humid_max)
	{
		humid = humid_max;
	}

	return humid;

#endif	
}


static uint16_t bmp_read_humidity(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t rx[2];
	uint8_t reg;
	uint32_t reg_data;

	reg = MBP280_HUMIDITY_REG;
	i2c_read_reg(&reg, rx, 2);

	reg_data = (((uint16_t)rx[0]) << 8) | ((uint16_t)rx[0]);

	printf("humid_reg=0x%02x\n", reg_data);

	return (humidity = bme280_calculate_humidity(reg_data));
}

/*********************************************************
 * 						pressure
 * *******************************************************/

static void bmp_read_press_calib_data(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t rx[2];
	uint8_t reg;

	printf("read pressure compensation: \n");

	reg = BME280_P1_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.p1 = ((uint16_t)rx[1] << 8) | (uint16_t)rx[0];
	printf("rx: %02x %02x, p1=0x%04x\n", rx[0], rx[1], calib_data.p1);

	reg = BME280_P2_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.p2 = ((uint16_t)rx[1] << 8) | (uint16_t)rx[0];
	printf("rx: %02x %02x, p2=0x%04x\n", rx[0], rx[1], calib_data.p2 & 0xFFFF);

	reg = BME280_P3_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.p3 = ((uint16_t)rx[1] << 8) | (uint16_t)rx[0];
	printf("rx: %02x %02x, p3=0x%04x\n", rx[0], rx[1], calib_data.p3);

	reg = BME280_P4_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.p4 = ((uint16_t)rx[1] << 8) | (uint16_t)rx[0];
	printf("rx: %02x %02x, p4=0x%04x\n", rx[0], rx[1], calib_data.p4);

	reg = BME280_P5_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.p5 = ((uint16_t)rx[1] << 8) | (uint16_t)rx[0];
	printf("rx: %02x %02x, p5=0x%04x\n", rx[0], rx[1], calib_data.p5);

	reg = BME280_P6_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.p6 = ((uint16_t)rx[1] << 8) | (uint16_t)rx[0];
	printf("rx: %02x %02x, p6=0x%03x\n", rx[0], rx[1], calib_data.p6 &0xFFFF);

	reg = BME280_P7_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.p7 = ((uint16_t)rx[1] << 8) | (uint16_t)rx[0];
	printf("rx: %02x %02x, p7=0x%04x\n", rx[0], rx[1], calib_data.p7);

	reg = BME280_P8_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.p8 = ((uint16_t)rx[1] << 8) | (uint16_t)rx[0];
	printf("rx: %02x %02x, p8=0x%03x\n", rx[0], rx[1], calib_data.p8 & 0xFFFF) ;

	reg = BME280_P9_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.p9 = ((uint16_t)rx[1] << 8) | (uint16_t)rx[0];
	printf("rx: %02x %02x, p9=0x%04x\n\n", rx[0], rx[1], calib_data.p9);
}

static double bme280_calculate_pressure(uint32_t reg_data)
{
#if 1	

	//printf("%s\n", __FUNCTION__);
	double var1;
	double var2;
	double var3;
	double press;
	double press_min = 30000.0;
	double press_max = 110000.0;

	var1 = ((double)calib_data.t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)calib_data.p6) / 32768.0;
	var2 = var2 + var1 * ((double)calib_data.p5) * 2.0;
	var2 = (var2 / 4.0) + (((double)calib_data.p4) * 65536.0);
	var3 = ((double)calib_data.p3) * var1 * var1 / 524288.0;
	var1 = (var3 + ((double)calib_data.p2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)calib_data.p1);

	/* avoid exception caused by division by zero */
	if (var1 > (0.0))
	{
		press = 1048576.0 - (double)reg_data;
		press = (press - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double)calib_data.p9) * press * press / 2147483648.0;
		var2 = press * ((double)calib_data.p8) / 32768.0;
		press = press + (var1 + var2 + ((double)calib_data.p7)) / 16.0;

		if (press < press_min)
		{
			press = press_min;
		}
		else if (press > press_max)
		{
			press = press_max;
		}
	}
	else /* Invalid case */
	{
		press = press_min;
	}

	return press;

#else

	int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    uint32_t var5;
    uint32_t press;
    uint32_t press_min = 30000;
    uint32_t press_max = 110000;

	var1 = (((int32_t)calib_data.t_fine) / 2) - (int32_t)64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_data.p6);
    var2 = var2 + ((var1 * ((int32_t)calib_data.p5)) * 2);
    var2 = (var2 / 4) + (((int32_t)calib_data.p4) * 65536);
    var3 = (calib_data.p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t)calib_data.p2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t)calib_data.p1)) / 32768;

    /* avoid exception caused by division by zero */
    if (var1)
    {
        var5 = (uint32_t)((uint32_t)1048576) - reg_data;
		press = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;

		if (press < 0x80000000)
        {
			press = (press << 1) / ((uint32_t)var1);
		}
        else
        {
			press = (press / (uint32_t)var1) * 2;
		}

        var1 = (((int32_t)calib_data.p9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
        var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_data.p8)) / 8192;
        pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_data.p7) / 16));

		if (press < press_min)
		{
			press = press_min;
		}
		else if (press > press_max)
		{
			press = press_max;
		}
    }
    else
    {
		press = press_min;
	}

	return press;

#endif	
}


static uint16_t bmp_read_pressure(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t rx[3];
	uint8_t reg;
	uint32_t reg_data;

	reg = MBP280_PRESSURE_REG;
	i2c_read_reg(&reg, rx, 3);

	reg_data = (((uint32_t)rx[0]) << 12) | (((uint32_t)rx[1]) << 4) | (((uint32_t)rx[2]) >> 4);

	printf("press_reg=0x%04x\n", reg_data);

	return (pressure = bme280_calculate_pressure(reg_data));
}


/*********************************************************
 * 					temperature
 * *******************************************************/

static void bmp_read_temp_calib_data(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t rx[2];
	uint8_t reg;

	printf("read temperatur compensation: \n");

	/* T1 */
	reg = BME280_T1_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.t1 = (((uint16_t)rx[1]) << 8) | ((uint16_t)rx[0]);
	printf("rx: %02x %02x, t1=0x%04x\n", rx[0], rx[1], calib_data.t1);

	/* T2 */
	reg = BME280_T2_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.t2 = (int16_t)(((uint16_t)rx[1]) << 8) | ((uint16_t)rx[0]);
	printf("rx: %02x %02x, t2=0x%04x\n", rx[0], rx[1], calib_data.t2);

	/* T3 */
	reg = BME280_T3_REG;
	i2c_read_reg(&reg, rx, 2);
	calib_data.t3 = (int16_t)(((uint16_t)rx[1]) << 8) | ((uint16_t)rx[0]);
	printf("rx: %02x %02x, t3=0x%04x\n\n", rx[0], rx[1], calib_data.t3);
}


static double bme280_calculate_temperature(uint32_t reg_data)
{
#if 1	
	//printf("%s\n", __FUNCTION__);
	double var1;
	double var2;
	double temp;
	double temp_min = -40;
	double temp_max = 85;

	var1 = (((double)reg_data) / 16384.0 - ((double)calib_data.t1) / 1024.0) * ((double)calib_data.t2);
	var2 = (((double)reg_data) / 131072.0 - ((double)calib_data.t1) / 8192.0);
	var2 = (var2 * var2) * ((double)calib_data.t3);
	calib_data.t_fine = (int32_t)(var1 + var2);
	temp = (var1 + var2) / 4850.0; // 5120.0

	if (temp < temp_min) {
		temp = temp_min;
	} else if (temp > temp_max) {
        temp = temp_max;
	}

	return temp;

#else

	//printf("%s\n", __FUNCTION__);
	int32_t var1;
	int32_t var2;
	int32_t temp;
	int32_t temp_min = -4000;
	int32_t temp_max = 8500;

	var1 = (int32_t)((reg_data / 8) - ((int32_t)calib_data.t1 * 2));
	var1 = (var1 * ((int32_t)calib_data.t2)) / 2048;
	var2 = (int32_t)((reg_data / 16) - ((int32_t)calib_data.t1));
	var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data.t3)) / 16384;
	calib_data.t_fine = var1 + var2;
	temp = (calib_data.t_fine * 5 + 128) / 256;

	if (temp < temp_min)
	{
		temp = temp_min;
	}
	else if (temp > temp_max)
	{
		temp = temp_max;
	}

	return temp;

#endif
}	


static uint16_t bmp_read_temperature(void)
{
	printf("%s\n", __FUNCTION__);
	uint8_t rx[3];
	uint8_t reg;
	uint32_t reg_data;

	reg = BME280_TEMPERATURE_REG;
	i2c_read_reg(&reg, rx, 3);

	reg_data = (((uint32_t)rx[0]) << 12) | (((uint32_t)rx[1]) << 4) | (((uint32_t)rx[2]) >> 4);

	printf("temp_reg=0x%04x\n", reg_data);

	return (temperature = bme280_calculate_temperature(reg_data));
}

/*********************************************************
 * 					Signal Handler
 * *******************************************************/
void sigterm_handler(int signal, siginfo_t *info, void *_unused)
{
	printf("\n\nReceived signo = %d sigcode = %d  signal from process with pid = %u\n\n", info->si_signo, info->si_code, info->si_pid);
	
	close(file);
	close(bme280_fd);
	
	exit(0);
}

/*********************************************************
 * 							main
 * *******************************************************/

int main( int argc, char *argv[])
{
	printf("%s >>\n", __FUNCTION__);

	char file_name[] = "/home/debian/tmp/bme280.log";
	char dev[] = I2C_DEVICE_PATH;
	char str[100];
	time_t raw_time;
	struct tm *current_time;
	
	struct sigaction action = {
		.sa_handler = NULL,
		.sa_sigaction = sigterm_handler,
		.sa_mask = 0,
		.sa_flags = SA_SIGINFO,
		.sa_restorer = NULL};
	sigaction(SIGTERM, &action, NULL);
	sigaction(SIGKILL, &action, NULL);
	sigaction(SIGINT, &action, NULL); // Ctrl+c
	

	if(1 < argc) {
		dev[strlen(dev)-1] = *argv[1];
	} else {
		/* default i2c device */
		dev[strlen(dev)-1] = 2 + 0x30;
	}

	if( 0 > (i2c_init(dev))) {
		printf("%d: Failed to initialize I2C bus\n", __LINE__);
		close(bme280_fd);
		printf("%s <<\n", __FUNCTION__);
		return -1;
	}

	if (0 > (file = open(file_name, O_CREAT | O_WRONLY | O_APPEND, S_IRUSR | S_IWUSR))) {
		printf("%d: cannot create/open file %s: %s\n", __LINE__, file_name, strerror(errno));
	} else {
		char tmp[] = "----------------------------------------> Measurement <----------------------------------------\n";
		write(file, tmp, strlen(tmp));
	}

	/* init BME280 sensor */
	bme280_init();

	bme280_sleep_mode();
	bme280_normal_mode();
	sleep(1);

	printf("Starting measurement: \n\n");
	
	while (1) {
		/* start measurement */
		//bme280_force_mode();

		/* read measurements */
		bme280_get_sensor_data();

		time(&raw_time);
		current_time = localtime(&raw_time);

		sprintf(str, "%d-%02d-%02d %02d:%02d:%02d\ttemperature: %0.2f [°C], pressure: %0.2f [hPa], humidity: %0.2f [\%rH]\n",
				current_time->tm_year + 1900,
				current_time->tm_mon + 1,
				current_time->tm_mday,
				current_time->tm_hour,
				current_time->tm_min,
				current_time->tm_sec,
				temperature, pressure / 100, humidity);

		if (0 < file)
		{
			write(file, str, strlen(str));
		}
		else
		{
			printf("%s\n", str);
			//printf("%d-%02d-%02d %02d:%02d:%02d\ttemperature: %0.2f [°C], pressure: %0.2f [hPa], humidity: %0.2f [\%rH]\n",
			//	   current_time->tm_year + 1900,
				//    current_time->tm_mon + 1,
				//    current_time->tm_mday,
				//    current_time->tm_hour,
				//    current_time->tm_min,
				//    current_time->tm_sec,
				//    temperature, pressure / 100, humidity);
			//printf("temperature: %ld [°C], pressure: %ld [hPa], humidity: %ld [\%]\n", temperature, pressure / 100, humidity);
		}
		//sleep(60); //1min
		sleep(1200);
	}

#if 0
	temperature = bmp_read_temperature();
	printf("temperature: %hf [°C]\n", temperature);

	pressure = bmp_read_pressure();
	printf("pressure: %f []\n", pressure);

	humidity = bmp_read_humidity();
	printf("humidity: %f [\%]\n", humidity);
#endif
	
	close(file);
	close(bme280_fd);
	
	return 1;
}
