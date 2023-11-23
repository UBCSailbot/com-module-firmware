#include "veml3328_hardware.h" 
#include "veml3328_software.h" 

I2C_HandleTypeDef hi2c1;
HAL_StatusTypeDef r,g,b;
uint16_t conf, r_data, g_data, b_data;
double r_lux;

// write to reg in i2c
HAL_StatusTypeDef veml3328_wr(uint8_t registerAddress, uint16_t value) {
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Write(&hi2c1, veml3328_addr<< 1, registerAddress, sizeof(registerAddress), (uint8_t*)&value, sizeof(value), 100);
	if (status) {
		return status;
	}

	return status;
}

// read from reg in i2c
HAL_StatusTypeDef veml3328_rd(uint8_t registerAddress, uint16_t* value) {
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c1, veml3328_addr << 1, registerAddress, sizeof(registerAddress), (uint8_t*)value, sizeof(*value), 100);
	if (status) {
		return status;
	}

	return status;
}

// initialize sensor
HAL_StatusTypeDef veml3328_init(void) {
	
	HAL_StatusTypeDef stat0, stat1, stat2, deviceId;
	uint16_t regVal;

	// Check i2c1 port 
	stat0 = HAL_I2C_Init(&hi2c1);
	if (stat0 != HAL_OK) {
		return 0;
	} 
	 
	// Check register device ID (should be 0x28)
	stat1 = veml3328_rd(veml3328_reg_deviceID, &regVal);
	deviceId = (uint8_t)(regVal & 0xFF);
	if (deviceId != 0x28) {
		return 0;
	}
	
	return 1;
}

// configure sensor. the mode here should be replaced with command registers
void veml3328_config(uint16_t mode){
	veml3328_wr(veml3328__conf, mode); // change 0000 to change the sensor config.
	// veml3328_rd(veml3328__conf, &conf); check if its configed right
}

// read lux value
void veml3328_rd_lux(void){
	r_lux = convert(r_data);
}

// read rgb value
void veml3328_rd_rgb(void){
    r = veml3328_rd(R_, &r_data);
    g = veml3328_rd(G_, &g_data);
    b = veml3328_rd(B_, &b_data);
}


