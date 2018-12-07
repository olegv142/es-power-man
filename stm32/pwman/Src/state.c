#include "state.h"
#include "main.h"
#include "stm32l0xx_hal.h"

struct state g_state;

extern I2C_HandleTypeDef hi2c2;

#define I2C_ADDR (0xb<<1)
#define I2C_TOUT 50

static inline void read_batt(uint8_t what, void* val, uint8_t vsize)
{
	if (HAL_OK != HAL_I2C_Mem_Read(&hi2c2, I2C_ADDR, what, 1, val, vsize, I2C_TOUT)) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

#define READ_BATT(what, val) read_batt(what, &val, sizeof(val))

static inline void query_batt_info(struct batt_info * batt)
{
	READ_BATT(0x8,  batt->temp);    // in 0.1K units
	READ_BATT(0x9,  batt->volt);    // in mV units
	READ_BATT(0xa,  batt->curr);    // in mA units
	READ_BATT(0xb,  batt->acurr);   // in mA units
	READ_BATT(0x3f, batt->cell[0]); // cell voltage in mV
	READ_BATT(0x3e, batt->cell[1]); // cell voltage in mV
	READ_BATT(0x3d, batt->cell[2]); // cell voltage in mV
	READ_BATT(0x54, batt->opst);    // operation status bits
}

#define BATT_INFO_POLL_DELAY 1000

// The routine called from the main application loop
void state_process(void)
{
	if ((uint32_t)(HAL_GetTick() - g_state.batt_ts) > BATT_INFO_POLL_DELAY)
	{
		query_batt_info(&g_state.batt);
		g_state.batt_ts = HAL_GetTick();
		g_state.is_charging = is_batt_charging(g_state.batt.opst);
	}
}

