#pragma once

#include <stdint.h>

typedef enum {
	st_idle,
	// Actually there are more than one active state - TBD
	st_active,
} status_t;

struct batt_info {
	uint16_t temp;    // in 0.1K units
	uint16_t volt;    // in mV units
	int16_t  curr;    // in mA units (negative for discharge)
	int16_t  acurr;   // in mA units (negative for discharge)
	uint16_t cell[3]; // cell voltage in mV
	uint16_t opst;    // operation status bits
};

struct state {
	status_t         status;
	uint32_t         status_ts;
	// Other stuff TBD
	struct batt_info batt;
	uint32_t         batt_ts;
	int              is_charging;
};

// Operation status flags
#define OPST_PRES  (1 << 8)  // System present
#define OPST_DSG   (1 << 9)  // Discharge enabled
#define OPST_CHG   (1 << 10) // Charging enabled
#define OPST_PCHG  (1 << 11) // Pre charging enabled
#define OPST_ACFET (1 << 12) // Adapter FET open
#define OPST_FUSE  (1 << 13) // Fuse status
#define OPST_ACLOW (1 << 14) // Adapter voltage is low

static inline int is_batt_charging(uint16_t opst)
{
	return !(opst & OPST_ACLOW) && (opst & OPST_CHG);
}

extern struct state g_state;

// The routine called from the main application loop
void state_process(void);
