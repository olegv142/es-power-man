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
	uint32_t chgst;   // charging status bits
};

struct state {
	status_t         status;
	uint32_t         status_ts;
	// Other stuff TBD
	struct batt_info batt;
	uint32_t         batt_ts;
	int              is_charging;
};

// Operation status flags (most interesting ones)
#define OPST_PRES  (1 << 8)  // System present
#define OPST_DSG   (1 << 9)  // Discharge enabled
#define OPST_CHG   (1 << 10) // Charging enabled
#define OPST_PCHG  (1 << 11) // Pre charging enabled
#define OPST_ACFET (1 << 12) // Adapter FET open
#define OPST_FUSE  (1 << 13) // Fuse status
#define OPST_ACLOW (1 << 14) // Adapter voltage is low

// Charging status bits (most interesting ones)
#define CHGST_PV  (1 << 16)  // Pre charge mode
#define CHGST_LV  (1 << 17)  // Low voltage mode
#define CHGST_MV  (1 << 18)  // Medium voltage mode
#define CHGST_HV  (1 << 19)  // High voltage mode
#define CHGST_IN  (1 << 20)  // Charging inhibited
#define CHGST_SU  (1 << 21)  // Charging suspended
#define CHGST_MC  (1 << 22)  // Maintenance charging
#define CHGST_VCT (1 << 23)  // Valid charging termination

static inline int is_batt_charging(uint16_t opst)
{
	return !(opst & OPST_ACLOW) && (opst & OPST_CHG);
}

extern struct state g_state;

// The routine called from the main application loop
void state_process(void);
