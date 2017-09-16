#ifndef __PLATFORM_IT9137_H__
#define __PLATFORM_IT9137_H__

#include "it9130.h"
#include "omega.h"
#include "tuner.h"
#include "cmd.h"
#include "error.h"
#include "register.h"
#include "standard.h"
#include "type.h"
#include "user.h"
#include "variable.h"

uint32_t it9137_init(void);
uint32_t it9137_deinit(void);
uint32_t it9137_reset(void);
uint32_t it9137_reboot(void);
uint32_t it9137_get_firmwareversion(void);
uint32_t it9137_acquire_channel(uint8_t chip,uint32_t frequency,uint16_t bandwidth);
uint32_t it9137_scan_channel(uint8_t chip,
																	uint32_t start_frequency,
																	uint32_t end_frequency,
																	uint16_t bandwidth,
																	uint16_t bw_total, // include guard band
																	long *strengthdbm);
uint32_t it9137_control_pid_filter(uint8_t chip,uint8_t control);
uint32_t it9137_reset_pid_filter(uint8_t chip);
uint32_t it9137_add_pid_filter(uint8_t chip,uint8_t index,Pid pid);
uint32_t it9137_control_power_saving(uint8_t chip,uint8_t control);
uint32_t it9137_check_tpslocked(uint8_t chip, void* istpslocked);
uint32_t it9137_check_mpeg2locked(uint8_t chip);
uint32_t it9137_get_channel_modulation(uint8_t chip);
uint32_t it9137_get_signal_quality(uint8_t chip);
uint32_t it9137_get_signal_quality_indication(uint8_t chip);
uint32_t it9137_get_signal_strength(uint8_t chip);
uint32_t it9137_get_signal_strength_indication(uint8_t chip);
// enable signal strength (dbm) reads, liyenho
uint32_t it9137_get_signal_strength_dbm(uint8_t chip, long *sigdbm);
// enable post viterbi error rate reads, liyenho
uint32_t it9137_get_postviterbi_bit_error_rate(uint8_t chip,double *ptvitber);
uint32_t it9137_get_statistic(uint8_t chip);
// enable signal S/N reads, liyenho
uint32_t it9137_get_snr(uint8_t chip, uint8_t *snr1);
uint32_t it9137_get_if_agc(uint8_t chip);
uint32_t it9137_get_rf_agc_gain(uint8_t chip);
uint32_t it9137_set_streamtype( StreamType  streamType);
uint32_t it9137_set_architecture( Architecture  architecture);
// video channel scan facility
uint32_t it9137_video_channel_scan(Booll ran_once);
// special register query function for rx ofdm debug, liyenho
uint32_t it9137_read_ofdm_registers(FILE *dump);
#endif
