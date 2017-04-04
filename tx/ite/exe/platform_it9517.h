#ifndef __PLATFORM_IT9517_H__
#define __PLATFORM_IT9517_H__

#include "it9510.h"

uint32_t it9517_loadIQ_calibration_table (const char*file_name);
uint32_t it9517_initialize (uint8_t id_bus,TsInterface stream_type);
uint32_t it9517_monitor_version (void) ;
uint32_t it9517_suspend(uint8_t enable);
uint32_t it9517_reset(void);
uint32_t it9517_reboot(void);
uint32_t it9517_set_channel_modulation(ChannelModulation channelModulation,NullPacketMode mode);
uint32_t it9517_set_frequency(uint32_t frequency);
uint32_t it9517_set_ts_interface(TsInterface   streamType);
uint32_t it9517_control_power_saving(uint8_t control);
uint32_t it9517_acquire_channel(uint32_t frequency,uint16_t bandwidth);
uint32_t it9517_enable_transmission_mode(uint8_t enable);
uint32_t it9517_get_output_gain(void);
uint32_t it9517_get_output_gain_range(uint32_t frequency,uint16_t bandwidth);
uint32_t it9517_adjust_output_gain(int gain);
uint32_t it9517_control_pidfilter(uint8_t control,uint8_t enable);
uint32_t it9517_reset_pidfilter(void);
uint32_t it9517_add_pidfilter(uint8_t index,uint32_t value);
uint32_t it9517_enable_tps_encryption(uint32_t key);
uint32_t it9517_disable_tps_encryption(void);
uint32_t it9517_aes_encryption(uint8_t *buf,uint8_t begin,Bool enable);
uint32_t it9517_pcr_restamp(PcrMode pcr_mode,uint8_t enable);
uint32_t it9517_check_tsbuffer_overflow(void);
uint32_t it9517_read_eeprom();
uint32_t it9517_write_eeprom();
uint32_t it9517_finalize(void);

#endif
