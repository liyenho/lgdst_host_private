/*
 * @(#)Afatech_OMEGA.h
 *
 * ==========================================================
 * Version: 2.0
 * Date:    2009.06.15
 * ==========================================================
 *
 * ==========================================================
 * History:
 *
 * Date         Author      Description
 * ----------------------------------------------------------
 *
 * 2009.06.15   M.-C. Ho    new tuner
 * ==========================================================
 *
 * Copyright 2007 Afatech, Inc. All rights reserved.
 *
 */


#ifndef __Afatech_OMEGA_H__
#define __Afatech_OMEGA_H__


#define OMEGA_VER_MAJOR    2
#define OMEGA_VER_MINOR    0


extern TunerDescription tunerDescription;


/**
 *
 */
uint32_t OMEGA_open (
Demodulator*    demodulator,
	uint8_t			chip
);


/**
 *
 */
uint32_t OMEGA_close (
	  uint8_t			chip
);


/**
 *
 */
uint32_t OMEGA_set (
Demodulator*    demodulator,
	  uint8_t			chip,
      uint16_t			bandwidth,
      uint32_t			frequency
);


/**
 * support LNA Type.
 *
 * @param demodulator the handle of demodulator.
 * @param supporttype the type of LNA .
 *        0: OMEGA ; 1: OMEGA_LNA_Config_1 ; 1: OMEGA_LNA_Config_2
 * @return Error_NO_ERROR: successful, non-zero error code otherwise.
 * @example <pre>
 *     uint32_t error = Error_NO_ERROR;
 *     DefaultDemodulator demod;
 *
 *     error = OMEGA_supportLNA(demod, 0); // 0: OMEGA ; 1: OMEGA_LNA_Config_1 ; 1: OMEGA_LNA_Config_2
 *     if (error){
 *		  printf("Initialize LNA type failed.0x%08x\n", error);
 *		  return;
 *	   }
 *     error = Demodulator_initialize (demod, streamType);
 * </pre>
 */
uint32_t OMEGA_supportLNA (
Demodulator*    demodulator,
      uint8_t            supporttype
 );
#endif