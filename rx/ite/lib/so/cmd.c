#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libusb.h>
#include "usb_rx.h"
#include "cmd.h"

extern struct libusb_device_handle*devh;
static uint8_t Cmd_sequence = 0;
extern pthread_mutex_t mux;


uint32_t Cmd_busTx (
     uint32_t           bufferLength,
    uint8_t*           buffer
) {
	 int32_t 		i, r, msg[80]; // access buffer
	 dev_access *acs = (dev_access*)msg;
    uint32_t     error = Error_NO_ERROR;

     acs->access = IT913X_WRITE;
     acs->dcnt = bufferLength;
     acs->addr = IT913X_ADDRESS;
     memcpy(acs->data, buffer, bufferLength);

    for (i = 0; i < User_RETRY_MAX_LIMIT; i++) {
		 	pthread_mutex_lock(&mux);
		r = libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
			pthread_mutex_unlock(&mux);
         if (r == (sizeof(*acs)+(acs->dcnt-1))) goto exit;
         else error = -r; // error # defined in libusb, liyenho

        short_sleep (/*0.1*/0.01);
    }

exit:
    return (error);
}


uint32_t Cmd_busRx (
    uint32_t           bufferLength,
    uint8_t*           buffer
) {
    int32_t	  i, r, msg[80]; // access buffer
	 dev_access *acs = (dev_access*)msg;
    uint32_t     error = Error_NO_ERROR;

     acs->access = IT913X_READ;
     acs->dcnt = bufferLength;
     acs->addr = IT913X_ADDRESS;

    for (i = 0; i < User_RETRY_MAX_LIMIT; i++) {
		 	pthread_mutex_lock(&mux);
        	r =	libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
			pthread_mutex_unlock(&mux);
				short_sleep(/*0.1*/0.01); 	// validate echo after 0.1 sec

         if (r != (sizeof(*acs)+(acs->dcnt-1)))
         	continue ;

			while(1) {
		 		pthread_mutex_lock(&mux);
				if (libusb_control_transfer(devh,CTRL_IN, USB_RQ,USB_HOST_MSG_RX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0)){
					pthread_mutex_unlock(&mux);
					break;
				}
				pthread_mutex_unlock(&mux);
				short_sleep(0.0005);
			}
			break ;
    }
	memcpy(buffer, acs->data, bufferLength);
exit:
    return (error);
}

uint32_t Cmd_addChecksum (
    uint32_t*          bufferLength,
    uint8_t*           buffer
) {
    uint32_t error  = Error_NO_ERROR;
    uint32_t loop   = (*bufferLength - 1) / 2;
    uint32_t remain = (*bufferLength - 1) % 2;
    uint32_t i;
    uint16_t  checksum = 0;

    for (i = 0; i < loop; i++)
        checksum = checksum + (uint16_t) (buffer[2 * i + 1] << 8) + (uint16_t) (buffer[2 * i + 2]);
    if (remain)
        checksum = checksum + (uint16_t) (buffer[*bufferLength - 1] << 8);

    checksum = ~checksum;
    buffer[*bufferLength]     = (uint8_t) ((checksum & 0xFF00) >> 8);
    buffer[*bufferLength + 1] = (uint8_t) (checksum & 0x00FF);
    buffer[0]                 = (uint8_t) (*bufferLength + 1);  // because buffer[0] indicates count which does NOT include itself, liyenho
    *bufferLength            += 2;

    return (error);
}


uint32_t Cmd_removeChecksum (
    uint32_t*          bufferLength,
    uint8_t*           buffer
) {
    uint32_t error    = Error_NO_ERROR;
    uint32_t loop     = (*bufferLength - 3) / 2;
    uint32_t remain   = (*bufferLength - 3) % 2;
    uint32_t i;
    uint16_t  checksum = 0;

    for (i = 0; i < loop; i++)
        checksum = checksum + (uint16_t) (buffer[2 * i + 1] << 8) + (uint16_t) (buffer[2 * i + 2]);
    if (remain)
        checksum = checksum + (uint16_t) (buffer[*bufferLength - 3] << 8);

    checksum = ~checksum;
    if (((uint16_t)(buffer[*bufferLength - 2] << 8) + (uint16_t)(buffer[*bufferLength - 1])) != checksum) {
        error = Error_WRONG_CHECKSUM;
        goto exit;
    }
    if (buffer[2])
        error = Error_FIRMWARE_STATUS | buffer[2];

    buffer[0]      = (uint8_t) (*bufferLength - 3);
    *bufferLength -= 2;

exit :
    return (error);
}

uint32_t Cmd_readRegisters (
    uint8_t            chip,
    Processor       processor,
    uint32_t           registerAddress,
    uint8_t            registerAddressLength,
    uint32_t           readBufferLength,
    uint8_t*           readBuffer
) {
    uint32_t       error = Error_NO_ERROR;
    uint16_t        command;
    uint8_t        buffer[255];
    uint32_t       bufferLength;
    uint32_t       sendLength;
    uint32_t       remainLength;
    uint32_t       i, k;
    uint32_t       maxFrameSize = 255;

    if (readBufferLength == 0) goto exit;
    if (registerAddressLength > 4) {
        error  = Error_PROTOCOL_FORMAT_INVALID;
        goto exit;
    }

    if ((readBufferLength + 5) > User_MAX_PKT_SIZE) {
        error = Error_INVALID_DATA_LENGTH;
        goto exit;
    }

    if ((readBufferLength + 5) > maxFrameSize) {
        error = Error_INVALID_DATA_LENGTH;
        goto exit;
    }


    /** add frame header */
    command   = Cmd_buildCommand (Command_REG_DEMOD_READ, processor, chip);
    buffer[1] = (uint8_t) (command >> 8);
    buffer[2] = (uint8_t) command;
    buffer[3] = (uint8_t) Cmd_sequence++;
    buffer[4] = (uint8_t) readBufferLength;
    buffer[5] = (uint8_t) registerAddressLength;
    buffer[6] = (uint8_t) (registerAddress >> 24); /** Get first byte of reg. address  */
    buffer[7] = (uint8_t) (registerAddress >> 16); /** Get second byte of reg. address */
    buffer[8] = (uint8_t) (registerAddress >> 8);  /** Get third byte of reg. address  */
    buffer[9] = (uint8_t) (registerAddress);       /** Get fourth byte of reg. address */

    /** add frame check-sum */
    bufferLength = 10;
    error = Cmd_addChecksum (&bufferLength, buffer);
    if (error) goto exit;


    /** send frame */
    i = 0;
    sendLength   = 0;
    remainLength = bufferLength;
    while (remainLength > 0) {
        i     = (remainLength > User_MAX_PKT_SIZE) ? (User_MAX_PKT_SIZE) : (remainLength);
        error = Cmd_busTx ( i, &buffer[sendLength]);
        if (error) goto exit;

        sendLength   += i;
        remainLength -= i;
    }

    usleep(100000/*50000*/);  // gap write and read ops with sufficient delay

    /** get reply frame */
    bufferLength = 5 + readBufferLength;
    error = Cmd_busRx (bufferLength, buffer);
    if (error) goto exit;

    /** remove check-sum from reply frame */
    error = Cmd_removeChecksum (&bufferLength, buffer);
    if (error) goto exit;

    for (k = 0; k < readBufferLength; k++) {
        readBuffer[k] = buffer[k + 3];
    }

exit :
    return (error);
}
uint32_t Cmd_writeRegisters (
    uint8_t            chip,
    Processor       processor,
    uint32_t           registerAddress,
    uint8_t            registerAddressLength,
    uint32_t           writeBufferLength,
    uint8_t*           writeBuffer
) {
    uint32_t       error = Error_NO_ERROR;
    uint16_t        command;
    uint8_t        buffer[255];
    uint32_t       bufferLength;
    uint32_t       remainLength;
    uint32_t       sendLength;
    uint32_t       i;
    uint32_t       maxFrameSize = 255;

    if (writeBufferLength == 0) goto exit;
    if (registerAddressLength > 4) {
        error  = Error_PROTOCOL_FORMAT_INVALID;
        goto exit;
    }

    if ((writeBufferLength + 12) > maxFrameSize) {
        error = Error_INVALID_DATA_LENGTH;
        goto exit;
    }


    /** add frame header */
    command   = Cmd_buildCommand (Command_REG_DEMOD_WRITE, processor, chip);
    buffer[1] = (uint8_t) (command >> 8);
    buffer[2] = (uint8_t) command;
    buffer[3] = (uint8_t) Cmd_sequence++;
    buffer[4] = (uint8_t) writeBufferLength;
    buffer[5] = (uint8_t) registerAddressLength;
    buffer[6] = (uint8_t) ((registerAddress) >> 24); /** Get first byte of reg. address  */
    buffer[7] = (uint8_t) ((registerAddress) >> 16); /** Get second byte of reg. address */
    buffer[8] = (uint8_t) ((registerAddress) >> 8);  /** Get third byte of reg. address  */
    buffer[9] = (uint8_t) (registerAddress );        /** Get fourth byte of reg. address */

    /** add frame data */
    for (i = 0; i < writeBufferLength; i++) {
        buffer[10 + i] = writeBuffer[i];
    }

    /** add frame check-sum */
    bufferLength = 10 + writeBufferLength;
    error = Cmd_addChecksum (&bufferLength, buffer);
    if (error) goto exit;

    /** send frame */
    i = 0;
    sendLength = 0;
    remainLength = bufferLength;
    while (remainLength > 0) {
        i     = (remainLength > User_MAX_PKT_SIZE) ? (User_MAX_PKT_SIZE) : (remainLength);
        error = Cmd_busTx (i, &buffer[sendLength]);
        if (error) goto exit;

        sendLength   += i;
        remainLength -= i;
    }

    usleep(100000/*50000*/);  // gap write and read ops with sufficient delay

    /** get reply frame */  // spec from China team not the Asic requirement! liyenho
    bufferLength = 5;
    error = Cmd_busRx ( bufferLength, buffer);
    if (error) goto exit;

    /** remove check-sum from reply frame */
    error = Cmd_removeChecksum (&bufferLength, buffer);
    if (error) goto exit;

exit :
    return (error);
}
uint32_t Cmd_sendCommand (
    uint16_t            command,
    uint8_t            chip,
    Processor       processor,
    uint32_t           writeBufferLength,
    uint8_t*           writeBuffer,
    uint32_t           readBufferLength,
    uint8_t*           readBuffer
) {
    uint32_t       error = Error_NO_ERROR;
    uint8_t        buffer[255];
    uint32_t       bufferLength;
    uint32_t       remainLength;
    uint32_t       sendLength;
    uint32_t       i, k;
    uint32_t       maxFrameSize = 255;

    if ((writeBufferLength + 6) > maxFrameSize) {
        error = Error_INVALID_DATA_LENGTH;
        goto exit;
    }

    if ((readBufferLength + 5) > User_MAX_PKT_SIZE) {
        error  = Error_INVALID_DATA_LENGTH;
        goto exit;
    }

    if ((readBufferLength + 5) > maxFrameSize) {
        error = Error_INVALID_DATA_LENGTH;
        goto exit;
    }

    if (writeBufferLength == 0) {
        command   = Cmd_buildCommand (command, processor, chip);
        buffer[1] = (uint8_t) (command >> 8);
        buffer[2] = (uint8_t) command;
        buffer[3] = (uint8_t) Cmd_sequence++;
        bufferLength = 4;
        error = Cmd_addChecksum (&bufferLength, buffer);
        if (error) goto exit;

        // send command packet
        i = 0;
        sendLength = 0;
        remainLength = bufferLength;
        while (remainLength > 0) {
            i = (remainLength > User_MAX_PKT_SIZE) ? (User_MAX_PKT_SIZE) : (remainLength);
            error = Cmd_busTx (i, &buffer[sendLength]);
            if (error) goto exit;

            sendLength   += i;
            remainLength -= i;
        }
    } else {
        command   = Cmd_buildCommand (command, processor, chip);
        buffer[1] = (uint8_t) (command >> 8);
        buffer[2] = (uint8_t) command;
        buffer[3] = (uint8_t) Cmd_sequence++;
        for (k = 0; k < writeBufferLength; k++)
            buffer[k + 4] = writeBuffer[k];


        bufferLength = 4 + writeBufferLength;
        error = Cmd_addChecksum (&bufferLength, buffer);
        if (error) goto exit;


        /** send command */
        i = 0;
        sendLength = 0;
        remainLength = bufferLength;
        while (remainLength > 0) {
            i     = (remainLength > User_MAX_PKT_SIZE) ? (User_MAX_PKT_SIZE) : (remainLength);
            error = Cmd_busTx (i, &buffer[sendLength]);
            if (error) goto exit;

            sendLength   += i;
            remainLength -= i;
        }
    }

    usleep(/*50000*/10000);  // gap write and read ops with sufficient delay
    bufferLength = 5 + readBufferLength;

    error = Cmd_busRx (bufferLength, buffer);
    if (error) goto exit;

    error = Cmd_removeChecksum (&bufferLength, buffer);
    if (error) goto exit;

    if (readBufferLength) {
        for (k = 0; k < readBufferLength; k++) {
            readBuffer[k] = buffer[k + 3];
        }
    }

exit :
    return (error);
}

uint32_t Cmd_writeTunerRegisters (
    IN  uint8_t            chip,
    IN  uint8_t            tunerAddress,
    IN  uint16_t            registerAddress,
    IN  uint8_t            registerAddressLength,
    IN  uint8_t            writeBufferLength,
    IN  uint8_t*           writeBuffer
) {
    uint32_t       error = Error_NO_ERROR;
    uint16_t        command;
    uint8_t        buffer[255];
    uint32_t       bufferLength;
    uint32_t       remainLength;
    uint32_t       sendLength;
    uint32_t       i;
    uint32_t       maxFrameSize = 255;


    if (writeBufferLength == 0) goto exit;


    if ((uint32_t)(writeBufferLength + 11) > maxFrameSize) {
        error  = Error_INVALID_DATA_LENGTH;
        goto exit;
    }

    /** add frame header */
    command   = Cmd_buildCommand (Command_REG_TUNER_WRITE, Processor_LINK, chip);
    buffer[1] = (uint8_t) (command >> 8);
    buffer[2] = (uint8_t) command;
    buffer[3] = (uint8_t) Cmd_sequence++;
    buffer[4] = (uint8_t) writeBufferLength;
    buffer[5] = (uint8_t) tunerAddress;
    buffer[6] = (uint8_t) registerAddressLength;
    buffer[7] = (uint8_t) (registerAddress >> 8);  /** Get high uint8_t of reg. address */
    buffer[8] = (uint8_t) (registerAddress);       /** Get low uint8_t of reg. address  */

    /** add frame data */
    for (i = 0; i < writeBufferLength; i++) {
        buffer[9 + i] = writeBuffer[i];
    }

    /** add frame check-sum */
    bufferLength = 9 + writeBufferLength;
    error = Cmd_addChecksum (&bufferLength, buffer);
    if (error) goto exit;

    /** send frame */
    i = 0;
    sendLength   = 0;
    remainLength = bufferLength;
    while (remainLength > 0) {
        i = (remainLength > User_MAX_PKT_SIZE) ? (User_MAX_PKT_SIZE) : (remainLength);
        error = Cmd_busTx ( i , &buffer[sendLength]);
        if (error) goto exit;

        sendLength   += i;
        remainLength -= i;
    }

    usleep(/*50000*/10000); //gap write and read ops with sufficient delay

    /** get reply frame */
    bufferLength = 5;
    error = Cmd_busRx ( bufferLength, buffer);
    if (error) goto exit;

    /** remove check-sum from reply frame */
    error = Cmd_removeChecksum ( &bufferLength, buffer);
    if (error) goto exit;

exit :
    return (error);
}

uint32_t Cmd_readTunerRegisters (
    IN  uint8_t            chip,
    IN  uint8_t            tunerAddress,
    IN  uint16_t            registerAddress,
    IN  uint8_t            registerAddressLength,
    IN  uint8_t            readBufferLength,
    IN  uint8_t*           readBuffer
) {
    uint32_t       error = Error_NO_ERROR;
    uint16_t        command;
    uint8_t        buffer[255];
    uint32_t       bufferLength;
    uint32_t       remainLength;
    uint32_t       sendLength;
    uint32_t       i, k;
    uint32_t       maxFrameSize = 255;

    if (readBufferLength == 0) goto exit;

    if ((uint32_t)(readBufferLength + 5) > User_MAX_PKT_SIZE) {
        error = Error_INVALID_DATA_LENGTH;
        goto exit;
    }

    if ((uint32_t)(readBufferLength + 5) > maxFrameSize) {
        error = Error_INVALID_DATA_LENGTH;
        goto exit;
    }

    /** add command header */
    command   = Cmd_buildCommand (Command_REG_TUNER_READ, Processor_LINK, chip);
    buffer[1] = (uint8_t) (command >> 8);
    buffer[2] = (uint8_t) command;
    buffer[3] = (uint8_t) Cmd_sequence++;
    buffer[4] = (uint8_t) readBufferLength;
    buffer[5] = (uint8_t) tunerAddress;
    buffer[6] = (uint8_t) registerAddressLength;
    buffer[7] = (uint8_t) (registerAddress >> 8);  /** Get high uint8_t of reg. address */
    buffer[8] = (uint8_t) (registerAddress);       /** Get low uint8_t of reg. address  */

    /** add frame check-sum */
    bufferLength = 9;
    error = Cmd_addChecksum ( &bufferLength, buffer);
    if (error) goto exit;

    /** send frame */
    i = 0;
    sendLength   = 0;
    remainLength = bufferLength;
    while (remainLength > 0) {
        i = (remainLength > User_MAX_PKT_SIZE) ? (User_MAX_PKT_SIZE) : (remainLength);
        error = Cmd_busTx ( i, &buffer[sendLength]);
        if (error) goto exit;

        sendLength   += i;
        remainLength -= i;
    }
	    usleep(/*50000*/10000);  // gap write and read ops with sufficient delay

    /** get reply frame */
    bufferLength = 5 + readBufferLength;
    error = Cmd_busRx ( bufferLength, buffer);
    if (error) goto exit;

    /** remove frame check-sum */
    error = Cmd_removeChecksum ( &bufferLength, buffer);
    if (error) goto exit;

    for (k = 0; k < readBufferLength; k++) {
        readBuffer[k] = buffer[k + 3];
    }

exit :
    return (error);
}


uint32_t Cmd_loadFirmware (
    IN  uint32_t           length,
    IN  uint8_t*           firmware
) {
    uint32_t       error = Error_NO_ERROR;
    uint16_t        command;
    uint32_t       loop;
    uint32_t       remain;
    uint32_t       i, j, k;
    uint8_t        buffer[255];
    uint32_t       payloadLength;
    uint32_t       bufferLength;
    uint32_t       remainLength;
    uint32_t       sendLength;
    uint32_t       maxFrameSize = 255;

    payloadLength = (maxFrameSize - 6);
    loop   = length / payloadLength;
    remain = length % payloadLength;

    k = 0;
    command = Cmd_buildCommand (Command_FW_DOWNLOAD, Processor_LINK, 0);
    for (i = 0; i < loop; i++) {
        buffer[1] = (uint8_t) (command >> 8);
        buffer[2] = (uint8_t) command;
        buffer[3] = (uint8_t) Cmd_sequence++;

        for (j = 0; j < payloadLength; j++)
            buffer[4 + j] = firmware[j + i*payloadLength];

        bufferLength = 4 + payloadLength;
        error = Cmd_addChecksum ( &bufferLength, buffer);
        if (error) goto exit;

        sendLength = 0;
        remainLength = maxFrameSize;
        while (remainLength > 0) {
            k     = (remainLength > User_MAX_PKT_SIZE) ? (User_MAX_PKT_SIZE) : (remainLength);
            error = Cmd_busTx ( k, &buffer[sendLength]);
            if (error) goto exit;

            sendLength   += k;
            remainLength -= k;
        }
    }
    usleep(/*50000*/10000);  // gap write and read ops with sufficient delay

    if (remain) {
        buffer[1] = (uint8_t) (command >> 8);
        buffer[2] = (uint8_t) command;
        buffer[3] = (uint8_t) Cmd_sequence++;

        for (j = 0; j < remain; j++)
            buffer[4 + j] = firmware[j + i*payloadLength];

        bufferLength = 4 + remain;
        error = Cmd_addChecksum ( &bufferLength, buffer);
        if (error) goto exit;

        sendLength   = 0;
        remainLength = bufferLength;
        while (remainLength > 0)
        {
            k = (remainLength > User_MAX_PKT_SIZE) ? (User_MAX_PKT_SIZE) : (remainLength);
            error = Cmd_busTx ( k, &buffer[sendLength]);
            if (error) goto exit;

            sendLength   += k;
            remainLength -= k;
        }
    }

exit :
    return (error);
}


uint32_t Cmd_reboot (
    IN  uint8_t            chip
) {
    uint32_t       error = Error_NO_ERROR;
    uint16_t        command;
    uint8_t        buffer[255];
    uint32_t       bufferLength;
    uint32_t       maxFrameSize = 255;

    command   = Cmd_buildCommand (Command_REBOOT, Processor_LINK, chip);
    buffer[1] = (uint8_t) (command >> 8);
    buffer[2] = (uint8_t) command;
    buffer[3] = (uint8_t) Cmd_sequence++;
    bufferLength = 4;
    error = Cmd_addChecksum ( &bufferLength, buffer);
    if (error) goto exit;

    error = Cmd_busTx ( bufferLength, buffer);
    if (error) goto exit;

exit :
    return (error);
}


