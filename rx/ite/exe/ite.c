#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <libusb.h>
#include "usb_rx.h"

extern struct libusb_device_handle *devh;
 static uint8_t Cmd_sequence = 0;

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

		r = libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
         if (r == (sizeof(*acs)+(acs->dcnt-1))) goto exit;
         else error = -r; // error # defined in libusb, liyenho

        short_sleep (0.1);
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

        	r =	libusb_control_transfer(devh,
						CTRL_OUT, USB_RQ,
						USB_HOST_MSG_TX_VAL,
						USB_HOST_MSG_IDX,
						acs, sizeof(*acs)+(acs->dcnt-1), 0);
				short_sleep(0.1); 	// validate echo after 0.1 sec

         if (r != (sizeof(*acs)+(acs->dcnt-1)))
         	continue ;

	  	 	while(0==libusb_control_transfer(devh,
					  	CTRL_IN, USB_RQ,
					  	USB_HOST_MSG_RX_VAL,
					  	USB_HOST_MSG_IDX,
					  	acs, sizeof(*acs)+(acs->dcnt-1), 0))
				short_sleep(0.0005);
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

    usleep(100000);  // gap write and read ops with sufficient delay

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

    usleep(100000);  // gap write and read ops with sufficient delay

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

    usleep(100000);  // gap write and read ops with sufficient delay
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

uint32_t Standard_readRegisters (
	uint8_t            chip,
	Processor       processor,
	uint32_t           registerAddress,
	uint8_t            bufferLength,
	uint8_t*           buffer
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t registerAddressLength;

	if (processor == Processor_LINK) {
		if (registerAddress > 0x000000FF) {
			registerAddressLength = 2;
		} else {
			registerAddressLength = 1;
		}
	} else {
		registerAddressLength = 2;
	}
	/*if (Cmd_readRegisters != NULL)*/ {
		error = Cmd_readRegisters (chip, processor, registerAddress, registerAddressLength, bufferLength, buffer);
	}

	return (error);
}
uint32_t Standard_writeRegisters (
	uint8_t            chip,
	Processor       processor,
	uint32_t           registerAddress,
	uint8_t            bufferLength,
	uint8_t*           buffer
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t registerAddressLength;

	if (processor == Processor_LINK) {
		if (registerAddress > 0x000000FF) {
			registerAddressLength = 2;
		} else {
			registerAddressLength = 1;
		}
	} else {
			registerAddressLength = 2;
	}
	/*if (Cmd_writeRegisters != NULL)*/ {
		error = Cmd_writeRegisters (chip, processor, registerAddress, registerAddressLength, bufferLength, buffer);
	}

	return (error);
}

uint32_t Standard_getFirmwareVersion (
	Processor       processor,
	uint32_t*          version
) {
	uint32_t error = Error_NO_ERROR;

	uint8_t writeBuffer[1] = {0,};
	uint8_t readBuffer[4] = {0,};
	uint8_t value = 0;

	/** Check chip version */
	error = Standard_readRegisters (0, Processor_LINK, /*chip_version_7_0*/0x0/*???*/, 1, &value);
	if (error) goto exit;

	if (value == 0xF8 || User_MAX_PKT_SIZE > 9) {
		/** Old version */
		writeBuffer[0] = 1;
		error = Cmd_sendCommand (Command_QUERYINFO, 0, processor, 1, writeBuffer, 4, readBuffer);
		if (error) goto exit;
	} else {
		/** New version */
		error = Cmd_sendCommand (Command_FW_DOWNLOAD_END, 0, Processor_LINK, 0, NULL, 0, NULL);
		if (error == 0x01000009) { /* Boot code*/
			readBuffer[0] = readBuffer[1] = readBuffer[2] = readBuffer[3] = 0;
			error = 0;
		} else if (error == 0x010000FA) { /* Firmware code*/
			if (processor == Processor_LINK)
			{
				error = Standard_readRegisters (0, Processor_LINK, /*link_version_31_24*/0x0/*???*/, 1, readBuffer);
				if (error) goto exit;

				error = Standard_readRegisters (0, Processor_LINK, /*link_version_23_16*/0x0/*???*/, 1, readBuffer + 1);
				if (error) goto exit;

				error = Standard_readRegisters (0, Processor_LINK, /*link_version_15_8*/0x0/*???*/, 1, readBuffer + 2);
				if (error) goto exit;

				error = Standard_readRegisters (0, Processor_LINK, /*link_version_7_0*/0x0/*???*/, 1, readBuffer + 3);
				if (error) goto exit;
			}
			else
			{
				error = Standard_readRegisters (0, Processor_OFDM, /*link_version_31_24*/0x0/*???*/, 1, readBuffer);
				if (error) goto exit;

				error = Standard_readRegisters (0, Processor_OFDM, /*link_version_23_16*/0x0/*???*/, 1, readBuffer + 1);
				if (error) goto exit;

				error = Standard_readRegisters (0, Processor_OFDM, /*link_version_15_8*/0x0/*???*/, 1, readBuffer + 2);
				if (error) goto exit;

				error = Standard_readRegisters (0, Processor_OFDM, /*link_version_7_0*/0x0/*???*/, 1, readBuffer + 3);
				if (error) goto exit;
			}
		} else /* error */
			goto exit;
	}

	*version = (uint32_t) (((uint32_t) readBuffer[0] << 24) + ((uint32_t) readBuffer[1] << 16) + ((uint32_t) readBuffer[2] << 8) + (uint32_t) readBuffer[3]);

exit :

	return (error);
}

