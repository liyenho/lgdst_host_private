#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <libusb.h>
#include "usb_rx.h"
#include "user.h"

/**
 * Variable of critical section
 */


uint32_t User_delay (
	
      uint32_t           dwMs
) {
      usleep(dwMs*1000);
    return (Error_NO_ERROR);
}


uint32_t User_enterCriticalSection (
) {
   
    return (Error_NO_ERROR);
}


uint32_t User_leaveCriticalSection (
) {
   
    return (Error_NO_ERROR);
}


uint32_t User_mpegConfig (
) {
    
    return (Error_NO_ERROR);
}


uint32_t User_busTx (
      uint32_t bufferLength,
      uint8_t* buffer
) {
   
     
    return (Error_NO_ERROR);
}


uint32_t User_busRx (
      uint32_t           bufferLength,
     uint8_t*           buffer
) {
    return (Error_NO_ERROR);

}

