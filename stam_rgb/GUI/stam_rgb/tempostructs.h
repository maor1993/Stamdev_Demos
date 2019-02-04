#ifndef TEMPOSTRUCTS_H
#define TEMPOSTRUCTS_H
#include <stdint.h>

namespace icd{

#define ICD_PREMBLE  0x9a5bc0dd
#define USB_MAX_MSG_SIZE 64

#define ICD_HEADER_SIZE 7

#define USB_REQ_MASTER_SEND 0


typedef struct __attribute__ ((packed))
{
    uint8_t nPremble[4];
    uint8_t nRed;
    uint8_t nBlue;
    uint8_t nGreen;

}icd_header;

}


#endif // TEMPOSTRUCTS_H
