#ifndef HW_RIO_RIO_H
#define HW_RIO_RIO_H

typedef struct RIOTx {
    uint8_t dest; /* device ID */
    uint64_t payload;
} RIOTx;

#endif // HW_RIO_RIO_H
