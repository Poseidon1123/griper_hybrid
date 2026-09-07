#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stdint.h>

typedef void (*SerialProtocolTxFn)(const char *text);

void SerialProtocol_Init(SerialProtocolTxFn tx_fn);
void SerialProtocol_ProcessLine(const char *line, uint32_t now_ms);
void SerialProtocol_SendStatus(void);

#endif
