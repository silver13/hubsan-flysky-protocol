#include <stdint.h>
#include "lib_digitalio.h"

void lib_soft_3_wire_spi_init(uint8_t SDIO_portandpinnumber, uint8_t SCK_portandpinnumber, uint8_t SCS_portandpinnumber );
void lib_soft_3_wire_spi_setCS(uint8_t state);
void lib_soft_3_wire_spi_write(uint8_t data);
uint8_t lib_soft_3_wire_spi_read(void);
