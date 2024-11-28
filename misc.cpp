#include "Arduino.h"
#include <SPI.h>
#include "no_os_spi.h"
#include "no_os_delay.h"
#include "no_os_alloc.h"
#include "misc.h"
#include "ad7293.h"

SPISettings parSPI(2000000, MSBFIRST, SPI_MODE0);

/* Generate microseconds delay. */
void no_os_udelay(uint32_t usecs){
  delayMicroseconds(usecs);
}

/* Generate miliseconds delay. */
void no_os_mdelay(uint32_t msecs){
  delay(msecs);
}

void *no_os_calloc(size_t nitems, size_t size){
  return calloc(nitems, size);
}

/* Deallocate memory previously allocated by a call to no_os_calloc or
 * no_os_malloc */
void no_os_free(void *ptr){
  free(ptr);
}

void ad_cs(int state){
	digitalWrite(AD_SPI_CS, state);
}

void ad_reset(int state){
	digitalWrite(AD_RST, state);
}

int32_t no_os_spi_init(struct no_os_spi_desc **desc, const struct no_os_spi_init_param *param){
    delay(500);
    pinMode(AD_SPI_CS, OUTPUT);
    pinMode(AD_RST, OUTPUT);

    SPI1.setSCK(AD_SPI_SCK);
    SPI1.setTX(AD_SPI_MOSI);
    SPI1.setRX(AD_SPI_MISO);
    SPI1.begin(false); //spi for power det adc
    return 0;
}

int32_t no_os_spi_remove(struct no_os_spi_desc *desc){
  return 0;
}

int32_t no_os_spi_write_and_read(struct no_os_spi_desc *desc, uint8_t *data, uint16_t bytes_number){

  SPI1.beginTransaction(parSPI);
  ad_cs(0);
  SPI1.transfer(data, data, bytes_number);
  ad_cs(1);
  SPI1.endTransaction();
  return 0;
}