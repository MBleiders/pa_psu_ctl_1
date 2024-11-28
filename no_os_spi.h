#ifndef _NO_OS_SPI_H_
#define _NO_OS_SPI_H_

struct no_os_spi_init_param {
  //dummy
};

struct no_os_spi_desc {
//dummy
};

int32_t no_os_spi_init(struct no_os_spi_desc **desc,
		       const struct no_os_spi_init_param *param);

int32_t no_os_spi_remove(struct no_os_spi_desc *desc);

int32_t no_os_spi_write_and_read(struct no_os_spi_desc *desc,
				 uint8_t *data,
				 uint16_t bytes_number);


#endif