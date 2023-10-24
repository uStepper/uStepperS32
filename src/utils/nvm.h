#ifndef __NVM_H
#define __NVM_H

#include <stdint.h>
#include <stm32f4xx_hal_flash.h>

class Nvm
{
  public:
	Nvm(uint32_t size);
	bool readNvm(uint32_t address, uint32_t size, uint8_t *data);
	bool readNvm(uint32_t address, uint32_t size, uint16_t *data);
	bool readNvm(uint32_t address, uint32_t size, uint32_t *data);
	bool writeNvm(uint32_t address, uint32_t size, uint8_t *data);
	bool writeNvm(uint32_t address, uint32_t size, uint16_t *data);
	bool writeNvm(uint32_t address, uint32_t size, uint32_t *data);
  private:
	const uint32_t nvmAddress = 0x08000000;
	uint32_t size;
};

#endif