// spi_protocol.cpp
#include "internal/spi_protocol.h"

namespace spi_protocol {

CRC8::CRC8(uint8_t polynomial) { BuildTable(polynomial); }

void CRC8::BuildTable(uint8_t polynomial) {
  for (int i = 0; i < 256; i++) {
    uint8_t crc = i;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ polynomial;
      } else {
        crc <<= 1;
      }
    }
    table_[i] = crc;
  }
}

uint8_t CRC8::Calculate(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc = table_[crc ^ data[i]];
  }
  return crc;
}

}  // namespace spi_protocol