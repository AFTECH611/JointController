// spi_protocol.h
#pragma once
#include <cstddef>
#include <cstdint>

namespace spi_protocol {

// Frame header/footer
constexpr uint8_t FRAME_HEADER[2] = {0x45, 0x54};  // "ET"
constexpr uint8_t FRAME_FOOTER[2] = {0x0D, 0x0A};  // CRLF

constexpr size_t SPI_FRAME_SIZE = 21;  // Fixed frame size

// CAN frame structure (extends to 29-bit CAN ID + 8 bytes data)
struct __attribute__((packed)) SpiCanFrame {
  uint8_t header[2];    // 0x45 0x54
  uint32_t can_id;      // 29-bit CAN ID (big endian)
  uint8_t data[8];      // CAN data
  uint8_t footer[2];    // 0x0D 0x0A
  uint8_t reserved[4];  // Reserved bytes
  uint8_t crc;          // CRC8
};

static_assert(sizeof(SpiCanFrame) == SPI_FRAME_SIZE, "Frame size mismatch");

// CRC8 calculator
class CRC8 {
 public:
  CRC8(uint8_t polynomial = 0x07);
  uint8_t Calculate(const uint8_t* data, size_t len);

 private:
  uint8_t table_[256];
  void BuildTable(uint8_t polynomial);
};

}  // namespace spi_protocol