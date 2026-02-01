#include <cstdio>
#include <cstdint>

void analyze_can_id(uint32_t can_id, const uint8_t* data) {
    printf("\n========== CAN ID ANALYSIS ==========\n");
    printf("Received CAN ID: 0x%08X\n\n", can_id);
    
    // Print as bytes
    printf("As bytes (little-endian layout in memory):\n");
    uint8_t* bytes = (uint8_t*)&can_id;
    printf("  [0] = 0x%02X\n", bytes[0]);
    printf("  [1] = 0x%02X\n", bytes[1]);
    printf("  [2] = 0x%02X\n", bytes[2]);
    printf("  [3] = 0x%02X\n", bytes[3]);
    
    // Try different extraction methods
    printf("\nExtraction methods:\n");
    
    uint8_t method1 = can_id & 0xFF;
    uint8_t method2 = (can_id >> 8) & 0xFF;
    uint8_t method3 = (can_id >> 16) & 0xFF;
    uint8_t method4 = (can_id >> 24) & 0xFF;
    
    printf("  Bits 0-7   (& 0xFF):          0x%02X\n", method1);
    printf("  Bits 8-15  ((>> 8) & 0xFF):   0x%02X  <-- This works for you\n", method2);
    printf("  Bits 16-23 ((>> 16) & 0xFF):  0x%02X\n", method3);
    printf("  Bits 24-31 ((>> 24) & 0xFF):  0x%02X\n", method4);
    
    // Parse as big-endian vs little-endian
    printf("\nIf interpreted as big-endian (network order):\n");
    printf("  Type:      0x%02X (bits 24-31)\n", method4);
    printf("  Reserved:  0x%02X (bits 16-23)\n", method3);
    printf("  Master:    0x%02X (bits 8-15)\n", method2);
    printf("  Motor ID:  0x%02X (bits 0-7)\n", method1);
    
    printf("\nIf interpreted as little-endian (Intel order):\n");
    printf("  Motor ID:  0x%02X (bits 24-31)\n", method4);
    printf("  Master:    0x%02X (bits 16-23)\n", method3);
    printf("  Reserved:  0x%02X (bits 8-15)\n", method2);
    printf("  Type:      0x%02X (bits 0-7)\n", method1);
    
    // Analyze data payload
    printf("\nData payload:\n");
    printf("  ");
    for (int i = 0; i < 8; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    
    // Try to parse as feedback (assuming RobStride protocol)
    printf("\nTrying to parse as RobStride feedback:\n");
    uint16_t pos_raw = (data[0] << 8) | data[1];
    uint16_t vel_raw = (data[2] << 8) | data[3];
    uint16_t torq_raw = (data[4] << 8) | data[5];
    uint16_t temp_raw = (data[6] << 8) | data[7];
    
    float position = ((float)pos_raw / 32767.0f - 1.0f) * 12.5f;
    float velocity = ((float)vel_raw / 32767.0f - 1.0f) * 44.0f;
    float torque = ((float)torq_raw / 32767.0f - 1.0f) * 14.0f;
    float temp = (float)temp_raw * 0.1f;
    
    printf("  Position: %.3f rad (raw: 0x%04X)\n", position, pos_raw);
    printf("  Velocity: %.3f rad/s (raw: 0x%04X)\n", velocity, vel_raw);
    printf("  Torque:   %.3f Nm (raw: 0x%04X)\n", torque, torq_raw);
    printf("  Temp:     %.1f C (raw: 0x%04X)\n", temp, temp_raw);
    
    printf("=====================================\n\n");
}