#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/time.h"

// Include SH2 library files
extern "C" {
    #include "../sh2/sh2.h"
    #include "../sh2/sh2_SensorValue.h"
    #include "../sh2/sh2_hal.h"
    #include "../sh2/sh2_err.h"
}

#define I2C_PORT   i2c0
#define I2C_SDA    0    // GP0, physical pin 2
#define I2C_SCL    1    // GP1, physical pin 4
#define I2C_BAUD   400000

static const uint8_t BNO085_ADDR = 0x4A;

// Global variables for sensor data
static sh2_SensorValue_t sensorValue;
static bool newLinearAccelData = false;
static bool newRotationData = false;

// HAL implementation for Raspberry Pi Pico I2C
static int hal_open(sh2_Hal_t *self);
static void hal_close(sh2_Hal_t *self);
static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static uint32_t hal_getTimeUs(sh2_Hal_t *self);

// HAL structure
static sh2_Hal_t hal = {
    .open = hal_open,
    .close = hal_close,
    .read = hal_read,
    .write = hal_write,
    .getTimeUs = hal_getTimeUs
};

// No buffer needed - we read directly into the SHTP buffer

// HAL implementation functions
static int hal_open(sh2_Hal_t *self) {
    printf("HAL: Opening I2C interface\n");
    
    // I2C init with conservative settings
    i2c_init(I2C_PORT, 100000); // Start with 100kHz (slower, more reliable)
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    printf("I2C initialized at 100kHz\n");
    
    // Wait for device to power up
    sleep_ms(500);
    
    // Try to detect the device at the expected address
    printf("Scanning for BNO085 at address 0x%02X...\n", BNO085_ADDR);
    uint8_t dummy;
    int result = i2c_read_blocking(I2C_PORT, BNO085_ADDR, &dummy, 1, false);
    if (result == 1) {
        printf("BNO085 detected!\n");
    } else {
        printf("BNO085 not responding (result: %d)\n", result);
    }
    
    // Additional wait for device stability
    sleep_ms(200);
    
    return 0; // Success
}

static void hal_close(sh2_Hal_t *self) {
    printf("HAL: Closing I2C interface\n");
    i2c_deinit(I2C_PORT);
}

static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    // BNO085 I2C CRITICAL: Read entire SHTP packet in ONE single transaction
    // The device has data ready for only ONE read operation
    
    static uint32_t lastReadTime = 0;
    uint32_t now = time_us_32();
    
    // Rate limit reads to avoid overwhelming the device (minimum 10ms between reads)
    if ((now - lastReadTime) < 10000) {
        return 0;
    }
    lastReadTime = now;
    
    // Read a reasonable chunk size in ONE transaction
    // Based on the pattern, we're seeing 8-byte packets consistently
    uint8_t tempBuffer[32];  // Read up to 32 bytes in one go
    int result = i2c_read_blocking(I2C_PORT, BNO085_ADDR, tempBuffer, sizeof(tempBuffer), false);
    
    if (result <= 0) {
        return 0; // No data available
    }
    
    // Parse SHTP header from what we actually received
    if (result < 4) {
        return 0; // Not enough data for SHTP header
    }
    
    uint16_t packetLength = (tempBuffer[1] << 8) | tempBuffer[0];
    packetLength &= 0x7FFF; // Clear continuation bit
    
    // Debug: Print what we actually received
    static uint32_t debugCount = 0;
    if ((debugCount++ % 20) == 0) {  // Print every 20th packet
        printf("I2C Read: got %d bytes, header: %02X %02X %02X %02X (claimed len=%d)\n", 
               result, tempBuffer[0], tempBuffer[1], tempBuffer[2], tempBuffer[3], packetLength);
    }
    
    // Check for "no data" indicators
    if (packetLength == 0 || packetLength == 0xFFFF || 
        (tempBuffer[0] == 0xFF && tempBuffer[1] == 0xFF)) {
        return 0; // No real data
    }
    
    // Use the actual bytes we received, not the claimed packet length
    // If the packet claims to be longer than what we read, use what we actually got
    int actualLength = (packetLength <= (uint16_t)result) ? packetLength : result;
    
    // Validate actual length
    if (actualLength < 4 || actualLength > len) {
        return 0; // Invalid length
    }
    
    // Copy the data we actually received
    memcpy(pBuffer, tempBuffer, actualLength);
    
    *t_us = now;
    return actualLength;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    if (len == 0) return 0;
    
    int result = i2c_write_blocking(I2C_PORT, BNO085_ADDR, pBuffer, len, false);
    
    if (result == (int)len) {
        return len;
    }
    
    return 0; // Error
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
    return time_us_32();
}

// Event callback for SH2 events
static void eventHandler(void *cookie, sh2_AsyncEvent_t *pEvent) {
    // Handle async events if needed
    printf("SH2 Event: %lu\n", pEvent->eventId);
    
    // Add specific handling for different event types
    switch (pEvent->eventId) {
        case 1: // SHTP_SHORT_FRAGMENT
            printf("  -> SHTP Short Fragment Error\n");
            break;
        case 2: // SHTP_TOO_LARGE_PAYLOADS
            printf("  -> SHTP Payload Too Large Error\n");
            break;
        case 3: // SHTP_BAD_RX_CHAN
            printf("  -> SHTP Bad RX Channel Error\n");
            break;
        default:
            printf("  -> Unknown SHTP Event\n");
            break;
    }
}

// Sensor callback for receiving sensor data
static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent) {
    int rc = sh2_decodeSensorEvent(&sensorValue, pEvent);
    if (rc != SH2_OK) {
        printf("Error decoding sensor event: %d\n", rc);
        return;
    }
    
    // Handle different sensor types
    switch (sensorValue.sensorId) {
        case SH2_LINEAR_ACCELERATION:
            printf("Linear Accel: %.3f, %.3f, %.3f m/s²\n",
                   sensorValue.un.linearAcceleration.x,
                   sensorValue.un.linearAcceleration.y,
                   sensorValue.un.linearAcceleration.z);
            newLinearAccelData = true;
            break;
            
        case SH2_ROTATION_VECTOR:
            printf("Quaternion: real=%.3f, i=%.3f, j=%.3f, k=%.3f (accuracy=%.3f)\n",
                   sensorValue.un.rotationVector.real,
                   sensorValue.un.rotationVector.i,
                   sensorValue.un.rotationVector.j,
                   sensorValue.un.rotationVector.k,
                   sensorValue.un.rotationVector.accuracy);
            newRotationData = true;
            break;
            
        default:
            printf("Sensor ID: 0x%02x\n", sensorValue.sensorId);
            break;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(5000);
    printf("Starting BNO085 with SH2 Library...\n");
    printf("I2C configured: SDA=GP%d, SCL=GP%d, Rate=100000Hz (conservative)\n", I2C_SDA, I2C_SCL);

    // Open SH2 session
    printf("Opening SH2 session...\n");
    int status = sh2_open(&hal, eventHandler, NULL);
    if (status != SH2_OK) {
        printf("Error opening SH2: %d\n", status);
        return -1;
    }
    
    printf("SH2 session opened successfully\n");
    
    // Wait a bit for the device to stabilize
    sleep_ms(100);
    
    // Register sensor callback
    printf("Registering sensor callback...\n");
    sh2_setSensorCallback(sensorHandler, NULL);
    
    // Get product ID to verify communication
    printf("Getting product ID...\n");
    sh2_ProductIds_t prodIds;
    status = sh2_getProdIds(&prodIds);
    if (status == SH2_OK) {
        printf("Product ID read successfully. Entries: %d\n", prodIds.numEntries);
        if (prodIds.numEntries > 0) {
            printf("SW Version: %d.%d.%d, Part: 0x%08lX\n", 
                   prodIds.entry[0].swVersionMajor,
                   prodIds.entry[0].swVersionMinor,
                   prodIds.entry[0].swVersionPatch,
                   prodIds.entry[0].swPartNumber);
        }
    } else {
        printf("Error reading product ID: %d\n", status);
    }
    
    // Configure sensors with a slower update rate initially
    sh2_SensorConfig_t config;
    memset(&config, 0, sizeof(config));
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.sniffEnabled = false;
    config.changeSensitivity = 0;
    config.reportInterval_us = 200000; // 5 Hz (200ms) - slower for initial testing
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;
    
    printf("Enabling Linear Acceleration sensor (5Hz)...\n");
    status = sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &config);
    if (status != SH2_OK) {
        printf("Error enabling linear acceleration: %d\n", status);
    } else {
        printf("Linear acceleration enabled successfully\n");
    }
    
    sleep_ms(100); // Wait between sensor enables
    
    printf("Enabling Rotation Vector sensor (5Hz)...\n");
    status = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);
    if (status != SH2_OK) {
        printf("Error enabling rotation vector: %d\n", status);
    } else {
        printf("Rotation vector enabled successfully\n");
    }
    
    printf("Sensors configured. Starting main loop...\n");
    printf("Waiting for sensor data...\n");
    
    uint32_t lastPrint = time_us_32();
    
    // Main service loop
    while (true) {
        sh2_service();
        
        // Print status every 5 seconds if no data received
        uint32_t now = time_us_32();
        if ((now - lastPrint) > 5000000) { // 5 seconds
            printf("Still running... (waiting for sensor data)\n");
            lastPrint = now;
        }
        
        sleep_ms(50); // Service every 50ms - much slower to reduce I2C traffic
    }
    
    // Clean up (never reached in this example)
    sh2_close();
    return 0;
}