#ifndef Adafruit_I2CDevice_h
#define Adafruit_I2CDevice_h

// #include <Arduino.h>
// #include <Wire.h>

///< The class which defines how we will talk to this device over I2C
class Adafruit_I2CDevice {
public:
  Adafruit_I2CDevice(uint8 addr, TwoWire *theWire = &Wire);
  uint8 address(void);
  bool begin(bool addr_detect = true);
  void end(void);
  bool detected(void);

  bool read(uint8 *buffer, size_t len, bool stop = true);
  bool write(const uint8 *buffer, size_t len, bool stop = true,
             const uint8 *prefix_buffer = nullptr, size_t prefix_len = 0);
  bool write_then_read(const uint8 *write_buffer, size_t write_len,
                       uint8 *read_buffer, size_t read_len,
                       bool stop = false);
  bool setSpeed(uint32_t desiredclk);

  /*!   @brief  How many bytes we can read in a transaction
   *    @return The size of the Wire receive/transmit buffer */
  size_t maxBufferSize() { return _maxBufferSize; }

private:
  uint8 _addr;
  TwoWire *_wire;
  bool _begun;
  size_t _maxBufferSize;
  bool _read(uint8 *buffer, size_t len, bool stop);
};

#endif // Adafruit_I2CDevice_h
