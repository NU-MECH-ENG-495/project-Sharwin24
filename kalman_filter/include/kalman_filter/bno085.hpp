#pragma once

class BNO085_IMU {

  public:
    BNO085_IMU();
    ~BNO085_IMU();

    void initialize(const int i2c_address);

};