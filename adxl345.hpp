#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <tuple>
#include <vector>
#include <utility>

// ---------------------------------------------------------------------------
// I2CBus — thin wrapper around Linux's i2c-dev kernel interface.
// Replaces smbus2.SMBus from the Python implementation.
// Requires /dev/i2c-N to exist (enable via raspi-config or `dtparam=i2c_arm=on`).
// ---------------------------------------------------------------------------
class I2CBus {
public:
    explicit I2CBus(int bus_num);
    ~I2CBus();

    uint8_t read_byte_data(uint8_t dev_addr, uint8_t reg_addr);
    void    write_byte_data(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);
    void    read_i2c_block_data(uint8_t dev_addr, uint8_t reg_addr,
                                uint8_t length, uint8_t* buf);

private:
    int fd_;
};

// ---------------------------------------------------------------------------
// OutputDataRate — allowable ODR values for the ADXL345 BW_RATE register.
// ---------------------------------------------------------------------------
enum class OutputDataRate : uint8_t {
    ODR_3200 = 0x0F,
    ODR_1600 = 0x0E,
    ODR_800  = 0x0D,
    ODR_400  = 0x0C,
    ODR_200  = 0x0B,
    ODR_100  = 0x0A,  // Default
    ODR_50   = 0x09,
    ODR_25   = 0x08,
    ODR_12P5 = 0x07,
    ODR_6P25 = 0x06,
    ODR_3P13 = 0x05,
    ODR_1P56 = 0x04,
    ODR_0P78 = 0x03,
    ODR_0P39 = 0x02,
    ODR_0P20 = 0x01,
    ODR_0P10 = 0x00,
};

double      odr_to_hz(OutputDataRate odr);
std::string odr_to_name(OutputDataRate odr);
const std::vector<std::pair<OutputDataRate, std::string>>& odr_list();

// ---------------------------------------------------------------------------
// Range — allowable acceleration range / resolution settings.
// RANGE_FULL enables the FULL_RES bit (4 mg/LSB across all ranges).
// ---------------------------------------------------------------------------
enum class Range : uint8_t {
    RANGE_2g   = 0b00,
    RANGE_4g   = 0b01,
    RANGE_8g   = 0b10,
    RANGE_16g  = 0b11,
    RANGE_FULL = 0xFF,  // Special: sets FULL_RES=1 in DATA_FORMAT
};

std::string range_to_name(Range r);
int         range_to_g(Range r);
const std::vector<std::pair<Range, std::string>>& range_list();

// ---------------------------------------------------------------------------
// Register — models a single ADXL345 register with named bit-fields.
// Mirrors the Python Register class (read-modify-write semantics).
// ---------------------------------------------------------------------------
class Register {
public:
    Register(uint8_t address, bool read_only,
             std::unordered_map<std::string, uint8_t> fields);

    void    bind(I2CBus* bus, uint8_t device_address);
    uint8_t read(const std::string& field) const;
    void    write(const std::string& field, uint8_t value);

private:
    uint8_t  register_address_;
    bool     read_only_;
    std::unordered_map<std::string, uint8_t> fields_;
    I2CBus*  bus_;
    uint8_t  device_address_;

    static uint8_t shift_for_mask(uint8_t mask);
};

// ---------------------------------------------------------------------------
// ADXL345 — driver for the Analog Devices ADXL345 3-axis accelerometer.
// Communicates over I2C using I2CBus.  Mirrors adxl345.py.
//
// Key performance improvement over Python: the scale factor used in get_accel()
// is cached when the range is set, eliminating one I2C read-per-sample.
// ---------------------------------------------------------------------------
class ADXL345 {
public:
    ADXL345(uint8_t address,
            I2CBus* bus,
            OutputDataRate odr       = OutputDataRate::ODR_100,
            Range          g_range   = Range::RANGE_FULL,
            int            watermark = 28);

    std::tuple<float, float, float> get_accel();

    void set_watermark(int watermark);
    void set_odr(OutputDataRate odr);

    // Public fields (mirrors Python attribute access pattern)
    uint8_t        address;
    I2CBus*        bus;
    int            watermark;
    OutputDataRate odr;
    Range          g_range;

    Register bandwidth_rate;
    Register power_control;
    Register interrupt_source;
    Register data_format;
    Register fifo_status;
    Register fifo_ctl;

private:
    // Cached g-per-LSB scale factor; updated in apply_range() so that
    // get_accel() does not need an extra I2C read per sample.
    float   scale_;

    void    bind_registers();
    void    apply_watermark();
    void    apply_odr();
    void    apply_range();
    int16_t to_signed_16bit(uint16_t value) const;
};
