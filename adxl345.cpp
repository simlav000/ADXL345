#include "adxl345.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <stdexcept>
#include <string>
#include <cstring>

// ===========================================================================
// I2CBus
// ===========================================================================

I2CBus::I2CBus(int bus_num) : fd_(-1) {
    std::string device = "/dev/i2c-" + std::to_string(bus_num);
    fd_ = open(device.c_str(), O_RDWR);
    if (fd_ < 0) {
        throw std::runtime_error("Failed to open I2C bus: " + device +
                                 " (is i2c enabled? run: sudo raspi-config)");
    }
}

I2CBus::~I2CBus() {
    if (fd_ >= 0) {
        close(fd_);
    }
}

// Use I2C_RDWR for a combined write-then-read in a single transaction,
// matching the behaviour of smbus2.read_byte_data().
uint8_t I2CBus::read_byte_data(uint8_t dev_addr, uint8_t reg_addr) {
    uint8_t reg_buf[1] = {reg_addr};
    uint8_t data_buf[1] = {0};

    struct i2c_msg msgs[2];
    msgs[0].addr  = dev_addr;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = reinterpret_cast<__u8*>(reg_buf);

    msgs[1].addr  = dev_addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = 1;
    msgs[1].buf   = reinterpret_cast<__u8*>(data_buf);

    struct i2c_rdwr_ioctl_data ioctl_data;
    ioctl_data.msgs  = msgs;
    ioctl_data.nmsgs = 2;

    if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) {
        throw std::runtime_error("I2C read_byte_data failed for reg 0x" +
                                 std::to_string(reg_addr));
    }
    return data_buf[0];
}

void I2CBus::write_byte_data(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
    uint8_t buf[2] = {reg_addr, value};

    struct i2c_msg msg;
    msg.addr  = dev_addr;
    msg.flags = 0;
    msg.len   = 2;
    msg.buf   = reinterpret_cast<__u8*>(buf);

    struct i2c_rdwr_ioctl_data ioctl_data;
    ioctl_data.msgs  = &msg;
    ioctl_data.nmsgs = 1;

    if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) {
        throw std::runtime_error("I2C write_byte_data failed for reg 0x" +
                                 std::to_string(reg_addr));
    }
}

void I2CBus::read_i2c_block_data(uint8_t dev_addr, uint8_t reg_addr,
                                  uint8_t length, uint8_t* buf) {
    uint8_t reg_buf[1] = {reg_addr};

    struct i2c_msg msgs[2];
    msgs[0].addr  = dev_addr;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = reinterpret_cast<__u8*>(reg_buf);

    msgs[1].addr  = dev_addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = length;
    msgs[1].buf   = reinterpret_cast<__u8*>(buf);

    struct i2c_rdwr_ioctl_data ioctl_data;
    ioctl_data.msgs  = msgs;
    ioctl_data.nmsgs = 2;

    if (ioctl(fd_, I2C_RDWR, &ioctl_data) < 0) {
        throw std::runtime_error("I2C read_i2c_block_data failed");
    }
}

// ===========================================================================
// OutputDataRate helpers
// ===========================================================================

double odr_to_hz(OutputDataRate odr) {
    switch (odr) {
        case OutputDataRate::ODR_3200: return 3200.0;
        case OutputDataRate::ODR_1600: return 1600.0;
        case OutputDataRate::ODR_800:  return  800.0;
        case OutputDataRate::ODR_400:  return  400.0;
        case OutputDataRate::ODR_200:  return  200.0;
        case OutputDataRate::ODR_100:  return  100.0;
        case OutputDataRate::ODR_50:   return   50.0;
        case OutputDataRate::ODR_25:   return   25.0;
        case OutputDataRate::ODR_12P5: return   12.5;
        case OutputDataRate::ODR_6P25: return    6.25;
        case OutputDataRate::ODR_3P13: return    3.13;
        case OutputDataRate::ODR_1P56: return    1.56;
        case OutputDataRate::ODR_0P78: return    0.78;
        case OutputDataRate::ODR_0P39: return    0.39;
        case OutputDataRate::ODR_0P20: return    0.20;
        case OutputDataRate::ODR_0P10: return    0.10;
        default:                       return    0.0;
    }
}

std::string odr_to_name(OutputDataRate odr) {
    switch (odr) {
        case OutputDataRate::ODR_3200: return "ODR_3200";
        case OutputDataRate::ODR_1600: return "ODR_1600";
        case OutputDataRate::ODR_800:  return "ODR_800";
        case OutputDataRate::ODR_400:  return "ODR_400";
        case OutputDataRate::ODR_200:  return "ODR_200";
        case OutputDataRate::ODR_100:  return "ODR_100";
        case OutputDataRate::ODR_50:   return "ODR_50";
        case OutputDataRate::ODR_25:   return "ODR_25";
        case OutputDataRate::ODR_12P5: return "ODR_12P5";
        case OutputDataRate::ODR_6P25: return "ODR_6P25";
        case OutputDataRate::ODR_3P13: return "ODR_3P13";
        case OutputDataRate::ODR_1P56: return "ODR_1P56";
        case OutputDataRate::ODR_0P78: return "ODR_0P78";
        case OutputDataRate::ODR_0P39: return "ODR_0P39";
        case OutputDataRate::ODR_0P20: return "ODR_0P20";
        case OutputDataRate::ODR_0P10: return "ODR_0P10";
        default:                       return "UNKNOWN";
    }
}

const std::vector<std::pair<OutputDataRate, std::string>>& odr_list() {
    static const std::vector<std::pair<OutputDataRate, std::string>> list = {
        {OutputDataRate::ODR_3200, "3200 Hz"},
        {OutputDataRate::ODR_1600, "1600 Hz"},
        {OutputDataRate::ODR_800,  "800 Hz"},
        {OutputDataRate::ODR_400,  "400 Hz"},
        {OutputDataRate::ODR_200,  "200 Hz"},
        {OutputDataRate::ODR_100,  "100 Hz"},
        {OutputDataRate::ODR_50,   "50 Hz"},
        {OutputDataRate::ODR_25,   "25 Hz"},
        {OutputDataRate::ODR_12P5, "12.5 Hz"},
        {OutputDataRate::ODR_6P25, "6.25 Hz"},
        {OutputDataRate::ODR_3P13, "3.13 Hz"},
        {OutputDataRate::ODR_1P56, "1.56 Hz"},
        {OutputDataRate::ODR_0P78, "0.78 Hz"},
        {OutputDataRate::ODR_0P39, "0.39 Hz"},
        {OutputDataRate::ODR_0P20, "0.20 Hz"},
        {OutputDataRate::ODR_0P10, "0.10 Hz"},
    };
    return list;
}

// ===========================================================================
// Range helpers
// ===========================================================================

std::string range_to_name(Range r) {
    switch (r) {
        case Range::RANGE_FULL: return "RANGE_FULL";
        case Range::RANGE_16g:  return "RANGE_16g";
        case Range::RANGE_8g:   return "RANGE_8g";
        case Range::RANGE_4g:   return "RANGE_4g";
        case Range::RANGE_2g:   return "RANGE_2g";
        default:                return "UNKNOWN";
    }
}

int range_to_g(Range r) {
    switch (r) {
        case Range::RANGE_FULL: return 16;
        case Range::RANGE_16g:  return 16;
        case Range::RANGE_8g:   return  8;
        case Range::RANGE_4g:   return  4;
        case Range::RANGE_2g:   return  2;
        default:                return  0;
    }
}

const std::vector<std::pair<Range, std::string>>& range_list() {
    static const std::vector<std::pair<Range, std::string>> list = {
        {Range::RANGE_FULL, "Full resolution"},
        {Range::RANGE_16g,  "±16g"},
        {Range::RANGE_8g,   "±8g"},
        {Range::RANGE_4g,   "±4g"},
        {Range::RANGE_2g,   "±2g"},
    };
    return list;
}

// ===========================================================================
// Register
// ===========================================================================

Register::Register(uint8_t address, bool read_only,
                   std::unordered_map<std::string, uint8_t> fields)
    : register_address_(address)
    , read_only_(read_only)
    , fields_(std::move(fields))
    , bus_(nullptr)
    , device_address_(0)
{}

void Register::bind(I2CBus* bus, uint8_t device_address) {
    bus_            = bus;
    device_address_ = device_address;
}

uint8_t Register::shift_for_mask(uint8_t mask) {
    uint8_t shift = 0;
    while (mask && !(mask & 1)) {
        ++shift;
        mask >>= 1;
    }
    return shift;
}

uint8_t Register::read(const std::string& field) const {
    if (!bus_) {
        throw std::runtime_error("Register not bound to a bus");
    }
    auto it = fields_.find(field);
    if (it == fields_.end()) {
        throw std::runtime_error("Unknown field: " + field);
    }
    uint8_t mask      = it->second;
    uint8_t shift     = shift_for_mask(mask);
    uint8_t reg_value = bus_->read_byte_data(device_address_, register_address_);
    return (reg_value & mask) >> shift;
}

void Register::write(const std::string& field, uint8_t value) {
    if (read_only_) {
        throw std::runtime_error("Cannot write to read-only register");
    }
    if (!bus_) {
        throw std::runtime_error("Register not bound to a bus");
    }
    auto it = fields_.find(field);
    if (it == fields_.end()) {
        throw std::runtime_error("Unknown field: " + field);
    }
    uint8_t mask      = it->second;
    uint8_t shift     = shift_for_mask(mask);
    uint8_t reg_value = bus_->read_byte_data(device_address_, register_address_);
    reg_value = static_cast<uint8_t>((reg_value & ~mask) | ((value << shift) & mask));
    bus_->write_byte_data(device_address_, register_address_, reg_value);
}

// ===========================================================================
// ADXL345
// ===========================================================================

ADXL345::ADXL345(uint8_t address, I2CBus* bus,
                  OutputDataRate odr, Range g_range, int watermark)
    : address(address)
    , bus(bus)
    , watermark(watermark)
    , odr(odr)
    , g_range(g_range)
    // -----------------------------------------------------------------------
    // Register definitions  (address, read_only, {field_name: bit_mask})
    // WARNING: Registers 0x01–0x1C are reserved. Do not touch!
    // -----------------------------------------------------------------------
    , bandwidth_rate(0x2C, false, {
        {"LOW_POWER", 0x10},
        {"RATE",      0x0F}
    })
    , power_control(0x2D, false, {
        {"LINK",       0x20},
        {"AUTO_SLEEP", 0x10},
        {"MEASURE",    0x08},  // 0 = standby, 1 = measurement
        {"SLEEP",      0x04},
        {"WAKEUP",     0x03}
    })
    , interrupt_source(0x30, true, {
        {"DATA_READY", 0x80},
        {"SINGLE_TAP", 0x40},
        {"DOUBLE_TAP", 0x20},
        {"ACTIVITY",   0x10},
        {"INACTIVITY", 0x08},
        {"FREE_FALL",  0x04},
        {"WATERMARK",  0x02},
        {"OVERRUN",    0x01}
    })
    , data_format(0x31, false, {
        {"SELF_TEST",  0x80},
        {"SPI",        0x40},
        {"INT_INVERT", 0x20},
        {"FULL_RES",   0x08},
        {"JUSTIFY",    0x04},
        {"RANGE",      0x03}
    })
    , fifo_status(0x39, true, {
        {"FIFO_TRIG", 0x80},
        {"ENTRIES",   0x3F}
    })
    , fifo_ctl(0x38, false, {
        {"MODE",    0xC0},
        // FIFO modes:
        //   00: Bypass  — FIFO off
        //   01: FIFO    — fills then stops
        //   10: Stream  — loops, overwrites oldest
        //   11: Trigger — holds data around trigger event
        {"TRIGGER", 0x20},
        {"SAMPLES", 0x1F}
        // D4–D0 set the watermark threshold (1–31).
        // The WATERMARK interrupt fires when FIFO reaches this level.
    })
    // scale_ is last in declaration order (private, after the Registers).
    // Default: full-resolution mode → 3.9 mg/LSB; updated in apply_range().
    , scale_(0.0039f)
{
    bind_registers();
    apply_watermark();
    apply_odr();
    apply_range();
}

void ADXL345::bind_registers() {
    bandwidth_rate.bind(bus, address);
    power_control.bind(bus, address);
    interrupt_source.bind(bus, address);
    data_format.bind(bus, address);
    fifo_status.bind(bus, address);
    fifo_ctl.bind(bus, address);
}

void ADXL345::apply_watermark() {
    fifo_ctl.write("SAMPLES", static_cast<uint8_t>(watermark));
}

void ADXL345::apply_odr() {
    bandwidth_rate.write("RATE", static_cast<uint8_t>(odr));
}

void ADXL345::apply_range() {
    if (g_range == Range::RANGE_FULL) {
        data_format.write("FULL_RES", 0x01);
        scale_ = 0.0039f;  // 3.9 mg/LSB in full-resolution mode
    } else {
        data_format.write("FULL_RES", 0x00);
        data_format.write("RANGE", static_cast<uint8_t>(g_range));
        // 10-bit mode: span = 2 * 2^(range_bits+1) g, 1024 counts per span
        uint8_t range_bits = static_cast<uint8_t>(g_range);
        float   span_g     = 2.0f * static_cast<float>(1 << (range_bits + 1));
        scale_             = span_g / 1024.0f;
    }
}

void ADXL345::set_watermark(int wm) {
    watermark = wm;
    apply_watermark();
}

void ADXL345::set_odr(OutputDataRate new_odr) {
    odr = new_odr;
    apply_odr();
}

int16_t ADXL345::to_signed_16bit(uint16_t value) const {
    // Casting uint16_t to int16_t performs the two's-complement sign extension
    // we need.  The behaviour is well-defined in C++20 and essentially
    // universally safe on any two's-complement platform (all modern hardware).
    return static_cast<int16_t>(value);
}

std::tuple<float, float, float> ADXL345::get_accel() {
    static const uint8_t REG_DATAX0 = 0x32;
    uint8_t data[6];
    bus->read_i2c_block_data(address, REG_DATAX0, 6, data);

    int16_t x = to_signed_16bit(static_cast<uint16_t>((data[1] << 8) | data[0]));
    int16_t y = to_signed_16bit(static_cast<uint16_t>((data[3] << 8) | data[2]));
    int16_t z = to_signed_16bit(static_cast<uint16_t>((data[5] << 8) | data[4]));

    // scale_ is cached in apply_range(), so no extra I2C read is needed here.
    return {x * scale_, y * scale_, z * scale_};
}
