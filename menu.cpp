/**
 * menu.cpp — interactive CLI menu for the ADXL345 C++ driver.
 *
 * Mirrors menu.py: lets the user choose OutputDataRate and Range,
 * confirm settings, then runs the measurement routine.
 *
 * Build:   make
 * Run:     sudo ./adxl345_measure
 *          (root or i2c group membership required for /dev/i2c-*)
 */

#include "adxl345.hpp"
#include "measure.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <stdexcept>

// ---------------------------------------------------------------------------
// select_odr — print a numbered list and return the user's choice.
// ---------------------------------------------------------------------------
static OutputDataRate select_odr(OutputDataRate default_odr) {
    const auto& list = odr_list();

    std::cout << "\n Select OutputDataRate"
              << " (press Enter for default: "
              << odr_to_name(default_odr) << ")\n";

    for (size_t i = 0; i < list.size(); ++i) {
        std::cout << i << ": " << list[i].second << '\n';
    }

    std::string line;
    std::cout << "> ";
    std::getline(std::cin, line);

    if (line.empty()) {
        return default_odr;
    }
    try {
        size_t idx = static_cast<size_t>(std::stoul(line));
        if (idx < list.size()) {
            return list[idx].first;
        }
    } catch (...) {}

    std::cout << "Invalid selection. Using default.\n";
    return default_odr;
}

// ---------------------------------------------------------------------------
// select_range — print a numbered list and return the user's choice.
// ---------------------------------------------------------------------------
static Range select_range(Range default_range) {
    const auto& list = range_list();

    std::cout << "\n Select Range"
              << " (press Enter for default: "
              << range_to_name(default_range) << ")\n";

    for (size_t i = 0; i < list.size(); ++i) {
        std::cout << i << ": " << list[i].second << '\n';
    }

    std::string line;
    std::cout << "> ";
    std::getline(std::cin, line);

    if (line.empty()) {
        return default_range;
    }
    try {
        size_t idx = static_cast<size_t>(std::stoul(line));
        if (idx < list.size()) {
            return list[idx].first;
        }
    } catch (...) {}

    std::cout << "Invalid selection. Using default.\n";
    return default_range;
}

// ---------------------------------------------------------------------------
// confirm_settings
// ---------------------------------------------------------------------------
static bool confirm_settings(OutputDataRate odr, Range g_range) {
    std::cout << "\nSelected settings:\n";
    std::cout << "ODR   : " << odr_to_name(odr)
              << " (" << odr_to_hz(odr) << " Hz)\n";
    std::cout << "Range : " << range_to_name(g_range) << '\n';

    std::cout << "Start measurement? (y/n): ";
    std::string line;
    std::getline(std::cin, line);
    return (!line.empty() && (line[0] == 'y' || line[0] == 'Y'));
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main() {
    constexpr int     BUS_NUM     = 1;
    constexpr uint8_t DEVICE_ADDR = 0x1D;

    OutputDataRate default_odr   = OutputDataRate::ODR_100;
    Range          default_range = Range::RANGE_FULL;

    OutputDataRate odr     = select_odr(default_odr);
    Range          g_range = select_range(default_range);

    std::cout << "Stationary? (y/n): ";
    std::string stat_line;
    std::getline(std::cin, stat_line);
    bool is_stationary = (!stat_line.empty() &&
                          (stat_line[0] == 'y' || stat_line[0] == 'Y'));

    if (!confirm_settings(odr, g_range)) {
        std::cout << "Startup aborted.\n";
        return 0;
    }

    try {
        I2CBus  bus(BUS_NUM);
        ADXL345 sensor(DEVICE_ADDR, &bus, odr, g_range, /*watermark=*/16);
        measure(sensor, is_stationary);
    } catch (const std::exception& e) {
        std::cerr << "\nError: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
