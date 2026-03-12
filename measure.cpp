#include "measure.hpp"
#include "adxl345.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>
#include <stdexcept>
#include <algorithm>

// ---------------------------------------------------------------------------
// init_adxl
// ---------------------------------------------------------------------------
void init_adxl(ADXL345& adxl) {
    std::cout << "\n=== ADXL345 INITIALIZATION ===\n";

    // Set FIFO to Stream mode (0b10)
    adxl.fifo_ctl.write("MODE", 0b10);

    // Enable measurement mode
    std::cout << "Enabling MEASUREMENT mode...\n";
    adxl.power_control.write("MEASURE", 1);

    // Verify settings
    std::cout << "\n*** REGISTER VERIFICATION ***\n";
    std::cout << "POWER_CTL.MEASURE:    "
              << static_cast<int>(adxl.power_control.read("MEASURE")) << "\n";
    std::cout << "BW_RATE.RATE:         0x"
              << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(adxl.bandwidth_rate.read("RATE"))
              << std::dec << "\n";
    std::cout << "FIFO_CTL.MODE:        0b"
              << static_cast<int>(adxl.fifo_ctl.read("MODE")) << "\n";
    std::cout << "FIFO_CTL.SAMPLES:     "
              << static_cast<int>(adxl.fifo_ctl.read("SAMPLES")) << "\n";
    std::cout << "FIFO_STATUS.ENTRIES:  "
              << static_cast<int>(adxl.fifo_status.read("ENTRIES")) << "\n";

    std::cout << "\nInitialization complete.\n\n";
}

// ---------------------------------------------------------------------------
// read_continuous
// ---------------------------------------------------------------------------
std::vector<Sample> read_continuous(ADXL345& adxl, double duration_seconds) {
    double sample_rate  = odr_to_hz(adxl.odr);
    double sample_period = 1.0 / sample_rate;
    int    watermark    = adxl.watermark;

    std::cout << "=== STARTING CONTINUOUS ACQUISITION ===\n";
    std::cout << "Duration:         " << duration_seconds << " s\n";
    std::cout << "Sample rate:      " << sample_rate << " Hz\n";
    std::cout << "Expected samples: ~"
              << static_cast<long>(duration_seconds * sample_rate) << "\n";
    std::cout << "Watermark set at: " << watermark << " samples\n";
    std::cout << "Reading when samples available...\n\n";

    int overflow_count = 0;
    int read_count     = 0;

    std::vector<Sample> raw_samples;
    // Reserve 10 % extra capacity to absorb timing jitter without reallocation.
    constexpr double RESERVE_FACTOR = 1.1;
    raw_samples.reserve(static_cast<size_t>(duration_seconds * sample_rate * RESERVE_FACTOR));

    // Clear interrupt flags before starting
    adxl.interrupt_source.read("WATERMARK");

    using Clock = std::chrono::steady_clock;
    auto start  = Clock::now();
    auto end    = start + std::chrono::duration<double>(duration_seconds);

    while (Clock::now() < end) {
        bool watermark_reached = adxl.interrupt_source.read("WATERMARK") != 0;
        bool overflow_occurred = adxl.interrupt_source.read("OVERRUN")   != 0;

        if (overflow_occurred) {
            ++overflow_count;
        }

        if (watermark_reached) {
            for (int i = 0; i < watermark; ++i) {
                auto [x, y, z] = adxl.get_accel();
                raw_samples.push_back({0.0, x, y, z});
            }
            read_count += watermark;
        }
    }

    // Assign evenly-spaced timestamps based on the configured ODR
    std::vector<Sample> timestamped;
    timestamped.reserve(raw_samples.size());
    for (size_t i = 0; i < raw_samples.size(); ++i) {
        timestamped.push_back({
            static_cast<double>(i) * sample_period,
            raw_samples[i].x,
            raw_samples[i].y,
            raw_samples[i].z
        });
    }

    std::cout << "=== ACQUISITION COMPLETE ===\n";
    std::cout << "Collected " << timestamped.size()
              << " samples in " << duration_seconds << " s\n";
    std::cout << "Expected: ~"
              << static_cast<long>(duration_seconds * sample_rate) << "\n";
    std::cout << "Overflow events:  " << overflow_count << "\n";
    std::cout << "Read operations:  " << read_count << "\n\n";

    return timestamped;
}

// ---------------------------------------------------------------------------
// write_to_csv
// ---------------------------------------------------------------------------
std::string write_to_csv(const std::vector<Sample>& samples,
                         const std::vector<std::string>& settings,
                         const std::string& filename) {
    std::string out_file = filename;
    if (out_file.empty()) {
        // Generate timestamped filename
        auto now      = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm     tm_info;
        localtime_r(&t, &tm_info);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_info);
        out_file = std::string("data/accelerometer_data_") + buf + ".csv";
    }

    std::cout << "Writing " << samples.size()
              << " samples to " << out_file << "...\n";

    std::ofstream csv(out_file);
    if (!csv.is_open()) {
        throw std::runtime_error("Failed to open output file: " + out_file);
    }

    // Metadata row
    for (size_t i = 0; i < settings.size(); ++i) {
        if (i > 0) csv << ',';
        csv << settings[i];
    }
    csv << '\n';

    // Header row
    csv << "time_s,x_g,y_g,z_g\n";

    // Data rows
    csv << std::fixed << std::setprecision(6);
    for (const auto& s : samples) {
        csv << s.timestamp << ','
            << s.x        << ','
            << s.y        << ','
            << s.z        << '\n';
    }

    return out_file;
}

// ---------------------------------------------------------------------------
// print_sample_preview
// ---------------------------------------------------------------------------
void print_sample_preview(const std::vector<Sample>& samples, int num_preview) {
    if (samples.empty()) {
        std::cout << "No samples to preview.\n";
        return;
    }

    std::cout << "=== SAMPLE PREVIEW ===\n";
    std::cout << std::left
              << std::setw(12) << "Time(s)"
              << std::setw(10) << "X(g)"
              << std::setw(10) << "Y(g)"
              << std::setw(10) << "Z(g)" << '\n';
    std::cout << std::string(42, '-') << '\n';

    auto print_row = [](const Sample& s) {
        std::cout << std::fixed << std::setprecision(6)
                  << std::left << std::setw(12) << s.timestamp
                  << std::setprecision(4)
                  << std::setw(10) << s.x
                  << std::setw(10) << s.y
                  << std::setw(10) << s.z << '\n';
    };

    int n = static_cast<int>(samples.size());
    int show_head = std::min(num_preview, n);

    for (int i = 0; i < show_head; ++i) {
        print_row(samples[static_cast<size_t>(i)]);
    }

    if (n > num_preview * 2) {
        std::cout << "...\n";
        for (int i = n - num_preview; i < n; ++i) {
            print_row(samples[static_cast<size_t>(i)]);
        }
    }

    std::cout << '\n';
}

// ---------------------------------------------------------------------------
// flush
// ---------------------------------------------------------------------------
void flush(ADXL345& adxl) {
    adxl.fifo_ctl.write("MODE", 0b00);  // Bypass — stops FIFO
    uint8_t entries = adxl.fifo_status.read("ENTRIES");
    while (entries != 0) {
        adxl.get_accel();
        entries = adxl.fifo_status.read("ENTRIES");
    }
}

// ---------------------------------------------------------------------------
// terminate
// ---------------------------------------------------------------------------
void terminate(ADXL345& adxl) {
    adxl.fifo_ctl.write("MODE", 0b00);  // Bypass — FIFO stops collecting
}

// ---------------------------------------------------------------------------
// measure
// ---------------------------------------------------------------------------
void measure(ADXL345& sensor, bool is_stationary) {
    init_adxl(sensor);

    constexpr double duration = 10.0;  // seconds

    std::vector<std::string> settings = {
        std::string("Stationary: ") + (is_stationary ? "true" : "false"),
        std::string("Output Data Rate: ") + std::to_string(odr_to_hz(sensor.odr)) + " Hz",
        std::string("Range: ") + std::to_string(range_to_g(sensor.g_range)) + " g"
    };

    auto samples  = read_continuous(sensor, duration);
    auto filename = write_to_csv(samples, settings);

    print_sample_preview(samples, 10);

    std::string sep(57, '=');
    std::cout << sep << '\n'
              << "Measurement complete!\n"
              << "Data saved to: " << filename << '\n'
              << sep << "\n\n";

    terminate(sensor);
}
