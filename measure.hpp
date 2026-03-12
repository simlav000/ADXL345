#pragma once

#include "adxl345.hpp"

#include <string>
#include <vector>
#include <tuple>

// Sample collected from the sensor: (timestamp_s, x_g, y_g, z_g)
struct Sample {
    double timestamp;
    float  x;
    float  y;
    float  z;
};

// Initialize the ADXL345 for continuous FIFO-based measurement.
// Sets FIFO to Stream mode and enables measurement mode, then prints
// a register verification summary to stdout.
void init_adxl(ADXL345& adxl);

// Continuously drain the FIFO for `duration_seconds`, collecting samples.
// Polls the WATERMARK interrupt flag and reads `watermark` samples each time
// the flag is set, mirroring the Python read_continuous() strategy.
// Returns a vector of timestamped samples.
std::vector<Sample> read_continuous(ADXL345& adxl, double duration_seconds = 10.0);

// Write samples plus metadata to a CSV file.
// If `filename` is empty, a timestamped name in the data/ directory is used.
// Returns the actual filename written.
std::string write_to_csv(const std::vector<Sample>& samples,
                         const std::vector<std::string>& settings,
                         const std::string& filename = "");

// Print the first and last `num_preview` samples as a table to stdout.
void print_sample_preview(const std::vector<Sample>& samples,
                          int num_preview = 10);

// Put FIFO into Bypass mode and drain any remaining entries.
void flush(ADXL345& adxl);

// Put FIFO into Bypass mode (stop collecting).
void terminate(ADXL345& adxl);

// Top-level measurement routine: init → acquire → CSV → preview → terminate.
void measure(ADXL345& sensor, bool is_stationary = false);
