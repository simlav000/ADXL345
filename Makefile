# ============================================================
# Makefile — ADXL345 C++ driver
#
# Build:
#   make              # compile adxl345_measure
#   make clean        # remove build artefacts
#
# Install dependencies on Raspberry Pi OS / Debian:
#   sudo apt-get install -y build-essential libi2c-dev
#
# The executable requires access to /dev/i2c-1.
# Either run with sudo, or add your user to the i2c group:
#   sudo usermod -aG i2c $USER
#   # then log out and back in
#
# The i2c bus must be enabled first:
#   sudo raspi-config  →  Interface Options → I2C → Enable
# ============================================================

CXX      := g++
CXXFLAGS := -O2 -Wall -Wextra -std=c++17
LDFLAGS  :=

TARGET := adxl345_measure

SRCS := adxl345.cpp measure.cpp menu.cpp
OBJS := $(SRCS:.cpp=.o)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Header dependencies
adxl345.o: adxl345.cpp adxl345.hpp
measure.o: measure.cpp measure.hpp adxl345.hpp
menu.o:    menu.cpp    measure.hpp adxl345.hpp

clean:
	rm -f $(OBJS) $(TARGET)
