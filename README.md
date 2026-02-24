# aurora_IMU_calibrator

A comprehensive tool for calibrating the gyroscope and magnetometer of esp32-based stratospheric probe sensors using temperature-dependent calibration techniques.

## Overview

This project performs multi-stage sensor calibration to account for temperature variations and sensor offset errors in IMU (Inertial Measurement Unit) devices. The calibration process uses real-world data collected during controlled thermal conditions to improve sensor accuracy.

## Features

- **Temperature-dependent calibration**: Collects sensor data during controlled temperature changes via heating film
- **Static calibration**: Performs fixed-temperature tests with random probe rotations to calculate sensor offsets at a single temperature point
- **Comprehensive analysis**: Python-based data analysis to extract:
  - Sensor offset values
  - Temperature sensitivity coefficients
  - Combined IMU calibration parameters

## How It Works

### Phase 1: Temperature Sweep Calibration
1. Collects sensor readings while temperature gradually changes (via internal heating film)
2. Captures gyroscope and magnetometer data across the temperature range
3. Enables analysis of sensor temperature dependency characteristics

### Phase 2: Static Temperature Calibration
1. Maintains fixed temperature conditions
2. Rotates the probe through various orientations
3. Calculates sensor offsets specific to that temperature

### Phase 3: Data Analysis
- Python scripts process both datasets
- Generates temperature-corrected calibration parameters
- Combines data from both phases for comprehensive IMU offset correction

## Project Structure

- `README.md` - This file
- Scripts and calibration utilities (C++/Python)

## Technology Stack

- **Language**: C++
- **Analysis**: Python scripts

## Getting Started

[Add instructions for setup, dependencies, and usage here]

## License

[Add license information]

## Contact

Created by [Iaroslav-Baskov](https://github.com/Iaroslav-Baskov)
