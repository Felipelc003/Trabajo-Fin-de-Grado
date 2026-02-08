#!/bin/bash
#
# This file is part of  Diseño Modular e Incremental para la 
# Navegación Autónoma Contextual en Sistemas Embebidos de Bajo Coste
#
# Copyright 2026 Felipe López Castro <i12locaf@uco.es>
#
# Diseño Modular e Incremental para la Navegación Autónoma Contextual 
# en Sistemas Embebidos de Bajo Coste is free software: you can redistribute it 
# and/or modify it under the terms of the GNU General Public License 
# as published by the Free Software Foundation, either version 3 of the License, 
# or  (at your option) any later version.
# 
# Diseño Modular e Incremental para la Navegación Autónoma Contextual 
# en Sistemas Embebidos de Bajo Coste is distributed in the hope that it will be useful, 
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Diseño Modular e Incremental para la Navegación Autónoma Contextual 
# en Sistemas Embebidos de Bajo Coste.
# If not, see <https://www.gnu.org/licenses/>.
#

# Stop on error
set -e

echo "Starting installation for Adeept PiCar-B on Raspbian Bookworm..."

# 1. Update system package lists
echo "Updating system package lists..."
sudo apt update

# 2. Install system dependencies (APT)
# python3-full: ensures venv is available
# python3-picamera2: essential for the camera on Bookworm (libcamera based)
# python3-opencv: optimized opencv for Pi
# libzbar0: required for pyzbar
# python3-rpi.gpio: system-level GPIO approach (though we also install in venv, this ensures system deps are there)
echo "Installing system dependencies..."
sudo apt install -y python3-full python3-picamera2 python3-opencv libzbar0 python3-tk i2c-tools python3-pip

# 3. Create Python Virtual Environment
# We use --system-site-packages to access python3-picamera2 and python3-opencv installed via apt
echo "Creating virtual environment (venv) with system site packages..."
if [ -d "venv" ]; then
    echo "Virtual environment 'venv' already exists. Skipping creation."
else
    python3 -m venv --system-site-packages venv
    echo "Virtual environment created."
fi

# 4. Install Python dependencies inside venv
echo "Installing Python libraries from requirements.txt..."
source venv/bin/activate

# Upgrade pip just in case
pip install --upgrade pip

# Install requirements
if [ -f "requirements.txt" ]; then
    pip install -r requirements.txt
else
    echo "Warning: requirements.txt not found!"
fi

echo "Installation complete!"
echo "To run the project, activate the environment first:"
echo "source venv/bin/activate"
