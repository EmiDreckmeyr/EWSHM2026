# EWSHM2026 – Bridge Communication Simulation Framework

This repository provides a workflow for generating realistic bridge node deployments, deriving propagation-aware coverage maps, and evaluating communication strategies using simulation.

---

## 1. Generating Node Locations (Google Earth)

Node placement along a bridge is obtained using Google Earth or generated programmatically.

### Option A – Using Google Earth
1. Open Google Earth and locate your bridge.
2. Use the **Path tool** to trace the bridge axis.
3. Export the path as a `.kml` file.
4. Extract GPS coordinates (latitude, longitude, altitude).
5. Interpolate points along the path to create node positions:
   - Uniform spacing (e.g., every 5–20 m)
   - Or structure-aware placement (deck, piers, off-bridge nodes)

### Option B – Python-Based Generation
If a set of GPS coordinates along the bridge is already available (e.g., from a KML file or manual sampling), evenly spaced node locations can be generated directly using a Python script.  
This approach allows:
- Automated interpolation along the bridge axis  
- Precise control over node spacing  
- Fast generation of large-scale deployments  

### Output
A CSV file containing: node_id, latitude, longitude, altitude


This dataset serves as the spatial input for propagation modeling and simulation.

---

## 2. Coverage Map Generation (Radio Mobile)

Propagation-aware coverage maps are generated using Radio Mobile.

Radio Mobile must be installed prior to use.  
Official website: http://www.ve2dbe.com/english1.html

### Steps
1. Install Radio Mobile from the official website.
2. Import terrain data (SRTM or equivalent).
3. Define radio parameters:
   - Frequency (e.g., 868 MHz for LoRa)
   - Transmission power
   - Antenna gain and height
   - Receiver sensitivity
4. Place transmitter(s) (node locations).
5. Generate coverage maps using the **Combined Cartesian** tool (path loss / received power).

---

## 3. Execution Environment

The workflow can be executed in multiple environments:

- Google Colab (recommended for quick testing)
- Local Python environments (e.g., Conda, venv)
- Jupyter Notebook
- HPC clusters (for large-scale simulations)

### Requirements
- Python 3.x
- NumPy
- Pandas
- Matplotlib
- Optional: SciPy, scikit-learn

---

## 4. Network Simulation with ns-3

For network-level evaluation, simulations are performed using ns-3.

Official website: https://www.nsnam.org/

### Setup
1. Install ns-3 (latest version recommended).
2. Verify installation:
   ```bash
   ./ns3 --version

### Running Simulations
1. Copy your simulation script into:
   ```bash
   ./ns-3/scratch/
3. Run using:
    ```bash
   ./ns3 run scratch/enddevice

## Output
The simulation generates:
1. CSV files containing energy consumption per node
2. Log files including ULDR (Uplink Data Rate) statistics

These outputs can be used for performance evaluation and comparison of different deployment and communication strategies.
   
