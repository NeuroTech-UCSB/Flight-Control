# Remote Control Pipeline

This pipeline integrates Brain-Computer Interface (BCI) data with drone flight control.

## Installation & Setup

Before running the pipeline, set up a virtual environment and install the required dependencies using pip.

```bash
# Create a virtual environment named '.venv'
python3 -m venv .venv 

# Activate the virtual environment (Mac/Linux)
source .venv/bin/activate

# Install dependencies
pip3 install -r reqs.txt

#run pipeline
python3 multithread.py
```

# Required Hardware 
- Cyton Board
- Ardupilot platform

# Interesting Files

- multithread.py: The main orchestrator of the pipeline. It uses threading to run concurrent tasks without blocking. It handles the MAVLink telemetry stream, actively monitors flight state and geofencing limits, processes incoming BCI commands, and executes flight primitives (like takeoff, navigation, and landing).

- bci.py: The interface for the EEG headset (e.g., Cyton board). It handles the serial connection, reads the raw bitstream, applies live digital filtering, and uses a pre-trained machine learning classifier (via joblib) to translate brainwave band powers into actionable flight states (e.g., RELAXED, FOCUSED, CLENCH).

- EEGRawFilter.py: Contains methods for bandpass filtering 