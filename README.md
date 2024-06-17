# Vicon Sync
A simple script using the Vicon Nexus SDK to automatically detect gait cycles within subjects and aggregate force plate data, eliminating the need to manually collect marker data for a trial.
## Setup
Setting up Vicon Sync for your own Vicon Nexus configuration is straightforward and easy to do.
### 1. Clone the repository
```bash
git clone https://github.com/artiehumphreys/vicon-sync
cd vicon-sync/
```
### 2. Install Vicon Nexus SDK
Install the Vicon Nexus Datastream SDK using the following link. This is required and cannot be installed using a package manager like pip.
https://www.vicon.com/software/datastream-sdk/
### 3. Install dependencies
Create a virtual environment and install the dependencies present within the ```requirements.txt``` file.
```bash
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```
### 4. Configure your trial
To configure your own trial within Vicon, ensure the path within the Vicon class is correctly configured to the location of your ```.c3d``` files. The path can be set within the ```vicon.py``` file in the ```src``` folder.
### 5. Collect data
After marking foot-strike and foot-up events using the Vicon Nexus application, you can run the data collection pipeline, the ```process_event.py``` file within the ```event_handling``` folder, to get all of the important trial information outside of the Vicon Nexus App.
