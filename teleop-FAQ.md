Before you look any further, remember to checkout ```teleop``` and run ```

## Python Virtual Environment

### Output
```
ImportError: Couldn't import Django. Are you sure it's installed and available on your PYTHONPATH environment variable? ...
```

### Explanation
You have not entered the Python virtual environment. Do enter the python venv, enter `mrover` in the shell. This macro runs:
```bash
$ cd ~/ros2_ws/src/mrover && source ~/ros2_ws/src/mrover/venv/bin/activate
```
which opens the python virtual environment and allows you to run the basestation. 

## CMake

some files in cmake are missing on teleop

ask john?

science_hw_bridge, etc

## manifpy

Ubuntu: 

```
sudo apt-get install libeigen3-dev
cd && git clone https://github.com/artivis/manif.git
cd manif
python3 -m pip install .
cd ~/ros2_ws/src/mrover
git submodule update --init deps/manif
```

neven sent thru slack

integrated with ansible playbook or build.sh somehow?

## extra bun and python installs

### bun

chart.js (probably not needed, chartjs needed to fake data, clean up on teleop?)


### pip3 (am i missing something or what)

django

daphne

channels
