Before you look any further, remember to checkout `teleop` and run ```./ansible.sh build.yml```, followed by ```./build.sh```. 

## Python venv

### Output

```
ImportError: Couldn't import Django. Are you sure it's installed and available on your PYTHONPATH environment variable? ...
```

### Solution

Enter `mrover` before trying to run basestation

You have not entered the Python virtual environment. Do enter the python venv, enter `mrover` in the shell. This macro runs:

```bash
$ cd ~/ros2_ws/src/mrover && source ~/ros2_ws/src/mrover/venv/bin/activate
```

which opens the python virtual environment and allows you to run the basestation.

---

## CMake

### Output

From your `./build.sh` output:

```
ZED not found
science_hw_bridge missing
```

### Solution

Some files that CMake expects are missing. You could try and remove the `#Perception` and `#Embedded` sections in `CMakeLists`, and reach out to your team lead for help.

---

## manifpy

The manifpy repository is not installed or activated correctly. Try the code below, running from your `mrover` directory. If that doesn't work, reach out to your team lead.

```
sudo apt-get install libeigen3-dev
cd && git clone https://github.com/artivis/manif.git
cd manif
python3 -m pip install .
cd ~/ros2_ws/src/mrover
git submodule update --init deps/manif
```

---
