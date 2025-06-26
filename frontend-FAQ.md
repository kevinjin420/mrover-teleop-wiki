# FAQ Section

FAQ for any errors you might encounter during installation or update

## Python Virtual Environment

### Output
```bash
ImportError: Couldn't import Django. Are you sure it's installed and available on your PYTHONPATH environment variable? ...
```

### Explanation
You have not entered the Python virtual environment. Do enter the python venv, enter `mrover` in the shell. This macro runs:
```bash
$ cd ~/ros2_ws/src/mrover && source ~/ros2_ws/src/mrover/venv/bin/activate
```
which opens the python virtual environment and allows you to run the basestation. 

## 