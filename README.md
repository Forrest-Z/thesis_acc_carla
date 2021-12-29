Running simulation:
* Download Carla 0.9.12 https://github.com/carla-simulator/carla/releases/tag/0.9.12
* Navigate to ```Carla_9_12/WindowsNoEditor/PythonAPI/``` in this repository.
* Copy folder ```FINAL``` to your ```CARLA path/PythonAPI/```
* Run with python 3.7 eg. for Windows ```py -3.7 main.py -x maps/test_1.xodr --no-rendering --distance 10```

Created PyGame window may be unresponsive for a few seconds after starting the scripts and display "Rendering Map" this is expected behavior.

If for some reason the script times out not connecting to the CARLA server please make sure it is running and re-run the script, this may be caused by loading map files.

Full map view mode with TAB key.
Exit with ESC in visu window or CTRL+C in cmd.
