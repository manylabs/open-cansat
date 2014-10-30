open-cansat
===========

The Edison includes one processor that can be used for Linux programs
and another processor that runs an emulation of an Arduino. The Arduino
emulator provides access to digital and analog input/output pins, though 
some may have different behavior than the Arduino standards.

Getting data from one side to the other is a bit challenging. Our solution
is to write data to a text file from the Arduino side and to read the text
file from the Linux side.

This repository includes code for both side. An Arduino sketch reads values 
from sensors attached to the Edison and logs the results, in a comma-separated
format to a file. A Python program reads that log file and POSTs that data
to a server when the Edison has a wifi connection.

The sections "Python" and "Arduino" go over the files in those directories,
where they should be placed in the Edison, and general use.

The "POSTing Data" section shows one example of posting data and explains how
you can modify the Python program to POST to other servers.

Finally, the "Setup for New Data" section shows how to save the logs and
configure the system for recording new data.

# Python #

The python code has two components. The first, ArduinoDataHandler.py, should be
placed in `/home/root/`. The second, ArduinoDataHandlerService.sh, should be
placed in `/etc/init.d/`. Make sure both are executable.

As it's currently setup, the python program looks for a file:
`/home/root/dataSetId.txt`

This file should only contain the ID of the data set to post to. If this file
doesn't exist, the script will quit. There's more information in the POSTing
Data section.

The Arduino sketch depends on the python script being run, at least once, before
it will log data. This is so the python script has a chance to setup other files
and write a header row to the csv before data is added. Once the script is added
to run at boot, this requirement will be satisfied.

## Starting at Boot ##

To enable starting the script at boot run:
`sudo update-rc.d ArduinoDataHandlerService.sh defaults`

You can disable it by running:
`sudo update-rc.d ArduinoDataHandlerService.sh remove`

## Created Files ##

The script will create several files and directories.

* `/root/home/sensor_logs/log.csv` This is the csv file for the sensor data
* `/root/home/sensor_logs/currentLine.txt` This keeps track of the current line
  in log.csv.
* `/root/home/ArduinoDataHandler_log.txt` This is the log for the service.

# Arduino #

The Arduino sketch will log lines of comma-separated sensor values to the file:
`/home/root/sensor_logs/log.csv`

This directory _must_ already exist. Normally this is taken care of by the
ArduinoDataHandler script starting on boot. If you've disabled this, then you
should run the ArduinoDataHandler at least once to create this directory. Data
_will not_ be saved otherwise.

Sometimes the Arduino sketch is overwritten on boot and will not start. The
sketch is saved to `/sketch/sketch.elf`. If you `ls -al /sketch/` when this
happens, you'll see that the file is empty. Once the sketch has been coppied and
is running, you should make a copy in another directory so you can replace the
overwritten sketch without using the Arduino IDE. For instance:

`cp /sketch/sketch.elf /home/root/sketch.elf.backup`

Then if the sketch is overwritten:

`cp /home/root/sketch.elf.backup /sketch/sketch.elf`

## LEDs ##

When the GPS does not yet have a fix, the bottom LED will flash. This LED will
become solid when the GPS has a fix.

# POSTing Data #

In the current setup, the CanSat will POST data to the Manylabs server when it
has a WiFi connection. You can also customize the program to upload to another
service or server.

If you're using the Manylabs platform, you'll need to create an account and
create a data set to hold your data. Once you have the ID, you can change the
data set that the CanSat will post to with:
`echo <data-set-id> > dataSetId.txt`

Replacing `<data-set-id>` with your data set. You can find more information
about using the Manylabs platform here:
https://www.manylabs.org/docs/general/creatingDataSets/

If you'd like to upload data to another service or server, you'll need to
customize the program. For most cases, you'll probably be able to get away with
only changing `postData` to use the specific format your server requires.

# Setup for New Data #

If you need to clear the data on the board, first stop the data handler script
and the autocapture script:
`/etc/init.d/ArduinoDataHandlerService.sh stop`
`/etc/init.d/videocapture stop`

Next, copy the `/home/root/sensor_logs/log.csv` file from the Edison. You should
rename the file so you can keep track if multiple logs need to be removed.

Then you can remove the sensor_logs directory and the script log:
`rm -r /home/root/sensor_logs` and `rm ArduinoDataHandler_log.txt`

When the Edison boots up, the ArduinoDataHandler script will recreate
sensor_logs and the files it contains.

Finally, copy off the mkv files in `Movies/autocapture/` and then:
`rm /home/root/Movies/autocapture/*.mkv".

If any of this is testing data, and not needed, you can simply delete it instead
of copying it off the Edison.

# Shutdown #

When you're turning off the Edison, you should hold down the power button for
approximately 8 seconds. Pulling the power without shutting the Edison down can
result in corrupted data or other strange behavior.

# Hardware Notes #

The Edison runs at 1.8V. In order to communicate with 3.3V and 5V devices, you'll
need to use a [level shifter](https://www.sparkfun.com/products/12009). 
You can run some components (e.g. red LEDs) with 1.8V. 

The mini breakout board includes connections for power and various input/output pins.
To find a particular Arduino pin, first determine which GPIO pin it is using the
Arduino breakout schematic. Then use the mini breakout schematic to locate the
pin on the breakout grid. You can find a partial pinout diagram in the Hardware
folder of this repository.

The holes in the mini breakout grid are sufficiently small that you can connect
headers without solder. Once you are done testing, you should solder the connection
so that they are more reliable.
