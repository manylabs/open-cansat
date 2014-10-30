This is the video package from Jun and Shaohan

# Files #

autocapture needs to go in `/usr/local/bin`
videocapture needs to go in `/etc/init.d`

Make sure both are executable.

# Starting at Boot #

To enable starting the script at boot run:
`sudo update-rc.d videocapture defaults`

You can disable it by running:
`sudo update-rc.d videocapture remove`
