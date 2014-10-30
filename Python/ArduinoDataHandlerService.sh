#!/bin/sh

### BEGIN INIT INFO
# Provides:          ArduinoDataHandler
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Post data from Arduino sketch to Manylabs data set
# Description:       Post data from Arduino sketch to Manylabs data set
### END INIT INFO

# script modified from:
# http://blog.scphillips.com/2013/07/getting-a-python-script-to-run-in-the-background-as-a-service-on-boot/

# Change the next 3 lines to suit where you install your script and what you want to call it
DIR=/home/root/
DAEMON=$DIR/ArduinoDataHandler.py
DAEMON_NAME=ArduinoDataHandlerService

# Add any command line options for your daemon here
DAEMON_OPTS=""

# This next line determines what user the script runs as.
# Root generally not recommended but necessary if you are using GPIO from Python.
DAEMON_USER=root

# The process ID of the script when it runs is stored here:
PIDFILE=/var/run/$DAEMON_NAME.pid

do_start () {
    start-stop-daemon --start --background --pidfile $PIDFILE --make-pidfile --user $DAEMON_USER --chuid $DAEMON_USER --startas $DAEMON -- $DAEMON_OPTS
}
do_stop () {
    start-stop-daemon --stop --pidfile $PIDFILE --retry 10
}

case "$1" in

    start|stop)
        do_${1}
        ;;

    restart|reload|force-reload)
        do_stop
        do_start
        ;;

    *)
        echo "Usage: /etc/init.d/$DAEMON_NAME {start|stop|restart}"
        exit 1
        ;;
esac
exit 0
