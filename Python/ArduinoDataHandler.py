#!/usr/bin/env python
import os
import urllib
import urllib2
import time
import logging
import traceback

# Set this to False when testing, True to actually perform the POST
POST_DATA = True

POST_URL = "http://www.manylabs.org/data/rpc/appendData/"

DATA_SET_ID_FILE = '/home/root/dataSetId.txt'
LOG_FILE = "/home/root/sensor_logs/log.csv"
LINE_FILE = "/home/root/sensor_logs/currentLine.txt"
DATA_HANDLER_LOG_FILE = "/home/root/ArduinoDataHandler_log.txt"

# This is the list of column names
# This sets the header line in the LOG_FILE and the keys for the form encoded
# data that is POSTed.
#
# The order and names matter: The order must match the order of the data the
# Arduino is logging to LOG_FILE. The names must match the names of the columns
# in the data set on Manylabs.org. This is case-sensitive.
COLUMN_LIST = [
	"timestamp",
	"Magnetometer_X", "Magnetometer_Y", "Magnetometer_Z",
	"Accelerometer_X", "Accelerometer_Y", "Accelerometer_Z",
	"Barometric_Pressure", "Temperature_1",
	"Relative_Humidity", "Temperature_2",
	"Latitude", "Longitude", "Altitude", "Speed"
]

LINE_READ_DELAY = 0.5 # Seconds to wait between checking for new data
POST_ERROR_DELAY = 0.5 # Seconds to wait after a failed post before trying again

# Setup logging
logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG, filename=DATA_HANDLER_LOG_FILE)

def checkForNewDataLoop(filename, lastLineRead):
	currentLineNumber = 0
	try:
		with open(filename, 'r') as logFile:
			while True:
				line = logFile.readline().strip()
				if line:
					currentLineNumber += 1

					# Otherwise check if we've read this line before
					if currentLineNumber > lastLineRead:

						# Don't want to post the header line
						if currentLineNumber > 1:

							# Keep trying to post until we succeed
							doPostData = True
							while doPostData:
								postSuccess = postData(line)
								if postSuccess:
									doPostData = False
								else:
									# Sleep a bit
									time.sleep(POST_ERROR_DELAY)

						# Update the last line read
						setLastLineRead(currentLineNumber)
				else:
					# Sleep a bit
					time.sleep(LINE_READ_DELAY)

	except IOError as error:
		logging.error("Could not open %s", filename)
		logging.error(error)
		logging.error(traceback.format_exc())
		raise # Rethrow the error so we can catch it outside

def postData(data):

	values = { "addTimestamp" : "1", "dataSetId" : DATA_SET_ID }

	splitData = data.split(",")
	if(len(splitData) != len(COLUMN_LIST)):
		logging.error("Invalid Data. Number of items doesn't match number of columns and will not be posted. Data:")
		logging.error(data)
		logging.error("Column List is: %s", COLUMN_LIST)
		return True

	for index, value in enumerate(splitData):
		key = COLUMN_LIST[index]
		values[key] = value

		# we don't want to post the timestamp
		if 'timestamp' in values:
			del values["timestamp"]


	if not POST_DATA:
		logging.info("Post data:")
		logging.info(values)
		return True

	encodedData = urllib.urlencode(values)
	request = urllib2.Request(POST_URL, encodedData)

	try:
		response = urllib2.urlopen(request)
	except urllib2.URLError as e:
		if hasattr(e, 'code'): # Something like 404
			logging.warning("Post Error Code: %s", e.code)
		if e.reason:
			logging.warning("Post Error Reason: %s", e.reason)
		return False
	else:
		logging.info("Data Posted")
		return True

	return False

def getLastLineRead():
	try:
		# Create or open LINE_FILE
		with open(LINE_FILE, 'a+') as lineFile:
			# Attempt to read line number
			line = lineFile.readline()
			# If there is a line, return it. If empty set to 0 and return
			if line:
				try:
					return int(line)
				except ValueError as error:
					logging.error("Error parsing last line read in file: %s", LINE_FILE)
					logging.error(error)
					logging.error(traceback.format_exc())
					raise # Rethrow the error so we can catch it outside
			else:
				lineFile.write("0\n")
				return 0
	except IOError as error:
		logging.error("Could not open %s", LINE_FILE)
		logging.error(error)
		logging.error(traceback.format_exc())
		raise # Rethrow the error so we can catch it outside

def setLastLineRead(lineNumber):
	# Open LINE_FILE
	with open(LINE_FILE, 'w') as lineFile:
		# Set last line read
		lineFile.write(str(lineNumber) + "\n")

def setupLog(logFile):
	head, tail = os.path.split(logFile)
	try:
		os.makedirs(head)
	except OSError as e:
		logging.warning("%s already exists or cannot be created", head)
	try:
		# Creating if it's not there
		if not os.path.isfile(logFile):
			with open(logFile, 'a') as log:
				# Add header line
				headerLine = ",".join(COLUMN_LIST)
				log.write(headerLine + "\n")
	except IOError as error:
		logging.error("Could not open %s", logFile)
		logging.error(error)
		logging.error(traceback.format_exc())
		raise # Rethrow the error so we can catch it outside

if __name__ == "__main__":
	# Get data set id
	try:
		with open(DATA_SET_ID_FILE, 'r') as dataSetIdFile:
			DATA_SET_ID = int(dataSetIdFile.readline().strip())
	except IOError as error:
		logging.error("Could not open %s to get data set id. Quitting", DATA_SET_ID_FILE)
		logging.error(error)
		exit(1)
	try:
		try:
			setupLog(LOG_FILE);
			lineNumber = getLastLineRead()
			logging.info("Starting Loop")
			checkForNewDataLoop(LOG_FILE, lineNumber)
		except Exception as error:
			logging.info("Quiting")
			logging.error(error)
			logging.error(traceback.format_exc())
			exit(1)
	except KeyboardInterrupt as error:
		logging.info("Stopping Loop")
		exit(0)
