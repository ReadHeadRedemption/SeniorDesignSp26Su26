#Importing modules from the pygerber library to parse Gerber files
from __future__ import annotations
from pygerber import gerber as gb 

#importing modules from gscrib convert Gerber files to G-code
from itertools import product
from gscrib import GCodeBuilder

#importing modules for GUI
from GUI import GUI

# misc imports
import zipfile


#global variables for paramreters 
'''Define Later with GUI input'''
nozzleDiameter = 0.4
maxBedSize = {"X": 200, "Y": 200, "Z": 200} #in mm

layerHeight = 0.2 #in mm
printSpeed = 60 #in mm/s
maxBedTemp = 180 #in celcius
flowRate = 5 #in mm/s


   


#read in the file from the user input and parse it using the parser and tokenizer

#once file is parsed use gscrib to convert the file
#need to set up pre generation parameters to read into marlin firmware





GUI()