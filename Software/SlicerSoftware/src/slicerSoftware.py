#Importing modules from the pygerber library to parse Gerber files
from __future__ import annotations
from pygerber import gerber as gb 

#importing modules from gscrib convert Gerber files to G-code
from itertools import product
from gscrib import GCodeBuilder

import pygerber

#importing modules for GUI
from GUI import GUI

#misc imports
import json

# config file location
configFilePath = "../src/config.json"

with open(configFilePath, "r") as f:
    print("load succseded")
    configFile = json.load(f)
    print(configFile)


#read in the file from the user input and parse it using the parser and tokenizer

#once file is parsed use gscrib to convert the file
#need to set up pre generation parameters to read into marlin firmware

#GUI()