import tkinter as tk
from tkinter import filedialog

from zipfile import ZipFile
import json

import configFunctions as cF

from inkGUI import inkGUI as iGUI

'''
GUI LAYOUT PLAN:
[Add ink but]   []      []      []
[Layer Height]  [input] []      []
[BED SIZE]      [Xin]   [Yin]   [Zin]
[GERBER FILE]   []      []      [Save]
[Gen G-code]    []      []      [] 
'''


GERBfile = "" #gerber file path global variable



#GUI functions
def saveInputMain(lH, x,y,z, file):
    layHeight = 0
    maxBed = [0,0,0]


    try:
        layHeight = float(lH.get()) if lH.get() != "" else 0
        maxBed = [float(x.get()) if (x.get() != "X" or x.get() == "") else 0,
                  float(y.get()) if (y.get() != "Y" or y.get() == "") else 0,
                  float(z.get()) if (z.get() != "Z" or z.get() == "") else 0]
    except ValueError:
        print("Invalid input. Please enter numeric values")

    if layHeight != 0:
        cF.updConf({"layerHeight": cF.unitConv(layHeight, cF.defUnits)})
    if maxBed[0] != 0 and maxBed[1] != 0 and maxBed[2] != 0:
        cF.updConf({"maxBedSize": [cF.unitConv(size, cF.defUnits) for size in maxBed]})

    if file != "":
        cF.updConf({"gerberFile": file})
  

#Import gerber files through the GUI
def importFile(): #take in zip of gerber files
    global GERBfile
    #take in zip
    #extract it
    #parse zip file
        #extract only copper layers
        #take in file parameters
    GERBfile = filedialog.askopenfilename(
        title="Select a ZIP file to extract",
        filetypes=[("ZIP files", "*.zip")]
    )
    if not GERBfile:
        print("Operation cancelled. No ZIP file selected.")
        return
    print(f"Selected ZIP file: {GERBfile}")

def GUI(): #GUI for the slicer software
    MainScreen = tk.Tk()
    MainScreen.title("Slicer Software")
    #MainScreen.geometry("400x200")
    MainScreen.grid

#input fields
    #open ink configuration GUI
    inkConfigButton = tk.Button(MainScreen, text="Configure Ink Properties", command=iGUI)
    inkConfigButton.grid(row=0, column=0, sticky="w")

    #Layer Height Input
    tk.Label(MainScreen, text="Layer Height (mm)").grid(row=1, column=0,sticky="w")
    layerHeightInput = tk.Entry(MainScreen,width=5)
    layerHeightInput.grid(row=1, column=1)
    print("Layer Height: ", layerHeightInput.get())


    #Bed Size Input
    tk.Label(MainScreen, text="Max Bed Size (X, Y, Z in mm)").grid(row=2, column=0,sticky="w")
    xIn = tk.Entry(MainScreen,width=5)
    xIn.grid(row=2, column=1, sticky="w")
    xIn.insert(0, "X")
    yIn = tk.Entry(MainScreen,width=5)
    yIn.grid(row=2, column=2)   
    yIn.insert(0, "Y")
    zIn = tk.Entry(MainScreen,width=5)
    zIn.grid(row=2, column=3, sticky="w")
    zIn.insert(0, "Z")

    uploadButton = tk.Button(MainScreen, text="Upload Gerber File", command=importFile)
    uploadButton.grid(row=3, column=0,sticky="w")

    saveButton = tk.Button(MainScreen, text="Save", 
                           command = lambda: saveInputMain(layerHeightInput, xIn, yIn, zIn, GERBfile))
    saveButton.grid(row=3, column=4,sticky="w")

    #tk.Label(MainScreen,text="").grid(row=3, column=1) #empty label for spacing

    
    '''Link to GCODE gen function later'''
    generateButton = tk.Button(MainScreen,text="Generate G-code",
                               command= None) 
    generateButton.grid(row=4, column=0,sticky="e")

    #start screen
    MainScreen.mainloop()

#GUI()