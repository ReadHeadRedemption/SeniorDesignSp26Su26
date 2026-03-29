import tkinter as tk
from tkinter import filedialog

from zipfile import ZipFile
import json

import configFunctions as cF

'''
INK WINDOW LAYOUT PLAN:
[Ink Name]      [input]         []
[INK TYPE]      [Conductive]    [Insulator]
[NOZZLE DIAM]   [input]         []
[Cureing Temp]  [input]         []
[Cureing Time]  [input]         []
[AddInk]        []              [Close]

'''



#button functions
def addHead(name, head, nD, cTe, cTi):
    print(f"{head}")
    try:
        headName = str(name.get()) if name.get() != "" else "NA"
        #print(f"Head name: {headName}")
        headType = (head.get()) if head.get() != "" else "NA"
        #print(f"Head type: {head}")
        nD = float(nD.get()) if nD.get() != "" else 0
        #print(f"Nozzle diameter: {nD}")
        cTe = float(cTe.get()) if cTe.get() != "" else 0
        #print(f"Cureing temperature: {cTe}")
        cTi = float(cTi.get()) if cTi.get() != "" else 0
        #print(f"Cureing time: {cTi}")
    except ValueError:
        print("Invalid input. Please enter numeric values")


    if headName != "NA" and head != "NA" and nD != 0 and cTe != 0 and cTi != 0:
        #print("Attempting update")
        update =   {
            f"{headName}" :{
                "headType": headType,
                "nozzleDiameter": nD,
                "curingTemperature": cTe,
                "curingTime": cTi
            }
        }

        cF.updConf(update)
    
    

def close(window):
    window.destroy()
    print("Ink configuration window closed")

#set up ink configuration GUI
def inkGUI():
    inkConfig = tk.Tk()
    inkConfig.title("Ink Configuration")
    inkConfig.geometry("400x300")

    #Ink Name
    inkNameLabel = tk.Label(inkConfig, text="Ink Name:")
    inkNameLabel.grid(row=0, column=0, sticky="w")

    inkName = tk.Entry(inkConfig)
    inkName.grid(row=0, column=1,sticky="w")

    #Ink Type
    inkTypeLabel = tk.Label(inkConfig, text="Ink Type:")
    inkTypeLabel.grid(row=1, column=0, sticky="w")
    
    inkTypes = (("Condutive", 'C'),
                ("Insulator", 'I'))
    inkTypeSelected = tk.StringVar(inkConfig,"1")
    for type in inkTypes:
        inkButton = tk.Radiobutton(inkConfig, 
                                   text = type[0],
                                   value = type[1], 
                                   variable = inkTypeSelected)
        inkButton.grid(row=1, column=inkTypes.index(type)+1, sticky="w")

    
    #Nozzle Diameter
    nozDiamLabel = tk.Label(inkConfig, text="Nozzle Diameter (mm):")
    nozDiamLabel.grid(row=2, column=0, sticky="w")

    nozDiam = tk.Entry(inkConfig)
    nozDiam.grid(row=2, column=1, sticky="w")

    #Cureing Temp
    cureTempLabel = tk.Label(inkConfig, text="Cureing Temperature (°C):")
    cureTempLabel.grid(row=3, column=0, sticky="w")

    cureTemp = tk.Entry(inkConfig)
    cureTemp.grid(row=3, column=1, sticky="w")
    #Cureing Time
    cureTimeLabel = tk.Label(inkConfig, text="Cureing Time (Min):")
    cureTimeLabel.grid(row=4, column=0, sticky="w")

    cureTime = tk.Entry(inkConfig)
    cureTime.grid(row=4, column=1, sticky="w")
    
    #Add Ink
    addInkButton = tk.Button(inkConfig, 
                             text="Add Ink", 
                             command=lambda: addHead(inkName,
                                                    inkTypeSelected,
                                                    nozDiam,
                                                    cureTemp,
                                                    cureTime))
    
    addInkButton.grid(row=5, column=0, sticky="w")

    #Close Button
    closeButton = tk.Button(inkConfig, text="Close", command=lambda: close(inkConfig))
    closeButton.grid(row=5, column=2, sticky="w")

    inkConfig.mainloop()
