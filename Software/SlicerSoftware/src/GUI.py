import tkinter as tk
from tkinter import filedialog

from zipfile import ZipFile
import json

#GUI functions
def saveInput(nI, lH, x,y,z):
    nozDiam = float(999)    #default testing value 
    layHeight = float(999)  #default testing value
    maxBed = [999.0, 999.0, 999.0]#default testing value
    file = "Hello World"    #default testing value

    


    try:
        nozDiam = float(nI.get()) if nI.get() != "" else nozDiam
        layHeight = float(lH.get()) if lH.get() != "" else layHeight
        maxBed = [float(x.get()) if (x.get() != "X" or x.get() == "") else maxBed[0],
                  float(y.get()) if (y.get() != "Y" or y.get() == "") else maxBed[1],
                  float(z.get()) if (z.get() != "Z" or z.get() == "") else maxBed[2]]
        print("Nozzle Diameter: ", nozDiam)
        print("Layer Height: ", layHeight)
        print("Max Bed Size: ", maxBed)
    except ValueError:
        print("Invalid input. Please enter valid numbers.")
    print(file)

    print("Writing Config File")
    
    data = {
        "nozzleDiameter": nozDiam,
        "layerHeight": layHeight,
        "maxBedSize": maxBed,
        "gerberFile": file
    }


    with open("./src/config.json", "w") as configFile:
        json.dump(data, configFile, indent=4)
 

    



#Import gerber files through the GUI
def importFile(): #take in zip of gerber files
    global file
    #take in zip
    #extract it
    #parse zip file
        #extract only copper layers
        #take in file parameters
    file = filedialog.askopenfilename(
        title="Select a ZIP file to extract",
        filetypes=[("ZIP files", "*.zip")]
    )
    if not file:
        print("Operation cancelled. No ZIP file selected.")
        return
    print("File imported: ", file)



def GUI(): #GUI for the slicer software
    MainScreen = tk.Tk()
    MainScreen.title("Slicer Software")
    #MainScreen.geometry("400x200")
    MainScreen.grid

#input fields
    #Nozzle Diam Input
    tk.Label(MainScreen, text="Nozzle Diameter (mm)").grid(row=0, column=0,sticky="w")
    nozzleInput = tk.Entry(MainScreen,width=5)
    nozzleInput.grid(row=0, column=1)
    print("Nozzle Diameter: ", nozzleInput.get())

    #Bed Size Input
    tk.Label(MainScreen, text="Max Bed Size (X, Y, Z in mm)").grid(row=1, column=0,sticky="w")
    xIn = tk.Entry(MainScreen,width=5)
    xIn.grid(row=1, column=1, sticky="w")
    xIn.insert(0, "X")
    yIn = tk.Entry(MainScreen,width=5)
    yIn.grid(row=1, column=2)   
    yIn.insert(0, "Y")
    zIn = tk.Entry(MainScreen,width=5)
    zIn.grid(row=1, column=3, sticky="w")
    zIn.insert(0, "Z")

    #Layer Height Input
    tk.Label(MainScreen, text="Layer Height (mm)").grid(row=2, column=0,sticky="w")
    layerHeightInput = tk.Entry(MainScreen,width=5)
    layerHeightInput.grid(row=2, column=1)
    print("Layer Height: ", layerHeightInput.get())

    tk.Label(MainScreen,text="").grid(row=3, column=1) #empty label for spacing

    uploadButton = tk.Button(MainScreen, text="Upload Gerber File", command=importFile)
    uploadButton.grid(row=4, column=0,sticky="w")

    generateButton = tk.Button(MainScreen,text="Generate G-code",
                               command= lambda:saveInput(nozzleInput, layerHeightInput, xIn, yIn, zIn))
    generateButton.grid(row=4, column=3,sticky="e")

    #start screen
    MainScreen.mainloop()

GUI()