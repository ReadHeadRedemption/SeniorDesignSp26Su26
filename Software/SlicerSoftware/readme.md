# This is the folder that contains the C.I.P slicer software
The slicer is designed to parse the gerber file into g-code for the printer. The slicer will take in the gerber as well as paremeters such as printer information, ink properties, and layer attributes to determine how many layers need to be printed as well as when a indulator layer needs to be added to prevent shorts/unintended connections

## Gerber file information
The slicer code will take in a series of gerber files read their information to decode the image and then in combination with printer and ink information design a gcode script to convert the gerber. 
## Printer information
The printer information will contain the build area as well as the location of the print head and the nozzel size on the extruder to work with the gerber file information and ink properties to 
## Ink properties
Ink properties will contain information s
## layer count 