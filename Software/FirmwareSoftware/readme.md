# This is the folder that contains the C.I.P firmware software
The firmware is designed to handle controling lower level printer functions as well as to communicate with the attached S.B.C for controling imageing of the final PCB. 

## G-Code parseing
The g-code parseing code is designed to read in the gcode file given to the microcontroller. The parseing code will read each instruction line by line and call the appropriate function such as moving the motor when G1 is called. The parseing code will also read in all parameters in the line to control coordinate movement and extrusion control.

## Motor driver control
The motor driver control code will take the parsed g-code and send the related codes to the motor drivers along with the nessecary information. The motor driver code is primarilly reliant on the motor driver library to interface with the stepper motors and their encoders.

## Heater plate 
The heater plate code is designed to monitor and control the heating plate to cure the inks. The heater plate code will recieve sensor input about the plate temperature and heat the plate to whatever temperature may needed for the ink properties

## Displaying information to the user interface
The display code is designed to handle the human interface. This will display a menu of options for the user to start/stop inkings as well as to control tool head swapping and to control settings. 

## Handleing interrupts
The interrupt handeling code is designed to handle button presses on the user interface as well as interrupts due the the limit switches on the modified ender being triggered. These interrupts can be used for locating homeing positions in the firmware as well as manually stopping/starting/pausing the printer as well. 

## Communicating with the S.B.C
The firmware will handle communications with the single board comptuer to manage running the image recognition software to verify the printed PCB against the inteded direction. The indended image will also be shared with the S.B.C to give it an image to compare againt. If the image verification fails at any layer the ink laying will be stopped and the user will restart the print. 
