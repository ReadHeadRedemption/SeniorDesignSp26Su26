When parsing perhaps
- Use parser library to parse the file as I go
- as im working convert each parsed section to a gcode step

- need to take in max x/y of the printer 
- need to take in need to take in nozzle thickness




-------------------------------------------------------------------------------------------
- for lines divide the star x/y by the thickness of the nozzle to get the ammounts of passes needed to fill it in
    - for loop of width divided by the nozzle size
- for curves
    - need to look more into curve reading for pygerber and generation in gscrib
        - maybe split it into lines to go up and down? (integration reference)

