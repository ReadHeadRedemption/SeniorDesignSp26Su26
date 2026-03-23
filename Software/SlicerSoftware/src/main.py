import configFunctions as cF
from GUI import GUI

def main():
    cF.defConfig()
    print("Default configuration set. Starting GUI...")
    GUI()

if __name__ == "__main__":
    main()