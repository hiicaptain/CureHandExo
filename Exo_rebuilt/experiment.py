"""----------------------------------------------------------------------------
@author:  Hao Yu, Edinburgh Centre for Robotics
Date of last update : 11/08/2023
Functions: The motor move 10 degree everytime I click the GO button on the GUI.
The motor angle and force values will be recorded. Angle need to be inputed on 
the GUI. 
----------------------------------------------------------------------------"""

# tkinter librairies
import tkinter as tk

# import the GUI
from build_gui import AppGUI

'''------------------------------- Main part -------------------------------'''
# Main which loops indefinitely 
def main():  
    root = tk.Tk()
    AppGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()


