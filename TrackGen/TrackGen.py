from pathlib import Path
import os
import shutil
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import math
from EUFS_Trimmed import generate_track

#Checks file for the saved root
filePaths = Path(__file__).parent
IPGPathFile= open(filePaths / "Files/IPGpath.txt","r")
IPGPath = IPGPathFile.read()
IPGPathFile.close
trackType = "Skidpad"
trackPathFile = ""
road = ""
pandaPath = Image.open(filePaths /"Files/redPanda.png")
genTrackPath = Image.open(filePaths /"Files/generatedTrack.png")
skidTrackPath = Image.open(filePaths /"Files/Skidpad.png")
accelTrackPath = Image.open(filePaths /"Files/Acceleration.png")
BGColour = "#0C1821"
txtColour = "#CCC9DC"

def addCone (coneType, n, x, y, info):
    road.write(f"Traffic.{n}.Name = CN{n}\n")
    road.write("Traffic.{n}.Info:\n".format(n=n))
    road.write(f"\t{info}\n")
    road.write(f"Traffic.{n}.DetectMask = 1 1\n")
    road.write(f"Traffic.{n}.UpdRate = 200\n")
    road.write(f"Traffic.{n}.AutoDrv.UpdRate = 200\n")
    road.write(f"Traffic.{n}.Lighting = 0\n")
    road.write(f"Traffic.{n}.FreeMotion = 0\n")
    road.write(f"Traffic.{n}.TrailerName = \n")
    road.write(f"Traffic.{n}.Template.FName = {coneType}\n")
    road.write(f"Traffic.{n}.AutoDriver.FName = \n")
    road.write(f"Traffic.{n}.Routing.Type = Route\n")
    road.write(f"Traffic.{n}.Routing.ObjId = 0\n")
    road.write(f"Traffic.{n}.StartPos.Type = Global\n")
    road.write(f"Traffic.{n}.StartPos.ObjId = 0\n")
    road.write(f"Traffic.{n}.StartPos = {x} {y} 0\n")
    road.write(f"Traffic.{n}.StartPos.Orientation.Type = Absolute\n")
    road.write(f"Traffic.{n}.StartPos.Orientation = 0.0 0.0 0.0\n")
    road.write(f"Traffic.{n}.nMan = 0\n")
    road.write(f"Traffic.{n}.Man.Start.Velocity = 0.0\n")
    road.write(f"Traffic.{n}.Man.TreatAtEnd = FreezePos\n")

def correctPath(newPath):
    global IPGPath
    IPGPathFile = open(filePaths / "Files/IPGpath.txt","w")
    IPGPathFile.write(newPath)
    IPGPath = newPath
    IPGPathFile.close

def confirmType(Type):
    global trackType
    trackType = Type

def confirmSaveName(Name):
    global trackPathFile
    trackPathFile = Path(IPGPath + "/" + Name)
    if trackPathFile.exists():
        show_frame(pages["confirmOverwrite"])
    else:
        createFile()

def createFile():
    global road
    copyLoc = filePaths / "Files/default.txt"
    shutil.copy(copyLoc, IPGPath)
    oldFile = Path(IPGPath + "/default.txt")
    if trackPathFile.exists():
        trackPathFile.unlink()
    oldFile.rename(trackPathFile)
    with open(filePaths / ("Files/" + trackType + ".csv"), "r") as conesCSV:
        dataConeCSV = conesCSV.read()
        nCones = 0
        for line in dataConeCSV.splitlines():
            if line != "":
                nCones = nCones +1
    road = open(trackPathFile, "a")
    road.write("Traffic.N = " + str(nCones-1) + "\n")
    count = 0
    for line in dataConeCSV.splitlines():
        if line != "":
            colourPH = line.split(",")[0]
            xPH = line.split(",")[1]
            yPH = line.split(",")[2]
            
            # Try and get an orientation, if it doesn't exist, assume it will be 0
            try:
            	thetaPH = float(line.split(",")[3]) * 180/math.pi
            except:
            	thetaPH = 0.0
            	
            if colourPH == "smallYellow":
                addCone("TrafficCone_Small_Yellow", count, xPH, yPH, "Right Yellow Cone")
            elif colourPH == "smallBlue":
                addCone("TrafficCone_Small_Blue", count, xPH, yPH, "Left Blue Cone")
            elif colourPH == "smallOrange":
                addCone("TrafficCone_Small_Orange", count, xPH, yPH, "Small Orange Cone")
            elif colourPH == "largeOrange":
                addCone("TrafficCone_Large_Orange", count, xPH, yPH, "Large Orange Start Cone")
            elif colourPH == "car":
                    road.write("Vehicle.StartPos.Type = Global\n")
                    road.write("Vehicle.StartPos = " + str(xPH) + " " + str(yPH) + " 0\n")
                    # ISAAC: I took this out of the general vehicle info, but doesn't seem to be needed with global positioning
                    # Vehicle.StartPos.Orientation.Type = Relative
                    road.write("Vehicle.StartPos.Orientation = " + str(thetaPH) + "\n")
            count += 1
    road.close()
    show_frame(pages["mainPage"])
    saveToast("Track Saved")
def saveToast(status):
    toast = tk.Label(root, text=status, bg="black", fg="white", padx=10, pady=5)
    toast.place(relx=0.5, rely=0.95, anchor='s')
    root.after(2000, toast.destroy)

def show_frame(frame):
    if frame == pages["mainPage"]:
        currentRoot.config(text=IPGPath)
        currentTrack.config(text=trackType)
    if frame == pages["generateTrack"]:
        genTrackPath = Image.open(filePaths /"Files/generatedTrack.png")
        smallGenTrack = genTrackPath.resize((350, 350))
        genTrack = ImageTk.PhotoImage(smallGenTrack)
        genTrackLayoutLabel.config(image=genTrack)
        genTrackLayoutLabel.image = genTrack 
        f = open("Files/generatedSeed.txt", "r")
        seedPH = f.read()
        genTrackLabel.config(text=seedPH)
    frame.tkraise()    

def resetSliders():
    genInputMCR.set(3)
    genInputMF.set(7)
    genInputA.set(1)
    genInputSA.set(0.4)
    genInputRA.set(0.5)
    genInputM.set(0)
    genInputSSL.set(6)
    genInputSSD.set(2)
    genInputMinCS.set(3)
    genInputMaxCS.set(5)
    genInputTW.set(3)
    genInputCSB.set(0.5)
    genInputSCS.set(0.5)   

root = tk.Tk()
root.title("IPG Cone Generator")
root.geometry("1200x600")
root.configure(bg=BGColour)

style = ttk.Style(root)
style.theme_use("default")
style.configure("Rounded.TButton",
                background="#3498DB",
                foreground="white",
                padding=10,
                relief="flat",
                borderwidth=0,
                font=("Helvetica", 12))
style.map("Rounded.TButton",
          background=[('active', '#2980B9')])

smallPanda = pandaPath.resize((200, 200))
smallGenTrack = genTrackPath.resize((350, 350))
smallAccelTrack = accelTrackPath.resize((550, 250))
smallSkidTrack = skidTrackPath.resize((350, 350))

panda = ImageTk.PhotoImage(smallPanda)
genTrack = ImageTk.PhotoImage(smallGenTrack)
skidTrack = ImageTk.PhotoImage(smallSkidTrack)
acellTrack = ImageTk.PhotoImage(smallAccelTrack)

pages = {
    "mainPage": tk.Frame(root, bg=BGColour),
    "correctPath": tk.Frame(root, bg=BGColour),
    "confirmType": tk.Frame(root, bg=BGColour),
    "confirmSaveName": tk.Frame(root, bg=BGColour),
    "confirmOverwrite": tk.Frame(root, bg=BGColour),
    "generateTrack": tk.Frame(root, bg=BGColour),
    "skidpadTrack": tk.Frame(root, bg=BGColour),
    "accelerationTrack": tk.Frame(root, bg=BGColour),
}

for frame in pages.values():
    frame.grid(row=0, column=0, sticky='nsew')
    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=1)
    pandaLabel = tk.Label(frame, image=panda, bg=BGColour)
    pandaLabel.place(relx=1.0, rely=1.0, anchor='se', x=-10, y=30)

#Main page
header = tk.Label(pages["mainPage"], text='Cone Generator!', font=("Arial", 18), bg=BGColour, fg=txtColour)
currentRootTxt = tk.Label(pages["mainPage"], text="Please Check the path is your IPG Data/TestRun.", font=("Arial", 12), bg=BGColour, fg=txtColour)
currentRoot = tk.Label(pages["mainPage"], text=IPGPath, font=("Arial", 12), bg=BGColour, fg=txtColour)
currentTrack = tk.Label(pages["mainPage"], text=f"Current Track: {trackType}", font=("Arial", 12), bg=BGColour, fg=txtColour)
changePathButton = ttk.Button(pages["mainPage"], text='Change Root', style="Rounded.TButton", command=lambda: show_frame(pages["correctPath"]))
confirmTypeButton = ttk.Button(pages["mainPage"], text='Select Track Type', style="Rounded.TButton", command=lambda: show_frame(pages["confirmType"]))
confirmSaveNameButton = ttk.Button(pages["mainPage"], text='Save Track', style="Rounded.TButton", command=lambda: show_frame(pages["confirmSaveName"]))

header.pack(pady=10)
currentRootTxt.pack(pady=5)
currentRoot.pack(pady=5)
changePathButton.pack(pady=10)
confirmTypeButton.pack(pady=10)
currentTrack.pack(pady=5)
confirmSaveNameButton.pack(pady=10)

#Correct path page
correctPathLabel = tk.Label(pages["correctPath"], text='Enter your new IPG Data/TestRun path:', font=("Arial", 12), bg=BGColour, fg=txtColour)
correctPathEntry = tk.Entry(pages["correctPath"], font=("Arial", 12), bg="white", fg="black", relief="flat", highlightthickness=1, highlightbackground="#3498DB", insertbackground="black")
correctPathButtonSave = ttk.Button(pages["correctPath"], text='Save', style="Rounded.TButton", command=lambda: [correctPath(correctPathEntry.get()), show_frame(pages["mainPage"]), saveToast("Root Saved")])
correctPathButtonCancel = ttk.Button(pages["correctPath"], text='Cancel', style="Rounded.TButton", command=lambda: [show_frame(pages["mainPage"]), saveToast("Root NOT Saved")])
correctPathLabel.pack(pady=10)
correctPathEntry.pack(pady=5)
correctPathButtonSave.pack(pady=10)
correctPathButtonCancel.pack(pady=5)

#Tracktype page
confirmTypeLabel = tk.Label(pages["confirmType"], text='What type of track would you like to generate?', font=("Arial", 12), bg=BGColour, fg=txtColour)
confirmTypeButtonSkid = ttk.Button(pages["confirmType"], text='Skidpad', style="Rounded.TButton", command=lambda: show_frame(pages["skidpadTrack"]))
confirmTypeButtonAcell = ttk.Button(pages["confirmType"], text='Acceleration', style="Rounded.TButton", command=lambda: show_frame(pages["accelerationTrack"]))
confirmTypeButtonRnd = ttk.Button(pages["confirmType"], text='Generate Track', style="Rounded.TButton", command=lambda: show_frame(pages["generateTrack"]))
confirmTypeButtonCancel = ttk.Button(pages["confirmType"], text='Cancel', style="Rounded.TButton", command=lambda: show_frame(pages["mainPage"]))
confirmTypeLabel.pack(pady=10)
confirmTypeButtonSkid.pack(pady=10)
confirmTypeButtonAcell.pack(pady=10)
confirmTypeButtonRnd.pack(pady=10)
confirmTypeButtonCancel.pack(pady=10)

#Skidpad page
skidTrackLabel =  tk.Label(pages["skidpadTrack"], font=("Arial", 12), bg=BGColour, fg=txtColour)
skidTrackLayoutLabel = tk.Label(pages["skidpadTrack"], image=skidTrack, bg=BGColour)
skidTrackSave = ttk.Button(pages["skidpadTrack"], text='Set', style="Rounded.TButton", command=lambda: [confirmType("Skidpad"), show_frame(pages["mainPage"]), saveToast("Track Type Set")])
skidCancel = ttk.Button(pages["skidpadTrack"], text='Cancel', style="Rounded.TButton", command=lambda: [show_frame(pages["mainPage"]), saveToast("NOT Saved")])
skidTrackLabel.pack(pady=10)
skidTrackLayoutLabel.place(relx=0.5, rely=0.4, anchor='center')
skidTrackSave.place(relx=0.65, rely=0.85, anchor="center")
skidCancel.place(relx=0.35, rely=0.85, anchor="center")

#AccelerationTrack page
accelTrackLabel =  tk.Label(pages["accelerationTrack"], font=("Arial", 12), bg=BGColour, fg=txtColour)
accelTrackLayoutLabel = tk.Label(pages["accelerationTrack"], image=acellTrack, bg=BGColour)
accelTrackSave = ttk.Button(pages["accelerationTrack"], text='Set', style="Rounded.TButton", command=lambda: [confirmType("Acceleration"), show_frame(pages["mainPage"]), saveToast("Track Type Set")])
accelCancel = ttk.Button(pages["accelerationTrack"], text='Cancel', style="Rounded.TButton", command=lambda: [show_frame(pages["mainPage"]), saveToast("NOT Saved")])
accelTrackLabel.pack(pady=10)
accelTrackLayoutLabel.place(relx=0.5, rely=0.4, anchor='center')
accelTrackSave.place(relx=0.65, rely=0.85, anchor="center")
accelCancel.place(relx=0.35, rely=0.85, anchor="center")

#Gen track page
genTrackLabel =  tk.Label(pages["generateTrack"], font=("Arial", 12), bg=BGColour, fg=txtColour)
genTrackLayoutLabel = tk.Label(pages["generateTrack"], image=genTrack, bg=BGColour)
genTrackSave = ttk.Button(pages["generateTrack"], text='Set', style="Rounded.TButton", command=lambda: [confirmType("Generated Track"), show_frame(pages["mainPage"]), saveToast("Track Type Set")])
genCancel = ttk.Button(pages["generateTrack"], text='Cancel', style="Rounded.TButton", command=lambda: [show_frame(pages["mainPage"]), saveToast("NOT Saved")])
genReset = ttk.Button(pages["generateTrack"], text='Reset', style="Rounded.TButton", command=lambda: [resetSliders(), saveToast("Settings Reset")])
genTrackRegen = ttk.Button(pages["generateTrack"], text='Regen', style="Rounded.TButton", command=lambda: [generate_track(int(genInputSeed.get()) if genInputSeed.get().isdigit() else None, 
                                                                                                                          genInputMCR.get(),
                                                                                                                          genInputMF.get(),
                                                                                                                          genInputA.get(),
                                                                                                                          genInputSA.get(),
                                                                                                                          genInputRA.get(),
                                                                                                                          genInputM.get(),
                                                                                                                          genInputSSL.get(),
                                                                                                                          genInputSSD.get(),
                                                                                                                          genInputMinCS.get(),
                                                                                                                          genInputMaxCS.get(),
                                                                                                                          genInputTW.get(),
                                                                                                                          genInputCSB.get(),
                                                                                                                          genInputSCS.get()),
                                                                                                                          saveToast("Track Regenerated"),show_frame(pages["generateTrack"])])
genTrackSetSeed = tk.Label(pages["generateTrack"], text='Set seed (numbers) or leave blank for random seed.', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputSeed = tk.Entry(pages["generateTrack"], font=("Arial", 12), bg="white", fg="black", relief="flat", highlightthickness=1, highlightbackground="#3498DB", insertbackground="black")
genInputMCRTxt = tk.Label(pages["generateTrack"], text='Min Corner Radius:', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputMCR = tk.Scale(pages["generateTrack"],  from_=2, to=20,resolution=1, orient=tk.HORIZONTAL, bg=BGColour,fg=txtColour,troughcolor=BGColour,activebackground=txtColour,highlightbackground=BGColour)
genInputMFTxt = tk.Label(pages["generateTrack"], text='Max Frequency:', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputMF = tk.Scale(pages["generateTrack"],  from_=1, to=15,resolution=1, orient=tk.HORIZONTAL, bg=BGColour,fg=txtColour,troughcolor=BGColour,activebackground=txtColour,highlightbackground=BGColour)
genInputATxt = tk.Label(pages["generateTrack"], text='Amplitude (/3):', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputA = tk.Scale(pages["generateTrack"],  from_=1, to=1.5,resolution=0.05, orient=tk.HORIZONTAL, bg=BGColour,fg=txtColour,troughcolor=BGColour,activebackground=txtColour,highlightbackground=BGColour)
genInputSATxt = tk.Label(pages["generateTrack"], text='Starting Amplitude:', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputSA = tk.Scale(pages["generateTrack"],  from_=0.1, to=0.5,resolution=0.05, orient=tk.HORIZONTAL, bg=BGColour,fg=txtColour,troughcolor=BGColour,activebackground=txtColour,highlightbackground=BGColour)
genInputRATxt = tk.Label(pages["generateTrack"], text='Relative Accuracy (/100):', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputRA = tk.Scale(pages["generateTrack"],  from_=0.1, to=1,resolution=0.1, orient=tk.HORIZONTAL, bg=BGColour,fg=txtColour,troughcolor=BGColour,activebackground=txtColour,highlightbackground=BGColour)
genInputMTxt = tk.Label(pages["generateTrack"], text='Margin:', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputM = tk.Scale(pages["generateTrack"],  from_=0, to=5,resolution=0.5, orient=tk.HORIZONTAL, bg=BGColour,fg=txtColour,troughcolor=BGColour,activebackground=txtColour,highlightbackground=BGColour)
genInputSSLTxt = tk.Label(pages["generateTrack"], text='Starting Straight Length:', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputSSL = tk.Scale(pages["generateTrack"],  from_=1, to=20,resolution=1, orient=tk.HORIZONTAL, bg=BGColour,fg=txtColour,troughcolor=BGColour,activebackground=txtColour,highlightbackground=BGColour)
genInputSSDTxt = tk.Label(pages["generateTrack"], text='Starting Straight Downsample:', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputSSD = tk.Scale(pages["generateTrack"],  from_=1, to=4,resolution=1, orient=tk.HORIZONTAL, bg=BGColour,fg=txtColour,troughcolor=BGColour,activebackground=txtColour,highlightbackground=BGColour)
genInputMinCSTxt = tk.Label(pages["generateTrack"], text='Min Cone Spacing(π/16):', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputMinCS = tk.Scale(pages["generateTrack"],  from_=1, to=5,resolution=0.5, orient=tk.HORIZONTAL, bg=BGColour,fg=txtColour,troughcolor=BGColour,activebackground=txtColour,highlightbackground=BGColour)
genInputMaxCSTxt = tk.Label(pages["generateTrack"], text='Max Cone Spacing:', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputMaxCS = tk.Scale(pages["generateTrack"],  from_=1, to=10,resolution=1, orient=tk.HORIZONTAL, bg=BGColour,fg=txtColour,troughcolor=BGColour,activebackground=txtColour,highlightbackground=BGColour)
genInputTWTxt = tk.Label(pages["generateTrack"], text='Track Width:', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputTW = tk.Scale(pages["generateTrack"],  from_=2, to=7,resolution=0.5, orient=tk.HORIZONTAL, bg=BGColour,fg=txtColour,troughcolor=BGColour,activebackground=txtColour,highlightbackground=BGColour)
genInputCSBTxt = tk.Label(pages["generateTrack"], text='Cone Spacing Bias:', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputCSB = tk.Scale(pages["generateTrack"],  from_=-2, to=2,resolution=0.5, orient=tk.HORIZONTAL, bg=BGColour,fg=txtColour,troughcolor=BGColour,activebackground=txtColour,highlightbackground=BGColour)
genInputSCSTxt = tk.Label(pages["generateTrack"], text='Starting Cone Spacing:', font=("Arial", 12), bg=BGColour, fg=txtColour)
genInputSCS = tk.Scale(pages["generateTrack"],  from_=0.1, to=1,resolution=0.1, orient=tk.HORIZONTAL, bg=BGColour,fg=txtColour,troughcolor=BGColour,activebackground=txtColour,highlightbackground=BGColour)


genTrackLabel.pack(pady=10)
genTrackLayoutLabel.place(relx=0.2, rely=0.4, anchor='center')
genReset.place(relx=0.5,rely=0.55, anchor="center")

genInputMCRTxt.place(relx=0.45,rely=0.1, anchor="center")
genInputMCR.place(relx=0.58,rely=0.08, anchor="center")
genInputMCR.set(3)
genInputMFTxt.place(relx=0.45,rely=0.17, anchor="center")
genInputMF.place(relx=0.58,rely=0.15, anchor="center")
genInputMF.set(7)
genInputATxt.place(relx=0.45,rely=0.24, anchor="center")
genInputA.place(relx=0.58,rely=0.22, anchor="center")
genInputA.set(1)
genInputSATxt.place(relx=0.45,rely=0.31, anchor="center")
genInputSA.place(relx=0.58,rely=0.29, anchor="center")
genInputSA.set(0.4)
genInputRATxt.place(relx=0.45,rely=0.38, anchor="center")
genInputRA.place(relx=0.58,rely=0.36, anchor="center")
genInputRA.set(0.5)
genInputMTxt.place(relx=0.45,rely=0.45, anchor="center")
genInputM.place(relx=0.58,rely=0.43, anchor="center")
genInputM.set(0)
genInputSSLTxt.place(relx=0.75,rely=0.1, anchor="center")
genInputSSL.place(relx=0.9,rely=0.08, anchor="center")
genInputSSL.set(6)
genInputSSDTxt.place(relx=0.75,rely=0.17, anchor="center")
genInputSSD.place(relx=0.9,rely=0.15, anchor="center")
genInputSSD.set(2)
genInputMinCSTxt.place(relx=0.75,rely=0.24, anchor="center")
genInputMinCS.place(relx=0.9,rely=0.22, anchor="center")
genInputMinCS.set(3)
genInputMaxCSTxt.place(relx=0.75,rely=0.31, anchor="center")
genInputMaxCS.place(relx=0.9,rely=0.29, anchor="center")
genInputMaxCS.set(5)
genInputTWTxt.place(relx=0.75,rely=0.38, anchor="center")
genInputTW.place(relx=0.9,rely=0.36, anchor="center")
genInputTW.set(3)
genInputCSBTxt.place(relx=0.75,rely=0.45, anchor="center")
genInputCSB.place(relx=0.9,rely=0.43, anchor="center")
genInputCSB.set(0.5)
genInputSCSTxt.place(relx=0.75,rely=0.52, anchor="center")
genInputSCS.place(relx=0.9,rely=0.50, anchor="center")
genInputSCS.set(0.5)

genInputSeed.place(relx=0.5, rely=0.72, anchor="center")
genTrackSetSeed.place(relx=0.50, rely=0.78, anchor="center")
genTrackSave.place(relx=0.5, rely=0.95, anchor="center")
genTrackRegen.place(relx=0.65, rely=0.85, anchor="center")
genCancel.place(relx=0.35, rely=0.85, anchor="center")

#Confirm save page
confirmSaveNameLabel = tk.Label(pages["confirmSaveName"], text='What would you like to name your track?', font=("Arial", 12), bg=BGColour, fg=txtColour)
confirmSaveNameEntry = tk.Entry(pages["confirmSaveName"], font=("Arial", 12), bg="white", fg="black", relief="flat", highlightthickness=1, highlightbackground="#3498DB", insertbackground="black")
confirmSaveNameButton = ttk.Button(pages["confirmSaveName"], text='Save', style="Rounded.TButton", command=lambda: confirmSaveName(confirmSaveNameEntry.get()))
confirmSaveNameCancel = ttk.Button(pages["confirmSaveName"], text='Cancel', style="Rounded.TButton", command=lambda: [show_frame(pages["mainPage"]), saveToast("Track NOT Saved")])
confirmSaveNameLabel.pack(pady=10)
confirmSaveNameEntry.pack(pady=5)
confirmSaveNameButton.pack(pady=10)
confirmSaveNameCancel.pack(pady=5)

#Confirm overwrite page
confirmOverwriteLabel = tk.Label(pages["confirmOverwrite"], text='File already exists. Would you like to overwrite it?', font=("Arial", 12), bg=BGColour, fg=txtColour)
confirmOverwriteButtonYes = ttk.Button(pages["confirmOverwrite"], text='Yes', style="Rounded.TButton", command=lambda: [createFile(), show_frame(pages["mainPage"]), saveToast("Track Saved")])
confirmOverwriteButtonNo = ttk.Button(pages["confirmOverwrite"], text='No', style="Rounded.TButton", command=lambda: [show_frame(pages["mainPage"]), saveToast("Track NOT Saved")])
confirmOverwriteLabel.pack(pady=10)
confirmOverwriteButtonYes.pack(pady=10)
confirmOverwriteButtonNo.pack(pady=5)

pages["mainPage"].tkraise()
root.mainloop()

show_frame(pages["mainPage"])
root.mainloop()



