REQUIREMENTS
-Ensure that under uwe-ipg-sim/Data/Road/ there is a file called "Do_Not_Change.rd5". If this file is missing, there is a copy inside the TrackGen folder.
-From the TrackGen folder, open terminal and run the requirements.txt (pip install -r requirements.txt)
-If you do not have access to IPG Carmaker, you can still generate .csv files. Post generation a copy will be stored in TrackGen/Files folder.

USAGE
-From the TrackGen folder, open terminal and run (python3 TrackGen.py)
-BEFORE SAVING ANY TRACK, ensure the test run path is correct, it is shown at the top of the homepage. It should look something like (/home/harry/Documents/GitHub/uwe-ipg-sim/Data/TestRun).
-The current track type is shown under the "Select Track Type" button. 
 To generate a track press "Select Track Type". From here, you can generate a random track, skidpad, or acceleration track.
-When generating random tracks I highly recommend leaving settings fileds blank for defult settings, its extreamly easy to cause IPG or the generator to crash with wacky tracks. Seeds do not carry setting information (currently), so ensure if you wish to share a seed you share the settings used. 
-To Save the track, press "set", this will return you to the homepage, then press "Save Track", enter a file name, and press "save".
-If there is a file conflict, you will be warned and have the choice to overwrite it.
