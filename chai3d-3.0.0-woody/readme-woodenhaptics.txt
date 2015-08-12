
   WoodenHaptics extension to Chai3D 3.0.0
   Written by Jonas Forsslund, KTH, 2014-12-08

   Unzip the contents of this zip-file's subfolder into your Chai3D folder
   (version used is chai3d-3.0.0-Makefiles.tar.gz (59930012 bytes))
   then run "make". 

   It will overwrite these files:
         Makefile.common.lin
         CHapticDeviceHandler.cpp 
         CGenericHapticDevice.h

   Note: To make sense of this you (currently) need to have a 
   Sensoray S826 DAQ PCIe card in your computer. You 
   also need to install the kernel module for that card.
   That driver is provided as source in /external/s826
   please read the readme.txt in that folder for instructions.

   This patch is licensed under: BSD

   Good luck and have fun!
   /Jonas (jofo02@kth.se)


