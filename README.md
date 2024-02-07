Usage:

After downloading this repo create a folder named "lokit" (folder where log files are stored) in the main folder (git doesn't track empty folders, and all file types pertaining to log data are ignored)

TODO: 

- Make the number of sensors and number of dancers dynamic (with a menu selection for number of dancers and sensors - this can depend on the already stored list [read further in the next parenthesis]),
  certain methods already implement a dynamic creation of resources based on these parameters but most of the program is hard coded
  (as are the serial numbers, maybe create a separate file for storing scanned sensors and associations, and implement a QR reader)
- fix visualizaitons to respect ordering (draw data of the same sensor in the same row)
- fix the visualization impossible to scroll to see all data plots on laptop monitors
- CALIBRATION!!!!!!!!!!!!!!!!!! sensors differ in calibration, try to stack up 2 or more sensors and look at orientation (euler) angles, the zero reference is different between sensors
- CALIBRATION 2!!!!!!!!!!!!!!!! fix acceleration and gyro (angular velocity) zero reference, to see this leave the sensors still, no matter the orientation (doesn't depend on force of gravity direction), the values are all diferent and non zero (not even close to zero, usually around 0.5 but even that differs between direcitons!!!!!!!!!!!!!!!!!!!!) as well as aforementioned angular velocities, they are NEVER zero (not even oscillating near it!!!)
  
 >> IF THERE IS NO COMPUTATIONAL SOLUTION FOR CALIBRATION, 3D PRINT A CASING STRUCTURE TO STACK THE SENSORS IN THE SAME ORIENTATION IN ORDER TO DO A CALIBRATION 

  

// ORIGINAL README:

Python 3.9 program code for Sonic Move project and dance theatre Minimi's Biodata Sonata performance premiering 15.11.2023 in Kuopio City Theatre, Finland. Human motion data recording through Movella (Xsens) motion capture sensors connected to an Awinda USB Dongle or an Awinda Station, sending it to Open Sound Control environment and displaying the live data graphically. It is also possible to run live recordings again by using log files and the GUI.

If you need to change one or more sensors, you can do it by editing `sensors.py` class dictionary `self.locations` starting from line 76. Also `plot_log()` - the last function of `sensors.py` - needs to be changed. The relevant section begins from line 277. 

Xsens Device API Linux and Windows Python wheel files with installation instructions are in the `wheels` folder.

Sonic Move project: https://uefconnect.uef.fi/en/group/sonic-move-creative-and-expressive-sonification-of-human-movement/

Minimi and Biodata Sonata: https://minimi.fi/fi_FI/biodatasonaatti-blogi/

Ylen artikkeli: https://yle.fi/a/74-20060177 (Finland's national public broadcasting company article)
