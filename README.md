ofxSiliconRetina
=====================================

![alt text](https://github.com/federicohyo/ofxSiliconRetina/blob/master/ofxaddons_thumbnail.png "ofxSiliconRetina")

![alt text](https://github.com/federicohyo/ofxSiliconRetina/blob/master/docs/viewer.png "Viewer")

The statistics shows the display running only at about 60 fps. Nonetheless, the event-data are being timestamped with a single micro-seconds (us) resolution (see https://inilabs.com/products/dynamic-and-active-pixel-vision-sensor/). 


Introduction
------------
This addon let you interface any Dynamic Vision Sensor to openframeworks.
The Dynamic Vision Sensors are frame-free vision sensors invented, produced and developped by inilabs.com.

License
-------
ofxSiliconRetina is distributed under the [MIT License](https://en.wikipedia.org/wiki/MIT_License), and you might consider using this for your repository. By default, `license.md` contains a copy of the MIT license to which you can add your name and the year.

Installation
------------
Any steps necessary to install your addon. Optimally, this means just dropping the folder into the `openFrameworks/addons/` folder.

Dependencies - included in this package -
------------
This addons includes libcaer and libusb, if missing the one from your architecture. 
Please compile and copy them into the libs folder.

libcaer can be obained here: https://github.com/inilabs/libcaer
libusb can be obtained here: https://github.com/libusb/libusb

OfxPlugins
-----------

ofxStats can be obtained here https://github.com/tado/ofxStats

HotKeys
-----------

s: enable/disable stats
d: enable/disable dvs
a: enable/disable aps
i: enable/disable imu
s: start/stop recording files* 

* aedat3.1 file format: https://inilabs.com/support/software/fileformat/ 


Compatibility
------------
Tested on OF release 0.9.x 

Known issues
------------

Please select the correct device by changing the definition line in ofxSiliconRetina/src/ofxDVS.hpp

PLEASE SELECT SENSOR FAMILY  DVS128 / DAVIS (FX2/FX3)
#define DAVIS

Version history
------------

### Version 0.12 (27 May 2017):

Added shader example, added keyboards shortcuts, and reduced usb latency (now it uses threads)

### Version 0.11 (25 May 2017):

Implemented frames, imu6 events 

### Version 0.1 (25 May 2017):

Implemented polarity events for DVS128, DAVIS240, DAVIS346B 

