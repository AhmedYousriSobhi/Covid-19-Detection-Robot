# Covid 19 Detection Robot
### D6t-44L-06: (Tested Alone and Worked)
   Check temperature using d6t-44l-06 Omron Thermal sensor.
   
   This sensor is 4*4 pixels, max range of detection is about 2.5m (The position of thermal sensor is Horizontal) from the sensor.
   
   When temperature higher than thereshold is detected then using Camera to detect if this high temp is from persons or not.
   
   Algorithm used is "Max area of island" : https://leetcode.com/articles/max-area-of-island/

## Pi Camera: (Tested Alone and Worked)
The pi Camera uses Real-Time object detection using OpenCV DNN "Deep Neural Networks"

site: https://docs.opencv.org/3.4/d2/d58/tutorial_table_of_content_dnn.html

It takes an image frame from the camera then adjust the size of it to be forwared to the DNN model and predict which object is detected.

The DNN object detecton detect up to 90 Class, our main object in here is Human, so we will ignore any other class.

Note: Localization is not included in this project, so if there are high temperature in front of the robot coming from high object non-human,
and there is a human next to it, so the robot will assume that the high temprature is coming from that person.
so to fix that problem, we need to add localization here.

### Stepper Motor: (Untested)

Using Stepper motor to adjust the height of Camera and Thermal sensor.

Stepper Motor: You can adjust its step according to what you prefere.

Step Mode: Full step (Mode2=0, Mode1=0, Mode0=0) with current limited to 71% of the maximum current.

For more accuracy you can use pigpio libarary instead of time.sleep as pigpio uses which provides hardware based PWM timing.

Before use, the PiGPIO daemon must be started using sudo pigpiod from a terminal.
sudo pigpiod

Two Stepper motor drivers connected in parallel to the same Two pins as one drivers requires arount 20uA, and the pin in raspberrye max output current is 16mA.

Datasheet: https://www.pololu.com/file/0J450/a4988_DMOS_microstepping_driver_with_translator.pdf

### DC Motors: (Untested)

Using 4 DC motors for motion controlled by two cytron dual channels motor drivers

### GPS: (Untested)

Gps connected by UART pins 14:TX,15:RX :

Using  Serial0, so check which is connected to serial0? ttyAMA0 or ttys0? then disable it.
*pynmeas libarary must be installed (links in sites sections.)

Txt File Creation: https://www.guru99.com/reading-and-writing-files-in-python.html    

### Uploading photos to a shareable folder in your google drive: (Tested alone & Worked)

First fill the information required in "Configuration.config" file.

To git your json secert file, follow the steps in this help guide "How to access google drive using services account json file"
link: https://help.talend.com/reader/E3i03eb7IpvsigwC58fxQg/ol2OwTHmFbDiMjQl3ES5QA

Github code in sites section below.

In output Directory: do not end the Path with '/'

https://www.youtube.com/embed/TPatxsJOXbY

### Sites:

DNN Code: https://heartbeat.fritz.ai/real-time-object-detection-on-raspberry-pi-using-opencv-dnn-98827255fa60

D6t-44l-06 Code: https://github.com/avninja/omrond6t

Stepper Motor: https://www.rototron.info/raspberry-pi-stepper-motor-tutorial/

Ublox M6 GPS: https://github.com/Knio/pynmea2

Photo Upload to google drive: https://github.com/MartinStolle/pi-upload-google-drive

## Schematic:
![ThermalRobot_bb_V5](https://user-images.githubusercontent.com/66730765/85927688-0409f780-b8a8-11ea-95f8-7a8b9b2f19c1.png)

##Notes:

Untested was just because i didn't have the hardware to check if the stepper/motors are moving or not.

Configuring the Motion will differ dependent on the Hardware used like types of Motors, wires connection. so you need to reconfig all the hardware connection to work perfectly with your robot design.

Uploading takes about 1 min to upload, depending on the internet speed and google response.
Video for capturing and Uploading :
https://youtu.be/TPatxsJOXbY

## Mechanical Part:
This robot was designed by Mechanical Engineer called #"Ahmed Tarek".
![106658532_2152473851564794_2022983822602779078_n](https://user-images.githubusercontent.com/66730765/97861062-da585580-1d0b-11eb-8de6-344703d110b9.jpg)
![107072873_2152474001564779_2066270111311350289_n](https://user-images.githubusercontent.com/66730765/97861180-0d9ae480-1d0c-11eb-9dbf-7e2d7960d26a.jpg)
![106031256_2152473994898113_2292379645124385839_n](https://user-images.githubusercontent.com/66730765/97861201-14295c00-1d0c-11eb-9e2c-5e1383115c4f.jpg)
![106908668_2152473774898135_31862728186524255_n](https://user-images.githubusercontent.com/66730765/97861229-1c819700-1d0c-11eb-9af6-e5d622abc94a.jpg)
![106995696_2152473871564792_3751203117740965211_o](https://user-images.githubusercontent.com/66730765/97861256-22777800-1d0c-11eb-8438-6b39f47b93e0.jpg)
![106613984_2152473818231464_6954148417154197432_n](https://user-images.githubusercontent.com/66730765/97861274-26a39580-1d0c-11eb-8111-288b15a9f4de.jpg)

