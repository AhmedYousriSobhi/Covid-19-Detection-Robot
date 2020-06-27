# Thermal-Robot


   ### D6t-44L-06: (Tested Alone and Worked)
        -Check temperature using d6t-44l-06 Omron Thermal sensor.
        -This sensor is 4*4 pixels, max range of detection is about 2.5m (The position of thermal sensor is Horizontal) from the sensor.
        -When temperature higher than thereshold is detected then using Camera to detect if this high temp is from persons or not.
        -Algorithm used is "Max area of island" : #https://leetcode.com/articles/max-area-of-island/

   ### Pi Camera: (Tested Alone and Worked)
        -The pi Camera uses Real-Time object detection using OpenCV DNN "Deep Neural Networks"
            site: https://docs.opencv.org/3.4/d2/d58/tutorial_table_of_content_dnn.html
        -It takes an image frame from the camera then adjust the size of it to be forwared to the DNN model and predict which object is detected.
        -The DNN object detecton detect up to 90 Class, our main object in here is Human, so we will ignore any other class.
        -Note: Localization is not included in this project, so if there are high temperature in front of the robot coming from high object non-human,
          and there is a human next to it, so the robot will assume that the high temprature is coming from that person.
          so to fix that problem, we need to add localization here.

   ### Stepper Motor: (Untested)
        -Using Stepper motor to adjust the height of Camera and Thermal sensor.
        -Stepper Motor: You can adjust its step according to what you prefere.
            -Step Mode: Full step (Mode2=0, Mode1=0, Mode0=0) with current limited to 71% of the maximum current.
            -For more accuracy you can use pigpio libarary instead of time.sleep as pigpio uses which provides hardware based PWM timing.
            -Before use, the PiGPIO daemon must be started using sudo pigpiod from a terminal.
                sudo pigpiod
        -Two Stepper motor drivers connected in parallel to the same Two pins as one drivers requires arount 20uA, and the pin in raspberrye max output current is 16mA.
    
   ### DC Motors: (Untested)
        -Using 4 DC motors for motion controlled by two cytron dual channels motor drivers

   ### GPS: (Untested)
        -Gps connected by UART pins 14:TX,15:RX :
            -Using  Serial0, so check which is connected to serial0? ttyAMA0 or ttys0? then disable it.
            *pynmeas libarary must be installed (links in sites sections.)
        Txt File Creation: https://www.guru99.com/reading-and-writing-files-in-python.html    

    ###Uploading photos to a shareable folder in your google drive: (Tested alone & Worked)
        -First fill the information required in "image-upload.config" file.
        -To git your json secert file, follow the steps in this help guide "How to access google drive using services account json file"
        link: https://help.talend.com/reader/E3i03eb7IpvsigwC58fxQg/ol2OwTHmFbDiMjQl3ES5QA
        -Github code in sites section below.

    In output Directory: do not end the Path with '/'

   ### Sites:
        DNN Code: https://heartbeat.fritz.ai/real-time-object-detection-on-raspberry-pi-using-opencv-dnn-98827255fa60
        D6t-44l-06 Code: https://github.com/avninja/omrond6t
        Stepper Motor: https://www.rototron.info/raspberry-pi-stepper-motor-tutorial/
        Ublox M6 GPS: https://github.com/Knio/pynmea2
        Photo Upload to google drive: https://github.com/MartinStolle/pi-upload-google-drive
