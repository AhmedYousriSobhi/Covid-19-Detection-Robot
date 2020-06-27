"""
    D6t-44L-06: (Tested Alone and Worked)
        -Check temperature using d6t-44l-06 Omron Thermal sensor.
        -This sensor is 4*4 pixels, max range of detection is about 2.5m (The position of thermal sensor is Horizontal) from the sensor.
        -When temperature higher than thereshold is detected then using Camera to detect if this high temp is from persons or not.
        -Algorithm used is "Max area of island" : #https://leetcode.com/articles/max-area-of-island/

    Pi Camera: (Tested Alone and Worked)
        -The pi Camera uses Real-Time object detection using OpenCV DNN "Deep Neural Networks"
            site: https://docs.opencv.org/3.4/d2/d58/tutorial_table_of_content_dnn.html
        -It takes an image frame from the camera then adjust the size of it to be forwared to the DNN model and predict which object is detected.
        -The DNN object detecton detect up to 90 Class, our main object in here is Human, so we will ignore any other class.
        -Note: Localization is not included in this project, so if there are high temperature in front of the robot coming from high object non-human,
          and there is a human next to it, so the robot will assume that the high temprature is coming from that person.
          so to fix that problem, we need to add localization here.

    Stepper Motor: (Untested)
        -Using Stepper motor to adjust the height of Camera and Thermal sensor.
        -Stepper Motor: You can adjust its step according to what you prefere.
            -Step Mode: Full step (Mode2=0, Mode1=0, Mode0=0) with current limited to 71% of the maximum current.
            -For more accuracy you can use pigpio libarary instead of time.sleep as pigpio uses which provides hardware based PWM timing.
            -Before use, the PiGPIO daemon must be started using sudo pigpiod from a terminal.
                sudo pigpiod
        -Two Stepper motor drivers connected in parallel to the same Two pins as one drivers requires arount 20uA, and the pin in raspberrye max output current is 16mA.
    
    DC Motors: (Untested)
        -Using 4 DC motors for motion controlled by two cytron dual channels motor drivers

    GPS: (Untested)
        -Gps connected by UART pins 14:TX,15:RX :
            -Using  Serial0, so check which is connected to serial0? ttyAMA0 or ttys0? then disable it.
            *pynmeas libarary must be installed (links in sites sections.)
        Txt File Creation: https://www.guru99.com/reading-and-writing-files-in-python.html    

    Uploading photos to a shareable folder in your google drive: (Tested alone & Worked)
        -First fill the information required in "image-upload.config" file.
        -To git your json secert file, follow the steps in this help guide "How to access google drive using services account json file"
        link: https://help.talend.com/reader/E3i03eb7IpvsigwC58fxQg/ol2OwTHmFbDiMjQl3ES5QA
        -Github code in sites section below.

    In output Directory: do not end the Path with '/'

    Sites:
        DNN Code: https://heartbeat.fritz.ai/real-time-object-detection-on-raspberry-pi-using-opencv-dnn-98827255fa60
        D6t-44l-06 Code: https://github.com/avninja/omrond6t
        Stepper Motor: https://www.rototron.info/raspberry-pi-stepper-motor-tutorial/
        Ublox M6 GPS: https://github.com/Knio/pynmea2
        Photo Upload to google drive: https://github.com/MartinStolle/pi-upload-google-drive

"""
import pigpio, time, crcmod.predefined, smbus
#Libararies needed for Camera:
import cv2
from picamera import PiCamera
#Libararies needed for Gps:
import serial, string, pynmea2
#Common Libararies For Motor dirvers:
import RPi.GPIO as IO
#Common Libararies used:
from time import sleep
#Libararies needed to upload Photos to Google Drive:
import configparser
import datetime
import logging
import os
import threading
import httplib2
# Required oauth2client==3.0.0
from apiclient import discovery
from apiclient import errors
from apiclient.http import MediaFileUpload

#Global Variables:
#Variable definition used in some functions:
NumOfFrames = 5  #used in checkPersons() function.
imageIndex = -1  #used in checkPersons() function.
txtIndex = -1
THERESHOLD_TEMPERATURE = 40  #used in compare() function.
THERESHOLD_PIXELS = 3 #determine if there is more than one high temp pixel.
outputDirectory = ""  #used in read_configuration() function.
ALARM_PIN = 13 #Alarm Pin.
LFM_PWM_PIN = 13 #Left Front Motor (PWM0).
LBM_PWM_PIN = 19 #Left Back Motor (PWM0).
RFM_PWM_PIN = 12 #Left Front Motor (PWM1).
RBM_PWM_PIN = 18 #Left Back Motor (PWM0).
LFM_DIR_PIN = 11 #Left Front Motor (PWM0).
LBM_DIR_PIN = 15 #Left Back Motor (PWM0).
RFM_DIR_PIN = 16 #Left Front Motor (PWM1).
RBM_DIR_PIN = 22 #Left Back Motor (PWM0).
FRQ = 100 #Used to adjust PWM.
strodLength = 150  #Length of (in mm) strode that hold the camera and the thermal sensor.
FULLSTEPLENGTH = 20 #Full step of steeper motor (in mm).
stepperDir = 26 #Step pin in stepper driver.
stepperStep = 24 #Direction pin in stepper driver.
"""******************************************************************************************************************"""
#Thermal Sensor:
class OmronD6T(object):
    def __init__(self, rasPiChannel=1, omronAddress=0x0a, arraySize=16):
        self.MAX_RETRIES = 5
        self.roomTemp = 0
        self.omronAddress = omronAddress
        self.arraySize = arraySize
        self.BUFFER_LENGTH=(arraySize * 2) + 3       # data buffer size
        self.CRC_ERROR_BIT = 0x04                    # the third bit
        self.CRC = 0xa4 / (16/arraySize)                             # if you take the CRC of the whole word including the PEC, this is what you get
        self.piGPIO = pigpio.pi()
        self.piGPIOver = self.piGPIO.get_pigpio_version()
        self.i2cBus = smbus.SMBus(1)
        time.sleep(0.1)                # Wait
        # initialize the device based on Omron's appnote 1
        self.retries = 0
        self.result = 0
        for i in range(0,self.MAX_RETRIES):
            time.sleep(0.05)                               # Wait a short time
            self.handle = self.piGPIO.i2c_open(rasPiChannel, omronAddress) # open Omron D6T device at address 0x0a on bus 1
            #print(self.handle)
            if self.handle > 0:
                self.result = self.i2cBus.write_byte(self.omronAddress,0x4c)
                break
            else:
                print('')
                print('***** Omron init error ***** handle='+str(self.handle)+' retries='+str(self.retries))
                self.retries += 1

    # function to read the omron temperature array
    def read(self):
        self.temperature_data_raw=[0]*self.BUFFER_LENGTH
        self.temperature=[0.0]*self.arraySize         # holds the recently measured temperature
        self.values=[0]*self.BUFFER_LENGTH

        # read the temperature data stream - if errors, retry
        retries = 0
        for i in range(0,self.MAX_RETRIES):
            time.sleep(0.05)                               # Wait a short time
            (self.bytes_read, self.temperature_data_raw) = self.piGPIO.i2c_read_device(self.handle, self.BUFFER_LENGTH)

            # Handle i2c error transmissions
            if self.bytes_read != self.BUFFER_LENGTH:
                print ('')
                print ('***** Omron Byte Count error ***** - bytes read: '+str(self.bytes_read))
                self.retries += 1                # start counting the number of times to retry the transmission

            if self.bytes_read == self.BUFFER_LENGTH:
                # good byte count, now check PEC

                t = (self.temperature_data_raw[1] << 8) | self.temperature_data_raw[0]
                self.tPATc = float(t)/10
                # if (degree_unit == 'F'):
                self.roomTemp = self.tPATc #self.C_to_F(self.tPATc)

                # Convert Raw Values to Temperature ('F)
                a = 0
                for i in range(2, len(self.temperature_data_raw)-2, 2):
                    self.temperature[a] = float((self.temperature_data_raw[i+1] << 8) | self.temperature_data_raw[i])/10 #self.C_to_F(float((self.temperature_data_raw[i+1] << 8) | self.temperature_data_raw[i])/10)
                    a += 1

                # Calculate the CRC error check code
                # PEC (packet error code) byte is appended at the end of each transaction. The byte is calculated as CRC-8 checksum, calculated over the entire message including the address and read/write bit. The polynomial used is x8+x2+x+1 (the CRC-8-ATM HEC algorithm, initialized to zero)
                self.crc8_func = crcmod.predefined.mkCrcFun('crc-8')
                for i in range(0,self.bytes_read):
                    self.values[i] = self.temperature_data_raw[i]

                self.string = "".join(chr(i) for i in self.values)
                self.crc = self.crc8_func(self.string)

                if self.crc != self.CRC:
                    print ('***** Omron CRC error ***** Expected '+'%02x'%self.CRC+' Calculated: '+'%02x'%self.crc)
                    self.retries += 1                # start counting the number of times to retry the transmission
                    self.bytes_read = 0           # error is passed up using bytes read
                else:
                    break    # crc is good and bytes_read is good

        return self.bytes_read, self.temperature

def compare(pixels):
    digitalized = [0]*len(pixels)
    for i in range (0, len(pixels)):
        if pixels[i] >= THERESHOLD_TEMPERATURE :
            digitalized[i] = 1
    return  digitalized

def conv2d(grid):
    n = 0
    inputGrid = [[],[],[],[]]
    for i in range(0,16):
        if(i % 4 == 0 and i!=0):
            n +=1
        inputGrid[n].append(grid[i])
        #print(inputGrid)
    return inputGrid

def maxAreaOfGrid(grid):
    seen = set()
    def area(r, c):
        if not (0 <= r < len(grid) and 0 <= c < len(grid[0])
                and (r, c) not in seen and grid[r][c]):
            return 0
        seen.add((r, c))
        return (1 + area(r+1, c) + area(r-1, c) +
                area(r, c-1) + area(r, c+1))

    return max(area(r, c)
                for r in range(len(grid))
                for c in range(len(grid[0])))

""" ****************************************************************************************************************** """

class DriveError(Exception):
    pass

class GoogleDrive:
    """
    Handling the Google Drive Access
    """

    SCOPES = ['https://www.googleapis.com/auth/drive']
    FOLDER_MIME = "application/vnd.google-apps.folder"

    def __init__(self, secret, name):
        if not os.path.exists(secret):
            raise DriveError("Secret file does not exists")
        self.client_secret_file = secret
        self.application_name = name
        self.logger = logging.getLogger('GoogleDriveUploader')
        self.service = self.authorize()

    def authorize(self):
        """Gets valid user credentials from storage.
        If nothing has been stored, or if the stored credentials are invalid,
        the OAuth2 flow is completed to obtain the new credentials.
        """
        from oauth2client.service_account import ServiceAccountCredentials
        scopes = self.SCOPES
        credentials = ServiceAccountCredentials.from_json_keyfile_name(
            self.client_secret_file, scopes=scopes)
        http = credentials.authorize(httplib2.Http())
        return discovery.build('drive', 'v3', http=http)

    def upload_image(self, filename, parents=None):
        """
        Upload image file
        :param filename: ...
        """
        media_body = MediaFileUpload(filename, mimetype='image/jpeg', resumable=True)
        if parents and isinstance(parents, str):
            parents = [parents]
        body = {
            'name': os.path.basename(filename),
            'description': '',
            'parents': parents
        }

        try:
            upload = self.service.files().create(body=body, media_body=media_body).execute()
            self.logger.info("Uploaded image to Drive (Id: %s)", upload['id'])
        except errors.HttpError as error:
            self.logger.error("Could not upload image %s", error)
            return False
        else:
            return True

    def upload_txt(self, filename, parents=None):
        """
        Upload image file
        :param filename: ...
        """
        media_body = MediaFileUpload(filename, mimetype='text/txt', resumable=True)
        if parents and isinstance(parents, str):
            parents = [parents]
        body = {
            'name': os.path.basename(filename),
            'description': '',
            'parents': parents
        }

        try:
            upload = self.service.files().create(body=body, media_body=media_body).execute()
            self.logger.info("Uploaded image to Drive (Id: %s)", upload['id'])
        except errors.HttpError as error:
            self.logger.error("Could not upload txt %s", error)
            return False
        else:
            return True        

    def create_folder(self, name, parents=None):
        """
        :param name:
        :param kwargs: Anything that create(body=kwargs) accepts
        """
        body = {
            'mimeType': self.FOLDER_MIME,
            'name': name,
        }
        if parents:
            body['parents'] = [parents]
        fid = self.service.files().create(body=body).execute()
        return fid

    def share_folder_with_users(self, fileid, emails):
        """
        Share the folder or file with a specific user.
        :param fileid: id of the object we want to share, can be file or folder
        :param emails: list of email addresses of the user to share the folder with.
        """
        for email in emails:
            if not self.share_folder_with_user(fileid, email):
                return False
        return True

    def share_folder_with_user(self, fileid, email):
        """
        Share the folder or file with a specific user.
        :param fileid: id of the object we want to share, can be file or folder
        :param email: email address of the user to share the folder with.
        """
        body = {
            'role': 'writer',
            'type': 'user',
            'emailAddress': email
        }
        self.logger.debug("Creating permission for id %s", fileid)
        try:
            self.service.permissions().create(fileId=fileid, body=body,
                                              sendNotificationEmail=False).execute()
        except errors.HttpError as error:
            self.logger.error("Unable to set permissions %s", error)
            return False
        else:
            return True

    def delete_file(self, fileid):
        """Delete a file using Files.Delete()
        (WARNING: deleting permanently deletes the file!)
        :param param: additional parameter to file.
        :type param: dict.
        :raises: ApiRequestError
        """
        try:
            self.service.files().delete(fileId=fileid).execute()
        except errors.HttpError as error:
            self.logger.error("Could not delete image %s", error)
            return False
        else:
            return True

    def search_files_image(self, mime_type=None):
        """
        Search files with given query, return name and id
        :returns: dict with keys name and id
        """
        if not mime_type:
            mime_type = "image/jpeg"

        query = "mimeType='%s'" % mime_type
        return self.query(query)

    def search_files_txt(self, mime_type=None):
        """
        Search files with given query, return name and id
        :returns: dict with keys name and id
        """
        if not mime_type:
            mime_type = "image/jpeg"

        query = "mimeType='%s'" % mime_type
        return self.query(query)

    def query(self, query):
        """
        :returns: dict with the id, name pair of the result
        """
        result = {}
        page_token = None
        while True:
            response = self.service.files().list(q=query,
                                                 spaces='drive',
                                                 fields='nextPageToken, files(id, name)',
                                                 pageToken=page_token).execute()
            for file in response.get('files', []):
                result[file.get('id')] = file.get('name')
                self.logger.info('Found file: %s (%s)', file.get('name'), file.get('id'))
            page_token = response.get('nextPageToken', None)
            if page_token is None:
                break
        return result

class Configuration:

    filename = os.path.join(os.getcwd(), "configuration.config")

    def __init__(self):
        self.logger = logging.getLogger('GoogleDriveUploader-Configuration')
        self._latest_uploaded = []
        self._shared_folder = []
        self.client_secret_file = ''
        self.application_name = ''
        self.search_directory = os.path.join(os.getcwd(), "timelapse")
        self._share_with = []
        self.date_directory = True
        self.interval = 30
        self.n_last_images = 5
        self.n_last_txts = 5
        self.image_name_starting_number = 0
        self.txt_name_starting_number = 0
        self.read_configuration()

    def read_configuration(self):
        '''
        Read configuration file
        '''
        config = configparser.ConfigParser()
        config.read(self.filename)
        self.latest_uploaded = config['Information']['latest_uploaded']
        self.client_secret_file = config['Drive']['client_secret_file']
        self.application_name = config['Drive']['application_name']
        self.share_with = config['Drive']['share_with']
        self.shared_folder = config['Drive']['shared_folder']
        self.search_directory = config['Application']['search_directory']
        if not os.path.exists(self.search_directory):
            self.logger.warning('Directory %s does not yet exists...', self.search_directory)
        self.date_directory = config['Application'].getboolean('date_directory')
        self.interval = int(config['Application']['interval'])
        self.n_last_images = int(config['Application']['n_last_images'])
        self.n_last_txts = int(config['Application']['n_last_txts'])
        self.image_name_starting_number = int(config['Application']['image_name_starting_number'])
        self.txt_name_starting_number = int(config['Application']['txt_name_starting_number'])
        imageIndex = self.image_name_starting_number
        txtIndex = self.txt_name_starting_number
        outputDirectory = config['Application']['search_directory']
        LFM_PWM_PIN = int(config['Pins']['LFM_PWM_PIN'])
        LBM_PWM_PIN = int(config['Pins']['LBM_PWM_PIN'])
        RFM_PWM_PIN = int(config['Pins']['RFM_PWM_PIN'])
        RBM_PWM_PIN = int(config['Pins']['RBM_PWM_PIN'])
        LFM_DIR_PIN = int(config['Pins']['LFM_DIR_PIN'])
        LBM_DIR_PIN = int(config['Pins']['LBM_DIR_PIN'])
        RFM_DIR_PIN = int(config['Pins']['RFM_DIR_PIN'])
        RBM_DIR_PIN = int(config['Pins']['RBM_DIR_PIN'])
        FRQ = int(config['Pins']['FRQ'])
        strodLength = int(config['Pins']['strodLength'])
        FULLSTEPLENGTH = int(config['Pins']['FULLSTEPLENGTH'])
        stepperDir = int(config['Pins']['stepperDir'])
        stepperStep = int(config['Pins']['stepperStep'])
        THERESHOLD_TEMPERATURE = int(config['Pins']['THERESHOLD_TEMPERATURE'])
        ALARM_PIN = int(config['Pins']['ALARM_PIN'])

        self.log_configuration()

    def write_configuration(self):
        '''
        Write configuration file
        '''
        config = configparser.ConfigParser()
        config['Information'] = {}
        config['Information']['latest_uploaded'] = ','.join(self.latest_uploaded)

        config['Drive'] = {}
        config['Drive']['client_secret_file'] = self.client_secret_file
        config['Drive']['application_name'] = self.application_name
        config['Drive']['share_with'] = ','.join(self.share_with)
        config['Drive']['shared_folder'] = ','.join(self.shared_folder)

        config['Application'] = {}
        config['Application']['search_directory'] = self.search_directory
        config['Application']['date_directory'] = str(self.date_directory)
        config['Application']['interval'] = str(self.interval)
        config['Application']['n_last_txts'] = str(self.n_last_txts)
        config['Application']['n_last_images'] = str(self.n_last_images)
        config['Application']['image_name_starting_number'] = str(imageIndex)
        config['Application']['txt_name_starting_number'] = str(txtIndex)
        
        config['Pins']['LFM_PWM_PIN'] = str(LFM_PWM_PIN)
        config['Pins']['LBM_PWM_PIN'] = str(LBM_PWM_PIN)
        config['Pins']['RFM_PWM_PIN'] = str(RFM_PWM_PIN)
        config['Pins']['RBM_PWM_PIN'] = str(RBM_PWM_PIN)
        config['Pins']['LFM_DIR_PIN'] = str(LFM_DIR_PIN)
        config['Pins']['LBM_DIR_PIN'] = str(LBM_DIR_PIN)
        config['Pins']['RFM_DIR_PIN'] = str(RFM_DIR_PIN)
        config['Pins']['RBM_DIR_PIN'] = str(RBM_DIR_PIN)
        config['Pins']['FRQ'] = str(FRQ)
        config['Pins']['strodLength'] = str(strodLength)
        config['Pins']['FULLSTEPLENGTH'] = str(FULLSTEPLENGTH)
        config['Pins']['stepperDir'] = str(stepperDir)
        config['Pins']['stepperStep'] = str(stepperStep)
        config['Pins']['THERESHOLD_TEMPERATURE'] = str(THERESHOLD_TEMPERATURE)
        config['Pins']['ALARM_PIN'] = str(ALARM_PIN)

        with open(self.filename, 'w') as configfile:
            config.write(configfile)

    def log_configuration(self):
        '''
        Just log the configuration
        '''
        self.logger.info("latest_uploaded: %s", self.latest_uploaded)
        self.logger.info("shared_folder: %s", self.shared_folder)
        self.logger.info("client_secret_file: %s", self.client_secret_file)
        self.logger.info("application_name: %s", self.application_name)
        self.logger.info("search_directory: %s", self.search_directory)
        self.logger.info("share_with: %s", self.share_with)
        self.logger.info("date_directory: %s", self.date_directory)
        self.logger.info("interval: %s", self.interval)
        self.logger.info("n_last_images: %s", self.n_last_images)
        self.logger.info("n_last_txts: %s", self.n_last_txts)
        self.logger.info("image_name_starting_number: %s", self.image_name_starting_number)

    @property
    def shared_folder(self):
        """Get list of people to share the uploads with."""
        return self._shared_folder

    @shared_folder.setter
    def shared_folder(self, value):
        if value:
            self._shared_folder = [i for i in value.split(',') if i]

    @property
    def share_with(self):
        """Get list of people to share the uploads with."""
        return self._share_with

    @share_with.setter
    def share_with(self, value):
        if value:
            self._share_with = [i for i in value.split(',') if i]

    @property
    def latest_uploaded(self):
        """Get list of people to share the uploads with."""
        return self._latest_uploaded

    @latest_uploaded.setter
    def latest_uploaded(self, value):
        if isinstance(value, str):
            self._latest_uploaded = [i for i in value.split(',') if i]
        elif isinstance(value, list):
            self._latest_uploaded = value


class Upload:

    def __init__(self):
        self.logger = logging.getLogger('GoogleDriveUploader')
        self.config = Configuration()
        self.drive = GoogleDrive(self.config.client_secret_file, self.config.application_name)

    def get_latest_images(self, directory, n_last_images):
        '''
        Returns the names of the n newest images in the directory
        '''
        latest = None
        try:
            latest = sorted([os.path.join(directory, f) for f in os.listdir(directory) if f.lower().endswith('.jpg')],
                            key=os.path.getctime, reverse=True)
        except ValueError:
            self.logger.error('No images found in directory %s', directory)

        if not latest or len(latest) == 0:
            return None
        return latest[0:n_last_images]

    def get_latest_txt(self, directory, n_last_txts):
        '''
        Returns the names of the n newest images in the directory
        '''
        latest = None
        try:
            latest = sorted([os.path.join(directory, f) for f in os.listdir(directory) if f.lower().endswith('.txt')],
                            key=os.path.getctime, reverse=True)
        except ValueError:
            self.logger.error('No txt found in directory %s', directory)

        if not latest or len(latest) == 0:
            return None
        return latest[0:n_last_txts]    

    def get_folder_or_create_it(self, foldername, parentid=None):
        """
        If the given folder does not exists, create it
        :returns: id
        """
        query = str.format("mimeType='{0}' and name='{1}'",
                           self.drive.FOLDER_MIME, foldername)
        if parentid:
            query += str.format(" and '{0}' in parents", parentid)

        resultid = None

        result = self.drive.query(query)
        if len(result) > 1:
            self.logger.warning("Multiple results found for folder. Using the first!")

        for key in result.keys():
            self.logger.info("Using key %s", key)
            resultid = key
            break

        if not resultid:
            if parentid:
                return self.drive.create_folder(foldername, parents=parentid).get('id')
            else:
                return self.drive.create_folder(foldername).get('id')
        else:
            return resultid

    def create_missing_date_folders(self, filename):
        """
        Returns the ID of the day folder and creates the folder if they does not exist
        :param filename: absolute path
        :returns: Tuple with foldername and ID
        """
        directory = os.path.dirname(filename)
        # remove the current directory
        directory = directory.split(os.getcwd())[1]
        # first element will be empty, therefore we remove it
        year, month, day = directory.split(os.sep)[2:]
        yid = self.get_folder_or_create_it(year)
        mid = self.get_folder_or_create_it(month, yid)
        did = self.get_folder_or_create_it(day, mid)

        return year, did

    def create_missing_folder(self, filename):
        """
        Returns the ID of folder and creates the folder if they does not exist
        :param filename: absolute path
        :returns: Tuple with foldername and ID
        """
        directory = os.path.dirname(filename)
        # get foldername
        directory = directory.split(os.sep)[-1]
        did = self.get_folder_or_create_it(directory)

        return directory, did

    def current_date_directory(self):
        '''Returns the current date directory where the images are stored in'''
        now = datetime.datetime.now()
        path = '{0}{1}{2}{3}{4}'.format(now.year, os.sep, now.month, os.sep, now.day)
        path = os.path.join(self.config.search_directory, path)
        if not os.path.exists(path):
            self.logger.warning('Directory %s does not yet exists...', path)
            return None
        return path

    def upload_newest_images(self):
        '''
        Looks into the timelapse directory and uploads the newest images
        '''
        if self.config.date_directory:
            path = self.current_date_directory()
            if not path:
                return
        else:
            path = self.config.search_directory

        images = self.get_latest_images(path, self.config.n_last_images)
        if not images:
            return

        self.logger.info("Newest images are %s", images)
        for image in images:
            if not self.upload_image(image):
                self.logger.warning("Unable to upload image %s", image)

        self.config.latest_uploaded = [os.path.basename(image) for image in images]
        self.config.write_configuration()

    def upload_newest_txt(self):
        '''
        Looks into the timelapse directory and uploads the newest txt
        '''
        if self.config.date_directory:
            path = self.current_date_directory()
            if not path:
                return
        else:
            path = self.config.search_directory

        txts = self.get_latest_txt(path, self.config.n_last_txts)
        if not txts:
            return

        self.logger.info("Newest txt are %s", txts)
        for txt in txts:
            if not self.upload_txt(txt):
                self.logger.warning("Unable to upload txt %s", txt)

        self.config.latest_uploaded = [os.path.basename(txt) for txt in txts]
        self.config.write_configuration()

    def upload_image(self, image):
        ''' Uploads newest image
        '''
        return_val = None
        if os.path.basename(image) in self.config.latest_uploaded:
            self.logger.info("Image %s already uploaded, will skip this one.", image)
            return None

        fid = None
        foldername = ''
        if self.config.date_directory:
            foldername, fid = self.create_missing_date_folders(image)
        else:
            foldername, fid = self.create_missing_folder(image)

        if self.drive.upload_image(image, fid):
            return_val = image

        if foldername not in self.config.shared_folder:
            files = self.drive.search_files_image(self.drive.FOLDER_MIME)
            for key, value in files.items():
                if value == foldername:
                    if self.drive.share_folder_with_users(key, self.config.share_with):
                        self.logger.info("Folder %s not yet shared, sharing it now and writing configuration",
                                         foldername)
                        self.config.shared_folder.append(foldername)
                    else:
                        self.config.shared_folder = []
        return return_val

    def upload_txt(self, txt):
        ''' Uploads newest txt
        '''
        return_val = None
        #this part is responsible for not uploading duplicate files.
        if os.path.basename(txt) in self.config.latest_uploaded:
            self.logger.info("Txt %s already uploaded, will skip this one.", txt)
            return None
        
        fid = None
        foldername = ''
        if self.config.date_directory:
            foldername, fid = self.create_missing_date_folders(txt)
        else:
            foldername, fid = self.create_missing_folder(txt)

        if self.drive.upload_txt(txt, fid):
            return_val = txt

        if foldername not in self.config.shared_folder:
            files = self.drive.search_files_txt(self.drive.FOLDER_MIME)
            for key, value in files.items():
                if value == foldername:
                    if self.drive.share_folder_with_users(key, self.config.share_with):
                        self.logger.info("Folder %s not yet shared, sharing it now and writing configuration",
                                         foldername)
                        self.config.shared_folder.append(foldername)
                    else:
                        self.config.shared_folder = []
        return return_val

    def __delete_all_files(self):
        '''
        Clears the complete drive, for debugging purposes
        '''
        files_image = self.drive.search_files_image(self.drive.FOLDER_MIME)
        for key, value in files_image.items():
            self.drive.delete_file(key)

        files_txt = self.drive.search_files_txt(self.drive.FOLDER_MIME)
        for key, value in files_txt.items():
            self.drive.delete_file(key)

    def check_for_new_images(self):
        '''
        Runs every n seconds and checks for new images
        '''
        try:
            #while True:
            timer = threading.Timer(self.config.interval, self.upload_newest_images)
            timer.start()
            timer.join()
        except KeyboardInterrupt:
            self.logger.info("Leaving timer thread. Goodbye!")

    def check_for_new_txt(self):
        '''
        Runs every n seconds and checks for new images
        '''
        try:
            #while True:
            timer = threading.Timer(self.config.interval, self.upload_newest_txt)
            timer.start()
            timer.join()
        except KeyboardInterrupt:
            self.logger.info("Leaving timer thread. Goodbye!")

def init_logging():
    """ initalize logging
    """
    # set up logging to file - see previous section for more details
    logging.basicConfig(level=logging.DEBUG,
                        filename='configuration.log',
                        filemode='w')
    # define a Handler which writes INFO messages or higher to the sys.stderr
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)



""" ****************************************************************************************************************** """

"""
Camera :
https://github.com/rdeepc/ExploreOpencvDnn
https://heartbeat.fritz.ai/real-time-object-detection-on-raspberry-pi-using-opencv-dnn-98827255fa60
"""
# Pretrained classes in the model
classNames = {0: 'background',
              1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane', 6: 'bus',
              7: 'train', 8: 'truck', 9: 'boat', 10: 'traffic light', 11: 'fire hydrant',
              13: 'stop sign', 14: 'parking meter', 15: 'bench', 16: 'bird', 17: 'cat',
              18: 'dog', 19: 'horse', 20: 'sheep', 21: 'cow', 22: 'elephant', 23: 'bear',
              24: 'zebra', 25: 'giraffe', 27: 'backpack', 28: 'umbrella', 31: 'handbag',
              32: 'tie', 33: 'suitcase', 34: 'frisbee', 35: 'skis', 36: 'snowboard',
              37: 'sports ball', 38: 'kite', 39: 'baseball bat', 40: 'baseball glove',
              41: 'skateboard', 42: 'surfboard', 43: 'tennis racket', 44: 'bottle',
              46: 'wine glass', 47: 'cup', 48: 'fork', 49: 'knife', 50: 'spoon',
              51: 'bowl', 52: 'banana', 53: 'apple', 54: 'sandwich', 55: 'orange',
              56: 'broccoli', 57: 'carrot', 58: 'hot dog', 59: 'pizza', 60: 'donut',
              61: 'cake', 62: 'chair', 63: 'couch', 64: 'potted plant', 65: 'bed',
              67: 'dining table', 70: 'toilet', 72: 'tv', 73: 'laptop', 74: 'mouse',
              75: 'remote', 76: 'keyboard', 77: 'cell phone', 78: 'microwave', 79: 'oven',
              80: 'toaster', 81: 'sink', 82: 'refrigerator', 84: 'book', 85: 'clock',
              86: 'vase', 87: 'scissors', 88: 'teddy bear', 89: 'hair drier', 90: 'toothbrush'}

#Initaite Camera:
camera = PiCamera()
# Loading model
model = cv2.dnn.readNetFromTensorflow('models/frozen_inference_graph.pb',
                                      'models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

def id_class_name(class_id, classes):
    for key, value in classes.items():
        if class_id == key:
            return value

def checkCamera():
    image = cv2.imread("image%d.jpg" ,imageIndex)
    image_height, image_width, _ = image.shape
    model.setInput(cv2.dnn.blobFromImage(image, size=(300, 300), swapRB=True))
    output = model.forward()
    for detection in output[0, 0, :, :]:
        confidence = detection[2]
        if confidence > .5:
            class_id = detection[1]
            class_name=id_class_name(class_id,classNames)
            print(str(str(class_id) + " " + str(detection[2])  + " " + class_name))
            box_x = detection[3] * image_width
            box_y = detection[4] * image_height
            box_width = detection[5] * image_width
            box_height = detection[6] * image_height
            cv2.rectangle(image, (int(box_x), int(box_y)), (int(box_width), int(box_height)), (23, 230, 210), thickness=1)
            cv2.putText(image,class_name ,(int(box_x), int(box_y+.05*image_height)),cv2.FONT_HERSHEY_SIMPLEX,(.005*image_width),(0, 0, 255))
    cv2.imshow('image', image)
    # cv2.imwrite("image_box_text.jpg",image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

"""
Function used to count how many persons in font of it.
It takes about 5 frames then gets the average to measure how many persons.
Takes no Arguments
Return the number of persons inside the Frame.
"""

def checkPersons():
    imageIndex += 1
    averageArray = [0]*NumOfFrames
    for index in range(0, len(averageArray)):
        # change the folder location.
        camera.capture(outputDirectory+'/image%s.jpg' %imageIndex)
        image = cv2.imread("image%s.jpeg" %imageIndex)
        image_height, image_width, _ = image.shape
        model.setInput(cv2.dnn.blobFromImage(image, size=(300, 300), swapRB=True))
        output = model.forward()
        for detection in output[0, 0, :, :]:
            confidence = detection[2]
            if confidence > .5:
                class_id = detection[1]
                if class_id == 1: #if human is detected?!
                    averageArray[index] += 1
    return round(sum(averageArray)/len(averageArray))


""" ****************************************************************************************************************** """

#Gps functions:
#Initiate UART:
port = "/dev/ttyAMA0"
ser=serial.serial(port, baudrate=9600, timeout=0.5)
def getLocation():
    lat = 0
    lng = 0
    dataout = pynmea2.NMEAStreamReader()
    newdata= ser.readline()
    if newdata[0:6] == "$GPRMC":
        newmsg = pynmea2.parse(newdata)
        lat = newmsg.latitude
        lng = newmsg.longitude
        gps = "Latitude="+ str(lat)+ "and Longitude="+ str(lng)
        print(gps)
        createTxtFile(lat, lng)

def createTxtFile(lat, lng):
    txtIndex +=1 
    txtfile = open("GPS Location %s.txt" %txtIndex, "a+")
    txtfile.write("Latitude  = %d\t" %lat)
    txtfile.write("longitude = %d \r\n" %lng)
    txtfile.close()

""" ****************************************************************************************************************** """

#Using Cytron Dual Channel Motor Driver for DC motors.
#Assuming True is Forward
#Assuming False is Backward

IO.setmode(IO.BOARD)
IO.setup(LFM_PWM_PIN, IO.OUT)
IO.setup(LBM_PWM_PIN, IO.OUT)
IO.setup(RFM_PWM_PIN, IO.OUT)
IO.setup(RBM_PWM_PIN, IO.OUT)
IO.setup(LFM_DIR_PIN, IO.OUT)
IO.setup(LBM_DIR_PIN, IO.OUT)
IO.setup(RFM_DIR_PIN, IO.OUT)
IO.setup(RBM_DIR_PIN, IO.OUT)

IO.setup(ALARM_PIN, IO.OUT)

LFM_PWM  = IO.PWM(LFM_PWM_PIN, FRQ)
LBM_PWM  = IO.PWM(LBM_PWM_PIN, FRQ)
RFM_PWM  = IO.PWM(RFM_PWM_PIN, FRQ)
RBM_PWM  = IO.PWM(RBM_PWM_PIN, FRQ)

def forward():
    LFM_PWM.ChangeDutyCycle(25)
    IO.output(LFM_DIR_PIN, True)
    LBM_PWM.ChangeDutyCycle(25)
    IO.output(LBM_DIR_PIN, True)
    RFM_PWM.ChangeDutyCycle(25)
    IO.output(RFM_DIR_PIN, True)
    RBM_PWM.ChangeDutyCycle(25)
    IO.output(RBM_DIR_PIN, True)

def backward():
    LFM_PWM.ChangeDutyCycle(25)
    IO.output(LFM_DIR_PIN, False)
    LBM_PWM.ChangeDutyCycle(25)
    IO.output(LBM_DIR_PIN, False)
    RFM_PWM.ChangeDutyCycle(25)
    IO.output(RFM_DIR_PIN, False)
    RBM_PWM.ChangeDutyCycle(25)
    IO.output(RBM_DIR_PIN, False)

def left():
    LFM_PWM.ChangeDutyCycle(25)
    IO.output(LFM_DIR_PIN, False)
    LBM_PWM.ChangeDutyCycle(25)
    IO.output(LBM_DIR_PIN, False)
    RFM_PWM.ChangeDutyCycle(25)
    IO.output(RFM_DIR_PIN, True)
    RBM_PWM.ChangeDutyCycle(25)
    IO.output(RBM_DIR_PIN, True)

def right():
    LFM_PWM.ChangeDutyCycle(25)
    IO.output(LFM_DIR_PIN, True)
    LBM_PWM.ChangeDutyCycle(25)
    IO.output(LBM_DIR_PIN, True)
    RFM_PWM.ChangeDutyCycle(25)
    IO.output(RFM_DIR_PIN, False)
    RBM_PWM.ChangeDutyCycle(25)
    IO.output(RBM_DIR_PIN, False)

def stop():
    LFM_PWM.ChangeDutyCycle(0)
    LBM_PWM.ChangeDutyCycle(0)
    RFM_PWM.ChangeDutyCycle(0)
    RBM_PWM.ChangeDutyCycle(0)

"""******************************************************************************************************************"""

"""
    Using full step.
    stepper motor's full step moves about 20mm.
    So depending on the length of stod you use in this robot, count how many steps.
    number of steps = strod's length in mm / 20 mm.

    This code may result in motor vibration and jerky motion especially at low speeds.
    One way to counter these result is with microstepping.
    The following code snippet is added to the code above.
    The mode GPIO pins are set as outputs.
    A dict holds the appropriate values for each stepping format.
    GPIO.output sets the mode to 1/32.
    Note that step count is multiplied by 32 because each rotation now takes 32 times as many cycles which results in more fluid motion.
    The delay is divided by 32 to compensate for the extra steps.
    MODE = (14, 15, 18)   # Microstep Resolution GPIO Pins
    GPIO.setup(MODE, GPIO.OUT)
    RESOLUTION = {'Full': (0, 0, 0),
                'Half': (1, 0, 0),
                '1/4': (0, 1, 0),
                '1/8': (1, 1, 0),
                '1/16': (0, 0, 1),
                '1/32': (1, 0, 1)}
    GPIO.output(MODE, RESOLUTION['1/32'])

    step_count = SPR * 32
    delay = .0208 / 32
"""
"""
    One issue with the last program is that it relies on teh python sleep method for timing which is not very reliable.
    so you can use PiGPIO library which provide hardware based PWM timing.
"""

"""******************************************************************************************************************"""

def main():

    try:
        """ Uncomment if you tested them alone.
        #Stepper Motor: Initialization and adjusting.
        CW = 1     # Clockwise Rotation
        CCW = 0    # Counterclockwise Rotation
        SPR = round(strodLength/ FULLSTEPLENGTH)   # Steps per Revolution
        IO.setup(stepperDir, IO.OUT)
        IO.setup(stepperStep, IO.OUT)
        IO.output(stepperDir, CW)
        #Adjust the step count and dela for you height and design.
        step_count = SPR
        delay = 0.0208
        for x in range(step_count):
            IO.output(stepperStep, True)
            time.sleep(delay)
            IO.output(stepperStep, False)
            time.sleep(delay)
        """
        #Initiate Thermal Sensor:
        omron = OmronD6T()
        #Initiate Uploading to google drive:
        init_logging()
        upload = Upload()
        while True:
            """
                Assuming Robot speed is slow.
            """
            #every 1 second:
            forward()
            time.sleep(1)
            pixelsTemp = [0]*16
            omron.read()
            #print ("ICTemp:",omron.roomTemp)
            for i in range(0,len(omron.temperature)):
                pixelsTemp[i] = omron.temperature[i]
            #Compare Tempratures with THERESHOLD Temperature:
            #Convert From 1D to 2D Array:
            digitalizedPixels = conv2d(compare(pixelsTemp))
            res = maxAreaOfGrid(digitalizedPixels)
            print(res)
            #check if max island of high pixels in 3 or higher:
            if(res > THERESHOLD_PIXELS):
                stop()
                #Check camera to see in this a person:
                detectedPersons = checkPersons()
                if detectedPersons != 0:
                    if detectedPersons > 1:
                        print("Two persons Close to each other")
                    else:
                        print("High Temprature person is detected")
                        # Buzzer On, Relay Active Low
                        IO.output(ALARM_PIN, False)
                        getLocation()
                        #upload._ImageUpload__delete_all_files()
                        upload.check_for_new_images()
                        upload.check_for_new_txt()
                        #Fire Alarm
                        #wait some time
                        time.sleep(1)
                        IO.output(ALARM_PIN, True)

    except KeyboardInterrupt:
        print("\n Ctrl-c Pressed, Stopping and exiting")
    finally:
        pigpio.stop()


if __name__ == '__main__':
    main()

