# Watchman Robot
Ever feel as if there is pair of eyes always watching you 24/7? Ever feel as if you are getting stalked? Well this robot does just that. The Watchman Robot useses a pair of mechanical eyes built from 3D printed parts along with a camera to keep a constant "eye" on you behind its creepy mask.  

| **Name** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Sharva P | Mission San Jose Highschool | Mechanical Engineering | Incoming Senior

<!-- **Replace the BlueStamp logo below with an image of yourself and your completed project. Follow the guide [here](https://tomcam.github.io/least-github-pages/adding-images-github-pages-site.html) if you need help.**

![Headstone Image](logo.svg)
-->
  
# Final Milestone
<iframe width="560" height="315" src="https://www.youtube.com/embed/uw-IKo6svxU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
The final version incorporates both the mechanical eye mechanism from Milestone 1 and the face tracking software from Milestone 2. As I explained in Milestone 2, once a face is detected in the live feed from the Pi Camera and coordinates of the face are sent to the arduino, they are translated into servo positions. This causes the eyes to move and follow the person in view around. The final thing I added to the project was a face so it doesn’t look like floating eyeballs but an actual human. There were a couple of issues that came up at the stage of combining the mechanisms. One was the fact the servo positions weren’t mechanically set to zero while building , which meant the positions in the code didn’t accurately translate to the right positions. To fix this, the whole eye mechanism had to be taken off. After that the servo screw attaching the "muscle" for the eye was taken and reset to the physical zero position. Another thing was the serial code being outputted by the raspberry pi, didn’t match the Arduino. This meant that the code sent through the Raspberry Pi board was too fast for the Arduino to read. This was fixed through manually setting the input and output speeds of the boards. The last problem I encountered was the fact that the Raspberry Pi and Arduino had to be powered separately otherwise the servos didn’t have enough power to work. This project was really fun to build and code and it gave me a true experience of the engineering process. It wasn’t easy, which is what I hoped for, and made sure I kept being persistent in finding the problems. Sometimes it could be as small as not a secure wire or a whole board corrupting. Looking back, the guide wasn’t well made for me, which meant this was pretty much a self interpreted project. Things I would add to the Watchman robot in the future would be some sort of speaker or microphone so the mask could convey some creepy message. Another thing that could be added would be a lower body or neck of some sort to make the robot more interactive and have a larger range of detection.


# Second Milestone
<iframe width="560" height="315" src="https://www.youtube.com/embed/iMNvpBLwmDA" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
For the Second Milestone, I wanted to demonstrate my face tracking software (which can be found below). The code is written in python and run from Thonny Python IDE. The code is sent through a Raspberry Pi Zero WH which has a AIY Vision Hat on top of it. Plugged into the Pi board are 3 different wires. The first wire is the HDMI cord to stream the output that is inside the SD card. The card is flashed with the latest Raspberry Pi Imager file. The second wire is used as a USB hub for the preripherals and the wire connecting the Raspberry Pi and the Arduino Uno R3. The final wire is the power which is connnected to a 5V power adapter. Using a ribbon cable, I am able to connect the Raspberry Pi to the Vision Hat. Another longer ribbon cable connects Vision Hat to the PiCamera V2.1. Here is a basic overview of how the code works: First the Camera is imported and the resolution and frame size is set. Then the image Annotator is imported. This is a tool that I can use in the code to identify and point out specific objects that I choose. So, I set the object to any faces, and once the face is found, it returns a XY coordinate to the board. The board then sends these codes to the arduino, where it converts these codes, to a calibrated servo position (eye mechanism). My main issue, that delayed me for two weeks, was the fact the electronics were faulty. First the PiCamera V2 is known to be faulty, and it turns out the first one I got didn't work in the first place. After replacing it, I later found out the Raspberry Pi board doesn't work either. Once replacing both electronics, my code ran smoothly. Another small challenge was due to the fact I am using a MacOS to work. Since the OS software cannot work as a monitor, I had to use OBS, Open Broadcasting Software, in order to stream the SD Card in the Pi Board. This made the process of booting up and updating electronics veryh complicated and inefficient. For my next and Final Milestone, I plan on finishing the complete mechanism, where the software and hardware can compile together. 


# First Milestone
<iframe width="560" height="315" src="https://www.youtube.com/embed/CKRm5gqWMoU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe> 

First, let's talk about the components of the robot. The eye mechanism is a complex robotic eye powered by a total of 6 servos. Four out of the six servos are used to control the up, down, left, and right movements of the eyeball, while the other two are used for closing and opening the eyelids. Inside the eyeballs, there are laser diodes to assist with calibration (Makes sure the face doesn't get a lazy eye). Next, let's discuss the Arduino board. I used an Arduino Uno R3 and a servo shield bby Adafruit, which is stacked on top of the Arduinio, to power and connect the multitude of servos. It also provides power to the laser diodes, which are regulated with a 5V relay. Lastly, I used an AIY vision kit to track human faces from the camera. The challenges I encountered were mainly related to the eye mechanism. Assembling the eye took almost 3 days due to the small size of the 3D printer and the parts that needed to be assembled. The fragility of the parts made them prone to snapping, requiring additional prints. Since everything was small, I had to tighten all the components by hand using an Allen key. Another challenge was figuring out the wiring necessary to power all the different parts. Since this was my first time using Arduino, I had to learn everything about it before assembling the components. Moving forward, my plan is to develop the code for the eye mechanism and the face tracking functionality.


# Starter Project
<iframe width="560" height="315" src="https://www.youtube.com/embed/pJAbVRicJ2c" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

For my starter project I decided to make the TV-B-Gone kit. This small compact device majorly impacts large TVs with just a push of a button. No matter the company, place, or area, this pocket device can turn any TV off. So let us talk about the key parts in this project. Starting with the power supply, it uses 2 AA batteries, producing 3V of power. To emit the light to interact with the TVs, it uses IR LEDs which emit at 940nm. Obviously we have to include the button to start the code and finally it uses things like resonators and transistors to keep the current and electricity constant throughout the device. A resonator acts a clock on a circuit board so the functions from the chip are running in a timely matter. Meanwhile, the transistor acts as a regulator to keep the current up to the specific voltage needed. This was a great project to introduce circuits and LEDs which will help me in my main project that incorporates laser diodes. 


<!-- # Schematics 
Here's where you'll put images of your schematics. [Tinkercad](https://www.tinkercad.com/blog/official-guide-to-tinkercad-circuits) and [Fritzing](https://fritzing.org/learning/) are both great resoruces to create professional schematic diagrams, though BSE recommends Tinkercad becuase it can be done easily and for free in the browser. 
-->

# Java

<details markdown="1">
<summary>Serial Servo Arduino</summary>

  
```Java

// This sketch receives serial information in the format <x,y,L1,L2> where the the variables (x,y,L1,L2) are degree values.
// Angle values are then adjusted with calibration values before they are sent to each servo

// This sketch is designed to work with "Adafruit 16-Channel 12-bit PWM/Servo Shield - I2C interface" on an Arduino Uno
// This sketch requires the adafruit_PWMServo library, they have a useful guide here: https://learn.adafruit.com/adafruit-16-channel-pwm-slash-servo-shield/using-the-adafruit-library

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};
int integerFromPC = 0;
float XFromPi = 0.0;
//float floatFromPC = 0.0;
float YFromPi = 0.0;
float L1FromPi = 0.0;
float L2FromPi = 0.0;

boolean newData = false;


//Servo Control Declarations
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Servo header position on adafruit servo hat (using positions 0-2 and 4-6)
#define X1SERVO 0 // X Servo (left-right) on right eye
#define Y1SERVO 1 // Y Servo (up-down) on right eye
#define L1SERVO 2 // Bottom Eyelid, controlled by right-side eyelid servo

#define X2SERVO 4 // X Servo (left-right) on left eye
#define Y2SERVO 5 // Y Servo (up-down) on left eye
#define L2SERVO 6 // Top Eyelid, controlled by left-side eyelid servo
#define LaserPin 2 // This pin is used to switch laser MOSFET on/off. Mosfet is used to turn laser off when eyelids are shut.

float CtsPerDeg = 2.8; // This is a measured value, "calibrated" by driving servo to 90ish degrees (from middle position) and measuring counts
int MidPtCounts = 368; // Derived from servo midpoint = 1.5ms, 4.07 microseconds per count 1500/4.07 = 368.55

//Degree Compensations to adjust mid-point of servos - Cross-Eye fixing, adjust for mechanical errors as well
// These values are obtained by using a calibration image on the raspberry pi. 
// I used the serial monitor to command the eyelids to zero position and then adjusted the zero point until the eyes and eyelids were properly adjusted (ie pointed dead center, eyes shut when <0,0,0,0> command is received)
float X1DegComp = 4;
float X2DegComp = -5;
float Y1DegComp = -7;
float Y2DegComp = -3;
float L1DegComp = -25;
float L2DegComp = -40;
float LidCloseComp = -12; // This adds extra "clamping" force to completely close the eyes
//float L1DegTrack = 10;  


void setup() {
    Serial.begin(115200);
    //Serial.println("This demo expects 4 pieces of data - 4 floating point values");
    //Serial.println("Enter data in this style <12.5, 12.0, 24.7, 13.0>  ");
    //Serial.println();
    pwm.begin();

    pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
    pinMode(LaserPin, OUTPUT);
    delay(10);
}

//============

void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        SetServoPosition();
//        showParsedData();
        newData = false;
    }
}

//============

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    XFromPi = atof(strtokIndx); // convert this part to a float
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    YFromPi = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    L1FromPi = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    L2FromPi = atof(strtokIndx);     // convert this part to a float

}

//============

void showParsedData() {
    Serial.print("X ");
    Serial.println(XFromPi);
    Serial.print("Y ");
    Serial.println(YFromPi);
    Serial.print("L1 ");
    Serial.println(L1FromPi);
    Serial.print("L2 ");
    Serial.println(L2FromPi);
}

//This function maps the Servo Angles and sends the commands to the servos

void SetServoPosition() {

  // Set X Positions
  pwm.setPWM(X1SERVO, 0, ConvertToCounts(XFromPi + X1DegComp));
  pwm.setPWM(X2SERVO, 0, ConvertToCounts(XFromPi + X2DegComp));
//  Serial.print("X ");
//    Serial.println((-1*(XFromPi + X2DegComp)));

  // Set Y Positions
  pwm.setPWM(Y1SERVO, 0, ConvertToCounts(YFromPi + Y1DegComp));
  pwm.setPWM(Y2SERVO, 0, ConvertToCounts(-1*(YFromPi + Y2DegComp)));

  // Set Laser On/Off and Set Eyelid Position
  if (L1FromPi == 0 and L2FromPi == 0) {
    digitalWrite(LaserPin, LOW);
    pwm.setPWM(L1SERVO, 0, ConvertToCounts(L1FromPi + L1DegComp + LidCloseComp)); //Bottom Eyelid
    pwm.setPWM(L2SERVO, 0, ConvertToCounts(L2FromPi + L2DegComp + LidCloseComp)); //Top Eyelid
  }
  else {
  digitalWrite(LaserPin, HIGH);
  pwm.setPWM(L1SERVO, 0, ConvertToCounts(L1FromPi + L1DegComp)); //Bottom Eyelid
  pwm.setPWM(L2SERVO, 0, ConvertToCounts(L2FromPi + L2DegComp)); //Top Eyelid
  }
//    Serial.println("L1FromPi =");
//    Serial.println(L1FromPi);
//    Serial.println("L2FromPi =");
//    Serial.println(L2FromPi);
  delay(5);                           // waits for the servo to get there
}

//This function converts degree values to servo "counts" as required by the Adafruit_PWMServo library

float ConvertToCounts(float Degrees) {
  float Counts;
  Counts = MidPtCounts + (CtsPerDeg * Degrees);
//  Serial.print("Counts ");
//  Serial.println(Counts);
    
           return Counts;
          
}

```

</details>

# Python

<details markdown="1">
<summary>Camera Preview</summary>

```Java

#/usr/bin/env python3

import picamera
from PIL import Image
from time import sleep

camera = picamera.PiCamera()
camera.resolution = (1640, 1232)
camera.framerate = 24
camera.start_preview()

// Load the arbitrarily sized image
img = Image.open('Crosshair_Black.png')
// Create an image padded to the required size with
// mode 'RGB'
pad = Image.new('RGB', (
    ((img.size[0] + 31) // 32) * 32,
    ((img.size[1] + 15) // 16) * 16,
    ))
// Paste the original image into the padded one
pad.paste(img, (0, 0))

// Add the overlay with the padded image as the source,
// but the original image's dimensions
o = camera.add_overlay(pad.tobytes(), size=img.size)
// By default, the overlay is in layer 0, beneath the
// preview (which defaults to layer 2). Here we make
// the new overlay semi-transparent, then move it above
// the preview
o.alpha = 128
o.layer = 3

// Wait indefinitely until the user terminates the script
while True:
    sleep(1)

```

</details>

<details markdown="1">
<summary>Eye Mechanism RaspPi</summary>

  
```Java


#/usr/bin/env python3

import argparse
import serial
import time

from aiy.vision.inference import CameraInference
from aiy.vision.models import face_detection
from examples.vision.annotator import Annotator
from picamera import PiCamera
from gpiozero import Button


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--num_frames',
        '-n',
        type=int,
        dest='num_frames',
        default=-1,
        help='Sets the number of frames to run for, otherwise runs forever.')
    args = parser.parse_args()
    ser = serial.Serial('/dev/ttyACM0', 115200, write_timeout=0.03)
    ser.write(b"<0.0,0.0,0.0,0.0>")

    with PiCamera() as camera:
        camera.sensor_mode = 4
        camera.resolution = (1640, 1232)
        camera.framerate = 30
        camera.start_preview()
        global shutdown_bool
        shutdown_bool = False
        position = 0
        NoFaceCount = 0
        NoFaceReset = 60
        NoFaceShut = 120
        Top_Lid_Offset = 25  # Top Lid = L2
        Bottom_Lid_Offset = 25  # Bottom Lid = L1
        Top_Lid_Limit = 45
        Bottom_Lid_Limit= 45
        step_size = 2.0

        X_Degrees = 0
        Y_Degrees = 0
        L1_Degrees = 0
        L2_Degrees = 0

        // Trying to slow down serial writing
        Write_Delay = 5  # Milliseconds
        Last_Write_Time = time.time() * 1000  # Milliseconds

        // Soft shutdown function
        def soft_shutdown():
            // X_Degrees = 0
            // Y_Degrees = 0
            // L1_Degrees = -60
            // L2_Degrees = -60
            // ser.write(b"<%d,%d,%d,%d>" % (X_Degrees, Y_Degrees, L1_Degrees, L2_Degrees))
            // camera.stop_preview()
            // quit()
            // thread.interrupt_main()
            print("Shutdown Initiated")
            global shutdown_bool
            shutdown_bool = True

        // Add shutdown function to button
        button = Button(23)
        button.when_pressed = soft_shutdown #pretty sure this doesn't work

        annotator = Annotator(camera, dimensions=(320, 240))
        scale_x = 320 / 1640
        scale_y = 240 / 1232

        // Transform is a function to transform the face bounding box. Incoming boxes are of the form (x, y, width, height). Scale and
        // transform to the form (x1, y1, x2, y2). X and Y are at the top left corner of the face bounding box.
        def transform(bounding_box):
            x, y, width, height = bounding_box
            return (scale_x * x, scale_y * y, scale_x * (x + width),
                    scale_y * (y + height))

        def x_y_to_angles(x, y):
            Span_X = 1640  // This is hard-coded resolution
            Span_Y = 1232
            MinDeg_X = 23  // These are hard coded servo angles, will need to be adjusted to match camera FOV
            MaxDeg_X = -23
            MinDeg_Y = -18
            MaxDeg_Y = 19

            SpanDeg_X = MaxDeg_X - MinDeg_X
            X_Degrees = MinDeg_X + (SpanDeg_X * (x / Span_X))

            SpanDeg_Y = MaxDeg_Y - MinDeg_Y
            Y_Degrees = MinDeg_Y + (SpanDeg_Y * (y / Span_Y))

            return X_Degrees, Y_Degrees

        with CameraInference(face_detection.model()) as inference:
            for i, result in enumerate(inference.run()):
                if i == args.num_frames:
                    break
                if shutdown_bool is True:
                    X_Degrees = 0
                    Y_Degrees = 0
                    L1_Degrees = 0
                    L2_Degrees = 0
                    ser.write(b"<%d,%d,%d,%d>" % (X_Degrees, Y_Degrees, L1_Degrees, L2_Degrees))
                    break
                faces = face_detection.get_faces(result)
                annotator.clear()
                for face in faces:
                    annotator.bounding_box(transform(face.bounding_box), fill=0)  // adding bounding box to preview
                annotator.update()
                print('Iteration #%d: num_faces=%d' % (i, len(faces)))

                if faces:  // Give servo commands if a face is detected
                    face = faces[0]
                    x_corner, y_corner, width, height = face.bounding_box  // bounding box is top left corner
                    x = x_corner + width / 2
                    y = y_corner + height / 2
                    print('             : Face is at X = %d' % x)
                    print('             : Face is at Y = %d' % y)

                    Current_Time = time.time() * 1000
                    if Current_Time >= (Last_Write_Time + Write_Delay):
                        X_Degrees, Y_Degrees = x_y_to_angles(x, y)

                        L1_Degrees = min(Y_Degrees + Bottom_Lid_Offset, Bottom_Lid_Limit)
                        L2_Degrees = min(-Y_Degrees + Top_Lid_Offset, Top_Lid_Limit)

                        ser.cancel_write()  # Cancels previous write
                        ser.reset_output_buffer()  # Removes any data that hasn't been sent yet
                        ser.write(b"<%d,%d,%d,%d>" % (X_Degrees, Y_Degrees, L1_Degrees, L2_Degrees))
                        NoFaceCount = 0


                        // print('            : X Servo Angle =%d' % X_Degrees)
                        // print('            : Y Servo Angle =%d' % Y_Degrees)
                        print(b"<%d,%d,%d,%d>" % (X_Degrees, Y_Degrees, L1_Degrees, L2_Degrees))
                        Last_Write_Time = time.time() * 1000
                    else:
                        print("Waiting for Write Delay")

                else:
                    print('NoFaceCount = %d' % NoFaceCount)
                    if NoFaceReset <= NoFaceCount < NoFaceShut:
                        X_Degrees = 0
                        Y_Degrees = 0
                        L1_Degrees = 12
                        L2_Degrees = 12
                        ser.write(b"<%d,%d,%d,%d>" % (X_Degrees, Y_Degrees, L1_Degrees, L2_Degrees))
                        NoFaceCount = NoFaceCount + 1

                    if NoFaceCount >= NoFaceShut:
                            L1_Degrees = 0
                            L2_Degrees = 0
                            ser.write(b"<%d,%d,%d,%d>" % (X_Degrees, Y_Degrees, L1_Degrees, L2_Degrees))
                            # NoFaceCount = NoFaceCount + 1

                    else:
                        NoFaceCount = NoFaceCount + 1

        camera.stop_preview()


if __name__ == '__main__':
    main()
  ```

</details>

# Debug Resources
These are all the resources I used to debug. Some pertained to my specific problem, others didn't. Each topic is listed, and has drop down under it.
<!--Serial Debug-->
- [serial.serialutil.SerialTimeoutException: Write timeout](https://github.com/espressif/esp-idf/issues/8670)
- [error": cannot open /dev/ttyACM0 port](https://forums.raspberrypi.com/viewtopic.php?t=308293)
- [ipmitool - can't find /dev/ipmi0 or /dev/ipmidev/0](https://serverfault.com/questions/480371/ipmitool-cant-find-dev-ipmi0-or-dev-ipmidev-0)
- [How to fix "SerialTimeoutException: Write timeout" error with NodeMCU and arduino ide](https://stackoverflow.com/questions/56197395/how-to-fix-serialtimeoutexception-write-timeout-error-with-nodemcu-and-arduin)
- [How to solve ReadTimeoutError: HTTPSConnectionPool(host='pypi.python.org', port=443) with pip?](https://stackoverflow.com/questions/43298872/how-to-solve-readtimeouterror-httpsconnectionpoolhost-pypi-python-org-port)
- [Idea to get serial ports working WITH external power supply](https://forum.arduino.cc/t/idea-to-get-serial-ports-working-with-external-power-supply/122413)
- 


# Bill of Materials
<details markdown="1">
<summary>List</summary>
<br>

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| 3D printer | Type does not matter. For this project I used: Ender-3 V2 Neo | $269.99 | <a href="https://store.creality.com/products/ender-3-v2-neo-3d-printer?official-website-product-ender-top=&spm=..product_f4732014-6975-47d2-afe0-f84196b41630.nav_link_store_1.1"> Link </a> |
|:--:|:--:|:--:|:--:|
| Filament | Will need lots of filament, common to get failed parts | $23.59 | <a href="(https://store.creality.com/products/ender-1-75mm-pla-3d-printing-filament-1kg-2p0t?spm=..collection_b55960dc-bbb9-47c1-a06f-657301786e15.albums_1.1&spm_prev=..index.products_display_nav_1.1)"> Link </a> |
|:--:|:--:|:--:|:--:|
| Soldering Iron + Solder | Connecting electrical components. PSA get extra solder | $11.68 | <a href="(https://www.amazon.com/Soldering-soldering-solder-adjustable-temperature/dp/B09DY7CCW5/ref=sr_1_24?keywords=soldering+iron+kit&qid=1689698535&sr=8-24)"> Link </a> |
|:--:|:--:|:--:|:--:|
| M2 Allen Key | For screwing the pieces for the eye mechanisms | $5.99 | <a href="(https://www.amazon.com/B-FUL-Allen-Wrench-Sizes-1-5-6mm/dp/B07PGVFL6W/ref=sr_1_6?hvadid=345542740441&hvdev=c&hvlocphy=9032171&hvnetw=g&hvqmt=e&hvrand=13597869483429660045&hvtargid=kwd-296398166094&hydadcr=29005_10168365&keywords=m2%2Ballen%2Bkey&qid=1689698611&sr=8-6&th=1)"> Link </a> |
|:--:|:--:|:--:|:--:|
| Laser Diodes (650 nm / 5 mW) x2 | To calibrate the eyes for tracking | $6.79 | <a href="(https://www.amazon.com/HiLetgo-10pcs-650nm-Diode-Laser/dp/B071FT9HSV/ref=sr_1_3?hvadid=580752791954&hvdev=c&hvlocphy=9032171&hvnetw=g&hvqmt=e&hvrand=6804941175747209734&hvtargid=kwd-60483869014&hydadcr=28958_14563911&keywords=5v+laser+diode&qid=1689698649&sr=8-3)"> Link </a> |
|:--:|:--:|:--:|:--:|
| M2 Screw Kit | Need variations of M2 screws, better to buy larger kit | $28.68 | <a href="(https://www.amazon.com/iExcell-Metric-Stainless-Washers-Assortment/dp/B082RCL2LW/ref=sr_1_16?crid=36JF9JBFJOTXX&keywords=hex+socket+cap+screws+M2&qid=1689698954&sprefix=hex+socket+cap+screws+m%2Caps%2C157&sr=8-16)"> Link </a> |
|:--:|:--:|:--:|:--:|
| GoBilda Servos x6 | Didn't use GoBilda, but preferable due to better torque and speed. Mechanism bound to run smoother. | $31.99/ea | <a href="(https://www.gobilda.com/2000-series-dual-mode-servo-25-2-torque/)"> Link </a> |
|:--:|:--:|:--:|:--:|
| Screwdriver | Many screws are needed to thread holes into 3D printed parts | $9.86 | <a href="https://www.amazon.com/SUNHZMCKP-Screwdriver-multi-purpose-screwdriver%EF%BC%8CHigh-Torx%EF%BC%8CSuitable/dp/B08BXJTQCV/ref=sr_1_23?keywords=screwdriver&qid=1689699340&sr=8-23"> Link </a> |
|:--:|:--:|:--:|:--:|
| Arduino Uno R3 board | Connects Servo, power, and laser diodes. Holds code for the eye mechanism. | $29.95 | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Google Vision Kit | The is the main hardware for the tracking mechanism. If discontinued, indivually order parts. | $99.95 | <a href="https://www.adafruit.com/product/3780"> Link </a> |
|:--:|:--:|:--:|:--:|
| Raspberry Pi Camera V2 | One is included in the Google Vision Kit, but these cameras are known for being faulty so buy an extra one in case. | $35.00 | <a href="https://www.amazon.com/Raspberry-Pi-Camera-Module-Megapixel/dp/B01ER2SKFS/ref=asc_df_B01ER2SKFS/?tag=hyprod-20&linkCode=df0&hvadid=309776868400&hvpos=&hvnetw=g&hvrand=6777190906928985617&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9032171&hvtargid=pla-406302832745&psc=1"> Link </a> |
|:--:|:--:|:--:|:--:|
| 9V Battery  | Power for Arduino Board | $9.99 | <a href="https://www.amazon.com/Amazon-Basics-Performance-All-Purpose-Batteries/dp/B0774D64LT/ref=sr_1_5?hvadid=410049284807&hvdev=c&hvlocphy=9032171&hvnetw=g&hvqmt=e&hvrand=7214474178270115369&hvtargid=kwd-464156346145&hydadcr=4229_11143345&keywords=9%2Bvolt&qid=1689699881&sr=8-5&th=1"> Link </a> |
|:--:|:--:|:--:|:--:|
| Wire | Need a whole spool for connecting servos, Raspberry Pi, Camera, laser diodes, power, and 5V relay | $14.99 | <a href="https://www.amazon.com/TUOFENG-Hookup-Wires-6-Different-Colored/dp/B07TX6BX47/ref=sr_1_29?hvadid=580555199979&hvdev=c&hvlocphy=9032171&hvnetw=g&hvqmt=e&hvrand=14313782213096234770&hvtargid=kwd-42459710&hydadcr=6891_13209663&keywords=electrical+wire&qid=1689699707&sr=8-29"> Link </a> |
|:--:|:--:|:--:|:--:|
| Battery power adapter | Can be plugged into the 9V battery and soldered onto the servo hat | $5.99 | <a href="https://www.amazon.com/Battery-Connector-I-Type-Plastic-Housing/dp/B07TRKYZCH"> Link </a> |
|:--:|:--:|:--:|:--:|
| Power Bank with USB A output at 5V | To power raspberry pi without needing an external wall | $12.99 | <a href="https://www.amazon.com/EnergyQC-Portable-Charging-Ultra-Compact-Compatible/dp/B09Z6S7L4P/ref=sr_1_11?keywords=5v+power+bank&qid=1689700110&sr=8-11"> Link </a> |
|:--:|:--:|:--:|:--:|
| Electrical Tape | General use | $24.99 | <a href="https://www.amazon.com/Weather-Resistant-Colored-Electrical-Nova-Electric/dp/B076JJDS1L/ref=sr_1_8?hvadid=616991206274&hvdev=c&hvlocphy=9032171&hvnetw=g&hvqmt=e&hvrand=3904053447894082767&hvtargid=kwd-21804356&hydadcr=24660_13611807&keywords=electrical+tape&qid=1689703562&sr=8-8"> Link </a> |
|:--:|:--:|:--:|:--:|
| MOS Module | Regulate current for the laser diodes | $10.99 | <a href="[https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/](https://www.amazon.com/Onyehn-Mosfet-Button-Arduino-Raspberry/dp/B07GLNCRR4)"> Link </a> |
|:--:|:--:|:--:|:--:|
| Heat Shrink | General Use | $16.99 | <a href="[https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/](https://www.amazon.com/Wirefy-275-Heat-Shrink-Tubing/dp/B084GWYX42/ref=sr_1_3?hvadid=664755229846&hvdev=c&hvlocphy=9032171&hvnetw=g&hvqmt=e&hvrand=11486415139271556957&hvtargid=kwd-341631528701&hydadcr=3644_13690564&keywords=wiring+heat+shrink+tubing&qid=1689703638&sr=8-3)"> Link </a> |
|:--:|:--:|:--:|:--:|
| HDMI Cable | Mini HDMI to HDMI | $10.98 | <a href="https://www.amazon.com/JSAUX-Aluminum-Compatible-Camcorder-Raspberry/dp/B08DK4LVYX/ref=sr_1_3?hvadid=233484030855&hvdev=c&hvlocphy=9032171&hvnetw=g&hvqmt=e&hvrand=17895052026618409799&hvtargid=kwd-7403862453&hydadcr=18887_10145554&keywords=mini+hdmi+to+hdmi&qid=1689703673&sr=8-3"> Link </a> |
|:--:|:--:|:--:|:--:|
| HDMI Capture card | HDMI to USB A | $18.99 | <a href="https://www.amazon.com/AMZHRLY-Recording-Camcorder-Streaming-Conference/dp/B0974MJY14/ref=sr_1_4?hvadid=570507335023&hvdev=c&hvlocphy=9032171&hvnetw=g&hvqmt=e&hvrand=6672548318186623628&hvtargid=kwd-4936107645&hydadcr=18007_13462294&keywords=hdmi+capture+card&qid=1689703742&sr=8-4"> Link </a> |
|:--:|:--:|:--:|:--:|
| USB A to USB C Adapter | Only needed for mac | $6.99 | <a href="https://www.amazon.com/Temdan-USB-Adapter-SuperSpeed-Transfer/dp/B0BMVHHT47/ref=sr_1_3?hvadid=557527747876&hvdev=c&hvlocphy=9032171&hvnetw=g&hvqmt=e&hvrand=9406352976371971107&hvtargid=kwd-350624214876&hydadcr=18004_13447364&keywords=usb+a+to+usb-c&qid=1689703846&sr=8-3"> Link </a> |
|:--:|:--:|:--:|:--:|
| Micro USB Port | Used to connect all the peripherals and serial | $6.99 | <a href="https://www.amazon.com/LoveRPi-MicroUSB-Port-Black-Raspberry/dp/B01HYJLZH6/ref=sr_1_3?hvadid=581221078510&hvdev=c&hvlocphy=9032171&hvnetw=g&hvqmt=e&hvrand=11821711121676020260&hvtargid=kwd-317261059863&hydadcr=24659_11410797&keywords=micro+usb+to+hub&qid=1689703973&s=electronics&sr=1-3"> Link </a> |
|:--:|:--:|:--:|:--:|
| Serial Wire | Send code from Raspberry Pi to Arduino | $6.99 | <a href="(https://www.amazon.com/DIYmall-Cable-Arduino-2560-Pack/dp/B09JRXT1TY/ref=sr_1_4?keywords=arduino+usb+cable&qid=1689704177&sr=8-4)"> Link </a> |
|:--:|:--:|:--:|:--:|
| Nylon Screw set | Fits Raspberry Pi boards to connect to 3D print | $16.99 | <a href="(https://www.amazon.com/Readytosky-Plastic-Machine-Assortment-Organizer/dp/B07Q3W65FV)"> Link </a> |
|:--:|:--:|:--:|:--:|
| Tripod (Optional) | Used to hold the face | $20.89 | <a href="https://www.amazon.com/AmazonBasics-Lightweight-Camera-Mount-Tripod/dp/B00XI87KV8/ref=sr_1_4?hvadid=616863175854&hvdev=c&hvlocphy=9032171&hvnetw=g&hvqmt=e&hvrand=9342669726832163304&hvtargid=kwd-10922890&hydadcr=24662_13611802&keywords=camera+tripod&qid=1689704480&sr=8-4"> Link </a> |
|:--:|:--:|:--:|:--:|
| Hex Nut | Used for Tripod; hold it in place | $5.98 | <a href="https://www.amazon.com/Prime-Line-9073221-Finished-Plated-50-Pack/dp/B07FCRWF5N/ref=sr_1_3?hvadid=234265091908&hvdev=c&hvlocphy=9032171&hvnetw=g&hvqmt=e&hvrand=2433767895350711731&hvtargid=kwd-378380042141&hydadcr=24664_9649023&keywords=hex+nut+1%2F4&qid=1689704895&sr=8-3"> Link </a> |
|:--:|:--:|:--:|:--:|
| Jumper wires M-F | Male to female jumper wires to connect relay | $5.49 | <a href="https://www.amazon.com/GenBasic-Solderless-Dupont-Compatible-Breadboard-Prototyping/dp/B077N643L7/ref=sr_1_4?hvadid=570432716151&hvdev=c&hvlocphy=9032171&hvnetw=g&hvqmt=e&hvrand=5073019930584598753&hvtargid=kwd-45429255008&hydadcr=19138_13375004&keywords=jumper%2Bwires%2Bmale%2Bto%2Bfemale&qid=1689704517&sr=8-4&th=1"> Link </a> |

</details>

