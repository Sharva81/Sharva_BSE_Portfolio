# Watchman Robot
Ever feel as if there is pair of eyes always watching you 24/7? Ever feel as if you are getting stalked? Well this robot does just that. The Watchman Robot useses a pair of mechanical eyes built from 3D printed parts along with a camera to keep a constant "eye" on you behind its creepy mask.  

| **Name** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Sharva P | Mission San Jose Highschool | Mechanical Engineering | Incoming Senior

<!-- **Replace the BlueStamp logo below with an image of yourself and your completed project. Follow the guide [here](https://tomcam.github.io/least-github-pages/adding-images-github-pages-site.html) if you need help.**

![Headstone Image](logo.svg)
-->
  
# Final Milestone

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


# Other Resources/Examples
One of the best parts about Github is that you can view how other people set up their own work. Here are some past BSE portfolios that are awesome examples. You can view how they set up their portfolio, and you can view their index.md files to understand how they implemented different portfolio components.
- [Example 1](https://trashytuber.github.io/YimingJiaBlueStamp/)
- [Example 2](https://sviatil0.github.io/Sviatoslav_BSE/)
- [Example 3](https://arneshkumar.github.io/arneshbluestamp/)

To watch the BSE tutorial on how to create a portfolio, click here.

-->



# Code

<details markdown="1">
  
<summary>Java/Arduino</summary>
<br>
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

</details>


<details markdown="1">
  
<summary>Python/Raspberry Pi</summary>
<br>
# Python

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
<!---
# Bill of Materials
Here's where you'll list the parts in your project. To add more rows, just copy and paste the example rows below.
Don't forget to place the link of where to buy each component inside the quotation marks in the corresponding row after href =. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize this to your project needs. 

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| 3D printer | Type does not matter. For this project I used: Ender-3 V2 Neo | $299.99 | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Filament | Will need lots of filament, common to get failed parts | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Soldering Iron + Solder | Connecting electrical components | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| M2 Allen Key | For screwing the pieces for the eye mechanisms | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Laser Diodes (650 nm / 5 mW) x2 | To calibrate the eyes for tracking | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| M2 Screw Kit | Need variations of M2 screws, better to buy larger kit | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| GoBilda Servos x6 | Didn't use GoBilda, but preferable due to better torque and speed. Mechanism bound to run smoother. | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Screwdrver | Many screws are needed to thread holes into 3D printed parts | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Arduino Uno R3 board | Connects Servo, power, and laser diodes. Holds code for the eye mechanism. | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Google Vision Kit | The is the main hardware for the tracking mechanism. | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Positive Wire | Need a whole spool for connecting servos, Raspberry Pi, Camera, laser diodes, power, and 5V relay | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Ground Wire | Need a whole spool for connecting servos, Raspberry Pi, Camera, laser diodes, power, annd 5V relay | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Raspberry Pi Camera V2 | One is included in the Google Vision Kit, but these cameras are known for being faulty so buy an extra one in case. | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| AA Batteries  | Power for Arduino Board | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Battery Pack | Need one that holds 4 AA Batteries | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|

