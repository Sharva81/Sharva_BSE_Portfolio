# Watchman Robot
Ever feel as if there is pair of eyes always watching you 24/7? Ever feel as if you are getting stalked? Well this robot does just that. The Watchman Robot useses a pair of mechanical eyes built from 3D printed parts along with a camera to keep a constant "eye" on you behind its creepy mask.  

| **Name** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Sharva P | Mission San Jose Highschool | Electrical Engineering | Incoming Senior

<!-- **Replace the BlueStamp logo below with an image of yourself and your completed project. Follow the guide [here](https://tomcam.github.io/least-github-pages/adding-images-github-pages-site.html) if you need help.**

![Headstone Image](logo.svg)
  
# Final Milestone
For your final milestone, explain the outcome of your project. Key details to include are:
- What you've accomplished since your previous milestone
- What your biggest challenges and triumphs were at BSE
- A summary of key topics you learned about
- What you hope to learn in the future after everything you've learned at BSE


# Second Milestone
For your second milestone, explain what you've worked on since your previous milestone. You can highlight:
- Technical details of what you've accomplished and how they contribute to the final goal
- What has been surprising about the project so far
- Previous challenges you faced that you overcame
- What needs to be completed before your final milestone 
-->

# First Milestone
<iframe width="560" height="315" src="https://www.youtube.com/embed/CKRm5gqWMoU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe> 

First let’s talk about the components of the Robot. The eye mechanism is a complex robotic eye that is powered with 6 servos in total. 4 of the 6 servos are used to control the up down left and right movements of the eyeball. The other 2 are used for closing and opening eyelids. Inside eyeballs are laser diodes to help calibrate. Next let’s talk about the arduino board. I used an Arduino Uno r3 and a servo sheild, whiched is stacked on top, to power servos. It also powers the laser diodes that are regulated with a 5V relay. Finally I used a AIY vision kit to track the human face from camera. The challenges I faced had to do mainly with the eye mechanism. The assembly of the eye was took almost 3 days because the 3D printer and the parts to be assembled were small. The fragility of the parts made it prone to snapping; therefore, needing more prints. Since they were small eveerything was handtighthened by a allen key. Next was to figure out the wiring needed to power all the different parts. This was the first time using Arduino so I had to learn everything about it before assembling. My plan for the future is to figure out the code for the eyemechanism and the face tracking.


# Starter Project
<iframe width="560" height="315" src="https://www.youtube.com/embed/pJAbVRicJ2c" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

For my starter project I decided to make the TV-B-Gone kit. This small compact device majorly impacts large TVs with just a push of a button. No matter the company, place, or area, this pocket device can turn any TV off. So let us talk about the key parts in this project. Starting with the power supply, it uses 2 AA batteries. To emit the light to interact with the TVs, it uses IR LEDs which emit at 940nm. Obviously we have to include the button to start the code and finally it uses things like resonators and transistors to keep the current and electricity constant throughout the device. This was a great project to introduce circuits and LEDs which will help me in my main project that incorporates laser diodes. 


<!-- # Schematics 
Here's where you'll put images of your schematics. [Tinkercad](https://www.tinkercad.com/blog/official-guide-to-tinkercad-circuits) and [Fritzing](https://fritzing.org/learning/) are both great resoruces to create professional schematic diagrams, though BSE recommends Tinkercad becuase it can be done easily and for free in the browser. 

# Code
Here's where you'll put your code. The syntax below places it into a block of code. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize it to your project needs. 

# Bill of Materials
Here's where you'll list the parts in your project. To add more rows, just copy and paste the example rows below.
Don't forget to place the link of where to buy each component inside the quotation marks in the corresponding row after href =. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize this to your project needs. 

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
|:--:|:--:|:--:|:--:|

# Other Resources/Examples
One of the best parts about Github is that you can view how other people set up their own work. Here are some past BSE portfolios that are awesome examples. You can view how they set up their portfolio, and you can view their index.md files to understand how they implemented different portfolio components.
- [Example 1](https://trashytuber.github.io/YimingJiaBlueStamp/)
- [Example 2](https://sviatil0.github.io/Sviatoslav_BSE/)
- [Example 3](https://arneshkumar.github.io/arneshbluestamp/)

To watch the BSE tutorial on how to create a portfolio, click here.

-->

# Code
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

```Python
#!/usr/bin/env python3

import picamera
from PIL import Image
from time import sleep

camera = picamera.PiCamera()
camera.resolution = (1640, 1232)
camera.framerate = 24
camera.start_preview()

# Load the arbitrarily sized image
img = Image.open('Crosshair_Black.png')
# Create an image padded to the required size with
# mode 'RGB'
pad = Image.new('RGB', (
    ((img.size[0] + 31) // 32) * 32,
    ((img.size[1] + 15) // 16) * 16,
    ))
# Paste the original image into the padded one
pad.paste(img, (0, 0))

# Add the overlay with the padded image as the source,
# but the original image's dimensions
o = camera.add_overlay(pad.tobytes(), size=img.size)
# By default, the overlay is in layer 0, beneath the
# preview (which defaults to layer 2). Here we make
# the new overlay semi-transparent, then move it above
# the preview
o.alpha = 128
o.layer = 3

# Wait indefinitely until the user terminates the script
while True:
    sleep(1)


#!/usr/bin/env python3

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

        # Trying to slow down serial writing
        Write_Delay = 5  # Milliseconds
        Last_Write_Time = time.time() * 1000  # Milliseconds

        # Soft shutdown function
        def soft_shutdown():
            # X_Degrees = 0
            # Y_Degrees = 0
            # L1_Degrees = -60
            # L2_Degrees = -60
            # ser.write(b"<%d,%d,%d,%d>" % (X_Degrees, Y_Degrees, L1_Degrees, L2_Degrees))
            # camera.stop_preview()
            # quit()
            # thread.interrupt_main()
            print("Shutdown Initiated")
            global shutdown_bool
            shutdown_bool = True

        # Add shutdown function to button
        button = Button(23)
        button.when_pressed = soft_shutdown #pretty sure this doesn't work

        annotator = Annotator(camera, dimensions=(320, 240))
        scale_x = 320 / 1640
        scale_y = 240 / 1232

        # Transform is a function to transform the face bounding box. Incoming boxes are of the form (x, y, width, height). Scale and
        # transform to the form (x1, y1, x2, y2). X and Y are at the top left corner of the face bounding box.
        def transform(bounding_box):
            x, y, width, height = bounding_box
            return (scale_x * x, scale_y * y, scale_x * (x + width),
                    scale_y * (y + height))

        def x_y_to_angles(x, y):
            Span_X = 1640  # This is hard-coded resolution
            Span_Y = 1232
            MinDeg_X = 23  # These are hard coded servo angles, will need to be adjusted to match camera FOV
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
                    annotator.bounding_box(transform(face.bounding_box), fill=0)  # adding bounding box to preview
                annotator.update()
                print('Iteration #%d: num_faces=%d' % (i, len(faces)))

                if faces:  # Give servo commands if a face is detected
                    face = faces[0]
                    x_corner, y_corner, width, height = face.bounding_box  # bounding box is top left corner
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


                        # print('            : X Servo Angle =%d' % X_Degrees)
                        # print('            : Y Servo Angle =%d' % Y_Degrees)
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
