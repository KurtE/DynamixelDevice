# Dynamixel Device 

Dynamixel Device library used to make Some Arduino devices act like devices on a Dynamixel device like an AX-12 servo. 

Note: the vast majority of this work was done by the Trossen Robotics User R3n33 as well as by Eric shown in the threads: 

http://forums.trossenrobotics.com/showthread.php?7690-DIY-FSR-Feet-for-the-HR-OS1

http://forums.trossenrobotics.com/showthread.php?7620-Camera-Vision-Processing-%28within-the-HR-OS1-Framework%29&p=69242#post69242

Warning: Work in Progress
---



There are a couple of example sketches that are using this library including;

RGB and Neopixel
---

There are two example sketches included here: 

**LEDHand** - This is r3n33's original sketch updated to changes in the librayr that was built to work with an Adafruit Pro Trinket 5v and one RGB led and was shown in the video as part of the camera vision thread mentioned earlier.

**Neopixel_Servo** - This is my WIP version of LedHand that was converted over to use Adafruit Neopixels to display.  The code is also setup to experiment spying on messages intended for different servos and can support multiple Neopixels, where in the current code can display a pixel for each logical leg of a hexapod...  I have been playing with this on both Adafruit Pro trinkets as well as on a Teensy 3.2

Trossen Robotics HROS1 FSR enabled feet
---

**FSR_Foot** - This is r3n33's sketch that reads 4 FSR sensors to detect when the HROS1's foot is on the ground.  Again there is a lot more details about this up on the forum thread: http://forums.trossenrobotics.com/showthread.php?7690-DIY-FSR-Feet-for-the-HR-OS1

Warning: Work in Progress
---


 