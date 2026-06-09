# Robotic Portrait Arm
A 3-jointed robotic arm that draws portraits from webcam input. The arm uses inverse kinematics to translate edge-detected strokes into servo angles, putting pen to paper.
<!---Talk about my three jointed arm, and talk about biggest challenges+modifications]--->

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Jotin S | Adlai E. Stevenson High School | Computer Engineering | Incoming Sophomore

**Replace the BlueStamp logo below with an image of yourself and your completed project. Follow the guide [here](https://tomcam.github.io/least-github-pages/adding-images-github-pages-site.html) if you need help.**

![Headstone Image](logo.svg)
  
# Final Milestone

**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**

<iframe width="560" height="315" src="https://www.youtube.com/embed/F7M7imOVGug" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

Modification milestone — adding the portrait-drawing CV pipeline on top of the base arm.



# Second Milestone

**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**

<iframe width="560" height="315" src="https://www.youtube.com/embed/y3VAmNlER5Y" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

Getting to start coding — writing the servo control logic and beginning the software side of the project.

# First Milestone

**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**

<iframe width="560" height="315" src="https://www.youtube.com/embed/CaCazFBhYKs" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

Rotating servos 90 degrees and getting the entire structure fully assembled and wired.

# Build Progress

**Day 1**

![nano board setup](nano-board-setup.png)

set up the nano board and shield for my base, also connected my servo into it along with battery case. went to building because i do not have the right port in my computer to get arduino set up and connected.

**Day 2**

![code screenshot](code-screenshot.png)

created program to rotate servos to what ever value is preferenced. for me i did 90 degrees.

![build progress day 2](build-progress-day2.png)

replaced the nano shield after i realized the original one didnt have battery installed, as i needed more power.

# Schematics 
Here's where you'll put images of your schematics. [Tinkercad](https://www.tinkercad.com/blog/official-guide-to-tinkercad-circuits) and [Fritzing](https://fritzing.org/learning/) are both great resoruces to create professional schematic diagrams, though BSE recommends Tinkercad becuase it can be done easily and for free in the browser. 

# Code

## Adjust Servo Rotation Angle

Reads a digit (1–9) from the serial monitor and maps it to a servo angle (20–180°). Sends the corresponding PWM pulse 50 times to hold the position.

```c++
int servopin = 10;  // servo signal line on digital pin 10
int myangle;        // angle variable (0–180)
int pulsewidth;     // pulse width variable
int val;            // serial input digit (0–9)

void servopulse(int servopin, int myangle) {
  pulsewidth = (myangle * 11) + 500;
  digitalWrite(servopin, HIGH);
  delayMicroseconds(pulsewidth);
  digitalWrite(servopin, LOW);
  delay(20 - pulsewidth / 1000);
}

void setup() {
  pinMode(servopin, OUTPUT);
  Serial.begin(9600);
  Serial.println("servo=o_seral_simple ready");
}

void loop() {
  val = Serial.read();

  if (val > '0' && val <= '9') {
    val = val - '0';
    val = val * (180 / 9);

    Serial.print("moving servo to ");
    Serial.print(val, DEC);
    Serial.println();

    for (int i = 0; i <= 50; i++) {
      servopulse(servopin, val);
    }
  }
}
```

# Bill of Materials
Here's where you'll list the parts in your project. To add more rows, just copy and paste the example rows below.
Don't forget to place the link of where to buy each component inside the quotation marks in the corresponding row after href =. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize this to your project needs. 

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |

# Other Resources/Examples
One of the best parts about Github is that you can view how other people set up their own work. Here are some past BSE portfolios that are awesome examples. You can view how they set up their portfolio, and you can view their index.md files to understand how they implemented different portfolio components.
- [Example 1](https://trashytuber.github.io/YimingJiaBlueStamp/)
- [Example 2](https://sviatil0.github.io/Sviatoslav_BSE/)
- [Example 3](https://arneshkumar.github.io/arneshbluestamp/)

To watch the BSE tutorial on how to create a portfolio, click here.
