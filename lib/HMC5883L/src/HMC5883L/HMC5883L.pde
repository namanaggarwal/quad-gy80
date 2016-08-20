
import processing.serial.*;

Serial myPort;
float deg;

float angle;
float jitter;

void setup() {

  // set window size
  size(640, 360);
  noStroke();

  // List all available serial ports
  printArray(Serial.list());

  // Probably open Serial.printArray()[0]
  myPort = new Serial(this, Serial.list()[0], 9600);

  // don't generate a serial event unless a newline character is detected
  myPort.bufferUntil('\n');

  
  fill(123, 23, 211, 112);
  rectMode(CENTER);
  angle = deg = 0;
}

void draw() {
  background(51);
  //  delay(100);
  // during even-numbered seconds (0, 2, 4, 6...)
  angle = deg * PI/180;
  //float c = cos(angle);
  translate(width/2, height/2);
  rotate(angle);
  rect(0, 0, 180, 180, 0, 18, 18, 18);
  
}

void serialEvent (Serial myPort) {
  // get ascii string:
  String inString = myPort.readStringUntil('\n');
  if (inString != null) {
    inString = trim(inString);
    deg = float(inString);
    println(deg);
  }
}
