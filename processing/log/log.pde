import processing.serial.*;
Serial mySerial;
PrintWriter output;
void setup() {
    mySerial = new Serial( this, Serial.list()[1], 9600);
    output = createWriter ("data.txt");
}

void draw() {
    if (mySerial.available() > 0) {
        String value = mySerial.readString();
        if(value != null) {
            output.print (value);
        }
    }
}

void keyPressed() {
    output.flush();
    output.close();
    exit();
}