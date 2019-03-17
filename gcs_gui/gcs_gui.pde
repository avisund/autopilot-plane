import java.lang.Math;
import processing.serial.*;


Serial myPort;
String oldval, val;
byte number = 0;

void setup()
{
  println("test");
  String portName = Serial.list()[1]; // change the 0 to a 1 or 2 etc. to match port
  myPort = new Serial(this, portName, 115200);
}

void draw()
{
  if ( myPort.available() > 0) 
  {
    val = myPort.readStringUntil('\n');
  } 
  if (val != oldval) {
    println(val);
    oldval = val;
  }
  delay(200);
  if (number >= pow(2, 8)) {
    number = 0;
  }
  myPort.write("Test " + number);
  //println("Test " + number);
  ++number;
}
