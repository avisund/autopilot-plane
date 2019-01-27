import processing.serial.*;

Serial myPort;
String oldval, val;

void setup()
{
  String portName = Serial.list()[1]; // change the 0 to a 1 or 2 etc. to match port
  myPort = new Serial(this, portName, 9600);
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
}
