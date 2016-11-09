void setup()
{
  //Inicia Serial:
  Serial.begin(115200);
  Serial3.begin(115200);
  //----------------------
  	Serial.flush();
	Serial3.flush();
  	Serial.println("print('Arduino <-> ESP ok!')");
}

void loop()
{
	if(Serial.available()){
	    char b = Serial.read();
	    Serial3.write(b);
	}

	if(Serial3.available()){
	    char b = Serial3.read();
	    Serial.write(b);
	}

	Serial.flush();
	Serial3.flush();
}