String buf;

void setup() {
  Serial.begin(115200);
  delay(500);



}

void loop() {

  if(Serial.available()){
        buf = Serial.readStringUntil("\n");
 
        Serial.println("O5" + buf );
    }
}
  /*
    for(int i = 50 ; i < 3000; i++){

      buf += F("O");
      buf += F(" ");
      buf += F("X");
      buf += String(i);
      buf +=F(" ");
      buf += F("Y0");
      buf += String(i);

       buf +=F(" ");
      buf += F("Y1");

      buf += String(i*2);
      buf +=F(" ");
      buf += F("Y2");


      buf += String(i*4);

      buf += F("\n");
      Serial.println(buf);
      buf = "";
      delay(50);
    /*
    Serial.print(String(i));
    Serial.print(" ");
    Serial.print(String(i));
    Serial.print("\n");
    delay(50)*/
