String buf;

void setup() {
Serial.begin(115200);
delay(500);



}

void loop() {
  
//Serial.println("G1 X100 Y100 F2000 T4000");


for(int i = 50 ; i < 3000; i++){
 
    
    //buf += F("X");
    buf += String(i);
    buf +=F(" "); 
    //buf += F("Y");
    buf += String(i);
     buf +=F(" "); 
     buf += String(i);
    buf +=F(" "); 
   
    buf += String(i*2);
    buf +=F(" "); 
     buf += String(i);
    buf +=F(" "); 
   
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
  }

  }
