#include <Serialize.h>

  Serializer se;
  char key[]={'[','2','4',']'}, LF='\n';
  //testes:
  int16_t t1 = -512;
  uint32_t t2 = 0xABCDEF44; // (2882400068)
  
void setup() {
  Serial.begin(9600);
//   Serial3.begin(9600);
  // put your setup code here, to run once:
  
  
//   Serial.print("variable: ");
//   Serial.print(x);
//   Serial.print(" /HEX: ");
//   Serial.print(x, HEX);
//   Serial.print("\tvariable: ");
//   Serial.print(y);
//   Serial.print(" /HEX: ");
//   Serial.println(y, HEX);
  
  for(int i=0;i<sizeof(key);i++){
    se.add(&key[i],1);
  }
  se.add(&t1,sizeof(t1));
  se.add(&t2,sizeof(t2));  
  
  Serial.println(t1,HEX);
}

void loop() {/*
  for(int i=0; i< se.getCount(); ++i){
    Serial3.write(se.getBufferIndex(i));
  }
    Serial.println();
    delay(100);*/
}
