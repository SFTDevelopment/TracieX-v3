#include <EEPROM.h>

void readMemory(){
  for(int i = 0; i < 512; i++){
    Serial.print(EEPROM.read(i));
    if(i<511)
      Serial.print(",");
    else
      Serial.println();
  }
}

bool writeMemory(uint16_t loc, uint8_t data){
  bool outcome = false;
  if(loc < 512){
    EEPROM.write(loc, data);
    outcome = true;
  }
  return outcome;
}
