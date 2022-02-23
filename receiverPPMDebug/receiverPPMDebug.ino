#include <PulsePosition.h>

PulsePositionInput RC;
float input[6];

void setup() {
  RC.begin(5);
}

void loop() {
  for(int channel = 0; channel < 6; channel++) {
    input[channel] = RC.read(channel+1);
    Serial.print(" | ");
    Serial.print(input[channel]);
  }
  Serial.println();
}
