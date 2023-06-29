//lookup table to relate encoder counts to number of layers (including half layers)
// int table[] = { 0, 566, 1164, 1736, 2331, 2930, 3507, 4104, 4679, 5259, 5836, 6407, 7008, 7538, 8174, 8743, 9300, 9853, 10410 };
int table[]={0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};

//function which reads lookup table
int lookup_table(float layers) {
  int index = layers * 2 - 2;
  if (index < 0) {
    Serial.println("Invalid layer input");
  }
  return table[index];
}