// Pin assignments
const int JL_X = A1;  // Left joystick X-axis
const int JL_Y = A2;  // Left joystick Y-axis
const int JR_X = A3;  // Right joystick X-axis
const int JR_Y = A4;  // Right joystick Y-axis
const int POT = A0;   // Potentiometer

// Variables to store readings
int jl_x_val, jl_y_val, jr_x_val, jr_y_val, pot_val;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Print header
  Serial.println("Joystick & Potentiometer Values:");
  Serial.println("JL_X\tJL_Y\tJR_X\tJR_Y\tPOT");
}

void loop() {
  // Read all analog inputs
  jl_x_val = analogRead(JL_X);
  jl_y_val = analogRead(JL_Y);
  jr_x_val = analogRead(JR_X);
  jr_y_val = analogRead(JR_Y);
  pot_val = analogRead(POT);
  
  // Print values in a tab-separated format
  Serial.print(jl_x_val);
  Serial.print("\t");
  Serial.print(jl_y_val);
  Serial.print("\t");
  Serial.print(jr_x_val);
  Serial.print("\t");
  Serial.print(jr_y_val);
  Serial.print("\t");
  Serial.println(pot_val);
  
  // Small delay for readability
  delay(100);
}