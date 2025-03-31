// Pin configuration
const int C = A0;  
const int R3 = A1;  
const int L1 = A2;  
const int R2 = A3;   
const int L3 = A4;  
const int L2 = A5;  
const int R1 = 2;   

const int threshold = 500; // Define threshold for HIGH/LOW decision

void setup() {
  Serial.begin(9600); // Start serial communication
  pinMode(R1, INPUT); // R1 is digital
  pinMode(R3, INPUT); // R3 is digital
}

void loop() {
  // Read analog values from sensors
  int L1_val = analogRead(L1);
  int L2_val = analogRead(L2);
  int L3_val = analogRead(L3);
  int C_val  = analogRead(C);
  int R2_val = analogRead(R2);

  // Convert analog readings to HIGH/LOW based on threshold
  String L1_status = (L1_val > threshold) ? "HIGH" : "LOW";
  String L2_status = (L2_val > threshold) ? "HIGH" : "LOW";
  String L3_status = (L3_val > threshold) ? "HIGH" : "LOW";
  String C_status  = (C_val > threshold)  ? "HIGH" : "LOW";
  String R2_status = (R2_val > threshold) ? "HIGH" : "LOW";

  // Read digital values directly
  String R1_status = digitalRead(R1) ? "HIGH" : "LOW";
  String R3_status = digitalRead(R3) ? "HIGH" : "LOW";

  // Print header
  Serial.println("\n+------+-----+-----+-----+-----+-----+-----+-----+");
  Serial.println("| IR#  | L1  | L2  | L3  |  C  | R1  | R2  | R3  |");
  Serial.println("+------+-----+-----+-----+-----+-----+-----+-----+");

  // Print sensor status as HIGH/LOW
  Serial.print("| Val  | ");
  Serial.print(L1_status); Serial.print(" | ");
  Serial.print(L2_status); Serial.print(" | ");
  Serial.print(L3_status); Serial.print(" | ");
  Serial.print(C_status);  Serial.print(" | ");
  Serial.print(R1_status); Serial.print(" | ");
  Serial.print(R2_status); Serial.print(" | ");
  Serial.print(R3_status); Serial.println(" |");

  Serial.println("+------+-----+-----+-----+-----+-----+-----+-----+");

  delay(1000); // Delay for readability
}
