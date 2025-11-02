void setup() {
    Serial.begin(9600); // Initialize serial communication
}

void loop() {
    // Check if there is any data available to be read
    if (Serial.available() > 0) {
        // Read the incoming byte
        char incomingByte = Serial.read();
        
        // Print the received byte
        Serial.print("Received: ");
        Serial.println(incomingByte);
    }
    else {
      Serial.println("<1;0;0>");
    }
    
    // Your other code here
    // ...
    Serial.println("<0;0;0>");
}
