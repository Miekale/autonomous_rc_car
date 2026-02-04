const byte HANDSHAKE_INIT = 0x11; 
const byte HANDSHAKE_ACK  = 0x12; 

void handshakeConnection() {
    while (true) {
        Serial.write(HANDSHAKE_INIT);
        delay(500); 

        if (Serial.available() > 0) {
            if (Serial.read() == HANDSHAKE_ACK) {
                return;
            }
        }
    }
}

void setup() {
  Serial.begin(115200);
  handshakeConnection(); 
}

void loop() {
    // pass
}