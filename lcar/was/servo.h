


void InitServos();

void SetServo(int ch,float f);  //-1.0 to 1.0

void RawSetServo(int ch,float v); //Limits -1.0 to +1.0

extern volatile bool ServoUsed[4];
