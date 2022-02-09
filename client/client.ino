// With the prescaler set to 128, and initial timer count set to 130, each tick
// variable increment represents exactly 16000 ticks of the raw clock.
extern uint32_t ticks = 0;

ISR(TIMER2_OVF_vect){
    ++ticks;

    TCNT2 = 130;
    TIFR2 = 0x00;
};

void setup() {
    Serial.begin(9600);

    TCCR2B = 0x00;        // Disable Timer2 while we set it up
    TCNT2  = 130;         // Reset Timer Count to 130 out of 255
    TIFR2  = 0x00;        // Timer2 INT Flag Reg: Clear Timer Overflow Flag
    TIMSK2 = 0x01;        // Timer2 INT Reg: Timer2 Overflow Interrupt Enable
    TCCR2A = 0x00;        // Timer2 Control Reg A: Wave Gen Mode normal
    TCCR2B = 0x05;        // Timer2 Control Reg B: Timer Prescaler set to 128
}


void loop() {
    if(ticks >= 60000){ // 60000 = 1min with 16MHz clock frequency
        ticks = 0;
        Serial.write(0);
    }
}

