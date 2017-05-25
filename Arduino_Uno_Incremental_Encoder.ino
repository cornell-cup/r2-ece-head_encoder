volatile uint16_t timercapture = 0;
int rising = 0;

void initTimer(void) {

  // Input Capture setup
  // ICNC1: Enable Input Capture Noise Canceler
  // ICES1: =1 for trigger on rising edge
  // CS10: =1 set prescaler to 1x system clock (F_CPU)
  TCCR1A = 0;
  TCCR1B = (0<<ICNC1) | (0<<ICES1) | (1<<CS10);
  TCCR1C = 0;

  //catchFallingEdge(); // initialize to catch
  { TCCR1B &= ~(1<<ICES1); TIFR1 |= (1<<ICF1); rising = 0; }

  // Interrupt setup
  // ICIE1: Input capture
  // TOIE1: Timer1 overflow
  TIFR1 = (1<<ICF1) | (1<<TOV1);        // clear pending
  TIMSK1 = (1<<ICIE1) | (1<<TOIE1); // and enable

  // Set up the Input Capture pin, ICP1, which corresponds to Arduino D8
  pinMode(8, INPUT);
  digitalWrite(8, 0);       // leave floating to count 60 Hz etc.
  //digitalWrite(8, 1);         // or enable the pullup
}

ISR(TIMER1_CAPT_vect) {
  union twobyte {
    uint32_t word;
    uint8_t  byte[2];
  } timevalue;

  timevalue.byte[0] = ICR1L;        // grab captured timer value
  timevalue.byte[1] = ICR1H;        // grab captured timer value

  timercapture = (ICR1H<<8) & ICR1L;

  // watch for the other edge to catch the half-pulse width
  //rising ? catchFallingEdge() : catchRisingEdge();
  if (rising) {
    TCCR1B &= ~(1<<ICES1);
    TIFR1 |= (1<<ICF1);
    rising = 0;
  }
  else {
    TCCR1B |= (1<<ICES1);
    TIFR1 |= (1<<ICF1);
    rising = 1;
  }
}

void setup(){
  initTimer();

  Serial.begin(115200);
  Serial.println("Starting");
}

void loop(){
  while(1){
  Serial.println("loop");
  Serial.println(timercapture);
  delay(500);
  }
}

