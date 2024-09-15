#define SELFSUPPLY_PIN PB0

#define DEBUGLED PB5
#define MAX_STANDBY_TIME 60000
//-------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------------
typedef enum {
  OFF,
  ON
} State_t;
//--------------------------------------------------------------------------------------------------------------------------
//  modes
typedef enum {
  DAY_MODE,
  NIGHT_MODE
} DayTime_t;

DayTime_t dayTime = DAY_MODE;
//--------------------------------------------------------------------------------------------------------------------------
// Blink modes
typedef enum {
  BLINKER_OFF,
  BLINKER_STANDBY,
  BLINKER_ON
} BlinkerStates_t;

BlinkerStates_t blinkerState = BLINKER_STANDBY;
//--------------------------------------------------------------------------------------------------------------------------
// Blink modes
typedef enum {
  BLINK_IDLE,
  BLINK_DAY_MODE,
  BLINK_NIGHT_MODE,
  TURN_LEFT,
  TURN_RIGHT,
  RANOM_STATE
} BlinkModes_t;

BlinkModes_t blinkingMode = BLINK_IDLE;
BlinkModes_t prevBlinkMode = BLINK_IDLE;
//--------------------------------------------------------------------------------------------------------------------------
#define BUTTON_L_PIN PD2  // Button connected to PD2 (INT0)
#define BUTTON_R_PIN PD4  // Button connected to PD4

#define DEBOUNCE_TIME 50         // Debounce time in milliseconds
#define HOLD_TIME 2000           // Time to detect button hold in milliseconds
#define CONSEC_PRESSES_TIME 500  // Time window for counting consecutive presses

// Button states
typedef enum {
  BUTTON_IDLE,
  BUTTON_DEBOUNCING,
  BUTTON_PRESSED,
  BUTTON_HOLD,
  BUTTON_COUNT_CONSEC_PRESSES  // State to track consecutive presses
} ButtonState_t;

// Button structure to hold state information for each button
typedef struct {
  uint8_t pin;                     // The pin where the button is connected
  ButtonState_t state;             // Current state of the button
  uint16_t debounceCounter;        // Debounce counter
  uint16_t pressDuration;          // How long the button has been pressed
  uint8_t pressCount;              // Number of consecutive presses
  uint16_t consecutivePressTimer;  // Timer for consecutive presses
  volatile uint8_t updateButtonEvent;
} Button;

// Two buttons
Button buttonR = { BUTTON_R_PIN, BUTTON_IDLE, 0, 0, 0, 0, 0 };
Button buttonL = { BUTTON_L_PIN, BUTTON_IDLE, 0, 0, 0, 0, 0 };
//--------------------------------------------------------------------------------------------------------------------------
// static volatile uint32_t millisTime = 0;

static volatile uint32_t computeBlinkingTime = 0;
static volatile uint8_t blinkPathStep = 0;
uint8_t setLedsDoneFlag = 0;
static volatile uint32_t standByTime = 0;
//--------------------------------------------------------------------------------------------------------------------------
#define MAX_blinkPathStepS 10  // Define the maximum number of blinkPathSteps in a blinking sequence

// Structure to define a blinking mode for all four channels
typedef struct {
  uint8_t leftRedPath[MAX_blinkPathStepS];      // Blinking pattern for left red light
  uint8_t leftYellowPath[MAX_blinkPathStepS];   // Blinking pattern for left yellow light
  uint8_t rightRedPath[MAX_blinkPathStepS];     // Blinking pattern for right red light
  uint8_t rightYellowPath[MAX_blinkPathStepS];  // Blinking pattern for right yellow light

  uint32_t delayBetweenBlinks[MAX_blinkPathStepS];  // Delay between each intensity change
  uint8_t endOfVector;                              // Position in vectors where blinking Path end
  uint8_t pointToLoop;                              // Position in vectors where the loop starts from
} BlinkMode;
//-------------------------------------------------------------
BlinkMode dayBlinkMode = {
  .leftRedPath = { 200, 0, 200, 0, 200, 0, 200, 0, 200, 0 },  // intensities
  .leftYellowPath = { 0 },
  .rightRedPath = { 200, 0, 200, 0, 200, 0, 200, 0, 200, 0 },
  .rightYellowPath = { 0 },
  .delayBetweenBlinks = { 15, 90, 15, 90, 15, 90, 15, 550, 90, 150 },  // delays
  .endOfVector = 9,                                                    // value indicating end of the pattern
  .pointToLoop = 2                                                     // value indicating loop start point
};
//-------------------------------------------------------------
BlinkMode nightBlinkMode = {
  .leftRedPath = { 180, 50, 180, 50, 180, 50, 180, 50, 180, 50 },
  .leftYellowPath = { 0 },
  .rightRedPath = { 180, 50, 180, 50, 180, 50, 180, 50, 180, 50 },
  .rightYellowPath = { 0 },
  .delayBetweenBlinks = { 15, 90, 15, 90, 15, 90, 15, 550, 90, 150 },
  .endOfVector = 9,
  .pointToLoop = 0
};
//-------------------------------------------------------------
BlinkMode turnLeftBlink = {
  .leftRedPath = { 0 },  // Example intensities
  .leftYellowPath = { 255, 0, 255, 0, 200, 0 },
  .rightRedPath = { 255, 200, 200, 200, 200, 200 },
  .rightYellowPath = { 0 },
  .delayBetweenBlinks = { 250, 200, 350, 300, 450, 350 },
  .endOfVector = 5,  // Example value, indicating end of the pattern
  .pointToLoop = 4   // Example value, indicating loop start point
};
//-------------------------------------------------------------
BlinkMode turnRightBlink = {
  .leftRedPath = { 255, 200, 200, 200, 200, 200 },  // Example intensities
  .leftYellowPath = { 0 },
  .rightRedPath = { 0 },
  .rightYellowPath = { 255, 0, 255, 0, 200, 0 },
  .delayBetweenBlinks = { 250, 200, 350, 300, 450, 350 },
  .endOfVector = 5,  // Example value, indicating end of the pattern
  .pointToLoop = 4   // Example value, indicating loop start point
};
//-------------------------------------------------------------
BlinkMode blinkIdle = {
  .leftRedPath = { 2 },  // Example intensities
  .leftYellowPath = { 0 },
  .rightRedPath = { 0 },
  .rightYellowPath = { 0 },
  .delayBetweenBlinks = { 100 },
  .endOfVector = 0,  // Example value, indicating end of the pattern
  .pointToLoop = 0   // Example value, indicating loop start point
};
//-------------------------------------------------------------
BlinkMode errorBlink = {
  .leftRedPath = { 200, 0 },  // Example intensities
  .leftYellowPath = { 0 },
  .rightRedPath = { 0 },
  .rightYellowPath = { 0 },
  .delayBetweenBlinks = { 400, 400 },
  .endOfVector = 1,  // Example value, indicating end of the pattern
  .pointToLoop = 0   // Example value, indicating loop start point
};
//--------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------------
int main() {
  initSelfSupply();  // Initialize self-supply
  initTimer0PWM();
  initTimer1();
  initTimer2PWM();
  btnInit();
  DDRB |= (1 << DEBUGLED);

  while (1) {
    //useless in this project
  }
}
//--------------------------------------------------------------------------------------------------------------------------
void computeBtnGestsTick() {
  // debugHi();
  standByTime++;
  if (blinkerState == BLINKER_STANDBY && standByTime > MAX_STANDBY_TIME) {
    blinkerState = BLINKER_OFF;
    standByTime = 0;
  }
  if ((buttonL.state == BUTTON_IDLE || buttonL.state == BUTTON_HOLD) && (buttonR.state == BUTTON_IDLE || buttonR.state == BUTTON_HOLD) && (buttonL.updateButtonEvent || buttonR.updateButtonEvent)) {  //if butttons was actioned
    // debugTog();
    // Handle button logic
    if (blinkerState == BLINKER_STANDBY) {
      if ((buttonL.state == BUTTON_HOLD && buttonR.state == BUTTON_HOLD) || (buttonL.state == BUTTON_IDLE && buttonL.pressCount >= 4) || (buttonR.state == BUTTON_IDLE && buttonR.pressCount >= 4)) {
        blinkerState = BLINKER_ON;
        blinkingMode = BLINK_DAY_MODE;
        standByTime = 0;
        blinkPathStep = 0;       // Reset blinking step
        buttonL.pressCount = 0;  // Reset press count after handling
        buttonR.pressCount = 0;  // Reset press count after handling
      }
    } else if (blinkerState == BLINKER_ON) {
      if ((buttonL.state == BUTTON_HOLD && buttonR.state == BUTTON_HOLD) || (buttonL.state == BUTTON_IDLE && buttonL.pressCount >= 4) || (buttonR.state == BUTTON_IDLE && buttonR.pressCount >= 4)) {
        blinkerState = BLINKER_OFF;
        blinkPathStep = 0;       // Reset blinking step
        buttonL.pressCount = 0;  // Reset press count after handling
        buttonR.pressCount = 0;  // Reset press count after handling
      } else if ((buttonL.state == BUTTON_IDLE && buttonL.pressCount == 3) || (buttonR.state == BUTTON_IDLE && buttonR.pressCount == 3)) {
        if (dayTime == DAY_MODE) {
          dayTime = NIGHT_MODE;
          blinkingMode = BLINK_NIGHT_MODE;
        } else if (dayTime == NIGHT_MODE) {
          dayTime = DAY_MODE;
          blinkingMode = BLINK_DAY_MODE;
        }
        blinkPathStep = 0;       // Reset blinking step
        buttonL.pressCount = 0;  // Reset press count after handling
        buttonR.pressCount = 0;  // Reset press count after handling
      } else if (buttonL.state == BUTTON_IDLE && buttonL.pressCount == 1) {
        if (blinkingMode == TURN_LEFT) {
          if (dayTime == DAY_MODE) {
            blinkingMode = BLINK_DAY_MODE;
          } else if (dayTime == NIGHT_MODE) {
            blinkingMode = BLINK_NIGHT_MODE;
          }
        } else {
          blinkingMode = TURN_LEFT;  // Set turn left mode after 1 press
        }
        blinkPathStep = 0;
        buttonL.pressCount = 0;  // Reset press count after handling
        buttonR.pressCount = 0;  // Reset press count after handling
      }
      if (buttonR.state == BUTTON_IDLE && buttonR.pressCount == 1) {
        if (blinkingMode == TURN_RIGHT) {
          if (dayTime == DAY_MODE) {
            blinkingMode = BLINK_DAY_MODE;
          } else if (dayTime == NIGHT_MODE) {
            blinkingMode = BLINK_NIGHT_MODE;
          }
        } else {
          blinkingMode = TURN_RIGHT;  // Set turn right mode after 1 press
        }
        blinkPathStep = 0;
        buttonL.pressCount = 0;  // Reset press count after handling
        buttonR.pressCount = 0;  // Reset press count after handling
      }
    }
    buttonL.updateButtonEvent = 0;
    buttonR.updateButtonEvent = 0;
  }
  //_NOP();  // Insert no operation instruction
}
//--------------------------------------------------------------------------------------------------------------------------
void initSelfSupply() {
  DDRB |= (1 << SELFSUPPLY_PIN);   // Set SELFSUPPLY_PIN as output
  PORTB |= (1 << SELFSUPPLY_PIN);  // Set pin high
  blinkerState = BLINKER_STANDBY;
}
void selfSupply(State_t state) {
  if (state == ON) {
    PORTB |= (1 << SELFSUPPLY_PIN);  // Set pin high
  } else {
    PORTB &= ~(1 << SELFSUPPLY_PIN);  // Set pin low
  }
}
//--------------------------------------------------------------------------------------------------------------------------
// tick function to be called every 1ms
void blinkerTick() {
  switch (blinkerState) {
    case BLINKER_ON:
      selfSupply(ON);  // Turn on self-supply
      // If the blinking state has changed, reset the blinking step and timing
      if (prevBlinkMode != blinkingMode) {
        blinkPathStep = 0;        // Reset step when mode changes
        setLedsDoneFlag = 0;      // Reset the done flag
        computeBlinkingTime = 0;  // Reset the timing
      }
      prevBlinkMode = blinkingMode;  // Update the previous mode
      // Call appropriate blinking function based on the current state
      switch (blinkingMode) {
        case BLINK_IDLE:
          computeBlinking(&blinkIdle);
          break;
        case BLINK_DAY_MODE:
          // Fallthrough intentional
          __attribute__((fallthrough));
        case BLINK_NIGHT_MODE:
          if (dayTime == DAY_MODE) {
            computeBlinking(&dayBlinkMode);
          } else if (dayTime == NIGHT_MODE) {
            computeBlinking(&nightBlinkMode);
          }
          break;
        // case BLINK_NIGHT_MODE:
        //   break;
        case TURN_LEFT:
          computeBlinking(&turnLeftBlink);
          break;
        case TURN_RIGHT:
          computeBlinking(&turnRightBlink);
          break;
        default:
          computeBlinking(&errorBlink);
          break;
      }
      break;

    case BLINKER_STANDBY:
      selfSupply(ON);  // Keep on self-supply in standby
      blinkingMode = BLINK_IDLE;
      computeBlinking(&blinkIdle);
      break;

    case BLINKER_OFF:
      selfSupply(OFF);  // Turn off self-supply
      computeBlinking(&errorBlink);
      break;
  }
}

void computeBlinking(BlinkMode* blinkMode) {
  computeBlinkingTime++;  // Increment the timing counter

  // Execute blink modes based on the current state
  if (!setLedsDoneFlag) {
    // Set the output signals based on the current step in the pattern
    setRightRed(blinkMode->rightRedPath[blinkPathStep]);
    setRightYellow(blinkMode->rightYellowPath[blinkPathStep]);
    setLeftRed(blinkMode->leftRedPath[blinkPathStep]);
    setLeftYellow(blinkMode->leftYellowPath[blinkPathStep]);

    setLedsDoneFlag = 1;  // Mark as done for this step
  }

  // Check if it's time to move to the next step
  if (computeBlinkingTime >= blinkMode->delayBetweenBlinks[blinkPathStep]) {
    // Move to the next step in the pattern
    if (blinkPathStep >= blinkMode->endOfVector) {
      blinkPathStep = blinkMode->pointToLoop;  // Reset to the loop start point
    } else {
      blinkPathStep++;  // Move to the next step
    }

    setLedsDoneFlag = 0;      // Reset the flag for the next step
    computeBlinkingTime = 0;  // Reset the timing counter
  }
}
//--------------------------------------------------------------------------------------------------------------------------
// Function to initialize button pin as input with pull-up resistor
void btnInit() {
  DDRD &= ~(1 << BUTTON_R_PIN) & ~(1 << BUTTON_L_PIN);  // Set BUTTON_R_PIN and BUTTON_L_PIN as input
  PORTD |= (1 << BUTTON_R_PIN) | (1 << BUTTON_L_PIN);   // Enable internal pull-up resistors
}
//-------------------------------------------------------------
// Function to check the state of a button
uint8_t isButtonPressed(Button* btn) {
  return !(PIND & (1 << btn->pin));  // Button is pressed if LOW
}
//-------------------------------------------------------------
// Function to update the state of a button (called every 1ms)
void updateButtonState(Button* btn) {
  // uint8_t isButtonPressed = !(PIND & (1 << btn->pin));  // Button is pressed if LOW
  btn->consecutivePressTimer++;

  switch (btn->state) {
    case BUTTON_IDLE:
      if (isButtonPressed(btn)) {
        btn->state = BUTTON_DEBOUNCING;
        btn->debounceCounter = 0;
      }
      break;

    case BUTTON_DEBOUNCING:
      if (btn->debounceCounter < DEBOUNCE_TIME) {
        btn->debounceCounter++;
      } else {
        if (isButtonPressed(btn)) {
          btn->updateButtonEvent = 1;
          btn->state = BUTTON_PRESSED;
          btn->pressDuration = 0;
        } else {
          btn->state = BUTTON_IDLE;
        }
      }
      break;

    case BUTTON_PRESSED:
      btn->pressDuration++;
      if (!isButtonPressed(btn)) {
        btn->pressCount++;
        btn->state = BUTTON_COUNT_CONSEC_PRESSES;
        btn->consecutivePressTimer = 0;  // Start tracking consecutive presses
      } else if (btn->pressDuration >= HOLD_TIME) {
        btn->state = BUTTON_HOLD;
      }
      if (btn->consecutivePressTimer >= CONSEC_PRESSES_TIME) {
        btn->pressCount = 0;  // Reset press count if time exceeded
      }
      break;

    case BUTTON_HOLD:
      if (!isButtonPressed(btn)) {
        btn->state = BUTTON_IDLE;
      }
      break;

    case BUTTON_COUNT_CONSEC_PRESSES:
      // btn->consecutivePressTimer++;
      if (btn->consecutivePressTimer >= CONSEC_PRESSES_TIME) {
        btn->state = BUTTON_IDLE;
        // btn->pressCount = 0;  // Reset press count if time exceeded
      } else if (isButtonPressed(btn)) {
        btn->state = BUTTON_DEBOUNCING;
        btn->debounceCounter = 0;  // Prepare for a new press
      }
      break;
  }
}

// Function to be called every 1ms (from a timer interrupt or main loop)
void btnTick() {
  updateButtonState(&buttonR);
  updateButtonState(&buttonL);
}
//--------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------------
// Initialize Timer0 and Timer2 for PWM
void initTimer0PWM() {
  // Set PD6 (OC0A), PD5 (OC0B) as output
  DDRD |= (1 << PD6) | (1 << PD5);

  // Timer 0 Configuration for Fast PWM on OC0A and OC0B
  // Fast PWM mode, non-inverting output on OC0A and OC0B, prescaler 64
  TCCR0A |= (1 << WGM00) | (1 << WGM01);    // Fast PWM mode
  TCCR0A |= (1 << COM0A1) | (1 << COM0B1);  // Non-inverting mode on OC0A and OC0B
  TCCR0B |= (1 << CS02);                    // Prescaler = 256
  // TCCR0B |= (1 << CS01) | (1 << CS00);      // Prescaler = 64
  // TCCR0B |= (1 << CS02) | (1 << CS00);      // Prescaler = 1024

  // Initialize duty cycle to 0
  OCR0A = 0;  // PWM duty cycle for OC0A (PD6)
  OCR0B = 0;  // PWM duty cycle for OC0B (PD5)
}
void initTimer2PWM() {
  // Set PB3 (OC2A) and PD3 (OC2B) as output
  DDRD |= (1 << PD3);
  DDRB |= (1 << PB3);

  // Timer 2 Configuration for Fast PWM on OC2A and OC2B
  // Fast PWM mode, non-inverting output on OC2A and OC2B, prescaler 64
  TCCR2A |= (1 << WGM20) | (1 << WGM21);    // Fast PWM mode
  TCCR2A |= (1 << COM2A1) | (1 << COM2B1);  // Non-inverting mode on OC2A and OC2B
  TCCR2B |= (1 << CS22) | (1 << CS21);      // Prescaler = 256
  // TCCR2B |= (1 << CS22);                    // Prescaler = 64
  // TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);  // Prescaler = 1024

  // Initialize duty cycle to 0
  OCR2A = 0;  // PWM duty cycle for OC2A (PB3)
  OCR2B = 0;  // PWM duty cycle for OC2B (PD3)
}
//-------------------------------------------------------------
// Set PWM duty cycle for each channel (0-255)
void setRightRed(uint8_t dutyOCR0A) {
  if (dutyOCR0A == 0) {
    PORTD &= ~(1 << PD6);      // Set pin low
    TCCR0A &= ~(1 << COM0A1);  // Disconnect PWM for PD6
  } else {
    TCCR0A |= (1 << COM0A1);  // Connect PWM for PD6
    OCR0A = dutyOCR0A;        // Set duty cycle for OCR0A (PD6)
  }
}

void setRightYellow(uint8_t dutyOCR0B) {
  if (dutyOCR0B == 0) {
    PORTD &= ~(1 << PD5);      // Set pin low
    TCCR0A &= ~(1 << COM0B1);  // Disconnect PWM for PD5
  } else {
    TCCR0A |= (1 << COM0B1);  // Connect PWM for PD5
    OCR0B = dutyOCR0B;        // Set duty cycle for OCR0B (PD5)
  }
}

void setLeftRed(uint8_t dutyOCR2A) {
  if (dutyOCR2A == 0) {
    PORTB &= ~(1 << PB3);      // Set pin low
    TCCR2A &= ~(1 << COM2A1);  // Disconnect PWM for PB3
  } else {
    TCCR2A |= (1 << COM2A1);  // Connect PWM for PB3
    OCR2A = dutyOCR2A;        // Set duty cycle for OCR2A (PB3)
  }
}

void setLeftYellow(uint8_t dutyOCR2B) {
  if (dutyOCR2B == 0) {
    PORTD &= ~(1 << PD3);      // Set pin low
    TCCR2A &= ~(1 << COM2B1);  // Disconnect PWM for PD3
  } else {
    TCCR2A |= (1 << COM2B1);  // Connect PWM for PD3
    OCR2B = dutyOCR2B;        // Set duty cycle for OCR2B (PD3)
  }
}
//--------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------------
void initTimer1() {
  // Ensure Timer1 is in a known state
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  // Set Timer1 to CTC mode
  TCCR1B |= (1 << WGM12);
  // Set prescaler to 64 (16 MHz / 64 = 250 kHz)
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Calculate and set OCR1A for 1 ms interrupt
  OCR1A = 249;
  // Enable Timer1 Compare Match A interrupt
  TIMSK1 |= (1 << OCIE1A);
  // Enable global interrupts
  sei();
}
//-------------------------------------------------------------
// Timer1 Compare Match A Interrupt Service Routine
ISR(TIMER1_COMPA_vect) {
  // millisTime++;
  debugHi();
  btnTick();
  blinkerTick();
  computeBtnGestsTick();
  debugLo();
}
//--------------------------------------------------------------------------------------------------------------------------

void debugHi() {
  PORTB |= (1 << DEBUGLED);  // Set DEBUGLED pin high
}

void debugLo() {
  PORTB &= ~(1 << DEBUGLED);  // Set DEBUGLED pin low
}

void debugTog() {
  PORTB ^= (1 << DEBUGLED);  // Toggle DEBUGLED pin
}