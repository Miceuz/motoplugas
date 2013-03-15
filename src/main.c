#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

#define UP_BUTTON PB0
#define DOWN_BUTTON PB1

#define LED_MID PC0
#define LED_TOP PC1
#define PROGRAM_BUTTON PC2
#define MODE_SENSE PC3
#define LED_BOT PC4

#define POS_BOT LED_BOT
#define POS_MID LED_MID
#define POS_TOP LED_TOP

#define HALL_SENSE PD2
#define SPEED_SELECT PD5
#define DOWN_SWITCH PD6
#define UP_SWITCH PD7
#define TRUE 1
#define FALSE 0

#define MODE_PROGRAM 1
#define MODE_RUN 2

#define BLINK_SLOW 10
#define BLINK_FAST 5


typedef struct {
    uint8_t pinBuffer;
    uint8_t pressed;
    uint8_t lastState;
} button_t;

volatile button_t progModeButton = {0xFF, FALSE, FALSE};
volatile button_t programButton = {0xFF, FALSE, FALSE};
volatile button_t upButton = {0xFF, FALSE, FALSE};
volatile button_t downButton = {0xFF, FALSE, FALSE};

volatile int32_t clicks = 0;
volatile uint8_t mode = MODE_RUN;
volatile uint8_t block = FALSE;

volatile uint8_t nextPosition = POS_BOT;
volatile uint8_t currPosition = POS_TOP;
volatile uint8_t blinkRate = BLINK_SLOW;
volatile uint8_t blinkCounter = BLINK_SLOW;

inline void setupGPIO() {
    DDRC |= _BV(LED_MID) | _BV(LED_TOP) | _BV(LED_BOT);
    PORTC &= ~(_BV(LED_MID) | _BV(LED_TOP) | _BV(LED_BOT));
    
    DDRD |= _BV(SPEED_SELECT) | _BV(DOWN_SWITCH) | _BV(UP_SWITCH);
    PORTD &= ~(_BV(SPEED_SELECT) | _BV(DOWN_SWITCH) | _BV(UP_SWITCH));
    
    DDRB &= ~(_BV(UP_BUTTON) | _BV(DOWN_BUTTON));
    PORTB |= _BV(UP_BUTTON) | _BV(DOWN_BUTTON);
    
    DDRC &= ~(_BV(PROGRAM_BUTTON) | _BV(MODE_SENSE));
    PORTC |= _BV(PROGRAM_BUTTON) | _BV(MODE_SENSE);
    
    DDRD &= ~(_BV(HALL_SENSE));
    PORTD |= _BV(HALL_SENSE);
}

inline void ledOn(uint8_t led) {
    PORTC |= _BV(led);
}

inline void ledOff(uint8_t led) {
    PORTC &= ~_BV(led);
}

inline void toggleLed(uint8_t led) {
    PORTC ^= _BV(led);
}

inline void allLedsOff() {
    ledOff(LED_TOP);
    ledOff(LED_MID);
    ledOff(LED_BOT);
}

inline void closeSwitch(uint8_t sw) {
    PORTD |= _BV(sw);
}

inline void openSwitch(uint8_t sw) {
    PORTD &= ~_BV(sw);
}

inline void startBlockTimeout() {
    block=TRUE;
    TIMSK |= _BV(TOIE1);
    TCCR1B |= _BV(CS12) | _BV(CS10);
}


uint8_t middlePositionTimeout = FALSE;
int32_t topThreshold, middleThreshold, bottomThreshold, currThreshold;

inline void startMiddlePositionTimeout() {
    middlePositionTimeout=TRUE;
    TIMSK |= _BV(TOIE1);
    TCCR1B |= _BV(CS12) | _BV(CS10);
    ledOff(LED_BOT);
}

inline void stopMiddlePositionTimeout() {
    TCCR1B = 0;
}

uint8_t getNextPosition() {
    if(POS_BOT == currPosition) {
        return POS_MID;
    } else if(POS_MID == currPosition) {
        return POS_TOP;
    } else if(POS_TOP == currPosition) {
        return POS_BOT;
    }
}

void setUpNextPosition() {
    if(POS_BOT == nextPosition) {
        currThreshold = middleThreshold;
    } else if(POS_MID == nextPosition) {
        currThreshold = topThreshold;
    } else if(POS_TOP == nextPosition) {
        currThreshold = bottomThreshold;
    }
    ledOff(currPosition);
    currPosition = nextPosition;
    ledOn(currPosition);
    nextPosition = getNextPosition();
}

/*
    Timer1 overflow interrupt releases buttons to user
 */
ISR(TIMER1_OVF_vect) {
    if(block) {
        block=FALSE;
    }
    if(middlePositionTimeout) {
        middlePositionTimeout = FALSE;
        setUpNextPosition();
    }
    TCCR1B = 0;
}


void debounce(volatile button_t *button, volatile uint8_t *port, volatile uint8_t pin){
    button->pinBuffer = (button->pinBuffer) << 1;
    button->pinBuffer |= ((*(port) & _BV(pin)) != 0);
    if(0 == button->pinBuffer) {
        button->pressed = TRUE;
    } else if(0xFF == button->pinBuffer){
        button->pressed = FALSE;
    }
}

/*
    Timer0 overflow interrupt takes care of button debouncing and led blinking
 */
ISR(TIMER0_OVF_vect) {
//    sei(); //allow Hall sensor interrupts inside as they are rather important
    debounce(&progModeButton, &PINC, MODE_SENSE);
        
    if(!downButton.pressed) {
        debounce(&upButton, &PINB, UP_BUTTON);
    }

    if(!upButton.pressed) {
        debounce(&downButton, &PINB, DOWN_BUTTON);
    }
        
    if(!upButton.pressed && !downButton.pressed) {
        debounce(&programButton, &PINC, PROGRAM_BUTTON);
    }
    
    if((MODE_PROGRAM == mode || MODE_RUN == mode) && !block) {
        if(0 == blinkCounter--) {
            toggleLed(nextPosition);
            blinkCounter = blinkRate;
        }
    }            

}


/*
    External interrupt gets executed on magnet pass over the Hall sensor
 */
ISR(INT0_vect) {
    if(upButton.pressed) {
        clicks++;
    }
    if(downButton.pressed) {
        clicks--;
    }
}

inline void blinkHello() {
    ledOn(LED_TOP);
    _delay_ms(200);
    ledOff(LED_TOP);
    ledOn(LED_MID);
    _delay_ms(200);
    ledOff(LED_MID);
    ledOn(LED_BOT);
    _delay_ms(200);
    ledOff(LED_BOT);
    
}

inline uint8_t canGoUp() {
    return MODE_PROGRAM == mode || (MODE_RUN == mode && (POS_MID == currPosition || POS_BOT == currPosition));
}

void onUpButtonPressed() {
    if(canGoUp()) {
        closeSwitch(UP_SWITCH);
        blinkRate = BLINK_FAST;
    }
}

void onUpButtonReleased() {
    openSwitch(UP_SWITCH);
    blinkRate = BLINK_SLOW;
}

inline uint8_t canGoDown() {
    return MODE_PROGRAM == mode || (MODE_RUN == mode && (POS_MID == currPosition || POS_TOP == currPosition));
}

void onDownButtonPressed() {
    if(canGoDown()) {
        closeSwitch(DOWN_SWITCH);
        blinkRate = BLINK_FAST;
    }
}

void onDownButtonReleased() {
    openSwitch(DOWN_SWITCH);
    blinkRate = BLINK_SLOW;
}

void onProgramButtonPressed() {
    if(MODE_PROGRAM == mode) {
        ATOMIC_BLOCK(ATOMIC_FORCEON)
        {
            ledOff(nextPosition);
            if(POS_BOT == nextPosition) {
                bottomThreshold = 0;
                clicks = 0;
            } else if(POS_MID == nextPosition) {
                middleThreshold = clicks;
            } else if(POS_TOP == nextPosition) {
                topThreshold = clicks;
                block = TRUE;
            }
            currPosition = nextPosition;
            nextPosition = getNextPosition();
        }
    }
}

void onProgModePressed() {
    allLedsOff();
    mode = MODE_PROGRAM;
}

void onProgModeReleased() {
    ledOff(nextPosition);
    currPosition = POS_TOP;
    nextPosition = POS_BOT;
    ledOn(currPosition);
    currThreshold = bottomThreshold;
    mode = MODE_RUN;
    block = FALSE;
}

void serviceButton(volatile button_t *button, void (*onPressed)(void), void (*onReleased)(void)) {
    if(button->pressed != button->lastState) {
        if(button->pressed) {
            if(0 != onPressed) {
                onPressed();
            }
        } else {
            if(0 != onReleased) {
                onReleased();
            }
        }
        button->lastState = button->pressed;
    }
}

int main (void) {
    setupGPIO();
    blinkHello();
    
    TIMSK |= _BV(TOIE0);//timer0 overflow interrupt enable
    TCCR0 |= _BV(CS02) | _BV(CS00);  // clk/1024

    MCUCR |= _BV(ISC01); //falling edge
    GICR |= _BV(INT0); //int0 external interrupt enable

    sei();
    ledOn(currPosition);
    
    while(1){
        serviceButton(&progModeButton, onProgModePressed, onProgModeReleased);
        if(MODE_PROGRAM == mode) {
            serviceButton(&programButton, onProgramButtonPressed, 0);
        }

        if(!block) {
            serviceButton(&upButton, onUpButtonPressed, onUpButtonReleased);
            serviceButton(&downButton, onDownButtonPressed, onDownButtonReleased);
            
            if(MODE_RUN == mode) {
                if(upButton.pressed) {
                    ATOMIC_BLOCK(ATOMIC_FORCEON) 
                    {
                        if((POS_MID == nextPosition && clicks >= middleThreshold) || (POS_TOP == nextPosition && clicks >= topThreshold)) {
                            startBlockTimeout();
                            setUpNextPosition();
                        } else {
                            stopMiddlePositionTimeout();
                        }
                    }
                } else if(downButton.pressed) {
                    ATOMIC_BLOCK(ATOMIC_FORCEON) {
                        if(POS_BOT == nextPosition && clicks <= bottomThreshold) {
                            startBlockTimeout();
                            setUpNextPosition();
                        } else {
                            stopMiddlePositionTimeout();
                        }
                    }
                } else if(POS_MID == nextPosition && clicks >= middleThreshold - 10) {
                    startMiddlePositionTimeout();
                }
            }
        } else {
            openSwitch(UP_SWITCH);
            openSwitch(DOWN_SWITCH);
        }
    }
}
