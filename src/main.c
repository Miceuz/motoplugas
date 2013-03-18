#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include "debounce.h"


#define UP_BUTTON PB0
#define DOWN_BUTTON PB1
#define PROGRAM_BUTTON PC2

#define LED_MID PC0
#define LED_TOP PC1
#define LED_BOT PC4

#define POS_BOT LED_BOT
#define POS_MID LED_MID
#define POS_TOP LED_TOP

#define MODE_TUMBLER PC3
#define MODE_PULL_DOWN PC5

#define HALL_SENSE PD2
#define SPEED_SELECT PD5
#define DOWN_SWITCH PD6
#define UP_SWITCH PD7
#define TRUE 1
#define FALSE 0

#define MODE_PROGRAM 1
#define MODE_RUN 2
#define MODE_MANUAL 3

#define BLINK_SLOW 10
#define BLINK_FAST 5

#define DIRECTION_UP 1
#define DIRECTION_DOWN 2

volatile switch_t progModeTumbler = {0xFF, FALSE, FALSE};
volatile switch_t programButton = {0xFF, FALSE, FALSE};
volatile switch_t upButton = {0xFF, FALSE, FALSE};
volatile switch_t downButton = {0xFF, FALSE, FALSE};

volatile int32_t clicks = 0;
volatile uint8_t mode = 0;
volatile uint8_t block = FALSE;

volatile uint8_t nextPosition = POS_BOT;
volatile uint8_t currPosition = POS_TOP;
volatile uint8_t blinkRate = BLINK_SLOW;
volatile uint8_t blinkCounter = BLINK_SLOW;

volatile uint8_t lastDirection = 0;

uint8_t modePullState = 0;
uint8_t stateOnPullDown = 0x0E;
uint8_t stateOnPullUp = 0x0E;
uint8_t lastTumblerState = 0;

uint8_t middlePositionTimeout = FALSE;
int32_t topThreshold, middleThreshold, bottomThreshold, currThreshold;

static inline void setupGPIO() {
    DDRC |= _BV(LED_MID) | _BV(LED_TOP) | _BV(LED_BOT);
    PORTC &= ~(_BV(LED_MID) | _BV(LED_TOP) | _BV(LED_BOT));
    
    DDRD |= _BV(SPEED_SELECT) | _BV(DOWN_SWITCH) | _BV(UP_SWITCH);
    PORTD &= ~(_BV(SPEED_SELECT) | _BV(DOWN_SWITCH) | _BV(UP_SWITCH));
    
    DDRB &= ~(_BV(UP_BUTTON) | _BV(DOWN_BUTTON));
    PORTB |= _BV(UP_BUTTON) | _BV(DOWN_BUTTON);
    
    DDRC |= _BV(MODE_PULL_DOWN);
    PORTC &= ~_BV(MODE_PULL_DOWN);
    
    DDRC &= ~(_BV(PROGRAM_BUTTON) | _BV(MODE_TUMBLER));
    PORTC |= _BV(PROGRAM_BUTTON) | _BV(MODE_TUMBLER);
    
    modePullState = 1;
    
    DDRD &= ~(_BV(HALL_SENSE));
    PORTD |= _BV(HALL_SENSE);
}

static inline void speedFull() {
    PORTD |= _BV(SPEED_SELECT);
}

static inline void speedSlow() {
    PORTD &= ~_BV(SPEED_SELECT);
}

static inline void ledOn(uint8_t led) {
    PORTC |= _BV(led);
}

static inline void ledOff(uint8_t led) {
    PORTC &= ~_BV(led);
}

static inline void toggleLed(uint8_t led) {
    PORTC ^= _BV(led);
}

static inline void blinkHello() {
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

static inline void allLedsOff() {
    ledOff(LED_TOP);
    ledOff(LED_MID);
    ledOff(LED_BOT);
}

static inline void modePullDown(){
    PORTC &= ~_BV(MODE_TUMBLER);    //turn off internal pull-up
    PORTC |= _BV(MODE_PULL_DOWN);   //turn on external pull-down
    modePullState = 0;
}

static inline void modePullUp(){
    PORTC &= ~_BV(MODE_PULL_DOWN);  //turn off external pull-down
    PORTC |= _BV(MODE_TUMBLER);     //turn on internal pull-up
    modePullState = 1;
}

static inline void closeSwitch(uint8_t sw) {
    PORTD |= _BV(sw);
}

static inline void openSwitch(uint8_t sw) {
    PORTD &= ~_BV(sw);
}

static inline void startBlockTimeout() {
    block=TRUE;
    TIMSK |= _BV(TOIE1);
    TCCR1B |= _BV(CS12) ;
}

static inline void startMiddlePositionTimeout() {
    middlePositionTimeout=TRUE;
    TIMSK |= _BV(TOIE1);
    TCCR1B |= _BV(CS12) | _BV(CS10);
    ledOff(LED_BOT);
}

void stopMiddlePositionTimeout() {
    TCCR1B = 0;
    middlePositionTimeout = FALSE;
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
    ledOff(currPosition);
    currPosition = nextPosition;
    ledOn(currPosition);
    nextPosition = getNextPosition();

    if(POS_BOT == nextPosition) {
        currThreshold = bottomThreshold;
        speedFull();
    } else if(POS_MID == nextPosition) {
        currThreshold = middleThreshold;
        speedSlow();
    } else if(POS_TOP == nextPosition) {
        currThreshold = topThreshold;
        speedFull();
    }
}

static inline uint8_t canGoUp() {
    return MODE_MANUAL == mode ||
    	   MODE_PROGRAM == mode ||
    	   (MODE_RUN == mode && (POS_MID == currPosition || POS_BOT == currPosition));
}

static inline uint8_t isGoingBelowPreviousThreshold() {
    return downButton.pressed &&
    ((POS_MID == nextPosition && clicks <= bottomThreshold) ||
     (POS_TOP == nextPosition && clicks <= middleThreshold));
}

static inline uint8_t canGoDown() {
    return MODE_MANUAL == mode || MODE_PROGRAM == mode || (MODE_RUN == mode && (POS_MID == currPosition || POS_TOP == currPosition));
}

void onUpButtonPressed() {
    if(canGoUp()) {
        lastDirection = DIRECTION_UP;
        closeSwitch(UP_SWITCH);
        blinkRate = BLINK_FAST;
    }
}

void onUpButtonReleased() {
    openSwitch(UP_SWITCH);
    blinkRate = BLINK_SLOW;
}

void onDownButtonPressed() {
    if(canGoDown()) {
        lastDirection = DIRECTION_DOWN;
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
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            currPosition = nextPosition;
            nextPosition = getNextPosition();

            ledOff(currPosition);
            if(POS_BOT == currPosition) {
                bottomThreshold = 0;
                clicks = 0;
            } else if(POS_MID == currPosition) {
                middleThreshold = (clicks > bottomThreshold) ? clicks : bottomThreshold;
                eeprom_write_dword((uint32_t*) 0, middleThreshold);
            } else if(POS_TOP == currPosition) {
            	topThreshold = (clicks > middleThreshold) ? clicks : middleThreshold;
                eeprom_write_dword((uint32_t*) 4, topThreshold);
                block = TRUE;
            }
        }
    }
}

static inline void changeMode(uint8_t newMode) {
    allLedsOff();
    stopMiddlePositionTimeout();
    
    if(MODE_RUN == newMode) {
//        currPosition = POS_TOP;
//        nextPosition = POS_BOT;
        ledOn(currPosition);
        currThreshold = bottomThreshold;
        block = FALSE;
    } else if (MODE_PROGRAM == newMode) {
    	if(POS_TOP == nextPosition) {
    		currPosition = POS_BOT;
    		nextPosition = POS_MID;
    	}
    }
    mode = newMode;
}

static inline void serviceTumbler(volatile switch_t *tumbler) {
    uint8_t newMode = 0;

    if(0xFF == tumbler->pinBuffer || 0x00 == tumbler->pinBuffer) {
        if(0 == modePullState) {
            stateOnPullDown = tumbler->pinBuffer;
            tumbler->pinBuffer=1;//supysam buferį, kad deboucerio interruptas turėtų ką veikt ir nekviestume serviceTmbler, kol neapspęsta nauja būsena
            modePullUp();
        } else if(1 == modePullState) {
            stateOnPullUp = tumbler->pinBuffer;
            tumbler->pinBuffer=1;
            modePullDown();
        }

        if(stateOnPullUp == 0xFF && stateOnPullDown == 0x00) {
            newMode = MODE_RUN;
        } else if(stateOnPullUp == 0x00 && stateOnPullDown == 0x00) {
            newMode = MODE_PROGRAM;
        } else if(stateOnPullUp == 0xFF && stateOnPullDown == 0xFF) {
            newMode = MODE_MANUAL;
        }
        if(mode != newMode) {
            changeMode(newMode);
        }
    }
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

/*
    External interrupt gets executed on magnet pass over the Hall sensor
 */
ISR(INT0_vect) {
    if(DIRECTION_UP == lastDirection) {
        clicks++;
    }
    if(DIRECTION_DOWN == lastDirection) {
        clicks--;
    }
}

/*
    Timer0 overflow interrupt takes care of button debouncing and led blinking
 */
ISR(TIMER0_OVF_vect) {
    debounce(&progModeTumbler, &PINC, MODE_TUMBLER);

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

int main (void) {
    setupGPIO();
    blinkHello();
    
    TIMSK |= _BV(TOIE0);//timer0 overflow interrupt enable
    TCCR0 |= _BV(CS02) | _BV(CS00);  // clk/1024

    MCUCR |= _BV(ISC01); //falling edge
    GICR |= _BV(INT0); //int0 external interrupt enable

    middleThreshold = (int32_t) eeprom_read_dword((uint32_t*) 0);
    topThreshold = (int32_t) eeprom_read_dword((uint32_t*) 4);
    clicks = topThreshold;
    speedFull();
    
    sei();
    ledOn(currPosition);
    
    while(1){
        serviceTumbler(&progModeTumbler);
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
                        stopMiddlePositionTimeout();
                        if((POS_MID == nextPosition && clicks >= middleThreshold) || (POS_TOP == nextPosition && clicks >= topThreshold)) {
                            startBlockTimeout();
                            blinkRate = BLINK_SLOW;
                            openSwitch(UP_SWITCH);
                            setUpNextPosition();
                        }
                    }
                } else if(downButton.pressed) {
                    ATOMIC_BLOCK(ATOMIC_FORCEON) {
                        stopMiddlePositionTimeout();
                        if(POS_BOT == nextPosition && clicks <= bottomThreshold) {
                            startBlockTimeout();
                            blinkRate = BLINK_SLOW;
                            openSwitch(DOWN_SWITCH);
                            setUpNextPosition();
                        }
                    }
                } else if(POS_MID == nextPosition && clicks >= middleThreshold - 10) {
                    startMiddlePositionTimeout();
                }
            } else if(MODE_PROGRAM == mode) {
                if(isGoingBelowPreviousThreshold()) {
                    openSwitch(UP_SWITCH);
                    openSwitch(DOWN_SWITCH);
                }
            }
        } else {
            openSwitch(UP_SWITCH);
            openSwitch(DOWN_SWITCH);
        }
    }
}
