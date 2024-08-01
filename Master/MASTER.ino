#include "avdweb_VirtualDelay.h"

#define RC_ANODE_PIN   2
#define RC_CATHODE_PIN 7
#define INTERNAL_LED_PIN  13
#define SWITCH_PIN  10
#define DEMODULATOR_OUTPUT_PIN 3  

#define SWITCH_DEB_TIME_TICK  50
#define SWITCH_OPEN   1
#define SWITCH_CLOSED 0

#define HALF_BIT_TIME_us    8000
#define INTERFAME_TIME_us 200000

#define CMD_SET_LED_ON       0
#define CMD_SET_LED_OFF   0xFF
#define CMD_SET_LED_BLINK 0xAA

#define SAMPLES_TO_SKIP 8

#define ACK   0x55AA
#define NACK  0x01FE

int txHalfBitCnt;
int txBit;
int switchPinSts;
int switchDebouncer;
int switchSts;
int switchStatusChangedEvent = 0;
byte cmdToSend; 
unsigned int  currentlyTransmittingCmd;
bool startTx;
int blinkStatus;

VirtualDelay delay_us(micros);
VirtualDelay rx_timeout_us(micros);

typedef enum
{
  RECEIVING     = 0,
  TRANSMITTING  = 1,
  
  COMMUNICATION_STATUS_SPACE_WIDTH
} COMMUNICATION_STATUS_t;

COMMUNICATION_STATUS_t COMMUNICATION_STATUS;

typedef enum
{
  WAIT_FOR_IDLE                   = 0,
  WAIT_FOR_FALLING_EDGE           = 1,
  WAIT_FOR_START_BIT              = 2,
  WAIT_FIRST_HALF_BIT_OF_PAYLOAD  = 3,
  WAIT_SECOND_HALF_BIT_OF_PAYLOAD = 4,
  CONFIRMING_ZERO                 = 5,
  CONFIRMING_ONE                  = 6,
  WAIT_FIRST_HALF_STOP_BIT        = 7,
  WAIT_SECOND_HALF_STOP_BIT       = 8,

  DECODER_FSM_STS_SPACE_WIDTH
} DECODER_FSM_STS_t;

DECODER_FSM_STS_t DECODER_FSM_STS;

byte rxBitCnt;
byte samplesToSkip;
unsigned int rxFrame;
byte decodedReply;

bool replyReceived;

void setup() {
  pinMode(RC_ANODE_PIN, OUTPUT);
  pinMode(RC_CATHODE_PIN, OUTPUT);

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  pinMode(INTERNAL_LED_PIN, OUTPUT);

  pinMode(DEMODULATOR_OUTPUT_PIN, OUTPUT);
  
  digitalWrite(RC_ANODE_PIN, LOW); 
  digitalWrite(RC_CATHODE_PIN, LOW);

  digitalWrite(INTERNAL_LED_PIN, HIGH);

  switchDebouncer = SWITCH_DEB_TIME_TICK;
  switchSts = SWITCH_OPEN;

  txHalfBitCnt = 0;
  cmdToSend = CMD_SET_LED_ON;

  delay(100);

  COMMUNICATION_STATUS = TRANSMITTING;
  startTx = true;
}

void debounceSwitch(void)
{
  switchPinSts = digitalRead(SWITCH_PIN);
  if(switchPinSts != switchSts)
  {
    if(switchDebouncer)
    {
      switchDebouncer--;
      if(!switchDebouncer)
      {
        switchSts ^= 1; // Toggle status  
        switchStatusChangedEvent = 1; 
      }  
    }   
  }
  else
  {
    switchDebouncer = SWITCH_DEB_TIME_TICK;  
    switchStatusChangedEvent = 0;
  }
}

typedef enum
{
  FORWARD_BIASING = 0,
  REVERSE_BIASING = 1,
  REVERSE_CAPACITANCE_DISCHARGING  = 2,
  DIGITAL_SAMPLING = 3,

  RX_BIT_FSM_STS_SPACE_WIDTH
  
} RX_BIT_FSM_STS_t;

RX_BIT_FSM_STS_t RX_BIT_FSM_STS;

int rxBit;
bool bitReceived;

// Set "bitReceived" flag on bit reception and put its logic state into "rxBit"
void bitReceiver(void)
{
  if(RX_BIT_FSM_STS == FORWARD_BIASING)
  {
      delay_us.start(800); // Ts = 800us +10us + 190us = 1ms, HALF_BIT_TIME = 4ms
      
      // RX Led in signalling mode
      pinMode(RC_ANODE_PIN, OUTPUT);
      pinMode(RC_CATHODE_PIN, OUTPUT);
      digitalWrite(RC_ANODE_PIN, LOW); 
      digitalWrite(RC_CATHODE_PIN, LOW); 

      RX_BIT_FSM_STS = REVERSE_BIASING; // FSM STATE TRANSITION
  }
  
  if(delay_us.elapsed())
  {
    if(RX_BIT_FSM_STS == REVERSE_BIASING)
    {
        // RX Led reverse charged
        delay_us.start(10);  
        digitalWrite(RC_ANODE_PIN, LOW); 
        digitalWrite(RC_CATHODE_PIN, HIGH);
        RX_BIT_FSM_STS = REVERSE_CAPACITANCE_DISCHARGING;
    }
    else if(RX_BIT_FSM_STS == REVERSE_CAPACITANCE_DISCHARGING) 
    {
       // RX Led spontaneous discharge
       delay_us.start(190); 
       pinMode(RC_CATHODE_PIN, INPUT);
       RX_BIT_FSM_STS = DIGITAL_SAMPLING;
     }
    else if(RX_BIT_FSM_STS == DIGITAL_SAMPLING) 
    {
       // RX Led spontaneous discharge
       rxBit = digitalRead(RC_CATHODE_PIN) ^ 1; // In order to receive in positive logic vs the tx
       bitReceived = true;
       RX_BIT_FSM_STS = FORWARD_BIASING; // Restart the cycle
     }
    else
    {
      RX_BIT_FSM_STS = FORWARD_BIASING; 
    }
  }  
}

int prevRxBit = 0;
bool edgeDetected = false;

void DecodeManchesterFrame(void)
{
  unsigned int receivedCommand;

  // Edge detector
  if(rxBit != prevRxBit)
  {
    edgeDetected = true;
  }
  else
  {
    edgeDetected = false;
  }
  
  prevRxBit = rxBit;
  
  switch(DECODER_FSM_STS)
  {
    case WAIT_FOR_IDLE:
      if(rxBit) DECODER_FSM_STS = WAIT_FOR_FALLING_EDGE;
    break;

    case WAIT_FOR_FALLING_EDGE:
      if(!rxBit)
      { 
        DECODER_FSM_STS = WAIT_FOR_START_BIT; 
        samplesToSkip = 3;  
      }
    break;  
     
    case WAIT_FOR_START_BIT:
     if(--samplesToSkip == 0)
     {
      if(!rxBit)
      {
        DECODER_FSM_STS = WAIT_FIRST_HALF_BIT_OF_PAYLOAD;
        rxBitCnt = 0;
        samplesToSkip = SAMPLES_TO_SKIP;
      }
      else 
        DECODER_FSM_STS = WAIT_FOR_FALLING_EDGE; 
     }
    break;   
    
    case WAIT_FIRST_HALF_BIT_OF_PAYLOAD:
      if(--samplesToSkip == 0)
      {
        samplesToSkip = SAMPLES_TO_SKIP;
        
        if(rxBit)
          DECODER_FSM_STS = CONFIRMING_ZERO;
        else
          DECODER_FSM_STS = CONFIRMING_ONE;
      }
    break;   
  
    case CONFIRMING_ZERO:

      if(edgeDetected)
        samplesToSkip = SAMPLES_TO_SKIP/2;
            
      if(--samplesToSkip == 0)
      {
        if(!rxBit) // Zero confirmed
        {
          rxBitCnt++;
          
          rxFrame>>=1;
          rxFrame &= 0x7FFF;

          samplesToSkip = SAMPLES_TO_SKIP;

          if(rxBitCnt < 16)
            DECODER_FSM_STS = WAIT_FIRST_HALF_BIT_OF_PAYLOAD;
          else
            DECODER_FSM_STS = WAIT_FIRST_HALF_STOP_BIT;  
        }
        else
          DECODER_FSM_STS = WAIT_FOR_IDLE;
      }    
    break;   

    
    case CONFIRMING_ONE:
      if(edgeDetected)
        samplesToSkip = SAMPLES_TO_SKIP/2;
              
      if(--samplesToSkip == 0)
      {
        if(rxBit) // One confirmed
        {
          rxBitCnt++;
          
          rxFrame>>=1;
          rxFrame |= 0x8000;

          samplesToSkip = SAMPLES_TO_SKIP;
            
          if(rxBitCnt < 16)
            DECODER_FSM_STS = WAIT_FIRST_HALF_BIT_OF_PAYLOAD;
          else
            DECODER_FSM_STS = WAIT_FIRST_HALF_STOP_BIT;  
        }
        else
          DECODER_FSM_STS = WAIT_FOR_IDLE;
      } 
    break;   
    
    case WAIT_FIRST_HALF_STOP_BIT:
       if(--samplesToSkip == 0)
       {
         if(!rxBit)
        {
          samplesToSkip = SAMPLES_TO_SKIP;
          DECODER_FSM_STS = WAIT_SECOND_HALF_STOP_BIT;
        }
        else
          DECODER_FSM_STS = WAIT_FOR_IDLE;   
       }
    break; 
      
    case WAIT_SECOND_HALF_STOP_BIT:
       if(--samplesToSkip == 0)
       {
         if(rxBit)
        {
          receivedCommand = (byte)(rxFrame & 0x00FF);
          
          if(receivedCommand == (byte)(~((rxFrame>>8) & 0x00FF)))
          {
            decodedReply = receivedCommand;
            replyReceived = true;
          }
          
          DECODER_FSM_STS = WAIT_FOR_FALLING_EDGE;
        }
        else
          DECODER_FSM_STS = WAIT_FOR_IDLE;   
       }    
    break;   
  }
}

void loop() {
  
  debounceSwitch();  
  
  if(switchStatusChangedEvent == 1 && switchSts == SWITCH_CLOSED)
  {
    if(cmdToSend == CMD_SET_LED_ON)
    {
      cmdToSend = CMD_SET_LED_OFF;
      blinkStatus = LOW;
    }
    else if(cmdToSend == CMD_SET_LED_OFF)
    { 
      cmdToSend = CMD_SET_LED_BLINK;
      blinkStatus = HIGH;
    }
    else if(cmdToSend == CMD_SET_LED_BLINK)
    {
      cmdToSend = CMD_SET_LED_ON;
      blinkStatus = HIGH;   
    }
  }

    digitalWrite(INTERNAL_LED_PIN, blinkStatus);

   switch(COMMUNICATION_STATUS)
   {
      case RECEIVING:
        if(rx_timeout_us.elapsed())
        {
          delay_us.stop();
          
          pinMode(RC_ANODE_PIN, OUTPUT);
          pinMode(RC_CATHODE_PIN, OUTPUT);
          digitalWrite(RC_ANODE_PIN, LOW); 
          digitalWrite(RC_CATHODE_PIN, LOW);
          
          COMMUNICATION_STATUS = TRANSMITTING;  
          startTx = true;
        }

        else
        {

          bitReceiver();
 
          if(bitReceived)
          {
            bitReceived = false;
            
            digitalWrite(DEMODULATOR_OUTPUT_PIN, rxBit); // DEBUG
            
            DecodeManchesterFrame();
        
            if(replyReceived)
            {
              replyReceived = false;
        
              if(decodedReply == ACK)
              {
                 rx_timeout_us.stop();
                 delay_us.stop();
                 COMMUNICATION_STATUS = TRANSMITTING;  
                 startTx = true;
              }
              
            } 
          }
        }

        
      break;

      case TRANSMITTING:
          if(startTx)
          {
            startTx = false;
            txHalfBitCnt = 0;
      
            currentlyTransmittingCmd = (((unsigned int) ~cmdToSend)<<8) + ((unsigned int) cmdToSend); // Latch command
            
            digitalWrite(RC_ANODE_PIN, HIGH); // FIRST HALF OF THE START BIT '0'
            delay_us.start(HALF_BIT_TIME_us);  
      
            if(cmdToSend == CMD_SET_LED_BLINK)
              blinkStatus ^= HIGH;
          }
        
          if(delay_us.elapsed())
          {
          
          if(txHalfBitCnt == 0)
          {
            digitalWrite(RC_ANODE_PIN, LOW); // SECOND HALF OF THE START BIT '0'
            delay_us.start(HALF_BIT_TIME_us);
          }
          else if(txHalfBitCnt <= 32) // 2 bytes * 8 bits * 2 half bit
          {
      
            if(txHalfBitCnt & 1) // FIRST HALF BIT OF THE PAYLOAD
            {
              txBit = currentlyTransmittingCmd & 0x0001; // LSB first command the its complemented one
              currentlyTransmittingCmd >>= 1;
      
              if(txBit)
                digitalWrite(RC_ANODE_PIN, LOW);
               else
                digitalWrite(RC_ANODE_PIN, HIGH);
            }
            else // SECOND HALF BIT OF THE PAYLOAD
            {
               if(txBit)
                digitalWrite(RC_ANODE_PIN, HIGH);
               else
                digitalWrite(RC_ANODE_PIN, LOW);       
            }
           
            delay_us.start(HALF_BIT_TIME_us);    
          }
          else if(txHalfBitCnt == 33)
          {
            digitalWrite(RC_ANODE_PIN, LOW); // FIRST HALF OF THE STOP BIT '1'
            delay_us.start(HALF_BIT_TIME_us);      
          }
          else if(txHalfBitCnt == 34)
          {
            digitalWrite(RC_ANODE_PIN, HIGH); // SECOND HALF OF THE STOP BIT '1'
            delay_us.start(HALF_BIT_TIME_us);      
          }
          else if(txHalfBitCnt == 35) // Wait ACK
          {
            COMMUNICATION_STATUS = RECEIVING;
            RX_BIT_FSM_STS = FORWARD_BIASING;
            bitReceived = false;

            rxBitCnt = 0;
            DECODER_FSM_STS = WAIT_FOR_IDLE;
            replyReceived = false;
            
            rx_timeout_us.start(400000); // 400ms TOT
            // delay_us.start(INTERFAME_TIME_us);  // 200ms
          }

          /*else
          {
            startTx = true;  
          }
          */
          txHalfBitCnt++;
        }
      break; 

      default:
        COMMUNICATION_STATUS = TRANSMITTING;
      break;
   }
  }
