#include "avdweb_VirtualDelay.h"

#define APPLIANCE_ANODE_PIN    12
#define APPLIANCE_CATHODE_PIN   8
#define DEMODULATOR_OUTPUT_PIN  3
#define INTERNAL_LED_PIN       13
#define DEBUG_PIN               7

#define CMD_SET_LED_ON       0
#define CMD_SET_LED_OFF   0xFF
#define CMD_SET_LED_BLINK 0xAA

#define SAMPLES_TO_SKIP 8

#define HALF_BIT_TIME_us    8000

#define ACK   0x55AA
#define NACK  0x01FE

typedef enum
{
  FORWARD_BIASING = 0,
  REVERSE_BIASING = 1,
  REVERSE_CAPACITANCE_DISCHARGING  = 2,
  DIGITAL_SAMPLING = 3,

  RX_BIT_FSM_STS_SPACE_WIDTH
  
} RX_BIT_FSM_STS_t;

RX_BIT_FSM_STS_t RX_BIT_FSM_STS;

VirtualDelay delay_us(micros);
VirtualDelay delay_us_blinking(micros);

bool bitReceived;
bool commandReceived;

int rxBit;

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
byte decodedCmd;

int GUI_lesStatus;

typedef enum
{
  RECEIVING     = 0,
  TRANSMITTING  = 1,
  
  COMMUNICATION_STATUS_SPACE_WIDTH
} COMMUNICATION_STATUS_t;

COMMUNICATION_STATUS_t COMMUNICATION_STATUS;

void setup() {
  // put your setup code here, to run once:
  pinMode(APPLIANCE_ANODE_PIN, OUTPUT);
  pinMode(APPLIANCE_CATHODE_PIN, OUTPUT);
  pinMode(DEMODULATOR_OUTPUT_PIN, OUTPUT);
  pinMode(INTERNAL_LED_PIN, OUTPUT);
  pinMode(DEBUG_PIN, OUTPUT);
  
  digitalWrite(APPLIANCE_ANODE_PIN, HIGH);  // Start with rx led solid ON
  digitalWrite(APPLIANCE_CATHODE_PIN, LOW);
  digitalWrite(INTERNAL_LED_PIN, HIGH);     // aligned to the builtin led
  digitalWrite(DEBUG_PIN, LOW);
  
  Serial.begin(115200);
  while (!Serial);

  COMMUNICATION_STATUS = RECEIVING;

  DECODER_FSM_STS = WAIT_FOR_IDLE;
  rxBitCnt = 0;

  GUI_lesStatus = 1;
  
  bitReceived = false;
  commandReceived = false;
  RX_BIT_FSM_STS = FORWARD_BIASING;
  delay(100);
}

// Set "bitReceived" flag on bit reception and put its logic state into "rxBit"
void bitReceiver(void)
{
  if(RX_BIT_FSM_STS == FORWARD_BIASING)
  {
      delay_us.start(800); // Ts = 800us +10us + 190us = 1ms, HALF_BIT_TIME = 4ms
      
      // RX Led in signalling mode
      pinMode(APPLIANCE_ANODE_PIN, OUTPUT);
      pinMode(APPLIANCE_CATHODE_PIN, OUTPUT);
    
      if(GUI_lesStatus)
      {
        digitalWrite(APPLIANCE_ANODE_PIN, HIGH); 
        digitalWrite(INTERNAL_LED_PIN, HIGH);
      } 
      else
      {
       digitalWrite(APPLIANCE_ANODE_PIN, LOW); 
       digitalWrite(INTERNAL_LED_PIN, LOW);
      }
       
      digitalWrite(APPLIANCE_CATHODE_PIN, LOW); 

      RX_BIT_FSM_STS = REVERSE_BIASING; // FSM STATE TRANSITION
  }
  
  if(delay_us.elapsed())
  {
    if(RX_BIT_FSM_STS == REVERSE_BIASING)
    {
        // RX Led reverse charged
        delay_us.start(10);  
        digitalWrite(APPLIANCE_ANODE_PIN, LOW); 
        digitalWrite(APPLIANCE_CATHODE_PIN, HIGH);
        RX_BIT_FSM_STS = REVERSE_CAPACITANCE_DISCHARGING;
    }
    else if(RX_BIT_FSM_STS == REVERSE_CAPACITANCE_DISCHARGING) 
    {
       // RX Led spontaneous discharge
       delay_us.start(190); 
       pinMode(APPLIANCE_CATHODE_PIN, INPUT);
       RX_BIT_FSM_STS = DIGITAL_SAMPLING;
     }
    else if(RX_BIT_FSM_STS == DIGITAL_SAMPLING) 
    {
       // RX Led spontaneous discharge
       rxBit = digitalRead(APPLIANCE_CATHODE_PIN) ^ 1; // In order to receive in positive logic vs the tx
       bitReceived = true;
       RX_BIT_FSM_STS = FORWARD_BIASING; // Restart the cycle
     }
    else
    {
      RX_BIT_FSM_STS = FORWARD_BIASING; 
    }
  }  
}

int debugStatus = 0;

void toggleDebugPin(void)
{
       debugStatus ^= 1;
       digitalWrite(DEBUG_PIN, debugStatus);  
}

void setDebugPin()
{
       debugStatus = 1;
       digitalWrite(DEBUG_PIN, debugStatus);    
}

void resetDebugPin()
{
       debugStatus = 0;
       digitalWrite(DEBUG_PIN, debugStatus);    
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

      setDebugPin();
    break;

    case WAIT_FOR_FALLING_EDGE:
      if(!rxBit) { DECODER_FSM_STS = WAIT_FOR_START_BIT; samplesToSkip = 3; resetDebugPin(); }
    break;  
     
    case WAIT_FOR_START_BIT:
     if(--samplesToSkip == 0)
     {
      if(!rxBit)
      {
        DECODER_FSM_STS = WAIT_FIRST_HALF_BIT_OF_PAYLOAD;
        rxBitCnt = 0;
        samplesToSkip = SAMPLES_TO_SKIP;
        toggleDebugPin();
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

          toggleDebugPin();
      }
    break;   
  
    case CONFIRMING_ZERO:

      if(edgeDetected)
        samplesToSkip = SAMPLES_TO_SKIP/2;
            
      if(--samplesToSkip == 0)
      {
        if(!rxBit) // Zero confirmed
        {
          toggleDebugPin();
          
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

        toggleDebugPin();
        
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
          toggleDebugPin();
          
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
         toggleDebugPin();
         
          receivedCommand = (byte)(rxFrame & 0x00FF);
          
          if(receivedCommand == (byte)(~((rxFrame>>8) & 0x00FF)))
          {
            decodedCmd = receivedCommand;
            commandReceived = true;
          }
          
          DECODER_FSM_STS = WAIT_FOR_FALLING_EDGE;
        }
        else
          DECODER_FSM_STS = WAIT_FOR_IDLE;   
       }    
    break;   
  }
}

bool cmdRecognized;

void executeCommand(void)
{
  cmdRecognized = true;
  
  switch(decodedCmd)
  {
    case CMD_SET_LED_ON:
      GUI_lesStatus = 1;
      Serial.println("ON");
    break;

    case CMD_SET_LED_OFF:
      GUI_lesStatus = 0;
      Serial.println("OFF");
    break;

    case CMD_SET_LED_BLINK:
      delay_us_blinking.start(500000);
      Serial.println("BLINK");
    break;
    
    default:
      // Discard unrecognized command
      cmdRecognized = false;
    break;
  }
}

bool startTx;
int txHalfBitCnt;
int txBit;
unsigned int reply;

void loop() {

  switch(COMMUNICATION_STATUS)
  {
    case RECEIVING:
      
      bitReceiver();
 
      if(bitReceived)
      {
        bitReceived = false;
        
        digitalWrite(DEMODULATOR_OUTPUT_PIN, rxBit); // DEBUG
        
        DecodeManchesterFrame();
    
        if(commandReceived)
        {
          commandReceived = false;
    
          executeCommand();

          COMMUNICATION_STATUS = TRANSMITTING; 
          startTx = true;
          
        }
      }
    break;

    case TRANSMITTING:

      if(startTx)
      {
        pinMode(APPLIANCE_ANODE_PIN, OUTPUT);
        pinMode(APPLIANCE_CATHODE_PIN, OUTPUT);
        
        startTx = false;
        txHalfBitCnt = 0;

        if(cmdRecognized)
          reply = ACK; 
        else
          reply = NACK;  
        
        digitalWrite(APPLIANCE_ANODE_PIN, HIGH); // FIRST HALF OF THE START BIT '0'
        digitalWrite(APPLIANCE_CATHODE_PIN, LOW); 
        
        delay_us.start(HALF_BIT_TIME_us);  
      }
    
    if(delay_us.elapsed())
    {
    
    if(txHalfBitCnt == 0)
    {
      digitalWrite(APPLIANCE_ANODE_PIN, LOW); // SECOND HALF OF THE START BIT '0'
      delay_us.start(HALF_BIT_TIME_us);
    }
    else if(txHalfBitCnt <= 32) // 2 bytes * 8 bits * 2 half bit
    {

      if(txHalfBitCnt & 1) // FIRST HALF BIT OF THE PAYLOAD
      {
        txBit = reply & 0x0001; // LSB first command the its complemented one
        reply >>= 1;

        if(txBit)
          digitalWrite(APPLIANCE_ANODE_PIN, LOW);
         else
          digitalWrite(APPLIANCE_ANODE_PIN, HIGH);
      }
      else // SECOND HALF BIT OF THE PAYLOAD
      {
         if(txBit)
          digitalWrite(APPLIANCE_ANODE_PIN, HIGH);
         else
          digitalWrite(APPLIANCE_ANODE_PIN, LOW);       
      }
     
      delay_us.start(HALF_BIT_TIME_us);    
    }
    else if(txHalfBitCnt == 33)
    {
      digitalWrite(APPLIANCE_ANODE_PIN, LOW); // FIRST HALF OF THE START BIT '1'
      delay_us.start(HALF_BIT_TIME_us);      
    }
    else if(txHalfBitCnt == 34)
    {
      digitalWrite(APPLIANCE_ANODE_PIN, HIGH); // SECOND HALF OF THE START BIT '1'
      delay_us.start(HALF_BIT_TIME_us);      
    }
    else if(txHalfBitCnt == 35) // Interframe
    {
     //delay(10);
     COMMUNICATION_STATUS = RECEIVING;
    }

    txHalfBitCnt++;
  }      

  
    break;

    default:
      COMMUNICATION_STATUS = RECEIVING;
    break;
  }



  if(decodedCmd == CMD_SET_LED_BLINK)
  {
    if(delay_us_blinking.elapsed())
    {
        GUI_lesStatus ^= 1;  
        delay_us_blinking.start(500000);
    }

         
  }
  //Serial.println(rxBit);
}
