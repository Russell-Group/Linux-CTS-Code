//////////////////////////////////////////////////////////////////////////////////
// LSU_CTS_control_v1.ino
// C. Domangue, T.Kutter - LSU
//////////////////////////////////////////////////////////////////////////////////
//
// Uses DIODE SENSORS (LS6-LS9) for level control - no Grenoble sensors required
//
// ----------------------------------------------------------------------------
//  STATE MACHINE STRUCTURE
//
// The state machine is now a TREE rooted at IDLE, with two parallel
// branches:
//
//                    IDLE (1)
//                   /        \
//              AFILL (2)    WGAS (3)
//                |            |
//             cutoff        CGAS (5)  
//                |            |
//              IDLE         IMER (4)
//                             |
//                          WARM (6) 
//                             |
//                           IDLE
//
// AFILL branch (storage dewar fill) is independent of the basin
// sequence
//
// Basin-cooldown sequence: WGAS -> CGAS -> IMER -> WARM -> IDLE.
//   WGAS  (3) - Warm GN purge + lid heater + fans. Dries the chamber
//               before any LN2 contact. Unchanged from previous versions.
//   CGAS  (5) - VERY GRADUAL fill to <= 0.5" (LS9 immersed). Closes SD
//               vents to build SD pressure, modulates the GN drive valve
//               based on LS9 status. Includes a 3-min cool-down soak
//               gate before the operator may advance to IMER. 
//   IMER  (4) - Gradual fill to ~5-7" (LS6 immersed, LS5 marginal),
//               hold at LS6. 
//   WARM  (6) - Drain LN2 (Phase 0: open vents + lid heat to evaporate),
//               then dry-out (Phase 1: WGAS-equivalent: GN flow + lid
//               heater + fans). Operator advances to IDLE when ready
//               (cycle complete).
//
// HAND mode mode-button mapping in tree:
//   IDLE:  ModeUp -> WGAS (start basin sequence)
//          ModeDown -> AFILL (start storage dewar fill)
//   WGAS:  ModeUp -> CGAS, ModeDown -> IDLE
//   CGAS:  ModeUp -> IMER (gated by 3-min soak), ModeDown -> WGAS
//   IMER:  ModeUp -> WARM, ModeDown -> CGAS
//   WARM:  ModeUp -> IDLE (cycle complete), ModeDown -> no-op
//   AFILL: ModeUp -> WGAS, ModeDown -> IDLE
//
// ============================================================================

#if defined(WIRING) && WIRING >= 100
  #include <Wiring.h>
#elif defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <Adafruit_AHTX0.h>

#include <Wire.h>
//#include <Adafruit_ADS1015.h>
#include <Adafruit_ADS1X15.h>

#include <TimerOne.h>


// Grenoble sensor control
bool GRENOBLE_ENABLED = false;

// AD7746 I2C address and register definitions
#define AD7746_ADDR 0x48
#define AD7746_REG_STATUS      0x00
#define AD7746_REG_CAP_DATA_H  0x01
#define AD7746_REG_CAP_DATA_M  0x02
#define AD7746_REG_CAP_DATA_L  0x03
#define AD7746_REG_CAP_SETUP   0x07
#define AD7746_REG_EXC_SETUP   0x09
#define AD7746_REG_CONFIGURATION 0x0A
#define AD7746_REG_CAPDAC_A    0x0B
#define AD7746_REG_CAPDAC_B    0x0C
#define AD7746_REG_CAP_OFFSET_H 0x0D
#define AD7746_REG_CAP_OFFSET_L 0x0E
#define AD7746_REG_CAP_GAIN_H  0x0F
#define AD7746_REG_CAP_GAIN_L  0x10
#define AD7746_REG_VOLT_GAIN_H 0x11
#define AD7746_REG_VOLT_GAIN_L 0x12

// include the library code:
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(52, 53, 51, 50, 49, 48);
//  This specifies LCD pins: RS, EN, D4, D5, D6, D7

// NO AD7746 library objects - using raw I2C instead


// Constants for Mega pin assignments
const int DewarPInterlock = 6; //Safety Relay Q5 - Dewar Overpressure OK, Hardware interlocked
const int DewarLInterlock = 4; //Safety Relay Q4 - Dewar Overfill OK, Hardware interlocked
const int BasinLInterlock = 5; //Safety Relay Q3 - Basin Overpressure OK, Hardware interlocked

const int EnableLid = 31;     //Enable Heated Lid 
const int LED_Fault = 33;     //Fault state LED indicator
const int LED_Idle = 34;      //Idle state LED indicator 
const int LED_Autofill = 35;  //Autofill state LED indicator 
const int LED_WarmGas = 36;   //Warm gas state LED indicator 
const int LED_ColdGas = 37;   //Cold gas state LED indicator 
const int LED_Immerse = 38;   //Immerse state LED indicator  
const int LED_Warmup = 39;   //Warmup state LED indicator 

const int MuxA = 68;         //Level sensor multiplexer select bit 3
const int MuxB = 67;         //Level sensor multiplexer select bit 2
const int MuxC = 65;         //Level sensor multiplexer select bit 1
const int MuxD = 66;         //Level sensor multiplexer select bit 0
const int ModeDown = 3;      //Button control, decrement state by 1
const int ModeUp = 2;        //Button control, increment state by 1

const int ExValveCMD1 = 29;
const int ExValveCMD2 = 28;
const int ExValveCMD3 = 27;  //Not Implemented
const int WarmGasCMD = 30;
const int HornCMD = 32;
const int LiquidCMD = 22;

//Modulating Valve DAC
const byte DacBits[8] = {47, 46, 45, 44, 43, 42, 41, 40};

//USB Data Downlink
const int UsbRst = 7;
const int UsbTx = 18;
const int UsbRx = 19;
const int UsbCTS = 9;
const int UsbRTS = 8;

//LVDS Data Link
const int LvdsEN = 10;
const int Line1RX = 17;
const int Line1TX = 16;
const int Line2RX = 15;
const int Line2TX = 14;

//I2C Switch
const int i2c0_SDA = 20;
const int i2c0_SCL = 21;
const int i2c0_Rst = 69;

//HOA Switch
const int Hand = 64;
const int Auto = 63;
const int Remote = 62;

//Auto Control Buttons
const int AfillStart = 61;
const int AImmerStart = 60;
const int AStop = 59;


//Misc Outputs
//+24V
const int MiscOut1 = 23;    //Accessory outputs, not used
const int MiscOut2 = 24;
const int MiscOut3 = 25;
const int MiscOut4 = 26;

//Stepper Control - Not implemented
const int stepperEN = 54;
const int stepperDIR = 55;
const int stepperPulse = 56;

//Display Color Arguments

char serialData;

  
const int i2cChannel0 = 1;
const int i2cChannel1 = 2;
const int i2cChannel2 = 4;
const int i2cChannel3 = 8;

int Sanity = 1; // flag for sanity checking
int MessCntr = 0; // message disply countdown
int TC_Level = 0; // Comes from L16 - L12, L16 is Overfill, L15 is full
int Dewar_Level = 0; //Comes fro L1 - L4, L1 is Overfill, L2 is full
int LevelSensor[16];
int LevelStatus[16];
int Pressure[2];      // read from ADC from analog sensor
float GrenobleLevel[2]; 
uint32_t GrenobleRaw[2] = {0, 0};
float   GrenobleRel[2] = {NAN, NAN};   // relative pF from library
uint8_t GrenobleCapDacA = 0;
uint8_t GrenobleCapDacB = 0;

float dewarPress = 0;
int STATE = 1;        // STATE ranges from 0 to 4
int PressureADC;
int Err_Code = 0; // start with no Error Code set
int P_Limit1 = 2500;  // 12" rise of LN2 = 0.35 PSI = 2.41 kPa = 0.417 VDC = 3337 counts (max during warm gas to prevent LN2 entry into Test Chamber)
int P_Limit2 = 9376;  //7500; // max during normal operation, exceed --> heater off, valve open
int P_Limit3 = 10400; //8500; // This is the electronic pressure control limit, causes system fault
int16_t adc0; // Force to be unsigned 16 bit integer 
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
int shortCycleTime = 30000;
unsigned long lastMillis = 0;

int chamberDAQ = 0;
int dewarDAQ = 0;
int allDAQ = 0;
int fastDAQ = 1;
int slowDAQ = 100;
int DAQrate = 100;
char valvePct = 0;
String valveStr;
float valveFlo = 0;
float valveCounts1 = 0;
int valveCounts2 = 0;

Adafruit_AHTX0 aht;

bool grenoble_print = false;

// Forward declarations for raw I2C Grenoble functions
bool ad7746_writeReg(uint8_t reg, uint8_t value);
uint8_t ad7746_readReg(uint8_t reg);
uint32_t ad7746_read24bit(uint8_t reg);
bool ad7746_init();
void GrenobleReadout_Raw();
void ad7746_dumpRegisters();  // Diagnostic function

// ============================================================================
// NEW VARIABLES FOR GRADUAL BASIN FILLING (DIODE SENSOR-BASED)
// ============================================================================
int immerse_stage = 0;
unsigned long immerse_timer = 0;
const unsigned long HOLD_TIME_MS = 600000;  // 10 minutes
const int VALVE_MIN = 0;
const int VALVE_MAX = 255;
const int VALVE_STARTUP = 40;
// ============================================================================

// ============================================================================
// CGAS / WARM state tracking (v16)
// ============================================================================
// CGAS soak gate:
//   Once LS9 first becomes immersed during CGAS, the firmware records the
//   timestamp. Operator-initiated transition CGAS -> IMER is suppressed
//   until CGAS_SOAK_MS has elapsed since LS9 first became immersed (the
//   minimum cool-down soak). Reset on every entry to CGAS.
const unsigned long CGAS_SOAK_MS = 180000UL;   // 3 minutes (was 10 min in v16)
unsigned long cgas_ls9_first_immersed_ms = 0;  // 0 = not yet immersed
bool cgas_soak_complete = false;

// CGAS DAC ramp:
//   The DAC value (GN modulating valve drive) ramps UP at a controlled
//   rate to avoid an abrupt fountain of LN into the basin when CGAS
//   begins. Decreases are immediate (safety always backs off promptly).
//   Reset on every entry to CGAS.
const unsigned long CGAS_DAC_RAMP_STEP_MS = 500UL;  // +1 count per 500 ms (40 in ~20 s)
int cgas_dac_current = 0;
unsigned long cgas_dac_last_step_ms = 0;

// WARM phase:
//   0 = drain phase: open vents, run lid heater, wait for LS9 to be dry
//       (LevelStatus[9] != 4) - LN2 has boiled off below 0.5".
//   1 = dry-out phase: same as WGAS (GN flow + lid heater + fans). The
//       operator advances WARM -> IDLE manually.
int warm_phase = 0;
// ============================================================================

// ============================================================================
// DUAL-TIMER DEWAR AUTOFILL CUTOFF (STATE 2)
// ============================================================================
// Cutoff requires BOTH of:
//   LS1 (Dewar Full,     21" probe) raw ADC IN (LS1_FULL_THRESHOLD     ..SATURATION_CEILING) for >= LS1_FULL_HOLD_MS
//   LS0 (Dewar Overfill, 25" probe) raw ADC IN (LS0_OVERFILL_THRESHOLD ..SATURATION_CEILING) for >= LS0_OVERFILL_HOLD_MS
//
// The upper bound (SATURATION_CEILING) rejects warm-temperature readings
// where the LM317-driven diode sensor saturates to ~32767. Without this
// ceiling, a fresh fill of a warm dewar would falsely register as "full"
// and close the valve immediately.
//
// Each timer is independent and resets if its sensor leaves its window
// (drops below threshold OR rises above the saturation ceiling).
// Once both are satisfied, autofill_full_latched holds the valves closed
// until STATE 2 is re-entered.
const int LS1_FULL_THRESHOLD     = 20500;       // raw ADC counts on LevelSensor[1]
const int LS0_OVERFILL_THRESHOLD = 21000;       // raw ADC counts on LevelSensor[0]
const int SATURATION_CEILING     = 22000;       // upper bound - readings above are warm/saturated
const unsigned long LS1_FULL_HOLD_MS     = 10000UL;  // 10 s sustained over LS1 threshold
const unsigned long LS0_OVERFILL_HOLD_MS =  5000UL;  //  5 s sustained over LS0 threshold

unsigned long ls1_full_first_ms     = 0;   // millis() when LS1 first crossed threshold; 0 = not currently in-window
unsigned long ls0_overfill_first_ms = 0;   // millis() when LS0 first crossed threshold; 0 = not currently in-window
bool autofill_full_latched = false;        // true once both timers have been satisfied

// Boot-phase flag: gates the audible horn alarm so spurious faults during
// setup() (e.g. transient sensor readings before the system stabilizes) do
// not produce a nuisance audible alarm on every power-up. Set true at the
// end of setup(). When false, Pulse_Horn() is a no-op (silently returns)
// while leaving all other fault behavior - LCD, LED, valve-close, serial
// log - fully intact.
bool boot_complete = false;
// ============================================================================

// ============================================================================
// ROAH SELECTOR STATE
// ============================================================================
// 4 valid positions, encoded by which (if any) of pins 64/63/62 is HIGH.
// ROAH_INVALID is reserved for wiring-fault detection (multiple bits HIGH,
// or any other unexpected combination).
enum ROAH_Position {
  ROAH_OFF = 0,
  ROAH_HAND,
  ROAH_AUTO,
  ROAH_REMOTE,
  ROAH_INVALID
};

ROAH_Position roah_current = ROAH_OFF;     // last-known position
ROAH_Position roah_previous = ROAH_OFF;    // for edge-logging

// AUTO button edge-detection state.
// AUTO buttons (B2-B6 panel pushbuttons on J15) are wired with external
// pull-downs (R60-R62 on schematic Sheet 10). They go HIGH when pressed,
// LOW when released - OPPOSITE polarity to ModeUp/ModeDown which are
// active-low. We track previous state to fire on the rising edge only,
// so a held button does not chain through multiple state transitions.
int afill_btn_prev = LOW;
int basin_btn_prev = LOW;
int stop_btn_prev  = LOW;

// Human-readable name for serial logs and diagnostic output.
static const char* ROAH_Name(ROAH_Position p) {
  switch (p) {
    case ROAH_OFF:     return "OFF";
    case ROAH_HAND:    return "HAND";
    case ROAH_AUTO:    return "AUTO";
    case ROAH_REMOTE:  return "REMOTE";
    case ROAH_INVALID: return "INVALID";
  }
  return "?";
}
// ============================================================================


void setup()
{
  // Suppress audible horn alarm during init (also during recovery path
  // where setup() is called from the STATE 0 acknowledge handler).
  // Re-armed at end of setup().
  boot_complete = false;
  digitalWrite(HornCMD, LOW);   // belt-and-suspenders: horn off during init

  Serial.begin(115200);
  delay(10);
  Serial.println(F("Initalizing CTS System..."));
  Serial.println(F("Setting I/O"));
  //LED indicators
  pinMode(LED_Fault, OUTPUT);
  pinMode(LED_Idle, OUTPUT);
  pinMode(LED_Autofill, OUTPUT);
  pinMode(LED_WarmGas, OUTPUT);   
  pinMode(LED_ColdGas, OUTPUT);
  pinMode(LED_Immerse, OUTPUT);
  pinMode(LED_Warmup, OUTPUT);

  //Safety interlock relays 
  pinMode(DewarPInterlock, OUTPUT);
  pinMode(DewarLInterlock, OUTPUT);
  pinMode(BasinLInterlock, OUTPUT);

  //Heated Lid relays
  pinMode(EnableLid, OUTPUT);

  //Valves
  pinMode(LiquidCMD, OUTPUT);   //Liquid valve     (SD fill)          - Wired for hardware safety interlock
  pinMode(ExValveCMD1, OUTPUT);   //Cold gas valve 1 NO (Dewar Vent)
  pinMode(ExValveCMD2, OUTPUT);   //Cold gas valve 2 NC (Dewar Vent)
  pinMode(WarmGasCMD, OUTPUT);    //Warm gas valve 2 (basin dry gas)

  pinMode(HornCMD, OUTPUT);

  //ADC Level sensor multiplexer
  pinMode(MuxA,OUTPUT);
  pinMode(MuxB,OUTPUT);
  pinMode(MuxC,OUTPUT);
  pinMode(MuxD,OUTPUT);

  //DAC Valve Outputs
  pinMode(DacBits[0], OUTPUT);
  pinMode(DacBits[1], OUTPUT);
  pinMode(DacBits[2], OUTPUT);
  pinMode(DacBits[3], OUTPUT);
  pinMode(DacBits[4], OUTPUT);
  pinMode(DacBits[5], OUTPUT);
  pinMode(DacBits[6], OUTPUT);
  pinMode(DacBits[7], OUTPUT);

  //Mode change buttons
  pinMode(ModeUp, INPUT);
  pinMode(ModeDown, INPUT);

  //ROAH Selector (external 10kOhm pull-downs on schematic, so plain INPUT)
  pinMode(Hand, INPUT);
  pinMode(Auto, INPUT);
  pinMode(Remote, INPUT);

  //Auto Control Buttons (external pull-downs on schematic, plain INPUT)
  pinMode(AfillStart, INPUT);
  pinMode(AImmerStart, INPUT);
  pinMode(AStop, INPUT);


  //I2C Switch
  pinMode(i2c0_Rst, OUTPUT);
  Wire.begin();
  Wire.setWireTimeout(25000, true);

  Serial.println(F("Testing I/O"));
  //Test cycle indicators, set relays off
  //LCD Self test message
 // lcd.begin(16, 2);
 // lcd.print(F("LSU CTS V2"));
 // delay(2000);

// Initialize LCD with proper timing
Serial.println(F("Initializing LCD..."));

// Give LCD time to power up (some LCDs need this)
delay(100);

lcd.begin(16, 2);
delay(50);  // Allow LCD to initialize

// Clear any garbage
lcd.clear();
delay(50);

// Set cursor and display test message
lcd.setCursor(0, 0);
lcd.print(F("LSU CTS V2"));
lcd.setCursor(0, 1);
lcd.print(F("Initializing..."));

Serial.println(F("LCD initialized"));
delay(2000);
lcd.clear();

  digitalWrite(LED_Fault, HIGH);
  delay(500);
  digitalWrite(LED_Fault, LOW);
  digitalWrite(LED_Idle, HIGH);
  delay(500);
  digitalWrite(LED_Idle, LOW);
  digitalWrite(LED_Autofill, HIGH);
  delay(500);
  digitalWrite(LED_Autofill, LOW);
  digitalWrite(LED_WarmGas, HIGH);
  delay(500);
  digitalWrite(LED_WarmGas, LOW);
  digitalWrite(LED_ColdGas, HIGH);
  delay(500);
  digitalWrite(LED_ColdGas, LOW);
  digitalWrite(LED_Immerse, HIGH);
  delay(500);
  digitalWrite(LED_Immerse, LOW);
  digitalWrite(LED_Warmup, HIGH);
  delay(500);
  digitalWrite(LED_Warmup, LOW);   

  digitalWrite(DewarPInterlock, LOW);
  digitalWrite(DewarLInterlock, LOW);
  digitalWrite(BasinLInterlock, LOW); 

  digitalWrite(LiquidCMD, LOW);     //LQ Vlv
  digitalWrite(ExValveCMD1, LOW);   //CGas 1
  digitalWrite(ExValveCMD2, LOW);   //CGas 2
  digitalWrite(ExValveCMD3, LOW);   //WGas 1
  digitalWrite(WarmGasCMD, LOW);    //WGas 2
  digitalWrite(EnableLid, LOW);
  digitalWrite(HornCMD, LOW);

  digitalWrite (MuxA,LOW);
  digitalWrite (MuxB,LOW);
  digitalWrite (MuxC,LOW);
  digitalWrite (MuxD,LOW);

  //I2C Reset Release
  digitalWrite(i2c0_Rst, HIGH);
  delay(100);

  //Set up ADC
  SetI2C(i2cChannel0);
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit =  0.125mV
  ads.begin();

  //Set up Grenoble Readout - Using RAW I2C (no library)
  Serial.println(F("[GRENOBLE] Initializing via raw I2C..."));
  
  // Set I2C speed
  Wire.setClock(100000);  // 100kHz
  delay(50);
  
  // Switch to mux channel 3
  if (SetI2C(i2cChannel3)) {
    Serial.println(F("[GRENOBLE] Switched to I2C mux channel 3"));
    delay(50);
    
    // Check if AD7746 present
    Wire.beginTransmission(AD7746_ADDR);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.println(F("[GRENOBLE] AD7746 detected at 0x48"));
      
      // Initialize AD7746 using raw I2C
      if (ad7746_init()) {
        Serial.println(F("[GRENOBLE] AD7746 initialized successfully!"));
        GRENOBLE_ENABLED = true;
      } else {
        Serial.println(F("[GRENOBLE] AD7746 initialization failed"));
        GRENOBLE_ENABLED = false;
      }
    } else {
      Serial.print(F("[GRENOBLE] AD7746 not found (error "));
      Serial.print(error);
      Serial.println(F(")"));
      GRENOBLE_ENABLED = false;
    }
  } else {
    Serial.println(F("[GRENOBLE] Could not set I2C mux to channel 3"));
    GRENOBLE_ENABLED = false;
  }
  
  Serial.print(F("[GRENOBLE] Final status: "));
  Serial.println(GRENOBLE_ENABLED ? "ENABLED" : "DISABLED");
 
  Read_Level_Sensors();
  Get_SD_Pressure();

  lcd.setCursor(0,0);     
  lcd.print(F("Enabling System:"));
  Serial.println(F("Enabling System"));
  delay(1500);
  digitalWrite(DewarPInterlock, HIGH);
  digitalWrite(DewarLInterlock, HIGH);
  digitalWrite(BasinLInterlock, HIGH); 
  TC_Level = 0; // an assumption, will test right away
  Sanity = 0; // for now disable Sanity checking
  SetState(1);  
  boot_complete = true;       // Enable audible horn alarm from this point on
  Serial.println(F("[BOOT] Setup complete - horn alarm armed"));
  //Serial.println(F("FLAG 1"));
   
}


// Program structure:
// -- If a command is received on the USB port then process command
// -- Read_Level_Sensors();  // sets TC_Level and can set STATE = 0 and Err_Code if there is a fault
// -- Get_SD_Pressure(); // sets Pressure and can set STATE = 0 and Err_Code if over-pressure detected
// -- Dole out messages on the LCD display

// Program hacking
// -- Don't mess with the state machine
// -- Don't mess with the functions after main()
// -- Do mess with the messages on LCD
// -- Do mess with USB commands and command processor
// -- Do consider creating a autocycle through the states
// ---- Warmgas for xx time, then coldgas for xx time, then immerse and change LCD color
// ---- then warmup cycle
    
void loop()
{
  // Sample the ROAH selector at the top of every loop iteration so all
  // downstream gating sees a fresh value. Logs position changes to serial.
  Update_ROAH();

  // OFF mode: force IDLE if not already there. Skip when STATE==0 so the
  // operator can still acknowledge a fault. Skip when STATE==1 so we
  // don't spam SetState calls every iteration.
  if (roah_current == ROAH_OFF && STATE != 0 && STATE != 1) {
    Serial.println(F("[ROAH] OFF position - forcing IDLE"));
    SetState(1);
  }

  // AUTO mode: poll the three dedicated panel buttons and dispatch
  // edge-triggered state changes. Skipped when STATE==0 because fault
  // must be acknowledged via ModeDown (see fault-ack exemption notes).
  if (roah_current == ROAH_AUTO && STATE != 0) {
    Process_Auto_Buttons();
  }

  // REMOTE mode: ABT/LVDS command handling will go here when implemented.
  // PLACEHOLDER - currently no-op. The ABT receiver should poll Serial2
  // or Serial3 (per schematic Sheet 13: MEGA-RX2/TX2/RX3/TX3) and
  // dispatch state changes only when RemoteControlsAllowed() is true.
  // USB-UART (Serial) commands below remain available always for diagnostics.
  // if (RemoteControlsAllowed()) { Process_ABT_Commands(); }

  //Serial.println(F("FLAG 2"));
  //Aquire and process commands
  if (Serial.available() > 0) {   
    serialData = Serial.read();
    switch (serialData) {
      case '0':
        if (RejectIfOff(0)) break;
        Serial.println(F("Setting STATE to 0 (FAULT)"));
        SetState(0);
        break;
      case '1':
        if (RejectIfOff(1)) break;
        Serial.println(F("Setting STATE to 1 (IDLE)"));
        SetState(1);
        break;
      case '2':
        if (RejectIfOff(2)) break;
        Serial.println(F("Setting STATE to 2 (AUTOFILL)"));
        SetState(2);
        break;
      case '3':
        if (RejectIfOff(3)) break;
        Serial.println(F("Setting STATE to 3 (WARM GAS)"));
        SetState(3);
        break;
      case '4':
        if (RejectIfOff(4)) break;
        Serial.println(F("Setting STATE to 4 (IMMERSE)"));
        SetState(4);
        break;
      case '5':
        if (RejectIfOff(5)) break;
        Serial.println(F("Setting STATE to 5 (COLD GAS)"));
        SetState(5);
        break;
      case '6':
        if (RejectIfOff(6)) break;
        Serial.println(F("Setting STATE to 6 (WARM-UP)"));
        SetState(6);
        break;
      case 's':
        Serial.println(F("Disabling Sanity checking"));
        Sanity = 0;
        break;
      case 'S':
        Serial.println(F("Enabling Sanity checking"));
        Sanity = 1;
        break;
      case 'X':
        Serial.println(F("Entering setup routine"));
        setup();
        break;
      case 'W':
        Test_ROAH_Wiring();
        break;
      case 'F':
        // Force CGAS soak gate to "complete" for dry-run testing.
        // Auto-cleared on next SetState(5) entry to CGAS.
        cgas_soak_complete = true;
        cgas_ls9_first_immersed_ms = 1;  // mark "timer started" for log consistency
        Serial.println(F("[CGAS] Soak gate FORCED complete by 'F' command (dry-run bypass)"));
        Serial.println(F("[CGAS] Operator may now advance CGAS -> IMER. Gate will reset on re-entry to CGAS."));
        break;
      case 'M':
      Serial.println(F("[DBG] ENTERED M CASE"));

      // Make sure we are on Grenoble mux channel
      if (!SetI2C(i2cChannel3)) {
        Serial.println(F("[DBG] FAIL SetI2C(ch3) inside M"));
        break;
      }

      //GrenobleRawDebugOnce();   // <-- should print two [DBG] lines
      //GrenobleReadout();        // your existing prints

      Serial.print(F("Pressure= ")); Serial.println(Pressure[0]);
      Serial.print(F("Pressure2:"));  Serial.println(Pressure[1]);

      for (int i = 0; i < 10; i++) {
        Serial.print(F("LevelSensor[")); Serial.print(i); Serial.print(F("]= "));
        Serial.println(LevelSensor[i]);
      }
        Serial.print(F("Pressure= "));Serial.println(Pressure[0]);
        Serial.print(F("Pressure2:"));Serial.println(Pressure[1]);
        Serial.print(F("LevelSensor[0]= "));Serial.println(LevelSensor[0]);
        Serial.print(F("LevelSensor[1]= "));Serial.println(LevelSensor[1]);
        Serial.print(F("LevelSensor[2]= "));Serial.println(LevelSensor[2]);
        Serial.print(F("LevelSensor[3]= "));Serial.println(LevelSensor[3]);
        Serial.print(F("LevelSensor[4]= "));Serial.println(LevelSensor[4]);
        Serial.print(F("LevelSensor[5]= "));Serial.println(LevelSensor[5]);
        Serial.print(F("LevelSensor[6]= "));Serial.println(LevelSensor[6]);
        Serial.print(F("LevelSensor[7]= "));Serial.println(LevelSensor[7]);
        Serial.print(F("LevelSensor[8]= "));Serial.println(LevelSensor[8]);
        Serial.print(F("LevelSensor[9]= "));Serial.println(LevelSensor[9]);
        Serial.print(F("[GR] CAPDAC_A=0x")); Serial.print(GrenobleCapDacA, HEX);
        Serial.print(F(" (en=")); Serial.print((GrenobleCapDacA & 0x80) ? 1 : 0);
        Serial.print(F(" code=")); Serial.print(GrenobleCapDacA & 0x3F);
        Serial.print(F(")  "));

        //Serial.print(F("CAPDAC_B=0x")); Serial.print(GrenobleCapDacB, HEX);
        //Serial.print(F(" (en=")); Serial.print((GrenobleCapDacB & 0x80) ? 1 : 0);
        //Serial.print(F(" code=")); Serial.print(GrenobleCapDacB & 0x3F);
        //Serial.println(F(")"));

        //Serial.print(F("[GR] CIN1 raw=0x")); printHex24(GrenobleRaw[0]);
        //Serial.print(F(" rel_pF=")); Serial.print(GrenobleRel[0], 5);
        //Serial.print(F(" abs_pF=")); Serial.println(GrenobleLevel[0], 5);

        //Serial.print(F("[GR] CIN2 raw=0x")); printHex24(GrenobleRaw[1]);
        //Serial.print(F(" rel_pF=")); Serial.print(GrenobleRel[1], 5);
        //Serial.print(F(" abs_pF=")); Serial.println(GrenobleLevel[1], 5);

        break;
      case 'D':
        Serial.println(F("Enabling Dewar Sensor DAQ"));
        dewarDAQ = 1;
        DAQrate = fastDAQ;
        break;
      case 'd':
        Serial.println(F("Disabling Dewar Sensor DAQ"));
        dewarDAQ = 0;
        DAQrate = slowDAQ;
        break;
      case 'B':
        Serial.println(F("Enabling Chamber Sensor DAQ"));
        chamberDAQ = 1;
        DAQrate = fastDAQ;
        break;
      case 'b':
        Serial.println(F("Disabling Chamber Sensor DAQ"));
        chamberDAQ = 0;
        DAQrate = slowDAQ;
        break;
      case 'G':
        Serial.println(F("Enabling General DAQ"));
        allDAQ = 1;
        DAQrate = fastDAQ;
        break;
      case 'g':
        Serial.println(F("Disabling General DAQ"));
        allDAQ = 0;
        DAQrate = slowDAQ;
        break;

case 'A':  // Autofill diagnostic
  Serial.println(F("=== AUTOFILL DIAGNOSTIC ==="));
  Serial.print(F("STATE: ")); Serial.println(STATE);
  Serial.print(F("Dewar_Level: ")); Serial.println(Dewar_Level);
  Serial.print(F("LiquidCMD pin state: ")); Serial.println(digitalRead(LiquidCMD));
  Serial.print(F("ExValveCMD2 pin state: ")); Serial.println(digitalRead(ExValveCMD2));
  Serial.print(F("millis(): ")); Serial.println(millis());
  Serial.print(F("lastMillis: ")); Serial.println(lastMillis);
  Serial.print(F("Time since last: ")); Serial.println(millis() - lastMillis);
  Serial.print(F("shortCycleTime: ")); Serial.println(shortCycleTime);
  break;

      case 'T':
        Serial.println(F("Running Grenoble CapDAC tune..."));
        //TuneGrenobleCapDAC();
        break;

      case 'v':
        valveStr = "";
        while(Serial.available() > 0) {
          //Empty out buffer
          valvePct = Serial.read();
        }
        Serial.println(F("Input value for valve pct:"));
        while(Serial.available() <= 0) {
          //Wait for input
        }

        while(Serial.available() > 0) {
          //Wait for input
          Serial.println(Serial.available());
          valvePct = Serial.read();
          //Serial.println(valvePct);
          if(valvePct != '\n' || valvePct != '\r') {
            valveStr += valvePct;
          }
        }
        //Serial.println(valveStr);
        valveFlo = valveStr.toFloat();
        //Serial.println(valveFlo);
        valveCounts1 = valveFlo / 100;
        //Serial.println(valveCounts1);
        valveCounts2 = valveCounts1 * 255;
        //Serial.println(valveCounts2);
        break;
      case '\r':
        break;
      case '\n':
        break;
      case 'R':
      case 'r':
        Serial.println(F("[CMD] Grenoble register dump requested..."));
        ad7746_dumpRegisters();
        break;
      case 'P':
      case 'p':
        Serial.print(F("[CMD] Grenoble debug printing "));
        grenoble_print = !grenoble_print;
        Serial.println(grenoble_print ? "ENABLED" : "DISABLED");
        break;
      default:
        Serial.println(F("State: 0=FAULT, 1=IDLE, 2=AFILL, 3=WGAS, 4=IMER, 5=CGAS, 6=WARM"));
        Serial.println(F("S or s to set Sanity checking"));
        Serial.println(F("M to report sensor values"));
        Serial.println(F("X to enter setup routine"));
        Serial.println(F("B or b to set Basin DAQ"));
        Serial.println(F("D or d to set Dewar DAQ"));
        Serial.println(F("G or g to toggle General DAQ"));
        Serial.println(F("R or r to dump Grenoble AD7746 registers"));
        Serial.println(F("P or p to toggle Grenoble debug printing"));
        Serial.println(F("W to run ROAH selector wiring self-test"));
        Serial.println(F("F to force CGAS soak gate complete (dry-run bypass)"));
        break;
    }      
  } // end of the process command section

  //Get_Lid_Temperature();

  Read_Level_Sensors();  // sets TC_Level and can set STATE = 0 and Err_Code if there is a fault

  if (allDAQ == 1 || dewarDAQ == 1 || chamberDAQ == 1) {
    if (GRENOBLE_ENABLED) {
      GrenobleReadout_Raw();
    }
  }


  Get_SD_Pressure(); // sets Pressure and can set STATE = 0 and Err_Code if over-pressure detected

  dewarPress = pressurePSI(Pressure[0]);

// Begin of a state machine with five states:
// STATE 0 is a system fault
// STATE 1 is IDLE
// STATE 2 is AUTOFILL
// STATE 3 is WARM Gas
// STATE 4 is Immerse the DUT

  if(chamberDAQ == 1) {
    Serial.print(F("millis:")); Serial.print(millis()); Serial.print(F(","));
    Serial.print(F("LevelSensor[5]:"));Serial.print(LevelSensor[5]);Serial.print(F(","));
    Serial.print(F("LevelSensor[6]:"));Serial.print(LevelSensor[6]);Serial.print(F(","));
    Serial.print(F("LevelSensor[7]:"));Serial.print(LevelSensor[7]);Serial.print(F(","));
    Serial.print(F("LevelSensor[8]:"));Serial.print(LevelSensor[8]);Serial.print(F(","));
    Serial.print(F("LevelSensor[9]:"));Serial.print(LevelSensor[9]);Serial.print(F(","));
    Serial.print(F("Grenoble1:"));Serial.print(GrenobleLevel[0], 5);Serial.print(F(","));
    Serial.print(F("Grenoble2:"));Serial.println(GrenobleLevel[1], 5);
  }

  if(dewarDAQ == 1) {
    Serial.print(F("millis:")); Serial.print(millis()); Serial.print(F(","));
    Serial.print(F("LevelSensor[0]:"));Serial.print(LevelSensor[0]);Serial.print(F(","));
    Serial.print(F("LevelSensor[1]:"));Serial.print(LevelSensor[1]);Serial.print(F(","));
    Serial.print(F("LevelSensor[2]:"));Serial.print(LevelSensor[2]);Serial.print(F(","));
    Serial.print(F("LevelSensor[3]:"));Serial.print(LevelSensor[3]);Serial.print(F(","));
    Serial.print(F("LevelSensor[4]:"));Serial.print(LevelSensor[4]);Serial.print(F(","));
    //    Serial.print(F("Pressure1:"));Serial.print(dewarPress);Serial.print(F(","));
    //    Serial.print(F("Pressure2:"));Serial.println(Pressure[1]);
    Serial.print(F("Pressure1_psi:")); Serial.print(pressurePSI(Pressure[0]), 3); Serial.print(F(","));
    Serial.print(F("Pressure2_psi:")); Serial.print(pressurePSI(Pressure[1]), 3); Serial.print(F(","));
    // Keep raw counts available for diagnostics:
    Serial.print(F("Pressure1_raw:")); Serial.print(Pressure[0]); Serial.print(F(","));
    Serial.print(F("Pressure2_raw:")); Serial.print(Pressure[1]); Serial.print(F(","));
  }

  if(allDAQ == 1) {
    Serial.print(F("millis:")); Serial.print(millis()); Serial.print(F(","));
    Serial.print(F("LevelSensor[0]:"));Serial.print(LevelSensor[0]);Serial.print(F(","));
    Serial.print(F("LevelSensor[1]:"));Serial.print(LevelSensor[1]);Serial.print(F(","));
    Serial.print(F("LevelSensor[2]:"));Serial.print(LevelSensor[2]);Serial.print(F(","));
    Serial.print(F("LevelSensor[3]:"));Serial.print(LevelSensor[3]);Serial.print(F(","));
    Serial.print(F("LevelSensor[4]:"));Serial.print(LevelSensor[4]);Serial.print(F(","));
    Serial.print(F("LevelSensor[5]:"));Serial.print(LevelSensor[5]);Serial.print(F(","));
    Serial.print(F("LevelSensor[6]:"));Serial.print(LevelSensor[6]);Serial.print(F(","));
    Serial.print(F("LevelSensor[7]:"));Serial.print(LevelSensor[7]);Serial.print(F(","));
    Serial.print(F("LevelSensor[8]:"));Serial.print(LevelSensor[8]);Serial.print(F(","));
    Serial.print(F("LevelSensor[9]:"));Serial.print(LevelSensor[9]);Serial.print(F(","));
    //    Serial.print(F("Pressure1:"));Serial.print(dewarPress);Serial.print(F(","));
    //    Serial.print(F("Pressure2:"));Serial.print(Pressure[1]);Serial.print(F(","));
    Serial.print(F("Pressure1_psi:")); Serial.print(pressurePSI(Pressure[0]), 3); Serial.print(F(","));
    Serial.print(F("Pressure2_psi:")); Serial.print(pressurePSI(Pressure[1]), 3); Serial.print(F(","));
    // Keep raw counts available for diagnostics:
    Serial.print(F("Pressure1_raw:")); Serial.print(Pressure[0]); Serial.print(F(","));
    Serial.print(F("Pressure2_raw:")); Serial.print(Pressure[1]); Serial.print(F(","));

    Serial.print(F("Grenoble1_abs:")); Serial.print(GrenobleLevel[0], 6); Serial.print(F(","));
    Serial.print(F("Grenoble1_raw:")); Serial.print(GrenobleRaw[0]);      Serial.print(F(","));
    Serial.print(F("Grenoble2_abs:")); Serial.print(GrenobleLevel[1], 6); Serial.print(F(","));
    Serial.print(F("Grenoble2_raw:")); Serial.println(GrenobleRaw[1]);

  }

  if (STATE == 0){ // SYSTEM FAULT!!!
    digitalWrite(DewarPInterlock, LOW);
    digitalWrite(DewarLInterlock, LOW);
    digitalWrite(BasinLInterlock, LOW); 
    digitalWrite(LiquidCMD, LOW);     //LQ Vlv
    digitalWrite(ExValveCMD1, LOW);   //CGas 1
    digitalWrite(ExValveCMD2, LOW);   //CGas 2
    digitalWrite(ExValveCMD3, LOW);   //WGas 1
    digitalWrite(WarmGasCMD, LOW);    //WGas 2
    digitalWrite(EnableLid, LOW);
   
    while(STATE == 0){
     // lcd.begin(16,2); // safety catch in case of corrupted LCD.  This seems to reset LCD...
      lcd.setCursor(0, 0);
      lcd.print(F("SYSTEM FAULT!!!!"));
      Serial.print(F("SYSTEM FAULT ="));
      Serial.println(Err_Code);
      delay(1500);
      lcd.setCursor(0, 0);
      if(Err_Code == 1) lcd.print(F("Chamber Overfill"));  
      if(Err_Code == 2) lcd.print(F("Over Pressure   "));  
      if(Err_Code == 3) lcd.print(F("Sensor short ckt"));  
      if(Err_Code == 4) lcd.print(F("Sensor open ckt "));  
      if(Err_Code == 5) lcd.print(F("Empty Dewar!"));  
      //if(Err_Code == 6) lcd.print(F("S1 not immersed!"));  
      if(Err_Code == 7) lcd.print(F("Dewar Overfilled"));  
      lcd.setCursor(0, 1);
      lcd.print(F("RESET: MODE DOWN"));  
      delay(500);
      if (digitalRead(ModeDown)==LOW){
        lcd.setCursor(0, 1);
        lcd.print(F("Hold Switch 5 s "));   
        delay(5000);
        if (digitalRead(ModeDown)==LOW) {
          setup();
        } 
      }
    }    
  }

  if (STATE == 1){ // IDLE - only NO vent open; NC vents closed

    digitalWrite(LiquidCMD, LOW);     // LN line valve closed
    // SD vent valves: 1 NO + 2 NC. Per v18d operator preference, ONLY
    // the NO vent is open during IDLE - the two NC vents stay closed
    // for a controlled single-path drain.
    digitalWrite(ExValveCMD1, LOW);   // NO vent de-energized = OPEN
    digitalWrite(ExValveCMD2, LOW);   // NC vent de-energized = CLOSED
    digitalWrite(ExValveCMD3, LOW);   // NC vent de-energized = CLOSED
    digitalWrite(WarmGasCMD, LOW);    // Basin warm gas off
    digitalWrite(EnableLid, LOW);     // Lid heater off
    SetDac(0);                         // GN modulating valve closed

    // HAND mode: ModeDown advances IDLE -> WGAS (start basin sequence).
    // ModeUp from IDLE is a no-op (no further retreat).
    // AFILL is no longer reachable from HAND mode - operator must switch
    // HOAR to AUTO and press START AUTOFILL.
    if (HandControlsAllowed()) {
      if (digitalRead(ModeDown)==LOW) {
        SetState(3);   // -> WGAS
      }
    }
  }

  if(STATE == 2) {    //Autofill SD

    digitalWrite(ExValveCMD1, LOW);   //CGas 1
    digitalWrite(ExValveCMD3, LOW);   //WGas 1
    digitalWrite(WarmGasCMD, LOW);    //WGas 2
    digitalWrite(EnableLid, LOW);
    //analogWrite(LED_Immerse, 0);
    SetDac(0);

  // DEBUG: Print status every 100 loops
  static int debugCounter = 0;
  debugCounter++;
  if (debugCounter >= 100) {
    debugCounter = 0;
    Serial.print(F("[AUTOFILL] LS0=")); Serial.print(LevelSensor[0]);
    Serial.print(F(" LS1=")); Serial.print(LevelSensor[1]);
    Serial.print(F(" Dewar_Level=")); Serial.print(Dewar_Level);
    Serial.print(F(" Pressure=")); Serial.print(Pressure[0]);
    Serial.print(F(" LiquidCMD=")); Serial.print(digitalRead(LiquidCMD));
    Serial.print(F(" latched=")); Serial.println(autofill_full_latched ? 1 : 0);
  }

    // ====================================================================
    // DUAL-TIMER FULL/OVERFILL CUTOFF WITH SATURATION CEILING
    // ====================================================================
    // Independent debounced timers on LS1 (Full) and LS0 (Overfill).
    // Each timer is "in-window" only when the sensor reads ABOVE its
    // immersed threshold AND BELOW the saturation ceiling. The ceiling
    // rejects the room-temperature saturation condition (~32767) where
    // the warm diode sensor reads at full ADC scale and would otherwise
    // be misread as "very wet". A timer resets to 0 the moment its
    // sensor leaves the window in either direction.
    //
    // The valve closes only when BOTH timers have been continuously
    // running long enough (LS1 in-window >= 10 s AND LS0 in-window >= 5 s).
    // Once latched, brief boil-off below LS0 does not re-open the valve.
    unsigned long now_ms = millis();

    // LS1 (Dewar Full) timer maintenance
    bool ls1_in_window = (LevelSensor[1] > LS1_FULL_THRESHOLD) &&
                         (LevelSensor[1] < SATURATION_CEILING);
    if (ls1_in_window) {
      if (ls1_full_first_ms == 0) {
        ls1_full_first_ms = now_ms;
        if (ls1_full_first_ms == 0) ls1_full_first_ms = 1;  // avoid 0 sentinel collision
        Serial.println(F("[AUTOFILL] LS1 in-window - starting 10s timer"));
      }
    } else {
      if (ls1_full_first_ms != 0) {
        if (LevelSensor[1] >= SATURATION_CEILING) {
          Serial.print(F("[AUTOFILL] LS1 saturated (warm), reading="));
          Serial.print(LevelSensor[1]);
          Serial.println(F(" - LS1 timer reset"));
        } else {
          Serial.println(F("[AUTOFILL] LS1 dropped below threshold - LS1 timer reset"));
        }
      }
      ls1_full_first_ms = 0;
    }

    // LS0 (Dewar Overfill) timer maintenance
    bool ls0_in_window = (LevelSensor[0] > LS0_OVERFILL_THRESHOLD) &&
                         (LevelSensor[0] < SATURATION_CEILING);
    if (ls0_in_window) {
      if (ls0_overfill_first_ms == 0) {
        ls0_overfill_first_ms = now_ms;
        if (ls0_overfill_first_ms == 0) ls0_overfill_first_ms = 1;
        Serial.println(F("[AUTOFILL] LS0 in-window - starting 5s timer"));
      }
    } else {
      if (ls0_overfill_first_ms != 0) {
        if (LevelSensor[0] >= SATURATION_CEILING) {
          Serial.print(F("[AUTOFILL] LS0 saturated (warm), reading="));
          Serial.print(LevelSensor[0]);
          Serial.println(F(" - LS0 timer reset"));
        } else {
          Serial.println(F("[AUTOFILL] LS0 dropped below threshold - LS0 timer reset"));
        }
      }
      ls0_overfill_first_ms = 0;
    }

    // Evaluate both timers (only if not already latched)
    if (!autofill_full_latched) {
      bool ls1_satisfied = (ls1_full_first_ms != 0) &&
                           ((now_ms - ls1_full_first_ms) >= LS1_FULL_HOLD_MS);
      bool ls0_satisfied = (ls0_overfill_first_ms != 0) &&
                           ((now_ms - ls0_overfill_first_ms) >= LS0_OVERFILL_HOLD_MS);

      if (ls1_satisfied && ls0_satisfied) {
        autofill_full_latched = true;
        Serial.println(F("[AUTOFILL] Cutoff condition met - LS1>=10s AND LS0>=5s in-window. Latching valves CLOSED."));
        // Drive the valves LOW now (the block below would also do this,
        // but SetState(1) below clears STATE so make the safe action
        // explicit and unconditional first).
        digitalWrite(LiquidCMD, LOW);
        digitalWrite(ExValveCMD2, LOW);
        // Auto-transition to IDLE so the panel LED/LCD clearly indicates
        // the fill is complete. The IDLE handler will reaffirm valves LOW.
        Serial.println(F("[AUTOFILL] Returning to IDLE."));
        SetState(1);
      }
    }

    // Only run the rest of the autofill-specific logic if we are still
    // in STATE 2. The auto-transition above changes STATE to 1, in which
    // case the valve-control and mode-button blocks below would fire on
    // stale assumptions (e.g. a held ModeUp could immediately advance
    // out of the just-entered IDLE).
    if (STATE == 2) {
      if (autofill_full_latched) {
        digitalWrite(LiquidCMD, LOW);     //LQ Vlv CLOSED
        digitalWrite(ExValveCMD2, LOW);   //Close NC Vents
      }
      else {                              //Open transfer dewar fill valve
        digitalWrite(LiquidCMD, HIGH);    //LQ Vlv OPEN
        digitalWrite(ExValveCMD2, HIGH);  //Open NC Vents
      }

      // AFILL is AUTO-mode-only as of v17. The previous HAND-mode
      // ModeUp/ModeDown hooks have been removed. To abort an autofill,
      // operator must use AUTO mode + STOP SEQUENCE button. Normal
      // completion is the dual-timer cutoff -> IDLE auto-transition.
    }
    
  }


  else if (STATE == 3){ //warm gas

    digitalWrite(LiquidCMD, LOW);     //LQ Vlv
    digitalWrite(ExValveCMD1, LOW);   //CGas 1
    digitalWrite(ExValveCMD2, LOW);   //CGas 2
    digitalWrite(ExValveCMD3, LOW);   //WGas 1
    SetDac(0);
    //analogWrite(LED_Immerse, 0);

    if(TC_Level == 0) { // Desired operating point; Lid on, Warm gas to basin on, vent open
      digitalWrite(EnableLid, HIGH);      
      digitalWrite(WarmGasCMD, HIGH);   //WGas 2   
      // message: Warming
    }
    else { // if not at Level 0, wait in idle state until this is the case
      digitalWrite(EnableLid, LOW); 
      digitalWrite(WarmGasCMD, LOW);    //WGas 2 
      // message: waiting for chamber to drain
    }
          
    if (HandControlsAllowed()) {
      if (digitalRead(ModeDown)==LOW) {
        SetState(5);   // -> CGAS (advance)
      }
      else if (digitalRead(ModeUp)==LOW) {
        SetState(1);   // -> IDLE (retreat)
      }
    }
  }

  //Not using Cold Gas for sequence
  /*
  else if (STATE == 3){ // cold gas
    //digitalWrite (TC_HEAT,Relay_Off); // TC heat always off in this mode

    if(TC_Level == 0){ //need to raise LN2 in chamber: Fans ON, if pressure OK then vent closed and SD heat on
    //digitalWrite (TC_FANS,Relay_On);
    if (Pressure < P_Limit2) {
      ValveClose();
    }
    else {
      ValveOpen();
    }
    Set_LCD_Color(VIOLET);
    // message: waiting for LN2 to enter chamber
    }

    if(TC_Level == 1){ // desired operating point: Fans on, SD heat off, if pressure OK then vents closed
    //digitalWrite (TC_FANS,Relay_On);
    //digitalWrite (SD_HEAT,Relay_Off);  
    if (Pressure < P_Limit2) ValveClose();
    else ValveOpen();  
    Set_LCD_Color(WHIT);
    // message: Cooling  
    }
    
    if(TC_Level == 2){ // LN2 above L2 sensor: Fans ON, SD heat off, vents open
    //digitalWrite (TC_FANS,Relay_On);
    //digitalWrite (VENT_NC,Relay_On);
    ValveOpen();
    //digitalWrite (SD_HEAT,Relay_Off);      
    Set_LCD_Color(WHIT);
    // message: Cooling and venting
    }

    if(TC_Level >= 3){  // LN2 is too high: Fans off, SD heat off, vents open
    //digitalWrite (TC_FANS,Relay_Off);
    //digitalWrite (VENT_NC,Relay_On);
    ValveOpen();
    //digitalWrite (SD_HEAT,Relay_Off);
    Set_LCD_Color(VIOLET);
    // message: waiting for chamber to drain    
    }
    
    if (digitalRead(ModeUp)==LOW) SetState(4);
    else if (digitalRead(ModeDown)==LOW) SetState(2);
    
  }
  */
  
  // ============================================================================
  // STATE 5: CGAS - VERY GRADUAL FILL TO LS9 (0.5") + 3-MIN COOL-DOWN SOAK
  // ============================================================================
  // CGAS replaces what was previously stages 0+1 of the old IMER state.
  // Fill the basin slowly, close SD vents to build pressure, modulate
  // the GN drive valve (DAC) based on LS9 status. Soak for 3 minutes
  // after LS9 first becomes immersed - the operator may not advance to
  // IMER until the soak gate is satisfied.
  //
  // Pressure protection from the original IMER state is preserved.
  else if (STATE == 5) { // CGAS - cold-gas gradual fill to 0.5"
    digitalWrite(LiquidCMD, LOW);      // LN line valve closed; flow via modulating valve
    digitalWrite(ExValveCMD3, LOW);    // NC vent closed (de-energized)
    digitalWrite(WarmGasCMD, LOW);     // Basin warm gas off
    digitalWrite(EnableLid, LOW);      // Lid heater off (chamber is being cooled)

    bool pressure_limiting = false;

    // ---- Pressure-based safety (vent opens ONLY in CRITICAL zone) ----
    if (Pressure[0] > 10000) {                 // Critical zone
      digitalWrite(ExValveCMD1, LOW);         // OPEN NO vent (true safety release)
      pressure_limiting = true;
      static unsigned long last_crit_msg = 0;
      if (millis() - last_crit_msg > 5000) {
        Serial.print(F("[CGAS] CRITICAL PRESSURE: "));
        Serial.print(Pressure[0]);
        Serial.println(F(" - STOPPED, venting"));
        last_crit_msg = millis();
      }
    }
    else if (Pressure[0] > 9376) {             // Warning zone: throttle only, NO vent
      pressure_limiting = true;
      digitalWrite(ExValveCMD1, HIGH);         // vent CLOSED (v18c: was LOW)
    }
    else if (Pressure[0] > 8500) {             // Caution zone
      pressure_limiting = true;
      digitalWrite(ExValveCMD1, HIGH);         // vent CLOSED
    }
    else {                                      // Normal zone
      digitalWrite(ExValveCMD1, HIGH);         // vent CLOSED (build SD pressure)
    }
    // ExValveCMD2 and ExValveCMD3 are the two NC vents - kept LOW
    // (de-energized = CLOSED) throughout CGAS so SD pressure can build.
    digitalWrite(ExValveCMD2, LOW);
    digitalWrite(ExValveCMD3, LOW);

    // ---- LS9-based GN modulation target (then ramped) ----
    // Target DAC value depending on basin LS9 status:
    //   LS9 < 2  (basin very dry):       40 - full rate
    //   LS9 == 2 (rising):               30 - gentle reduction
    //   LS9 == 3 (close to immersion):   15 - gentle approach
    //   LS9 == 4 (immersed):              0 - hold
    int valve_target = 0;
    if (!pressure_limiting || Pressure[0] < 10000) {
      if (LevelStatus[9] < 2) {
        valve_target = VALVE_STARTUP;              // 40
      }
      else if (LevelStatus[9] == 2) {
        valve_target = (VALVE_STARTUP * 3) / 4;    // 30
      }
      else if (LevelStatus[9] == 3) {
        valve_target = (VALVE_STARTUP * 3) / 8;    // 15
      }
      else if (LevelStatus[9] == 4) {
        // LS9 immersed - record first-immersion timestamp for soak gate
        if (cgas_ls9_first_immersed_ms == 0) {
          cgas_ls9_first_immersed_ms = millis();
          if (cgas_ls9_first_immersed_ms == 0) cgas_ls9_first_immersed_ms = 1; // sentinel-safe
          Serial.println(F("[CGAS] LS9 immersed - 3 min soak timer started"));
        }
        valve_target = 0;
      }

      // Apply pressure-based reduction to target (preserved from original IMER)
      if (Pressure[0] > 9376) valve_target = valve_target / 4;
      else if (Pressure[0] > 8500) valve_target = valve_target / 2;
    }
    // If pressure_limiting (critical zone), valve_target stays 0.

    // ---- Ramp current DAC toward target ----
    // Ramp UP at +1 per CGAS_DAC_RAMP_STEP_MS (smooth pressure build).
    // Ramp DOWN immediately (safety always backs off promptly).
    if (valve_target < cgas_dac_current) {
      cgas_dac_current = valve_target;
    } else if (valve_target > cgas_dac_current) {
      if (millis() - cgas_dac_last_step_ms >= CGAS_DAC_RAMP_STEP_MS) {
        cgas_dac_current++;
        cgas_dac_last_step_ms = millis();
      }
    }
    SetDac(cgas_dac_current);

    // Periodic ramp status print (every ~5 s) so operator can see progress
    static unsigned long last_ramp_msg = 0;
    if (millis() - last_ramp_msg > 5000) {
      Serial.print(F("[CGAS] DAC="));
      Serial.print(cgas_dac_current);
      Serial.print(F(" target="));
      Serial.print(valve_target);
      Serial.print(F(" LS9="));
      Serial.print(LevelStatus[9]);
      Serial.print(F(" P="));
      Serial.println(Pressure[0]);
      last_ramp_msg = millis();
    }

    // ---- Soak gate ----
    if (cgas_ls9_first_immersed_ms != 0 && !cgas_soak_complete) {
      unsigned long elapsed = millis() - cgas_ls9_first_immersed_ms;
      if (elapsed >= CGAS_SOAK_MS) {
        cgas_soak_complete = true;
        Serial.println(F("[CGAS] 3-min soak complete - operator may advance to IMER"));
      }
      // Periodic soak progress message
      static unsigned long last_soak_msg = 0;
      if (millis() - last_soak_msg > 30000) {
        Serial.print(F("[CGAS] Soak: "));
        Serial.print(elapsed / 1000);
        Serial.print(F("s / "));
        Serial.print(CGAS_SOAK_MS / 1000);
        Serial.print(F("s, P="));
        Serial.println(Pressure[0]);
        last_soak_msg = millis();
      }
    }

    // ---- Basin overfill protection ----
    if (LevelStatus[5] == 4) {
      Serial.println(F("[CGAS] ERROR: Basin overfill!"));
      Err_Code = 1;
      SetState(0);
    }

    // ---- HAND mode mode-buttons ----
    if (HandControlsAllowed()) {
      if (digitalRead(ModeDown)==LOW) {
        // ModeDown = advance forward through basin sequence (gated by soak)
        if (cgas_soak_complete) {
          SetState(4);   // -> IMER
        } else {
          // Soak not yet complete - reject the advance with a short serial
          // note AND a single 500 ms horn blip. Rate-limited to one per 2 s
          // via last_reject so a held button does not chain blips.
          static unsigned long last_reject = 0;
          if (millis() - last_reject > 2000) {
            Serial.println(F("[CGAS] Cannot advance to IMER - 3-min soak still in progress"));
            Horn_ShortBlip();
            last_reject = millis();
          }
        }
      }
      else if (digitalRead(ModeUp)==LOW) {
        SetState(3);   // -> WGAS (retreat)
      }
    }
  }

  // ============================================================================
  // STATE 4: IMER - GRADUAL FILL TO LS6 (~5"), HOLD AT OPERATING LEVEL
  // ============================================================================
  // IMER takes over from CGAS once the soak gate is satisfied. Fill the
  // basin to LS6 (5"), hold there as the final operating level. Old
  // stages 2+3 of the previous IMER implementation; renumbered to 0+1
  // here so immerse_stage starts from 0 in this state.
  else if (STATE == 4){ //Immerse
    digitalWrite(LiquidCMD, LOW);      // LN line valve closed; flow via modulating valve
    digitalWrite(ExValveCMD2, LOW);    // SD vent closed (build SD pressure)
    digitalWrite(ExValveCMD3, LOW);    // WGas 1 closed
    digitalWrite(WarmGasCMD, LOW);
    digitalWrite(EnableLid, LOW);
    
    // ========================================================================
    // PRESSURE-BASED SAFETY CONTROL (applies to all stages)
    // ========================================================================
    int valve_command = 0;
    bool pressure_limiting = false;
    
    if (Pressure[0] > 10000) {  // Critical zone
      valve_command = 0;
      digitalWrite(ExValveCMD1, LOW);  // OPEN vent
      pressure_limiting = true;
      static unsigned long last_crit_msg = 0;
      if (millis() - last_crit_msg > 5000) {
        Serial.print(F("[IMMERSE] CRITICAL PRESSURE: "));
        Serial.print(Pressure[0]);
        Serial.println(F(" - STOPPED, venting"));
        last_crit_msg = millis();
      }
    }
    else if (Pressure[0] > 9376) {  // Warning zone: throttle only, vent stays CLOSED
      pressure_limiting = true;
      digitalWrite(ExValveCMD1, HIGH);    // vent CLOSED (v18c: was LOW)
      static unsigned long last_warn_msg = 0;
      if (millis() - last_warn_msg > 10000) {
        Serial.print(F("[IMMERSE] WARNING PRESSURE: "));
        Serial.print(Pressure[0]);
        Serial.println(F(" - reducing flow"));
        last_warn_msg = millis();
      }
    }
    else if (Pressure[0] > 8500) {  // Caution zone
      pressure_limiting = true;
      digitalWrite(ExValveCMD1, HIGH);
    }
    else {
      digitalWrite(ExValveCMD1, HIGH);
    }
    
    // ========================================================================
    // STAGE-BASED FILLING CONTROL (if not pressure-limited)
    // ========================================================================
    if (!pressure_limiting || Pressure[0] < 10000) {
      
      switch (immerse_stage) {
        
        // ====================================================================
        // STAGE 0 (renumbered): Gradual fill from LS9 (already wet) to LS6 (~5")
        // ====================================================================
        case 0:
          if (LevelStatus[8] < 4) {
            valve_command = VALVE_STARTUP;  // 40
          }
          else if (LevelStatus[7] < 4) {
            if (LevelStatus[7] == 3) {
              valve_command = VALVE_STARTUP / 2;  // 20
            } else {
              valve_command = VALVE_STARTUP;  // 40
            }
          }
          else if (LevelStatus[6] < 4) {
            if (LevelStatus[6] == 3) {
              valve_command = VALVE_STARTUP / 4;  // 10
            } else if (LevelStatus[6] == 2) {
              valve_command = VALVE_STARTUP / 2;  // 20
            }
          }
          else if (LevelStatus[6] == 4) {
            Serial.println(F("[IMMERSE] Stage 0 complete - LS6 reached"));
            immerse_stage = 1;
            valve_command = VALVE_STARTUP / 8;  // 5
          }
          break;
        
        // ====================================================================
        // STAGE 1 (renumbered): Hold at LS6 (final operating level)
        // ====================================================================
        case 1:
          if (LevelStatus[6] == 2) {
            valve_command = VALVE_STARTUP / 2;  // 20
          }
          else if (LevelStatus[6] == 3) {
            valve_command = VALVE_STARTUP / 4;  // 10
          }
          else if (LevelStatus[6] == 4) {
            if (LevelStatus[5] == 4) {
              valve_command = 0;  // Stop - level too high (LS5 immersed)
            } else {
              valve_command = VALVE_STARTUP / 8;  // 5 - trickle to maintain
            }
          }
          break;
      }
      
      // ====================================================================
      // APPLY PRESSURE-BASED REDUCTION
      // ====================================================================
      if (Pressure[0] > 9376) {  // Warning zone
        valve_command = valve_command / 4;  // Reduce by 75%
      }
      else if (Pressure[0] > 8500) {  // Caution zone
        valve_command = valve_command / 2;  // Reduce by 50%
      }
      
      SetDac(valve_command);
    }
    
    // ========================================================================
    // SAFETY CHECKS
    // ========================================================================
    
    // Basin overfill check (single-sensor: LS5 immersed = above intended range)
    if (LevelStatus[5] == 4) {
      Serial.println(F("[IMMERSE] ERROR: Basin overfill!"));
      Err_Code = 1;
      SetState(0);
    }

    // HAND mode mode-buttons (v17: ModeDown advances, ModeUp retreats):
    //   ModeDown -> WARM (advance: data-taking complete, dry out chamber)
    //   ModeUp   -> CGAS (retreat)
    if (HandControlsAllowed()) {
      if (digitalRead(ModeDown)==LOW) {
        immerse_stage = 0;
        SetState(6);   // -> WARM (advance)
      }
      else if (digitalRead(ModeUp)==LOW) {
        immerse_stage = 0;
        SetState(5);   // -> CGAS (retreat)
      }
    }
  }

  // ============================================================================
  // STATE 6: WARM - DRAIN (idle-like) + WGAS-equivalent dry-out
  // ============================================================================
  // Phase 0 (drain): all valves in default state (same as IDLE). LN2 boils
  //                  off naturally; we wait for LS9 to be dry (LS9 != 4).
  //                  No active heating during drain.
  // Phase 1 (dry-out): WGAS pattern verbatim - lid heater + warm gas + fans
  //                    when TC_Level == 0, otherwise wait. Operator advances
  //                    WARM -> IDLE manually (ModeDown) when ready.
  else if (STATE == 6) { // WARM

    if (warm_phase == 0) {
      // ---- Drain phase: ALL THREE SD vents OPEN for fast drainage ----
      // v19: more aggressive venting than IDLE so residual LN2 boils off
      // faster before phase 1 (dry-out) begins.
      digitalWrite(LiquidCMD, LOW);
      digitalWrite(ExValveCMD1, LOW);    // NO vent OPEN
      digitalWrite(ExValveCMD2, HIGH);   // NC vent energized = OPEN (v19)
      digitalWrite(ExValveCMD3, HIGH);   // NC vent energized = OPEN (v19)
      digitalWrite(WarmGasCMD, LOW);
      digitalWrite(EnableLid, LOW);
      SetDac(0);

      if (LevelStatus[9] != 4) {
        Serial.println(F("[WARM] Drain phase complete - LS9 dry, switching to dry-out"));
        warm_phase = 1;
      } else {
        static unsigned long last_drain_msg = 0;
        if (millis() - last_drain_msg > 30000) {
          Serial.println(F("[WARM] Drain phase: waiting for LN2 to evaporate below LS9"));
          last_drain_msg = millis();
        }
      }
    }
    else {
      // ---- Dry-out phase: WGAS pattern ----
      digitalWrite(LiquidCMD, LOW);
      digitalWrite(ExValveCMD1, LOW);
      digitalWrite(ExValveCMD2, LOW);
      digitalWrite(ExValveCMD3, LOW);
      SetDac(0);

      if (TC_Level == 0) { // chamber drained, ready to warm
        digitalWrite(EnableLid, HIGH);
        digitalWrite(WarmGasCMD, HIGH);
      } else {              // wait for chamber drain
        digitalWrite(EnableLid, LOW);
        digitalWrite(WarmGasCMD, LOW);
      }
    }

    // HAND mode mode-buttons (v17: ModeDown advances, ModeUp no-op):
    //   ModeDown -> IDLE (advance / cycle complete)
    //   ModeUp   -> no-op (no useful retreat from WARM; if abort needed,
    //               operator should switch HOAR to AUTO + STOP SEQUENCE)
    if (HandControlsAllowed()) {
      if (digitalRead(ModeDown)==LOW) {
        SetState(1);   // -> IDLE (cycle complete)
      }
    }
  }


  // This marks the end of the state machine
  // Now cycle through messages on the LCD display

  MessCntr++; // a simple counter for doling out user messages

  if (MessCntr == 3){
    //lcd.begin(16,2); // safety catch in case of corrupted LCD.  This seems to reset LCD...
    lcd.clear();
    lcd.print(F("Chamber Level: "));
    lcd.print(TC_Level);
    lcd.setCursor(0, 1);
    lcd.print(F("Pressure:  "));
    lcd.print(Pressure[1]);
  }

  if (MessCntr == 6){
    lcd.clear();
    lcd.print(F("Dewar full=1300:"));
    lcd.setCursor(0,1);
    lcd.print(LevelSensor[1]-25400);
  }


  if (MessCntr == 9){
    lcd.setCursor(0, 0);
    lcd.print(F("S1 counts:      "));
    lcd.setCursor(11, 0);
    lcd.print(LevelSensor[0]);
  
    lcd.setCursor(0, 1);
    lcd.print(F("S2 counts:      "));
    lcd.setCursor(11, 1);
    lcd.print(LevelSensor[1]);
  }

  if (MessCntr == 12){
    lcd.setCursor(0, 0);
    lcd.print(F("S3 counts:      "));
    lcd.setCursor(11, 0);
    lcd.print(LevelSensor[2]);
  
    lcd.setCursor(0, 1);
    lcd.print(F("L1 counts:      "));
    lcd.setCursor(11, 1);
    lcd.print(LevelSensor[3]);
  }

  if (MessCntr == 15){
    lcd.setCursor(0, 0);
    lcd.print(F("L2 counts:      "));
    lcd.setCursor(11, 0);
    lcd.print(LevelSensor[4]);
  
    lcd.setCursor(0, 1);
    lcd.print(F("L3 counts:      "));
    lcd.setCursor(11, 1);
    lcd.print(LevelSensor[5]);
  }

  if (MessCntr == 18){
    lcd.setCursor(0, 0);
    lcd.print(F("L4 counts:      "));
    lcd.setCursor(11, 0);
    lcd.print(LevelSensor[6]);
  
    lcd.setCursor(0, 1);
    lcd.print(F("L5 counts:      "));
    lcd.setCursor(11, 1);
    lcd.print(LevelSensor[7]);
  }
  
  if (STATE == 4 && MessCntr == 21) {
    lcd.clear();
    lcd.print(F("Imm Stage:"));
    lcd.print(immerse_stage);
    lcd.setCursor(0, 1);
    lcd.print(F("LS9:"));
    lcd.print(LevelSensor[9]);
  }

  if (MessCntr > 23){
    MessCntr = 0;
  }

  //Serial.println(F("FLAG 6"));
}

// ============================================================================
// ROAH selector helpers
// ----------------------------------------------------------------------------
// Read_ROAH_Position()      - sample the three pins, decode position
// Update_ROAH()              - call once per loop iteration, log changes
// HandControlsAllowed()     - true when ModeUp/ModeDown should be honored
// AutoControlsAllowed()     - true when START/AFILL/STOP should be honored
// RemoteControlsAllowed()   - true when ABT/LVDS commands should be honored
// Test_ROAH_Wiring()         - interactive self-test via serial 'W' command
// ============================================================================

ROAH_Position Read_ROAH_Position() {
  int h = digitalRead(Hand);
  int a = digitalRead(Auto);
  int r = digitalRead(Remote);
  // Encode the three bits into a small integer for a clean switch.
  int pattern = (h ? 4 : 0) | (a ? 2 : 0) | (r ? 1 : 0);
  switch (pattern) {
    case 0b000: return ROAH_OFF;
    case 0b100: return ROAH_HAND;
    case 0b010: return ROAH_AUTO;
    case 0b001: return ROAH_REMOTE;
    default:    return ROAH_INVALID;   // multiple bits HIGH = wiring fault
  }
}

void Update_ROAH() {
  ROAH_Position p = Read_ROAH_Position();
  if (p != roah_current) {
    roah_previous = roah_current;
    roah_current = p;
    Serial.print(F("[ROAH] Position changed: "));
    Serial.print(ROAH_Name(roah_previous));
    Serial.print(F(" -> "));
    Serial.println(ROAH_Name(roah_current));
    if (p == ROAH_INVALID) {
      Serial.println(F("[ROAH] WARNING: invalid position decoded - wiring fault?"));
    }
  }
}

bool HandControlsAllowed()   { return (roah_current == ROAH_HAND); }
bool AutoControlsAllowed()   { return (roah_current == ROAH_AUTO); }
bool RemoteControlsAllowed() { return (roah_current == ROAH_REMOTE); }

// AUTO button edge detection. Returns true exactly once when the named
// button transitions LOW->HIGH (pressed). The previous-state global is
// updated on every call so subsequent calls return false until the
// button is released and pressed again. AUTO buttons are active-HIGH
// per the schematic (pulled DOWN to GND, button shorts to +5V).
bool AfillStartPressed() {
  int now = digitalRead(AfillStart);
  bool fired = (now == HIGH && afill_btn_prev == LOW);
  afill_btn_prev = now;
  return fired;
}
bool BasinStartPressed() {
  int now = digitalRead(AImmerStart);
  bool fired = (now == HIGH && basin_btn_prev == LOW);
  basin_btn_prev = now;
  return fired;
}
bool StopSequencePressed() {
  int now = digitalRead(AStop);
  bool fired = (now == HIGH && stop_btn_prev == LOW);
  stop_btn_prev = now;
  return fired;
}

// Apply the AUTO mode button-press semantics to the current state.
// Called once per loop iteration when ROAH==AUTO. Each helper edge-
// detects its button so a single press maps to a single state change.
void Process_Auto_Buttons() {
  if (AfillStartPressed()) {
    Serial.println(F("[AUTO] START AUTOFILL pressed -> SetState(2)"));
    SetState(2);
    return;  // SetState may have side effects; bail to avoid double-action
  }
  if (BasinStartPressed()) {
    // Forward step in the basin sequence: WGAS -> CGAS -> IMER -> WARM -> IDLE.
    // Always passes through WGAS first to ensure the lid warms/purges
    // before any LN2 contact. CGAS -> IMER advance is gated by the
    // 3-min soak timer (same gate as in HAND mode).
    Serial.print(F("[AUTO] START BASIN pressed -> "));
    switch (STATE) {
      case 1:  // IDLE -> WGAS
      case 2:  // AFILL -> WGAS (operator wants to start basin sequence)
        Serial.println(F("SetState(3) WGAS"));
        SetState(3);
        break;
      case 3:  // WGAS -> CGAS (lid is warm/dry, begin gradual cold-gas fill)
        Serial.println(F("SetState(5) CGAS"));
        SetState(5);
        break;
      case 5:  // CGAS -> IMER (gated by 3-min soak)
        if (cgas_soak_complete) {
          Serial.println(F("SetState(4) IMER"));
          SetState(4);
        } else {
          Serial.println(F("CGAS->IMER blocked: 3-min soak still in progress"));
          Horn_ShortBlip();   // "input rejected" annunciation
        }
        break;
      case 4:  // IMER -> WARM (data taking complete)
        Serial.println(F("SetState(6) WARM"));
        SetState(6);
        break;
      case 6:  // WARM -> IDLE (cycle complete)
        Serial.println(F("SetState(1) IDLE - cycle complete"));
        SetState(1);
        break;
      default:
        Serial.println(F("ignored (current state not eligible)"));
        break;
    }
    return;
  }
  if (StopSequencePressed()) {
    Serial.println(F("[AUTO] STOP SEQUENCE pressed -> SetState(1) IDLE"));
    SetState(1);
    return;
  }
}

// Interactive wiring cross-check. Sweeps through H -> A -> O -> R.
// Operator should start with the selector in HAND. Each step waits
// indefinitely for the expected position. Abort via any printable
// character on the serial line.
void Test_ROAH_Wiring() {
  Serial.println();
  Serial.println(F("=============================================================="));
  Serial.println(F("[ROAH SELF-TEST] Wiring cross-check"));
  Serial.println(F("[ROAH SELF-TEST] Start with the selector in HAND."));
  Serial.println(F("[ROAH SELF-TEST] Sweep order: HAND -> AUTO -> OFF -> REMOTE"));
  Serial.println(F("[ROAH SELF-TEST] Send a printable character (not Enter) to abort."));
  Serial.println(F("=============================================================="));

  // Drain leftover RX bytes (the CR/LF from the 'W<Enter>' command).
  while (Serial.available() > 0) Serial.read();

  const ROAH_Position sequence[4] = { ROAH_HAND, ROAH_AUTO, ROAH_OFF, ROAH_REMOTE };
  bool all_pass = true;

  for (int i = 0; i < 4; i++) {
    ROAH_Position expected = sequence[i];
    Serial.println();
    Serial.print(F("[ROAH SELF-TEST] STEP "));
    Serial.print(i + 1); Serial.print(F(" of 4: rotate to "));
    Serial.print(ROAH_Name(expected));
    Serial.println(F("..."));

    ROAH_Position last_seen = ROAH_INVALID;
    bool reached = false;
    bool aborted = false;

    while (!reached && !aborted) {
      // Allow user abort on a real character (not CR/LF)
      if (Serial.available() > 0) {
        int c = Serial.read();
        if (c != '\r' && c != '\n' && c != -1) {
          aborted = true;
          break;
        }
      }

      ROAH_Position p = Read_ROAH_Position();
      if (p != last_seen) {
        last_seen = p;
        Serial.print(F("[ROAH SELF-TEST]   currently reading: "));
        Serial.print(ROAH_Name(p));
        Serial.print(F("  (Hand=")); Serial.print(digitalRead(Hand));
        Serial.print(F(" Auto=")); Serial.print(digitalRead(Auto));
        Serial.print(F(" Remote=")); Serial.print(digitalRead(Remote));
        Serial.println(F(")"));
      }

      if (p == expected) {
        delay(250);  // debounce
        if (Read_ROAH_Position() == expected) {
          reached = true;
          break;
        }
      }
      delay(50);
    }

    if (aborted) {
      Serial.println(F("[ROAH SELF-TEST] ABORTED by operator"));
      roah_current = Read_ROAH_Position();
      roah_previous = roah_current;
      return;
    }

    Serial.print(F("[ROAH SELF-TEST]   PASS - "));
    Serial.print(ROAH_Name(expected));
    Serial.println(F(" position confirmed"));
  }
  (void)all_pass;  // unused now (no FAIL path with no timeout) but keep
                   // the variable in case future revisions add per-step
                   // failure modes other than abort.

  Serial.println();
  Serial.println(F("=============================================================="));
  Serial.println(F("[ROAH SELF-TEST] OVERALL RESULT: PASS - all 4 positions wired correctly"));
  Serial.println(F("=============================================================="));
  roah_current = Read_ROAH_Position();
  roah_previous = roah_current;
}

// ============================================================================
// Horn alarm patterns
// ----------------------------------------------------------------------------
// Pulse_Horn() is called from SetState(0). It reads two globals:
//   - boot_complete: gates the audible alarm during setup() to avoid
//     spurious horns from transient sensor readings at boot.
//   - Err_Code:      selects the pattern that best matches the fault
//     severity / nature.
// On return, the horn pin is guaranteed LOW. Blocking by design - see
// v10 / v11 changelog notes.
// ============================================================================

// 3 short HIGH pulses, 1 s each, separated by 1 s gaps. Used for overfill
// and other severe faults. Total envelope ~5 s.
static void Horn_ThreePulse() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(HornCMD, HIGH);
    delay(1000);
    digitalWrite(HornCMD, LOW);
    if (i < 2) delay(1000);
  }
  digitalWrite(HornCMD, LOW);  // defensive
}

// 2 short HIGH pulses, 1 s each, separated by 1 s gap. Used for
// over-pressure and sensor faults - distinguishable by ear from the
// 3-pulse overfill pattern. Total envelope ~3 s.
static void Horn_TwoPulse() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(HornCMD, HIGH);
    delay(1000);
    digitalWrite(HornCMD, LOW);
    if (i < 1) delay(1000);
  }
  digitalWrite(HornCMD, LOW);  // defensive
}

// Single 2 s HIGH pulse. Used for empty-dewar annunciation - more of a
// "hey, you should know about this" tone than a "danger" alarm.
static void Horn_LongTone() {
  digitalWrite(HornCMD, HIGH);
  delay(2000);
  digitalWrite(HornCMD, LOW);
  digitalWrite(HornCMD, LOW);  // defensive
}

// Short distinct pulse used for "input rejected" annunciation - e.g. a
// serial state-change command attempted while ROAH is in OFF. Total
// envelope 500 ms. Distinct from any fault pattern and short enough not
// to be confused with a real alarm.
static void Horn_ShortBlip() {
  // Always sound this even if !boot_complete - the OFF-violation case
  // can only fire after the operator has typed a command, which means
  // setup() has long since completed. The boot-suppression isn't
  // relevant here.
  digitalWrite(HornCMD, HIGH);
  delay(500);
  digitalWrite(HornCMD, LOW);
}

// Returns true if the operator's serial state-change command should be
// REFUSED because ROAH is in OFF. Produces the operator feedback
// (serial warning + short horn pulse) as a side effect. Caller should
// `if (RejectIfOff(n)) break;` from the serial dispatch case before
// calling SetState.
bool RejectIfOff(int requested_state) {
  if (roah_current != ROAH_OFF) return false;
  Serial.print(F("[ROAH] WARNING: ROAH selector is in OFF position. "));
  Serial.print(F("Serial state-change command to STATE "));
  Serial.print(requested_state);
  Serial.println(F(" REFUSED."));
  Serial.println(F("[ROAH] Move selector out of OFF to allow state changes."));
  Horn_ShortBlip();
  return true;
}

void Pulse_Horn() {
  // Suppress audible alarm during setup() / recovery init
  if (!boot_complete) {
    Serial.println(F("[HORN] Suppressed during init phase"));
    digitalWrite(HornCMD, LOW);
    return;
  }

  // Dispatch on Err_Code (set by the caller before SetState(0))
  switch (Err_Code) {
    case 5:  // Empty Dewar - annunciation tone
      Serial.println(F("[HORN] Empty-dewar tone (1x 2s)"));
      Horn_LongTone();
      break;

    case 1:  // Chamber overfill
    case 7:  // Dewar overfill
      Serial.println(F("[HORN] Overfill alarm (3x 1s)"));
      Horn_ThreePulse();
      break;

    case 2:  // Over-pressure
    case 3:  // Sensor short
    case 4:  // Sensor open
      Serial.print(F("[HORN] Pressure/sensor fault alarm (2x 1s) for Err_Code="));
      Serial.println(Err_Code);
      Horn_TwoPulse();
      break;

    default: // Any unrecognized code - treat as urgent
      Serial.print(F("[HORN] Generic critical alarm (3x 1s) for Err_Code="));
      Serial.println(Err_Code);
      Horn_ThreePulse();
      break;
  }
}

void SetState(int n){
  // ==========================================================================
  // CRITICAL SAFETY: Close all valves IMMEDIATELY before state transition
  // This prevents gas leaks during state changes
  // ==========================================================================
  //digitalWrite(LiquidCMD, LOW);      // Liquid valve
  //digitalWrite(ExValveCMD1, LOW);    // Cold gas valve 1 (NO)
  //digitalWrite(ExValveCMD2, LOW);    // Cold gas valve 2 (NC)
  //digitalWrite(ExValveCMD3, LOW);    // Warm gas valve 1
  //digitalWrite(WarmGasCMD, LOW);     // Warm gas valve 2
  //digitalWrite(EnableLid, LOW);      // Heated lid off
  //SetDac(0);                          // Modulating valve CLOSED (critical!)
  
  // Update state variable
  STATE=n;
  
   // ============================================================
  // Close valves for safety - EXCEPT when entering autofill
  // ============================================================
  if (n != 2) {  // <-- Don't close valves when entering autofill!
    digitalWrite(LiquidCMD, LOW);
    digitalWrite(ExValveCMD1, LOW);
    digitalWrite(ExValveCMD2, LOW);
    digitalWrite(ExValveCMD3, LOW);
    digitalWrite(WarmGasCMD, LOW);
    digitalWrite(EnableLid, LOW);
    SetDac(0);
  } else {
    // Entering autofill - just close non-autofill valves
    digitalWrite(ExValveCMD1, LOW);
    digitalWrite(ExValveCMD3, LOW);
    digitalWrite(WarmGasCMD, LOW);
    digitalWrite(EnableLid, LOW);
    SetDac(0);
    // Don't touch LiquidCMD or ExValveCMD2 - let State 2 logic handle them
  }
  
  // Update LEDs
  digitalWrite(LED_Idle, LOW);
  digitalWrite(LED_Autofill, LOW);
  digitalWrite(LED_WarmGas, LOW);
  digitalWrite(LED_ColdGas, LOW);
  digitalWrite(LED_Immerse, LOW);
  digitalWrite(LED_Warmup, LOW);
  digitalWrite(LED_Fault, LOW);
  
  // Set LCD and turn on appropriate LED
  lcd.setCursor(0,0);        
  if (n==0) {
    digitalWrite(LED_Fault, HIGH);
    lcd.print(F("  SYSTEM FAULT  "));
    Serial.println(F("[STATE] Entered FAULT - sounding horn alarm"));
    Pulse_Horn();   // pattern dispatched by Err_Code; silent if !boot_complete
  }
  else if (n==1) {
    digitalWrite(LED_Idle, HIGH);
    lcd.print(F("Mode: Idle......"));
  }
  else if (n==2) {
    digitalWrite(LED_Autofill, HIGH);
    lcd.print(F("Mode: Autofill SD"));
    Sanity = 0;  // Disable sanity during autofill
    // Reset dual-timer cutoff state for a clean autofill session
    ls1_full_first_ms = 0;
    ls0_overfill_first_ms = 0;
    autofill_full_latched = false;
    Serial.println(F("[STATE] Entered Autofill - LS1/LS0 timers cleared"));
  }
  else if (n==3) {
    digitalWrite(LED_WarmGas, HIGH);
    lcd.print(F("Mode: Warm Gas.."));
  }
  else if (n==4) {
    digitalWrite(LED_Immerse, HIGH);
    lcd.print(F("Mode: Immerse..."));
    immerse_stage = 0;
    immerse_timer = 0;
    Serial.println(F("[IMMERSE] Starting fill to LS6 (~5\" operating level)"));
  }
  else if (n==5) {
    digitalWrite(LED_ColdGas, HIGH);
    lcd.print(F("Mode: Cold Gas.."));
    // Reset CGAS soak gate for a clean entry
    cgas_ls9_first_immersed_ms = 0;
    cgas_soak_complete = false;
    // Reset DAC ramp - start at 0, will ramp up smoothly to the
    // LS9-status target
    cgas_dac_current = 0;
    cgas_dac_last_step_ms = millis();
    SetDac(0);
    Serial.println(F("[CGAS] Starting gradual fill to LS9 (0.5\")"));
  }
  else if (n==6) {
    digitalWrite(LED_Warmup, HIGH);
    lcd.print(F("Mode: Warm-up..."));
    warm_phase = 0;
    Serial.println(F("[WARM] Starting drain phase (lid heat to evaporate LN2)"));
  }
 
  
  delay(500);  // Visual feedback delay
  
  // Log state change for debugging
  Serial.print(F("State changed to: "));
  Serial.println(STATE);
}

void Read_Level_Sensors(){ 

  SetI2C(i2cChannel0);
  for(int i = 0; i < 10; i++) {

    SetMux(i);

    adc0 = ads.readADC_SingleEnded(0);         //ADC channel 0 is assigned to level sensors

    LevelSensor[i]=adc0;
    if (adc0 < 10000) {
      LevelStatus[i] = 1; // Too Low -- shorted out?
    }      
    else if (adc0 < 16000) {
      LevelStatus[i] = 2; // ~room temp
    }
    else if (adc0 < 19500) {
      LevelStatus[i] = 3; // in cold gas
    }
    else if (adc0 < 25000) {
      LevelStatus[i] = 4; // immersed
    }
    else {
      LevelStatus[i] = 5; // Too High -- open circuit? 
    } 
  }

  //Set Test Basin Level
  //Sensors 6-9 deprecated by Grenoble sensors.  Sensor 5 only used for overfill protection.
  TC_Level = 0; 
  if (LevelStatus[9] == 4) {
    //TC_Level = 1;
  }
  if (LevelStatus[8] == 4) {
    //TC_Level = 2;
  }
  if (LevelStatus[7] == 4) {
    //TC_Level = 3;
  }
  if (LevelStatus[6] == 4) {
    //TC_Level = 4;   //TC Full
  }
  if (LevelStatus[5] == 4) {
    TC_Level = 5; //TC Overfull
    Err_Code = 1; // This is chamber overfill
    SetState(0); 
  }

  //Set Dewar Level
  Dewar_Level = 0; 

  if (LevelStatus[4] == 4) {
    Dewar_Level = 1;    //Dewar Empty
  }
  if (LevelStatus[3] == 4) {
    Dewar_Level = 2;    //1/3 Full
  }
  if (LevelStatus[2] == 4) {
    Dewar_Level = 3;    //2/3 Full
  }
  if (LevelStatus[1] == 4) {
    Dewar_Level = 4;   //Dewar Full
  }
 // if (LevelStatus[4] == 4) {
   // Err_Code = 7;       // This is dewar overfill
   // SetState(0); 
   // Dewar_Level = 5;
  //}

  if (Sanity == 1){ // then perform the sanity checks
    for(int i = 0; i < 16; i++) {
      if (LevelStatus[i] == 1) {
        Err_Code = 3;
        SetState(0); // shorted sensor circuit
      }
      if (LevelStatus[i] == 5) {
        Err_Code = 4;
        SetState(0); // open sensor circuit
      }
    }
    
    if (LevelStatus[4] == 2) {
      Err_Code = 5; //Indicates empty Dewar
      SetState(0); 
    }
  } // end of sanity checking

}

void Get_SD_Pressure(){
  SetI2C(i2cChannel0);
  adc0 = ads.readADC_SingleEnded(1);
  Pressure[0] = adc0;

  adc0 = ads.readADC_SingleEnded(2);
  Pressure[1] = adc0;

  // Don't trigger pressure fault during autofill
  if (Pressure[0] > P_Limit3 && STATE != 2) {  // <-- Added STATE check
    Err_Code = 2;
    SetState(0);
  }
//  else if (Pressure[0] > P_Limit3 && STATE == 2) {
//   // During autofill, just close valve but stay in autofill
//   digitalWrite(LiquidCMD, LOW);
//    digitalWrite(ExValveCMD2, LOW);
//    Serial.println(F("[AUTOFILL] High pressure - valve closed (staying in autofill)"));
//  }
}

// void Get_Lid_Temperature() {
//   SetI2C(i2cChannel2);
  
//   //aht.begin();

//   sensors_event_t hum, temp;
//   aht.getEvent(&hum, &temp);

//   //Serial.println(hum.relative_humidity);
//   //Serial.println(temp.temperature);
// }


void SetMux(int ch)
{
   if (ch==0){digitalWrite (MuxD,LOW);digitalWrite (MuxC,LOW);digitalWrite (MuxB,LOW);digitalWrite (MuxA,LOW);}
   if (ch==1){digitalWrite (MuxD,HIGH);digitalWrite (MuxC,LOW);digitalWrite (MuxB,LOW);digitalWrite (MuxA,LOW);}
   if (ch==2){digitalWrite (MuxD,LOW);digitalWrite (MuxC,HIGH);digitalWrite (MuxB,LOW);digitalWrite (MuxA,LOW);}
   if (ch==3){digitalWrite (MuxD,HIGH);digitalWrite (MuxC,HIGH);digitalWrite (MuxB,LOW);digitalWrite (MuxA,LOW);}
   if (ch==4){digitalWrite (MuxD,LOW);digitalWrite (MuxC,LOW);digitalWrite (MuxB,HIGH);digitalWrite (MuxA,LOW);}
   if (ch==5){digitalWrite (MuxD,HIGH);digitalWrite (MuxC,LOW);digitalWrite (MuxB,HIGH);digitalWrite (MuxA,LOW);}
   if (ch==6){digitalWrite (MuxD,LOW);digitalWrite (MuxC,HIGH);digitalWrite (MuxB,HIGH);digitalWrite (MuxA,LOW);}
   if (ch==7){digitalWrite (MuxD,HIGH);digitalWrite (MuxC,HIGH);digitalWrite (MuxB,HIGH);digitalWrite (MuxA,LOW);}
   if (ch==8){digitalWrite (MuxD,LOW);digitalWrite (MuxC,LOW);digitalWrite (MuxB,LOW);digitalWrite (MuxA,HIGH);}
   if (ch==9){digitalWrite (MuxD,HIGH);digitalWrite (MuxC,LOW);digitalWrite (MuxB,LOW);digitalWrite (MuxA,HIGH);}
   if (ch==10){digitalWrite (MuxD,LOW);digitalWrite (MuxC,HIGH);digitalWrite (MuxB,LOW);digitalWrite (MuxA,HIGH);}
   if (ch==11){digitalWrite (MuxD,HIGH);digitalWrite (MuxC,HIGH);digitalWrite (MuxB,LOW);digitalWrite (MuxA,HIGH);}
   if (ch==12){digitalWrite (MuxD,LOW);digitalWrite (MuxC,LOW);digitalWrite (MuxB,HIGH);digitalWrite (MuxA,HIGH);}
   if (ch==13){digitalWrite (MuxD,HIGH);digitalWrite (MuxC,LOW);digitalWrite (MuxB,HIGH);digitalWrite (MuxA,HIGH);}
   if (ch==14){digitalWrite (MuxD,LOW);digitalWrite (MuxC,HIGH);digitalWrite (MuxB,HIGH);digitalWrite (MuxA,HIGH);}
   if (ch==15){digitalWrite (MuxD,HIGH);digitalWrite (MuxC,HIGH);digitalWrite (MuxB,HIGH);digitalWrite (MuxA,HIGH);}
   delay(2);
}

float pressurePSI(int pressure) {
  //10-2V, divided to 4 to 0.8
  // 1x gain   +/- 4.096V  1 bit =  0.125mV
  //15 psi = 32,000 Counts, 0 psi = 6,400 Counts
  // float temp1 = pressure - 6400;
  // float temp2 = temp1 * 0.000125;
  // return temp2;

  // Sensor: 0 PSI = 6400 counts, 15 PSI = 32000 counts
  // Span: 25600 counts = 15 PSI → 0.000586 PSI/count
  float psi = (pressure - 6400) * (15.0 / 25600.0);
  return psi;
}

// void SetI2C(int ch)
// {
//   int test = 0;

//   //Serial.println(F("FLAG 3.1.2"));
//   //I2C SW Address is 0x70
//   //Channel 0 = 0x1, Level and Pressure
//   //Channel 1 = 0x2, Lid supply air
//   //Channel 2 = 0x4, Lid return air
//   //Channel 3 = 0x8, Basin Level capacitive sensors
//   Wire.beginTransmission(0x70);
//   Wire.write(ch);
//   Wire.endTransmission(true);

//   Wire.requestFrom(0x70, 1);
//   test = Wire.read();
//   //Serial.println(test);
//   //Serial.println(F("FLAG 3.1.3"));

// }

bool SetI2C(int ch)
{
  Wire.beginTransmission(0x70);
  Wire.write((uint8_t)ch);
  uint8_t err = Wire.endTransmission(true);
  delayMicroseconds(300);
  return (err == 0);
}

// void SetI2C(int ch)
// {
//   Wire.beginTransmission(0x70);
//   Wire.write(ch);
//   Wire.endTransmission(true);
//   delayMicroseconds(300);
// }





void SetDac(byte level) {
  for (int i = 0; i < 8; i++) {
      digitalWrite(DacBits[i], (level >> i) & 0x01);
  }

}

/*

bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void scanI2C() {
  byte error, address;
  int nDevices = 0;

  Serial.println(F("I2C scan start"));

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("I2C device found at 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.println(address, HEX);
      nDevices++;
    }
  }

  if (nDevices == 0) {
    Serial.println(F("No I2C devices found"));
  }

  Serial.println(F("I2C scan end"));
}


// dac_code: 0..127
void setCapDAC_A(uint8_t dac_code) {
  uint8_t reg = 0x80 | (dac_code & 0x7F);  // enable + value in bits 6..0
  ad->writeCapDacARegister(reg, true);
}

void setCapDAC_B(uint8_t dac_code) {
  uint8_t reg = 0x80 | (dac_code & 0x7F);  // enable + value in bits 6..0
  ad->writeCapDacBRegister(reg, true);
}



void setupAD7746() {
  // Make sure we are on the right I2C mux channel if you use one
  if (!SetI2C(i2cChannel3)) {
  Serial.println(F("[AD] FAIL: SetI2C(ch3) in setupAD7746"));
  return;
  }
  delay(2);


  // 1) Reset once
  ad->reset();
  delay(5);

  // 2) Configure once
  ad->configAD7746_EVAL_ruler();
  ad->setExtensionFactor(0.);
  ad->setSingleMode(0, 0);
  ad->disableTempRead();

  // Enable CapDAC A and B in *hardware* (writes AD7746 registers).
  // 0x20 is a reasonable starting code.
  ad->setCapaDac_A(0x20);   // writes CapDAC A reg: 0x80 | code
  ad->setCapaDac_B(0x20);   // writes CapDAC B reg: 0x80 | code

  // IMPORTANT: startConversionCin1/Cin2() overwrites CapDAC using these variables
  ad->calibratedCapaDac_A_cin1 = 0x20;
  ad->calibratedCapaDac_A_cin2 = 0x20;


  // read back and print to confirm it's actually set in the chip
  Serial.print(F("[AD] CapDAC A reg now = 0x"));
  Serial.println(ad->readCapDacARegister(), HEX);
  Serial.print(F("[AD] CapDAC B reg now = 0x"));
  Serial.println(ad->readCapDacBRegister(), HEX);


  // --- Verify key registers after config ---
  int cap_setup_i = ad->readCapSetupRegister();
  int vt_setup_i  = ad->readVtSetupRegister();
  int exc_setup_i = ad->readExcSetupRegister();
  int config_i    = ad->readConfigurationRegister();

  uint8_t cap_setup = (cap_setup_i < 0) ? 0xFF : (uint8_t)cap_setup_i;
  uint8_t vt_setup  = (vt_setup_i  < 0) ? 0xFF : (uint8_t)vt_setup_i;
  uint8_t exc_setup = (exc_setup_i < 0) ? 0xFF : (uint8_t)exc_setup_i;
  uint8_t config    = (config_i    < 0) ? 0xFF : (uint8_t)config_i;

  Serial.print(F("[AD] CAP_SETUP=0x")); Serial.println(cap_setup, HEX);
  Serial.print(F("[AD] VT_SETUP =0x")); Serial.println(vt_setup, HEX);
  Serial.print(F("[AD] EXC_SETUP=0x")); Serial.println(exc_setup, HEX);
  Serial.print(F("[AD] CONFIG   =0x")); Serial.println(config, HEX);

}

void TuneGrenobleCapDAC() {
  // Make sure we are on the AD7746 mux channel
  if (!SetI2C(i2cChannel3)) {
    Serial.println(F("[TUNE] FAIL: SetI2C(ch3)"));
    return;
  }

  // Target: bring CAPDATA close to 0x800000 (0 pF)
  const uint32_t TARGET = 0x800000UL;

  uint8_t bestCode1 = 0;
  uint8_t bestCode2 = 0;
  uint32_t bestErr1 = 0xFFFFFFFFUL;
  uint32_t bestErr2 = 0xFFFFFFFFUL;
  uint32_t bestRaw1 = 0;
  uint32_t bestRaw2 = 0;



  // Sweep CapDAC code 0..127
  for (uint8_t code = 0; code <= 0x7F; code++) {

    // IMPORTANT:
    // Your library overwrites CapDAC_A during startConversionCin1/Cin2 using:
    //   calibratedCapaDac_A_cin1 / calibratedCapaDac_A_cin2
    // So we must set those variables, not just write the register once.
    ad->calibratedCapaDac_A_cin1 = code;
    ad->calibratedCapaDac_A_cin2 = code;

    // Also write the actual register so the chip is definitely updated now
    ad->setCapaDac_A(code);

    // Give it a moment
    delay(5);

    // Read CIN1
    ad->setCin1();
    ad->get_dataCin1(false);
    uint32_t raw1 = (ad->capaDAC & 0x00FFFFFFUL);
    uint32_t err1 = (raw1 > TARGET) ? (raw1 - TARGET) : (TARGET - raw1);

    // Read CIN2
    ad->setCin2();
    ad->get_dataCin2(false);
    uint32_t raw2 = (ad->capaDAC & 0x00FFFFFFUL);
    uint32_t err2 = (raw2 > TARGET) ? (raw2 - TARGET) : (TARGET - raw2);

    if (err1 < bestErr1) { bestErr1 = err1; bestCode1 = code; bestRaw1 = raw1; }
    if (err2 < bestErr2) { bestErr2 = err2; bestCode2 = code; bestRaw2 = raw2; }

    // If both channels are very close to mid-scale, stop early
    if (bestErr1 < 0x2000 && bestErr2 < 0x2000) break;
  }

  Serial.print(F("[TUNE] Best CIN1: code=")); Serial.print(bestCode1);
  Serial.print(F(" raw=0x")); Serial.println(bestRaw1, HEX);

  Serial.print(F("[TUNE] Best CIN2: code=")); Serial.print(bestCode2);
  Serial.print(F(" raw=0x")); Serial.println(bestRaw2, HEX);

  // Apply the best codes (leave them set)
  ad->calibratedCapaDac_A_cin1 = bestCode1;
  ad->calibratedCapaDac_A_cin2 = bestCode2;

  // If you want a single code for both channels, uncomment this:
  // uint8_t one = (bestErr1 < bestErr2) ? bestCode1 : bestCode2;
  // ad->calibratedCapaDac_A_cin1 = one;
  // ad->calibratedCapaDac_A_cin2 = one;

  // Write the hardware register to match CIN1's code right now (library will swap per-channel on next start)
  ad->setCapaDac_A(ad->calibratedCapaDac_A_cin1);

  Serial.println(F("[TUNE] Done."));
}

*/

/*

static void printHex24(uint32_t v) {
  v &= 0x00FFFFFFUL;
  if (v < 0x100000) Serial.print(F("0"));
  if (v < 0x010000) Serial.print(F("0"));
  if (v < 0x001000) Serial.print(F("0"));
  if (v < 0x000100) Serial.print(F("0"));
  if (v < 0x000010) Serial.print(F("0"));
  Serial.print(v, HEX);
}

*/

/*

void GrenobleRawDebugOnce()
{
  // Must already be on mux channel 3

  // Make sure CAPDACs stay off
  ad->writeCapDacARegister(0x00, true);
  ad->writeCapDacBRegister(0x00, true);
  delay(5);

  // ---- CIN1 ----
  ad->writeCapSetupRegister(0x80, true); // CAPEN=1, CIN2=0, CAPDIFF=0
  delay(5);

  ad->startConversionCin1(false);
  delay(120);                 // <-- give it plenty of time for a fresh conversion
  ad->read_data(false);       // <-- THIS reads STATUS + CAPDATA + VTDATA and fills ad->capaDAC
  uint32_t r1 = ad->capaDAC & 0x00FFFFFFUL;

  Serial.print(F("[DBG] CAP_SETUP=0x")); Serial.print(ad->readCapSetupRegister(), HEX);
  Serial.print(F(" CIN1 CAPDATA=0x")); printHex24(r1);
  Serial.println();

  // ---- CIN2 ----
  ad->writeCapSetupRegister(0xC0, true); // CAPEN=1, CIN2=1, CAPDIFF=0
  delay(5);

  ad->startConversionCin2(false);
  delay(120);
  ad->read_data(false);
  uint32_t r2 = ad->capaDAC & 0x00FFFFFFUL;

  Serial.print(F("[DBG] CAP_SETUP=0x")); Serial.print(ad->readCapSetupRegister(), HEX);
  Serial.print(F(" CIN2 CAPDATA=0x")); printHex24(r2);
  Serial.println();
}

*/

/*

void GrenobleReadout() {  
  // Always select the mux channel before talking to 0x48
  if (!SetI2C(i2cChannel3)) {
    Serial.println(F("[GR] FAIL: SetI2C(ch3)"));
    GrenobleLevel[0] = NAN;
    GrenobleLevel[1] = NAN;
    return;
  }

  if (!i2cDevicePresent(0x48)) {
    Serial.println(F("[GR] AD7746 not present at 0x48 on ch3"));
    GrenobleLevel[0] = NAN;
    GrenobleLevel[1] = NAN;
    return;
  }

  // diagnostic
  ad->writeCapDacARegister(0x00, true);  // disable CAPDAC A
  ad->writeCapDacBRegister(0x00, true);  // disable CAPDAC B
  delay(10);

  uint8_t cap_before_1 = (uint8_t)ad->readCapSetupRegister();
  //Serial.print(F("[GR] CAP_SETUP before CIN1 = 0x")); Serial.println(cap_before_1, HEX);

  // Read CIN1 (status return value is from get_dataCin1)
  ad->setCin1();
  int s1 = ad->get_dataCin1(false);
  uint8_t  st1  = ad->status;
  uint32_t raw1 = ad->capaDAC & 0x00FFFFFFUL;
  double   rel1 = ad->capaciteCin1;   // “relative” pF value computed in library
  double   abs1 = ad->getAbsoluteCapaCin1();

  uint8_t cap_after_1 = (uint8_t)ad->readCapSetupRegister();
  //Serial.print(F("[GR] CAP_SETUP after  CIN1 = 0x")); Serial.println(cap_after_1, HEX);

  uint8_t cap_before_2 = (uint8_t)ad->readCapSetupRegister();
  //Serial.print(F("[GR] CAP_SETUP before CIN2 = 0x")); Serial.println(cap_before_2, HEX);

  // Read CIN2
  ad->setCin2();
  int s2 = ad->get_dataCin2(false);
  uint8_t  st2  = ad->status;
  uint32_t raw2 = ad->capaDAC & 0x00FFFFFFUL;
  double   rel2 = ad->capaciteCin2;
  double   abs2 = ad->getAbsoluteCapaCin2();

  uint8_t cap_after_2 = (uint8_t)ad->readCapSetupRegister();
  //Serial.print(F("[GR] CAP_SETUP after  CIN2 = 0x")); Serial.println(cap_after_2, HEX);

  GrenobleRel[0] = rel1;
  GrenobleRel[1] = rel2;

  // Read the digital capacitors actually programmed into the AD7746
  GrenobleCapDacA = (uint8_t)ad->readCapDacARegister();
  GrenobleCapDacB = (uint8_t)ad->readCapDacBRegister();


  static bool once = true;
  if (once) {
    once = false;
    Serial.print(F("[GR] RAW1 24b = 0x")); printHex24(raw1); Serial.println();
    Serial.print(F("[GR] RAW2 24b = 0x")); printHex24(raw2); Serial.println();

  }


  if (grenoble_print) {
    Serial.print(F("[GR] CIN1: s=")); Serial.print(s1);
    Serial.print(F(" status=0x")); Serial.print(st1, HEX);
    Serial.print(F(" raw=0x"));
    if (raw1 < 0x100000) Serial.print(F("0"));
    if (raw1 < 0x010000) Serial.print(F("0"));
    if (raw1 < 0x001000) Serial.print(F("0"));
    if (raw1 < 0x000100) Serial.print(F("0"));
    if (raw1 < 0x000010) Serial.print(F("0"));
    Serial.print(raw1, HEX);
    Serial.print(F(" rel_pF=")); Serial.print(rel1, 4);
    Serial.print(F(" abs_pF=")); Serial.println(abs1, 4);

    Serial.print(F("[GR] CIN2: s=")); Serial.print(s2);
    Serial.print(F(" status=0x")); Serial.print(st2, HEX);
    Serial.print(F(" raw=0x"));
    if (raw2 < 0x100000) Serial.print(F("0"));
    if (raw2 < 0x010000) Serial.print(F("0"));
    if (raw2 < 0x001000) Serial.print(F("0"));
    if (raw2 < 0x000100) Serial.print(F("0"));
    if (raw2 < 0x000010) Serial.print(F("0"));
    Serial.print(raw2, HEX);
    Serial.print(F(" rel_pF=")); Serial.print(rel2, 4);
    Serial.print(F(" abs_pF=")); Serial.println(abs2, 4);

  }


  GrenobleLevel[0] = abs1;
  GrenobleLevel[1] = abs2;
  GrenobleRaw[0]   = raw1;
  GrenobleRaw[1]   = raw2;


  // if (GrenobleLevel[1] < 0.53) {
  //   Serial.println(F("[GR] WARNING: CIN2 looks open/unplugged"));
  // }
  delay(50);  // limit Grenoble update to ~20 Hz


}

*/

// ============================================================================
// RAW I2C FUNCTIONS FOR AD7746 (bypassing buggy library)
// ============================================================================
//
// CRITICAL FIX APPLIED (4/17/26):
// The original code had CONFIGURATION register set to 0x01 (IDLE mode).
// This caused the chip to ignore channel selections in CAP_SETUP register.
//
// AD7746 CONFIGURATION Register (0x0A) bit definitions:
// Bits 6-4: Capacitance channel select
//   000 = idle (measurements disabled)
//   001 = CIN1 single-ended
//   010 = CIN2 single-ended  
//   011 = CIN1 differential
//   100 = CIN2 differential
// Bits 1-0: Conversion mode
//   01 = continuous conversion
//   10 = single conversion
//   11 = power down
//
// Correct values:
// CIN1: 0x21 (bits 6-4 = 001, bits 1-0 = 01) = CIN1 continuous
// CIN2: 0x41 (bits 6-4 = 010, bits 1-0 = 01) = CIN2 continuous
//
// The CAP_SETUP register (0x07) must ALSO be set:
// CIN1: 0x80 (bit 7 = enable, bit 6 = 0 for CIN1)
// CIN2: 0xC0 (bit 7 = enable, bit 6 = 1 for CIN2)
//
// BOTH registers must be set correctly for channel selection to work!
// ============================================================================

// Write a single register to AD7746
bool ad7746_writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(AD7746_ADDR);
  Wire.write(reg);
  Wire.write(value);
  byte error = Wire.endTransmission();
  return (error == 0);
}

// Read a single register from AD7746
uint8_t ad7746_readReg(uint8_t reg) {
  Wire.beginTransmission(AD7746_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);  // Send restart
  
  Wire.requestFrom(AD7746_ADDR, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0x00;
}

// Read 24-bit value from AD7746 (3 consecutive registers)
uint32_t ad7746_read24bit(uint8_t reg) {
  Wire.beginTransmission(AD7746_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);  // Send restart
  
  Wire.requestFrom(AD7746_ADDR, (uint8_t)3);
  
  uint32_t result = 0;
  if (Wire.available() >= 3) {
    result = (uint32_t)Wire.read() << 16;  // High byte
    result |= (uint32_t)Wire.read() << 8;   // Mid byte
    result |= (uint32_t)Wire.read();        // Low byte
  }
  
  return result;
}

// Initialize AD7746 chip via raw I2C
bool ad7746_init() {
  Serial.println(F("[AD7746] Starting initialization..."));
  
  // Step 1: Reset (write 0x80 to configuration register)
  Serial.println(F("[AD7746] Resetting chip..."));
  if (!ad7746_writeReg(AD7746_REG_CONFIGURATION, 0x80)) {
    Serial.println(F("[AD7746] ERROR: Reset write failed"));
    return false;
  }
  delay(100);
  
  // Step 2: Configure capacitance measurement
  // CAP_SETUP: 0x80 = CAPEN=1, CIN2=0 (select CIN1), CAPDIFF=0
  Serial.println(F("[AD7746] Configuring for capacitance measurement..."));
  if (!ad7746_writeReg(AD7746_REG_CAP_SETUP, 0x80)) {
    Serial.println(F("[AD7746] ERROR: CAP_SETUP write failed"));
    return false;
  }
  delay(20);
  
  // Step 3: Configure excitation - Back to original 0x0B
  // This worked for CIN2 (gave 2.18 pF reading)
  // CIN1 issue must be something else
  Serial.println(F("[AD7746] Setting excitation to 0x0B (original working value for CIN2)..."));
  if (!ad7746_writeReg(AD7746_REG_EXC_SETUP, 0x0B)) {
    Serial.println(F("[AD7746] ERROR: EXC_SETUP write failed"));
    return false;
  }
  delay(20);
  
  // Verify excitation setting
  uint8_t exc_check = ad7746_readReg(AD7746_REG_EXC_SETUP);
  Serial.print(F("[AD7746] EXC_SETUP readback = 0x"));
  Serial.println(exc_check, HEX);
  
  // Read CAP_SETUP to see default channel config
  uint8_t cap_setup = ad7746_readReg(AD7746_REG_CAP_SETUP);
  Serial.print(F("[AD7746] CAP_SETUP readback = 0x"));
  Serial.println(cap_setup, HEX);
  
  // Step 4: Set CapDAC A for CIN1 to MINIMUM
  // Baseline reading was 28 pF - WAY too high! Need minimal offset
  Serial.println(F("[AD7746] Setting CapDAC A to MINIMUM..."));
  if (!ad7746_writeReg(AD7746_REG_CAPDAC_A, 0x80)) {  // 0x00 = zero offset
    Serial.println(F("[AD7746] ERROR: CapDAC A write failed"));
    return false;
  }
  delay(20);
  
  // Step 5: Set CapDAC B for CIN2 to MINIMUM
  // Same issue - 28.3 pF baseline too high
  Serial.println(F("[AD7746] Setting CapDAC B to MINIMUM..."));
  if (!ad7746_writeReg(AD7746_REG_CAPDAC_B, 0x80)) {  // 0x00 = zero offset
    Serial.println(F("[AD7746] ERROR: CapDAC B write failed"));
    return false;
  }
  delay(20);
  
  // Step 6: Set default to CIN2 mode (will switch during readout)
  // CONFIGURATION: 0x41 = CIN2 single-ended, continuous conversion
  // Bits 6-4 = 010 (CIN2), Bits 1-0 = 01 (continuous)
  Serial.println(F("[AD7746] Setting default mode to CIN2 continuous..."));
  if (!ad7746_writeReg(AD7746_REG_CONFIGURATION, 0x41)) {
    Serial.println(F("[AD7746] ERROR: Configuration write failed"));
    return false;
  }
  delay(100);
  
  // Step 7: Verify CapDAC settings
  uint8_t capdac_a = ad7746_readReg(AD7746_REG_CAPDAC_A);
  uint8_t capdac_b = ad7746_readReg(AD7746_REG_CAPDAC_B);
  
  Serial.print(F("[AD7746] CapDAC_A = 0x"));
  Serial.print(capdac_a, HEX);
  Serial.print(F(", CapDAC_B = 0x"));
  Serial.println(capdac_b, HEX);
  
  if (capdac_a < 0x80 || capdac_b < 0x80) {
    Serial.println(F("[AD7746] WARNING: CapDAC not enabled properly"));
    return false;
  }
  
  Serial.println(F("[AD7746] Initialization complete!"));
  return true;
}

// Read Grenoble sensors using raw I2C
void GrenobleReadout_Raw() {
  if (!GRENOBLE_ENABLED) {
    GrenobleLevel[0] = 0.0;
    GrenobleLevel[1] = 0.0;
    return;
  }
  
  // Make sure we're on the right I2C mux channel
  if (!SetI2C(i2cChannel3)) {
    static unsigned long last_warn = 0;
    if (millis() - last_warn > 10000) {
      Serial.println(F("[GR] WARNING: Could not set I2C mux channel"));
      last_warn = millis();
    }
    GrenobleLevel[0] = 0.0;
    GrenobleLevel[1] = 0.0;
    return;
  }
  
  // ============================================================================
  // READ CIN1
  // ============================================================================
  
  // Configure for CIN1
  ad7746_writeReg(AD7746_REG_CAP_SETUP, 0x80);  // Enable CIN1 channel
  
  // CRITICAL FIX: Set CONFIGURATION register to select CIN1!
  // 0x21 = CIN1 single-ended (bits 6-4 = 001), continuous mode (bits 1-0 = 01)
  ad7746_writeReg(AD7746_REG_CONFIGURATION, 0x21);
  
  delay(100);  // Wait for channel switch and conversion
  
  // Wait for conversion complete (status bit 0 = 0 when ready)
  unsigned long start = millis();
  while (millis() - start < 200) {  // 200ms timeout
    uint8_t status = ad7746_readReg(AD7746_REG_STATUS);
    if (!(status & 0x01)) {  // Bit 0 = 0 when data ready
      break;
    }
    delay(1);
  }
  
  // Read 24-bit capacitance data for CIN1
  uint32_t raw1 = ad7746_read24bit(AD7746_REG_CAP_DATA_H);
  
  // ============================================================================
  // READ CIN2
  // ============================================================================
  
  // Configure for CIN2
  ad7746_writeReg(AD7746_REG_CAP_SETUP, 0xC0);  // Enable CIN2 channel
  
  // CRITICAL FIX: Set CONFIGURATION register to select CIN2!
  // 0x41 = CIN2 single-ended (bits 6-4 = 010), continuous mode (bits 1-0 = 01)
  ad7746_writeReg(AD7746_REG_CONFIGURATION, 0x41);
  
  delay(100);  // Wait for channel switch and conversion
  
  // Wait for conversion complete
  start = millis();
  while (millis() - start < 200) {
    uint8_t status = ad7746_readReg(AD7746_REG_STATUS);
    if (!(status & 0x01)) {
      break;
    }
    delay(1);
  }
  
  // Read 24-bit capacitance data for CIN2
  uint32_t raw2 = ad7746_read24bit(AD7746_REG_CAP_DATA_H);
  
  // Convert raw values to pF
  // AD7746: 0x800000 = 0pF, each LSB = 4fF (femtofarads)
  // Formula: pF = (raw - 0x800000) * 0.000004
  float pF1 = ((int32_t)raw1 - 0x800000) * 0.000004;
  float pF2 = ((int32_t)raw2 - 0x800000) * 0.000004;
  
  // Store results
  GrenobleLevel[0] = pF1;
  GrenobleLevel[1] = pF2;
  GrenobleRaw[0] = raw1;
  GrenobleRaw[1] = raw2;
  
  // Optional debug output
  if (grenoble_print) {
    Serial.print(F("[GR] CIN1: raw=0x"));
    Serial.print(raw1, HEX);
    Serial.print(F(" pF="));
    Serial.print(pF1, 4);
    Serial.print(F(" | CIN2: raw=0x"));
    Serial.print(raw2, HEX);
    Serial.print(F(" pF="));
    Serial.println(pF2, 4);
  }
  
  delay(50);  // Limit update rate
}

// ============================================================================
// DIAGNOSTIC FUNCTION - Dump AD7746 Registers
// ============================================================================
// Call this function to verify proper configuration
// Usage: Type 'D' in serial monitor to dump registers
void ad7746_dumpRegisters() {
  if (!GRENOBLE_ENABLED) {
    Serial.println(F("[AD7746] Grenoble not enabled, cannot dump registers"));
    return;
  }
  
  // Switch to correct I2C channel
  if (!SetI2C(i2cChannel3)) {
    Serial.println(F("[AD7746] Could not set I2C mux channel"));
    return;
  }
  
  Serial.println(F("========================================"));
  Serial.println(F("[AD7746] REGISTER DUMP"));
  Serial.println(F("========================================"));
  
  // Read key registers
  uint8_t status = ad7746_readReg(AD7746_REG_STATUS);
  uint8_t cap_setup = ad7746_readReg(AD7746_REG_CAP_SETUP);
  uint8_t exc_setup = ad7746_readReg(AD7746_REG_EXC_SETUP);
  uint8_t configuration = ad7746_readReg(AD7746_REG_CONFIGURATION);
  uint8_t capdac_a = ad7746_readReg(AD7746_REG_CAPDAC_A);
  uint8_t capdac_b = ad7746_readReg(AD7746_REG_CAPDAC_B);
  
  // Display with interpretation
  Serial.print(F("STATUS (0x00):        0x")); 
  Serial.print(status, HEX);
  Serial.print(F(" - "));
  if (status & 0x01) Serial.print(F("BUSY "));
  if (status & 0x02) Serial.print(F("EXCEEDS "));
  if (status & 0x04) Serial.print(F("C_RDY "));
  Serial.println();
  
  Serial.print(F("CAP_SETUP (0x07):     0x")); 
  Serial.print(cap_setup, HEX);
  Serial.print(F(" - "));
  if (cap_setup & 0x80) Serial.print(F("ENABLED "));
  if (cap_setup & 0x40) {
    Serial.print(F("CIN2"));
  } else {
    Serial.print(F("CIN1"));
  }
  Serial.println();
  
  Serial.print(F("EXC_SETUP (0x09):     0x")); 
  Serial.println(exc_setup, HEX);
  
  Serial.print(F("CONFIGURATION (0x0A): 0x")); 
  Serial.print(configuration, HEX);
  Serial.print(F(" - "));
  uint8_t channel = (configuration >> 4) & 0x07;
  switch(channel) {
    case 0: Serial.print(F("IDLE")); break;
    case 1: Serial.print(F("CIN1 single-ended")); break;
    case 2: Serial.print(F("CIN2 single-ended")); break;
    case 3: Serial.print(F("CIN1 differential")); break;
    case 4: Serial.print(F("CIN2 differential")); break;
    default: Serial.print(F("UNKNOWN"));
  }
  Serial.print(F(", Mode="));
  uint8_t mode = configuration & 0x07;
  if (mode == 1) Serial.print(F("CONTINUOUS"));
  else if (mode == 2) Serial.print(F("SINGLE"));
  else if (mode == 3) Serial.print(F("POWER_DOWN"));
  else Serial.print(F("UNKNOWN"));
  Serial.println();
  
  Serial.print(F("CAPDAC_A (0x0B):      0x")); 
  Serial.print(capdac_a, HEX);
  if (capdac_a & 0x80) {
    Serial.print(F(" - ENABLED, offset="));
    Serial.print(capdac_a & 0x7F);
  }
  Serial.println();
  
  Serial.print(F("CAPDAC_B (0x0C):      0x")); 
  Serial.print(capdac_b, HEX);
  if (capdac_b & 0x80) {
    Serial.print(F(" - ENABLED, offset="));
    Serial.print(capdac_b & 0x7F);
  }
  Serial.println();
  
  Serial.println(F("========================================"));
  
  // Verification
  Serial.println(F("\n[AD7746] CONFIGURATION VERIFICATION:"));
  if (configuration == 0x01) {
    Serial.println(F("ERROR: Configuration set to 0x01 (IDLE mode)"));
    Serial.println(F("  Channel selection will not work!"));
    Serial.println(F("  Should be 0x21 for CIN1 or 0x41 for CIN2"));
  } else if (configuration == 0x21 || configuration == 0x41) {
    Serial.println(F("OK: Configuration correctly set for active measurement"));
  } else {
    Serial.print(F("WARNING: Unexpected configuration value: 0x"));
    Serial.println(configuration, HEX);
  }
  Serial.println(F("========================================\n"));
}
