//free
#include <Adafruit_SSD1306.h>
#include <Joystick.h>
#include <Wire.h>
//#include <Encoder.h>

/*
 * Encoder rotaryEncoder(DT, CLK);
 * Interrupt : 0, 1, 2, 3, 7 priorité pour rotary encoder
 * A2 et A3 réservé pour I2C
 */

/*
********************************************************************************
**********************************GESTION GLOBAL********************************
********************************************************************************
*/

/*
TODO 

ROTARY !
créer
un joystick pour les pulses
un joystick pour les rotary knob (10 ?)
X joystick pour les axes (TOUT A LA FIN)

récupérer les rotary input des autres cartes de façon spécifique

SWITCH !
un joystick état
un joystick pulse
Récupérer les switch input des autres cartes de façon spécifique


Switch on off joystick pulse
Switch on off joystick rotary
Switch on off joystick axe
Switch on off joystick état
Switch on off joystick pulse
switch mom + - change display


tableau string écran

*/

// écrans display
#define DISPLAYCOUNT 32
#define DISPLAYCONFCOUNT 3
const String displayText[DISPLAYCONFCOUNT][DISPLAYCOUNT]={
  {"Button1"    ,"Button2"    ,"Button3"    ,"Button4"    ,"Button5"    ,"Button6"    ,"Button7"    ,"Button8"    ,"Button9"    ,"Button10"    ,"Button11"    ,"Button12"    ,"Button13"    ,"Button14"    ,"Button15"    ,"Button16"    ,"Button17"    ,"Button18"    ,"Button19"    ,"Button20"    ,"Button21"    ,"Button22"     ,"Button23"    ,"Button24"    ,"Button25"    ,"Button26"    ,"Button27"    ,"Button28"    ,"Button29"    ,"Button30"    ,"Button31"    ,"Button32"},
  {"Button1"    ,"Button2"    ,"Button3"    ,"Button4"    ,"Button5"    ,"Button6"    ,"Button7"    ,"Button8"    ,"Button9"    ,"Button10"    ,"Button11"    ,"Button12"    ,"Button13"    ,"Button14"    ,"Button15"    ,"Button16"    ,"Button17"    ,"Button18"    ,"Button19"    ,"Button20"    ,"Button21"    ,"Button22"     ,"Button23"    ,"Button24"    ,"Button25"    ,"Button26"    ,"Button27"    ,"Button28"    ,"Button29"    ,"Button30"    ,"Button31"    ,"Button32"},
  {"Button1"    ,"Button2"    ,"Button3"    ,"Button4"    ,"Button5"    ,"Button6"    ,"Button7"    ,"Button8"    ,"Button9"    ,"Button10"    ,"Button11"    ,"Button12"    ,"Button13"    ,"Button14"    ,"Button15"    ,"Button16"    ,"Button17"    ,"Button18"    ,"Button19"    ,"Button20"    ,"Button21"    ,"Button22"     ,"Button23"    ,"Button24"    ,"Button25"    ,"Button26"    ,"Button27"    ,"Button28"    ,"Button29"    ,"Button30"    ,"Button31"    ,"Button32"}
};


// Screen
#define OLED_RESET  -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(128, 32, &Wire, OLED_RESET);

// Constantes global
#define BUTTON_MAX 250 // (254 max)
#define T_CYCLE 5
#define T_RAZROT 15
#define PAD_COUNT 7 // 1 base button (joy*2+sliders) / 1 state switch / 1 pulse switch / 1 pulses / 1 rotary knob / X axes : (rotary)/11
#define VIDE 255
#define DISPLAY_I2C 0x3c

//TYPES
#define TYPE_BUTTON 1
#define TYPE_JOY 2
#define TYPE_SLIDER 3
#define TYPE_ROTARY 4
#define TYPE_SWITCH2 5
#define TYPE_SWITCH3 6


uint8_t BOARD_COUNT;
uint8_t * rotaryCount;
uint8_t * buttonCount;
uint8_t * sliderCount;
uint8_t * joystickCount;
uint8_t * displayCount;
uint8_t * switch2Count;
uint8_t * switch3Count;


#define I2C_DISPLAY_ADDR 0x3c
// adresses i2c : écrans, mega1, nano1, mega2, nano2
//uint8_t i2c_addr[BUTTON_MAX] = {0x3c,11,12,13,14};
uint8_t * i2c_addr;

// nombre de boutons
int realButtonTotal = 0;
int buttonTotal=0;
int rotaryTotal=0;
int sliderTotal=0;
int joystickTotal=0;

// button order
int rotaryPulse1 = 0;

// map analog
#define maxRotaryPot 63
#define minRotaryPot 0

// Déclaration des PADS (différents périphériques)
/*
1 base button (joy*2+sliders)
1 state switch
1 pulse switch
1 pulses
1 rotary knob
2 axes
*/
Joystick_ pads[PAD_COUNT] = {
  Joystick_(0x03, JOYSTICK_TYPE_JOYSTICK, BUTTON_MAX, 0, true, true, true, true, true, true, true, true, true, true, true),
  Joystick_(0x04, JOYSTICK_TYPE_JOYSTICK, BUTTON_MAX, 0, false, false, false, false, false, false, false, false, false, false, false),
  Joystick_(0x05, JOYSTICK_TYPE_JOYSTICK, BUTTON_MAX, 0, false, false, false, false, false, false, false, false, false, false, false),
  Joystick_(0x06, JOYSTICK_TYPE_JOYSTICK, BUTTON_MAX, 0, false, false, false, false, false, false, false, false, false, false, false),
  Joystick_(0x07, JOYSTICK_TYPE_JOYSTICK, BUTTON_MAX, 0, false, false, false, false, false, false, false, false, false, false, false),
  Joystick_(0x08, JOYSTICK_TYPE_MULTI_AXIS, 0, 0, true, true, true, true, true, true, true, true, true, true, true),
  Joystick_(0x09, JOYSTICK_TYPE_MULTI_AXIS, 0, 0, true, true, true, true, true, true, true, true, true, true, true)
};


#define padBase 0
#define padSwitchState 1
#define padSwitchPulse 2
#define padRotaryPulse 3
#define padRotaryKnob 4
#define padRotaryAxe 5
#define padRotaryAxeCount 2


#define CONF_BOARDS 7 // nb display | nb button | nb switch 2 | nb switch 3 | nb rotary | nb axes | nb joy

// MASQUE
//const uint8_t MASQUE_5b=0x1F;  //Plus utilisé
#define EXTERNAL_DISPLAY_ON 0x88
#define EXTERNAL_DISPLAY_OFF 0x80


uint8_t padSwitchStateEnable;
uint8_t padSwitchPulseEnable;
uint8_t padRotaryPulseEnable;
uint8_t padRotaryKnobEnable;
uint8_t padRotaryAxeEnable;

// Last state of the button
uint8_t * lastButtonState;
uint8_t * lastAxisState; 
//uint8_t * rotaryConf; // pulse = 0  | potentiometre = 1  ordre : voir board[]
uint8_t ** axePad; // un tableau par board , index : numéro de l'axe du board, value : numéro du pad
uint8_t ** axeNum; // un tableau par board , index : numéro de l'axe du board, value : numéro du pad

// contenu des écrans
String * displayTxt;
uint8_t displayTotal=0;

/*
********************************************************************************
**********************************GESTION LOCAL********************************
********************************************************************************
*/
// DISPLAY
#define LOCAL_DISPLAY_NB 0
const uint8_t localDisplay[LOCAL_DISPLAY_NB] = {};

/*
// ROTARY ENCODER
#define LOCAL_ROTARY_COUNT  5
// tableau des encoder rotatif (DT,CLK)
Encoder rotaryEncoder[LOCAL_ROTARY_COUNT] ={
  Encoder(0, 8),
  Encoder(1, 9),
  Encoder(2, 10),
  Encoder(3, 11),
  Encoder(7, 12)
};
*/

// BUTTON
#define LOCAL_SINGLE_BUTTON_NB 1
const uint8_t localSingleButton[LOCAL_SINGLE_BUTTON_NB]={6};

//SLIDER
#define LOCAL_SLIDER_NB 0
//const int localSliders[LOCAL_SLIDER_NB] = {A0, A1};
const int localSliders[LOCAL_SLIDER_NB] = {};


//JOYSTICK
#define LOCAL_JOY_NB 0
//const int localJoys[LOCAL_JOY_NB*2] = {A4, A5};
const int localJoys[LOCAL_JOY_NB*2] = {};


void setup() {
  // Attend l'initialisation des autres boards
  //Serial.begin(9600); DEBUG
  delay(2000);
  // init i2c
  Wire.begin(); 
  // find devices
  i2cScann();
  // initialise les tableaux en fonction du nombre de devices
  initTab();
  // récupère les conf (nb de boutons et d'axe de chaque boards)
  getBoardsConf();
  // initialise les variables en fonctions des conf des boards
  initVar();
  // Init axis tab
  initAxis();

   // init lastButtonState
  for (int i = 0; i<buttonTotal; i++){
    lastButtonState[i]=0;
  }

  // init button order
  rotaryPulse1 = realButtonTotal;

  // Init Rotary
  /*
  for(int i = 0;i<rotaryTotal;i++){
      rotaryConf[i]=0; // configure tous les rotary en pulse
  }
  */
 /*
  for(int i = 0;i<rotaryCount[0];i++){
      rotaryEncoder[i].write(0); // configure tous les rotary à 0
  }      
  */

  // Init Button Pins TESTS
  initButton();

  // Init pads 
  for (int i = 0; i <PAD_COUNT;i++)
  {
    pads[i].begin();
  }
  
  setAxisRange();

  // Init screens
  initDisplay();
}

void initTab(){
  // nombre de boutons local(leonardo), mega1, nano1, mega2, nano2
  /*
  rotaryCount[BOARD_COUNT] = {5,6,0,0,0};
  buttonCount[BOARD_COUNT] = {1,28,0,0,0};
  sliderCount[BOARD_COUNT] = {2,2,0,0,0};
  joystickCount[BOARD_COUNT] = {1,0,0,0,0};
  displayCount[BOARD_COUNT] = {0,16,0,0,0};
  */
  rotaryCount= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));
  buttonCount= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));
  sliderCount= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));
  joystickCount= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));
  displayCount= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));
  switch2Count= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));
  switch3Count= (uint8_t*)malloc(BOARD_COUNT*sizeof(uint8_t));
  axePad= (uint8_t**)malloc(BOARD_COUNT*sizeof(uint8_t*));
  axeNum= (uint8_t**)malloc(BOARD_COUNT*sizeof(uint8_t*));
}

void initVar(){
  // Init Button and display count
  for (int i = 0; i<BOARD_COUNT;i++){
    buttonTotal+=(rotaryCount[i]*2)+buttonCount[i];
    realButtonTotal+=buttonCount[i];
    rotaryTotal+=rotaryCount[i];
    joystickTotal+=joystickCount[i];
    sliderTotal+=sliderCount[i];
    axePad[i]=(uint8_t*)malloc((joystickCount[i]*2 + sliderCount[i] + rotaryCount[i]) *sizeof(uint8_t));
    axeNum[i]=(uint8_t*)malloc((joystickCount[i]*2 + sliderCount[i] + rotaryCount[i]) *sizeof(uint8_t));
    displayTotal+=displayCount[i];
  }
  
  // Init & malloc tab
  lastButtonState=(uint8_t*)malloc(buttonTotal*sizeof(uint8_t));
  lastAxisState=(uint8_t*)malloc((sliderTotal+rotaryTotal+joystickTotal*2)*sizeof(uint8_t));
  //rotaryConf=(uint8_t*)malloc(rotaryTotal*sizeof(uint8_t));
  displayTxt=(String*)malloc(displayTotal*sizeof(String));
}

void i2cScann(){
  byte error, address;
  uint8_t nDevices = 0;
  uint8_t devices[127];
  
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error==0 && address!=0x3c)
    {
      nDevices++;
      devices[nDevices]=address;
    }
  }
  BOARD_COUNT=nDevices;
  i2c_addr=(uint8_t*)malloc((BOARD_COUNT)*sizeof(uint8_t));
  for (uint8_t i=0;i<nDevices;i++)
  {
    i2c_addr[i]=devices[i];
  }
}

void getBoardsConf(){
  for (uint8_t j=0;j<BOARD_COUNT;j++)
  {
    Wire.beginTransmission(i2c_addr[j]); 
    uint8_t buff=VIDE;
    uint8_t nbRequete=CONF_BOARDS;
    Wire.write( &buff, 1);          
    Wire.endTransmission();

    Wire.requestFrom(i2c_addr[j], nbRequete);

    displayCount[j]=Wire.read();
    buttonCount[j]=Wire.read();
    switch2Count[j]=Wire.read();
    switch3Count[j]=Wire.read();
    rotaryCount[j]=Wire.read();
    sliderCount[j]=Wire.read();
    joystickCount[j]=Wire.read();
//nb display | nb button | nb switch 2 | nb switch 3 | nb rotary | nb axes | nb joy
    delay(100);
  }

}

void loop() {
  requestButtonUpdate();
  delay(T_CYCLE);
}

void initDisplay(){
  // initialisation des display local des pins en output et en écoute (high)
  uint8_t disp = 0;
  for (disp = 0; disp<LOCAL_DISPLAY_NB;disp++ )
  {
    pinMode(localDisplay[disp],OUTPUT);
    digitalWrite(localDisplay[disp],HIGH);
  }

  // initialisation des display externes des pins en output et en écoute (high)
  for (int i =0 ; i< BOARD_COUNT;i++)
  {
      for (uint8_t j =0 ; j< displayCount[i];j++)
      {
        uint8_t buff[2]={EXTERNAL_DISPLAY_ON,j};
        Wire.beginTransmission(i2c_addr[i]);   
        Wire.write(buff,2);
        Wire.endTransmission(); 


       // sendI2CMsg(msg, i2c_addr[i]); //mets le premier bite de j à 1 pour mettre j en high
      }
  }

  delay(1000); // on attend que tous les écran sont prêts

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(100);
        display.clearDisplay();
        display.setTextSize(2); // Draw 2X-scale text
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println(F("test"));
        display.display();

  for (int i = 0; i<LOCAL_DISPLAY_NB;i++ )
  {
    digitalWrite(localDisplay[i],LOW);
  }

  // mets les display externe en low
  for (int i =0 ; i< BOARD_COUNT;i++)
  {
      for (uint8_t j =0 ; j< displayCount[i];j++)
      {
        uint8_t buff[2]={EXTERNAL_DISPLAY_OFF,j};
        Wire.beginTransmission(i2c_addr[i]);   
        Wire.write(buff,2);
        Wire.endTransmission(); 
      }
  }

  //  rempli le tableau de base des écrans
  for (int i = 0; i<displayTotal; i++)
  {
    displayTxt[i]=i;
  }

  delay(1000); // on attend que tous les écran sont prêts

  refreshScreen();
}

void confScreen(uint8_t confN){
  // initialisation des display local des pins en output et en écoute (high)

  int disp = 0;
  while (disp<LOCAL_DISPLAY_NB)
  {
      digitalWrite(localDisplay[disp],HIGH);
      display.clearDisplay();
      if (displayText[confN][disp].length()<4)
      {
        display.setTextSize(5);
      }
      else if (displayText[confN][disp].length()<7)
      {
        display.setTextSize(3);
      }
      else
      {
        display.setTextSize(1);
      }
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      //display.println(F(displayText[confN][disp]));
      display.println(displayText[confN][disp]);
      display.display();
      digitalWrite(localDisplay[disp],LOW);
      delay(100);
      disp++;
  }
  // initialisation des display externes des pins en output et en écoute (high)
  for (int i =0 ; i< BOARD_COUNT;i++)
  {
      for (uint8_t j =0 ; j< displayCount[i];j++)
      {
        uint8_t buff[2]={EXTERNAL_DISPLAY_ON,j};
        Wire.beginTransmission(i2c_addr[i]);   
        Wire.write(buff,2);
        Wire.endTransmission(); 

        delay(100);

        display.clearDisplay();
        if (displayText[confN][disp].length()<4)
        {
          display.setTextSize(5);
        }
        else if (displayText[confN][disp].length()<7)
        {
          display.setTextSize(3);
        }
        else
        {
          display.setTextSize(1);
        }
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        //display.println(F(displayText[confN][disp]));
        display.println(displayText[confN][disp]);
        display.display();      // Show initial text
        delay(100);
        uint8_t buff2[2]={EXTERNAL_DISPLAY_OFF,j};
        Wire.beginTransmission(i2c_addr[i]);   
        Wire.write(buff2,2);
        Wire.endTransmission(); 
        disp++;
      }
  }
}

void refreshScreen(){
  // initialisation des display local des pins en output et en écoute (high)

  int disp = 0;
  while (disp<LOCAL_DISPLAY_NB)
  {
      digitalWrite(localDisplay[disp],HIGH);
      display.clearDisplay();
      display.setTextSize(5); // Draw 2X-scale text
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      //display.println(F("test"));
      display.println(disp);
      display.display();      // Show initial text
      digitalWrite(localDisplay[disp],LOW);
      delay(100);
      disp++;
  }
  // initialisation des display externes des pins en output et en écoute (high)
  for (int i =0 ; i< BOARD_COUNT;i++)
  {
      for (uint8_t j =0 ; j< displayCount[i];j++)
      {/*
        uint16_t msg = j;
        msg|=EXTERNAL_DISPLAY_ON;
        sendI2CMsg(msg, i2c_addr[i]); //mets le premier bite de j à 1 pour mettre j en high
*/
        uint8_t buff[2]={EXTERNAL_DISPLAY_ON,j};
        Wire.beginTransmission(i2c_addr[i]);   
        Wire.write(buff,2);
        Wire.endTransmission(); 

        delay(100);

        display.clearDisplay();
        display.setTextSize(3); // Draw 2X-scale text
       //display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
       // display.println(F("test"));
        display.println(disp);
        display.display();      // Show initial text

        delay(100);
        uint8_t buff2[2]={EXTERNAL_DISPLAY_OFF,j};
        Wire.beginTransmission(i2c_addr[i]);   
        Wire.write(buff2,2);
        Wire.endTransmission(); 
        disp++;
      }
  }
}

void initButton(){
  for (uint8_t i =0; i<LOCAL_SINGLE_BUTTON_NB; i++)
  {
    pinMode(localSingleButton[i], INPUT_PULLUP);
  }
}

// TODO test
// void initAxis2() dans l'ordre on s'en fout des joy
void initAxis(){
  for (uint8_t i = 0; i<LOCAL_SLIDER_NB; i++)
  {
     pinMode(localSliders[i], INPUT);
  }

  for (uint8_t i = 0; i<(LOCAL_JOY_NB*2); i++)
  {
     pinMode(localJoys[i], INPUT);
  }
  
  uint8_t axeCount = 0;
  uint8_t boardCount = 0;

  // Init Axe tab
  for (uint8_t i=0;i<PAD_COUNT;i++)
  {
    for (uint8_t j = 0; j<11 && boardCount<BOARD_COUNT;j++)
    {
      axePad[boardCount][axeCount]=i;
      axeNum[boardCount][axeCount]=j;
      axeCount++;
      if (axeCount>=(joystickCount[boardCount]*2+sliderCount[boardCount]+rotaryCount[boardCount]))
      {
        axeCount=0;
        boardCount++;
      }
    }
  }
}

void initAxis2(){

  for (uint8_t i = 0; i<LOCAL_SLIDER_NB; i++)
  {
     pinMode(localSliders[i], INPUT);
  }

    for (uint8_t i = 0; i<(LOCAL_JOY_NB*2); i++)
  {
     pinMode(localJoys[i], INPUT);
  }
  
  uint8_t axeConf[PAD_COUNT][11];  
  uint8_t padCount = 0;
  uint8_t axeCount = 0;

  for (uint8_t i=0;i<PAD_COUNT;i++)
  {
    for (uint8_t j = 0; j<11;j++)
    {
      axeConf[i][j]=1;
    }
  }

  // Fill Axe tab with joy

  for (uint8_t i=0;i<BOARD_COUNT;i++)
  {
    uint8_t j = 0;
    while(j<(joystickCount[i]) && padCount<PAD_COUNT && axeCount<11)
    {
      axePad[i][j*2]=padCount;
      axePad[i][(j*2)+1]=padCount;
      axeNum[i][j*2]=axeCount;
      axeConf[padCount][axeCount]=0;
      axeCount++;
      axeNum[i][(j*2)+1]=axeCount;
      axeConf[padCount][axeCount]=0;
      axeCount++;
      if (axeCount==4)
      {
        axeCount=0;
        padCount++;
      }
      j++;
    }
  }

  // Fill Axe tab with slider
  padCount = 0;
  axeCount = 0;
  for (uint8_t i=0;i<BOARD_COUNT;i++)
  {
    uint8_t j = 0;
    while(j<(sliderCount[i]) && padCount<PAD_COUNT && axeCount<11)
    {
      while(axeConf[padCount][axeCount]==0)
      {
        axeCount++;
        if(axeCount>10)
        {
          axeCount=0;
          padCount++;
        }
      }
        axePad[i][j]=padCount;
        axeNum[i][j]=axeCount;
        axeConf[padCount][axeCount]=0;
        axeCount++;
        if(axeCount>10){
          axeCount=0;
          padCount++;
        }
        j++;
      }
    }

  // Fill Axe tab with rotary
  for (uint8_t i=0;i<BOARD_COUNT;i++)
  {
    uint8_t j = 0;
    while(j<(rotaryCount[i]) && padCount<PAD_COUNT && axeCount<11)
    {
      while(axeConf[padCount][axeCount]==0)
      {
        axeCount++;
        if(axeCount>10)
        {
          axeCount=0;
          padCount++;
        }
      }
      axePad[i][j]=padCount;
      axeNum[i][j]=axeCount;
      axeConf[padCount][axeCount]=0;
      axeCount++;
      if(axeCount>10){
          axeCount=0;
          padCount++;
      }
      j++;
    }
  }
}
    
void requestButtonUpdate()
{
  // Read Joys values
  getLocalJoy();

  // Read slider values
  getLocalSlider();

  // Read rotary encoder
//  getLocalRotary();

  // Read local button values
  getLocalButton();
  
//  requestMega1Update();
//  requestMega2Update();
//  requestBoardsUpdate();
  delay(T_RAZROT);

  // RAZ rotary encoder 
  for(int i=rotaryTotal; i<BUTTON_MAX ;i++)
  {
    pads[0].setButton(i,0);

  }
}

void getLocalJoy()
{
  for(uint8_t i=0;i<(LOCAL_JOY_NB*2);i++)
  {
      updateAxis(0,i,analogRead(localJoys[i])/4);
  }
}

void getLocalSlider()
{
  for(uint8_t i=0;i<LOCAL_SLIDER_NB;i++)
  {
      updateAxis(0,LOCAL_JOY_NB*2+i,analogRead(localSliders[i])/4);
  }
}

void getLocalButton(){
  for (uint8_t i =0; i<LOCAL_SINGLE_BUTTON_NB; i++)
  {
    int currentButtonState = !digitalRead(localSingleButton[i]);
    if (currentButtonState != lastButtonState[i])
    {
      pads[0].setButton(i, currentButtonState);
      lastButtonState[i] = currentButtonState;
    }
  }
}

/*
void getLocalRotary(){
  long rotaryRead;
  for(int i = 0; i<rotaryCount[0]; i++){
    rotaryRead = rotaryEncoder[i].read();
    if (rotaryConf[i]==0) //Rotary pulse
    {
      if (rotaryRead<0) {
        pads[0].setButton(rotaryPulse1+(i*2),1);
        rotaryEncoder[i].write(0);
      }
      else if (rotaryRead>0) {
        pads[0].setButton(rotaryPulse1+(i*2)+1,1);
        rotaryEncoder[i].write(0);
      }
    }
    else // Rotary pot
    {
        int rotaryWrite;
        if (rotaryRead>maxRotaryPot)
        {
            rotaryEncoder[i].write(maxRotaryPot);
            rotaryWrite = maxRotaryPot;
        }
        else if(rotaryRead<minRotaryPot)
        {
            rotaryEncoder[i].write(minRotaryPot);
            rotaryWrite = minRotaryPot;
        }
        else
        {
            rotaryWrite=rotaryRead;
        }
        updateAxis(0,i+(joystickCount[0]*2)+sliderCount[0],map(rotaryWrite,minRotaryPot,maxRotaryPot,0,255));
    }
  }
}
*/

void setAxisRange()
{
  for (uint8_t joyNum=0; joyNum<PAD_COUNT; joyNum++)
  {
    pads[joyNum].setXAxisRange(0,255);
    pads[joyNum].setYAxisRange(0,255);
    pads[joyNum].setZAxisRange(0,255);
    pads[joyNum].setRxAxisRange(0,255);
    pads[joyNum].setRyAxisRange(0,255);
    pads[joyNum].setRzAxisRange(0,255);
    pads[joyNum].setRudderRange(0,255);
    pads[joyNum].setThrottleRange(0,255);
    pads[joyNum].setAcceleratorRange(0,255);
    pads[joyNum].setBrakeRange(0,255);
    pads[joyNum].setSteeringRange(0,255);
  }
}

void updateAxis(uint8_t board,uint8_t number,uint8_t value) 
{
  switch(axeNum[board][number])
  {
    case 0 :  pads[axePad[board][number]].setXAxis(value);     break;
    case 1 :  pads[axePad[board][number]].setYAxis(value);     break;
    case 2 :  pads[axePad[board][number]].setRxAxis(value);    break;
    case 3 :  pads[axePad[board][number]].setRyAxis(value);    break;
    case 4 :  pads[axePad[board][number]].setZAxis(value);     break;
    case 5 :  pads[axePad[board][number]].setRzAxis(value);    break;
    case 6 :  pads[axePad[board][number]].setRudder(value);    break;
    case 7 :  pads[axePad[board][number]].setThrottle(value);  break;
    case 8 :  pads[axePad[board][number]].setAccelerator(value);break;
    case 9 :  pads[axePad[board][number]].setBrake(value);     break;
    case 10 : pads[axePad[board][number]].setSteering(value);  break;
  }
}

void requestBoardsUpdate(){
  uint8_t data_count;
  uint8_t j;
  uint8_t nbResponse;
  uint8_t response[nbResponse];

  for (int i=0;i<BOARD_COUNT;i++)
  {

    j=0;
    Wire.requestFrom(i2c_addr[i], nbResponse);   
    while (j<data_count && Wire.available())
    {
      response[j]=Wire.read();
      j++;
    }

    if (response[0]==TYPE_BUTTON)
    {
      //TODO trouver le numéro global à partir du bouton local
      lastButtonState[response[1]]=response[2];
      pads[0].setButton(response[1], response[2]);
    }
    else if (response[0]==TYPE_JOY)
    {
      //TODO
    }    
    else if (response[0]==TYPE_SLIDER)
    {
      //TODO
    }   
    else if (response[0]==TYPE_ROTARY)
    {
      //TODO
    }   
    else if (response[0]==TYPE_SWITCH2)
    {
      //TODO
    }
    else if (response[0]==TYPE_SWITCH3)
    {
      //TODO
    }          
  }
}

/*
void requestMega1Update(){
  uint8_t j=1;
  uint8_t data_count=sliderCount[j]+ joystickCount[j]*2 + rotaryCount[j] +2;

  uint8_t response1[data_count];
  uint8_t i;

  // RECUP DATA MEGA 1
  for(i=0;i<data_count;i++){response1[i]=0;}
  i=0;
  Wire.requestFrom(i2c_addr[j], data_count);   
  while (i<MEGA1_DATA_COUNT && Wire.available()){
    response1[i]=Wire.read();
    i++;
  }
  pads.setRudder(response1[0]);
  pads.setThrottle(response1[1]);
  pads.setAccelerator(response1[2]);
  pads.setBrake(response1[3]);
  i=4;
  while (i<data_count){
    lastButtonState[response1[i]]=response1[i+1];
    pads.setButton(response1[i], response1[i+1]);
    i=i+2;
  }

}

void requestMega2Update(){
  uint8_t j=3;
  uint8_t data_count=sliderCount[j]+ joystickCount[j]*2 + rotaryCount[j] +2;

  uint8_t response1[data_count];
  uint8_t i;

  // RECUP DATA MEGA 1
  for(i=0;i<data_count;i++){response1[i]=0;}
  i=0;
  Wire.requestFrom(i2c_addr[j], data_count);   
  while (i<MEGA1_DATA_COUNT && Wire.available()){
    response1[i]=Wire.read();
    i++;
  }
  i=0;
  while (i<data_count){
    lastButtonState[response1[i]]=response1[i+1];
    pads.setButton(response1[i], response1[i+1]);
    i=i+2;
  }

}
*/
void sendI2CMsg(uint16_t msg, int adresse){ 
  Wire.beginTransmission(adresse); 
  Wire.write((byte *)&msg, 2);          
  Wire.endTransmission(); 
}

void screenDebug(String msg){
      digitalWrite(localDisplay[0],HIGH);
      display.clearDisplay();
      display.setTextSize(5); // Draw 2X-scale text
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      //display.println(F("test"));
      display.println(msg);
      display.display();      // Show initial text
      digitalWrite(localDisplay[0],LOW);
}
