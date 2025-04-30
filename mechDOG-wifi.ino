/*
Modified for WiFi control by Adam Harnisch
Original Author: Edu Dickel (https://www.youtube.com/mechdickel)
Description: WiFi-controlled walking gait (trot), turnings, and poses for mechDOG
*/

#include <LSS.h>
#include <WiFiS3.h>

float initPos11 =      -125;
float initPos12 =      -295;
float initPos13 =       180;
float initPos21 = initPos11;
float initPos22 = initPos12;
float initPos23 = initPos13;
float initPos31 = initPos11;
float initPos32 = initPos12;
float initPos33 = initPos13;
float initPos41 = initPos11;
float initPos42 = initPos12;
float initPos43 = initPos13;

float currPos11, prevPos11, targPos11;
float currPos12, prevPos12, targPos12;
float currPos13, prevPos13, targPos13;
float currPos21, prevPos21, targPos21;
float currPos22, prevPos22, targPos22;
float currPos23, prevPos23, targPos23;
float currPos31, prevPos31, targPos31;
float currPos32, prevPos32, targPos32;
float currPos33, prevPos33, targPos33;
float currPos41, prevPos41, targPos41;
float currPos42, prevPos42, targPos42;
float currPos43, prevPos43, targPos43;

// Offsets for the calibration JIG
  float offset11 =    0;
  float offset12 = -450;
  float offset13 =  900;
  float offset21 =    0;
  float offset22 = -450;
  float offset23 =  900;
  float offset31 =    0;
  float offset32 = -450;
  float offset33 =  900;
  float offset41 =    0;
  float offset42 = -450;
  float offset43 =  900;

byte i;   // increment() counter
byte ird; // increment rate divider
byte state;
byte prevState;
byte phase = 1;
byte phaseDelay = 50;
byte queryDelay = 2;

bool wakeUpDone = false; // to indicate if wakeUp() has reached its target positions
bool moving = false;     // indicates if it is running a motion function

// WiFi credentials
const char* ssid = "Wifiname";
const char* password = "password";
WiFiServer server(80);  // HTTP server on port 80

// Servo definitions
LSS myLSS11 = LSS(11); // RR hip roll
LSS myLSS12 = LSS(12); // RR hip pitch
LSS myLSS13 = LSS(13); // RR knee
LSS myLSS21 = LSS(21); // RF hip roll
LSS myLSS22 = LSS(22); // RF hip pitch
LSS myLSS23 = LSS(23); // RF knee
LSS myLSS31 = LSS(31); // LR hip roll
LSS myLSS32 = LSS(32); // LR hip pitch
LSS myLSS33 = LSS(33); // LR knee
LSS myLSS41 = LSS(41); // LF hip 
LSS myLSS42 = LSS(42); // LF hip pitch
LSS myLSS43 = LSS(43); // LF knee

// Movement states 
enum MovementState {
  STOPPED,
  WAKEUP,
  TROT_FORWARD,
  TROT_F_SLOW,
  TROT_BACKWARD,
  TURN_RIGHT,
  TURN_LEFT,
  TROT_RIGHT,
  TROT_LEFT,
  TROT_P,
  PITCH_UP,
  PITCH_DOWN,
  ROLL_RIGHT,
  ROLL_LEFT,
  REST_POSITION
};

MovementState currentState = STOPPED;
MovementState previousState = STOPPED;

void setup() { 
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Initialize LSS bus (using Serial1 for servos)
  Serial1.begin(115200);
  LSS::initBus(Serial1, 115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  
  server.begin();

  // Gyre Direction
  Serial1.write("#11CG1\r#12CG1\r#13CG1\r");    // RR leg servos = CW
  Serial1.write("#21CG1\r#22CG1\r#23CG1\r");    // RF leg servos = CW
  Serial1.write("#31CG-1\r#32CG-1\r#33CG-1\r"); // LR leg servos = CCW
  Serial1.write("#41CG-1\r#42CG-1\r#43CG-1\r"); // LF leg servos = CCW

  // Enable Motion Profile
  Serial1.write("#254CEM0\r");  // EM0 = motion profile disabled

  // Filter Position Count
  Serial1.write("#254FPC5");    // default = 5
  
  // Angular Stiffness
  Serial1.write("#254CAS-6\r"); // default = 0 & works with -6

  // Angular Holding Stiffness
  Serial1.write("#254CAH4\r");  // default = 4 & works with 4

  // LED Color
  Serial1.write("#254CLED3\r"); // 3 = blue
  
  Serial1.write("#254RESET\r");   
  delay(2000);  // wait 2 sec for reboot

  queryPos();   // get positions at startup
  setPrevPos();
  
  // Start in rest position
  rest();
}

void loop() {
  // Handle WiFi clients
  WiFiClient client = server.available();
  
  if (client) {
    // Read the HTTP request
    String request = client.readStringUntil('\r');
    client.flush();
    
    // Movement Commands
    if (request.indexOf("/wakeup") != -1) {
      wakeUpDone = false; // Reset wakeup state
      currentState = WAKEUP;
    } 
    else if (request.indexOf("/forward") != -1) {
      currentState = TROT_FORWARD;
    }
    else if (request.indexOf("/forwardslow") != -1) {
      currentState = TROT_F_SLOW;
    }
    else if (request.indexOf("/backward") != -1) {
      currentState = TROT_BACKWARD;
    }
    else if (request.indexOf("/right") != -1) {
      currentState = TURN_RIGHT;
    }
    else if (request.indexOf("/left") != -1) {
      currentState = TURN_LEFT;
    }
    else if (request.indexOf("/sideright") != -1) {
      currentState = TROT_RIGHT;
    }
    else if (request.indexOf("/sideleft") != -1) {
      currentState = TROT_LEFT;
    }
    else if (request.indexOf("/trotplace") != -1) {
      currentState = TROT_P;
    }
    
    // Orientation Commands
    else if (request.indexOf("/pitchup") != -1) {
      currentState = PITCH_UP;
    }
    else if (request.indexOf("/pitchdown") != -1) {
      currentState = PITCH_DOWN;
    }
    else if (request.indexOf("/rollright") != -1) {
      currentState = ROLL_RIGHT;
    }
    else if (request.indexOf("/rollleft") != -1) {
      currentState = ROLL_LEFT;
    }
    
    // System Commands
    else if (request.indexOf("/stop") != -1) {
      currentState = STOPPED;
    }
    else if (request.indexOf("/rest") != -1) {
      currentState = REST_POSITION;
    }
    
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();

    client.println("<h1>MechDog Control</h1>");

    // Movement Section
    client.println("<h2>motion</h2>");
    client.println("<p><a href='/forward'>Forward</a> ");
    client.println("<a href='/backward'>Backward</a></p>");

    client.println("<p><a href='/right'>Turn Right</a> ");
    client.println("<a href='/left'>Turn Left</a> ");
    client.println("<a href='/sideright'>Side Right</a> ");
    client.println("<a href='/sideleft'>Side Left</a></p>");

    // Orientation Section
    client.println("<h2>Orientation</h2>");
    client.println("<p><a href='/pitchup'>Pitch Up</a> ");
    client.println("<a href='/pitchdown'>Pitch Down</a> ");
    client.println("<a href='/rollright'>Roll Right</a> ");
    client.println("<a href='/rollleft'>Roll Left</a></p>");

    // System Commands
    client.println("<h2>System</h2>");
    client.println("<p><a href='/wakeup'>Wake Up</a> ");
    client.println("<a href='/stop'>Stop</a> ");
    client.println("<a href='/rest'>Rest</a></p>");

    delay(10);
    client.stop();
  }
  // Execute movement based on current state
  switch(currentState) {
    case TROT_FORWARD: trot_F(); break;
    case TROT_F_SLOW: trot_F_SLOW(); break;
    case TROT_BACKWARD: trot_B(); break;
    case TURN_RIGHT: turn_R(); break;
    case TURN_LEFT: turn_L(); break;
    case TROT_RIGHT: trot_R(); break;
    case TROT_LEFT: trot_L(); break;
    case TROT_P: trot_P(); break;
    case PITCH_UP: look_U(); break;
    case PITCH_DOWN: look_D(); break;
    case ROLL_RIGHT: roll_R(); break;
    case ROLL_LEFT: roll_L(); break;
    case REST_POSITION: rest(); break;
    case STOPPED: 
    default: Stop(); break;
  }
}

void queryPos() // query positions and set them as current positions
{
  currPos11 = myLSS11.getPosition() + offset11; delay(queryDelay);
  currPos12 = myLSS12.getPosition() + offset12; delay(queryDelay);
  currPos13 = myLSS13.getPosition() + offset13; delay(queryDelay);
  currPos21 = myLSS21.getPosition() + offset21; delay(queryDelay);
  currPos22 = myLSS22.getPosition() + offset22; delay(queryDelay);
  currPos23 = myLSS23.getPosition() + offset23; delay(queryDelay);
  currPos31 = myLSS31.getPosition() + offset31; delay(queryDelay);
  currPos32 = myLSS32.getPosition() + offset32; delay(queryDelay);
  currPos33 = myLSS33.getPosition() + offset33; delay(queryDelay);
  currPos41 = myLSS41.getPosition() + offset41; delay(queryDelay);
  currPos42 = myLSS42.getPosition() + offset42; delay(queryDelay);
  currPos43 = myLSS43.getPosition() + offset43; delay(queryDelay);
}

void wakeUp()
{
  moving = true;
  ird = 100;

  if (phase == 1)
    {
      targPos11 =  -90;   targPos21 = targPos11;   targPos31 = targPos11;   targPos41 = targPos11; 
      targPos12 = -450;   targPos22 = targPos12;   targPos32 = targPos12;   targPos42 = targPos12;
      targPos13 =  900;   targPos23 = targPos13;   targPos33 = targPos13;   targPos43 = targPos13;

  if (i < ird)
    {
      increment();
      i++;
    }

  else 
    {
      setPrevPos();
      delay(1000);
      phase = 2;
      i = 0;
    }          
  }


  else if (phase == 2)
  {
  targPos11 = initPos11;   targPos21 = initPos21;   targPos31 = initPos31;   targPos41 = initPos41; 
  targPos12 = initPos12;   targPos22 = initPos22;   targPos32 = initPos32;   targPos42 = initPos42;
  targPos13 = initPos13;   targPos23 = initPos23;   targPos33 = initPos33;   targPos43 = initPos43;

  if (i < ird)
    {
      increment();
      i++;
    }

  else 
    {
      wakeUpDone = true;
    }          
  }

  updatePos();
}

void trot_F()
{
  moving = true;
  ird = 10; // define in how many increments each "phase" will be divided (the higher the "ird", the slower the movement)

  if (phase == 1)
  {
    // target positions for phase 1
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -505;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  495;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();       // run increment function until target positions get reached
      i++;
    }
    else
    {
      delay(phaseDelay); // wait a little for it to get there
      setPrevPos();      // store current positions ("currPos") as previous positions ("prevPos") for the next function
      phase = 2;         // and proceed to phase 2
      i = 0;             // reset "i" for the next phase
    }
  }

  else if (phase == 2)
  {
    // target positions for phase 2
    // RR leg FRONT     // RF leg BACK      // LR leg BACK           // LF leg FRONT
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 =   -5;   targPos22 = -395;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 = -105;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos();
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP             // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -505;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  495;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos();
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg BACK      // RF leg FRONT     // LR leg FRONT          // LF leg BACK
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -395;   targPos22 =   -5;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 = -105;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos(); // if last phase reach its target positions ("targPos")...
      phase = 1;    // ... return to phase 1
      i = 0;
    }
  }

  updatePos();
}

// Trot 10
void trot_F_SLOW()
{
  moving = true;
  ird = 8; // define in how many increments each "phase" will be divided (the higher the "ird", the slower the movement)

  if (phase == 1)
  {
    // target positions for phase 1
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -380;   targPos22 = -220;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  545;   targPos23 =  240;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();       // run increment function until target positions get reached
      i++;
    }
    else
    {
      delay(phaseDelay); // wait a little for it to get there
      setPrevPos();      // store current positions ("currPos") as previous positions ("prevPos") for the next function
      phase = 2;         // and proceed to phase 2
      i = 0;             // reset "i" for the next phase
    }
  }

  else if (phase == 2)
  {
    // target positions for phase 2
    // RR leg FRONT     // RF leg BACK      // LR leg BACK           // LF leg FRONT
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 =  -15;   targPos22 = -365;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  205;   targPos23 =  145;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos();
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP             // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -220;   targPos22 = -380;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  240;   targPos23 =  545;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos();
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg BACK      // RF leg FRONT     // LR leg FRONT          // LF leg BACK
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -365;   targPos22 =  -15;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  145;   targPos23 =  205;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos(); // if last phase reach its target positions ("targPos")...
      phase = 1;    // ... return to phase 1
      i = 0;
    }
  }

  updatePos();
}

// Trot Backward NEW
void trot_B()
{
  moving = true;
  ird = 10; // define in how many increments each "phase" will be divided (the higher the "ird", the slower the movement)

  if (phase == 1)
  {
    // target positions for phase 1
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -415;   targPos22 = -245;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  540;   targPos23 =  230;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();       // run increment function until target positions get reached
      i++;
    }
    else
    {
      delay(phaseDelay); // wait a little for it to get there
      setPrevPos();      // store current positions ("currPos") as previous positions ("prevPos") for the next function
      phase = 2;         // and proceed to phase 2
      i = 0;             // reset "i" for the next phase
    }
  }

  else if (phase == 2)
  {
    // target positions for phase 2
    // RR leg B         // RF leg F         // LR leg F              // LF leg B
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -375;   targPos22 =  -45;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  130;   targPos23 =  215;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos();
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP             // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -245;   targPos22 = -415;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  230;   targPos23 =  540;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos();
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg F         // RF leg B         // LR leg B              // LF leg F
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 =  -45;   targPos22 = -375;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  215;   targPos23 =  130;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos(); // if last phase reach its target positions ("targPos")...
      phase = 1;    // ... return to phase 1
      i = 0;
    }
  }

  updatePos();
}

void turn_R()
{
  moving = true;
  ird = 8;

  if (phase == 1)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP             // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -505;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  495;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 2;
      i = 0;
    }
  }

  else if (phase == 2)
  {
    // RR leg OUT       // RF leg OUT       // LR leg OUT            // LF leg OUT
    targPos11 =   -5;   targPos21 =   -5;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -250;   targPos22 = -250;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  105;   targPos23 =  105;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -505;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  495;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg IN        // RF leg IN        // LR leg IN             // LF leg IN
    targPos11 = -245;   targPos21 = -245;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -310;   targPos22 = -310;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  200;   targPos23 =  200;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 1;
      i = 0;
    }
  }

  updatePos();
}

void turn_L()
{
  moving = true;
  ird = 8;

  if (phase == 1)
  {
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -505;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  495;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 2;
      i = 0;
    }
  }

  else if (phase == 2)
  {
    // RR leg OUT       // RF leg OUT       // LR leg OUT            // LF leg OUT
    targPos11 =   -5;   targPos21 =   -5;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -250;   targPos22 = -250;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  105;   targPos23 =  105;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP           // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -505;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  495;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg IN        // RF leg IN        // LR leg IN             // LF leg IN
    targPos11 = -245;   targPos21 = -245;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -310;   targPos22 = -310;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  200;   targPos23 =  200;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 1;
      i = 0;
    }
  }

  updatePos();
}

void trot_R()
{
  moving = true;
  ird = 8;

  if (phase == 1)
  {
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -505;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  495;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 2;
      i = 0;
    }
  }

  else if (phase == 2)
  {
    // RR leg OUT       // RF leg IN        // LR leg OUT            // LF leg IN
    targPos11 =   -5;   targPos21 = -245;   targPos31 = targPos11;   targPos41 = targPos21;
    targPos12 = -250;   targPos22 = -310;   targPos32 = targPos12;   targPos42 = targPos22;
    targPos13 =  105;   targPos23 =  200;   targPos33 = targPos13;   targPos43 = targPos23;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP           // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -505;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  495;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg IN        // RF leg OUT       // LR leg IN             // LF leg OUT
    targPos11 = -245;   targPos21 =   -5;   targPos31 = targPos11;   targPos41 = targPos21;
    targPos12 = -310;   targPos22 = -250;   targPos32 = targPos12;   targPos42 = targPos22;
    targPos13 =  200;   targPos23 =  105;   targPos33 = targPos13;   targPos43 = targPos23;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 1;
      i = 0;
    }
  }

  updatePos();
}

void trot_L()
{
  moving = true;
  ird = 8;

  if (phase == 1)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP             // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -505;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  495;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 2;
      i = 0;
    }
  }

  else if (phase == 2)
  {
    // RR leg OUT       // RF leg IN        // LR leg OUT            // LF leg IN
    targPos11 =   -5;   targPos21 = -245;   targPos31 = targPos11;   targPos41 = targPos21;
    targPos12 = -250;   targPos22 = -310;   targPos32 = targPos12;   targPos42 = targPos22;
    targPos13 =  105;   targPos23 =  200;   targPos33 = targPos13;   targPos43 = targPos23;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -505;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  495;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg IN        // RF leg OUT       // LR leg IN             // LF leg OUT
    targPos11 = -245;   targPos21 =   -5;   targPos31 = targPos11;   targPos41 = targPos21;
    targPos12 = -310;   targPos22 = -250;   targPos32 = targPos12;   targPos42 = targPos22;
    targPos13 =  200;   targPos23 =  105;   targPos33 = targPos13;   targPos43 = targPos23;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 1;
      i = 0;
    }
  }

  updatePos();
}

void trot_P()
{
  moving = true;
  ird = 10;
  if (phase == 1)
  {
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -505;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  495;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 2;
      i = 0;
    }
  }

  else if (phase == 2)
  {
    // RR leg DOWN      // RF leg DOWN      // LR leg DOWN           // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP             // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -505;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  495;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg DOWN      // RF leg DOWN      // LR leg DOWN           // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 1;
      i = 0;
    }
  }

  updatePos();
}

void look_U()
{
  moving = true;
  ird = 100;

  targPos11 = -125;   targPos21 =  -85;   targPos31 = targPos11;   targPos41 = targPos21;
  targPos12 = -440;   targPos22 =  -90;   targPos32 = targPos12;   targPos42 = targPos22;
  targPos13 =  145;   targPos23 = -445;   targPos33 = targPos13;   targPos43 = targPos23;

  if (i < ird)
  {
    increment();
    i++;
  }

  updatePos();
}

void look_D()
{
  moving = true;
  ird = 100;

  targPos11 =  -85;   targPos21 = -125;   targPos31 = targPos11;   targPos41 = targPos21;
  targPos12 =  200;   targPos22 = -180;   targPos32 = targPos12;   targPos42 = targPos22;
  targPos13 = -480;   targPos23 =  270;   targPos33 = targPos13;   targPos43 = targPos23;

  if (i < ird)
  {
    increment();
    i++;
  }

  updatePos();
}

void roll_R()
{
  moving = true;
  ird = 40;

  targPos11 =  185;   targPos21 = targPos11;   targPos31 = -385;   targPos41 = targPos31;
  targPos12 = -395;   targPos22 = targPos12;   targPos32 = -170;   targPos42 = targPos32;
  targPos13 =  335;   targPos23 = targPos13;   targPos33 =  -35;   targPos43 = targPos33;

  if (i < ird)
  {
    increment();
    i++;
  }

  updatePos();
}

void roll_L()
{
  moving = true;
  ird = 40;

  targPos11 = -385;   targPos21 = targPos11;   targPos31 =  185;   targPos41 = targPos31;
  targPos12 = -170;   targPos22 = targPos12;   targPos32 = -395;   targPos42 = targPos32;
  targPos13 =  -35;   targPos23 = targPos13;   targPos33 =  335;   targPos43 = targPos33;

  if (i < ird)
  {
    increment();
    i++;
  }

  updatePos();
}

void rest()
{
  moving = true;
  ird = 100;

  targPos11 = -125;   targPos21 = targPos11;   targPos31 = targPos11;   targPos41 = targPos11;
  targPos12 = -450;   targPos22 = targPos12;   targPos32 = targPos12;   targPos42 = targPos12;
  targPos13 =  900;   targPos23 = targPos13;   targPos33 = targPos13;   targPos43 = targPos13;

  if (i < ird)
  {
    increment();
    i++;
  }

  updatePos();
}

void increment() // divides the difference between target positions and previous positions in small increments
{
  currPos11 += (targPos11 - prevPos11)/(ird);
  currPos12 += (targPos12 - prevPos12)/(ird);
  currPos13 += (targPos13 - prevPos13)/(ird);
  currPos21 += (targPos21 - prevPos21)/(ird);
  currPos22 += (targPos22 - prevPos22)/(ird);
  currPos23 += (targPos23 - prevPos23)/(ird);
  currPos31 += (targPos31 - prevPos31)/(ird);
  currPos32 += (targPos32 - prevPos32)/(ird);
  currPos33 += (targPos33 - prevPos33)/(ird);
  currPos41 += (targPos41 - prevPos41)/(ird);
  currPos42 += (targPos42 - prevPos42)/(ird);
  currPos43 += (targPos43 - prevPos43)/(ird);
}

void setPrevPos() // set the current positions as previous positions for the next function 
{
 prevPos11 = currPos11;
 prevPos12 = currPos12;
 prevPos13 = currPos13;
 prevPos21 = currPos21;
 prevPos22 = currPos22;
 prevPos23 = currPos23;
 prevPos31 = currPos31;
 prevPos32 = currPos32;
 prevPos33 = currPos33;
 prevPos41 = currPos41;
 prevPos42 = currPos42;
 prevPos43 = currPos43;
}

void updatePos() // move to new calculated positions
{

//  Serial.println(state);
  
  myLSS11.move(currPos11 - offset11);
  myLSS12.move(currPos12 - offset12);
  myLSS13.move(currPos13 - offset13);
  myLSS21.move(currPos21 - offset21);
  myLSS22.move(currPos22 - offset22);
  myLSS23.move(currPos23 - offset23);
  myLSS31.move(currPos31 - offset31);
  myLSS32.move(currPos32 - offset32);
  myLSS33.move(currPos33 - offset33);
  myLSS41.move(currPos41 - offset41);
  myLSS42.move(currPos42 - offset42);
  myLSS43.move(currPos43 - offset43);
}

void Stop() // runs between one function and other to make sure we start from "common" positions (i.e., initial positions)
{ 
  ird = 50;
  
  if (moving == true)
    {
      setPrevPos();
      
      targPos11 = initPos11;   targPos21 = initPos21;   targPos31 = initPos31;   targPos41 = initPos41; 
      targPos12 = initPos12;   targPos22 = initPos22;   targPos32 = initPos32;   targPos42 = initPos42;
      targPos13 = initPos13;   targPos23 = initPos23;   targPos33 = initPos33;   targPos43 = initPos43;
      
      for (int s = 0; s <= ird; s++)
        {
          increment();
          updatePos();
        }

      moving = false; // once Stop() reaches its target positions
    }

  else if (moving == false)
    {  
      setPrevPos();  
      i = 0;     // reset increment counter for next function
      phase = 1; // reset phase indicator for next function
    }
}