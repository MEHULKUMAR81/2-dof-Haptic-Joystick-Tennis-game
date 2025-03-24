import processing.serial.*;

Serial myPort;  // The Serial port object
int updatedPosY = 0;  // Variable to store Y position from Arduino (racket position)
int updatedPosX = 0;  // Variable to store X position (horizontal control)

int x_wall = 450;     // Centerline dividing the court into two halves (fixed for AI's side)
int w = 15;           // Width of the racket
int h = 70;           // Height of the racket
int r = 15;           // Radius of the ball
int wallHeight = 600; // Height of the window

float ballX = 150;    // Ball X position
float ballY = 300;    // Ball Y position
float ballSpeedX = 4; // Ball speed in X direction
float ballSpeedY = 2; // Ball speed in Y direction

float racketX = 75;   // Initial position of racket (left side)
float racketY = 300;  // Y position of the racket (vertical control)
float racketVelocityX = 0;  // Horizontal velocity of the racket

float aiRacketX = 825;  // AI's racket X position (right side, fixed)
float aiRacketY = 300;  // AI's racket Y position (vertical control)

boolean ballMissed = false;
boolean ballHit = false;  // Flag to check if the ball has been hit

int timePlayed = 0;  // Timer to track how long the game has been running

float damping = 0.1;  // Damping factor for racket movement (smaller values = more damping)
float maxSpeed = 6;   // Max speed for racket movement

void setup() {
  size(900, 600);
  background(0);  // Set initial background

  // Initialize serial communication
  String portName = Serial.list()[0];  // Get the first available port
  myPort = new Serial(this, "COM6", 115200);  // Set the baud rate to 115200, matching the Arduino
}

void draw() {
  drawTennisCourt();  // Draw the tennis court background
  
  // Read the incoming data from the Arduino (assuming Arduino sends updatedPosY and updatedPosX)
  if (myPort.available() > 0) {
    String inputString = myPort.readStringUntil('\n');  // Read data from the Arduino
    if (inputString != null) {
      inputString = trim(inputString);  // Remove any extra spaces or newlines
      String[] values = split(inputString, ',');  // Assuming Arduino sends comma-separated values
      if (values.length == 2) {
        updatedPosX = int(values[0]);  // Get X position (horizontal control)
        updatedPosY = int(values[1]);  // Get Y position (vertical control)
      }
    }
  }
  
  // Update player's racket position using the vertical position from Arduino (updatedPosY)
  float targetY = map(updatedPosY, 0, 130, -10, wallHeight+10);  // Map the sensor reading (0-1023) to the screen height
  racketY += (targetY - racketY) * damping;  // Smooth transition to target

  // Gradual deceleration near boundaries to prevent bounce or recoil
  if (racketY <= h / 2) {
    racketY = h / 2;  // Lock to the top boundary
  } else if (racketY >= wallHeight - h / 2) {
    racketY = wallHeight - h / 2;  // Lock to the bottom boundary
  }

  // Smooth horizontal movement using damping
  float targetX = map(updatedPosX, 0, 130, -10, x_wall+10);  // Map the X sensor reading to the left half of the screen
  racketVelocityX = (targetX - racketX) * damping;  // Apply damping to the horizontal velocity
  racketX += racketVelocityX;  // Update racket position based on velocity

  // Constrain racket movement within the workspace (left half of the court)
  racketX = constrain(racketX, w / 2, x_wall - w / 2);  // Constrain player racket to left half

  // Check if the player is moving, and send texture feedback command if moving
  if (abs(updatedPosX - 512) > 50 || abs(updatedPosY - 512) > 50) {
    myPort.write("UserMoving\n");  // Send "UserMoving" to Arduino for continuous texture feedback
  } else {
    myPort.write("Stop\n");  // Send "Stop" when no movement is detected
  }
  
  // Draw the ball (moving)
  fill(255, 0, 0);
  ellipse(ballX, ballY, 2 * r, 2 * r);  // Draw the smaller ball
  
  // Draw the player's racket (now vertical)
  fill(0, 255, 0);  // Green for the player's racket
  rect(racketX - w / 2, racketY - h / 2, w, h);  // Player's racket (flipped to vertical)
  
  // Draw the AI's racket (now vertical)
  fill(0, 0, 255);  // Blue color for the AI racket
  rect(aiRacketX - w / 2, aiRacketY - h / 2, w, h);  // AI's racket (flipped to vertical)
  
  // Update the ball position
  ballX += ballSpeedX;
  ballY += ballSpeedY;

  // Constrain ball movement within the workspace (screen bounds)
  ballX = constrain(ballX, r, width - r);  // Ball shouldn't go past the left or right edges
  ballY = constrain(ballY, r, wallHeight - r);  // Ball shouldn't go past the top or bottom

  // Check for collision with the top and bottom of the window
  if (ballY - r <= 0 || ballY + r >= wallHeight) {
    ballSpeedY *= -1; // Reverse vertical direction
  }
 // Check for collision OF RACKET with the top of the window
  if (racketY+20 + h / 2 >= wallHeight) {
    myPort.write("WallHitTop\n"); // Reverse vertical direction
  }
   // Check for collision OF RACKET with the bottom of the window
  if (racketY-10 - h / 2 <= 0 ) {
    myPort.write("WallHitBottom\n"); // Reverse vertical direction
  }
  
  // Check for collision OF RACKET with the bACK of the window
  if (racketX -10 - w <= 0 ) {
    myPort.write("WallHitBack\n"); // Reverse vertical direction
  }
   
  // Check for collision OF RACKET with the Net of the window
  if ( racketX +10 + w >= 450) {
    myPort.write("WallHitNet\n"); // Reverse vertical direction
  }
  
  
  // Check for ball collision with the player's racket
  if (ballX - r <= racketX + w / 2 && ballX + r >= racketX - w / 2 && ballY + r >= racketY - h / 2 && ballY - r <= racketY + h / 2) {
    ballSpeedX = -1*ballSpeedX; // Ball bounces back
    ballSpeedY = random(-2, 2); // Slightly randomize vertical speed after bounce
    ballHit = true; // Mark that the ball has been hit
    
    // Send motor feedback to Arduino when the ball is hit
    myPort.write("BallHit\n");  // Send "BallHit" command to Arduino for high tension feedback
  }

  // Check for ball collision with the AI's racket
  if (ballX + r >= aiRacketX - w / 2 && ballX - r <= aiRacketX + w / 2 && ballY + r >= aiRacketY - h / 2 && ballY - r <= aiRacketY + h / 2) {
    ballSpeedX *= -1; // Ball bounces back
    ballSpeedY = random(-2, 2); // Slightly randomize vertical speed after bounce
    ballHit = true; // Mark that the ball has been hit
  }

  // If the player misses the ball (ball goes past racket)
  if (ballX - r <= 0) {
    if (!ballHit) {
      ballMissed = true;
      resetGame();  // Reset both the ball and racket
    }
    ballHit = false;  // Reset hit flag after ball passes racket
    
    // Send motor feedback to stop the motors when the ball is in motion
    myPort.write("BallMoving\n");  // Send "BallMoving" command to Arduino to stop feedback
  }

  // Increase difficulty over time (gradual increase in speed)
  timePlayed++;
  if (timePlayed % 500 == 0) {  // Increase speed every 500 frames
    ballSpeedX *= 1.05;  // Gradually increase X speed by 5%
    ballSpeedY *= 1.05;  // Gradually increase Y speed by 5%
    ballSpeedX = constrain(ballSpeedX, 4, 10);  // Limit maximum speed in X direction
    ballSpeedY = constrain(ballSpeedY, 2, 10);  // Limit maximum speed in Y direction
  }

  // AI controlled racket (moves vertically to follow the ball)
  if (ballY < aiRacketY + h / 2) {
    aiRacketY -= 2;  // AI moves up
  } else if (ballY > aiRacketY - h / 2) {
    aiRacketY += 2;  // AI moves down
  }
  aiRacketY = constrain(aiRacketY, h / 2, wallHeight - h / 2);  // Constrain AI's racket within the boundaries
}

void drawTennisCourt() {
  background(0);  // Set background to black
  stroke(255);  // Set stroke to white for lines
  line(x_wall, 0, x_wall, wallHeight);  // Draw the dividing line
  
  // Draw dashed line
  float lineSpacing = 20;
  for (float i = 0; i < wallHeight; i += lineSpacing * 2) {
    line(x_wall - 2, i, x_wall - 2, i + lineSpacing);  // Draw dashed line
  }
  
  noFill();
  rect(0, 0, width, height);  // Draw the court boundary
}

// Function to reset the game when the player misses the ball
void resetGame() {
  ballX = 150;
  ballY = 300;
  ballSpeedX = 4;
  ballSpeedY = 2;
  racketY = 300;  // Reset racket position
  aiRacketY = 300;  // Reset AI racket position
  timePlayed = 0;  // Reset game timer
  ballMissed = false;
  ballHit = false;
}
