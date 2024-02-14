let p_0, p, dp;  // Resting position, initial position and velocity
let k, c;  // Spring and Damper constants
let dt;  // Time step
let diam; // Circle diameter
let m; // Mass

function setup() {
  createCanvas(windowWidth, windowHeight);
  p_0 = createVector(windowWidth/2, windowHeight/2);
  p = createVector(windowWidth/3, windowHeight/3);
  dp = createVector(0, 0);
  k = 1.0;
  c = 0.1;
  diam = 20;
  m = diam / 1000
  dt = 1/60;
  frameRate(60);
}

function draw() {
  background(0, 100);
  let acceleration = (x, dx) => (-k*x - c*dx) / m;
  let euler_integrate = (x, dx) => p5.Vector.add(x, p5.Vector.mult(dx, dt));
  
  let ddp = createVector(acceleration(p.x - p_0.x, dp.x), acceleration(p.y - p_0.y, dp.y));
  dp = euler_integrate(dp, ddp);
  p = euler_integrate(p, dp);
  circle(p.x, p.y, diam);
}

/* Functions */
function mousePressed() {
  p.x = mouseX;
  p.y = mouseY;
  dp.x = 0.0;
  dp.y = 0.0;
}

function mouseDragged() {
}