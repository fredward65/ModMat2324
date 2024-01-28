let t = 0;

function setup() {
  // put setup code here
  createCanvas(windowWidth, windowHeight);
  frameRate(60);
}

function draw() {
  // put drawing code here
  background(0);
  circle(width/2, height/2, 250 * (1 + sin(t)));
  t += 0.05;
}
