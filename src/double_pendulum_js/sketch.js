let dt;  // Time step

function setup() {
  createCanvas(windowWidth, windowHeight);
  dt = 1/60;
  frameRate(60);
}

function draw() {
  background(0, 100);
  translate(width/2, height/2);
  stroke(255);
  line(0, 0, 0, 100)
  line(0, 100, 100, 200)
}
