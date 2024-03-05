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
  let x1 = 0;
  let y1 = 100;
  let x2 = 50;
  let y2 = 200;
  line(0, 0, x1, y1)
  circle(x1, y1, 20)
  line(x1, y1, x2, y2)
  circle(x2, y2, 20)
}
