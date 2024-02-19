/* Three Body Problem - http://www.scholarpedia.org/article/Three_body_problem */

let dt;
let grid;
let bodies = [];

function setup() {
  createCanvas(windowWidth, windowHeight);
  frameRate(60);
  dt = 1/60;

  bodies[0] = new Body(200, createVector(width/2, height/2), createVector(0, 0));
  
  background(0);
}

function draw() {
  background(0, 10);
  bodies[0].show();
}