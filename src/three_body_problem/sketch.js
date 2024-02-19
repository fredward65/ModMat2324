/* Three Body Problem - http://www.scholarpedia.org/article/Three_body_problem */

let dt;
let grid;
let bodies = [];

function setup() {
  createCanvas(windowWidth, windowHeight);
  frameRate(60);
  dt = 1/60;

  for (let i = 0; i < 5; i++){
    bodies[i] = new Body(random(100, 200),
                         createVector(random(width), random(height)),
                         p5.Vector.random2D().setMag(random(-100, 100)));
  }
  background(0);
}

function draw() {
  background(0, 10);
  for (let i = 0; i < bodies.length; i++){
    for (let j = 0; j < bodies.length; j++){
      if (i != j) bodies[i].attract(bodies[j]);
    }
    bodies[i].show();
  }
}