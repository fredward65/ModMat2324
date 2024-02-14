/* Boids, by Craig Reynolds - https://www.red3d.com/cwr/boids/ */

let boids = [];
let count = 10;

let t = 0;

function setup() {
  createCanvas(windowWidth, windowHeight);
  frameRate(60);

  for (i = 0; i < count; i++){
    boids[i] = new Boid(width/2, height/2);
  }
  boids[0].color = 'rgb(0, 255, 0)';
}

function draw() {
  background(0);
  for (i = 0; i < boids.length; i++){
    boids[i].angle = PI * noise(i, t);
    boids[i].update();
    boids[i].show();
  }
  t += 0.01;
}
