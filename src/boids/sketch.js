/* Boids, by Craig Reynolds - https://www.red3d.com/cwr/boids/ */

let boids = [];
let count = 100;

function setup() {
  createCanvas(windowWidth, windowHeight);
  frameRate(60);

  for (i = 0; i < count; i++){
    r_x = random(width)
    r_y = random(height)
    boids[i] = new Boid(r_x, r_y);
  }
  boids[0].color = 'rgb(0, 255, 0)';
}

function draw() {
  background(0);
  let p_boids = boids;
  for (i = 0; i < boids.length; i++){
    boids[i].steer(p_boids);
    boids[i].update();
    boids[i].show();
  }
}
