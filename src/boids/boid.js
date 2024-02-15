class Boid {
  constructor(x, y, angle=0, color = 255){
    this.pos = createVector(x, y);
    this.dir = createVector(0, 1);
    this.vel = 5;
    this.size = 50;
    this.color = color;
    this.angle = angle;
  }

  update(){
    let dir = p5.Vector.rotate(this.dir, this.angle - 0.5*PI)
    dir = p5.Vector.mult(dir, this.vel)
    this.pos.add(dir);
    if(this.pos.x < 0) this.pos.x = width;
    if(this.pos.y < 0) this.pos.y = height;
    if(this.pos.x > width) this.pos.x = 0;
    if(this.pos.y > height) this.pos.y = 0;
  }

  show(){
    let a = floor(this.size / 2);
    let b = floor(a / 2);
    noStroke();
    fill(this.color);
    push();
    translate(this.pos.x, this.pos.y);
    rotate(this.angle - 0.5*PI)
    triangle(-b, -a, 0, a, b, -a);
    pop();
  }
}