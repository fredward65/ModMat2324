let G = 5e1;  // 6.67408e-11

class Body {
  constructor(mass, pos, vel){
    this.pos = pos;
    this.vel = vel;
    this.mass = mass * 1e3;
    this.radius = mass / 2;
  }

  attract(other){
    /* F = G * (m1 * m2) / (d**2) */
    let d = p5.Vector.dist(this.pos, other.pos);
    let dir = p5.Vector.sub(this.pos, other.pos).normalize();
    let F = G * (this.mass * other.mass) / (d * d);
    F = dir.setMag(F).limit(1e8);
    other.applyForce(F);
  }

  applyForce(F){
    let acc = p5.Vector.div(F, this.mass);
    this.vel.add(p5.Vector.mult(acc, dt));
    this.pos.add(p5.Vector.mult(this.vel, dt));
    if (this.pos.x >  width) this.pos.x = 0;
    if (this.pos.x <      0) this.pos.x = width;
    if (this.pos.y > height) this.pos.y = 0;
    if (this.pos.y <      0) this.pos.y = height;
  }

  show(){
    noStroke();
    circle(this.pos.x, this.pos.y, this.radius);
  }
}