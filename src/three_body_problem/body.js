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

  }

  applyForce(F){
    let acc = p5.Vector.div(F, this.mass);
    this.vel.add(p5.Vector.mult(acc, dt));
    this.pos.add(p5.Vector.mult(this.vel, dt));
  }

  show(){
    noStroke();
    circle(this.pos.x, this.pos.y, this.radius);
  }
}