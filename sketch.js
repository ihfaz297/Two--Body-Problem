let m1Slider, m2Slider, distSlider, gSlider, timeSlider, corSlider, resetButton;
let bodies = [];
let trailLayer;
let G = 1;
let timeScale = 1;
let cor = 1; // Coefficient of restitution
let softening = 10; // Base softening parameter
const BASE_DT = 0.02; // Smaller base time step
let comX, comY, totalMomentum, prevComX, prevComY, comDrift;
let lastCollisionFrame = -100; // For collision debugging

function setup() {
  createCanvas(windowWidth, windowHeight);
  trailLayer = createGraphics(width, height);
  trailLayer.background(0);

  // Sliders & Controls
  m1Slider = createSlider(10, 100, 30);
  m1Slider.position(20, 20);
  m2Slider = createSlider(10, 100, 30);
  m2Slider.position(20, 50);
  distSlider = createSlider(50, 400, 200);
  distSlider.position(20, 80);
  gSlider = createSlider(0.1, 5, 1, 0.1);
  gSlider.position(20, 110);
  timeSlider = createSlider(0.1, 5, 1, 0.1);
  timeSlider.position(20, 140);
  corSlider = createSlider(0, 1, 1, 0.1);
  corSlider.position(20, 170);

  resetButton = createButton('Reset');
  resetButton.position(20, 200);
  resetButton.mousePressed(resetSimulation);

  resetSimulation();
}

function resetSimulation() {
  bodies = [];
  G = gSlider.value();

  let dist = distSlider.value();
  let m1 = m1Slider.value();
  let m2 = m2Slider.value();

  let totalMass = m1 + m2;
  let omega = sqrt(G * totalMass / pow(dist, 3));
  
  bodies.push(new Body(-dist/2, 0, 0, omega * dist/2, m1, [0, 150, 255]));
  bodies.push(new Body(dist/2, 0, 0, -omega * dist/2, m2, [255, 100, 0]));

  trailLayer.clear();
  trailLayer.background(0);
  prevComX = null;
}

function draw() {
  G = gSlider.value();
  timeScale = timeSlider.value();
  cor = corSlider.value();

  updateMasses();
  softening = 10 * sqrt((bodies[0].mass + bodies[1].mass) / 20);

  let minDist = p5.Vector.dist(bodies[0].pos, bodies[1].pos);
  let dt = BASE_DT * constrain(minDist / 100, 0.1, 1);
  let numSubSteps = max(1, floor(timeScale * 50 * (BASE_DT / dt))); // Adjusted for smaller BASE_DT

  for (let i = 0; i < numSubSteps; i++) {
    for (let body of bodies) {
      body.computeForce(bodies);
    }
    for (let body of bodies) {
      body.verletStep(dt);
    }
    handleCollisions(dt);
  }

  if (frameCount % 2 === 0) {
    updateTrails();
  }

  background(0);
  image(trailLayer, 0, 0);

  if (frameCount - lastCollisionFrame < 10) {
    fill(255, 255, 255, 150);
    noStroke();
    let midPoint = p5.Vector.lerp(bodies[0].pos, bodies[1].pos, 0.5);
    let s = screenPos(midPoint.x, midPoint.y);
    ellipse(s.x, s.y, 80); // Brighter, larger flash
  }

  for (let body of bodies) {
    body.display();
  }

  computeCOMandMomentum();
  drawCenterOfMass();
  drawUI();
}

function updateMasses() {
  let m1 = m1Slider.value();
  let m2 = m2Slider.value();

  if (abs(bodies[0].mass - m1) > 0.1 || abs(bodies[1].mass - m2) > 0.1) {
    let v1 = p5.Vector.sub(bodies[0].pos, bodies[0].prevPos).div(BASE_DT);
    let v2 = p5.Vector.sub(bodies[1].pos, bodies[1].prevPos).div(BASE_DT);
    let p1 = p5.Vector.mult(v1, bodies[0].mass);
    let p2 = p5.Vector.mult(v2, bodies[1].mass);
    let totalP = p5.Vector.add(p1, p2);

    bodies[0].mass = m1;
    bodies[1].mass = m2;
    bodies[0].radius = sqrt(m1) * 0.8;
    bodies[1].radius = sqrt(m2) * 0.8;

    let totalMass = m1 + m2;
    if (totalMass > 0) {
      let v1New = p5.Vector.div(totalP, totalMass);
      let v2New = v1New;
      bodies[0].prevPos = p5.Vector.sub(bodies[0].pos, p5.Vector.mult(v1New, BASE_DT));
      bodies[1].prevPos = p5.Vector.sub(bodies[1].pos, p5.Vector.mult(v2New, BASE_DT));
    }
  }
}

function computeCOMandMomentum() {
  let totalMass = 0, xSum = 0, ySum = 0;
  let momentumX = 0, momentumY = 0;
  for (let body of bodies) {
    totalMass += body.mass;
    xSum += body.pos.x * body.mass;
    ySum += body.pos.y * body.mass;
    let vel = p5.Vector.sub(body.pos, body.prevPos).div(BASE_DT);
    momentumX += body.mass * vel.x;
    momentumY += body.mass * vel.y;
  }
  comX = totalMass > 0 ? xSum / totalMass : 0;
  comY = totalMass > 0 ? ySum / totalMass : 0;
  totalMomentum = createVector(momentumX, momentumY).mag();

  if (prevComX !== null) {
    comDrift = createVector(comX - prevComX, comY - prevComY).mag();
  } else {
    comDrift = 0;
  }
  prevComX = comX;
  prevComY = comY;
}

function handleCollisions(dt) {
  let b1 = bodies[0];
  let b2 = bodies[1];
  
  // Simple discrete collision check with interpolation
  let distVec = p5.Vector.sub(b2.pos, b1.pos);
  let distance = distVec.mag();
  let minDist = (sqrt(b1.mass) + sqrt(b2.mass)) * 1.5; // Larger radius

  if (distance < minDist) {
    lastCollisionFrame = frameCount;
    console.log("Collision detected: distance=", distance, "minDist=", minDist, "cor=", cor);
    
    distVec.normalize();
    let v1 = p5.Vector.sub(b1.pos, b1.prevPos).div(dt);
    let v2 = p5.Vector.sub(b2.pos, b2.prevPos).div(dt);
    let relVel = p5.Vector.sub(v1, v2).dot(distVec);
    
    // Apply collision impulse (no relVel < 0 check)
    let impulse = (1 + cor) * relVel / (b1.mass + b2.mass);
    let impulseVec = distVec.mult(impulse);
    
    // Update velocities
    b1.prevPos = p5.Vector.sub(b1.pos, p5.Vector.sub(v1, p5.Vector.mult(impulseVec, b1.mass)).mult(dt));
    b2.prevPos = p5.Vector.sub(b2.pos, p5.Vector.add(v2, p5.Vector.mult(impulseVec, b2.mass)).mult(dt));
    
    // Separate bodies
    let correction = distVec.copy().mult((minDist - distance) * 0.5);
    b1.pos.sub(correction);
    b2.pos.add(correction);
    
    // For cor = 0, ensure bodies stick by setting equal velocities
    if (cor === 0) {
      let totalMass = b1.mass + b2.mass;
      let vAvg = p5.Vector.add(p5.Vector.mult(v1, b1.mass), p5.Vector.mult(v2, b2.mass)).div(totalMass);
      b1.prevPos = p5.Vector.sub(b1.pos, p5.Vector.mult(vAvg, dt));
      b2.prevPos = p5.Vector.sub(b2.pos, p5.Vector.mult(vAvg, dt));
    }
  }
}

function updateTrails() {
  trailLayer.fill(0, 10);
  trailLayer.noStroke();
  trailLayer.rect(0, 0, width, height);
  for (let body of bodies) {
    body.drawTrail(trailLayer);
  }
}

function drawCenterOfMass() {
  let pulse = 50 + 20 * sin(frameCount * 0.1);
  noStroke();
  fill(0, 255, 0, 100);
  let s = screenPos(comX, comY);
  ellipse(s.x, s.y, pulse);
  fill(0, 255, 0);
  ellipse(s.x, s.y, 8);
}

function drawUI() {
  fill(255);
  textSize(14);
  text('Mass 1: ' + m1Slider.value().toFixed(2), m1Slider.x * 2 + m1Slider.width, 35);
  text('Mass 2: ' + m2Slider.value().toFixed(2), m2Slider.x * 2 + m2Slider.width, 65);
  text('Distance: ' + distSlider.value(), distSlider.x * 2 + distSlider.width, 95);
  text('G: ' + G.toFixed(2), gSlider.x * 2 + gSlider.width, 125);
  text('Time Scale: ' + timeScale.toFixed(2), timeSlider.x * 2 + timeSlider.width, 155);
  text('Restitution: ' + cor.toFixed(2), corSlider.x * 2 + corSlider.width, 185);
  text('Total Momentum: ' + totalMomentum.toFixed(4), 20, 215);
  text('COM Drift: ' + comDrift.toFixed(4), 20, 245);
  if (frameCount - lastCollisionFrame < 10) {
    fill(255, 0, 0);
    text('COLLISION!', 20, 275);
  }
}

function screenPos(x, y) {
  return createVector(width / 2 + x, height / 2 + y);
}

class Body {
  constructor(x, y, vx, vy, mass, color) {
    this.pos = createVector(x, y);
    this.prevPos = createVector(x - vx * BASE_DT, y - vy * BASE_DT);
    this.mass = mass;
    this.color = color;
    this.force = createVector(0, 0);
    this.radius = sqrt(mass) * 0.8;
  }

  computeForce(others) {
    this.force.set(0, 0);
    for (let other of others) {
      if (other === this) continue;
      
      let r = p5.Vector.sub(other.pos, this.pos);
      let distSq = r.magSq() + softening;
      let f = (G * this.mass * other.mass) / (distSq * sqrt(distSq));
      
      this.force.add(p5.Vector.mult(r, f));
    }
  }

  verletStep(dt) {
    let accel = p5.Vector.div(this.force, this.mass);
    let newPos = p5.Vector.sub(
      p5.Vector.mult(this.pos, 2),
      this.prevPos
    ).add(p5.Vector.mult(accel, dt * dt));
    
    this.prevPos = this.pos.copy();
    this.pos = newPos;
  }

  drawTrail(pg) {
    pg.noStroke();
    pg.fill(this.color[0], this.color[1], this.color[2], 150);
    let s = screenPos(this.pos.x, this.pos.y);
    pg.ellipse(s.x, s.y, 3);
  }

  display() {
    push();
    let s = screenPos(this.pos.x, this.pos.y);
    translate(s.x, s.y);
    noStroke();
    
    fill(this.color[0], this.color[1], this.color[2], 50);
    ellipse(0, 0, this.radius * 3);
    fill(this.color[0], this.color[1], this.color[2], 100);
    ellipse(0, 0, this.radius * 2);
    
    fill(this.color);
    ellipse(0, 0, this.radius * 2);
    
    fill(255, 255, 255, 100);
    ellipse(-this.radius/3, -this.radius/3, this.radius/2);
    pop();
  }
}
