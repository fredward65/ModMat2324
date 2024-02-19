/* The Game of Life, by John Conway - https://web.stanford.edu/class/sts145/Library/life.pdf */

let res = 50;
let grid;
let start = false;

function setup() {
  createCanvas(800, 600);
  frameRate(10);
  cols = floor(width  / res);
  rows = floor(height / res);
  grid = new Array(cols).fill(0).map(() => new Array(rows).fill(0));
}

function draw() {
  background(0);
  stroke(128 * (1 - start), 128 * start, 128 * start);

  if (start){
    /* Game rules here */
    
  }

  for (let i = 0; i < grid.length; i++){
    for (let j = 0; j < grid[j].length; j++){
      let state = 255 * grid[i][j];
      fill(state);
      square(i * res, j * res, res);
    }
  }
}

function mouseClicked() {
  let i = floor(mouseX / res);
  let j = floor(mouseY / res);
  grid[i][j] = 1 - grid[i][j];
}

function keyPressed() {
  if (keyCode === ENTER){
    start = !start;
  }
  if (keyCode === BACKSPACE){
    grid = new Array(cols).fill(0).map(() => new Array(rows).fill(0));
    start = false;
  }
}