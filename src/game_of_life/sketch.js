/* The Game of Life, by John Conway - https://web.stanford.edu/class/sts145/Library/life.pdf */

let res = 10;
let cols, rows;
let grid;
let start = false;

function setup() {
  createCanvas(800, 600);
  frameRate(20);
  cols = floor(width  / res);
  rows = floor(height / res);
  grid = new Array(cols).fill(0).map(() => new Array(rows).fill(0));
  for (let i = 0; i < grid.length; i++){
    for (let j = 0; j < grid[j].length; j++){
      grid[i][j] = noise(i, j) > 0.5;
    }
  }
}

function draw() {
  background(0);
  stroke(128 * (1 - start), 128 * start, 128 * start);

  if (start){
    /* Game rules here */
    let n_grid = new Array(cols).fill(0).map(() => new Array(rows).fill(0));

    for (let i = 0; i < grid.length; i++){
      for (let j = 0; j < grid[j].length; j++){
        n_grid[i][j] = grid[i][j];

        let counter = 0;
        for (let i_ = -1; i_ <= 1; i_++){
          for (let j_ = -1; j_ <= 1; j_++){
              let i_2 = i + i_;
              let j_2 = j + j_;
              i_2 = (i_2 + cols) % cols;
              j_2 = (j_2 + rows) % rows;
              counter += grid[i_2][j_2];
          }
        }
        counter += -grid[i][j];
        
        if ((counter < 2 || counter > 3) && grid[i][j] == 1){
          n_grid[i][j] = 0;
        }

        if (counter == 3 && grid[i][j] == 0){
          n_grid[i][j] = 1;
        }
      }
    }

    grid = n_grid;
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