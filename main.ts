// Sokoban solver for sokoban-solver-01-run-20261121 (make.fun).
//
// Approach: the board's live squares (squares from which a box can still
// reach a goal) split into independent rooms, so each room is solved on its
// own and the answers are concatenated; walking between rooms is free, so
// room-optimal pushes add up to the level's optimum. Each room is solved by
// A* over pushes (not steps): a state is the sorted box squares plus the
// representative (lowest square) of the player's reachable region, so
// walking between pushes collapses into macro moves. The heuristic is the
// minimum-cost bipartite matching of boxes to goals over true single-box
// push distances (reverse pull BFS from every goal), admissible for the
// push count. States are pruned by dead squares, frozen boxes and solid
// 2x2 blocks. Searches are budgeted by expansion counts, never by the
// clock, so two runs write byte-identical answers. If a room resists the
// optimal A*, weighted A* fallbacks still try to answer it, and if a room
// resists everything the whole level is searched as one problem.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const DIRS = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];
const INF = 1e9;

// ---------- level parsing (mirrors the checker's XSB reading) ----------
function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals = [];
  const boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// ---------- static analysis ----------
// distGoal[g][cell] = fewest pushes to bring one box from cell to goal g,
// ignoring the other boxes; INF when impossible (a dead square for g).
// Computed for one problem's goal list on the plain wall grid.
function computeDistances(w, wall, goals) {
  const n = wall.length;
  const distGoal = [];
  for (const g of goals) {
    const dist = new Int32Array(n).fill(INF);
    dist[g] = 0;
    const queue = [g];
    let head = 0;
    while (head < queue.length) {
      const c = queue[head++];
      for (const [dx, dy] of DIRS) {
        const d = dy * w + dx;
        const from = c - d; // box stood at `from`, was pushed into c
        const stand = c - 2 * d; // player stood behind it
        if (from < 0 || from >= n || stand < 0 || stand >= n) continue;
        if (wall[from] || wall[stand]) continue;
        if (dist[from] === INF) {
          dist[from] = dist[c] + 1;
          queue.push(from);
        }
      }
    }
    distGoal.push(dist);
  }
  // dead[cell]: no goal is reachable from cell at all.
  const dead = new Uint8Array(n).fill(1);
  for (let c = 0; c < n; c++) {
    if (wall[c]) continue;
    for (let gi = 0; gi < goals.length; gi++) {
      if (distGoal[gi][c] < INF) { dead[c] = 0; break; }
    }
  }
  return { distGoal, dead };
}

// Minimum-cost matching of boxes to distinct goals (Hungarian, O(n^2 m)).
// cost[b][g] is the push distance; returns INF when no finite matching.
function matchingCost(cost, nb, ng) {
  if (nb === 0) return 0;
  // Potentials formulation for rectangular assignment (n <= m required).
  if (nb > ng) return INF;
  const u = new Float64Array(nb + 1);
  const v = new Float64Array(ng + 1);
  const p = new Int32Array(ng + 1).fill(-1); // goal -> box matched
  const way = new Int32Array(ng + 1).fill(-1);
  for (let i = 0; i < nb; i++) {
    p[ng] = i;
    let j0 = ng;
    const minv = new Float64Array(ng + 1).fill(Infinity);
    const used = new Uint8Array(ng + 1);
    do {
      used[j0] = 1;
      const i0 = p[j0];
      let delta = Infinity;
      let j1 = -1;
      for (let j = 0; j < ng; j++) {
        if (used[j]) continue;
        const cur = cost[i0 * ng + j] - u[i0] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
      }
      for (let j = 0; j <= ng; j++) {
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
        else minv[j] -= delta;
      }
      j0 = j1;
    } while (p[j0] !== -1);
    do {
      const j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 !== ng);
  }
  let total = 0;
  for (let j = 0; j < ng; j++) {
    if (p[j] >= 0) {
      const c = cost[p[j] * ng + j];
      if (c >= INF) return INF;
      total += c;
    }
  }
  return total;
}

// ---------- deadlock checks on a box set ----------
// boxSet: Set of box squares; goalSet: Set of goal squares.
function isDeadlocked(P, statics, boxSet) {
  const { w, wall } = P;
  const { dead } = statics;
  // Cells a box can never pass through: walls plus other rooms' boxes.
  const solidCell = (c) => wall[c] || P.blocked[c];
  // Dead square.
  for (const b of boxSet) {
    if (dead[b] && !P.goalSet.has(b)) return true;
  }
  // Solid 2x2 of walls/boxes with a box off its goal never moves again.
  for (const b of boxSet) {
    if (P.goalSet.has(b)) continue;
    const bx = b % w;
    const by = (b / w) | 0;
    for (const [ox, oy] of [[0, 0], [-1, 0], [0, -1], [-1, -1]]) {
      const x0 = bx + ox;
      const y0 = by + oy;
      if (x0 < 0 || y0 < 0 || x0 + 1 >= w) continue;
      const cells = [y0 * w + x0, y0 * w + x0 + 1, (y0 + 1) * w + x0, (y0 + 1) * w + x0 + 1];
      let solid = true;
      for (const c of cells) {
        if (!solidCell(c) && !boxSet.has(c)) { solid = false; break; }
      }
      if (solid) return true;
    }
  }
  // Freeze: a box boxed in on one horizontal and one vertical side by walls
  // or already-frozen boxes can never move; iterate to a fixpoint.
  const frozen = new Set();
  let changed = true;
  while (changed) {
    changed = false;
    for (const b of boxSet) {
      if (frozen.has(b)) continue;
      const blocked = (c) => solidCell(c) || frozen.has(c);
      const horiz = blocked(b - 1) || blocked(b + 1);
      const vert = blocked(b - w) || blocked(b + w);
      if (horiz && vert) { frozen.add(b); changed = true; }
    }
  }
  for (const b of frozen) {
    if (!P.goalSet.has(b)) return true;
  }
  return false;
}

// ---------- the push search ----------
// weight = 1 is plain A* (push-optimal); weight > 1 trades optimality for
// speed. Returns the full move string, or null when the budget runs out.
function search(P, statics, budget, weight) {
  const { w, wall, goals } = P;
  const { distGoal } = statics;
  const nb = P.boxes.length;
  const ng = goals.length;
  const costArr = new Float64Array(nb * ng);
  const heuristic = (boxes) => {
    for (let b = 0; b < nb; b++) {
      for (let g = 0; g < ng; g++) costArr[b * ng + g] = distGoal[g][boxes[b]];
    }
    return matchingCost(costArr, nb, ng);
  };

  // Player reachability with parent tracking, reused scratch buffers.
  const n = wall.length;
  const seen = new Int32Array(n);
  const prev = new Int32Array(n);
  const prevDir = new Int32Array(n);
  let stamp = 0;
  // Flood from `start` avoiding walls and boxes; returns the region's
  // representative (its lowest square) and fills prev/prevDir for paths.
  function flood(start, boxSet) {
    stamp++;
    const queue = [start];
    seen[start] = stamp;
    prev[start] = -1;
    let rep = start;
    let head = 0;
    while (head < queue.length) {
      const c = queue[head++];
      if (c < rep) rep = c;
      for (let di = 0; di < 4; di++) {
        const [dx, dy] = DIRS[di];
        const nc = c + dy * w + dx;
        if (nc < 0 || nc >= n || seen[nc] === stamp) continue;
        if (wall[nc] || P.blocked[nc] || boxSet.has(nc)) continue;
        seen[nc] = stamp;
        prev[nc] = c;
        prevDir[nc] = di;
        queue.push(nc);
      }
    }
    return rep;
  }
  function walkPath(to) {
    // Reconstruct the walk from the flood source to `to` (reverse letters).
    let s = '';
    let c = to;
    while (prev[c] !== -1) {
      s = DIRS[prevDir[c]][2] + s;
      c = prev[c];
    }
    return s;
  }
  // A second, path-free flood on its own buffers: computing a child's
  // region representative must not clobber the parent's flood above.
  const seen2 = new Int32Array(n);
  let stamp2 = 0;
  function floodRep(start, boxSet) {
    stamp2++;
    const queue = [start];
    seen2[start] = stamp2;
    let rep = start;
    let head = 0;
    while (head < queue.length) {
      const c = queue[head++];
      if (c < rep) rep = c;
      for (let di = 0; di < 4; di++) {
        const [dx, dy] = DIRS[di];
        const nc = c + dy * w + dx;
        if (nc < 0 || nc >= n || seen2[nc] === stamp2) continue;
        if (wall[nc] || P.blocked[nc] || boxSet.has(nc)) continue;
        seen2[nc] = stamp2;
        queue.push(nc);
      }
    }
    return rep;
  }

  const startBoxes = P.boxes.slice();
  const startSet = new Set(startBoxes);
  if (isDeadlocked(P, statics, startSet)) return null;
  const h0 = heuristic(startBoxes);
  if (h0 >= INF) return null;

  // Node store.
  const nBoxes = [startBoxes];
  const nPlayer = [P.player]; // actual player square (a box's old square)
  const nG = [0];
  const nParent = [-1];
  const nMove = ['']; // walk + push letters that created this node
  const best = new Map();
  const keyOf = (rep, boxes) => String.fromCharCode(rep, ...boxes);
  const startRep = flood(P.player, startSet);
  best.set(keyOf(startRep, startBoxes), 0);

  // Binary heap of node indices ordered by f, then deeper g, then id.
  const heap = [0];
  const fOf = [h0 * weight];
  const less = (a, b) => {
    if (fOf[a] !== fOf[b]) return fOf[a] < fOf[b];
    if (nG[a] !== nG[b]) return nG[a] > nG[b];
    return a < b;
  };
  function heapPush(id) {
    heap.push(id);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const t = heap[i]; heap[i] = heap[p]; heap[p] = t;
      i = p;
    }
  }
  function heapPop() {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < heap.length && less(heap[l], heap[m])) m = l;
        if (r < heap.length && less(heap[r], heap[m])) m = r;
        if (m === i) break;
        const t = heap[i]; heap[i] = heap[m]; heap[m] = t;
        i = m;
      }
    }
    return top;
  }

  let expansions = 0;
  while (heap.length > 0) {
    const id = heapPop();
    const boxes = nBoxes[id];
    const g = nG[id];
    // Solved when every box sits on a goal.
    let solved = true;
    for (const b of boxes) if (!P.goalSet.has(b)) { solved = false; break; }
    if (solved) {
      const parts = [];
      let cur = id;
      while (cur !== -1) { parts.push(nMove[cur]); cur = nParent[cur]; }
      return parts.reverse().join('');
    }
    if (expansions >= budget) return null;
    expansions++;
    const boxSet = new Set(boxes);
    flood(nPlayer[id], boxSet);
    const reachStamp = stamp;
    for (let bi = 0; bi < nb; bi++) {
      const b = boxes[bi];
      for (let di = 0; di < 4; di++) {
        const [dx, dy, letter] = DIRS[di];
        const d = dy * w + dx;
        const dest = b + d;
        const stand = b - d;
        if (dest < 0 || dest >= n || stand < 0 || stand >= n) continue;
        if (wall[dest] || P.blocked[dest] || boxSet.has(dest)) continue;
        if (seen[stand] !== reachStamp) continue;
        if (statics.dead[dest] && !P.goalSet.has(dest)) continue;
        const childBoxes = boxes.slice();
        childBoxes[bi] = dest;
        childBoxes.sort((x, y) => x - y);
        const childSet = new Set(childBoxes);
        if (isDeadlocked(P, statics, childSet)) continue;
        const h = heuristic(childBoxes);
        if (h >= INF) continue;
        // The walk must be read from this flood, before the child's flood
        // overwrites the path tree; the player ends on the box's old square.
        const walk = walkPath(stand);
        const childRep = floodRep(b, childSet);
        const key = keyOf(childRep, childBoxes);
        const ng2 = g + 1;
        const known = best.get(key);
        if (known !== undefined && known <= ng2) continue;
        best.set(key, ng2);
        const nid = nBoxes.length;
        nBoxes.push(childBoxes);
        nPlayer.push(b);
        nG.push(ng2);
        nParent.push(id);
        nMove.push(walk + letter);
        fOf[nid] = ng2 + weight * h;
        heapPush(nid);
      }
    }
  }
  return null;
}

// Solve one problem (a whole level or one component of it): optimal A*
// first, then progressively greedier fallbacks so a hard problem still
// gets an answer. Budgets are expansion counts, never the clock.
function solveProblem(P) {
  const statics = computeDistances(P.w, P.wall, P.goals);
  const nb = P.boxes.length;
  const optimalBudget = nb <= 4 ? 600000 : nb <= 8 ? 900000 : 1400000;
  let ans = search(P, statics, optimalBudget, 1);
  if (ans !== null) return ans;
  ans = search(P, statics, 500000, 2);
  if (ans !== null) return ans;
  ans = search(P, statics, 500000, 5);
  return ans;
}

// Where does the player end after replaying `moves` from this state?
// Used to hand the true player square to the next component's search.
function playerAfter(L, boxes, player, moves) {
  const boxSet = new Set(boxes);
  let p = player;
  for (const ch of moves) {
    const di = 'udlr'.indexOf(ch);
    const [dx, dy] = DIRS[di];
    const d = dy * L.w + dx;
    const next = p + d;
    if (boxSet.has(next)) {
      boxSet.delete(next);
      boxSet.add(next + d);
    }
    p = next;
  }
  return p;
}

function solveLevel(text) {
  const L = parseLevel(text);
  if (L.boxes.length === 0) return '';
  const noBlock = new Uint8Array(L.wall.length);
  // Live squares (a box there can still reach some goal) split the board
  // into independent rooms: a box in a successful solution only ever stands
  // on live squares and moves between neighbours, so it never leaves its
  // live component, and boxes in different components never interact. Solve
  // each component on its own and concatenate: the push counts add, and a
  // level whose rooms are independent is optimal room by room.
  const global = computeDistances(L.w, L.wall, L.goals);
  const compOf = new Int32Array(L.wall.length).fill(-1);
  const comps = [];
  for (let c = 0; c < L.wall.length; c++) {
    if (L.wall[c] || global.dead[c] || compOf[c] !== -1) continue;
    const id = comps.length;
    const cells = [];
    const queue = [c];
    compOf[c] = id;
    while (queue.length > 0) {
      const cur = queue.pop();
      cells.push(cur);
      for (const [dx, dy] of DIRS) {
        const nc = cur + dy * L.w + dx;
        if (nc < 0 || nc >= L.wall.length) continue;
        if (L.wall[nc] || global.dead[nc] || compOf[nc] !== -1) continue;
        compOf[nc] = id;
        queue.push(nc);
      }
    }
    comps.push(cells);
  }
  const compBoxes = comps.map(() => []);
  const compGoals = comps.map(() => []);
  let separable = comps.length > 1;
  for (const b of L.boxes) {
    const id = compOf[b];
    if (id === -1) separable = false;
    else compBoxes[id].push(b);
  }
  for (const g of L.goals) {
    const id = compOf[g];
    if (id === -1) separable = false;
    else compGoals[id].push(g);
  }
  if (separable) {
    for (let k = 0; k < comps.length; k++) {
      if (compBoxes[k].length !== compGoals[k].length) { separable = false; break; }
    }
  }
  if (!separable) {
    const P = {
      w: L.w, wall: L.wall, blocked: noBlock,
      boxes: L.boxes.slice(), goals: L.goals.slice(),
      goalSet: new Set(L.goals), player: L.player,
    };
    return solveProblem(P) ?? '';
  }
  // Solve room by room, largest first; the player walks between rooms for
  // free, so the order costs no pushes.
  const order = comps.map((_, k) => k)
    .filter((k) => compBoxes[k].length > 0)
    .sort((a, b) => compBoxes[b].length - compBoxes[a].length || a - b);
  let player = L.player;
  let boxesNow = L.boxes.slice();
  let answer = '';
  for (const k of order) {
    const blocked = new Uint8Array(L.wall.length);
    for (const b of boxesNow) {
      if (compOf[b] !== k) blocked[b] = 1;
    }
    const P = {
      w: L.w, wall: L.wall, blocked,
      boxes: compBoxes[k].slice().sort((a, b) => a - b),
      goals: compGoals[k].slice().sort((a, b) => a - b),
      goalSet: new Set(compGoals[k]), player,
    };
    const part = solveProblem(P);
    if (part === null) {
      // A room resisted every budget: fall back to the whole level.
      const whole = {
        w: L.w, wall: L.wall, blocked: noBlock,
        boxes: L.boxes.slice(), goals: L.goals.slice(),
        goalSet: new Set(L.goals), player: L.player,
      };
      return solveProblem(whole) ?? '';
    }
    answer += part;
    player = playerAfter(L, boxesNow, player, part);
    // This room's boxes now rest on its goals.
    boxesNow = boxesNow.filter((b) => compOf[b] !== k).concat(compGoals[k]);
  }
  return answer;
}

// ---------- main ----------
for (const id of readdirSync(I).sort()) {
  let moves = '';
  try {
    moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
