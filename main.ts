// Sokoban solver: per-room push-optimal A*.
// Each level is a hallway (the player's row) plus 1-4 rooms off it, each with one door.
// Boxes never leave their room, so every room is solved independently from its door
// and the room solutions are concatenated with walks along the hallway.
// A* is over pushes (cost 1 per push) with a Hungarian-assignment heuristic on
// BFS distances (admissible, so the first goal state popped is push-optimal),
// dead-square pruning (reverse pull BFS from the goals) and 2x2 freeze pruning.
// Deterministic: the search budget is a fixed expansion count, never the clock.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const LETTER = ['u', 'd', 'l', 'r'];
const MAX_EXPANSIONS_PER_ROOM = 400_000;

type Room = {
  cells: number[]; // room floor cells (door cell included)
  cellSet: Set<number>;
  door: number; // room cell adjacent to the hallway
  entry: number; // hallway cell adjacent to the door; the player starts here
  boxes: number[];
  goals: number[];
};

// BFS shortest walk on the whole board. Passable: not a wall and not a box.
function bfsWalk(from: number, to: number, wall: Uint8Array, boxes: Set<number>, w: number, h: number): string | null {
  if (from === to) return '';
  const prev = new Int32Array(w * h).fill(-2);
  const prevDir = new Int8Array(w * h).fill(-1);
  const q = [from];
  prev[from] = -1;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi]!;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d]!;
      const ny = cy + DY[d]!;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const n = ny * w + nx;
      if (wall[n] || boxes.has(n) || prev[n] !== -2) continue;
      prev[n] = c;
      prevDir[n] = d;
      if (n === to) {
        let moves = '';
        let cur = n;
        while (cur !== from) {
          moves = LETTER[prevDir[cur]!]! + moves;
          cur = prev[cur]!;
        }
        return moves;
      }
      q.push(n);
    }
  }
  return null;
}

// BFS walk confined to a set of floor cells (avoids boxes).
function bfsWalkConfined(from: number, to: number, boxes: number[], floor: Set<number>, w: number, h: number): string | null {
  if (from === to) return '';
  const boxSet = new Set(boxes);
  const prev = new Map<number, number>([[from, -1]]);
  const prevDir = new Map<number, number>();
  const q = [from];
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi]!;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d]!;
      const ny = cy + DY[d]!;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const n = ny * w + nx;
      if (!floor.has(n) || boxSet.has(n) || prev.has(n)) continue;
      prev.set(n, c);
      prevDir.set(n, d);
      if (n === to) {
        let moves = '';
        let cur = n;
        while (cur !== from) {
          moves = LETTER[prevDir.get(cur)!]! + moves;
          cur = prev.get(cur)!;
        }
        return moves;
      }
      q.push(n);
    }
  }
  return null;
}

// BFS distances from a start cell over a set of floor cells (ignoring boxes).
function bfsDistances(start: number, floor: Set<number>, w: number, h: number): Map<number, number> {
  const dist = new Map<number, number>([[start, 0]]);
  const q = [start];
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi]!;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d]!;
      const ny = cy + DY[d]!;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const n = ny * w + nx;
      if (!floor.has(n) || dist.has(n)) continue;
      dist.set(n, dist.get(c)! + 1);
      q.push(n);
    }
  }
  return dist;
}

// Reachable region for the player: floor cells reachable without moving a box.
function reachableRegion(player: number, boxes: Set<number>, floor: Set<number>, w: number, h: number): number[] {
  const seen = new Set<number>([player]);
  const q = [player];
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi]!;
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d]!;
      const ny = cy + DY[d]!;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const n = ny * w + nx;
      if (!floor.has(n) || boxes.has(n) || seen.has(n)) continue;
      seen.add(n);
      q.push(n);
    }
  }
  return [...seen].sort((a, b) => a - b);
}

// Solve one room from its entry cell. Returns the move string and the end state,
// or null if the search budget runs out.
function solveRoom(
  room: Room,
  startPlayer: number,
  startBoxes: number[],
  wall: Uint8Array,
  w: number,
  h: number,
): { moves: string; player: number; boxes: number[] } | null {
  const n = room.boxes.length;
  if (n === 0) return { moves: '', player: startPlayer, boxes: startBoxes.slice() };
  // Only this room's boxes belong in the room's search; the rest stay put.
  const roomStart = startBoxes.filter((b) => room.cellSet.has(b)).sort((a, b) => a - b);
  const goalSet = new Set(room.goals);
  // Player floor: the room's cells plus the entry cell in the hallway.
  const playerFloor = new Set<number>(room.cellSet);
  playerFloor.add(room.entry);
  // Dead squares: cells a box can never be pushed away from towards a goal.
  // Reverse pull BFS from the goals: a box at n can be pushed to a live cell c
  // when the player can stand at 2n - c; with n = c + d that cell is c + 2d.
  const live = new Set<number>(room.goals);
  {
    const q = [...room.goals];
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi]!;
      const cx = c % w;
      const cy = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = cx + DX[d]!;
        const ny = cy + DY[d]!;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        const nn = ny * w + nx; // the cell the box is pulled to
        if (!room.cellSet.has(nn) || live.has(nn)) continue;
        const bx = cx + 2 * DX[d]!;
        const by = cy + 2 * DY[d]!;
        if (bx < 0 || bx >= w || by < 0 || by >= h) continue;
        const b = by * w + bx; // where the player must stand to push nn to c
        if (!playerFloor.has(b)) continue;
        live.add(nn);
        q.push(nn);
      }
    }
  }
  // BFS distances from each goal over the room's cells (ignoring boxes).
  const distFromGoal = room.goals.map((g) => bfsDistances(g, room.cellSet, w, h));
  // Minimum assignment of boxes to goals by BFS distance (brute-force permutations).
  const perms: number[][] = [];
  {
    const idx = room.goals.map((_, i) => i);
    const gen = (a: number[], k: number): void => {
      if (k === 1) {
        perms.push(a.slice());
        return;
      }
      for (let i = 0; i < k; i++) {
        gen(a, k - 1);
        const j = k % 2 === 1 ? 0 : i;
        [a[j]!, a[k - 1]!] = [a[k - 1]!, a[j]!];
      }
    };
    gen(idx, idx.length);
  }
  const heuristic = (boxes: number[]): number => {
    let best = Infinity;
    for (const p of perms) {
      let s = 0;
      for (let i = 0; i < n; i++) {
        const d = distFromGoal[p[i]!]!.get(boxes[i]!);
        if (d === undefined) {
          s = Infinity;
          break;
        }
        s += d;
      }
      if (s < best) best = s;
    }
    return best;
  };
  const isDead = (boxes: number[]): boolean => {
    for (const b of boxes) {
      if (!goalSet.has(b) && !live.has(b)) return true;
    }
    // 2x2 freeze: a 2x2 block of walls/boxes holding a box off a goal never moves.
    const boxSet = new Set(boxes);
    const blocked = (c: number): boolean => {
      const cx = c % w;
      const cy = (c / w) | 0;
      if (cx < 0 || cx >= w || cy < 0 || cy >= h) return true;
      // The entry cell is free for the player to stand on (to push the door box).
      if (c === room.entry) return false;
      return wall[c]! === 1 || !room.cellSet.has(c) || boxSet.has(c);
    };
    for (const b of boxes) {
      if (goalSet.has(b)) continue;
      const bx = b % w;
      const by = (b / w) | 0;
      for (const [ox, oy] of [[0, 0], [-1, 0], [0, -1], [-1, -1]] as const) {
        const x = bx + ox;
        const y = by + oy;
        if (x < 0 || y < 0 || x + 1 >= w || y + 1 >= h) continue;
        const c00 = y * w + x;
        const c10 = y * w + x + 1;
        const c01 = (y + 1) * w + x;
        const c11 = (y + 1) * w + x + 1;
        if (blocked(c00) && blocked(c10) && blocked(c01) && blocked(c11)) return true;
      }
    }
    return false;
  };

  // A* over pushes. A state is the boxes plus the player's reachable region.
  const sBoxes: number[][] = [];
  const sPlayer: number[] = [];
  const sRegion: number[][] = [];
  const sParent: number[] = [];
  const sPushBox: number[] = [];
  const sPushDir: number[] = [];
  const sG: number[] = [];
  const bestG = new Map<string, number>();
  const keyOf = (boxes: number[], region: number[]): string => boxes.join(',') + '|' + region.join(',');
  const startRegion = reachableRegion(startPlayer, new Set(roomStart), playerFloor, w, h);
  const startSorted = roomStart;
  sBoxes.push(startSorted);
  sPlayer.push(startPlayer);
  sRegion.push(startRegion);
  sParent.push(-1);
  sPushBox.push(-1);
  sPushDir.push(-1);
  sG.push(0);
  bestG.set(keyOf(startSorted, startRegion), 0);
  // Binary heap of [f, g, state], smallest f first, then largest g, then earliest state.
  const heap: [number, number, number][] = [];
  const less = (a: [number, number, number], b: [number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): [number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };
  heapPush([heuristic(startSorted), 0, 0]);
  let expanded = 0;
  let goalState = -1;
  while (heap.length > 0 && expanded < MAX_EXPANSIONS_PER_ROOM) {
    const [, g, s] = heapPop();
    const boxes = sBoxes[s]!;
    if (bestG.get(keyOf(boxes, sRegion[s]!)) !== g) continue;
    if (boxes.every((b) => goalSet.has(b))) {
      goalState = s;
      break;
    }
    expanded++;
    const region = new Set(sRegion[s]!);
    const boxSet = new Set(boxes);
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi]!;
      const bx = b % w;
      const by = (b / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = bx + DX[d]!;
        const ny = by + DY[d]!;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        const dest = ny * w + nx;
        if (!room.cellSet.has(dest) || boxSet.has(dest)) continue;
        const behind = (by - DY[d]!) * w + (bx - DX[d]!);
        if (!region.has(behind)) continue;
        const nb = boxes.slice();
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        if (isDead(nb)) continue;
        const nRegion = reachableRegion(b, new Set(nb), playerFloor, w, h);
        const k = keyOf(nb, nRegion);
        const prev = bestG.get(k);
        if (prev !== undefined && prev <= g + 1) continue;
        bestG.set(k, g + 1);
        const t = sBoxes.length;
        sBoxes.push(nb);
        sPlayer.push(b);
        sRegion.push(nRegion);
        sParent.push(s);
        sPushBox.push(b);
        sPushDir.push(d);
        sG.push(g + 1);
        heapPush([g + 1 + heuristic(nb), g + 1, t]);
      }
    }
  }
  if (goalState < 0) return null;
  // Reconstruct the pushes, then replay them with walks to build the move string.
  const pushes: { box: number; dir: number }[] = [];
  for (let t = goalState; t > 0; t = sParent[t]!) {
    pushes.push({ box: sPushBox[t]!, dir: sPushDir[t]! });
  }
  pushes.reverse();
  let moves = '';
  let boxes = startSorted.slice();
  let player = startPlayer;
  for (const { box, dir } of pushes) {
    const behind = box - (DY[dir]! * w + DX[dir]!);
    const walk = bfsWalkConfined(player, behind, boxes, playerFloor, w, h);
    if (walk === null) return null;
    moves += walk;
    moves += LETTER[dir]!;
    const idx = boxes.indexOf(box);
    boxes[idx] = box + (DY[dir]! * w + DX[dir]!);
    boxes.sort((a, b) => a - b);
    player = box;
  }
  // Merge the room's solved boxes back into the level's full box list.
  const merged = startBoxes.filter((b) => !room.cellSet.has(b)).concat(boxes).sort((a, b) => a - b);
  return { moves, player, boxes: merged };
}

function solve(text: string): string {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h);
  const goals = new Set<number>();
  let boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      wall[i] = ch === '#' ? 1 : 0;
      if ('.*+'.includes(ch)) goals.add(i);
      if ('$*'.includes(ch)) boxes.push(i);
      if ('@+'.includes(ch)) player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  const hy = (player / w) | 0;
  // Rooms: connected components of non-wall cells off the hallway row.
  const seen = new Uint8Array(w * h);
  const rooms: Room[] = [];
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const i = y * w + x;
      if (y === hy || wall[i] || seen[i]) continue;
      const comp: number[] = [];
      const q = [i];
      seen[i] = 1;
      while (q.length) {
        const c = q.pop()!;
        comp.push(c);
        const cx = c % w;
        const cy = (c / w) | 0;
        for (let d = 0; d < 4; d++) {
          const nx = cx + DX[d]!;
          const ny = cy + DY[d]!;
          if (ny === hy || nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
          const ni = ny * w + nx;
          if (wall[ni] || seen[ni]) continue;
          seen[ni] = 1;
          q.push(ni);
        }
      }
      const cellSet = new Set(comp);
      const doors = comp.filter((c) => {
        const cy = (c / w) | 0;
        return cy + 1 === hy || cy - 1 === hy;
      });
      const door = doors[0]!;
      const dcy = (door / w) | 0;
      const entry = dcy < hy ? door + w : door - w;
      rooms.push({
        cells: comp,
        cellSet,
        door,
        entry,
        boxes: boxes.filter((b) => cellSet.has(b)),
        goals: [...goals].filter((g) => cellSet.has(g)),
      });
    }
  }
  // Rooms whose door cell is a goal go last: a box ending on the door would
  // block the player's exit from that room.
  rooms.sort((a, b) => {
    const ag = goals.has(a.door) ? 1 : 0;
    const bg = goals.has(b.door) ? 1 : 0;
    if (ag !== bg) return ag - bg;
    return (a.door % w) - (b.door % w);
  });
  let moves = '';
  let curBoxes = boxes.slice();
  let curPlayer = player;
  for (const room of rooms) {
    const walk = bfsWalk(curPlayer, room.entry, wall, new Set(curBoxes), w, h);
    if (walk === null) return '';
    moves += walk;
    curPlayer = room.entry;
    const res = solveRoom(room, curPlayer, curBoxes, wall, w, h);
    if (res === null) return '';
    moves += res.moves;
    curPlayer = res.player;
    curBoxes = res.boxes;
  }
  return moves;
}

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
