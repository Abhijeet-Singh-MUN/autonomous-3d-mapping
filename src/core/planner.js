const PATH_NEIGHBORS_3D = [];

for (let dz = -1; dz <= 1; dz += 1) {
  for (let dy = -1; dy <= 1; dy += 1) {
    for (let dx = -1; dx <= 1; dx += 1) {
      if (dx === 0 && dy === 0 && dz === 0) {
        continue;
      }
      PATH_NEIGHBORS_3D.push({
        dx,
        dy,
        dz,
        cost: Math.hypot(dx, dy, dz)
      });
    }
  }
}

export function searchPath3D(grid, start, goal, mode) {
  if (!grid || !start || !goal) {
    return { path: [], cost: Infinity };
  }

  const startKey = grid.packIndex(start.col, start.row, start.layer);
  const goalKey = grid.packIndex(goal.col, goal.row, goal.layer);
  const open = new Map([[startKey, start]]);
  const cameFrom = new Map();
  const gScore = new Map([[startKey, 0]]);
  const config = plannerConfigFor(mode);
  const fScore = new Map([[startKey, heuristic3D(start, goal) * config.heuristicWeight]]);

  while (open.size) {
    let currentKey = null;
    let current = null;
    let bestScore = Infinity;

    open.forEach((cell, key) => {
      const score = fScore.get(key) ?? Infinity;
      if (score < bestScore) {
        bestScore = score;
        currentKey = key;
        current = cell;
      }
    });

    if (!current || currentKey === null) {
      break;
    }

    if (currentKey === goalKey) {
      return reconstructPath3D(grid, cameFrom, currentKey);
    }

    open.delete(currentKey);

    PATH_NEIGHBORS_3D.forEach(({ dx, dy, dz, cost }) => {
      const next = {
        col: current.col + dx,
        row: current.row + dy,
        layer: current.layer + dz
      };
      if (!grid.inBounds(next.col, next.row, next.layer)) {
        return;
      }
      const nextKey = grid.packIndex(next.col, next.row, next.layer);
      if (!grid.traversable[nextKey] && nextKey !== goalKey) {
        return;
      }

      const tentative = (gScore.get(currentKey) ?? Infinity) + cost;
      if (tentative < (gScore.get(nextKey) ?? Infinity)) {
        cameFrom.set(nextKey, currentKey);
        gScore.set(nextKey, tentative);
        const h = heuristic3D(next, goal);
        const priority = tentative * config.gFactor + h * config.heuristicWeight;
        fScore.set(nextKey, priority);
        if (!open.has(nextKey)) {
          open.set(nextKey, next);
        }
      }
    });
  }

  return { path: [], cost: Infinity };
}

function reconstructPath3D(grid, cameFrom, currentKey) {
  const path = [];
  let cursor = currentKey;
  while (cursor !== undefined) {
    path.unshift(grid.unpackIndex(cursor));
    cursor = cameFrom.get(cursor);
  }
  let cost = 0;
  for (let index = 1; index < path.length; index += 1) {
    const a = path[index - 1];
    const b = path[index];
    cost += heuristic3D(a, b);
  }
  return { path, cost };
}

function plannerConfigFor(mode) {
  switch (mode) {
    case 'weighted':
      return { heuristicWeight: 1.8, gFactor: 1 };
    case 'greedy':
      return { heuristicWeight: 2.6, gFactor: 0.2 };
    default:
      return { heuristicWeight: 1, gFactor: 1 };
  }
}

function heuristic3D(a, b) {
  return Math.hypot(a.col - b.col, a.row - b.row, a.layer - b.layer);
}
