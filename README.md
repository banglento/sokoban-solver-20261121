# Sokoban solver — sokoban-solver-01-run-20261121

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/banglento/sokoban-solver-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/banglento/sokoban-solver-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

Entry for make.fun's **sokoban-solver-01-run-20261121** competition,
"Build the best Sokoban solver, 21 November 2026" ($100 prize pool,
$10 to each of 1st–10th place). One program, `main.ts` (Node 22, standard
library only), answers all 5 instances of a set in a single run: it reads
`/input/<id>/level.txt` (XSB) and writes `/output/<id>/solution.txt` with
the player's moves in LURD. Scoring gives 1,000 points per instance only
for a push-optimal solution (P* pushes); one push over P* earns less than
half, so the solver is built to be push-optimal, not merely correct.

## Approach

1. **Room decomposition.** A square is *live* when a box standing there can
   still be pushed onto some goal (found by reverse pull BFS from every
   goal). In a successful solution a box only ever stands on live squares
   and moves between neighbours, so it never leaves its live connected
   component — and in these generated levels the components are exactly
   the rooms. Boxes in different components never interact and walking
   between rooms is free, so each room is solved independently and the
   move strings are concatenated; room-optimal pushes add up to the
   level's optimum.
2. **Push-level A\*.** Each room is searched over pushes, not steps: a
   state is the sorted box squares plus the representative (lowest
   square) of the player's reachable region, so the walking between two
   pushes collapses into one macro move, reconstructed by flood fill.
3. **Matching heuristic.** The A* estimate is the minimum-cost bipartite
   matching (Hungarian algorithm) of boxes to goals over true single-box
   push distances — an admissible lower bound on the remaining pushes,
   much sharper than a sum of nearest-goal distances.
4. **Deadlock pruning.** States are cut when a box rests on a dead
   square, when a solid 2×2 block of walls/boxes traps a box off its
   goal, or when the freeze fixpoint (a box walled in on one horizontal
   and one vertical side by walls or already-frozen boxes) catches a box
   off its goal.
5. **Deterministic budgets.** Every search is capped by expansion counts,
   never by the clock, and ties break by fixed square order, so two runs
   of the program write byte-identical answers. A room that resists the
   optimal A* falls back to weighted A* (weights 2, then 5), and a room
   that resists everything triggers a whole-level search, so a hard
   level still gets an answer rather than none.

## Practice-set results

Measured with the kit's own scorer (`program/scorer/score.ts`) on the
kit's practice set. The house baseline is make.fun's plain-astar 1.0.0.

| Instance | P* pushes | Baseline pushes | Baseline points | This solver pushes | This solver points |
|----------|-----------|-----------------|-----------------|--------------------|--------------------|
| i001     | 13        | 13              | 1,000           | 13                 | 1,000              |
| i002     | 16        | unsolved        | 0               | 16                 | 1,000              |
| i003     | 29        | unsolved        | 0               | 29                 | 1,000              |
| i004     | 47        | unsolved        | 0               | 47                 | 1,000              |
| i005     | 60        | unsolved        | 0               | 60                 | 1,000              |
| **Total score** |    |                 | **200,000**     |                    | **1,000,000**      |

The baseline (move-level A*, Manhattan heuristic, 200,000-expansion cap)
solves only the smallest instance before its budget runs out. This solver
solves all five at exactly P* pushes — the maximum score of 1,000,000 —
in about 0.3 s for the whole set on one core, far inside the 60 s sandbox
budget, and repeated runs produce byte-identical output files.

## Running locally

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=<kit>/practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

## Licence

MIT — see `LICENSE`.
