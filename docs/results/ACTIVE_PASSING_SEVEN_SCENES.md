# Simple Active Passing: seven classic scenarios

This is the frozen seed-17, single-run result for controller source commit
`572325f` (`active_passing_simple.py`). Each scenario used the same controller;
there was no scenario-name branch in the control logic.

| Scenario | Result | Main behavior | Maximum lateral displacement |
| --- | --- | --- | ---: |
| Static obstruction | PASS | Right-side pass, recover, goal | 1.425 m |
| Head-on passing | PASS | Right-side pass, recover, goal | 1.425 m |
| Overtaking | PASS | Passed from behind to ahead, recovered, reached goal | 1.425 m |
| Perpendicular crossing | PASS | Yielded, resumed, reached goal | 0.000 m |
| Diagonal crossing | PASS | Yielded, resumed, reached goal | 0.000 m |
| Side-offset static | PASS | Adjusted right-side offset for footprint clearance | 1.868 m |
| Cut-in / merge | PASS | Yielded, passed, recovered, reached goal | 2.217 m |

All seven runs completed the route with `collision_proxy=false` and
`static_collision=false`. The three passing scenarios also met their specific
passing/overtaking success checks. These are **one run per scenario**, not a
multi-seed statistical benchmark or a real-world safety guarantee.

The full local record is `outputs/active_passing_simple/` in the original
worktree. Raw traces and first-/third-person MP4s are intentionally excluded
from Git; the table above preserves the compact results without implying that
the videos are available in this repository.
