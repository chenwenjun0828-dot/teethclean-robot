# Add Y/Z corridor and replan

1. Keep X unchanged; add directed projected-normal Y reference with configurable
   offset and 60 degree initial corridor. Derive Z by the right-hand relation.
2. Filter reachability and IK candidates with one degree margin; constrain shortest
   roll interpolation to remain inside the corridor. Penalize spatial roll variation.
3. Independently audit actual timed Y/Z axes and physical spin speed/acceleration.
   Bind policy to request/report/reference and reject plans missing these checks.
4. Replan both jaws at a 5 degree roll grid. Try local bridge trimming before global
   trim alternatives, preserve four-tooth minimum coverage and single trace block.
5. Improve local timing convergence; retain exact polynomial limit checks and full
   motion audits. Explicitly scale for physical axial spin limits. Run headless dynamics.
6. Test kinematics against real MuJoCo Jacobians, directed axis limits, graph smoothness,
   forbidden wraparound and policy provenance. Record results and coverage tradeoffs.
