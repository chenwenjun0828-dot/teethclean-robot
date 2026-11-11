# Open-20 Tool Angle Search

The user requests a visually appropriate scaler pose at jaw opening 20 degrees
that passes all planning and simulation stages. Small deviations from the
in-plane gingival-normal projection constraint are explicitly allowed.

Keep the existing perception and planning algorithms, noncontact 1.5 mm offset,
0.15 mm clearance, 0.15 rad joint step, interpolation tolerances and simulation
speed profile. Search tilt, fixed roll and tilt azimuth; initially prefer
azimuth within 30 degrees of either in-plane normal and roll near 190 degrees.
Default mounting is preferred, with the previously approved 150 mm closer mount
as a secondary experiment. Jaw opening remains exactly 20 degrees.

Use inexpensive analytic IK/graph screening on saved path candidates to reject
obvious discontinuities, then run the unmodified complete planner on promising
angles, including angle-dependent reachable source windows, path construction,
mesh clearance, edge checks, refinement, approach, timing and independent FK
and motion audits. Screening success alone is never an executable result.
Run position-servo simulation only on independently validated trajectories.

Render multiple actual-model views along successful trajectories, including
source arcs and synthetic connectors. Record the active working-end direction
separately from the shaft angle; do not equate shaft tilt to clinical tip
angulation. Manufacturer reference: NSK oral-hygiene tip guide,
https://www.nsk-dental.com/admin/wp-content/uploads/oral_hygiene_tip_guide.pdf
(side of the terminal working end, shallow angle to tooth surface).
Images and three-point tooth planes are geometric proxies, not clinical
validation. The existing workflow treats four middle teeth and simulates
noncontact motion, not actual force-controlled scaling or a complete mouth.

Keep all search attempts and exact reproducible configurations in tmp. Publish
only a result that passes all checks, with coverage, residual errors, minimum
clearance and actual execution status. If constant angles cannot pass, report
the limitation with evidence; do not silently relax safety/accuracy thresholds.
