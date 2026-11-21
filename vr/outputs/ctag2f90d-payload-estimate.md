# CTAG2F90-D empty gripper estimate

Offline worksheet, 2026-11-19. Each gripper has a nominal mass of 0.8 kg
(Changingtek selection catalog V3.1, printed page 20, PDF page 24).
The drawing shows heights of 205 and 222.5 mm for the illustrated jaw
positions. It does not specify component masses, center of mass or inertia.

Distance below is measured from the gripper's own mounting face toward the
fingertips. To illustrate sensitivity, approximate the housing and drive as
concentrated at 50 mm and the moving links and fingers at 150 mm. Both
locations and all mass splits below are assumptions, not measured values.

| Housing mass fraction | Axial center of mass |
| --- | --- |
| 60% | 90 mm |
| 70% | 80 mm |
| 80% | 70 mm |

For either gripper the nominal scenario is 0.8 kg at 80 mm. The three
scenarios are not bounds on the actual center of mass. At a horizontal
offset of 80 mm the gravity moment about the mounting face is 0.63 N m.
An axial error of 20 mm gives up to 0.16 N m of gravity moment error.

The operator confirms direct mounting without an adapter on both arms.
For an offline inertia estimate, define an axial coordinate from the
mounting face toward the fingers and assume zero lateral offset. Model
the housing as a 0.56 kg solid cylinder of radius 32 mm and length 100 mm
centered at 50 mm. Model the fingers and links as a coaxial 0.24 kg solid
cylinder of radius 40 mm and length 100 mm centered at 150 mm. These
effective radii approximate distributed material; the real fingers open.

For each cylinder, transverse inertia is m*(3*r^2+h^2)/12 and axial
inertia is m*r^2/2, using meters. Shifting each transverse inertia to the
combined center of mass adds m*(z-0.08)^2. The estimated principal moments
about the center of mass are 0.002586, 0.002586 and 0.000479 kg m^2;
the last is axial. About the mounting-face center the transverse moments
would instead be 0.007706 kg m^2; the axial moment is unchanged.
Products of inertia vanish by the assumed symmetry. These are different
reference points for the same model, not two alternative tunings.

These quantities are in the physical gripper frame. Encoding them for
Apex requires the SDK's input units, coordinate axes and inertia reference
point. No robot configuration or mode change is part of this worksheet.
