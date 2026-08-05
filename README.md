# GRASPAW

A robot arm on a boat that picks trash out of the water.

This was my M.Tech thesis at IIT Hyderabad (2025–2026), supervised by Prof. R. Prashant Kumar. I built the whole thing — the boat, the arm, the vision, the software — and then took it outside and found out what actually breaks.

📹 **[Watch it pick up an object and place it](https://drive.google.com/file/d/19y88qOxWlNAnvGD7hjhhpgPssVoBzQ9n/view?usp=sharing)**

---

## Why bother

Most low-cost cleanup boats drag a net or a conveyor and scoop up whatever floats into them. That works, but it's indiscriminate — you collect everything, including things you'd rather not.

I wanted the boat to *choose*: see a specific object, reach out, and pick that one up. Which sounds like a small change and isn't, because now perception has to talk to a manipulator sitting on a platform that won't hold still.

---

## How it's put together

```
USB camera ──► find the object (OpenCV) ──► where is it?
                                                │
                                                ▼
                                    work out the joint angles
                                                │
                                                ▼
                    send to servos ──► 5-DOF arm ──► grab it, drop it in the bin
```

- **Boat:** PVC hull, two brushless thrusters, differential steering, driven over WiFi from an ESP32
- **Arm:** 3D-printed, 5 degrees of freedom plus a gripper, Feetech STS3215 smart servos on a serial bus
- **Brain:** Raspberry Pi onboard, talking to the servos at 1 Mbaud over half-duplex serial

---

## Seeing things on water

Water is a miserable surface to do vision on. The sun reflects off it, ripples make the reflections move, and half of what your detector finds is glare.

The pipeline is deliberately simple — HSV colour segmentation, then Hough circles to find round floating objects — but with a lot of preprocessing to kill reflections: median blur, morphological opening, and parameters tuned outdoors rather than at a desk. Parameters that worked perfectly indoors were useless the first time I took it out.

---

## The kinematics problem, and what I did about it

Textbook inverse kinematics assumes your robot matches its model. Mine didn't. 3D-printed links aren't the exact length you designed, hobby servos have backlash, and the zero position of each joint is wherever you decided it was during calibration. So the solver would confidently return joint angles that were correct for a robot I didn't own, and the gripper would land a couple of centimetres off.

So I stopped using the model. Instead I moved the arm by hand to a grid of known positions, wrote down what the servos read, and for any new target I blend the nearest taught points (inverse-distance weighting, k=4, p=2). The manufacturing error is baked into the measurements, so it doesn't matter that the model is wrong — I never ask the model anything.

📹 **[Checking positioning accuracy against a calibration grid](https://drive.google.com/file/d/1-OLu75R4NQuBHhhrjh9LcSupnYN3qVIO/view?usp=sharing)** — the arm commanded to known points on a checkerboard, to see where it actually lands.

**The catch:** the grid belongs to that one physical arm. Change a link, replace a servo, crash it — and you teach it all over again. Analytic IK doesn't have that problem. It's a genuine trade, not a free win.

---

## Simulation

Before touching hardware I modelled the arm in URDF/Xacro and checked reachability, joint limits and planning in MoveIt 2 and Gazebo, using position-only IK since 5 DOF can't satisfy full pose anyway.

📹 **[Pick and place in Gazebo](https://drive.google.com/file/d/1Wp98C3db8G3Mc64qr6rZjcL8hFTYX7ZU/view?usp=sharing)**

The simulation setup is in this repo, and it's where the next round of experiments will run.

---

## What I used

ROS 2 · MoveIt 2 · Gazebo · RViz · OpenCV · Python · URDF/Xacro · ESP32 · Raspberry Pi · Feetech servo SDK

---

## Things that broke

The honest section, and the part I learned most from:

- **Power browned out** whenever several servos moved at once. The arm would just die mid-reach. Fixed by actually budgeting the supply instead of assuming.
- **Servos disagreed with each other** — firmware and protocol differences between units I'd assumed were identical.
- **The gripper encoder wrapped around** at its multi-turn seam, so the gripper would open when told to close. Needed wrap-aware logic.
- **Vision fell apart outdoors.** See above. This one took the longest.

---

## What it doesn't do

- No learned detector — it's classical CV, tuned for specific objects
- The teach grid only works for this exact arm
- Assumes the target isn't moving much, and doesn't compensate for the boat rocking
- The boat is driven by a human; it doesn't navigate itself

---

## What I'm doing now

Rebuilding the arm to run a proper experiment: how much worse does model-based IK actually get as the hardware drifts from its model, and how many taught points do you need before the teach-grid beats it? Positioning error, repeatability and grasp success, measured properly this time, across teach-grid density and injected model error — in simulation first, then on hardware.

Design files, code and data will go up here as it progresses.

---

## About the code

This is thesis code. It works, but it wasn't written to be read by anyone else. I'm cleaning it up as I go through the rebuild — if something's unclear, ask.

---

## Get in touch

**Battagani Vijaya Sathwik** — M.Tech, Mechanics and Design, IIT Hyderabad
[sathwik.bv@gmail.com](mailto:sathwik.bv@gmail.com) · [LinkedIn](https://www.linkedin.com/in/vijaya-sathwik-battagani-95200a23b/)
