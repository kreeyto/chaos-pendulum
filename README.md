# Chaos Pendulum (MATLAB)

This project contains an interactive and animated MATLAB script that simulates the motion of a **double pendulum** — a classic example of a chaotic system in physics.

## 🧠 What is a double pendulum?

Imagine two masses hanging from ropes, with one rope attached to the other in sequence. When released, the weights swing back and forth in a highly complex and unpredictable way. This system is very sensitive to initial conditions and is a textbook example of deterministic chaos.

## 📂 Structure

- `pendulum.m`: the only file in the project. It contains the full simulation logic, animation, and plotting routines.

## ▶️ How it works

1. When you run `pendulum.m`, the program asks for the desired simulation duration.
2. It then solves a system of differential equations that governs the motion of the double pendulum.
3. The animation shows the pendulums moving in real time.
4. At the end:
   - If the simulation completes normally, a single graph is displayed showing the full trajectory.
   - If the simulation is stopped early, two plots appear:
     - One showing the trajectory up to the stopping point.
     - Another showing the full potential trajectory (as if the simulation had run to the end).

## 💻 Requirements

- MATLAB (R2020a or later is recommended)

## 📌 Notes

This project is great for visualizing the behavior of nonlinear systems and exploring the concept of chaos. It can be used for educational purposes or simply as a scientific curiosity.

---
