# 🤖 Arduino Mega 2560 Line Follower Robot (PID Based)

> “A small robot, a few sensors, a piece of code… yet it opens the door to a whole world of control, optimization, and engineering passion.”

This project is a **line follower robot** built with an **Arduino Mega 2560**, **5 infrared sensors**, and a **PID controller**.  
The robot is designed with **two distinct modes of operation** that share a **single PID control loop**:

- **ALIGN mode**: The robot remains still and rotates in place until it is perfectly aligned with the track.  
- **RUN mode**: The robot moves forward smoothly, automatically reducing speed on sharp turns to maintain line tracking stability.  

The deeper goal of this project is not just to make a robot follow a line. It is a platform for learning about **real-time embedded systems**, **control theory**, and the **practical art of tuning and optimization**. It shows how a handful of sensors, motors, and some clever programming can lead to a robot that “thinks” about its path.

---

## ⚙️ Hardware Components

- **Arduino Mega 2560** – the main controller with multiple external interrupts  
- **5 digital infrared line sensors** – detect black line against light background  
- **Dual H-Bridge motor driver** – controls two DC motors with PWM speed signals  
- **2 DC motors with wheels** – provide traction and movement  
- **Push button** (D21) – toggles between ALIGN and RUN modes  
- **Battery pack** – LiPo or NiMH for mobility  

### Pin Mapping

| Signal         | Mega 2560 Pin |
|----------------|---------------|
| PI1            | D2  (INT4)    |
| PI2            | D3  (INT5)    |
| PI3            | D18 (INT3)    |
| PI4            | D19 (INT2)    |
| PI5            | D20 (INT1)    |
| RUN button     | D21 (INT0)    |
| Motor L IN1    | D8            |
| Motor L IN2    | D9            |
| Motor R IN1    | D10           |
| Motor R IN2    | D11           |
| ENA (PWM left) | D12           |
| ENB (PWM right)| D13           |

> If the robot turns the wrong way, simply flip the `SENSOR_LEFT_IS_PI1` constant in code or swap motor driver wires.

---

## 🎯 Control Principle

The robot continuously calculates an **error value** based on sensor readings. Each of the 5 sensors is given a weight (-2 to +2), and the error is the weighted average of active sensors. This error feeds into a **PID controller**:

- **Proportional (Kp)**: immediate correction proportional to error  
- **Integral (Ki)**: accumulates small persistent errors to remove long-term drift  
- **Derivative (Kd)**: predicts future error by measuring rate of change, reducing overshoot and oscillation  
- **Anti-Windup (Kaw)**: prevents integral overflow when actuators are saturated  
- **Low-pass derivative filter**: smooths out sensor noise before applying derivative term  

Motor PWM outputs are computed as:

```cpp
pwmL = base - u_cmd
pwmR = base + u_cmd
```

- In **ALIGN mode**, base = 0 → robot rotates until centered  
- In **RUN mode**, base speed is dynamically reduced depending on |u| (turn sharpness), allowing high speed on straight lines but careful tracking in curves  

This creates a robot that is both **fast and stable**.

---

## 🚀 Getting Started

1. Flash the provided code to your **Arduino Mega 2560**.  
2. Place the robot on a light-colored track with a black line.  
3. Open the Serial Monitor at **115200 baud**.  
4. On startup, the robot is in **ALIGN mode**.  
5. Press the D21 button to switch between **RUN** and **ALIGN**.  
6. Use serial commands to tune PID parameters in real time.  

---

## 🎛️ Serial Commands (Live Tuning)

- `show` – display current parameters  
- `kp 60` – set Kp = 60  
- `ki +0.01` – increase Ki by 0.01  
- `kd 20` – set Kd = 20  
- `pid 55 0 25` – set Kp=55, Ki=0, Kd=25  
- `base 120` – set base speed  
- `kspeed 0.6` – set speed reduction factor on turns  
- `aligndb 0.15` – set ALIGN deadband  
- `mode run | align | auto` – force mode  

This interactive tuning lets you find the perfect balance between **speed** and **stability**.

---

## 🖥️ Serial Log Example

```text
RUN   bits=11100 e=-0.33 u=-45 base=120 L=75 R=165
ALIGN bits=00100 e=0.00  u=0   base=0   L=0  R=0
```

- `bits` – 5 sensor states (1 = black line detected)  
- `e` – error value (-2 to +2)  
- `u` – PID controller output  
- `base` – current base speed  
- `L/R` – PWM signals sent to motors  

---

## 🔧 Quick Tuning Guide

| Symptom                           | Adjustment |
|-----------------------------------|------------|
| Vibrates on straight line         | Increase **Kd**, decrease **Kp** |
| Corrects too slowly               | Increase **Kp** |
| Misses curves                     | Increase **Kp**, decrease **kspeed** |
| Doesn’t move                      | Increase **minPWM** |
| Shaky in ALIGN mode               | Increase **alignDeadbandE** |

> Tuning is as much art as science. The right values depend on your motors, wheels, track surface, and even battery level.

---

## 🌱 Why This Project Matters

This line follower is **more than just a toy**. It demonstrates the practical intersection of multiple engineering disciplines:

- **Electronics**: reading digital sensors, driving motors with PWM, debouncing inputs  
- **Control theory**: implementing PID, filtering signals, preventing windup  
- **Embedded programming**: using interrupts, timers, and efficient code on microcontrollers  
- **Problem solving**: adjusting parameters, testing on real tracks, iterating to improve performance  

It is a **hands-on learning tool**:  
- For beginners, it’s a fun way to understand the basics of automation.  
- For advanced learners, it’s a testbed for experimenting with **adaptive control, sensor fusion, or even machine learning**.  

In short: A small robot on a simple line, but a **big step in the journey of mastering robotics and automation**.

---

## 📄 License

MIT License – free to use, learn from, and improve.

---
