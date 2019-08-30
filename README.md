# Robotics Intensive Training

## Introduction

This is the project of lesson **Robotics Intensive Training** in ZJU. 

We got the result: 17-3 in task_1, 7-14 in task_2, 7-23 in task_3.

This result is not really good. However, we hope it can help you with your new ideas.

The whole project is based on python3.

## Features

- **RRT** for global planner and multiple improvement version such as RRT with predicting expanding radius based on velocity and acceleration

- Local planner with **potential feild** to calculate velocity, which considering tangential speed and radial speed helps us get through task_3.

- **Multithreading** to send message with higher than 60fps

- A primary **Simulation** of task_2 based on multithreading

- A easy **frame** to try your algorithm

- A beautiful **visualization** of your predicting path

## Installation

### Requirements

- Windows
- python 3.5+
- protobuf
- ZJU client

## Getting Started

### EASY FRAME

Run main.py to test your algorithm. 

You can change the variable to quickly test.

- global_p(for path planning) 
  Your algorithm should contain functions you will use in **RUN**. Based on mine, you should contain funciton **Generate_Path** which returns a status, a tree of your planning and its lines, function **Get_path** which returns the final path and path_lines, function **merge** for the purpose that you want your path to be straight which returns path and path_lines.

- local_p(for velocity calculating) 
  You should finish your velocity calculating and sending in this part. We provide functions such as **path_control**, **point_control**, **line_control**

- RUN(for your different purpose like change the expanding radius in RRT when your car is too near with the barrier)
  You should finish all the other things in this part. And we provide different verison of different purpose.

### Multithreading

Run final.py to get better perfomance based on the algorithm in main.py and all you need to do is to change your code following the codes in thread_global and thread_local

- Please make your sending fps lower than 200, or your client will crash

### TEST

At the end of the lesson, we finally use main.py with RRT for task_1, and biubiu.py for task_2, and task_3 with different parameters and potential feild to calculate velocity based on multithreading. 

## collaborators
The collaborators of this project are [BaixuLi](https://github.com/BaixuLi) and [kyleleey](https://github.com/kyleleey) 
