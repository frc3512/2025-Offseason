# FRC Team 3512's Offseason code, using the 2025 summer robot, Karkinos

## Goals

* Implement a functional IO codebase, as practice for the 2026 season
* Practice optimal ways of double binding, for a solo driver
* Use as a reference to teach the new students how to work with IO

## Game

This code is based on the 2025 game, Reefscape. It uses elements like the Reef, Barge, and Processor

## Robot

The robot that this code is being used on is the robot our team constructed over the summer, Karkinos.

### Robot Features

* 3 degrees of freedom, compared to the 1 degree on our COMP bot
* Universal intake, with a CanandColor for game peice detection, that can ground intake and preform all actions
* No form of climber :(
* MK4i Swerve Drive
* Robot uses Kraken X60's and X44's only

### Subsytems

#### Arm

120 degrees rotational arm, using a cyclodial gearbox and a Canandmag to ensure a vetical zero. Mounted to The elevator carriage

#### Elevator

Two stage elevator + carriage, powered by 2 Kraken X60's. Interally belted and continuous. Roughly 60 inches of max height

#### Intake

Claw shapped intake, for both game peices. Canandcolor mounted in between wheels to detect if we have a game peice, and which one

#### Wrist

90 Degree wrist, rotates intake to allow ground intake as well as scoring

## Current Version:
3.0.1
