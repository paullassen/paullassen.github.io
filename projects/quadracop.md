---
layout: default
title: Beaglebone-Quadracopter
parent: Projects
nav_order: 1
has_children: true
---
# **BeagleBone-Quadracop**
The goal of this project is to take the lessons I have learned over the course of the last several years and combine them to build a quadracopter from scratch.

## Background
I finished my Master's Degree in Electrical Engineering a few weeks ago at the beginning of May. I plan on moving back to the US at the end of summer. This hobby project is a way for me to stay active and engaged in a way that is relevant to my learnings. The decision to build a UAV from the ground up has been driven by a desire to tackle the impostor syndrome which I have felt upon completing my degree.

I intend to document my work here so that it will be repeatable by an interested reader.

## Design Choices
### Base Platform
A BeagleBone Blue serves as the heart of this project, because that's what I had lying around. The reason I had it lying around was because I was curious about the two PRUs onboard the TI am3358. The PRUs (**P**rogrammable **R**eal-Time **U**nits) are, as their name suggests, programmable microcontrollers, capable of running parallel to the main ARM cpu. My plan is to offload the high-speed time sensitive processes to the PRUs.

### Aerial Platform Design
This is a fancy title for how many propellers I want to fly with. I'm starting with a quadracopter because they are a "solved" platform. There are lots of resources on building and controlling quadracopters, as well as a large body of research. This is the first UAV I'm building for personal use (I built a hexacopter for my thesis), so I'm trying not to stray too far from the beaten path.

## Current Status
- 50Hz pulses programmed for all 8 channels using PRU1

## In Progress
- Analysis of I2C
- Acquiring all of the necessary parts
