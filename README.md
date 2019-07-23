# PiGaCo Code Repository

Wayland & (Arch) Linux based Arcade Gaming Console

This repository contains all the code required to run the PiGaCo
system.

# About this Repository and Architecture

This is a reduced system compared to the original vision. It is minimal
in its complexity and functions by directly injecting player input into
the Linux input handling subsystem. The vision of networked other players
joining games can still be completed one day, but until then, fun should
be had with the existing system.

This has the additional benefit of providing a solid base that can be
expanded and built upon step-by-step. In the next steps, the most
important parts are added - after that, only imagination stops a thing
like this!

# Components

## pigacompositor

Wayland Compositor (based on tinywl) to display
games.

## pigainput

Handles inputs and LED outputs on the device. Connects via USB serial
and unix pipe.

## pigaprobe

Utility to test the LED outputs.

## pigaprotocol

Protocol that is used by pigainput specified in a C header file.

## arduino

Arduino onboard code.
