# PiGaCo Code Repository

Wayland & (Arch) Linux based Arcade Gaming Console

This repository contains all the code required to run the PiGaCo
system.

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
