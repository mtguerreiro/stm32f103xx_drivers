    .cpu cortex-m3
    .eabi_attribute 20, 1
    .eabi_attribute 21, 1
    .eabi_attribute 23, 3
    .eabi_attribute 24, 1
    .eabi_attribute 25, 1
    .eabi_attribute 26, 1
    .eabi_attribute 30, 6
    .eabi_attribute 34, 1
    .eabi_attribute 18, 4
    .align  1
    .global delaysSub
    .arch armv7-m
    .syntax unified
    .thumb
    .thumb_func
    .fpu softvfp
delaysSub:
    sub r0, #1
    cmp r0, #0
    bne delaysSub
    bx  lr
