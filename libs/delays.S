.cpu cortex-m3
.arch armv7-m
.syntax unified
.thumb
.thumb_func

.global delaysLoop
delaysLoop:
    subs r0, #1
delaysLoopLoop:
    subs r0, #1
    bne delaysLoopLoop
    bx  lr

.global delaysus
delaysus:
	mov r1, #12
	mul r0, r1, r0
    subs r0, #1
delaysusLoop:
    subs r0, #1
    bne delaysusLoop
    bx  lr
