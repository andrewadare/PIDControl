# PIDControl
A simple platform-independent C++ PID controller class

## About this class:
- No dependencies on Arduino or any other MCU framework
- Minimal API: fields are public for simple access
- The core logic is all in the `update` function. It has basic provisions for reducing derivative kick and integrator windup.
- No manual/auto selection (always on; switch externally if needed.)
- This is a conventional reverse-acting loop. If a direct-acting loop is needed,
  negate the output (possibly with an offset).

## Useful references
This draws from the [Arduino library](https://github.com/br3ttb/Arduino-PID-Library) by Brett Beauregard. See also the the author's [accompanying explanations](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/).

Fixing PID [part 1](http://www.controleng.com/single-article/fixing-pid/3975cad3f121d8df3fc0fd67660822b1.html), [part 2](http://www.controleng.com/single-article/fixing-pid-part-2/733b03c4156175cebc59206ca529d9ec.html?tx_ttnews%5BsViewPointer%5D=1), and [part 3](http://www.controleng.com/single-article/fixing-pid-part-3/ceebd45d413d52b55da6a9c161dd177d.html) by Vance VanDoren

[PID Without a PhD](http://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf) by Tim Wescott

[Feedback Systems](www.cds.caltech.edu/~murray/books/AM05/pdf/am08-complete_22Feb09.pdf) textbook by K. Astrom and R. Murray