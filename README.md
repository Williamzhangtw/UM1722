Learning curve of FreeRTOS
## UM1722 examples
### Thread Creation example
* The aim of this example is to explain how to create threads using CMSIS-RTOS based on FreeRTOS API.
The example implements two threads running with the same priority, which execute in a periodic cycle.
Below details about each thread execution.
Thread 1: this thread toggles the LED1 each 200 ms for 5 seconds and then it suspends itself, after 
5 seconds the thread 2 resume the execution of thread 1 which toggles the LED1each 400 ms for the next 5 seconds
#### timeline
0s-------------------------------5s----------------------10s-------------------15S
* 0s: Thread 1 toggles LED1 each 200ms;Thread 2 toggles LED1 each 500ms
* 5s:Thread1 suspends itself
* 10s:  Thread2 resume thread1; thread2 suspend itself;thread1 LED1 each 400ms
* 15s: thread1 resume thread2; end of the cycle!
