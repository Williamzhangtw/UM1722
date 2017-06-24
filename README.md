Learning curve of FreeRTOS
# UM1722 examples
## 01:Thread Creation example
* The aim of this example is to explain how to create threads using CMSIS-RTOS based on FreeRTOS API.
The example implements two threads running with the same priority, which execute in a periodic cycle.
Below details about each thread execution.
Thread 1: this thread toggles the LED1 each 200 ms for 5 seconds and then it suspends itself, after 
5 seconds the thread 2 resume the execution of thread 1 which toggles the LED1each 400 ms for the next 5 seconds
### timeline
0s-------------------------------5s----------------------10s-------------------15S
* 0s: Thread 1 toggles LED1 each 200ms;Thread 2 toggles LED1 each 500ms
* 5s:Thread1 suspends itself
* 10s:  Thread2 resume thread1; thread2 suspend itself;thread1 LED1 each 400ms
* 15s: thread1 resume thread2; end of the cycle!
## 02:add doxygen
* Don't slect the prepare for compressed HTML(.chm) otherwise,the search function will be lost.
## 03:Semaphores examples
### timeline
0s-------------------------------5s----------------------10s
* 0s:Thread1(H) obtains semaphore
* 5s:Thread1 release the semaphore and suspends itself.Thread2 obtains the semaphore,resume Thread1
* 10s:Thread2 release the semaphore,end of cycle
## 04:SemaphoresISR examples
### timeline
0s-------------------------------x-y-z-------------------10s
* 0s:No semaphore,Thread 1 blocked
* xs:button click,EXTI ISR created a semaphore
* y:Thread1 unblocked,and get the semaphore
* z:No semaphore again!Thread1 blocked
