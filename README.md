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
## 02:Semaphores examples
### timeline
0s-------------------------------5s----------------------10s
* 0s:Thread1(H) obtains semaphore
* 5s:Thread1 release the semaphore and suspends itself.Thread2 obtains the semaphore,resume Thread1
* 10s:Thread2 release the semaphore,end of cycle
## 03:SemaphoresISR examples
### timeline
0s-------------------------------x-y-z-------------------10s
* 0s:No semaphore,Thread 1 blocked
* x:button click,EXTI ISR created a semaphore
* y:Thread1 unblocked,and get the semaphore
* z:No semaphore again!Thread1 blocked
## mutex
1. cubemx 
* FreeRTOS configuration->Include parameters->Enable eTaskGetState
### timeline
0s-----------a-b-c-d-e-f-g-h-i-j-k-
* 0s:thread1(H) take the mutex,and it calls the delay
* a:thread2(M)found no mutex,blocked
* b:thread3(L)found no mutex,blocked
* c:thread1 release mutex,and suspend itself<-
* d:thread2 take the mutex,and then release the mutex,finally suspend itself
* e:thread3 take the mutex,and then resume thread2
* f:thread2 found no mutex,blocked
* g:thread3 resume thread1
* h:thread1 found no mutex,blocked
* i:thread3 release mutex
* j:thread1 take the mutex,and it calls the delay
* k:thread3 found no mutex,blocked.->
## queue
### timeline
0s-a-b--------------1s
* 0s:Thread1 put a message then delay
* a:Thread2 get the message 
* b:Thread2 try to get the message again,but no message,then blocked
* 1s:Thread1 put a message then delay.end of cycle
## timer
1. cubemx
* FreeRTOS configuration->Config parameters->Enable USE_TIMER
* create a timer
### timeline
startup---x-------
* startup:initiate the timer then strart the timer
* x:timer is using osWaitUntil() blink every 200ms
## LowPower
1. cubemx
* set all free pins as analog(to optimize the power consumption)
2. add some code to configFreeRTOS.h
```
#if configUSE_TICKLESS_IDLE == 1 
#define configPRE_SLEEP_PROCESSING                        PreSleepProcessing
#define configPOST_SLEEP_PROCESSING                       PostSleepProcessing
#endif /* configUSE_TICKLESS_IDLE == 1 */
```
every tick,mcu will wakeup,if no intend task need to do,it will sleep again.otherwise,It won't sleep till next tick!