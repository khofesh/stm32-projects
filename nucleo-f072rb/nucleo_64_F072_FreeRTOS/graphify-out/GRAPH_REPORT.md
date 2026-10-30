# Graph Report - nucleo_64_F072_FreeRTOS  (2026-09-03)

## Corpus Check
- 42 files · ~201,706 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 559 nodes · 1510 edges · 27 communities
- Extraction: 89% EXTRACTED · 11% INFERRED · 0% AMBIGUOUS · INFERRED: 167 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Queue & Semaphore API
- Task API & Macros
- ARM-CM0 Port Core
- Stream Buffers
- Software Timers
- Config & Headers
- Co-routines
- Task Scheduling Internals
- Event Groups
- Application Tasks
- MPU Queue Wrappers
- MPU Timer Wrappers
- Task Creation Internals
- MPU Task-State Wrappers
- Heap & Task Termination
- Task Status & Runtime
- Atomic Operations
- MPU Utility Wrappers
- MPU Runtime Counters
- MPU Event Group Wrappers
- Restricted Tasks
- MPU Queue Sets
- MPU Timeout Wrappers

## God Nodes (most connected - your core abstractions)
1. `xTaskResumeAll()` - 26 edges
2. `vTaskSuspendAll()` - 23 edges
3. `uxListRemove()` - 20 edges
4. `xQueueSemaphoreTake()` - 18 edges
5. `xQueueGenericSend()` - 17 edges
6. `xQueueReceive()` - 16 edges
7. `prvInitialiseNewTask()` - 15 edges
8. `xQueuePeek()` - 14 edges
9. `xTaskRemoveFromEventList()` - 14 edges
10. `prvAddCurrentTaskToDelayedList()` - 14 edges

## Surprising Connections (you probably didn't know these)
- `main()` --calls--> `xTaskCreate()`  [INFERRED]
  app/src/main.c → FreeRTOS/tasks.c
- `vTask1()` --calls--> `vTaskDelay()`  [INFERRED]
  app/src/main.c → FreeRTOS/tasks.c
- `vTask2()` --calls--> `vTaskDelay()`  [INFERRED]
  app/src/main.c → FreeRTOS/tasks.c
- `main()` --calls--> `vTaskStartScheduler()`  [INFERRED]
  app/src/main.c → FreeRTOS/tasks.c
- `main()` --calls--> `BSP_Console_Init()`  [INFERRED]
  app/src/main.c → bsp/src/bsp.c

## Import Cycles
- None detected.

## Communities (27 total, 0 thin omitted)

### Community 0 - "Queue & Semaphore API"
Cohesion: 0.10
Nodes (69): BaseType_t, QueueHandle_t, QueueSetHandle_t, QueueSetMemberHandle_t, TaskHandle_t, TickType_t, UBaseType_t, pcQueueGetName() (+61 more)

### Community 1 - "Task API & Macros"
Cohesion: 0.08
Nodes (44): ConstTaskHandle_t, eNotifyAction, TaskHandle_t, TaskHookFunction_t, UBaseType_t, freertos_tasks_c_additions_init(), pcTaskGetName(), prvResetNextTaskUnblockTime() (+36 more)

### Community 2 - "ARM-CM0 Port Core"
Cohesion: 0.06
Nodes (36): eSleepModeStatus, BaseType_t, configSTACK_DEPTH_TYPE, StackType_t, TaskFunction_t, TaskHandle_t, TickType_t, xMPU_SETTINGS (+28 more)

### Community 3 - "Stream Buffers"
Cohesion: 0.12
Nodes (41): else, BaseType_t, StreamBufferHandle_t, TickType_t, UBaseType_t, if(), prvBytesInBuffer(), prvBytesInBufferMeetTriggerLevel() (+33 more)

### Community 4 - "Software Timers"
Cohesion: 0.14
Nodes (36): BaseType_t, TaskHandle_t, TickType_t, TimerHandle_t, UBaseType_t, pcTimerGetName(), prvCheckForValidListAndQueue(), prvGetNextExpireTime() (+28 more)

### Community 5 - "Config & Headers"
Cohesion: 0.07
Nodes (13): MPU_SETTINGS, ulContext, ulTaskFlags, xRegionsSettings, MPURegionSettings, ulRASR, ulRBAR, SYSTEM_CALL_STACK_INFO (+5 more)

### Community 6 - "Co-routines"
Cohesion: 0.11
Nodes (27): crCOROUTINE_CODE, BaseType_t, List_t, STATIC, TickType_t, UBaseType_t, prvCheckDelayedList(), prvCheckPendingReadyList() (+19 more)

### Community 7 - "Task Scheduling Internals"
Cohesion: 0.16
Nodes (27): vEventGroupDelete(), BaseType_t, List_t, ListItem_t, TickType_t, prvAddCurrentTaskToDelayedList(), prvSearchForNameWithinSingleList(), pvTaskGetThreadLocalStoragePointer() (+19 more)

### Community 8 - "Event Groups"
Cohesion: 0.18
Nodes (24): BaseType_t, EventBits_t, EventGroupHandle_t, TickType_t, UBaseType_t, prvTestWaitCondition(), uxEventGroupGetNumber(), vEventGroupClearBitsCallback() (+16 more)

### Community 9 - "Application Tasks"
Cohesion: 0.13
Nodes (16): TaskHandle_t, main(), SystemClock_Config(), vApplicationStackOverflowHook(), vTask1(), vTask2(), my_printf(), print() (+8 more)

### Community 10 - "MPU Queue Wrappers"
Cohesion: 0.13
Nodes (23): BaseType_t, QueueHandle_t, TickType_t, MPU_pcQueueGetName(), MPU_ulTaskGenericNotifyTake(), MPU_vQueueAddToRegistry(), MPU_vQueueUnregisterQueue(), MPU_vTaskDelay() (+15 more)

### Community 11 - "MPU Timer Wrappers"
Cohesion: 0.17
Nodes (19): StreamBufferHandle_t, TimerHandle_t, MPU_pcTimerGetName(), MPU_pvTimerGetTimerID(), MPU_uxTimerGetReloadMode(), MPU_vTimerSetReloadMode(), MPU_vTimerSetTimerID(), MPU_xStreamBufferBytesAvailable() (+11 more)

### Community 12 - "Task Creation Internals"
Cohesion: 0.30
Nodes (19): configSTACK_DEPTH_TYPE, StackType_t, TaskFunction_t, prvAddNewTaskToReadyList(), prvCreateIdleTasks(), prvCreateStaticTask(), prvCreateTask(), prvInitialiseNewTask() (+11 more)

### Community 13 - "MPU Task-State Wrappers"
Cohesion: 0.12
Nodes (18): configSTACK_DEPTH_TYPE, eTaskState, TaskHandle_t, TaskHookFunction_t, MPU_eTaskGetState(), MPU_pvTaskGetThreadLocalStoragePointer(), MPU_uxTaskGetStackHighWaterMark2(), MPU_vTaskGetInfo() (+10 more)

### Community 14 - "Heap & Task Termination"
Cohesion: 0.18
Nodes (17): pvPortMalloc(), vPortFree(), STATIC, portTASK_FUNCTION(), prvCheckTasksWaitingTermination(), prvDeleteTCB(), prvGetExpectedIdleTime(), prvInitialiseTaskLists() (+9 more)

### Community 15 - "Task Status & Runtime"
Cohesion: 0.22
Nodes (16): configRUN_TIME_COUNTER_TYPE, eTaskState, TaskStatus_t, eTaskGetState(), prvCallForEachTask(), prvForEachTaskInList(), prvGetTotalRunTime(), prvTaskStatusArrayWriter() (+8 more)

### Community 16 - "Atomic Operations"
Cohesion: 0.28
Nodes (12): Atomic_Add_u32(), Atomic_AND_u32(), Atomic_CompareAndSwap_u32(), Atomic_CompareAndSwapPointers_p32(), Atomic_Decrement_u32(), Atomic_Increment_u32(), Atomic_NAND_u32(), Atomic_OR_u32() (+4 more)

### Community 17 - "MPU Utility Wrappers"
Cohesion: 0.20
Nodes (10): UBaseType_t, MPU_ulTaskGenericNotifyValueClear(), MPU_uxEventGroupGetNumber(), MPU_uxQueueMessagesWaiting(), MPU_uxQueueSpacesAvailable(), MPU_uxTaskGetNumberOfTasks(), MPU_uxTaskGetStackHighWaterMark(), MPU_uxTaskPriorityGet() (+2 more)

### Community 18 - "MPU Runtime Counters"
Cohesion: 0.29
Nodes (7): configRUN_TIME_COUNTER_TYPE, TaskStatus_t, MPU_ulTaskGetIdleRunTimeCounter(), MPU_ulTaskGetIdleRunTimePercent(), MPU_ulTaskGetRunTimeCounter(), MPU_ulTaskGetRunTimePercent(), MPU_uxTaskGetSystemState()

### Community 19 - "MPU Event Group Wrappers"
Cohesion: 0.38
Nodes (7): EventBits_t, EventGroupHandle_t, MPU_xEventGroupClearBits(), MPU_xEventGroupSetBits(), MPU_xEventGroupSync(), MPU_xEventGroupWaitBitsEntry(), xEventGroupWaitBitsParams_t

### Community 20 - "Restricted Tasks"
Cohesion: 0.48
Nodes (7): prvCreateRestrictedStaticTask(), prvCreateRestrictedTask(), xTaskCreateRestricted(), xTaskCreateRestrictedAffinitySet(), xTaskCreateRestrictedStatic(), xTaskCreateRestrictedStaticAffinitySet(), TaskParameters_t

### Community 21 - "MPU Queue Sets"
Cohesion: 0.67
Nodes (4): QueueSetHandle_t, QueueSetMemberHandle_t, MPU_xQueueAddToSet(), MPU_xQueueSelectFromSet()

### Community 23 - "MPU Timeout Wrappers"
Cohesion: 0.67
Nodes (3): TimeOut_t, MPU_vTaskSetTimeOutState(), MPU_xTaskCheckForTimeOut()

## Knowledge Gaps
- **9 isolated node(s):** `ulRBAR`, `ulRASR`, `ulSystemCallStackBuffer`, `pulSystemCallStack`, `pulTaskStack` (+4 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 68 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `vTaskDelay()` connect `Task Scheduling Internals` to `Application Tasks`, `Task API & Macros`?**
  _High betweenness centrality (0.045) - this node is a cross-community bridge._
- **Why does `vTask2()` connect `Application Tasks` to `Task Scheduling Internals`?**
  _High betweenness centrality (0.030) - this node is a cross-community bridge._
- **Why does `main()` connect `Application Tasks` to `Task API & Macros`, `Task Creation Internals`?**
  _High betweenness centrality (0.029) - this node is a cross-community bridge._
- **Are the 10 inferred relationships involving `xTaskResumeAll()` (e.g. with `vEventGroupDelete()` and `xEventGroupSetBits()`) actually correct?**
  _`xTaskResumeAll()` has 10 INFERRED edges - model-reasoned connections that need verification._
- **What connects `ulRBAR`, `ulRASR`, `ulSystemCallStackBuffer` to the rest of the system?**
  _9 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Queue & Semaphore API` be split into smaller, more focused modules?**
  _Cohesion score 0.09730848861283643 - nodes in this community are weakly interconnected._
- **Should `Task API & Macros` be split into smaller, more focused modules?**
  _Cohesion score 0.08244897959183674 - nodes in this community are weakly interconnected._