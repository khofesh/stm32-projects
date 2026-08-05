# ThreadX and NetXDuo Architecture

## Overview

This project uses Azure RTOS (now Eclipse ThreadX), which consists of multiple components:

- **ThreadX**: Real-time operating system kernel
- **NetXDuo**: TCP/IP networking stack built on top of ThreadX

While they appear as separate applications, they are tightly integrated and share the same ThreadX kernel.

## Initialization Flow

```
main.c
    │
    └── MX_ThreadX_Init()
            │
            └── tx_kernel_enter()
                    │
                    └── tx_application_define()   [app_azure_rtos.c]
                            │
                            ├── Create TX byte pool (2KB)
                            │       │
                            │       └── App_ThreadX_Init()   [app_threadx.c]
                            │               │
                            │               └── Your application threads
                            │                   (e.g., sensor threads)
                            │
                            └── Create NX byte pool (160KB)
                                    │
                                    └── MX_NetXDuo_Init()   [app_netxduo.c]
                                            │
                                            └── Networking threads
                                                (IP thread, DHCP, MQTT, etc.)
```

## File Structure

| File | Purpose |
|------|---------|
| `AZURE_RTOS/App/app_azure_rtos.c` | Main entry point (`tx_application_define`) |
| `AZURE_RTOS/App/app_azure_rtos_config.h` | Memory pool size configurations |
| `Core/Src/app_threadx.c` | ThreadX application threads |
| `NetXDuo/App/app_netxduo.c` | NetXDuo networking initialization |

## Memory Pools

Each component has its own byte pool for memory allocation:

| Pool | Size | Purpose |
|------|------|---------|
| `tx_app_byte_pool` | 2 KB | ThreadX application threads |
| `nx_byte_pool` | 160 KB | NetXDuo networking stack |

Configured in `app_azure_rtos_config.h`:
```c
#define TX_APP_MEM_POOL_SIZE    (2 * 1024)
#define NX_APP_MEM_POOL_SIZE    163840
```

## Relationship Between ThreadX and NetXDuo

```
┌─────────────────────────────────────────────────────────────┐
│                    ThreadX Scheduler                         │
│  (manages all threads - both application and NetXDuo)        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────────┐    ┌─────────────────────────────┐    │
│  │  Your Threads    │    │    NetXDuo Internal         │    │
│  │  (app_threadx.c) │    │    Threads                  │    │
│  │                  │    │                             │    │
│  │  - Temperature   │    │  - IP Thread                │    │
│  │  - Motion        │    │  - DHCP Client Thread       │    │
│  │  - Custom tasks  │    │  - MQTT Client Thread       │    │
│  │                  │    │  - etc.                     │    │
│  └──────────────────┘    └─────────────────────────────┘    │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Key Points

1. **Single Kernel**: Both your application threads and NetXDuo threads run under the same ThreadX kernel scheduler.

2. **Separate Initialization**: Each component has its own init function:
   - `App_ThreadX_Init()` - called first
   - `MX_NetXDuo_Init()` - called second

3. **Separate Memory**: Each component uses its own memory pool, preventing one from exhausting the other's memory.

4. **Thread Priority**: All threads (application + networking) compete for CPU time based on their priorities. NetXDuo threads typically use priorities in the range of 1-15.

5. **Dependencies**: NetXDuo depends on ThreadX. You cannot use NetXDuo without ThreadX, but you can use ThreadX without NetXDuo.

## Adding Your Own Threads

Add your threads in `app_threadx.c` within `App_ThreadX_Init()`:

```c
UINT App_ThreadX_Init(VOID *memory_ptr)
{
    // Create your threads here
    tx_thread_create(&my_thread, "My Thread",
                     my_thread_entry, 0,
                     my_stack, STACK_SIZE,
                     PRIORITY, PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    return TX_SUCCESS;
}
```

## References

- [Eclipse ThreadX Documentation](https://github.com/eclipse-threadx/threadx)
- [Eclipse NetXDuo Documentation](https://github.com/eclipse-threadx/netxduo)
- [STM32 Azure RTOS Documentation](https://wiki.stmicroelectronics.cn/stm32mcu/wiki/Introduction_to_THREADX)
