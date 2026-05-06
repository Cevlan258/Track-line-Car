/**
  * 中文说明：本段为工程生成代码说明。
  */

/* 包含文件 */
#include <errno.h>
#include <stdint.h>
#include <stddef.h>

/**
  * 中文说明：本段为工程生成代码说明。
  */
static uint8_t *__sbrk_heap_end = NULL;

/**
  * 中文说明：本段为工程生成代码说明。
  */
void *_sbrk(ptrdiff_t incr)
{
  extern uint8_t _end; /* 链接脚本中定义的符号 */
  extern uint8_t _estack; /* 链接脚本中定义的符号 */
  extern uint32_t _Min_Stack_Size; /* 链接脚本中定义的符号 */
  const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
  const uint8_t *max_heap = (uint8_t *)stack_limit;
  uint8_t *prev_heap_end;

  /* 首次调用时初始化堆结束位置 */
  if (NULL == __sbrk_heap_end)
  {
    __sbrk_heap_end = &_end;
  }

  /* 防止堆增长到保留的 MSP 栈区域 */
  if (__sbrk_heap_end + incr > max_heap)
  {
    errno = ENOMEM;
    return (void *)-1;
  }

  prev_heap_end = __sbrk_heap_end;
  __sbrk_heap_end += incr;

  return (void *)prev_heap_end;
}

#if defined(__PICOLIBC__)
  // Picolibc 期望系统调用名称不带前导下划线。
  // 这里创建强别名，让 sbrk() 调用解析到 _sbrk() 实现。
  __strong_reference(_sbrk, sbrk);
#endif
