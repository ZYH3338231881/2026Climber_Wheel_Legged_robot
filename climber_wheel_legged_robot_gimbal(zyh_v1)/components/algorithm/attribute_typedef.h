#ifndef ATTRIBUTE_TYPEDEF_H
#define ATTRIBUTE_TYPEDEF_H

// 定义一些常用的属性宏，用于定义函数、变量、结构体等的属性。

#ifndef __packed__
/// @brief 取消结构体成员之间的对齐，使结构体紧凑排列。
/// @note 注意和ARMCC编译器中自带的`__packed`关键字使用方法不太一样。
#define __packed__ __attribute__((packed))
#endif

#ifndef __aligned
/// @brief 指定变量或结构体的对齐方式，以字节为单位。
#define __aligned(x) __attribute__((aligned(x)))
#endif

#ifndef __section
/// @brief 指定变量或函数存放在特定的段中。
#define __section(x) __attribute__((section(x)))
#endif

#ifndef __weak
/// @brief 标记为弱符号，如果有多个同名符号，优先使用强符号。
#define __weak __attribute__((weak))
#endif

#ifndef __noreturn
/// @brief 标记函数不返回。
#define __noreturn __attribute__((noreturn))
#endif

#ifndef __naked
/// @brief 指定函数不使用函数调用的方式，直接跳转到函数的入口地址。
#define __naked __attribute__((naked))
#endif

#ifndef __unused
/// @brief 标记为未使用的函数或变量，防止编译器产生警告。
#define __unused __attribute__((unused))
#endif

#ifndef __used
/// @brief 标记为使用的函数或变量。
#define __used __attribute__((used))
#endif

#ifndef __deprecated
/// @brief 标记为过时的函数或变量，在使用时会产生警告。
#define __deprecated __attribute__((deprecated))
#endif

#ifndef __always_inline
/// @brief 强制内联函数，即使-O0编译选项也会内联。
#define __always_inline __attribute__((always_inline))
#endif

#ifndef __format
/// @brief 格式化检查，用于检查printf、scanf等函数的参数格式是否正确。
#define __format __attribute__((format))
#endif

#endif  // ATTRIBUTE_TYPEDEF_H
/*------------------------------ End of File ------------------------------*/
