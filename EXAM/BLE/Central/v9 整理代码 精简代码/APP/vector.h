#ifndef VECTOR_H
#define VECTOR_H

#include <stddef.h> // 用于 size_t

typedef struct {
    int *data;      // 动态数组指针
    size_t size;    // 当前元素数量
    size_t capacity; // 当前容量
} Vector;

// 初始化 Vector
void Vector_Init(Vector *vec, size_t initial_capacity);

// 销毁 Vector
void Vector_Destroy(Vector *vec);

// 动态扩容
void Vector_Resize(Vector *vec);

// 添加元素到末尾
void Vector_PushBack(Vector *vec, int value);

// 删除末尾元素
void Vector_PopBack(Vector *vec);

// 访问元素
int Vector_At(Vector *vec, size_t index);

// 打印 Vector
void Vector_Print(Vector *vec);

#endif // VECTOR_H
