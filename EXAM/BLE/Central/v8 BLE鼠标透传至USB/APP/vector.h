#ifndef VECTOR_H
#define VECTOR_H

#include <stddef.h> // ���� size_t

typedef struct {
    int *data;      // ��̬����ָ��
    size_t size;    // ��ǰԪ������
    size_t capacity; // ��ǰ����
} Vector;

// ��ʼ�� Vector
void Vector_Init(Vector *vec, size_t initial_capacity);

// ���� Vector
void Vector_Destroy(Vector *vec);

// ��̬����
void Vector_Resize(Vector *vec);

// ���Ԫ�ص�ĩβ
void Vector_PushBack(Vector *vec, int value);

// ɾ��ĩβԪ��
void Vector_PopBack(Vector *vec);

// ����Ԫ��
int Vector_At(Vector *vec, size_t index);

// ��ӡ Vector
void Vector_Print(Vector *vec);

#endif // VECTOR_H
