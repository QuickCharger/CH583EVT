#include "vector.h"
#include <stdio.h>
#include <stdlib.h>

void Vector_Init(Vector *vec, size_t initial_capacity) {
    vec->data = (int *)malloc(initial_capacity * sizeof(int));
    if (vec->data == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(1);
    }
    vec->size = 0;
    vec->capacity = initial_capacity;
}

void Vector_Destroy(Vector *vec) {
    free(vec->data);
    vec->data = NULL;
    vec->size = 0;
    vec->capacity = 0;
}

void Vector_Resize(Vector *vec) {
    if (vec->size >= vec->capacity) {
        vec->capacity *= 2; // 容量翻倍
        vec->data = (int *)realloc(vec->data, vec->capacity * sizeof(int));
        if (vec->data == NULL) {
            fprintf(stderr, "Memory reallocation failed\n");
            exit(1);
        }
    }
}

void Vector_PushBack(Vector *vec, int value) {
    Vector_Resize(vec); // 检查是否需要扩容
    vec->data[vec->size] = value;
    vec->size++;
}

void Vector_PopBack(Vector *vec) {
    if (vec->size > 0) {
        vec->size--;
    } else {
        fprintf(stderr, "Vector is empty\n");
    }
}

int Vector_At(Vector *vec, size_t index) {
    if (index < vec->size) {
        return vec->data[index];
    } else {
        fprintf(stderr, "Index out of bounds\n");
        exit(1);
    }
}

void Vector_Print(Vector *vec) {
    printf("Vector: ");
    for (size_t i = 0; i < vec->size; i++) {
        printf("%d ", vec->data[i]);
    }
    printf("\n");
}
