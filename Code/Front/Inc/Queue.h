#ifndef _Queue
#define _Queue

#include <stdio.h>
#include <stdlib.h>

typedef int	element;

typedef struct 
{
	int front;
	int rear;
	int* queue;
	int size;
	
}Queue_TypeDef;

void init(Queue_TypeDef *q);
int is_empty(Queue_TypeDef *q);
int is_full(Queue_TypeDef *q);
element enqueue(Queue_TypeDef *q, element item);
element dequeue(Queue_TypeDef *q);
void define_array(Queue_TypeDef *q);

#endif
