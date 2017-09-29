#include "Queue.h"


void init(Queue_TypeDef *q){
	q->front = 0;
	q->rear = 0;
} 

int is_empty(Queue_TypeDef *q){
	return q->front == q->rear;
}


int is_full(Queue_TypeDef *q){
	return (q->rear+1) % q->size == q->front;
}

void enqueue(Queue_TypeDef *q, element item){
	if (is_full(q)){
		
	}
	else {
		q->rear = (q->rear+1) % q->size;
		q->queue[q->rear] = item;
	}
}

element dequeue(Queue_TypeDef *q){
	if (is_empty(q)){
		
	}
	else {
		q->front = (q->front+1) % q->size;
	}
	return q->queue[q->front];
}

element peek(Queue_TypeDef *q){
    if (is_full(q)) {
    }
    return q->queue[(q->front+1) % q->size];
}

void define_array(Queue_TypeDef *q)
{
	q->queue = (int *)malloc(q->size * sizeof(int));
}

