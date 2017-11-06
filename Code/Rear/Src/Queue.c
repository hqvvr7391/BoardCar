#include "Queue.h"
#include <stdlib.h>


void init(Queue_TypeDef *q){
	
	int i;
	q->front = 0;
	q->rear = 0;
	
	for(i=0; i < q->size; i++)		q->queue[i] = 0;
} 

int is_empty(Queue_TypeDef *q){
	return q->front == q->rear;
}


int is_full(Queue_TypeDef *q){
	return (q->rear+1) % q->size == q->front;
}

element enqueue(Queue_TypeDef *q, element item){
	int i = q->rear;
	

	
	while((i + 1) % q->size != (q->front + 1) % q->size ) {
			i = (i + 1) % q->size;
			q->queue[i] = item;
		}
	
	if (is_full(q)){
		dequeue(q);

	}
	
	q->rear = (q->rear+1) % q->size;
	q->queue[q->rear] = item;
	
	return q->rear;
}

element dequeue(Queue_TypeDef *q){
	if (is_empty(q)){
		return 0 ;
	}
	else if(is_full(q)){
		q->front = (q->front+1) % q->size;
	}
	return q->front;
}

element peek(Queue_TypeDef *q){
    if (is_full(q)) {
    }
    return q->queue[(q->front+1) % q->size];
}

void define_array(Queue_TypeDef *q)
{
	q->size = q->size + 1;
	q->queue = (int *)malloc(q->size* sizeof(int));
	init(q);
}

