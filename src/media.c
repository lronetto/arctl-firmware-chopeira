/*
 * media.c
 *
 *  Created on: 10 de jul de 2016
 *      Author: Leandro
 */

#include "media.h"

void media_Init(Media_T *m,int n){
	m->n=n;
	m->cnt=0;
	memset(m->vals,0,sizeof(m->vals));
}
void media_process(Media_T *m,float input){
	float media=0;
	int i;
	for(i=0;i<m->n-1;i++){
		m->vals[m->n-i-1]=m->vals[m->n-i-2];
		media+=m->vals[m->n-i-1];
	}
	m->vals[0]=input;
	media+=input;
	media/=m->n;
	if(m->cnt < m->n){
		m->cnt++;
		m->media=input;
	}
	else m->media=media;
}
