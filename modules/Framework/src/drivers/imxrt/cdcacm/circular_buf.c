/****************************************************************************
** Copyright (C) 2015-2016 Jin Jing <jingj@timestech.org>
** Version 1.1.0
** 提供标准C实现的循环缓冲区接口，详情参考Linux kernel kfifo.
** 
**
**
*****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "circular_buf.h"

#if 0
#define min(x,y) ({ \
	unsigned int _x = (x);	\
	unsigned int _y = (y);	\
	(void) (&_x == &_y);	\
	_x < _y ? _x : _y; })
#else
#define min(x,y)    ((x) < (y) ? (x) : (y))
#endif

int init_circular_buf(struct circular_buffer *buf, unsigned int size)
{
	buf->size = size;
	buf->in = buf->out = 0;

	buf->buffer = (unsigned char *)malloc(buf->size);
	if(buf->buffer == NULL)
	{
		printf("circular buffer malloc failed!\n");
		return -1;
	}

	memset(buf->buffer, 0, buf->size);
	return 0;
}

void free_circular_buf(struct circular_buffer *buf)
{
	if(buf != NULL)
	{
        buf->size = 0;
        buf->in = buf->out = 0;
		
		if(buf->buffer != NULL)
		{
       		free(buf->buffer);
			buf->buffer = NULL;
		}
	}
}

void clear_circular_buf(struct circular_buffer *buf)
{	
	buf->in=buf->out=0;
}

unsigned int get_data_count(struct circular_buffer *buf)
{
    return (buf->in - buf->out);
}

unsigned int get_free_count(struct circular_buffer *buf)
{
    return (buf->size - (buf->in - buf->out));
}


unsigned int read_circular_buf(struct circular_buffer *buf, unsigned char *pdata, unsigned int count)
{
	unsigned int l;

    count = min(count, buf->in - buf->out);

    l = min(count, buf->size - (buf->out & (buf->size - 1)));
    
    memcpy(pdata, buf->buffer + (buf->out & (buf->size - 1)), l);
    
    memcpy(pdata + l, buf->buffer, count - l);

    buf->out += count;
    
	return count;
}

unsigned int write_circular_buf(struct circular_buffer *buf, unsigned char *pdata, unsigned int count)
{
	unsigned int l;

    count = min(count,buf->size - buf->in + buf->out);

    l = min(count, buf->size - (buf->in  & (buf->size - 1)));

    memcpy(buf->buffer + (buf->in & (buf->size - 1)), pdata, l);

    memcpy(buf->buffer, pdata + l, count - l);

    buf->in += count;

	return count;
}

