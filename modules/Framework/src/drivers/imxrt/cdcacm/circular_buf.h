#define __CIRCLUAR_BUF_H__
#ifdef __CIRCLUAR_BUF_H__

#ifdef __cplusplus
extern  "C" {                                                   /* See Note #1.                        */
#endif


struct circular_buffer {
    unsigned int size;
    unsigned int in;
    unsigned int out;
    unsigned char *buffer;
};

int init_circular_buf(struct circular_buffer *buf, unsigned int size);

void free_circular_buf(struct circular_buffer *buf);

void clear_circular_buf(struct circular_buffer *buf);

unsigned int get_data_count(struct circular_buffer *buf);

unsigned int get_free_count(struct circular_buffer *buf);

unsigned int write_circular_buf(struct circular_buffer *buf, unsigned char *data, unsigned int count);

unsigned int read_circular_buf(struct circular_buffer *buf, unsigned char *data, unsigned int count);

#ifdef __cplusplus
}                                                               /* End of 'extern'al C lang linkage.   */
#endif

#endif
