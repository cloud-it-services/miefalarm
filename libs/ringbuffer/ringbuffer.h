//Wolfram Gürlich 2020

/*define an RBUF instance as local or global variable and a pointer to it*/
/*variable xxx for BUF_PT will be of pointer type 'struct xxx_t*'  */

#define RBUF_DEFINE(BUF_PT,T,SIZE) \
   struct BUF_PT##_t{ \
     uint16_t head; \
     uint16_t tail; \
     T buf[SIZE]; \
     } BUF_PT##_instance = {.head=0 , .tail=0}; \
   struct BUF_PT##_t *BUF_PT=&BUF_PT##_instance



/*define a new RBUF type for dynamic creation, parameter passing and embedding in another structure */
#define RBUF_TYPE(TNAME,T,SIZE) \
   typedef struct { \
     uint16_t head; \
     uint16_t tail; \
     T buf[SIZE]; \
   } TNAME


/*allocate an RBUF on the heap*/
#define RBUF_CREATE(TNAME) \
((TNAME*)calloc(1,sizeof(TNAME)))

/*add element*/
#define RBUF_ADD(BUF_PT,ELEM) \
((BUF_PT)->head=((BUF_PT)->head+1)%(sizeof((BUF_PT)->buf)/sizeof((BUF_PT)->buf[0])), \
(BUF_PT)->buf[(BUF_PT)->head]=(ELEM))

/*fetch element*/
#define RBUF_FETCH(BUF_PT) \
((BUF_PT)->tail=((BUF_PT)->tail+1)%(sizeof((BUF_PT)->buf)/sizeof((BUF_PT)->buf[0])), \
(BUF_PT)->buf[(BUF_PT)->tail])

/*check if RBUF is empty*/
#define RBUF_IS_EMPTY(BUF_PT) \
((BUF_PT)->head==(BUF_PT)->tail)

/*check if RBUF is full*/
#define RBUF_IS_FULL(BUF_PT) \
((BUF_PT)->tail==((BUF_PT)->head+1)%(sizeof((BUF_PT)->buf)/sizeof((BUF_PT)->buf[0])))

/*clear RBUF*/
#define RBUF_CLEAR(BUF_PT) \
(BUF_PT)->tail=(BUF_PT)->head=0



