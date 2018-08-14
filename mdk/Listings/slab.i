#line 1 "..\\rt_thread\\kernel\\slab.c"

























 




































 

#line 1 "..\\rt_thread\\include\\rthw.h"

























 




#line 1 "..\\rt_thread\\include\\rtthread.h"




























 




#line 1 "..\\app\\rtconfig.h"
 



 


 


 


 


 
 


 





 


 





 
 


 


 


 


 


 
 


 


 
 

 


 
 



 



 
 





 

 



 
 

 




 
 
 

 
 


 


 

 

 

 







 

 

 


 
 








 

 




 
 



 


 


 


 


 


 


 


 


 
 

 
 

 





 





 





 


 


 


 


 


 


 


 



 
 
 

 

 

 

 

 

 
 
 

 
 
 
 

 
 
 










 

#line 35 "..\\rt_thread\\include\\rtthread.h"
#line 1 "..\\rt_thread\\include\\rtdebug.h"


















 




#line 25 "..\\rt_thread\\include\\rtdebug.h"

 


 








































 




#line 82 "..\\rt_thread\\include\\rtdebug.h"







 
#line 104 "..\\rt_thread\\include\\rtdebug.h"




 
#line 128 "..\\rt_thread\\include\\rtdebug.h"

#line 137 "..\\rt_thread\\include\\rtdebug.h"

#line 36 "..\\rt_thread\\include\\rtthread.h"
#line 1 "..\\rt_thread\\include\\rtdef.h"
































 




 
#line 40 "..\\rt_thread\\include\\rtdef.h"







 

 

 




 



 
typedef signed   char                   rt_int8_t;       
typedef signed   short                  rt_int16_t;      
typedef signed   long                   rt_int32_t;      
typedef unsigned char                   rt_uint8_t;      
typedef unsigned short                  rt_uint16_t;     
typedef unsigned long                   rt_uint32_t;     
typedef int                             rt_bool_t;       

 
typedef long                            rt_base_t;       
typedef unsigned long                   rt_ubase_t;      

typedef rt_base_t                       rt_err_t;        
typedef rt_uint32_t                     rt_time_t;       
typedef rt_uint32_t                     rt_tick_t;       
typedef rt_base_t                       rt_flag_t;       
typedef rt_ubase_t                      rt_size_t;       
typedef rt_ubase_t                      rt_dev_t;        
typedef rt_base_t                       rt_off_t;        

 



 

 





 
#line 1 "D:\\EngineeringSoftware\\Keil_core\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"
 
 
 





 










#line 27 "D:\\EngineeringSoftware\\Keil_core\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"








 

 
 
#line 57 "D:\\EngineeringSoftware\\Keil_core\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"
    typedef struct __va_list { void *__ap; } va_list;

   






 


   










 


   















 




   

 


   




 



   





 







#line 138 "D:\\EngineeringSoftware\\Keil_core\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"



#line 147 "D:\\EngineeringSoftware\\Keil_core\\ARM\\ARMCC\\Bin\\..\\include\\stdarg.h"

 

#line 96 "..\\rt_thread\\include\\rtdef.h"
#line 102 "..\\rt_thread\\include\\rtdef.h"
     






#line 173 "..\\rt_thread\\include\\rtdef.h"

 

typedef int (*init_fn_t)(void);
#line 198 "..\\rt_thread\\include\\rtdef.h"

 

 
 

 

 

 

 



 










 


 




 














 

 

 
#line 262 "..\\rt_thread\\include\\rtdef.h"

 







 








 







 


struct rt_list_node
{
    struct rt_list_node *next;                           
    struct rt_list_node *prev;                           
};
typedef struct rt_list_node rt_list_t;                   



 

 



 




 
struct rt_object
{
    char       name[8];                        
    rt_uint8_t type;                                     
    rt_uint8_t flag;                                     




    rt_list_t  list;                                     
};
typedef struct rt_object *rt_object_t;                   

















 
enum rt_object_class_type
{
    RT_Object_Class_Thread = 0,                          

    RT_Object_Class_Semaphore,                           


    RT_Object_Class_Mutex,                               


    RT_Object_Class_Event,                               


    RT_Object_Class_MailBox,                             


    RT_Object_Class_MessageQueue,                        





    RT_Object_Class_MemPool,                             


    RT_Object_Class_Device,                              

    RT_Object_Class_Timer,                               



    RT_Object_Class_Unknown,                             
    RT_Object_Class_Static = 0x80                        
};



 
struct rt_object_information
{
    enum rt_object_class_type type;                      
    rt_list_t                 object_list;               
    rt_size_t                 object_size;               
};



 
#line 396 "..\\rt_thread\\include\\rtdef.h"

 



 

 



 

















 






 
struct rt_timer
{
    struct rt_object parent;                             

    rt_list_t        row[1];

    void (*timeout_func)(void *parameter);               
    void            *parameter;                          

    rt_tick_t        init_tick;                          
    rt_tick_t        timeout_tick;                       
};
typedef struct rt_timer *rt_timer_t;

 



 

 



 



 
#line 468 "..\\rt_thread\\include\\rtdef.h"



 







 
struct rt_thread
{
     
    char        name[8];                       
    rt_uint8_t  type;                                    
    rt_uint8_t  flags;                                   





    rt_list_t   list;                                    
    rt_list_t   tlist;                                   

     
    void       *sp;                                      
    void       *entry;                                   
    void       *parameter;                               
    void       *stack_addr;                              
    rt_uint32_t stack_size;                              

     
    rt_err_t    error;                                   

    rt_uint8_t  stat;                                    

     
    rt_uint8_t  current_priority;                        
    rt_uint8_t  init_priority;                           




    rt_uint32_t number_mask;


     
    rt_uint32_t event_set;
    rt_uint8_t  event_info;


    rt_ubase_t  init_tick;                               
    rt_ubase_t  remaining_tick;                          

    struct rt_timer thread_timer;                        

    void (*cleanup)(struct rt_thread *tid);              

    rt_uint32_t user_data;                               
};
typedef struct rt_thread *rt_thread_t;

 



 

 



 











 
struct rt_ipc_object
{
    struct rt_object parent;                             

    rt_list_t        suspend_thread;                     
};




 
struct rt_semaphore
{
    struct rt_ipc_object parent;                         

    rt_uint16_t          value;                          
};
typedef struct rt_semaphore *rt_sem_t;





 
struct rt_mutex
{
    struct rt_ipc_object parent;                         

    rt_uint16_t          value;                          

    rt_uint8_t           original_priority;              
    rt_uint8_t           hold;                           

    struct rt_thread    *owner;                          
};
typedef struct rt_mutex *rt_mutex_t;





 






 
struct rt_event
{
    struct rt_ipc_object parent;                         

    rt_uint32_t          set;                            
};
typedef struct rt_event *rt_event_t;





 
struct rt_mailbox
{
    struct rt_ipc_object parent;                         

    rt_uint32_t         *msg_pool;                       

    rt_uint16_t          size;                           

    rt_uint16_t          entry;                          
    rt_uint16_t          in_offset;                      
    rt_uint16_t          out_offset;                     

    rt_list_t            suspend_sender_thread;          
};
typedef struct rt_mailbox *rt_mailbox_t;





 
struct rt_messagequeue
{
    struct rt_ipc_object parent;                         

    void                *msg_pool;                       

    rt_uint16_t          msg_size;                       
    rt_uint16_t          max_msgs;                       

    rt_uint16_t          entry;                          

    void                *msg_queue_head;                 
    void                *msg_queue_tail;                 
    void                *msg_queue_free;                 
};
typedef struct rt_messagequeue *rt_mq_t;


 



 

 




 

#line 706 "..\\rt_thread\\include\\rtdef.h"




 
struct rt_mempool
{
    struct rt_object parent;                             

    void            *start_address;                      
    rt_size_t        size;                               

    rt_size_t        block_size;                         
    rt_uint8_t      *block_list;                         

    rt_size_t        block_total_count;                  
    rt_size_t        block_free_count;                   

    rt_list_t        suspend_thread;                     
    rt_size_t        suspend_thread_count;               
};
typedef struct rt_mempool *rt_mp_t;


 




 

 



 
enum rt_device_class_type
{
    RT_Device_Class_Char = 0,                            
    RT_Device_Class_Block,                               
    RT_Device_Class_NetIf,                               
    RT_Device_Class_MTD,                                 
    RT_Device_Class_CAN,                                 
    RT_Device_Class_RTC,                                 
    RT_Device_Class_Sound,                               
    RT_Device_Class_Graphic,                             
    RT_Device_Class_I2CBUS,                              
    RT_Device_Class_USBDevice,                           
    RT_Device_Class_USBHost,                             
    RT_Device_Class_SPIBUS,                              
    RT_Device_Class_SPIDevice,                           
    RT_Device_Class_SDIO,                                
    RT_Device_Class_PM,                                  
    RT_Device_Class_Pipe,                                
    RT_Device_Class_Portal,                              
    RT_Device_Class_Timer,                               
	RT_Device_Class_Miscellaneous,                       
	RT_Device_Class_Unknown                              
};



 






























 





 
#line 817 "..\\rt_thread\\include\\rtdef.h"

typedef struct rt_device *rt_device_t;


 
struct rt_device
{
    struct rt_object          parent;                    

    enum rt_device_class_type type;                      
    rt_uint16_t               flag;                      
    rt_uint16_t               open_flag;                 

    rt_uint8_t                ref_count;                 
    rt_uint8_t                device_id;                 

     
    rt_err_t (*rx_indicate)(rt_device_t dev, rt_size_t size);
    rt_err_t (*tx_complete)(rt_device_t dev, void *buffer);

     
    rt_err_t  (*init)   (rt_device_t dev);
    rt_err_t  (*open)   (rt_device_t dev, rt_uint16_t oflag);
    rt_err_t  (*close)  (rt_device_t dev);
    rt_size_t (*read)   (rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size);
    rt_size_t (*write)  (rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size);
    rt_err_t  (*control)(rt_device_t dev, rt_uint8_t cmd, void *args);

    void                     *user_data;                 
};



 
struct rt_device_blk_geometry
{
    rt_uint32_t sector_count;                            
    rt_uint32_t bytes_per_sector;                        
    rt_uint32_t block_size;                              
};



 
struct rt_device_blk_sectors
{
    rt_uint32_t sector_begin;                            
    rt_uint32_t sector_end;                              
};



 





 
#line 882 "..\\rt_thread\\include\\rtdef.h"

 
enum
{
    RTGRAPHIC_PIXEL_FORMAT_MONO = 0,
    RTGRAPHIC_PIXEL_FORMAT_GRAY4,
    RTGRAPHIC_PIXEL_FORMAT_GRAY16,
    RTGRAPHIC_PIXEL_FORMAT_RGB332,
    RTGRAPHIC_PIXEL_FORMAT_RGB444,
    RTGRAPHIC_PIXEL_FORMAT_RGB565,
    RTGRAPHIC_PIXEL_FORMAT_RGB565P,
    RTGRAPHIC_PIXEL_FORMAT_BGR565 = RTGRAPHIC_PIXEL_FORMAT_RGB565P,
    RTGRAPHIC_PIXEL_FORMAT_RGB666,
    RTGRAPHIC_PIXEL_FORMAT_RGB888,
    RTGRAPHIC_PIXEL_FORMAT_ARGB888,
    RTGRAPHIC_PIXEL_FORMAT_ABGR888,
};



 




 
struct rt_device_graphic_info
{
    rt_uint8_t  pixel_format;                            
    rt_uint8_t  bits_per_pixel;                          
    rt_uint16_t reserved;                                

    rt_uint16_t width;                                   
    rt_uint16_t height;                                  

    rt_uint8_t *framebuffer;                             
};



 
struct rt_device_rect_info
{
    rt_uint16_t x;                                       
    rt_uint16_t y;                                       
    rt_uint16_t width;                                   
    rt_uint16_t height;                                  
};



 
struct rt_device_graphic_ops
{
    void (*set_pixel) (const char *pixel, int x, int y);
    void (*get_pixel) (char *pixel, int x, int y);

    void (*draw_hline)(const char *pixel, int x1, int x2, int y);
    void (*draw_vline)(const char *pixel, int x, int y1, int y2);

    void (*blit_line) (const char *pixel, int x, int y, rt_size_t size);
};


 


#line 999 "..\\rt_thread\\include\\rtdef.h"





#line 37 "..\\rt_thread\\include\\rtthread.h"
#line 1 "..\\rt_thread\\include\\rtservice.h"

























 










 

 



 






 
static __inline void rt_list_init(rt_list_t *l)
{
    l->next = l->prev = l;
}






 
static __inline void rt_list_insert_after(rt_list_t *l, rt_list_t *n)
{
    l->next->prev = n;
    n->next = l->next;

    l->next = n;
    n->prev = l;
}






 
static __inline void rt_list_insert_before(rt_list_t *l, rt_list_t *n)
{
    l->prev->next = n;
    n->prev = l->prev;

    l->prev = n;
    n->next = l;
}




 
static __inline void rt_list_remove(rt_list_t *n)
{
    n->next->prev = n->prev;
    n->prev->next = n->next;

    n->next = n->prev = n;
}




 
static __inline int rt_list_isempty(const rt_list_t *l)
{
    return l->next == l;
}






 








 












 


 





#line 38 "..\\rt_thread\\include\\rtthread.h"
#line 1 "..\\rt_thread\\include\\rtm.h"


















 




#line 25 "..\\rt_thread\\include\\rtm.h"
#line 1 "..\\rt_thread\\include\\rtthread.h"




























 

#line 26 "..\\rt_thread\\include\\rtm.h"

#line 56 "..\\rt_thread\\include\\rtm.h"

#line 39 "..\\rt_thread\\include\\rtthread.h"







 

 



 
void rt_system_object_init(void);
struct rt_object_information *
rt_object_get_information(enum rt_object_class_type type);
void rt_object_init(struct rt_object         *object,
                    enum rt_object_class_type type,
                    const char               *name);
void rt_object_detach(rt_object_t object);
rt_object_t rt_object_allocate(enum rt_object_class_type type,
                               const char               *name);
void rt_object_delete(rt_object_t object);
rt_bool_t rt_object_is_systemobject(rt_object_t object);
rt_object_t rt_object_find(const char *name, rt_uint8_t type);


void rt_object_attach_sethook(void (*hook)(struct rt_object *object));
void rt_object_detach_sethook(void (*hook)(struct rt_object *object));
void rt_object_trytake_sethook(void (*hook)(struct rt_object *object));
void rt_object_take_sethook(void (*hook)(struct rt_object *object));
void rt_object_put_sethook(void (*hook)(struct rt_object *object));


 



 

 



 
void rt_system_tick_init(void);
rt_tick_t rt_tick_get(void);
void rt_tick_set(rt_tick_t tick);
void rt_tick_increase(void);
rt_tick_t rt_tick_from_millisecond(rt_uint32_t ms);

void rt_system_timer_init(void);
void rt_system_timer_thread_init(void);

void rt_timer_init(rt_timer_t  timer,
                   const char *name,
                   void (*timeout)(void *parameter),
                   void       *parameter,
                   rt_tick_t   time,
                   rt_uint8_t  flag);
rt_err_t rt_timer_detach(rt_timer_t timer);
rt_timer_t rt_timer_create(const char *name,
                           void (*timeout)(void *parameter),
                           void       *parameter,
                           rt_tick_t   time,
                           rt_uint8_t  flag);
rt_err_t rt_timer_delete(rt_timer_t timer);
rt_err_t rt_timer_start(rt_timer_t timer);
rt_err_t rt_timer_stop(rt_timer_t timer);
rt_err_t rt_timer_control(rt_timer_t timer, rt_uint8_t cmd, void *arg);

rt_tick_t rt_timer_next_timeout_tick(void);
void rt_timer_check(void);


void rt_timer_timeout_sethook(void (*hook)(struct rt_timer *timer));


 



 

 



 
rt_err_t rt_thread_init(struct rt_thread *thread,
                        const char       *name,
                        void (*entry)(void *parameter),
                        void             *parameter,
                        void             *stack_start,
                        rt_uint32_t       stack_size,
                        rt_uint8_t        priority,
                        rt_uint32_t       tick);
rt_err_t rt_thread_detach(rt_thread_t thread);
rt_thread_t rt_thread_create(const char *name,
                             void (*entry)(void *parameter),
                             void       *parameter,
                             rt_uint32_t stack_size,
                             rt_uint8_t  priority,
                             rt_uint32_t tick);
rt_thread_t rt_thread_self(void);
rt_thread_t rt_thread_find(char *name);
rt_err_t rt_thread_startup(rt_thread_t thread);
rt_err_t rt_thread_delete(rt_thread_t thread);

rt_err_t rt_thread_yield(void);
rt_err_t rt_thread_delay(rt_tick_t tick);
rt_err_t rt_thread_control(rt_thread_t thread, rt_uint8_t cmd, void *arg);
rt_err_t rt_thread_suspend(rt_thread_t thread);
rt_err_t rt_thread_resume(rt_thread_t thread);
void rt_thread_timeout(void *parameter);



 
void rt_thread_idle_init(void);

void rt_thread_idle_sethook(void (*hook)(void));

void rt_thread_idle_excute(void);



 
void rt_system_scheduler_init(void);
void rt_system_scheduler_start(void);

void rt_schedule(void);
void rt_schedule_insert_thread(struct rt_thread *thread);
void rt_schedule_remove_thread(struct rt_thread *thread);

void rt_enter_critical(void);
void rt_exit_critical(void);
rt_uint16_t rt_critical_level(void);


void rt_scheduler_sethook(void (*hook)(rt_thread_t from, rt_thread_t to));


 



 

 



 



 
rt_err_t rt_mp_init(struct rt_mempool *mp,
                    const char        *name,
                    void              *start,
                    rt_size_t          size,
                    rt_size_t          block_size);
rt_err_t rt_mp_detach(struct rt_mempool *mp);
rt_mp_t rt_mp_create(const char *name,
                     rt_size_t   block_count,
                     rt_size_t   block_size);
rt_err_t rt_mp_delete(rt_mp_t mp);

void *rt_mp_alloc(rt_mp_t mp, rt_int32_t time);
void rt_mp_free(void *block);


void rt_mp_alloc_sethook(void (*hook)(struct rt_mempool *mp, void *block));
void rt_mp_free_sethook(void (*hook)(struct rt_mempool *mp, void *block));







 
void rt_system_heap_init(void *begin_addr, void *end_addr);

void *rt_malloc(rt_size_t nbytes);
void rt_free(void *ptr);
void *rt_realloc(void *ptr, rt_size_t nbytes);
void *rt_calloc(rt_size_t count, rt_size_t size);
void *rt_malloc_align(rt_size_t size, rt_size_t align);
void rt_free_align(void *ptr);

void rt_memory_info(rt_uint32_t *total,
                    rt_uint32_t *used,
                    rt_uint32_t *max_used);


void *rt_page_alloc(rt_size_t npages);
void rt_page_free(void *addr, rt_size_t npages);



void rt_malloc_sethook(void (*hook)(void *ptr, rt_uint32_t size));
void rt_free_sethook(void (*hook)(void *ptr));




#line 261 "..\\rt_thread\\include\\rtthread.h"

 



 

 




 
rt_err_t rt_sem_init(rt_sem_t    sem,
                     const char *name,
                     rt_uint32_t value,
                     rt_uint8_t  flag);
rt_err_t rt_sem_detach(rt_sem_t sem);
rt_sem_t rt_sem_create(const char *name, rt_uint32_t value, rt_uint8_t flag);
rt_err_t rt_sem_delete(rt_sem_t sem);

rt_err_t rt_sem_take(rt_sem_t sem, rt_int32_t time);
rt_err_t rt_sem_trytake(rt_sem_t sem);
rt_err_t rt_sem_release(rt_sem_t sem);
rt_err_t rt_sem_control(rt_sem_t sem, rt_uint8_t cmd, void *arg);





 
rt_err_t rt_mutex_init(rt_mutex_t mutex, const char *name, rt_uint8_t flag);
rt_err_t rt_mutex_detach(rt_mutex_t mutex);
rt_mutex_t rt_mutex_create(const char *name, rt_uint8_t flag);
rt_err_t rt_mutex_delete(rt_mutex_t mutex);

rt_err_t rt_mutex_take(rt_mutex_t mutex, rt_int32_t time);
rt_err_t rt_mutex_release(rt_mutex_t mutex);
rt_err_t rt_mutex_control(rt_mutex_t mutex, rt_uint8_t cmd, void *arg);





 
rt_err_t rt_event_init(rt_event_t event, const char *name, rt_uint8_t flag);
rt_err_t rt_event_detach(rt_event_t event);
rt_event_t rt_event_create(const char *name, rt_uint8_t flag);
rt_err_t rt_event_delete(rt_event_t event);

rt_err_t rt_event_send(rt_event_t event, rt_uint32_t set);
rt_err_t rt_event_recv(rt_event_t   event,
                       rt_uint32_t  set,
                       rt_uint8_t   opt,
                       rt_int32_t   timeout,
                       rt_uint32_t *recved);
rt_err_t rt_event_control(rt_event_t event, rt_uint8_t cmd, void *arg);





 
rt_err_t rt_mb_init(rt_mailbox_t mb,
                    const char  *name,
                    void        *msgpool,
                    rt_size_t    size,
                    rt_uint8_t   flag);
rt_err_t rt_mb_detach(rt_mailbox_t mb);
rt_mailbox_t rt_mb_create(const char *name, rt_size_t size, rt_uint8_t flag);
rt_err_t rt_mb_delete(rt_mailbox_t mb);

rt_err_t rt_mb_send(rt_mailbox_t mb, rt_uint32_t value);
rt_err_t rt_mb_send_wait(rt_mailbox_t mb,
                         rt_uint32_t  value,
                         rt_int32_t   timeout);
rt_err_t rt_mb_recv(rt_mailbox_t mb, rt_uint32_t *value, rt_int32_t timeout);
rt_err_t rt_mb_control(rt_mailbox_t mb, rt_uint8_t cmd, void *arg);





 
rt_err_t rt_mq_init(rt_mq_t     mq,
                    const char *name,
                    void       *msgpool,
                    rt_size_t   msg_size,
                    rt_size_t   pool_size,
                    rt_uint8_t  flag);
rt_err_t rt_mq_detach(rt_mq_t mq);
rt_mq_t rt_mq_create(const char *name,
                     rt_size_t   msg_size,
                     rt_size_t   max_msgs,
                     rt_uint8_t  flag);
rt_err_t rt_mq_delete(rt_mq_t mq);

rt_err_t rt_mq_send(rt_mq_t mq, void *buffer, rt_size_t size);
rt_err_t rt_mq_urgent(rt_mq_t mq, void *buffer, rt_size_t size);
rt_err_t rt_mq_recv(rt_mq_t    mq,
                    void      *buffer,
                    rt_size_t  size,
                    rt_int32_t timeout);
rt_err_t rt_mq_control(rt_mq_t mq, rt_uint8_t cmd, void *arg);


 




 

 



 
rt_device_t rt_device_find(const char *name);

rt_err_t rt_device_register(rt_device_t dev,
                            const char *name,
                            rt_uint16_t flags);
rt_err_t rt_device_unregister(rt_device_t dev);
rt_err_t rt_device_init_all(void);

rt_err_t
rt_device_set_rx_indicate(rt_device_t dev,
                          rt_err_t (*rx_ind)(rt_device_t dev, rt_size_t size));
rt_err_t
rt_device_set_tx_complete(rt_device_t dev,
                          rt_err_t (*tx_done)(rt_device_t dev, void *buffer));

rt_err_t  rt_device_init (rt_device_t dev);
rt_err_t  rt_device_open (rt_device_t dev, rt_uint16_t oflag);
rt_err_t  rt_device_close(rt_device_t dev);
rt_size_t rt_device_read (rt_device_t dev,
                          rt_off_t    pos,
                          void       *buffer,
                          rt_size_t   size);
rt_size_t rt_device_write(rt_device_t dev,
                          rt_off_t    pos,
                          const void *buffer,
                          rt_size_t   size);
rt_err_t  rt_device_control(rt_device_t dev, rt_uint8_t cmd, void *arg);

 


#line 447 "..\\rt_thread\\include\\rtthread.h"



 



 
void rt_interrupt_enter(void);
void rt_interrupt_leave(void);



 
rt_uint8_t rt_interrupt_get_nest(void);


void rt_components_init(void);
void rt_components_board_init(void);




 

 



 



void rt_kprintf(const char *fmt, ...);

rt_int32_t rt_vsprintf(char *dest, const char *format, va_list arg_ptr);
rt_int32_t rt_vsnprintf(char *buf, rt_size_t size, const char *fmt, va_list args);
rt_int32_t rt_sprintf(char *buf ,const char *format, ...);
rt_int32_t rt_snprintf(char *buf, rt_size_t size, const char *format, ...);


rt_device_t rt_console_set_device(const char *name);
rt_device_t rt_console_get_device(void);


rt_err_t rt_get_errno(void);
void rt_set_errno(rt_err_t no);
int *_rt_errno(void);






void *rt_memset(void *src, int c, rt_ubase_t n);
void *rt_memcpy(void *dest, const void *src, rt_ubase_t n);

rt_int32_t rt_strncmp(const char *cs, const char *ct, rt_ubase_t count);
rt_int32_t rt_strcmp (const char *cs, const char *ct);
rt_size_t rt_strlen (const char *src);
char *rt_strdup(const char *s);

char *rt_strstr(const char *str1, const char *str2);
rt_int32_t rt_sscanf(const char *buf, const char *fmt, ...);
char *rt_strncpy(char *dest, const char *src, rt_ubase_t n);
void *rt_memmove(void *dest, const void *src, rt_ubase_t n);
rt_int32_t rt_memcmp(const void *cs, const void *ct, rt_ubase_t count);
rt_uint32_t rt_strcasecmp(const char *a, const char *b);

void rt_show_version(void);


extern void (*rt_assert_hook)(const char* ex, const char* func, rt_size_t line);
void rt_assert_set_hook(void (*hook)(const char* ex, const char* func, rt_size_t line));

void rt_assert_handler(const char* ex, const char* func, rt_size_t line);


 





#line 32 "..\\rt_thread\\include\\rthw.h"







 
void rt_hw_cpu_icache_enable(void);
void rt_hw_cpu_icache_disable(void);
rt_base_t rt_hw_cpu_icache_status(void);
void rt_hw_cpu_dcache_enable(void);
void rt_hw_cpu_dcache_disable(void);
rt_base_t rt_hw_cpu_dcache_status(void);
void rt_hw_cpu_reset(void);
void rt_hw_cpu_shutdown(void);

rt_uint8_t *rt_hw_stack_init(void       *entry,
                             void       *parameter,
                             rt_uint8_t *stack_addr,
                             void       *exit);



 
typedef void (*rt_isr_handler_t)(int vector, void *param);

struct rt_irq_desc
{
    rt_isr_handler_t handler;
    void            *param;


    char             name[8];
    rt_uint32_t      counter;

};



 
void rt_hw_interrupt_init(void);
void rt_hw_interrupt_mask(int vector);
void rt_hw_interrupt_umask(int vector);
rt_isr_handler_t rt_hw_interrupt_install(int              vector,
                                         rt_isr_handler_t handler,
                                         void            *param,
                                         char            *name);

rt_base_t rt_hw_interrupt_disable(void);
void rt_hw_interrupt_enable(rt_base_t level);



 
void rt_hw_context_switch(rt_uint32_t from, rt_uint32_t to);
void rt_hw_context_switch_to(rt_uint32_t to);
void rt_hw_context_switch_interrupt(rt_uint32_t from, rt_uint32_t to);

void rt_hw_console_output(const char *str);

void rt_hw_backtrace(rt_uint32_t *fp, rt_uint32_t thread_entry);
void rt_hw_show_memory(rt_uint32_t addr, rt_uint32_t size);



 
void rt_hw_exception_install(rt_err_t (*exception_handle)(void *context));





#line 66 "..\\rt_thread\\kernel\\slab.c"
#line 67 "..\\rt_thread\\kernel\\slab.c"




 

static rt_size_t used_mem, max_mem;



static void (*rt_malloc_hook)(void *ptr, rt_size_t size);
static void (*rt_free_hook)(void *ptr);



 

 






 
void rt_malloc_sethook(void (*hook)(void *ptr, rt_size_t size))
{
    rt_malloc_hook = hook;
}
;






 
void rt_free_sethook(void (*hook)(void *ptr))
{
    rt_free_hook = hook;
}
;

 























































 



 
typedef struct slab_chunk
{
    struct slab_chunk *c_next;
} slab_chunk;



 
typedef struct slab_zone
{
    rt_int32_t  z_magic;         
    rt_int32_t  z_nfree;         
    rt_int32_t  z_nmax;          

    struct slab_zone *z_next;    
    rt_uint8_t  *z_baseptr;      

    rt_int32_t  z_uindex;        
    rt_int32_t  z_chunksize;     

    rt_int32_t  z_zoneindex;     
    slab_chunk  *z_freechunk;    
} slab_zone;

#line 201 "..\\rt_thread\\kernel\\slab.c"

static slab_zone *zone_array[72];    
static slab_zone *zone_free;             

static int zone_free_cnt;
static int zone_size;
static int zone_limit;
static int zone_page_cnt;




 





 



struct memusage
{
    rt_uint32_t type:2 ;         
    rt_uint32_t size:30;         
};
static struct memusage *memusage = (0);



static rt_uint32_t heap_start, heap_end;

 
struct rt_page_head
{
    struct rt_page_head *next;       
    rt_size_t page;                  

     
    char dummy[4096 - (sizeof(struct rt_page_head*) + sizeof (rt_size_t))];
};
static struct rt_page_head *rt_page_list;
static struct rt_semaphore heap_sem;

void *rt_page_alloc(rt_size_t npages)
{
    struct rt_page_head *b, *n;
    struct rt_page_head **prev;

    if(npages == 0)
        return (0);

     
    rt_sem_take(&heap_sem, -1);
    for (prev = &rt_page_list; (b = *prev) != (0); prev = &(b->next))
    {
        if (b->page > npages)
        {
             
            n       = b + npages;
            n->next = b->next;
            n->page = b->page - npages;
            *prev   = n;
            break;
        }

        if (b->page == npages)
        {
             
            *prev = b->next;
            break;
        }
    }

     
    rt_sem_release(&heap_sem);

    return b;
}

void rt_page_free(void *addr, rt_size_t npages)
{
    struct rt_page_head *b, *n;
    struct rt_page_head **prev;

    if (!(addr != (0))) { rt_assert_handler("addr != RT_NULL", __FUNCTION__, 287); };
    if (!((rt_uint32_t)addr % 4096 == 0)) { rt_assert_handler("(rt_uint32_t)addr % RT_MM_PAGE_SIZE == 0", __FUNCTION__, 288); };
    if (!(npages != 0)) { rt_assert_handler("npages != 0", __FUNCTION__, 289); };

    n = (struct rt_page_head *)addr;

     
    rt_sem_take(&heap_sem, -1);

    for (prev = &rt_page_list; (b = *prev) != (0); prev = &(b->next))
    {
        if (!(b->page > 0)) { rt_assert_handler("b->page > 0", __FUNCTION__, 298); };
        if (!(b > n || b + b->page <= n)) { rt_assert_handler("b > n || b + b->page <= n", __FUNCTION__, 299); };

        if (b + b->page == n)
        {
            if (b + (b->page += npages) == b->next)
            {
                b->page += b->next->page;
                b->next  = b->next->next;
            }

            goto _return;
        }

        if (b == n + npages)
        {
            n->page = b->page + npages;
            n->next = b->next;
            *prev   = n;

            goto _return;
        }

        if (b > n + npages)
            break;
    }

    n->page = npages;
    n->next = b;
    *prev   = n;

_return:
     
    rt_sem_release(&heap_sem);
}



 
static void rt_page_init(void *addr, rt_size_t npages)
{
    if (!(addr != (0))) { rt_assert_handler("addr != RT_NULL", __FUNCTION__, 339); };
    if (!(npages != 0)) { rt_assert_handler("npages != 0", __FUNCTION__, 340); };

    rt_page_list = (0);
    rt_page_free(addr, npages);
}








 
void rt_system_heap_init(void *begin_addr, void *end_addr)
{
    rt_uint32_t limsize, npages;

    do { rt_base_t level; level = rt_hw_interrupt_disable(); if (rt_interrupt_get_nest() != 0) { rt_kprintf("Function[%s] shall not used in ISR\n", __FUNCTION__); if (!(0)) { rt_assert_handler("0", __FUNCTION__, 358); } } rt_hw_interrupt_enable(level); } while (0);

     
    heap_start = ((((rt_uint32_t)begin_addr) + (4096) - 1) & ~((4096) - 1));
    heap_end   = (((rt_uint32_t)end_addr) & ~((4096) - 1));

    if (heap_start >= heap_end)
    {
        rt_kprintf("rt_system_heap_init, wrong address[0x%x - 0x%x]\n",
                   (rt_uint32_t)begin_addr, (rt_uint32_t)end_addr);

        return;
    }

    limsize = heap_end - heap_start;
    npages  = limsize / 4096;

     
    rt_sem_init(&heap_sem, "heap", 1, 0x00);

    do { if (0) rt_kprintf ("heap[0x%x - 0x%x], size 0x%x, 0x%x pages\n", heap_start, heap_end, limsize, npages); } while (0);


     
    rt_page_init((void *)heap_start, npages);

     
    zone_size = (32 * 1024);
    while (zone_size < (128 * 1024) && (zone_size << 1) < (limsize/1024))
        zone_size <<= 1;

    zone_limit = zone_size / 4;
    if (zone_limit > (16 * 1024))
        zone_limit = (16 * 1024);

    zone_page_cnt = zone_size / 4096;

    do { if (0) rt_kprintf ("zone size 0x%x, zone page count 0x%x\n", zone_size, zone_page_cnt); } while (0);


     
    limsize  = npages * sizeof(struct memusage);
    limsize  = (((limsize) + (4096) - 1) & ~((4096) - 1));
    memusage = rt_page_alloc(limsize/4096);

    do { if (0) rt_kprintf ("memusage 0x%x, size 0x%x\n", (rt_uint32_t)memusage, limsize); } while (0);

}




 
static __inline int zoneindex(rt_uint32_t *bytes)
{
     
    rt_uint32_t n = (rt_uint32_t)*bytes;

    if (n < 128)
    {
        *bytes = n = (n + 7) & ~7;

         
        return(n / 8 - 1);
    }
    if (n < 256)
    {
        *bytes = n = (n + 15) & ~15;

        return(n / 16 + 7);
    }
    if (n < 8192)
    {
        if (n < 512)
        {
            *bytes = n = (n + 31) & ~31;

            return(n / 32 + 15);
        }
        if (n < 1024)
        {
            *bytes = n = (n + 63) & ~63;

            return(n / 64 + 23);
        }
        if (n < 2048)
        {
            *bytes = n = (n + 127) & ~127;

            return(n / 128 + 31);
        }
        if (n < 4096)
        {
            *bytes = n = (n + 255) & ~255;

            return(n / 256 + 39);
        }
        *bytes = n = (n + 511) & ~511;

        return(n / 512 + 47);
    }
    if (n < 16384)
    {
        *bytes = n = (n + 1023) & ~1023;

        return(n / 1024 + 55);
    }

    rt_kprintf("Unexpected byte count %d", n);

    return 0;
}



 

 











 
void *rt_malloc(rt_size_t size)
{
    slab_zone *z;
    rt_int32_t zi;
    slab_chunk *chunk;
    struct memusage *kup;

     
    if (size == 0)
        return (0);






    


 
    if (size >= zone_limit)
    {
        size = (((size) + (4096) - 1) & ~((4096) - 1));

        chunk = rt_page_alloc(size >> 12);
        if (chunk == (0))
            return (0);

         
        kup = (&memusage[((rt_uint32_t)(chunk) - heap_start) >> 12]);
        kup->type = 0x02;
        kup->size = size >> 12;

        do { if (0) rt_kprintf ("malloc a large memory 0x%x, page cnt %d, kup %d\n", size, size >> 12, ((rt_uint32_t)chunk - heap_start) >> 12); } while (0);





         
        rt_sem_take(&heap_sem, -1);


        used_mem += size;
        if (used_mem > max_mem)
            max_mem = used_mem;

        goto done;
    }

     
    rt_sem_take(&heap_sem, -1);

    






 
    zi = zoneindex(&size);
    if (!(zi < 72)) { rt_assert_handler("zi < NZONES", __FUNCTION__, 550); };

    do { if (0) rt_kprintf ("try to malloc 0x%x on zone: %d\n", size, zi); } while (0);

    if ((z = zone_array[zi]) != (0))
    {
        if (!(z->z_nfree > 0)) { rt_assert_handler("z->z_nfree > 0", __FUNCTION__, 556); };

         
        if (--z->z_nfree == 0)
        {
            zone_array[zi] = z->z_next;
            z->z_next = (0);
        }

        




 
        if (z->z_uindex + 1 != z->z_nmax)
        {
            z->z_uindex = z->z_uindex + 1;
            chunk = (slab_chunk *)(z->z_baseptr + z->z_uindex * size);
        }
        else
        {
             
            chunk = z->z_freechunk;

             
            z->z_freechunk = z->z_freechunk->c_next;
        }


        used_mem += z->z_chunksize;
        if (used_mem > max_mem)
            max_mem = used_mem;


        goto done;
    }

    






 
    {
        rt_int32_t off;

        if ((z = zone_free) != (0))
        {
             
            zone_free = z->z_next;
            -- zone_free_cnt;
        }
        else
        {
             
            rt_sem_release(&heap_sem);

             
            z = rt_page_alloc(zone_size / 4096);
            if (z == (0))
                goto fail;

             
            rt_sem_take(&heap_sem, -1);

            do { if (0) rt_kprintf ("alloc a new zone: 0x%x\n", (rt_uint32_t)z); } while (0);


             
            for (off = 0, kup = (&memusage[((rt_uint32_t)(z) - heap_start) >> 12]); off < zone_page_cnt; off ++)
            {
                kup->type = 0x01;
                kup->size = off;

                kup ++;
            }
        }

         
        rt_memset(z, 0, sizeof(slab_zone));

         
        off = sizeof(slab_zone);

        


 
        if ((size | (size - 1)) + 1 == (size << 1))
            off = (off + size - 1) & ~(size - 1);
        else
            off = (off + (8 - 1)) & ~(8 - 1);

        z->z_magic     = 0x51ab51ab;
        z->z_zoneindex = zi;
        z->z_nmax      = (zone_size - off) / size;
        z->z_nfree     = z->z_nmax - 1;
        z->z_baseptr   = (rt_uint8_t *)z + off;
        z->z_uindex    = 0;
        z->z_chunksize = size;

        chunk = (slab_chunk *)(z->z_baseptr + z->z_uindex * size);

         
        z->z_next = zone_array[zi];
        zone_array[zi] = z;


        used_mem += z->z_chunksize;
        if (used_mem > max_mem)
            max_mem = used_mem;

    }

done:
    rt_sem_release(&heap_sem);

    do { if ((rt_malloc_hook) != (0)) rt_malloc_hook ((char *)chunk, size); } while (0);

    return chunk;

fail:
    rt_sem_release(&heap_sem);

    return (0);
}
;








 
void *rt_realloc(void *ptr, rt_size_t size)
{
    void *nptr;
    slab_zone *z;
    struct memusage *kup;

    if (ptr == (0))
        return rt_malloc(size);
    if (size == 0)
    {
        rt_free(ptr);

        return (0);
    }






    


 
    kup = (&memusage[((rt_uint32_t)((rt_uint32_t)ptr & ~(4096 - 1)) - heap_start) >> 12]);
    if (kup->type == 0x02)
    {
        rt_size_t osize;

        osize = kup->size << 12;
        if ((nptr = rt_malloc(size)) == (0))
            return (0);
        rt_memcpy(nptr, ptr, size > osize ? osize : size);
        rt_free(ptr);

        return nptr;
    }
    else if (kup->type == 0x01)
    {
        z = (slab_zone *)(((rt_uint32_t)ptr & ~(4096 - 1)) -
                          kup->size * 4096);
        if (!(z->z_magic == 0x51ab51ab)) { rt_assert_handler("z->z_magic == ZALLOC_SLAB_MAGIC", __FUNCTION__, 736); };

        zoneindex(&size);
        if (z->z_chunksize == size)
            return(ptr);  

        



 
        if ((nptr = rt_malloc(size)) == (0))
            return (0);

        rt_memcpy(nptr, ptr, size > z->z_chunksize ? z->z_chunksize : size);
        rt_free(ptr);

        return nptr;
    }

    return (0);
}
;












 
void *rt_calloc(rt_size_t count, rt_size_t size)
{
    void *p;

     
    p = rt_malloc(count * size);

     
    if (p)
        rt_memset(p, 0, count * size);

    return p;
}
;






 
void rt_free(void *ptr)
{
    slab_zone *z;
    slab_chunk *chunk;
    struct memusage *kup;

     
    if (ptr == (0))
        return ;

    do { if ((rt_free_hook) != (0)) rt_free_hook (ptr); } while (0);

#line 813 "..\\rt_thread\\kernel\\slab.c"

     
#line 825 "..\\rt_thread\\kernel\\slab.c"

    kup = (&memusage[((rt_uint32_t)((rt_uint32_t)ptr & ~(4096 - 1)) - heap_start) >> 12]);
     
    if (kup->type == 0x02)
    {
        rt_uint32_t size;

         
        rt_sem_take(&heap_sem, -1);
         
        size = kup->size;
        kup->size = 0;


        used_mem -= size * 4096;

        rt_sem_release(&heap_sem);

        do { if (0) rt_kprintf ("free large memory block 0x%x, page count %d\n", (rt_uint32_t)ptr, size); } while (0);



         
        rt_page_free(ptr, size);

        return;
    }

     
    rt_sem_take(&heap_sem, -1);

     
    z = (slab_zone *)(((rt_uint32_t)ptr & ~(4096 - 1)) -
                      kup->size * 4096);
    if (!(z->z_magic == 0x51ab51ab)) { rt_assert_handler("z->z_magic == ZALLOC_SLAB_MAGIC", __FUNCTION__, 859); };

    chunk          = (slab_chunk *)ptr;
    chunk->c_next  = z->z_freechunk;
    z->z_freechunk = chunk;


    used_mem -= z->z_chunksize;


    


 
    if (z->z_nfree++ == 0)
    {
        z->z_next = zone_array[z->z_zoneindex];
        zone_array[z->z_zoneindex] = z;
    }

    




 
    if (z->z_nfree == z->z_nmax &&
        (z->z_next || zone_array[z->z_zoneindex] != z))
    {
        slab_zone **pz;

        do { if (0) rt_kprintf ("free zone 0x%x\n", (rt_uint32_t)z, z->z_zoneindex); } while (0);


         
        for (pz = &zone_array[z->z_zoneindex]; z != *pz; pz = &(*pz)->z_next)
            ;
        *pz = z->z_next;

         
        z->z_magic = -1;

         
        z->z_next = zone_free;
        zone_free = z;

        ++ zone_free_cnt;

         
        if (zone_free_cnt > 2)
        {
            register rt_base_t i;

            z         = zone_free;
            zone_free = z->z_next;
            -- zone_free_cnt;

             
            for (i = 0, kup = (&memusage[((rt_uint32_t)(z) - heap_start) >> 12]); i < zone_page_cnt; i ++)
            {
                kup->type = 0x00;
                kup->size = 0;
                kup ++;
            }

             
            rt_sem_release(&heap_sem);

             
            rt_page_free(z, zone_size / 4096);

            return;
        }
    }
     
    rt_sem_release(&heap_sem);
}
;


void rt_memory_info(rt_uint32_t *total,
                    rt_uint32_t *used,
                    rt_uint32_t *max_used)
{
    if (total != (0))
        *total = heap_end - heap_start;

    if (used  != (0))
        *used = used_mem;

    if (max_used != (0))
        *max_used = max_mem;
}

#line 965 "..\\rt_thread\\kernel\\slab.c"

 

