#line 1 "..\\bsp\\serial.c"

































 

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





#line 37 "..\\bsp\\serial.c"
#line 38 "..\\bsp\\serial.c"
#line 1 "..\\rt_thread\\include\\rtdevice.h"























 




#line 30 "..\\rt_thread\\include\\rtdevice.h"







 
struct rt_completion
{
    rt_uint32_t flag;

     
    rt_list_t suspended_list;
};

 
struct rt_ringbuffer
{
    rt_uint8_t *buffer_ptr;
    




















 
    rt_uint16_t read_mirror : 1;
    rt_uint16_t read_index : 15;
    rt_uint16_t write_mirror : 1;
    rt_uint16_t write_index : 15;
    
 
    rt_int16_t buffer_size;
};

 
struct rt_portal_device
{
    struct rt_device parent;
    struct rt_device *write_dev;
    struct rt_device *read_dev;
};

 

enum rt_pipe_flag
{
     
    RT_PIPE_FLAG_NONBLOCK_RDWR = 0x00,
     
    RT_PIPE_FLAG_BLOCK_RD = 0x01,
     
    RT_PIPE_FLAG_BLOCK_WR = 0x02,
    

 
    RT_PIPE_FLAG_FORCE_WR = 0x04,
};

struct rt_pipe_device
{
    struct rt_device parent;

     
    struct rt_ringbuffer ringbuffer;

    enum rt_pipe_flag flag;

     
    rt_list_t suspended_read_list;
    rt_list_t suspended_write_list;

    struct rt_portal_device *write_portal;
    struct rt_portal_device *read_portal;
};








struct rt_data_item;


 
struct rt_data_queue
{
    rt_uint16_t size;
    rt_uint16_t lwm;
    rt_bool_t   waiting_lwm;

    rt_uint16_t get_index;
    rt_uint16_t put_index;

    struct rt_data_item *queue;

    rt_list_t suspended_push_list;
    rt_list_t suspended_pop_list;

     
    void (*evt_notify)(struct rt_data_queue *queue, rt_uint32_t event);
};

 
struct rt_workqueue
{
	rt_list_t   work_list;
	rt_thread_t work_thread;
};

struct rt_work
{
	rt_list_t list;

	void (*work_func)(struct rt_work* work, void* work_data);
	void *work_data;
};



 
void rt_completion_init(struct rt_completion *completion);
rt_err_t rt_completion_wait(struct rt_completion *completion,
                            rt_int32_t            timeout);
void rt_completion_done(struct rt_completion *completion);






 
void rt_ringbuffer_init(struct rt_ringbuffer *rb,
                        rt_uint8_t           *pool,
                        rt_int16_t            size);
rt_size_t rt_ringbuffer_put(struct rt_ringbuffer *rb,
                            const rt_uint8_t     *ptr,
                            rt_uint16_t           length);
rt_size_t rt_ringbuffer_put_force(struct rt_ringbuffer *rb,
                                  const rt_uint8_t     *ptr,
                                  rt_uint16_t           length);
rt_size_t rt_ringbuffer_putchar(struct rt_ringbuffer *rb,
                                const rt_uint8_t      ch);
rt_size_t rt_ringbuffer_putchar_force(struct rt_ringbuffer *rb,
                                      const rt_uint8_t      ch);
rt_size_t rt_ringbuffer_get(struct rt_ringbuffer *rb,
                            rt_uint8_t           *ptr,
                            rt_uint16_t           length);
rt_size_t rt_ringbuffer_getchar(struct rt_ringbuffer *rb, rt_uint8_t *ch);

enum rt_ringbuffer_state
{
    RT_RINGBUFFER_EMPTY,
    RT_RINGBUFFER_FULL,
     
    RT_RINGBUFFER_HALFFULL,
};

static __inline rt_uint16_t rt_ringbuffer_get_size(struct rt_ringbuffer *rb)
{
    if (!(rb != (0))) { rt_assert_handler("rb != RT_NULL", __FUNCTION__, 208); };
    return rb->buffer_size;
}

static __inline enum rt_ringbuffer_state
rt_ringbuffer_status(struct rt_ringbuffer *rb)
{
    if (rb->read_index == rb->write_index)
    {
        if (rb->read_mirror == rb->write_mirror)
            return RT_RINGBUFFER_EMPTY;
        else
            return RT_RINGBUFFER_FULL;
    }
    return RT_RINGBUFFER_HALFFULL;
}

 
static __inline rt_uint16_t rt_ringbuffer_data_len(struct rt_ringbuffer *rb)
{
    switch (rt_ringbuffer_status(rb))
    {
    case RT_RINGBUFFER_EMPTY:
        return 0;
    case RT_RINGBUFFER_FULL:
        return rb->buffer_size;
    case RT_RINGBUFFER_HALFFULL:
    default:
        if (rb->write_index > rb->read_index)
            return rb->write_index - rb->read_index;
        else
            return rb->buffer_size - (rb->read_index - rb->write_index);
    };
}

 




 
rt_err_t rt_pipe_init(struct rt_pipe_device *pipe,
                      const char *name,
                      enum rt_pipe_flag flag,
                      rt_uint8_t *buf,
                      rt_size_t size);
rt_err_t rt_pipe_detach(struct rt_pipe_device *pipe);

rt_err_t rt_pipe_create(const char *name, enum rt_pipe_flag flag, rt_size_t size);
void rt_pipe_destroy(struct rt_pipe_device *pipe);




 

rt_err_t rt_portal_init(struct rt_portal_device *portal,
                        const char *portal_name,
                        const char *write_dev,
                        const char *read_dev);
rt_err_t rt_portal_detach(struct rt_portal_device *portal);


rt_err_t rt_portal_create(const char *name,
                          const char *write_dev,
                          const char *read_dev);
void rt_portal_destroy(struct rt_portal_device *portal);




 
rt_err_t rt_data_queue_init(struct rt_data_queue *queue,
                            rt_uint16_t           size,
                            rt_uint16_t           lwm,
                            void (*evt_notify)(struct rt_data_queue *queue, rt_uint32_t event));
rt_err_t rt_data_queue_push(struct rt_data_queue *queue,
                            const void           *data_ptr,
                            rt_size_t             data_size,
                            rt_int32_t            timeout);
rt_err_t rt_data_queue_pop(struct rt_data_queue *queue,
                           const void          **data_ptr,
                           rt_size_t            *size,
                           rt_int32_t            timeout);
rt_err_t rt_data_queue_peak(struct rt_data_queue *queue,
                            const void          **data_ptr,
                            rt_size_t            *size);
void rt_data_queue_reset(struct rt_data_queue *queue);




 
struct rt_workqueue *rt_workqueue_create(const char* name, rt_uint16_t stack_size, rt_uint8_t priority);
rt_err_t rt_workqueue_destroy(struct rt_workqueue* queue);
rt_err_t rt_workqueue_dowork(struct rt_workqueue* queue, struct rt_work* work);
rt_err_t rt_workqueue_cancel_work(struct rt_workqueue* queue, struct rt_work* work);

static __inline void rt_work_init(struct rt_work* work, void (*work_func)(struct rt_work* work, void* work_data),
    void* work_data)
{
    rt_list_init(&(work->list));
    work->work_func = work_func;
    work->work_data = work_data;
}


#line 321 "..\\rt_thread\\include\\rtdevice.h"



























#line 356 "..\\rt_thread\\include\\rtdevice.h"

































#line 39 "..\\bsp\\serial.c"
#line 40 "..\\bsp\\serial.c"
#line 1 "..\\bsp\\serial.h"

























 




#line 32 "..\\bsp\\serial.h"
#line 33 "..\\bsp\\serial.h"

#line 46 "..\\bsp\\serial.h"













































 
#line 103 "..\\bsp\\serial.h"

struct serial_configure
{
    rt_uint32_t baud_rate;

    rt_uint32_t data_bits               :4;
    rt_uint32_t stop_bits               :2;
    rt_uint32_t parity                  :2;
    rt_uint32_t bit_order               :1;
    rt_uint32_t invert                  :1;
    rt_uint32_t bufsz                   :16;
    rt_uint32_t reserved                :4;
};



 
struct rt_serial_rx_fifo
{
     
    rt_uint8_t *buffer;

    rt_uint16_t put_index, get_index;
};

struct rt_serial_tx_fifo
{
    struct rt_completion completion;
};



 
struct rt_serial_rx_dma
{
    rt_bool_t activated;
};

struct rt_serial_tx_dma
{
    rt_bool_t activated;
    struct rt_data_queue data_queue;
};

struct rt_serial_device
{
    struct rt_device          parent;

    const struct rt_uart_ops *ops;
    struct serial_configure   config;

    void *serial_rx;
    void *serial_tx;
};
typedef struct rt_serial_device rt_serial_t;



 
struct rt_uart_ops
{
    rt_err_t (*configure)(struct rt_serial_device *serial, struct serial_configure *cfg);
    rt_err_t (*control)(struct rt_serial_device *serial, int cmd, void *arg);

    int (*putc)(struct rt_serial_device *serial, char c);
    int (*getc)(struct rt_serial_device *serial);

    rt_size_t (*dma_transmit)(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction);
};

void rt_hw_serial_isr(struct rt_serial_device *serial, int event);

rt_err_t rt_hw_serial_register(struct rt_serial_device *serial,
                               const char              *name,
                               rt_uint32_t              flag,
                               void                    *data);

#line 41 "..\\bsp\\serial.c"


 
static __inline int _serial_poll_rx(struct rt_serial_device *serial, rt_uint8_t *data, int length)
{
    int ch;
    int size;

    if (!(serial != (0))) { rt_assert_handler("serial != RT_NULL", __FUNCTION__, 49); };
    size = length;

    while (length)
    {
        ch = serial->ops->getc(serial);
        if (ch == -1) break;

        *data = ch;
        data ++; length --;

        if (ch == '\n') break;
    }

    return size - length;
}

static __inline int _serial_poll_tx(struct rt_serial_device *serial, const rt_uint8_t *data, int length)
{
    int size;
    if (!(serial != (0))) { rt_assert_handler("serial != RT_NULL", __FUNCTION__, 69); };

    size = length;
    while (length)
    {
        


 
        if (*data == '\n' && (serial->parent.open_flag & 0x040))
        {
            serial->ops->putc(serial, '\r');
        }

        serial->ops->putc(serial, *data);

        ++ data;
        -- length;
    }

    return size - length;
}



 
static __inline int _serial_int_rx(struct rt_serial_device *serial, rt_uint8_t *data, int length)
{
    int size;
    struct rt_serial_rx_fifo* rx_fifo;

    if (!(serial != (0))) { rt_assert_handler("serial != RT_NULL", __FUNCTION__, 100); };
    size = length;

    rx_fifo = (struct rt_serial_rx_fifo*) serial->serial_rx;
    if (!(rx_fifo != (0))) { rt_assert_handler("rx_fifo != RT_NULL", __FUNCTION__, 104); };

     
    while (length)
    {
        int ch;
        rt_base_t level;

         
        level = rt_hw_interrupt_disable();
        if (rx_fifo->get_index != rx_fifo->put_index)
        {
            ch = rx_fifo->buffer[rx_fifo->get_index];
            rx_fifo->get_index += 1;
            if (rx_fifo->get_index >= serial->config.bufsz) rx_fifo->get_index = 0;
        }
        else
        {
             
            rt_hw_interrupt_enable(level);
            break;
        }

         
        rt_hw_interrupt_enable(level);

        *data = ch & 0xff;
        data ++; length --;
    }

    return size - length;
}

static __inline int _serial_int_tx(struct rt_serial_device *serial, const rt_uint8_t *data, int length)
{
    int size;
    struct rt_serial_tx_fifo *tx;

    if (!(serial != (0))) { rt_assert_handler("serial != RT_NULL", __FUNCTION__, 142); };

    size = length;
    tx = (struct rt_serial_tx_fifo*) serial->serial_tx;
    if (!(tx != (0))) { rt_assert_handler("tx != RT_NULL", __FUNCTION__, 146); };

    while (length)
    {
        if (serial->ops->putc(serial, *(char*)data) == -1)
        {
            rt_completion_wait(&(tx->completion), -1);
            continue;
        }

        data ++; length --;
    }

    return size - length;
}







 
static rt_size_t rt_dma_calc_recved_len(struct rt_serial_device *serial) {
    static rt_size_t rx_length;
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;

    if (!(rx_fifo != (0))) { rt_assert_handler("rx_fifo != RT_NULL", __FUNCTION__, 173); };

    rx_length = (rx_fifo->put_index >= rx_fifo->get_index)? (rx_fifo->put_index - rx_fifo->get_index):
                        (serial->config.bufsz - (rx_fifo->get_index - rx_fifo->put_index));
    return rx_length;
}






 
static void rt_dma_recv_update_get_index(struct rt_serial_device *serial, rt_size_t len) {
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;

    if (!(rx_fifo != (0))) { rt_assert_handler("rx_fifo != RT_NULL", __FUNCTION__, 189); };
    if (!(len <= rt_dma_calc_recved_len(serial))) { rt_assert_handler("len <= rt_dma_calc_recved_len(serial)", __FUNCTION__, 190); };

    rx_fifo->get_index += len;
    if (rx_fifo->get_index > serial->config.bufsz ) {
        rx_fifo->get_index -= serial->config.bufsz;
    }
}






 
static void rt_dma_recv_update_put_index(struct rt_serial_device *serial, rt_size_t len) {
    struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
    rt_size_t i;

    if (!(rx_fifo != (0))) { rt_assert_handler("rx_fifo != RT_NULL", __FUNCTION__, 208); };

    if (rx_fifo->get_index <= rx_fifo->put_index) {
        rx_fifo->put_index += len;
         
        if (rx_fifo->put_index >= serial->config.bufsz) {
            for (i = 0; i <= len / serial->config.bufsz; i++) {
                rx_fifo->put_index -= serial->config.bufsz;
            }
             
            if (rx_fifo->put_index >= rx_fifo->get_index) {
                rx_fifo->get_index = rx_fifo->put_index + 1;
            }
        }
    } else {
        rx_fifo->put_index += len;
        if(rx_fifo->put_index >= rx_fifo->get_index) {
             
            if(rx_fifo->put_index >= serial->config.bufsz) {
                for (i = 0; i <= len / serial->config.bufsz; i++) {
                    rx_fifo->put_index -= serial->config.bufsz;
                }
            }
             
            rx_fifo->get_index = rx_fifo->put_index + 1;
        }
    }
}



 
static __inline int _serial_dma_rx(struct rt_serial_device *serial, rt_uint8_t *data, int length)
{
    rt_base_t level;

    if (!((serial != (0)) && (data != (0)))) { rt_assert_handler("(serial != RT_NULL) && (data != RT_NULL)", __FUNCTION__, 244); };

    level = rt_hw_interrupt_disable();

    if (serial->config.bufsz == 0) {
        int result = 0;
        struct rt_serial_rx_dma *rx_dma;

        rx_dma = (struct rt_serial_rx_dma*)serial->serial_rx;
        if (!(rx_dma != (0))) { rt_assert_handler("rx_dma != RT_NULL", __FUNCTION__, 253); };

        if (rx_dma->activated != 1)
        {
            rx_dma->activated = 1;
            if (!(serial->ops->dma_transmit != (0))) { rt_assert_handler("serial->ops->dma_transmit != RT_NULL", __FUNCTION__, 258); };
            serial->ops->dma_transmit(serial, data, length, 0x01);
        }
        else result = -7;
        rt_hw_interrupt_enable(level);

        if (result == 0) return length;

        rt_set_errno(result);
        return 0;
    } else {
        struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *) serial->serial_rx;
        rt_size_t recv_len = 0, fifo_recved_len = rt_dma_calc_recved_len(serial);

        if (!(rx_fifo != (0))) { rt_assert_handler("rx_fifo != RT_NULL", __FUNCTION__, 272); };

        if (length < fifo_recved_len) {
            recv_len = length;
        } else {
            recv_len = fifo_recved_len;
        }

        if (rx_fifo->get_index + recv_len < serial->config.bufsz) {
            rt_memcpy(data, rx_fifo->buffer + rx_fifo->get_index, recv_len);
        } else {
            rt_memcpy(data, rx_fifo->buffer + rx_fifo->get_index,
                    serial->config.bufsz - rx_fifo->get_index);
            rt_memcpy(data + serial->config.bufsz - rx_fifo->get_index, rx_fifo->buffer,
                    recv_len + rx_fifo->get_index - serial->config.bufsz);
        }
        rt_dma_recv_update_get_index(serial, recv_len);
        rt_hw_interrupt_enable(level);
        return recv_len;
    }
}

static __inline int _serial_dma_tx(struct rt_serial_device *serial, const rt_uint8_t *data, int length)
{
    rt_base_t level;
    rt_err_t result;
    struct rt_serial_tx_dma *tx_dma;

    tx_dma = (struct rt_serial_tx_dma*)(serial->serial_tx);

    result = rt_data_queue_push(&(tx_dma->data_queue), data, length, -1);
    if (result == 0)
    {
        level = rt_hw_interrupt_disable();
        if (tx_dma->activated != 1)
        {
            tx_dma->activated = 1;
            rt_hw_interrupt_enable(level);

             
            serial->ops->dma_transmit(serial, (rt_uint8_t *)data, length, 0x02);
        }
        else
        {
            rt_hw_interrupt_enable(level);
        }

        return length;
    }
    else
    {
        rt_set_errno(result);
        return 0;
    }
}

 


 
static rt_err_t rt_serial_init(struct rt_device *dev)
{
    rt_err_t result = 0;
    struct rt_serial_device *serial;

    if (!(dev != (0))) { rt_assert_handler("dev != RT_NULL", __FUNCTION__, 337); };
    serial = (struct rt_serial_device *)dev;

     
    serial->serial_rx = (0);
    serial->serial_tx = (0);

     
    if (serial->ops->configure)
        result = serial->ops->configure(serial, &serial->config);

    return result;
}

static rt_err_t rt_serial_open(struct rt_device *dev, rt_uint16_t oflag)
{
    struct rt_serial_device *serial;

    if (!(dev != (0))) { rt_assert_handler("dev != RT_NULL", __FUNCTION__, 355); };
    serial = (struct rt_serial_device *)dev;

     
    if ((oflag & 0x200) && !(dev->flag & 0x200))
        return -8;
    if ((oflag & 0x800) && !(dev->flag & 0x800))
        return -8;
    if ((oflag & 0x100) && !(dev->flag & 0x100))
        return -8;
    if ((oflag & 0x400) && !(dev->flag & 0x400))
        return -8;

     
    dev->open_flag = oflag & 0xff;

     
    if (serial->serial_rx == (0))
    {
        if (oflag & 0x200)
        {
            if (serial->config.bufsz == 0) {
                struct rt_serial_rx_dma* rx_dma;

                rx_dma = (struct rt_serial_rx_dma*) rt_malloc (sizeof(struct rt_serial_rx_dma));
                if (!(rx_dma != (0))) { rt_assert_handler("rx_dma != RT_NULL", __FUNCTION__, 380); };
                rx_dma->activated = 0;

                serial->serial_rx = rx_dma;
            } else {
                struct rt_serial_rx_fifo* rx_fifo;

                rx_fifo = (struct rt_serial_rx_fifo*) rt_malloc (sizeof(struct rt_serial_rx_fifo) +
                    serial->config.bufsz);
                if (!(rx_fifo != (0))) { rt_assert_handler("rx_fifo != RT_NULL", __FUNCTION__, 389); };
                rx_fifo->buffer = (rt_uint8_t*) (rx_fifo + 1);
                rt_memset(rx_fifo->buffer, 0, serial->config.bufsz);
                rx_fifo->put_index = 0;
                rx_fifo->get_index = 0;
                serial->serial_rx = rx_fifo;
                 
                serial->ops->control(serial, 0x03, (void *) 0x200);
            }
            dev->open_flag |= 0x200;
        }
        else if (oflag & 0x100)
        {
            struct rt_serial_rx_fifo* rx_fifo;

            rx_fifo = (struct rt_serial_rx_fifo*) rt_malloc (sizeof(struct rt_serial_rx_fifo) +
                serial->config.bufsz);
            if (!(rx_fifo != (0))) { rt_assert_handler("rx_fifo != RT_NULL", __FUNCTION__, 406); };
            rx_fifo->buffer = (rt_uint8_t*) (rx_fifo + 1);
            rt_memset(rx_fifo->buffer, 0, serial->config.bufsz);
            rx_fifo->put_index = 0;
            rx_fifo->get_index = 0;

            serial->serial_rx = rx_fifo;
            dev->open_flag |= 0x100;
             
            serial->ops->control(serial, 0x10, (void *)0x100);
        }
        else
        {
            serial->serial_rx = (0);
        }
    }

    if (serial->serial_tx == (0))
    {
        if (oflag & 0x800)
        {
            struct rt_serial_tx_dma* tx_dma;

            tx_dma = (struct rt_serial_tx_dma*) rt_malloc (sizeof(struct rt_serial_tx_dma));
            if (!(tx_dma != (0))) { rt_assert_handler("tx_dma != RT_NULL", __FUNCTION__, 430); };
            tx_dma->activated = 0;

            rt_data_queue_init(&(tx_dma->data_queue), 8, 4, (0));
            serial->serial_tx = tx_dma;

            dev->open_flag |= 0x800;
        }
        else if (oflag & 0x400)
        {
            struct rt_serial_tx_fifo *tx_fifo;

            tx_fifo = (struct rt_serial_tx_fifo*) rt_malloc(sizeof(struct rt_serial_tx_fifo));
            if (!(tx_fifo != (0))) { rt_assert_handler("tx_fifo != RT_NULL", __FUNCTION__, 443); };

            rt_completion_init(&(tx_fifo->completion));
            serial->serial_tx = tx_fifo;

            dev->open_flag |= 0x400;
             
            serial->ops->control(serial, 0x10, (void *)0x400);
        }
        else
        {
            serial->serial_tx = (0);
        }
    }

    return 0;
}

static rt_err_t rt_serial_close(struct rt_device *dev)
{
    struct rt_serial_device *serial;

    if (!(dev != (0))) { rt_assert_handler("dev != RT_NULL", __FUNCTION__, 465); };
    serial = (struct rt_serial_device *)dev;

     
    if (dev->ref_count > 1) return 0;

    if (dev->open_flag & 0x100)
    {
        struct rt_serial_rx_fifo* rx_fifo;

        rx_fifo = (struct rt_serial_rx_fifo*)serial->serial_rx;
        if (!(rx_fifo != (0))) { rt_assert_handler("rx_fifo != RT_NULL", __FUNCTION__, 476); };

        rt_free(rx_fifo);
        serial->serial_rx = (0);
        dev->open_flag &= ~0x100;
         
        serial->ops->control(serial, 0x11, (void*)0x100);
    }
    else if (dev->open_flag & 0x200)
    {
        if (serial->config.bufsz == 0) {
            struct rt_serial_rx_dma* rx_dma;

            rx_dma = (struct rt_serial_rx_dma*)serial->serial_rx;
            if (!(rx_dma != (0))) { rt_assert_handler("rx_dma != RT_NULL", __FUNCTION__, 490); };

            rt_free(rx_dma);
        } else {
            struct rt_serial_rx_fifo* rx_fifo;

            rx_fifo = (struct rt_serial_rx_fifo*)serial->serial_rx;
            if (!(rx_fifo != (0))) { rt_assert_handler("rx_fifo != RT_NULL", __FUNCTION__, 497); };

            rt_free(rx_fifo);
        }
         
        serial->ops->control(serial, 0x11, (void *) 0x200);
        serial->serial_rx = (0);
        dev->open_flag &= ~0x200;
    }

    if (dev->open_flag & 0x400)
    {
        struct rt_serial_tx_fifo* tx_fifo;

        tx_fifo = (struct rt_serial_tx_fifo*)serial->serial_tx;
        if (!(tx_fifo != (0))) { rt_assert_handler("tx_fifo != RT_NULL", __FUNCTION__, 512); };

        rt_free(tx_fifo);
        serial->serial_tx = (0);
        dev->open_flag &= ~0x400;
         
        serial->ops->control(serial, 0x11, (void*)0x400);
    }
    else if (dev->open_flag & 0x800)
    {
        struct rt_serial_tx_dma* tx_dma;

        tx_dma = (struct rt_serial_tx_dma*)serial->serial_tx;
        if (!(tx_dma != (0))) { rt_assert_handler("tx_dma != RT_NULL", __FUNCTION__, 525); };

        rt_free(tx_dma);
        serial->serial_tx = (0);
        dev->open_flag &= ~0x800;
    }

    return 0;
}

static rt_size_t rt_serial_read(struct rt_device *dev,
                                rt_off_t          pos,
                                void             *buffer,
                                rt_size_t         size)
{
    struct rt_serial_device *serial;

    if (!(dev != (0))) { rt_assert_handler("dev != RT_NULL", __FUNCTION__, 542); };
    if (size == 0) return 0;

    serial = (struct rt_serial_device *)dev;

    if (dev->open_flag & 0x100)
    {
        return _serial_int_rx(serial, buffer, size);
    }
    else if (dev->open_flag & 0x200)
    {
        return _serial_dma_rx(serial, buffer, size);
    }

    return _serial_poll_rx(serial, buffer, size);
}

static rt_size_t rt_serial_write(struct rt_device *dev,
                                 rt_off_t          pos,
                                 const void       *buffer,
                                 rt_size_t         size)
{
    struct rt_serial_device *serial;

    if (!(dev != (0))) { rt_assert_handler("dev != RT_NULL", __FUNCTION__, 566); };
    if (size == 0) return 0;

    serial = (struct rt_serial_device *)dev;

    if (dev->open_flag & 0x400)
    {
        return _serial_int_tx(serial, buffer, size);
    }
    else if (dev->open_flag & 0x800)
    {
        return _serial_dma_tx(serial, buffer, size);
    }
    else
    {
        return _serial_poll_tx(serial, buffer, size);
    }
}

static rt_err_t rt_serial_control(struct rt_device *dev,
                                  rt_uint8_t        cmd,
                                  void             *args)
{
    struct rt_serial_device *serial;

    if (!(dev != (0))) { rt_assert_handler("dev != RT_NULL", __FUNCTION__, 591); };
    serial = (struct rt_serial_device *)dev;

    switch (cmd)
    {
        case 0x02:
             
            dev->flag |= 0x020;
            break;

        case 0x01:
             
            dev->flag &= ~0x020;
            break;

        case 0x03:
			if (args)
			{
				 
				serial->config = *(struct serial_configure *)args;

				if (dev->ref_count)
				{
		             
		            serial->ops->configure(serial, (struct serial_configure *)args);
				}
			}
			
            break;

        default :
             
            serial->ops->control(serial, cmd, args);
            break;
    }

    return 0;
}



 
rt_err_t rt_hw_serial_register(struct rt_serial_device *serial,
                               const char              *name,
                               rt_uint32_t              flag,
                               void                    *data)
{
    struct rt_device *device;
    if (!(serial != (0))) { rt_assert_handler("serial != RT_NULL", __FUNCTION__, 639); };

    device = &(serial->parent);

    device->type        = RT_Device_Class_Char;
    device->rx_indicate = (0);
    device->tx_complete = (0);

    device->init        = rt_serial_init;
    device->open        = rt_serial_open;
    device->close       = rt_serial_close;
    device->read        = rt_serial_read;
    device->write       = rt_serial_write;
    device->control     = rt_serial_control;
    device->user_data   = data;

     
    return rt_device_register(device, name, flag);
}

 
void rt_hw_serial_isr(struct rt_serial_device *serial, int event)
{
    switch (event & 0xff)
    {
        case 0x01:
        {
            int ch = -1;
            rt_base_t level;
            struct rt_serial_rx_fifo* rx_fifo;

             
            rx_fifo = (struct rt_serial_rx_fifo*)serial->serial_rx;
            if (!(rx_fifo != (0))) { rt_assert_handler("rx_fifo != RT_NULL", __FUNCTION__, 672); };

            while (1)
            {
                ch = serial->ops->getc(serial);
                if (ch == -1) break;


                 
                level = rt_hw_interrupt_disable();

                rx_fifo->buffer[rx_fifo->put_index] = ch;
                rx_fifo->put_index += 1;
                if (rx_fifo->put_index >= serial->config.bufsz) rx_fifo->put_index = 0;

                 
                if (rx_fifo->put_index == rx_fifo->get_index)
                {
                    rx_fifo->get_index += 1;
                    if (rx_fifo->get_index >= serial->config.bufsz) rx_fifo->get_index = 0;
                }

                 
                rt_hw_interrupt_enable(level);
            }

             
            if (serial->parent.rx_indicate != (0))
            {
                rt_size_t rx_length;

                 
                level = rt_hw_interrupt_disable();
                rx_length = (rx_fifo->put_index >= rx_fifo->get_index)? (rx_fifo->put_index - rx_fifo->get_index):
                    (serial->config.bufsz - (rx_fifo->get_index - rx_fifo->put_index));
                rt_hw_interrupt_enable(level);

                serial->parent.rx_indicate(&serial->parent, rx_length);
            }
            break;
        }
        case 0x02:
        {
            struct rt_serial_tx_fifo* tx_fifo;

            tx_fifo = (struct rt_serial_tx_fifo*)serial->serial_tx;
            rt_completion_done(&(tx_fifo->completion));
            break;
        }
        case 0x04:
        {
            const void *data_ptr;
            rt_size_t data_size;
            const void *last_data_ptr;
            struct rt_serial_tx_dma* tx_dma;

            tx_dma = (struct rt_serial_tx_dma*) serial->serial_tx;

            rt_data_queue_pop(&(tx_dma->data_queue), &last_data_ptr, &data_size, 0);
            if (rt_data_queue_peak(&(tx_dma->data_queue), &data_ptr, &data_size) == 0)
            {
                 
                tx_dma->activated = 1;
                serial->ops->dma_transmit(serial, (rt_uint8_t *)data_ptr, data_size, 0x02);
            }
            else
            {
                tx_dma->activated = 0;
            }

             
            if (serial->parent.tx_complete != (0))
            {
                serial->parent.tx_complete(&serial->parent, (void*)last_data_ptr);
            }
            break;
        }
        case 0x03:
        {
            int length;

             
            length = (event & (~0xff)) >> 8;

            if (serial->config.bufsz == 0) {
                struct rt_serial_rx_dma* rx_dma;

                rx_dma = (struct rt_serial_rx_dma*)serial->serial_rx;
                if (!(rx_dma != (0))) { rt_assert_handler("rx_dma != RT_NULL", __FUNCTION__, 760); };

                if (!(serial->parent . rx_indicate != (0))) { rt_assert_handler("serial->parent.rx_indicate != RT_NULL", __FUNCTION__, 762); };
                serial->parent.rx_indicate(&(serial->parent), length);
                rx_dma->activated = 0;
            } else {
                 
                rt_dma_recv_update_put_index(serial, length);
                 
                if (serial->parent.rx_indicate != (0)) {
                    serial->parent.rx_indicate(&(serial->parent), rt_dma_calc_recved_len(serial));
                }
            }
            break;
        }
    }
}
