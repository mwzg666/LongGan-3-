#ifndef IO_H_
#define IO_H_

#if 0
#define IOOut(io_name,io_mask,data_mask)\
          do\
          {\
           io_name##SEL&=~(io_mask);\
           io_name##OUT=(io_name##OUT& ~(io_mask))|((data_mask) & (io_mask));\
           io_name##DIR|=(io_mask);\
          }while(0)

#endif
		  
#define IODireOut(io_name,io_mask)\
         do\
         {\
          io_name##DIR|=(io_mask);\
         }while(0)
         
#define IOWrite(io_name,io_mask,data_mask)\
	    io_name##OUT=(io_name##OUT & ~(io_mask)) | ((data_mask) & (io_mask))
	

#define IODireIn(io_name,io_mask)\
        do\
        {\
         io_name##DIR&=~(io_mask);\
        }while(0)  
	     
#define IORead(io_name) io_name##IN

#if 0
#define IODefineInt(io_name,io_mask,triggle) \
         do\
         {\
          io_name##SEL&=~(io_mask);\
          io_name##DIR&=~(io_mask);\
          io_name##IES=triggle>0?io_name##IES|(io_mask) : io_name##IES&~(io_mask);\
          io_name##IFG&=~(io_mask);\
          io_name##IE|=(io_mask);\
         }while(0)
#endif

#endif
