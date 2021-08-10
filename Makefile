target := libfusion.so
library := 
objects += $(patsubst %.c*,%.o,$(wildcard *.c*))

.PHONY:all
all:$(target) ndk

%.o : %.c
	gcc -Wp,-MD,.$@.d -c -o $@ $<

$(target) : $(objects) 
	g++ --shared $^ $(library)  -o $@

ndk:
	ndk-build

clean:
	rm -rf libs *.o $(target)
	ndk-build clean
