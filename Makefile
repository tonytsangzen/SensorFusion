target := libsensorfusion.so
library := 
objects += $(patsubst %.c*,%.o,$(wildcard *.c*))

.PHONY:all
all:$(target) ndk

%.o : %.c
	gcc -Wp,-MD,.$@.d -c -o $@ $<

$(target) : $(objects) 
	g++ --shared $^ $(library)  -o $@

ndk:
	export PATH=${PATH}:/Users/cengtao/Library/Android/sdk/ndk//19.2.5345600
	ndk-build

clean:
	rm -rf libs *.o $(target)
