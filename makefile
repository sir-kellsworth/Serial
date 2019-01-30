GCC		= g++
MAKE		= make
OPTIMIZE	= -O2 -DSUPPORT_LH7 -DMKSTEMP
CFLAGS		= -std=c++11 -Winline -pipe -g

Serial.so : Serial.cpp Serial.h
	$(GCC) $(CFLAGS) -fPIC -shared Serial.cpp -o libSerial.so.1
	mv libSerial.so.1 lib/

test : test.cpp
	$(GCC) $(CFLAGS) test.cpp -o test -lSerial

install :
	scp Serial.h /usr/include/
	scp lib/libSerial.so.1 /usr/local/lib/libSerial.so
	chmod +x /usr/local/lib/libSerial.so
	ldconfig

uninstall :
	rm /usr/include/Serial.h
	rm /usr/local/lib/libSerial.so

clean :
	rm lib/libSerial.so.1
