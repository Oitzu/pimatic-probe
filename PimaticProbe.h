#ifndef PimaticProbe_h
#define PimaticProbe_h
#endif

class PimaticProbe
{
	public:
		PimaticProbe( int transmitPin, int systemcode);
		void transmit(bool positive, unsigned long Counter, int Type, int repeats);
	
	private:
		void itob(unsigned long integer, int length, int *array);
		unsigned long power2(int power);
		void sendPair(bool b);
		void sendBit(bool b);
};