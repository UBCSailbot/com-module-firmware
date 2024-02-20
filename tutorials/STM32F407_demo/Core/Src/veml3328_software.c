/* Convert sensor reading value to LUX*/
int convert(int value) {
	/*pre set number. Get this from datasheet*/
	double x = 0.384;
	double lux;
	
	lux = value * x;
	return lux;
}



