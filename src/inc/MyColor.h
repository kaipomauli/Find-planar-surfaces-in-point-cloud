#pragma once
class MyColor {
private:
	int r;
	int g;
	int b;
public:
	
	MyColor(int input_r=0, int input_g=0, int input_b=0);
	int red(void) { return r; }
	int green(void) { return g; }
	int blue(void) { return b; }
};