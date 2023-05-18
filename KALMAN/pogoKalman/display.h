
//##############################################
/*   Quick animations with the Pogobot LEDs   */


// set all leds to the same color
void set_all_leds(int r, int g, int b);

// all leds are set to the same color (red, then green, then blue, then white)
void anim_same(void);

// all leds blink (they alternate between Color(r, g, b) and Color(0, 0, 0) number_of_blinks times)
void anim_blink(int r, int g, int b, int number_of_blinks);

//#####################################
/*   Display things to the console   */


// prints a float to the console
// precision should be 10, 100, 1000... for 1, 2, 3... decimals to print
// using printf("%f", floatVar) doesn't work on Pogobots. They do not need %f, for they are superior, and now posses their own way of printing floats. 
void print_float(float i, int precision);

// prints the len floats in list, with a \t in between each of them
void print_f_list(float* list, int len, int precision);

// print a rows*6 matrix (useful to display imu and Kalman values)
void print_f_matrix(float mat[][6], int rows);
