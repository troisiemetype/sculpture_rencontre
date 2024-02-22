#include <tinyNeoPixel_Static.h>
#include <ptc_touch.h>

#include "vector.h"

const uint8_t LED = 3;
const uint8_t PIN_NEO = 8;

const uint16_t UPDATE_RATE = 50;
const uint16_t UPDATE_DELAY = 1000 / UPDATE_RATE;

const uint32_t MAX_FORCE = 160000;
const uint32_t MAX_IDLE_FORCE = 2000000;
const uint32_t MAX_SPEED = 200;

const uint8_t NUM_NEO = 48;

uint8_t pixels[NUM_NEO * 3];

tinyNeoPixel leds = tinyNeoPixel(NUM_NEO, PIN_NEO, NEO_RGB, pixels);

const uint16_t SRAM_START = 0x0000;		// SRAM start is 0x3800, but registers can be used as well, some have undecided reset value.
const uint16_t SRAM_END = 0x3F00;		// 

uint32_t seed;

uint32_t last;

cap_sensor_t nodes[3];

struct field_t{
	Vect pos;
	int32_t force;
	uint16_t inc;
	uint16_t dec;
	bool active;
	uint8_t color[3];
};

// Base position for each first led in each colomn.
Vect ledPos[3];

// Attraction is the position of the touch pad, overflow is the center bottom of the sculpture.
field_t attraction[3];
field_t overflow[3];

// Idle field are thre moving fields, for idle animation (i.e. when nobody touches the sculpture)
// One for each base color
field_t idleField[3];
// Move are the three vectors that are used to move idle fields.
Vect move[3];

bool state = false;

// read the unitialized ram for seeding random.
void makeSeed(){
	uint8_t *p;

	for(uint16_t i = SRAM_START ; i < SRAM_END; ++i){
		p = (uint8_t*)i;
		seed ^= (uint32_t)(*p) << ((i % 4) * 8);
	}
}

// Alternative to the Arduino random() function.
uint32_t xorshift(uint32_t max = 0){
	seed ^= seed << 13;
	seed ^= seed >> 17;
	seed ^= seed << 5;

	if(max != 0) return (uint64_t)seed * max / ((uint32_t)(-1));
	else return seed;
}

void ptc_event_callback(const ptc_cb_event_t eventType, cap_sensor_t* node) {
	// switch statement is maybe not the best suited, as the event type uses bitmasks.
	switch(eventType){
		case PTC_CB_EVENT_TOUCH:									// 0x10
			break;
		case PTC_CB_EVENT_WAKE_TOUCH:								// 0x11
			break;
		case PTC_CB_EVENT_WAKE_NO_TOUCH:							// 0x12
			break;
		case PTC_CB_EVENT_TOUCH_DETECT:								// 0x13
			attraction[ptc_get_node_id(node)].active = true;
			break;
		case PTC_CB_EVENT_TOUCH_RELEASE:							// 0x14
			attraction[ptc_get_node_id(node)].active = false;
			break;
		case PTC_CB_EVENT_CONV_CMPL:								// 0x20
			break;
		case PTC_CB_EVENT_CONV_MUTUAL_CMPL:							// 0x21
			break;
		case PTC_CB_EVENT_CONV_SELF_CMPL:							// 0x24
			break;
		case PTC_CB_EVENT_CONV_SHIELD_CMPL:							// 0x28
			break;
		case PTC_CB_EVENT_CONV_CALIB:								// 0x40
			break;
		case PTC_CB_EVENT_ERR_CALIB:								// 0x41
			break;
		case PTC_CB_EVENT_ERR_CALIB_LOW:							// 0x42
			break;
		case PTC_CB_EVENT_ERR_CALIB_HIGH:							// 0x43
			break;
		case PTC_CB_EVENT_ERR_CALIB_TO:								// 0x44
			break;
		default:
			break;
	}
}

// Test the field state and change if needed.
bool changeField(field_t* f, uint32_t max = MAX_FORCE){
	if(f->active && (f->force < max)){
		f->force += f->inc;
		return true;
	} else if(!f->active && f->force > 0){
		f->force -= f->dec;
		return true;
	}
	return false;
}

// Compute the power of the touch fields, and their impact on the overflow field.
void computeFields(){
	for(uint8_t i = 0; i < 3; ++i){
		field_t* f = &attraction[i];
		field_t* ovf = &overflow[i];

//		Serial.printf("i %i : %i, force %li\n", p, f->active, f->force);
		
		// We compute the field force of the pilar.
		changeField(f);

		// We also compute the center field power, according to the three pilars.
		uint32_t dist = ovf->pos.dist(f->pos);
		uint32_t coeff = pow(f->force / dist, 2);
		if(coeff > 0xff) coeff = 0xff;

		ovf->active = (coeff > 190) ? true : false;

		if(changeField(ovf)){
			ovf->color[i] = 0xff;
		} else {
			ovf->color[i] = 0;
		}
	}
}

// Compute each led according to the touch field
void applyTouchField(){
	for(uint8_t i = 0; i < 24; ++i){
		uint8_t p = i / 8;
		uint8_t n = i % 8;

		field_t* f = &attraction[p];

		// Get the distance from the attractor to the current led.
		Vect v = ledPos[p];
		v.add(0, 0, 1000 * n - 4500);
		uint32_t dist = v.dist(f->pos);
		// Compute the light power
		uint32_t c1 = pow(f->force / dist, 2);
//		Serial.printf("led %i:%i : dist %li ; coeff %li\n", p, n, dist, c1);
		if(c1 > 0xff) c1 = 0xff;

//		Serial.printf("led %i:%i : dist %li ; force %li ; coeff %li\n", p, n, dist, f->force, c1);
//		Serial.printf("color : %08lx\n", color);

		uint32_t color = c1 << (16 - p * 8);
		
		// Apply color to leds.
		leds.setPixelColor(16 * p + n, color);
		leds.setPixelColor(16 * p + n + 8, color);
	}
}

// Modify led value according to overflow field
void applyOverflowField(){
	for(uint8_t i = 0; i < NUM_NEO; ++i){
		// First get the current color of the led (which has been set just before according to what is touched or not).
		uint32_t color = leds.getPixelColor(i);
		uint16_t c[3];
		c[0] = (color >> 16) & 0xff;
		c[1] = (color >> 8) & 0xff;
		c[2] = (color >> 0) & 0xff;

		uint8_t p = i / 16;
		uint8_t n = i % 8;

//		Serial.printf("led %i ; p %i ; n %i\n", i, p, n);
//		Serial.printf("color %i : %i : %i\n", c[0], c[1], c[2]);

		Vect v = ledPos[p];
		v.add(0, 0, 1000 * n - 4500);

		// The three overflow attractors are in the same position, we can thus use one of them.
		uint32_t dist = v.dist(overflow[0].pos);
//		Serial.printf("distance : %li\n", dist);

		for(uint8_t j = 0; j < 3; ++j){
			uint32_t c2 = pow(overflow[j].force / dist, 2);
//			Serial.printf("led %i : dist %li ; coeff %li\n", i, dist, c2);
			if(c2 > 0xff) c2 = 0xff;
	
			c[j] += c2;
			if(c[j] > 0xff) c[j] = 0xff;
//			Serial.printf("color : %08lx\n", color);
		}

		leds.setPixelColor(i, c[0], c[1], c[2]);
	}
}

void applyIdleField(){
	for(uint8_t i = 0; i < NUM_NEO; ++i){
		uint8_t p = i / 16;
		uint8_t n = i % 8;

		bool right = false;
		if((i % 16) != n) right = true;

		Vect v = ledPos[p];
		v.add(0, 0, 1000 * n - 4500);
		if(right) v.rotateZ(-PI / 12);
		else v.rotateZ(PI / 12);

		uint32_t c[3];

		for(uint8_t j = 0; j < 3; ++j){
			uint32_t dist = v.dist(idleField[j].pos);
			c[j] = idleField[j].force / pow(dist / 10, 2);
//			c[j] *= 1000000 / idleField[j].force;
//			if(j == 0 && i < 8) Serial.printf("%i:%i:%i dist : %li, force : %li\n", p, n, right, dist, c[j]);

			if(c[j] > 0xff) c[j] = 0xff;
		}
		leds.setPixelColor(i, c[0], c[1], c[2]);
	}
}

void moveIdleField(){
	for(uint8_t i = 0; i < 3; ++i){
		idleField[i].pos.add(move[i]);

		int32_t x = idleField[i].pos.x();
		int32_t y = idleField[i].pos.y();
		int32_t z = idleField[i].pos.z();

//		if(i == 0) Serial.printf("x %li; y %li; z %li\n", x, y, z);

		if(x > 20000) move[i].x(xorshift(MAX_SPEED) * -1);
		if(x < -20000) move[i].x(xorshift(MAX_SPEED));

		if(y > 20000) move[i].y(xorshift(MAX_SPEED) * -1);
		if(y < -20000) move[i].y(xorshift(MAX_SPEED));

		if(z > 20000) move[i].z(xorshift(MAX_SPEED) * -1);
		if(z < -20000) move[i].z(xorshift(MAX_SPEED));
	}
}

void setup(){

	makeSeed();

	ptc_add_selfcap_node(&nodes[0], PIN_TO_PTC(PIN_PA4), 0);    
	ptc_add_selfcap_node(&nodes[1], PIN_TO_PTC(PIN_PA5), 0);
	ptc_add_selfcap_node(&nodes[2], PIN_TO_PTC(PIN_PA6), 0);

	ptc_get_sm_settings()->touched_max_nom = 255;

	pinMode(LED, OUTPUT);
	pinMode(PIN_NEO, OUTPUT);


//	Serial.begin(115200);

//	Serial.printf("update delay : %i\n", UPDATE_DELAY);

	// As Vect uses int32_t for coordinates, we  scale up the physical coordinates by 100
	attraction[0].pos.set(0, -4000, 5000);						//   0,		-40		, 100
	attraction[1].pos.set(2000, 3460, 5000);					//  20,		 34,6 	, 100
	attraction[2].pos.set(-2000, 3460, 5000);					// -20,		 34,6	, 100

	for(uint8_t i = 0; i < 3; ++i){
		attraction[i].force = 0;
		attraction[i].active = false;
		attraction[i].inc = 2000;
		attraction[i].dec = 1000;

		overflow[i].pos.set(0, 0, -5000);
		overflow[i].force = 0;
		overflow[i].inc = 1200;
		overflow[i].dec = 1200;
		overflow[i].active = false;

		idleField[i].pos.set(0, 0, 0);
		idleField[i].force = MAX_IDLE_FORCE;
		idleField[i].inc = 200000;
		idleField[i].dec = 200000;
		idleField[i].active = true;

		ledPos[i] = attraction[i].pos;
		ledPos[i].sub(0, 0, 4500);

		move[i].set(xorshift(MAX_SPEED) - MAX_SPEED * 2, xorshift(MAX_SPEED) - MAX_SPEED * 2, xorshift(MAX_SPEED) - MAX_SPEED * 2);
	}
}

void loop(){
	uint32_t now = millis();
	// First we update the touch reading.
	ptc_process(now);

	// Now we update the neopixels if the time time has come.
	if((now - last) < UPDATE_DELAY) return;

	last = now;

	leds.show();

	state = false;

	for(uint8_t i = 0; i < 3; ++i){
//		state |= attraction[i].active;
		state |= (attraction[i].force != 0) ? true : false;

		idleField[i].active = state;
//		changeField(&idleField[i], MAX_IDLE_FORCE);
	}

	// Turn the led On if there is a touch.
//	digitalWrite(LED, state);

	computeFields();


	if(state){
		// If there is a touch, we apply the fields linked to each pilar.
		applyTouchField();

		applyOverflowField();
	} else {
		// Else we use a random moving field.
		applyIdleField();
		moveIdleField();
	}
}
