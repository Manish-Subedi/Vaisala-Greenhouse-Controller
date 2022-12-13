/*
 * IntegerEdit.cpp
 *
 *  Created on: 2.2.2016
 *      Author: krl
 */

#include "IntegerEdit.h"
#include <cstdio>
std::atomic<bool> IntegerEdit::integer_changed{ false };

IntegerEdit::IntegerEdit(LiquidCrystal *lcd_, std::string editTitle, int upp, int low, int st): lcd(lcd_), title(editTitle), upper(upp), lower(low), step(st) {
	value = lower;
	edit = lower;
	focus = false;
}

IntegerEdit::~IntegerEdit() {
}

void IntegerEdit::increment() {

	if((edit += step) > upper){
		edit = upper;
	}

}

void IntegerEdit::decrement() {

	if((edit -= step) < lower){
		edit = lower;
	}
}

void IntegerEdit::accept() {
	save();
	integer_changed = true;
}

void IntegerEdit::cancel() {
	edit = value;
}


void IntegerEdit::setFocus(bool focus) {
	this->focus = focus;
}

bool IntegerEdit::getFocus() {
	return this->focus;
}

void IntegerEdit::display() {
	lcd->clear();
	lcd->setCursor(0,0);
	lcd->print(title);
	lcd->setCursor(0,1);
	char s[17];
	if(focus) {
		snprintf(s, 17, "     [%4d]     ", edit);
	}
	else {
		snprintf(s, 17, "      %4d      ", edit);
	}
	lcd->print(s);
}


void IntegerEdit::save() {
	// set current value to be same as edit value
	value = edit;
	// todo: save current value for example to EEPROM for permanent storage
}


int IntegerEdit::getValue() {
	return value;
}
void IntegerEdit::setValue(int value) {
	if(value <= upper && value >= lower){
		edit = value;
		save();
	}

}
