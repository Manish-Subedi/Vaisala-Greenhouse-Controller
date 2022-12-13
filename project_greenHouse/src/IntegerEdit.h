/*
 * IntegerEdit.h
 *
 *  Created on: 2.2.2016
 *      Author: krl
 */

#ifndef INTEGEREDIT_H_
#define INTEGEREDIT_H_

#include "PropertyEdit.h"
#include "LiquidCrystal.h"
#include <string>
#include <atomic>
class IntegerEdit: public PropertyEdit {
public:
	IntegerEdit(LiquidCrystal *lcd_, std::string editTitle, int upper, int lower, int step);
	virtual ~IntegerEdit();
	void increment();
	void decrement();
	void accept();
	void cancel();
	void setFocus(bool focus);
	bool getFocus();
	void display();
	int getValue();
	void setValue(int value);

	static std::atomic<bool> integer_changed;
private:
	void save();
	void displayEditValue();
	LiquidCrystal *lcd;
	std::string title;
	int value;
	int edit;
	bool focus;
	int upper;
	int lower;
	int step;
};

#endif /* INTEGEREDIT_H_ */
