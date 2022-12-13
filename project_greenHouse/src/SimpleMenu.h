/*
 * SimpleMenu.h
 *
 *  Created on: 3.2.2016
 *      Author: krl
 */

#ifndef SIMPLEMENU_H_
#define SIMPLEMENU_H_
#include <vector>

#include "MenuItem.h"

class SimpleMenu {
public:
	SimpleMenu();
	virtual ~SimpleMenu();
	void addItem(MenuItem *item);
	void event(MenuItem::menuEvent e);
	int getIndex();
private:
	std::vector<MenuItem *> items;
	int position;
};

#endif /* SIMPLEMENU_H_ */
