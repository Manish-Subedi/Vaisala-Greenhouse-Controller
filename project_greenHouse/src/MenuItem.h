/*
 * MenuItem.h
 *
 *  Created on: 1.2.2016
 *      Author: krl
 */

#ifndef MENUITEM_H_
#define MENUITEM_H_

#include "PropertyEdit.h"


class MenuItem {
public:
	enum menuEvent {
		up,
		down,
		ok,
		back,
		show
	};
	MenuItem(PropertyEdit *property);
	virtual ~MenuItem();
	virtual bool event(menuEvent e);
private:
	PropertyEdit *pe;
};

#endif /* MENUITEM_H_ */
