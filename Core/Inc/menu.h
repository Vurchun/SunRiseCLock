#ifndef _menu_H_
#define _menu_H_

#include <stdio.h>
#include <stdbool.h>
#include "Clock.h"
#include "Buzzer.h"

int menuArraySize = 18;        	// Встановлюємо розмір масиву
int actualIndex = 0;
bool isParamEditMode = false;	// Прапорець режиму редагування параметра
int tmpVValue = 0;       		// Тимчасова змінна для зберігання змінюваного параметра


struct strMenu {
			int id;         // Унікальний ідентифікаційний індекс ID
			int parentid;   // ID батька
			bool isParam;   // Чи є пункт змінним параметром
			char _name[4];  // Назва
			int value;      // Поточне значення
			int _min;       // Мінімально можливе значення параметра
			int _max;       // Максимально можливе значення параметра
};
/* PPPP
 * P__0			Time_Now
 * 		P_0.0	Hour_Now
 * 		P_0.1	Minute_Now
 * 		P_0.2	Set
 * P__1			Time_Rise
 * 		P_1.0	Hour_Rise
 * 		P_1.1	Minute_Rise
 * P__2			Rising_Parametrs
 * 		P_2.0	Period_Rising
 * P__3			Alarm_Parametrs
 * 		P_3.0	Alarm_Status
 * 		P_3.1	Alarm_Melody
 * 		P_3.2	Alarm_Melody_test
 * P__4			Menu_Parametrs
 * 		P_4.0	Numbers_Change_Style
 * 		P_4.1	Menu_Night_Mode
 * P__5			Clock
 */

struct strMenu menu[] = {                         // Встановлюємо пункти меню
	  {0, -1,    false,	"PPPP",		0, 0, 	0},
	  //-----------------------------------------------------------------------
	  {1, 0,     false,	"P__0",		0, 0, 	0},
	  {2, 1,     true,	"P_00",	0, 2, 	0},
	  {3, 2,     true,	"P_01", 	0, 59, 	0},
	  {4, 1,     false, "P_02", 	0, 0, 	0},
	  //-----------------------------------------------------------------------
	  {5, 0,     false, "P__1", 	0, 0, 	0},
	  {6, 5,     true,	"P_10", 	0, 23, 	0},
	  {7, 5,     true,	"P_11", 	0, 59, 	0},
	  //-----------------------------------------------------------------------
	  {8, 0,   	 false, "P__2", 	0, 0, 	0},
	  {9, 8,     true,	"P_20", 	0, 255, 64},
	  //-----------------------------------------------------------------------
	  {10, 0,    false, "P__3", 	0, 0, 	0},
	  {11, 10,   true,	"P_30", 	0, 1, 	1},
	  {12, 10,   true,	"P_31", 	0, 8, 	0},
	  {13, 10,   false,	"P_32", 	0, 0, 	0},
	  //-----------------------------------------------------------------------
	  {14, 0,    false, "P__4", 	0, 0, 	1},
	  {15, 14,   true,	"P_40", 	0, 3, 	0},
	  {16, 14,   true,	"P_41", 	0, 1, 	1},
	  {17, 0,    false, "P__5", 	0, 0, 	0}
	  //-----------------------------------------------------------------------
};


char* setActualMenu(int v, int h) {
	if (v != 0) {               // Рухаємося по вертикалі
		if (v == -1) {            // Команда ВГОРУ (скасування)
			if (isParamEditMode) { // Якщо параметр у режимі редагування, то скасовуємо зміни
				isParamEditMode = false;
			} else { // Якщо пункт меню не у режимі редагування, переміщаємося до батька
				if (menu[actualIndex].parentid > 0) { // Якщо є куди переміщатися вгору (ParentID>0)
					actualIndex = getMenuIndexByID(menu[actualIndex].parentid);
				}
			}
		} else {                        // Якщо команда ВН�?З - входу/редагування
			if (menu[actualIndex].isParam && !isParamEditMode) { // Якщо не в режимі редагування, то ...
				isParamEditMode = true; // Переходимо в режим редагування параметра
				tmpVValue = menu[actualIndex].value; // Тимчасовій змінній присвоюємо актуальне значення параметра
			} else if (menu[actualIndex].isParam && isParamEditMode) { // Якщо в режимі редагування
				menu[actualIndex].value = tmpVValue; // Зберігаємо задане значення
				isParamEditMode = false;      // І виходимо з режиму редагування

			} else {
				bool nochild = true;  // Прапорець, чи є дочірні елементи
				for (int i = 0; i < menuArraySize; i++) {
					if (menu[i].parentid == menu[actualIndex].id) {
						actualIndex = i; // Якщо є, робимо перший попавшийся актуальним елементом
						nochild = false;  // Потомки є
						break;            // Виходимо з for
					}
				}
				if (nochild) { // Якщо ж потомків немає, воспринимаємо як команду
					switch (menu[actualIndex].id) { // Serial.println("Executing command...");         // І тут обробляємо за власним баченням
					case 4:						// Зберігаємо налаштування з комірки памті
						setTimeNow();
						break;
					case 13:						// Завантажуємо налаштування з комірки памті
						testMelody();
						break;
					case 17:
						while (Clock()){Clock();}
						break;
					default:
						break;
					}
				}
			}
		}
	}

	if (h != 0) {             // Якщо горизонтальна навігація
		if (isParamEditMode) {  // У режимі редагування параметра
			tmpVValue += h;        // Змінюємо його значення і ...
			// ... контролюємо, щоб воно залишилося в заданому діапазоні
			if (tmpVValue > menu[actualIndex]._max)
				tmpVValue = menu[actualIndex]._min;
			if (tmpVValue < menu[actualIndex]._min)
				tmpVValue = menu[actualIndex]._max;
		} else { // Якщо режим редагування не активний, навігація серед дочірніх одного батька
			actualIndex = getNearMenuIndexByID(menu[actualIndex].parentid,
					menu[actualIndex].id, h);
		}
	}
	// Отображаем информацию
	if (isParamEditMode) {
		int tmpV[4] = {};
		tmpV[0]=	tmpVValue/1000;
		tmpV[1]=	tmpVValue/100 	- 	tmpV[0]*10;
		tmpV[2]=	tmpVValue/10 	- 	tmpV[0]*100 	- tmpV[1]*10;
		tmpV[3]=	tmpVValue 		- 	tmpV[0]*1000 	- tmpV[1]*100 	- tmpV[2]*10;
		return tmpV;
	} else {
		return menu[actualIndex]._name;
	}
}
int getMenuIndexByID(int id) { // Функція отримання індексу пункту меню за його ID
	for (int i = 0; i < menuArraySize; i++) {
		if (menu[i].id == id)
			return i;
	}
	return -1;
}
int getNearMenuIndexByID(int parentid, int id, int side) { // Функція отримання індексу пункту меню наступного або попереднього від актуального
	int prevID = -1;      // Змінна для зберігання індексу попереднього елемента
	int nextID = -1;        // Змінна для зберігання індексу наступного елемента
	int actualID = -1;     // Змінна для зберігання індексу актуального елемента

	int firstID = -1;  // Змінна для зберігання індексу першого елемента
	int lastID = -1;   // Змінна для зберігання індексу останнього елемента

	for (int i = 0; i < menuArraySize; i++) {
		if (menu[i].parentid == parentid) { // Перебираємо всі елементи з одним батьківським ID
			if (firstID == -1)
				firstID = i;     // Запам'ятовуємо перший елемент списку

			if (menu[i].id == id) {
				actualID = i;  // Запам'ятовуємо актуальний елемент списку
			} else {
				if (actualID == -1) { // Якщо зустрівся елемент до актуального, робимо його попереднім
					prevID = i;
				} else if (actualID != -1 && nextID == -1) { // Якщо зустрівся елемент після актуального, робимо його наступним
					nextID = i;
				}
			}
			lastID = i;  // Кожний наступний елемент - останній
		}
	}

	if (nextID == -1)
		nextID = firstID; // Якщо наступного елемента немає - по колу видаємо перший
	if (prevID == -1)
		prevID = lastID; // Якщо попереднього елемента немає - по колу видаємо останній
	if (side == -1)
		return prevID; // В залежності від напрямку обертання, видаємо потрібний індекс
	else
		return nextID;
	return -1;
}
#endif
