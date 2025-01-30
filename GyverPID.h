/*
 GyverPID - Bibliothèque de contrôleurs PID pour Arduino
 Documentation : https://alexgyver.ru/gyverpid/
 GitHub : https://github.com/GyverLibs/GyverPID
 Possibilités :
 - Le temps d'un calcul est d'environ 70 µs
 - Mode opératoire par valeur ou par son changement (pour l'intégration des processus)
 - Renvoie le résultat à l'aide du minuteur intégré ou en mode manuel
 - Calibrateurs de coefficients intégrés
 - Mode de fonctionnement par erreur et par erreur de mesure
 - Optimiseurs de somme intégrale intégrés

 Alex Gyver, alex@alexgyver.ru
 https://alexgyver.ru/
 Licence MIT

 Versions:
 v1.1 - définitions supprimées
 v1.2 - définitions renvoyées
 v1.3 - les calculs sont accélérés, la bibliothèque est plus légère
 v2.0 - la logique de travail a été légèrement repensée, le code a été amélioré, simplifié et allégé
 v2.1 - intégrale déplacée vers le public
 v2.2 - Optimisation des calculs
 v2.3 - ajout du mode PID_INTEGRAL_WINDOW
 v2.4 - implémentation ajoutée à la classe
 v3.0
 - Ajout du mode d'optimisation pour la composante intégrale (voir doc)
 - Ajout de calibrateurs de coefficients automatiques (voir exemples et documentation)
 v3.1 - mode ON_RATE fixe, ajout d'une limitation automatique de int. montants
 v3.2 - une petite optimisation, ajout de getResultNow
 v3.3 - dans les tuners, vous pouvez passer un autre gestionnaire de classe Stream pour le débogage
*/

#ifndef GyverPID_h
#define GyverPID_h
#include <Arduino.h>

#if defined(PID_INTEGER)	// расчёты с целыми числами
typedef int datatype;
#else						// расчёты с float числами
typedef float datatype;
#endif

#define NORMAL 0
#define REVERSE 1
#define ON_ERROR 0
#define ON_RATE 1

class GyverPID {
public:
    // ==== datatype это float или int, в зависимости от выбранного (см. пример integer_calc) ====
    GyverPID(){}
    
    // kp, ki, kd, dt
    GyverPID(float new_kp, float new_ki, float new_kd, int16_t new_dt = 100) {
        setDt(new_dt);
        Kp = new_kp;
        Ki = new_ki;
        Kd = new_kd;
    }	

    // направление регулирования: NORMAL (0) или REVERSE (1)
    void setDirection(boolean direction) {				
        _direction = direction;
    }
    
    // режим: работа по входной ошибке ON_ERROR (0) или по изменению ON_RATE (1)
    void setMode(boolean mode) {						
        _mode = mode;
    }
    
    // лимит выходной величины (например для ШИМ ставим 0-255)
    void setLimits(int min_output, int max_output) {	
        _minOut = min_output;
        _maxOut = max_output;
    }
    
    // установка времени дискретизации (для getResultTimer)
    void setDt(int16_t new_dt) {						
        _dt_s = new_dt / 1000.0f;
        _dt = new_dt;
    }
    
    datatype setpoint = 0;		// заданная величина, которую должен поддерживать регулятор
    datatype input = 0;			// сигнал с датчика (например температура, которую мы регулируем)
    datatype output = 0;		// выход с регулятора на управляющее устройство (например величина ШИМ или угол поворота серво)
    float Kp = 0.0;				// коэффициент P
    float Ki = 0.0;				// коэффициент I
    float Kd = 0.0;				// коэффициент D	
    float integral = 0.0;		// интегральная сумма
    
    // возвращает новое значение при вызове (если используем свой таймер с периодом dt!)
    datatype getResult() {
        datatype error = setpoint - input;			// ошибка регулирования
        datatype delta_input = prevInput - input;	// изменение входного сигнала за dt
        prevInput = input;							// запомнили предыдущее
        if (_direction) {							// смена направления
            error = -error;
            delta_input = -delta_input;
        }
        output = _mode ? 0 : (error * Kp); 			// пропорциональая составляющая
        output += delta_input * Kd / _dt_s;			// дифференциальная составляющая

#if (PID_INTEGRAL_WINDOW > 0)
        // ЭКСПЕРИМЕНТАЛЬНЫЙ РЕЖИМ ИНТЕГРАЛЬНОГО ОКНА
        if (++t >= PID_INTEGRAL_WINDOW) t = 0; 		// перемотка t
        integral -= errors[t];     					// вычитаем старое	
        errors[t] = error * Ki * _dt_s;  			// запоминаем в массив
        integral += errors[t];         				// прибавляем новое
#else		
        integral += error * Ki * _dt_s;				// обычное суммирование инт. суммы
#endif	

#ifdef PID_OPTIMIZED_I
        // ЭКСПЕРИМЕНТАЛЬНЫЙ РЕЖИМ ОГРАНИЧЕНИЯ ИНТЕГРАЛЬНОЙ СУММЫ
        output = constrain(output, _minOut, _maxOut);
        if (Ki != 0) integral = constrain(integral, (_minOut - output) / (Ki * _dt_s), (_maxOut - output) / (Ki * _dt_s));
#endif

        if (_mode) integral += delta_input * Kp;			// режим пропорционально скорости
        integral = constrain(integral, _minOut, _maxOut);	// ограничиваем инт. сумму
        output += integral;									// интегральная составляющая
        output = constrain(output, _minOut, _maxOut);		// ограничиваем выход
        return output;
    }
    
    // возвращает новое значение не ранее, чем через dt миллисекунд (встроенный таймер с периодом dt)
    datatype getResultTimer() {
        if (millis() - pidTimer >= _dt) {
            pidTimer = millis();
            getResult();
        }
        return output;
    }
    
    // посчитает выход по реальному прошедшему времени между вызовами функции
    datatype getResultNow() {
        setDt(millis() - pidTimer);
        pidTimer = millis();
        return getResult();
    }
    
private:
    int16_t _dt = 100;		// время итерации в мс
    float _dt_s = 0.1;		// время итерации в с
    boolean _mode = 0, _direction = 0;
    int _minOut = 0, _maxOut = 255;	
    datatype prevInput = 0;	
    uint32_t pidTimer = 0;
#if (PID_INTEGRAL_WINDOW > 0)
    datatype errors[PID_INTEGRAL_WINDOW];
    int t = 0;	
#endif
};
#endif