Регулятор-стабилизатор 
предназначен для плавной регулировки переменного среднеквадратичного напряжения на активной нагрузке
мощностью не более 3500 Вт(зависит от исполнения) в пределах 40 – 250 вольт, но не выше входного. 
Регулировка осуществляется фазовым способом, т.е. изменением угла открытия симистора.

Имеет три режима работы: 
1.	Стабилизация. 
В этом режиме стремится поддержать заданное выходное напряжение.
Выходное напряжение регулируется вращением энкодера с шагом 0.5 вольта.
2.	Разгон.
Подает на выход полное входное напряжение.
Вращением энкодера устанавливается требуемое выходное напряжение, которое будет установлено при переходе в режим стабилизации.
3.	Стоп.
Выходное напряжение выключено.
Вращением энкодера устанавливается требуемое выходное напряжение, которое будет установлено при переходе в режим стабилизации.
Переход между режимами осуществляется нажатием на рукоятку энкодера. Есть два вида нажатий – короткое нажатие и длительное удержание. Логика работы в таблице ниже. По вертикали режимы работы, по горизонтали виды нажатия, на пересечении получаемый режим.
                                   Нажатие
Режим	Короткое нажатие	Длительное удержание
Стабилизация	Стоп	Разгон
Разгон	Стабилизация	Стоп
Стоп	Стабилизация	Разгон

Вся информация о режиме работы, состоянии стабилизации, требуемом и выходном напряжении выводится на индикатор.

1.	Режим стабилизации. Надпись «Выход», индикатор стабилизации в виде дуги (нет стабилизации) или круга(выходное напряжение стабилизировано в допустимом коридоре), значение выходного напряжения.
   
![m0](https://github.com/user-attachments/assets/7df4c724-3a89-4deb-9126-f1cf4c06f6d6)
   
2.	Режим разгона. Надпись «РАЗГОН», индикатор целевого напряжения в режиме стабилизации, значение выходного напряжения.
	
![m1](https://github.com/user-attachments/assets/a73c69d0-28d7-4515-95b4-8bba51cd8085)

3.	Режим стоп. Надпись «СТОП», индикатор целевого напряжения в режиме стабилизации, значение выходного напряжения.
	
![m2](https://github.com/user-attachments/assets/ac14fcb7-4f03-4ce8-a9b1-f923d543fd06)

4.	Аварийное отключение внешним сигналом. Нагрузка отключена, выход из режима только вручную нажатием и удержанием кнопки энкодера.
	
![m3](https://github.com/user-attachments/assets/4181efdd-7c57-43e8-9356-39de07433f1e)

При включении с нажатой кнопкой энкодера устанавливается значение выходного напряжения 125 вольт, при отпущенной – 0 вольт, режим работы «Стабилизация».
Дополнительные возможности:
1.	Удаленное управление через UART с гальваноразвязкой.
2.	Аварийное выключение внешним сигналом 3-12 вольт.
3.	Управление устройством дополнительного разгона.



