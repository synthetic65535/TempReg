﻿# PID-регулятор температуры
* Микроконтроллер: STM32F303
* Среда разработки: STM32 CubeMX 5.6.0, Keil uVision5.29.0
* Команды USART: "$P,<число>", "$I,<число>", "$D,<число>", "$T,<число>" - Установка коэффициентов PID-регулятора и целевой температуры.
* Термодатчик подключается к выводу PA0
* Нагреватель управляется выводом PB12
